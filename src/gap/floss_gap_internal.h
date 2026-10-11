/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_gap_internal.h - Private glue between the GAP source files.
 *
 * GAP is split by concern so each file stays readable on its own:
 *
 *   floss_gap.c          settings, controller bring-up sequence, listeners,
 *                        per-link state, connection events, link policy API
 *   floss_gap_security.c pairing / authentication / encryption events
 *   floss_gap_bonds.c    persistent bond database (link keys)
 *   floss_gap_eir.c      Extended Inquiry Response builder
 *
 * Nothing here is visible outside src/gap/. Cross-file names use the
 * floss_gap_ prefix like everything else, plus a module noun (link_, sec_,
 * bonds_, eir_) so a grep finds the owning file.
 */
#ifndef FLOSS_GAP_INTERNAL_H
#define FLOSS_GAP_INTERNAL_H

#include "floss_internal.h"

/** Handle value used in events when the link has no HCI handle (yet). */
#define FLOSS_GAP_NO_HANDLE 0xFFFFu

/* ------------------------------------------------------------------------ */
/* Per-link GAP state (floss_gap.c)                                          */
/* ------------------------------------------------------------------------ */

/**
 * What GAP remembers about one Classic baseband link, on top of HCI's
 * floss_conn_t. Keyed by peer address because the pairing events (IO
 * capability, link key, PIN...) only carry the address, and because the
 * slot must exist before the handle does (Connection Request, paging).
 */
typedef struct {
    bool used;
    bool has_handle;          /**< Connection Complete seen */
    uint16_t handle;
    floss_addr_t addr;
    uint32_t cod;             /**< peer Class of Device (from Connection Request) */
    bool outgoing;            /**< we paged this peer (floss_gap_connect) */

    /* security progress */
    bool authenticated;       /**< Authentication Complete (success) or encryption seen */
    bool auth_pending;       /**< Authentication Requested sent, waiting for completion */
    bool encrypt_pending;     /**< Set Connection Encryption sent */
    bool pairing;             /**< pairing (SSP or legacy PIN) in progress */
    bool pairing_fail_reported; /**< PAIRING_FAILED already emitted for this attempt */
    bool confirm_pending;     /**< USER_CONFIRM emitted, waiting for floss_gap_user_confirm */
    bool have_session_key;    /**< non-bondable pairing: key kept for this link only */
    floss_link_key_t session_key;

    /** Reports "already secure" to L2CAP outside its call stack, or retries a security request. */
    floss_timer_t sec_timer;
} floss_gap_link_t;

/** Link slot for a peer address, or NULL. */
floss_gap_link_t *floss_gap_link_by_addr(const floss_addr_t *addr);
/** Link slot for an HCI handle, or NULL. */
floss_gap_link_t *floss_gap_link_by_handle(uint16_t handle);
/** Find or allocate the slot for addr. NULL when the table is full. */
floss_gap_link_t *floss_gap_link_get(const floss_addr_t *addr);

/** HCI handle of a Classic link to addr, or FLOSS_GAP_NO_HANDLE. */
uint16_t floss_gap_handle_for_addr(const floss_addr_t *addr);

/** Deliver an event to every listener. */
void floss_gap_emit(const floss_gap_evt_t *evt);
/** Shorthand for the common (type, handle, addr, status) events. */
void floss_gap_emit_simple(floss_gap_evt_type_t type, uint16_t handle, const floss_addr_t *addr, uint8_t status);

/** True once the controller is up and GAP may send commands directly. */
bool floss_gap_hci_live(void);

/** Local name as stored for Write Local Name (248 bytes, zero padded) + its length. */
const uint8_t *floss_gap_local_name(uint8_t *len);

/* ------------------------------------------------------------------------ */
/* Security (floss_gap_security.c)                                           */
/* ------------------------------------------------------------------------ */

/** Reset security settings to defaults (called by floss_gap_init). */
void floss_gap_sec_init(void);
/** Should the bring-up sequence enable Secure Connections host support? */
bool floss_gap_sec_want_sc(void);
/**
 * Handle a pairing / security HCI event. Returns false if `code` is not a
 * security event (the caller then handles or ignores it).
 */
bool floss_gap_sec_on_event(uint8_t code, const uint8_t *params, uint8_t len);
/** Timer callback reporting an already-encrypted link to L2CAP. */
void floss_gap_sec_timer_fn(floss_timer_t *t, void *arg);

/* ------------------------------------------------------------------------ */
/* Bond database (floss_gap_bonds.c)                                         */
/* ------------------------------------------------------------------------ */

/** Serialised bond record: version + addr6 + key16 + type1 + cod3 + seq4. */
#define FLOSS_GAP_BOND_REC_VERSION 1u
#define FLOSS_GAP_BOND_REC_LEN     31u

/** Encode one bond (explicit byte layout, no struct casts). Returns FLOSS_GAP_BOND_REC_LEN. */
uint16_t floss_gap_bond_encode(const floss_bond_t *bond, uint8_t out[FLOSS_GAP_BOND_REC_LEN]);
/** Decode a stored record. false if the length/version is wrong or the address is zero. */
bool floss_gap_bond_decode(const uint8_t *in, int len, floss_bond_t *bond);

/** Load every slot from the HAL store (called by floss_gap_init). */
void floss_gap_bonds_load(void);
/**
 * Create or replace the bond for addr, mark it most recent, persist it and
 * record it as the last host. Evicts the least recently used bond when full.
 */
const floss_bond_t *floss_gap_bond_save(const floss_addr_t *addr, const floss_link_key_t *key, uint32_t cod);
/** Mark an existing bond as the most recently used (no flash write if it already is). */
void floss_gap_bond_touch(const floss_addr_t *addr);

/* ------------------------------------------------------------------------ */
/* EIR (floss_gap_eir.c)                                                     */
/* ------------------------------------------------------------------------ */

/** Size of the EIR data block in Write Extended Inquiry Response. */
#define FLOSS_GAP_EIR_LEN 240u
/** Max 16-bit service UUIDs advertised in the EIR. */
#define FLOSS_GAP_EIR_MAX_UUIDS 8u

/** Forget registered UUIDs (called by floss_gap_init). */
void floss_gap_eir_init(void);
/**
 * Build an EIR block: name (complete, or shortened if it does not fit) then
 * the complete list of 16-bit UUIDs. Writes exactly `cap` bytes (zero padded).
 * @return number of significant bytes.
 */
uint16_t floss_gap_eir_build(uint8_t *out, uint16_t cap, const uint8_t *name, uint16_t name_len,
                             const uint16_t *uuids, uint8_t n_uuids);
/** Build the current EIR and queue Write Extended Inquiry Response. */
floss_err_t floss_gap_eir_send(floss_hci_cmd_cb cb, void *ctx);

#endif /* FLOSS_GAP_INTERNAL_H */
