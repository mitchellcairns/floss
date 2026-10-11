/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_l2cap_priv.h - Private glue between the two halves of Classic L2CAP.
 *
 * L2CAP is split in two files so each stays readable on its own:
 *
 *   floss_l2cap.c      channel table, PSM registry, CID allocation, the data
 *                      path (RX demux, TX header push, credit wake-ups) and
 *                      the public API entry points.
 *   floss_l2cap_sig.c  the signaling channel (CID 0x0001): command parsing,
 *                      the per-channel connect/configure/disconnect state
 *                      machine, security gating and request timeouts.
 *
 * Each channel lives in a slot. The public floss_l2cap_chan_t is the FIRST
 * member of the slot, so a channel pointer handed to the application converts
 * back to its slot with a plain cast (C guarantees a pointer to a struct and
 * to its first member are interchangeable). The slot carries what the
 * application never needs to see: the request timer, wake-up flags and the
 * configuration accumulator.
 */
#ifndef FLOSS_L2CAP_PRIV_H
#define FLOSS_L2CAP_PRIV_H

#include "floss_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Tunables                                                                 */
/* ------------------------------------------------------------------------ */

/** Response timeout for our signaling requests (spec RTX: 1..60 s). */
#ifndef FLOSS_L2CAP_RTX_US
#define FLOSS_L2CAP_RTX_US 10000000u
#endif

/** Extended timeout once the peer answered "pending" (spec ERTX: 60..300 s). */
#ifndef FLOSS_L2CAP_ERTX_US
#define FLOSS_L2CAP_ERTX_US 60000000u
#endif

/** How long a channel may wait for link authentication + encryption. */
#ifndef FLOSS_L2CAP_SECURITY_TIMEOUT_US
#define FLOSS_L2CAP_SECURITY_TIMEOUT_US 30000000u
#endif

/** How long we wait for the peer's Configuration Request after ours succeeded. */
#ifndef FLOSS_L2CAP_CONFIG_TIMEOUT_US
#define FLOSS_L2CAP_CONFIG_TIMEOUT_US 30000000u
#endif

/** Registered server PSMs (HID control + interrupt, SDP, RFCOMM, spare). */
#ifndef FLOSS_L2CAP_MAX_SERVERS
#define FLOSS_L2CAP_MAX_SERVERS 6
#endif

/** Minimum MTU every L2CAP implementation must accept on a Classic channel. */
#define FLOSS_L2CAP_MIN_MTU      48
/** MTU assumed when the peer's Configuration Request carries no MTU option. */
#define FLOSS_L2CAP_DEFAULT_MTU  672

/* ------------------------------------------------------------------------ */
/* Signaling command codes and result values (wire values)                  */
/* ------------------------------------------------------------------------ */

#define FLOSS_L2CAP_SIG_CMD_REJECT     0x01
#define FLOSS_L2CAP_SIG_CONN_REQ       0x02
#define FLOSS_L2CAP_SIG_CONN_RSP       0x03
#define FLOSS_L2CAP_SIG_CONFIG_REQ     0x04
#define FLOSS_L2CAP_SIG_CONFIG_RSP     0x05
#define FLOSS_L2CAP_SIG_DISC_REQ       0x06
#define FLOSS_L2CAP_SIG_DISC_RSP       0x07
#define FLOSS_L2CAP_SIG_ECHO_REQ       0x08
#define FLOSS_L2CAP_SIG_ECHO_RSP       0x09
#define FLOSS_L2CAP_SIG_INFO_REQ       0x0A
#define FLOSS_L2CAP_SIG_INFO_RSP       0x0B

/* Command Reject reasons */
#define FLOSS_L2CAP_REJ_NOT_UNDERSTOOD 0x0000
#define FLOSS_L2CAP_REJ_MTU_EXCEEDED   0x0001
#define FLOSS_L2CAP_REJ_INVALID_CID    0x0002

/* Connection Response results (also reported through on_open on failure) */
#define FLOSS_L2CAP_CONN_SUCCESS        0x0000
#define FLOSS_L2CAP_CONN_PENDING        0x0001
#define FLOSS_L2CAP_CONN_PSM_UNSUPP     0x0002
#define FLOSS_L2CAP_CONN_SECURITY_BLOCK 0x0003
#define FLOSS_L2CAP_CONN_NO_RESOURCES   0x0004
#define FLOSS_L2CAP_CONN_INVALID_SCID   0x0006
#define FLOSS_L2CAP_CONN_SCID_IN_USE    0x0007

/* Connection Response status (when result == pending) */
#define FLOSS_L2CAP_CONN_STATUS_NONE      0x0000
#define FLOSS_L2CAP_CONN_STATUS_AUTH_PEND 0x0001

/* Configuration Response results */
#define FLOSS_L2CAP_CFG_SUCCESS      0x0000
#define FLOSS_L2CAP_CFG_UNACCEPTABLE 0x0001
#define FLOSS_L2CAP_CFG_REJECTED     0x0002
#define FLOSS_L2CAP_CFG_UNKNOWN      0x0003
#define FLOSS_L2CAP_CFG_PENDING      0x0004

/* Configuration option types (bit 7 = hint: may be ignored if not understood) */
#define FLOSS_L2CAP_OPT_HINT         0x80
#define FLOSS_L2CAP_OPT_MTU          0x01
#define FLOSS_L2CAP_OPT_FLUSH_TO     0x02
#define FLOSS_L2CAP_OPT_QOS          0x03
#define FLOSS_L2CAP_OPT_RFC          0x04
#define FLOSS_L2CAP_OPT_FCS          0x05
#define FLOSS_L2CAP_OPT_EFS          0x06
#define FLOSS_L2CAP_OPT_EWS          0x07

/* Information Request types / results */
#define FLOSS_L2CAP_INFO_CL_MTU        0x0001
#define FLOSS_L2CAP_INFO_EXT_FEATURES  0x0002
#define FLOSS_L2CAP_INFO_FIXED_CHANS   0x0003
#define FLOSS_L2CAP_INFO_SUCCESS       0x0000
#define FLOSS_L2CAP_INFO_NOT_SUPPORTED 0x0001

/* ------------------------------------------------------------------------ */
/* Local failure statuses passed to on_open() (never collide with L2CAP     */
/* results 0x0000..0x00FF or HCI reasons 0x00..0xFF)                        */
/* ------------------------------------------------------------------------ */

#define FLOSS_L2CAP_STATUS_TIMEOUT       0xFF01 /**< peer never answered a request */
#define FLOSS_L2CAP_STATUS_REMOTE_CLOSED 0xFF02 /**< peer disconnected before open */
#define FLOSS_L2CAP_STATUS_CONFIG_FAILED 0xFF03 /**< configuration could not converge */
#define FLOSS_L2CAP_STATUS_REJECTED      0xFF04 /**< peer sent Command Reject */
#define FLOSS_L2CAP_STATUS_PROTOCOL      0xFF05 /**< malformed response */
#define FLOSS_L2CAP_STATUS_ABORTED       0xFF06 /**< generic local abort */

/* ------------------------------------------------------------------------ */
/* Channel slot                                                             */
/* ------------------------------------------------------------------------ */

/* chan.cfg_flags bits: configuration handshake progress. */
#define FLOSS_L2CAP_CFG_OUT_DONE 0x01 /**< peer accepted our Configuration Request */
#define FLOSS_L2CAP_CFG_IN_DONE  0x02 /**< we accepted the peer's (final) request */
#define FLOSS_L2CAP_CFG_IN_CONT  0x04 /**< peer is mid-way through a continued request */

/** Room for the options we echo back in a non-success Configuration Response. */
#define FLOSS_L2CAP_CFG_RSP_OPTS_MAX 24

typedef struct {
    floss_l2cap_chan_t chan;   /**< MUST be first (see overview) */
    floss_timer_t timer;       /**< RTX / ERTX / security / config guard */
    bool in_use;
    bool finishing;            /**< terminal callback in progress (re-entry guard) */
    bool opened;               /**< reached OPEN at least once -> on_close, not on_open */
    bool user_closing;         /**< app called floss_l2cap_disconnect() */
    bool wants_can_send;       /**< a send returned BUSY; wake with on_can_send */
    uint8_t sec;               /**< floss_sec_level_t required before accepting/requesting */
    uint8_t peer_req_id;       /**< identifier of the peer's Connection Request (pending reply) */
    uint8_t cfg_retries;       /**< our Configuration Requests re-sent after "unacceptable" */
    uint16_t fail_status;      /**< on_open status to report once a teardown completes */
    /* Accumulated answer to the peer's (possibly continued) Configuration Request. */
    uint16_t cfg_in_result;
    uint8_t cfg_in_opts_len;
    uint8_t cfg_in_opts[FLOSS_L2CAP_CFG_RSP_OPTS_MAX];
} floss_l2cap_slot_t;

/** A registered server PSM. ops == NULL marks a free entry. */
typedef struct {
    uint16_t psm;
    uint8_t sec;
    const floss_l2cap_ops_t *ops;
    void *user;
} floss_l2cap_server_t;

/** The channel table (floss_l2cap.c). Iterate with FLOSS_MAX_L2CAP_CHANNELS. */
extern floss_l2cap_slot_t floss_l2cap_slots[FLOSS_MAX_L2CAP_CHANNELS];

static inline floss_l2cap_slot_t *floss_l2cap_slot_of(floss_l2cap_chan_t *chan) {
    return (floss_l2cap_slot_t *)chan;
}

/* ---- floss_l2cap.c ------------------------------------------------------ */

/** Take a free slot for `handle` with a fresh local CID and default MTUs. NULL if full. */
floss_l2cap_slot_t *floss_l2cap_slot_alloc(uint16_t handle);
/** Live slot on `handle` whose local CID is `cid` (any state), or NULL. */
floss_l2cap_slot_t *floss_l2cap_slot_by_local_cid(uint16_t handle, uint16_t cid);
/** Live slot on `handle` whose remote CID is `cid` (any state), or NULL. */
floss_l2cap_slot_t *floss_l2cap_slot_by_remote_cid(uint16_t handle, uint16_t cid);
/** Server registered for `psm`, or NULL. */
const floss_l2cap_server_t *floss_l2cap_server_find(uint16_t psm);
/**
 * Terminal transition: fire exactly one of on_open(chan, status) (channel never
 * opened and the app did not close it) or on_close(chan), then free the slot.
 * Safe to call twice; the second call is ignored.
 */
void floss_l2cap_slot_finish(floss_l2cap_slot_t *s, uint16_t status);

/* ---- floss_l2cap_sig.c -------------------------------------------------- */

void floss_l2cap_sig_init(void);
/** A C-frame payload (after the basic header) received on CID 0x0001. */
void floss_l2cap_sig_on_frame(uint16_t handle, const uint8_t *data, uint16_t len);
/** Send our Connection Request for an outgoing slot (state -> WAIT_CONN_RSP). */
floss_err_t floss_l2cap_sig_connect(floss_l2cap_slot_t *s);
/** Send a Disconnection Request (state -> WAIT_DISC_RSP, RTX armed). */
void floss_l2cap_sig_disconnect(floss_l2cap_slot_t *s);
/** Answer an incoming request we parked as "pending" with a refusal `result`. */
void floss_l2cap_sig_refuse_pending(floss_l2cap_slot_t *s, uint16_t result);
/** Timer callback for every slot timer (arg = slot). */
void floss_l2cap_sig_timeout(floss_timer_t *t, void *arg);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_L2CAP_PRIV_H */
