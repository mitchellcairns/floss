/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_smp.h - LE Security Manager (SMP): pairing, bonding, LE privacy keys.
 *
 * ===========================================================================
 *  WHAT IT DOES
 *
 *  Runs the Security Manager Protocol on fixed L2CAP channel 0x0006 of every
 *  LE link and answers the controller's Long Term Key requests:
 *
 *   - LE legacy pairing (Just Works, Passkey Entry) and LE Secure Connections
 *     (Just Works, Numeric Comparison, Passkey Entry). OOB is not supported.
 *   - Peripheral (responder) role is the primary target; the central
 *     (initiator) role is supported too (floss_smp_request_security()).
 *   - ECDH runs in the controller (HCI LE Read Local P-256 Public Key /
 *     LE Generate DHKey), so no big-number code lives in the host.
 *   - Bonds (LTK + peer identity + peer IRK) persist through the HAL
 *     key/value store; resolvable private addresses of bonded peers are
 *     recognised with their IRK.
 *
 *  Defaults suit a game controller: NoInputNoOutput (Just Works), bondable,
 *  no MITM requirement, Secure Connections preferred but not required.
 *
 *  Security levels reported here and to ATT (floss_smp_security_level()):
 *    1 = no encryption
 *    2 = encrypted, unauthenticated key (Just Works)
 *    3 = encrypted, authenticated key (MITM protected)
 *    4 = encrypted, authenticated LE Secure Connections key (128-bit)
 *
 *  Persistent storage keys used (see floss_hal.h):
 *    FLOSS_SMP_STORE_KEY_IRK            our Identity Resolving Key (generated once)
 *    FLOSS_SMP_STORE_KEY_BOND_BASE + i  LE bond i (0 .. FLOSS_MAX_BONDS-1)
 * ===========================================================================
 */
#ifndef FLOSS_SMP_H
#define FLOSS_SMP_H

#include "floss_types.h"

#if FLOSS_ENABLE_LE

#ifdef __cplusplus
extern "C" {
#endif

#define FLOSS_SMP_STORE_KEY_IRK       0x01F0
#define FLOSS_SMP_STORE_KEY_BOND_BASE 0x0200

/* ------------------------------------------------------------------------ */
/* Configuration                                                            */
/* ------------------------------------------------------------------------ */

/** IO capabilities as sent in the Pairing Request/Response. */
#define FLOSS_SMP_IO_DISPLAY_ONLY       0x00
#define FLOSS_SMP_IO_DISPLAY_YES_NO     0x01
#define FLOSS_SMP_IO_KEYBOARD_ONLY      0x02
#define FLOSS_SMP_IO_NO_INPUT_NO_OUTPUT 0x03
#define FLOSS_SMP_IO_KEYBOARD_DISPLAY   0x04

/** Key distribution bits (Pairing Request/Response key distribution fields). */
#define FLOSS_SMP_DIST_ENC  0x01 /**< legacy LTK + EDIV/Rand */
#define FLOSS_SMP_DIST_ID   0x02 /**< IRK + identity address */
#define FLOSS_SMP_DIST_SIGN 0x04 /**< CSRK (not supported, always cleared) */
#define FLOSS_SMP_DIST_LINK 0x08 /**< cross-transport key derivation (not supported) */

typedef struct {
    uint8_t io_capability;        /**< FLOSS_SMP_IO_*, default NO_INPUT_NO_OUTPUT */
    bool pairable;                /**< accept NEW pairings (default true); bonded peers may always re-pair */
    bool bondable;                /**< request bonding and store keys (default true) */
    bool mitm;                    /**< require an authenticated (MITM) method (default false) */
    bool secure_connections;      /**< offer LE Secure Connections (default true) */
    bool secure_connections_only; /**< refuse LE legacy pairing (default false) */
    uint8_t min_key_size;         /**< smallest acceptable encryption key size, 7..16 (default 7) */
    /**
     * Keys we distribute (FLOSS_SMP_DIST_*), default ENC | ID.
     * ENC only applies when we are the responder (a legacy LTK given by the
     * central is only usable with the roles reversed).
     */
    uint8_t local_keys;
    /**
     * Keys we ask the peer for, default ENC | ID. ENC only applies when we are
     * the initiator (central), for the same reason.
     */
    uint8_t peer_keys;
} floss_smp_config_t;

/** Fill with the defaults described above. */
void floss_smp_config_defaults(floss_smp_config_t *cfg);
/** Apply a configuration (takes effect for the next pairing). */
floss_err_t floss_smp_set_config(const floss_smp_config_t *cfg);

/**
 * Our own LE address as used on the air AND as identity address (type 0
 * public, 1 random static). Called by the LE module when it advertises with
 * a random static address. Default: the controller's public address.
 */
void floss_smp_set_local_addr(uint8_t type, const floss_addr_t *addr);

/** Our Identity Resolving Key (16 bytes, wire order), e.g. for the controller resolving list. */
const uint8_t *floss_smp_local_irk(void);

/* ------------------------------------------------------------------------ */
/* Events                                                                   */
/* ------------------------------------------------------------------------ */

/** Reasons (Pairing Failed codes, plus FLOSS_SMP_REASON_TIMEOUT which never goes on the air). */
#define FLOSS_SMP_REASON_PASSKEY_ENTRY_FAILED   0x01
#define FLOSS_SMP_REASON_OOB_NOT_AVAILABLE      0x02
#define FLOSS_SMP_REASON_AUTH_REQUIREMENTS      0x03
#define FLOSS_SMP_REASON_CONFIRM_VALUE_FAILED   0x04
#define FLOSS_SMP_REASON_PAIRING_NOT_SUPPORTED  0x05
#define FLOSS_SMP_REASON_ENC_KEY_SIZE           0x06
#define FLOSS_SMP_REASON_CMD_NOT_SUPPORTED      0x07
#define FLOSS_SMP_REASON_UNSPECIFIED            0x08
#define FLOSS_SMP_REASON_REPEATED_ATTEMPTS      0x09
#define FLOSS_SMP_REASON_INVALID_PARAMETERS     0x0A
#define FLOSS_SMP_REASON_DHKEY_CHECK_FAILED     0x0B
#define FLOSS_SMP_REASON_NUMERIC_COMPARISON_FAILED 0x0C
#define FLOSS_SMP_REASON_KEY_REJECTED           0x0F
#define FLOSS_SMP_REASON_TIMEOUT                0xF0 /**< local: 30 s SMP timeout */

typedef enum {
    FLOSS_SMP_EVT_PAIRING_STARTED = 1, /**< (handle, addr) */
    FLOSS_SMP_EVT_PAIRING_COMPLETE,    /**< (handle, addr, bonded, level) */
    FLOSS_SMP_EVT_PAIRING_FAILED,      /**< (handle, addr, reason) */
    FLOSS_SMP_EVT_NUMERIC_COMPARISON,  /**< show passkey, answer with floss_smp_confirm() */
    FLOSS_SMP_EVT_PASSKEY_DISPLAY,     /**< show passkey to the user */
    FLOSS_SMP_EVT_PASSKEY_INPUT,       /**< ask the user, answer with floss_smp_passkey_reply() */
    FLOSS_SMP_EVT_SECURITY_CHANGED,    /**< link encrypted with a stored bond, or encryption lost (handle, level) */
    FLOSS_SMP_EVT_BOND_DELETED,        /**< (addr, addr_type) */
} floss_smp_evt_type_t;

typedef struct {
    floss_smp_evt_type_t type;
    uint16_t handle;
    uint8_t addr_type;   /**< peer identity (if known) or connection address */
    floss_addr_t addr;
    uint8_t reason;      /**< PAIRING_FAILED: FLOSS_SMP_REASON_* */
    uint8_t level;       /**< PAIRING_COMPLETE / SECURITY_CHANGED: security level 1..4 */
    bool bonded;         /**< PAIRING_COMPLETE: keys were stored */
    uint32_t passkey;    /**< NUMERIC_COMPARISON / PASSKEY_DISPLAY: 0..999999 */
} floss_smp_evt_t;

typedef void (*floss_smp_evt_fn)(const floss_smp_evt_t *evt, void *user);

#define FLOSS_SMP_MAX_LISTENERS 2
/** Register an event listener (up to FLOSS_SMP_MAX_LISTENERS). */
floss_err_t floss_smp_add_listener(floss_smp_evt_fn fn, void *user);

/** Answer FLOSS_SMP_EVT_NUMERIC_COMPARISON. */
floss_err_t floss_smp_confirm(uint16_t handle, bool accept);

/** Pass to floss_smp_passkey_reply() to cancel passkey entry. */
#define FLOSS_SMP_PASSKEY_REJECT 0xFFFFFFFFu
/** Answer FLOSS_SMP_EVT_PASSKEY_INPUT with 0..999999 (or FLOSS_SMP_PASSKEY_REJECT). */
floss_err_t floss_smp_passkey_reply(uint16_t handle, uint32_t passkey);

/* ------------------------------------------------------------------------ */
/* Security on a link                                                       */
/* ------------------------------------------------------------------------ */

/**
 * Ask for the link to be secured.
 *  - Peripheral: sends a Security Request with our AuthReq; the central then
 *    either encrypts with an existing bond or starts pairing.
 *  - Central: encrypts with the stored bond if it satisfies our
 *    configuration, otherwise starts pairing.
 */
floss_err_t floss_smp_request_security(uint16_t handle);

/** Current security level of a link (1..4), 0 if the handle is unknown. */
uint8_t floss_smp_security_level(uint16_t handle);

/* ------------------------------------------------------------------------ */
/* Bond database                                                            */
/* ------------------------------------------------------------------------ */

#define FLOSS_SMP_BOND_AUTHENTICATED 0x01 /**< key from an MITM-protected method */
#define FLOSS_SMP_BOND_SC            0x02 /**< LE Secure Connections key */
#define FLOSS_SMP_BOND_PEER_LTK      0x04 /**< legacy LTK distributed by the peer: usable when we are central */
#define FLOSS_SMP_BOND_HAS_IRK       0x08 /**< peer_irk valid */

/** One LE bond. All multi-byte values in wire order (LSB first). */
typedef struct {
    uint8_t addr_type;     /**< peer identity address type (0 public, 1 random static) */
    floss_addr_t addr;     /**< peer identity address */
    uint8_t flags;         /**< FLOSS_SMP_BOND_* */
    uint8_t key_size;      /**< negotiated encryption key size (7..16) */
    uint8_t ltk[16];
    uint16_t ediv;         /**< legacy only (0 for LE Secure Connections) */
    uint8_t rand[8];       /**< legacy only (0 for LE Secure Connections) */
    uint8_t peer_irk[16];
    uint32_t seq;          /**< recency: higher = more recently used */
} floss_smp_bond_t;

uint8_t floss_smp_bond_count(void);
/** Bond by index 0..count-1 (order is storage order, not recency). */
const floss_smp_bond_t *floss_smp_bond_at(uint8_t idx);
/**
 * Find the bond for an over-the-air address: identity address match, or a
 * resolvable private address generated from a bonded peer's IRK. NULL if none.
 */
const floss_smp_bond_t *floss_smp_resolve(uint8_t addr_type, const floss_addr_t *addr);
/** Delete the bond with this identity (or resolvable) address. */
floss_err_t floss_smp_bond_delete(uint8_t addr_type, const floss_addr_t *addr);
floss_err_t floss_smp_bond_delete_all(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_ENABLE_LE */

#endif /* FLOSS_SMP_H */
