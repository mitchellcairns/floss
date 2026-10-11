/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_gap.h - Generic Access: visibility, connections, pairing/bonding,
 * link policy (sniff, role, supervision timeout).
 *
 * Pairing model: the controller does the cryptography. The host answers the
 * controller's questions (IO capability, user confirmation, PIN, link key)
 * and stores the resulting link keys via the HAL key/value store.
 *
 * Defaults are tuned for a game controller: Secure Simple Pairing with
 * NoInputNoOutput ("just works"), bonding enabled, legacy PIN "0000"
 * fallback, page scan tuned for fast reconnects, role switch allowed.
 */
#ifndef FLOSS_GAP_H
#define FLOSS_GAP_H

#include "floss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Class of Device helpers                                                  */
/* ------------------------------------------------------------------------ */

/** Major service class bits. */
#define FLOSS_COD_SERVICE_LIMITED_DISCOVERABLE (1ul << 13)
/** Major device class: peripheral (keyboard, mouse, gamepad...). */
#define FLOSS_COD_MAJOR_PERIPHERAL (0x05ul << 8)
/** Peripheral minor classes. */
#define FLOSS_COD_MINOR_JOYSTICK   (0x01ul << 2)
#define FLOSS_COD_MINOR_GAMEPAD    (0x02ul << 2)
#define FLOSS_COD_MINOR_KEYBOARD   (0x10ul << 2)
#define FLOSS_COD_MINOR_POINTING   (0x20ul << 2)
/**
 * Ready-made: a gamepad (0x002508). Includes the limited-discoverable service
 * bit, which is what commercial controllers advertise and some hosts filter on.
 */
#define FLOSS_COD_GAMEPAD (FLOSS_COD_SERVICE_LIMITED_DISCOVERABLE | FLOSS_COD_MAJOR_PERIPHERAL | FLOSS_COD_MINOR_GAMEPAD)

/* ------------------------------------------------------------------------ */
/* Events to the application                                                */
/* ------------------------------------------------------------------------ */

typedef enum {
    FLOSS_GAP_EVT_READY = 1,          /**< controller up, settings applied */
    FLOSS_GAP_EVT_CONNECTED,          /**< baseband link open (handle, addr) */
    FLOSS_GAP_EVT_CONNECT_FAILED,     /**< outgoing page failed (addr, status) */
    FLOSS_GAP_EVT_DISCONNECTED,       /**< (handle, addr, reason) */
    FLOSS_GAP_EVT_PAIRING_STARTED,    /**< (handle, addr) */
    FLOSS_GAP_EVT_PAIRING_COMPLETE,   /**< new link key stored (handle, addr) */
    FLOSS_GAP_EVT_PAIRING_FAILED,     /**< (handle, addr, status) */
    FLOSS_GAP_EVT_AUTHENTICATED,      /**< (handle, status) */
    FLOSS_GAP_EVT_ENCRYPTION_CHANGE,  /**< (handle, status, encrypted) */
    FLOSS_GAP_EVT_MODE_CHANGE,        /**< (handle, mode, interval) */
    FLOSS_GAP_EVT_ROLE_CHANGE,        /**< (handle, role) */
    FLOSS_GAP_EVT_USER_CONFIRM,       /**< numeric comparison: (handle, passkey). Answer with floss_gap_user_confirm() if auto-accept off */
    FLOSS_GAP_EVT_PASSKEY_DISPLAY,    /**< (handle, passkey) */
    FLOSS_GAP_EVT_RSSI,               /**< (handle, rssi) result of floss_gap_read_rssi */
    FLOSS_GAP_EVT_HARDWARE_ERROR,     /**< controller reported a hardware error (status=code) */
} floss_gap_evt_type_t;

typedef struct {
    floss_gap_evt_type_t type;
    uint16_t handle;
    floss_addr_t addr;
    uint8_t status;     /**< HCI status / disconnect reason */
    uint8_t mode;       /**< MODE_CHANGE: FLOSS_LINK_MODE_*; ROLE_CHANGE: role */
    uint16_t interval;  /**< MODE_CHANGE: sniff interval (slots) */
    uint32_t passkey;   /**< USER_CONFIRM / PASSKEY_DISPLAY */
    int8_t rssi;        /**< RSSI */
    bool encrypted;     /**< ENCRYPTION_CHANGE */
} floss_gap_evt_t;

typedef void (*floss_gap_evt_fn)(const floss_gap_evt_t *evt, void *user);

/**
 * Register an event listener. Several listeners may be registered (the HID
 * layer is one); each sees every event. Up to 4.
 */
floss_err_t floss_gap_add_listener(floss_gap_evt_fn fn, void *user);

/* ------------------------------------------------------------------------ */
/* Identity & visibility (may be called before or after READY)               */
/* ------------------------------------------------------------------------ */

/** Local friendly name (UTF-8, <= 248 bytes). Also placed in the EIR. */
floss_err_t floss_gap_set_name(const char *name);

/** 24-bit Class of Device, e.g. FLOSS_COD_GAMEPAD. */
floss_err_t floss_gap_set_class(uint32_t cod);

/**
 * discoverable: answer inquiries (show up in "add device" lists).
 * connectable:  answer pages (allow hosts to connect).
 */
floss_err_t floss_gap_set_scan(bool discoverable, bool connectable);

/**
 * Page scan timing (slots of 0.625 ms). Smaller interval = faster reconnect,
 * more power. Default interval 0x0400 window 0x0012 (interlaced). Gamepads
 * often use 0x0100/0x0012 while waiting for the host.
 */
floss_err_t floss_gap_set_page_scan(uint16_t interval, uint16_t window, bool interlaced);

/** Rebuild and resend the Extended Inquiry Response (name + service UUIDs). Called automatically. */
floss_err_t floss_gap_update_eir(void);

/** Add a 16-bit service UUID to the EIR (SDP calls this per record). */
floss_err_t floss_gap_eir_add_uuid16(uint16_t uuid);

/* ------------------------------------------------------------------------ */
/* Connections                                                              */
/* ------------------------------------------------------------------------ */

/** Page a remote device (e.g. reconnect to the bonded host). */
floss_err_t floss_gap_connect(const floss_addr_t *addr);
/** Cancel a pending outgoing page. */
floss_err_t floss_gap_connect_cancel(const floss_addr_t *addr);
/** Disconnect (reason default FLOSS_HCI_ERR_REMOTE_USER_TERMINATED). */
floss_err_t floss_gap_disconnect(uint16_t handle, uint8_t reason);

/* ------------------------------------------------------------------------ */
/* Security                                                                 */
/* ------------------------------------------------------------------------ */

/** IO capabilities (as in the IO Capability Request Reply). */
#define FLOSS_IO_CAP_DISPLAY_ONLY       0x00
#define FLOSS_IO_CAP_DISPLAY_YES_NO     0x01
#define FLOSS_IO_CAP_KEYBOARD_ONLY      0x02
#define FLOSS_IO_CAP_NO_INPUT_NO_OUTPUT 0x03

typedef struct {
    uint8_t io_capability;    /**< default NO_INPUT_NO_OUTPUT */
    bool bondable;            /**< store link keys (default true) */
    bool mitm_required;       /**< default false (gamepads cannot do MITM) */
    bool auto_accept_confirm; /**< auto "yes" to numeric comparison (default true) */
    bool secure_connections;  /**< enable SC host support if controller has it (default true) */
    const char *legacy_pin;   /**< for pre-2.1 hosts, default "0000" */
    bool pairable;            /**< accept NEW pairings (default true). Bonded peers always allowed. */
} floss_gap_security_t;

/** Fill with defaults described above. */
void floss_gap_security_defaults(floss_gap_security_t *sec);
floss_err_t floss_gap_set_security(const floss_gap_security_t *sec);

/** Answer a FLOSS_GAP_EVT_USER_CONFIRM. */
floss_err_t floss_gap_user_confirm(uint16_t handle, bool accept);

/** Start authentication (+ encryption once authenticated) on a link. */
floss_err_t floss_gap_authenticate(uint16_t handle);

/* ------------------------------------------------------------------------ */
/* Bond database                                                            */
/* ------------------------------------------------------------------------ */

typedef struct {
    floss_addr_t addr;
    floss_link_key_t key;
    uint32_t cod;      /**< peer class of device (if known) */
    uint32_t seq;      /**< recency: higher = more recently used */
} floss_bond_t;

/** Number of stored bonds. */
uint8_t floss_gap_bond_count(void);
/** Get bond by index (0..count-1). */
const floss_bond_t *floss_gap_bond_at(uint8_t idx);
/** Most recently used bond (the host to reconnect to), or NULL. */
const floss_bond_t *floss_gap_bond_last(void);
/** Find bond by address. */
const floss_bond_t *floss_gap_bond_find(const floss_addr_t *addr);
floss_err_t floss_gap_bond_delete(const floss_addr_t *addr);
floss_err_t floss_gap_bond_delete_all(void);

/* ------------------------------------------------------------------------ */
/* Link policy / power / latency                                            */
/* ------------------------------------------------------------------------ */

/** Link policy bits. */
#define FLOSS_LINK_POLICY_ROLE_SWITCH 0x0001
#define FLOSS_LINK_POLICY_HOLD        0x0002
#define FLOSS_LINK_POLICY_SNIFF       0x0004

/** Default policy for new links (default ROLE_SWITCH | SNIFF). */
floss_err_t floss_gap_set_default_link_policy(uint16_t policy);
floss_err_t floss_gap_set_link_policy(uint16_t handle, uint16_t policy);

/**
 * Enter sniff mode. Intervals in slots (0.625 ms). For a gamepad the latency
 * floor is the sniff interval: e.g. max_interval 6 = 3.75 ms.
 */
floss_err_t floss_gap_sniff(uint16_t handle, uint16_t min_interval, uint16_t max_interval, uint16_t attempt, uint16_t timeout);
floss_err_t floss_gap_sniff_exit(uint16_t handle);
floss_err_t floss_gap_sniff_subrating(uint16_t handle, uint16_t max_latency, uint16_t min_remote_timeout, uint16_t min_local_timeout);

/** Supervision timeout in slots (0x7D00 = 20 s default). */
floss_err_t floss_gap_set_supervision_timeout(uint16_t handle, uint16_t slots);

/**
 * Automatic flush timeout in ms (0 = infinite / reliable). With a finite
 * value the controller drops FLUSHABLE packets it could not deliver in
 * time: stale gamepad reports are discarded instead of delaying fresh ones.
 */
floss_err_t floss_gap_set_flush_timeout(uint16_t handle, uint16_t ms);

/**
 * Ask the link for a quality of service (HCI QoS Setup, guaranteed service).
 * The controller negotiates with the central, which sets the poll interval
 * (how often it gives us a transmit opportunity) from latency_us. This is how
 * a peripheral asks a host for ~1 ms polling instead of the default ~25 ms.
 * token_rate in bytes/s (0 = let the controller choose).
 */
floss_err_t floss_gap_qos_setup(uint16_t handle, uint32_t latency_us, uint32_t token_rate);

/** Request a role switch (0 = become central, 1 = become peripheral). */
floss_err_t floss_gap_switch_role(uint16_t handle, uint8_t role);

/** Read RSSI; result arrives as FLOSS_GAP_EVT_RSSI. */
floss_err_t floss_gap_read_rssi(uint16_t handle);

/* ------------------------------------------------------------------------ */
/* Lifecycle (called by floss.c)                                            */
/* ------------------------------------------------------------------------ */

void floss_gap_init(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_GAP_H */
