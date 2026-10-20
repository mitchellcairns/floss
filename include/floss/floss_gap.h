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
    FLOSS_GAP_EVT_INQUIRY_RESULT,     /**< a device answered our inquiry: (addr, cod, rssi, eir/eir_len) */
    FLOSS_GAP_EVT_INQUIRY_COMPLETE,   /**< inquiry over (status; 0 also when cancelled). Exactly one per start. */
} floss_gap_evt_type_t;

/** INQUIRY_RESULT rssi value when the controller did not report one (standard Inquiry Result). */
#define FLOSS_GAP_RSSI_UNKNOWN 127

typedef struct {
    floss_gap_evt_type_t type;
    uint16_t handle;
    floss_addr_t addr;
    uint8_t status;     /**< HCI status / disconnect reason */
    uint8_t mode;       /**< MODE_CHANGE: FLOSS_LINK_MODE_*; ROLE_CHANGE: role */
    uint16_t interval;  /**< MODE_CHANGE: sniff interval (slots) */
    uint32_t passkey;   /**< USER_CONFIRM / PASSKEY_DISPLAY */
    int8_t rssi;        /**< RSSI (INQUIRY_RESULT: FLOSS_GAP_RSSI_UNKNOWN if not reported) */
    bool encrypted;     /**< ENCRYPTION_CHANGE */
    uint32_t cod;       /**< INQUIRY_RESULT: the device's Class of Device */
    /**
     * INQUIRY_RESULT: the device's Extended Inquiry Response (name, service
     * UUIDs...) as [len][type][data] structures, valid during the callback
     * only; NULL / 0 when the device sent none. Parse with floss_gap_eir_find().
     */
    const uint8_t *eir;
    uint8_t eir_len;
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

/**
 * How long a page (an outgoing connection) tries before failing, in ms. Default 5120.
 * Kept across controller restarts.
 */
floss_err_t floss_gap_set_page_timeout(uint16_t ms);

/** Rebuild and resend the Extended Inquiry Response (name + service UUIDs). Called automatically. */
floss_err_t floss_gap_update_eir(void);

/** Add a 16-bit service UUID to the EIR (SDP calls this per record). */
floss_err_t floss_gap_eir_add_uuid16(uint16_t uuid);

/**
 * Add a 128-bit service UUID to the EIR (Complete List of 128-bit Service
 * UUIDs, type 0x07). uuid_be: 16 bytes, big-endian as written. Up to
 * FLOSS_GAP_EIR_MAX_UUID128 (2).
 */
floss_err_t floss_gap_eir_add_uuid128(const uint8_t uuid_be[16]);

/* EIR data types (shared with LE advertising data). */
#define FLOSS_EIR_UUID16_SOME   0x02
#define FLOSS_EIR_UUID16_ALL    0x03
#define FLOSS_EIR_UUID128_SOME  0x06
#define FLOSS_EIR_UUID128_ALL   0x07
#define FLOSS_EIR_NAME_SHORT    0x08
#define FLOSS_EIR_NAME_COMPLETE 0x09

/**
 * Find the first [len][type][data] structure of `type` in EIR (or LE
 * advertising) data. Returns its payload and length, or NULL. Malformed
 * data (a length running past the end) ends the search safely.
 */
const uint8_t *floss_gap_eir_find(const uint8_t *eir, uint8_t len, uint8_t type, uint8_t *out_len);

/** True if the EIR lists this 128-bit service UUID (complete or incomplete list). uuid_be as written. */
bool floss_gap_eir_has_uuid128(const uint8_t *eir, uint8_t len, const uint8_t uuid_be[16]);

/* ------------------------------------------------------------------------ */
/* Discovery (central side: finding devices)                                */
/* ------------------------------------------------------------------------ */

/**
 * Start a general inquiry (GIAC). Every answer arrives as
 * FLOSS_GAP_EVT_INQUIRY_RESULT (the controller may report a device more
 * than once); the end as FLOSS_GAP_EVT_INQUIRY_COMPLETE.
 *
 * duration_1_28s: 1..48 (x 1.28 s). max_results: stop after this many
 * answers (0 = unlimited). Paging is not possible while an inquiry runs:
 * cancel it first. A later floss_gap_connect() to a device seen here uses
 * the page scan mode and clock offset it reported (much faster page).
 */
floss_err_t floss_gap_inquiry_start(uint8_t duration_1_28s, uint8_t max_results);
/** Stop a running inquiry; INQUIRY_COMPLETE (status 0) follows. */
floss_err_t floss_gap_inquiry_cancel(void);
/** True between floss_gap_inquiry_start() and INQUIRY_COMPLETE. */
bool floss_gap_inquiry_active(void);

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
    /**
     * When a bonded host rejects our key (PIN/key missing, authentication
     * failure), forget the bond for this address pair so the next attempt
     * pairs afresh (default true). Bonds made under another local address
     * are never touched. Set false to keep keys until the app deletes them.
     */
    bool forget_bond_on_key_missing;
    /**
     * Optional legacy PIN provider, for PINs that are binary (may contain zero
     * bytes) or differ per peer. Write up to 16 bytes into pin and return the
     * length; 0 refuses the pairing. When NULL, legacy_pin is used. Called
     * from floss_poll(); `pairable` still applies.
     */
    uint8_t (*pin_handler)(const floss_addr_t *peer, uint8_t pin[16]);
} floss_gap_security_t;

/** Fill with defaults described above. */
void floss_gap_security_defaults(floss_gap_security_t *sec);
floss_err_t floss_gap_set_security(const floss_gap_security_t *sec);
/** The current settings (to change one field: get, modify, set). */
void floss_gap_get_security(floss_gap_security_t *sec);

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
    /**
     * Our own address when the bond was made. A host keys its bond by the
     * address pair, so a device that changes its address (one per mode, say)
     * has one bond per local address. All-zero = unknown (a record from an
     * older firmware): it matches any local address.
     */
    floss_addr_t local;
} floss_bond_t;

/*
 * Bonds are keyed by the pair (peer address, our address). Lookups, saves
 * and deletes by peer address use our CURRENT address (floss_hci_controller()
 * ->addr); floss_gap_bond_count()/floss_gap_bond_at() list every bond.
 */

/** Number of stored bonds. */
uint8_t floss_gap_bond_count(void);
/** Get bond by index (0..count-1). */
const floss_bond_t *floss_gap_bond_at(uint8_t idx);
/** Most recently used bond under our current address (the host to reconnect to), or NULL. */
const floss_bond_t *floss_gap_bond_last(void);
/** Find bond by address. */
const floss_bond_t *floss_gap_bond_find(const floss_addr_t *addr);
/**
 * Store a link key obtained out of band (e.g. paired over another transport)
 * as a bond, marked most recently used and persisted like any other. It is
 * tied to our current address; called before READY (address not known yet)
 * the bond matches any local address.
 */
floss_err_t floss_gap_bond_add(const floss_addr_t *addr, const floss_link_key_t *key);
floss_err_t floss_gap_bond_delete(const floss_addr_t *addr);
floss_err_t floss_gap_bond_delete_all(void);
/**
 * Forget a host: every bond with it, under any local address, whether the
 * stack loaded them this boot or they are only stored (it need not be
 * running). NOT_FOUND when there were none.
 */
floss_err_t floss_gap_bond_forget(const floss_addr_t *addr);

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

/* ------------------------------------------------------------------------ */
/* Adaptive frequency hopping (Classic)                                     */
/* ------------------------------------------------------------------------ */

/**
 * Controller channel assessment on/off (HCI Write AFH Channel Assessment
 * Mode). While on, a controller periodically measures every channel to
 * classify interference. On the CYW43439 each measurement leaves the link
 * for ~25 ms (79 channels x 312.5 us) every few seconds: visible as report
 * gaps on a latency-critical link. See docs/PERFORMANCE.md.
 */
floss_err_t floss_gap_set_afh_assessment(bool enabled);

/**
 * Host channel classification (HCI Set AFH Host Channel Classification):
 * 79-bit map, bit n = channel n; 1 = usable/unknown, 0 = bad. Applies to
 * links where we are central. At least 20 channels must stay usable.
 */
floss_err_t floss_gap_set_afh_host_channels(const uint8_t map[10]);

/** Result of floss_gap_read_afh_map: map bit n = channel n in use. */
typedef void (*floss_gap_afh_map_fn)(uint16_t handle, uint8_t status, uint8_t afh_mode, const uint8_t map[10],
                                     void *user);
/** Read the hopping map a link uses right now (HCI Read AFH Channel Map). */
floss_err_t floss_gap_read_afh_map(uint16_t handle, floss_gap_afh_map_fn fn, void *user);

/**
 * Result of floss_gap_read_clock: the clock in half slots (312.5 us ticks, 28 bits), and for the
 * piconet clock its accuracy in the same ticks (0xFFFF unknown).
 */
typedef void (*floss_gap_clock_fn)(uint16_t handle, uint8_t status, uint32_t clock, uint16_t accuracy, void *user);
/**
 * Read a clock (HCI Read Clock): which = 1 the piconet clock of the link `handle` (the central's,
 * as this controller follows it), which = 0 our own native clock. For timing traffic to the
 * central's slots; the controller reads the clock somewhere between sending the command and its
 * answer. One read at a time.
 */
floss_err_t floss_gap_read_clock(uint16_t handle, uint8_t which, floss_gap_clock_fn fn, void *user);

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
