/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_hid.h - HID device role (gamepads, keyboards, mice, ...).
 *
 * What it does for you:
 *  - Registers HID control (PSM 0x11) and interrupt (PSM 0x13) servers and
 *    the HID + Device ID SDP records, sets class of device and EIR.
 *  - Handles every HIDP transaction: HANDSHAKE, HID_CONTROL (suspend,
 *    exit suspend, virtual cable unplug), GET/SET_REPORT, GET/SET_PROTOCOL,
 *    GET/SET_IDLE, DATA in both directions.
 *  - Reconnects to the last bonded host (control first, then interrupt).
 *  - Sends input reports with the lowest latency the link allows:
 *
 *  THE INPUT PATH ("latest wins mailbox")
 *  --------------------------------------
 *  Your code calls floss_hid_set_input() whenever the controller state
 *  changes - from the main loop, an ISR, or another core. It only copies the
 *  bytes into a per-report-ID slot (seqlock, no locks held, no allocation).
 *  The stack sends the newest contents:
 *    - report_interval_us == 0: as soon as a controller ACL credit is free
 *      (event driven, "send on change");
 *    - report_interval_us  > 0: on a drift-free microsecond timer, every
 *      interval, if the data changed (or always, see send_unchanged).
 *  A report that is replaced before it is sent is simply superseded (counted
 *  in stats as coalesced) - a gamepad must never queue stale state.
 *  Input packets are FLUSHABLE + PRIORITY, so with a flush timeout set the
 *  controller drops reports it could not deliver in time.
 *
 *  KEYBOARDS, MICE AND THE BOOT PROTOCOL
 *  -------------------------------------
 *  A keyboard or mouse that sets cfg.boot_device may be switched by the host
 *  (PC firmware, pre-boot environments, minimal hosts) to BOOT protocol,
 *  where it must send the fixed formats of the USB HID 1.11 boot appendix
 *  instead of its descriptor's reports:
 *
 *    keyboard (ID 1): modifiers, reserved, 6 key codes        (8 bytes)
 *    mouse    (ID 2): buttons, X, Y [, wheel, ...]            (>= 3 bytes)
 *
 *  Every connection starts in REPORT protocol. Boot reports have their own
 *  two mailbox slots, filled with floss_hid_set_boot_input(); the stack
 *  sends from the slots of the CURRENT protocol only and, right after a
 *  switch, sends the newest state of the new protocol's slots once. So the
 *  simplest application publishes both formats on every change:
 *
 *      floss_hid_set_input(KBD_REPORT_ID, report, sizeof report);
 *      floss_hid_set_boot_input(FLOSS_HID_BOOT_KEYBOARD, boot, 8);
 *
 *  (two small copies, no protocol bookkeeping). An application that prefers
 *  to publish one format only can follow on_set_protocol / floss_hid_protocol()
 *  (and must then publish the new format when the protocol changes).
 *  In boot protocol the report ID byte is always on the wire (even with a
 *  descriptor without IDs): output reports such as the keyboard LEDs reach
 *  on_output_report with report_id FLOSS_HID_BOOT_KEYBOARD, and GET_REPORT
 *  (input) is answered from the boot slots. SET_PROTOCOL(boot) on a device
 *  without cfg.boot_device is refused with ERR_INVALID_PARAMETER.
 *
 *  Large transfers on the control channel: a SET_REPORT larger than the
 *  channel MTU arrives as SET_REPORT + DATC continuations and is reassembled
 *  (up to report ID + FLOSS_HID_MAX_REPORT bytes) before the callbacks run;
 *  GET_REPORT answers larger than the host's MTU leave as DATA + DATC.
 */
#ifndef FLOSS_HID_H
#define FLOSS_HID_H

#include "floss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Report types (HIDP) */
#define FLOSS_HID_REPORT_OTHER   0
#define FLOSS_HID_REPORT_INPUT   1
#define FLOSS_HID_REPORT_OUTPUT  2
#define FLOSS_HID_REPORT_FEATURE 3

/* Protocol modes */
#define FLOSS_HID_PROTOCOL_BOOT   0
#define FLOSS_HID_PROTOCOL_REPORT 1

/* Handshake result codes (return these from get/set report callbacks) */
#define FLOSS_HID_HS_SUCCESSFUL          0x0
#define FLOSS_HID_HS_NOT_READY           0x1
#define FLOSS_HID_HS_ERR_INVALID_REPORT_ID 0x2
#define FLOSS_HID_HS_ERR_UNSUPPORTED_REQUEST 0x3
#define FLOSS_HID_HS_ERR_INVALID_PARAMETER 0x4
#define FLOSS_HID_HS_ERR_UNKNOWN         0xE
#define FLOSS_HID_HS_ERR_FATAL           0xF

/* HID_CONTROL operations */
#define FLOSS_HID_CTRL_NOP                  0x0
#define FLOSS_HID_CTRL_HARD_RESET           0x1
#define FLOSS_HID_CTRL_SOFT_RESET           0x2
#define FLOSS_HID_CTRL_SUSPEND              0x3
#define FLOSS_HID_CTRL_EXIT_SUSPEND         0x4
#define FLOSS_HID_CTRL_VIRTUAL_CABLE_UNPLUG 0x5

/* Boot protocol report IDs (fixed by the boot protocol; floss_hid_set_boot_input) */
#define FLOSS_HID_BOOT_KEYBOARD 1 /**< input: 8 bytes; output: 1 byte of LEDs */
#define FLOSS_HID_BOOT_MOUSE    2 /**< input: buttons, X, Y (+ optional wheel, ...) */

#define FLOSS_HID_BOOT_KEYBOARD_LEN  8
#define FLOSS_HID_BOOT_MOUSE_MIN_LEN 3

/* Boot keyboard modifier byte (byte 0 of the keyboard boot report) */
#define FLOSS_HID_MOD_LCTRL  0x01
#define FLOSS_HID_MOD_LSHIFT 0x02
#define FLOSS_HID_MOD_LALT   0x04
#define FLOSS_HID_MOD_LGUI   0x08
#define FLOSS_HID_MOD_RCTRL  0x10
#define FLOSS_HID_MOD_RSHIFT 0x20
#define FLOSS_HID_MOD_RALT   0x40
#define FLOSS_HID_MOD_RGUI   0x80

/* Boot keyboard LED output byte */
#define FLOSS_HID_LED_NUM_LOCK    0x01
#define FLOSS_HID_LED_CAPS_LOCK   0x02
#define FLOSS_HID_LED_SCROLL_LOCK 0x04
#define FLOSS_HID_LED_COMPOSE     0x08
#define FLOSS_HID_LED_KANA        0x10

/* Boot mouse button byte */
#define FLOSS_HID_MOUSE_LEFT   0x01
#define FLOSS_HID_MOUSE_RIGHT  0x02
#define FLOSS_HID_MOUSE_MIDDLE 0x04

/* SDP device subclass values */
#define FLOSS_HID_SUBCLASS_JOYSTICK 0x04
#define FLOSS_HID_SUBCLASS_GAMEPAD  0x08
#define FLOSS_HID_SUBCLASS_KEYBOARD 0x40
#define FLOSS_HID_SUBCLASS_MOUSE    0x80
#define FLOSS_HID_SUBCLASS_COMBO    0xC0

typedef enum {
    FLOSS_HID_STATE_IDLE = 0,      /**< no host */
    FLOSS_HID_STATE_CONNECTING,    /**< paging host / opening channels */
    FLOSS_HID_STATE_CONNECTED,     /**< both channels open: reports flow */
    FLOSS_HID_STATE_DISCONNECTING,
} floss_hid_state_t;

typedef struct {
    /* --- identity / SDP --------------------------------------------------- */
    const char *name;                /**< local name + SDP service name */
    const uint8_t *report_descriptor;
    uint16_t report_descriptor_len;
    uint8_t subclass;                /**< FLOSS_HID_SUBCLASS_* */
    uint32_t class_of_device;        /**< 0 = derive from subclass */
    uint16_t vendor_id_source;       /**< Device ID record: 1 or 2 (0 = no DID record) */
    uint16_t vendor_id;
    uint16_t product_id;
    uint16_t version;
    /**
     * Supports boot protocol (keyboards, mice). Which boot reports exist
     * follows the subclass: KEYBOARD -> keyboard, MOUSE -> mouse, COMBO (or
     * anything else) -> both.
     */
    bool boot_device;
    bool virtual_cable;              /**< default true */
    bool reconnect_initiate;         /**< device reconnects to host (default true) */
    bool normally_connectable;
    bool battery_power;
    bool remote_wake;

    /* --- behaviour -------------------------------------------------------- */
    /** 0 = send on change as fast as credits allow; >0 = paced period in us. */
    uint32_t report_interval_us;
    /** With pacing: also send when data did not change (some hosts need a steady stream). */
    bool send_unchanged;
    /** Automatic flush timeout for the link in ms (0 = reliable). Default 0. */
    uint16_t flush_timeout_ms;
    /** Become discoverable automatically when there is no bond. */
    bool auto_discoverable;
    /** Page the last bonded host at start-up. */
    bool auto_reconnect;
    /**
     * Latency profile: if non-zero, request sniff mode with this max interval
     * (slots) once connected; 0 keeps the link active (lowest latency, more
     * power). Hosts may impose their own.
     */
    uint16_t sniff_max_interval;
    /**
     * Max input reports handed to the controller but not yet acknowledged
     * (0 = as many as the controller has buffers). Keeping this small (1-2)
     * is the single most important latency setting: reports queued inside the
     * controller cannot be replaced by newer state, so each one in the queue
     * adds a full air-interface round of delay. Default 2. The effective cap
     * is multiplied by the chipset's completion_batch (CYW43439: 2 -> 4),
     * because a controller that reports completions late/in pairs would
     * otherwise leave transmit opportunities unused.
     */
    uint8_t max_inflight;
    /**
     * If non-zero, request this poll latency from the host after connecting
     * (floss_gap_qos_setup). 1250 us asks for polling every 2 slots. Default 0.
     */
    uint32_t qos_latency_us;
    /**
     * Let our controller run adaptive-hopping channel assessment. As a
     * peripheral the device only assesses to report to the host, and on the
     * CYW43439 each assessment leaves the link for ~25 ms (periodic report
     * gaps). The host keeps assessing on its own. Default false.
     */
    bool afh_assessment;

    /* --- integration ------------------------------------------------------ */
    /**
     * Serve this pre-serialised HID service record instead of building one
     * from the fields above (for hosts that check the record byte for byte).
     * Same format as floss_sdp_register(): it must start with attribute
     * 0x0000 as a uint32, whose value is replaced by the assigned handle.
     * Copied at floss_hid_init(). NULL = build the record.
     */
    const uint8_t *sdp_record;
    uint16_t sdp_record_len;
    /**
     * Accept and open the HID channels without authentication or encryption,
     * for hosts that authenticate on their own terms and break when the
     * device asks for security first. Default false (channels need an
     * encrypted link).
     */
    bool no_security;
    /**
     * The application alone decides when the device is discoverable and
     * connectable (floss_gap_set_scan()); the HID layer never changes it.
     * auto_discoverable is then ignored. Default false.
     */
    bool app_controls_scan;
} floss_hid_config_t;

/** Fill a config with gamepad-friendly defaults (name/descriptor still required). */
void floss_hid_config_defaults(floss_hid_config_t *cfg);

typedef struct {
    /** Both channels open; reports may be sent. */
    void (*on_connected)(const floss_addr_t *host);
    /** Host gone (reason = HCI reason or 0 for local). */
    void (*on_disconnected)(uint8_t reason);
    /**
     * Output report from the host (rumble, LEDs...). Arrives via DATA on the
     * interrupt channel or SET_REPORT(OUTPUT) on control. report_id is 0 when
     * the descriptor uses no IDs.
     */
    void (*on_output_report)(uint8_t report_id, const uint8_t *data, uint16_t len);
    /**
     * GET_REPORT. Write the report (WITHOUT the ID byte) into out (cap bytes),
     * set *len, and return FLOSS_HID_HS_SUCCESSFUL, or return an error
     * handshake. If NULL, input reports are answered from the mailbox.
     */
    uint8_t (*on_get_report)(uint8_t type, uint8_t report_id, uint8_t *out, uint16_t cap, uint16_t *len);
    /** SET_REPORT (feature/output on control). Return a handshake code. */
    uint8_t (*on_set_report)(uint8_t type, uint8_t report_id, const uint8_t *data, uint16_t len);
    /**
     * SET_PROTOCOL (boot/report) accepted. Not called for the REPORT
     * protocol every connection implicitly starts in.
     */
    void (*on_set_protocol)(uint8_t protocol);
    /** HID_CONTROL operations (suspend, exit suspend, unplug...). */
    void (*on_control)(uint8_t op);
    /**
     * Called just before a paced report is sent (report_interval_us > 0):
     * the last moment to call floss_hid_set_input() for this frame. Use it
     * to sample inputs "just in time" and minimise report age.
     */
    void (*on_report_tick)(void);
    /**
     * A floss_hid_connect() (or auto reconnect) gave up without a session.
     * reason: HCI status of the last failure (page timeout, key missing...,
     * FLOSS_HCI_ERR_LOCAL_HOST_TERMINATED when the link came up but the
     * session did not). channel_status: on_open status of the HID channel
     * that failed to open on the last attempt (FLOSS_L2CAP_CONN_* or
     * FLOSS_L2CAP_STATUS_*), 0 if no channel failed.
     */
    void (*on_connect_failed)(uint8_t reason, uint16_t channel_status);
} floss_hid_callbacks_t;

/** Initialise. Call after floss_init(), before floss_start(). cfg and descriptor must stay valid. */
floss_err_t floss_hid_init(const floss_hid_config_t *cfg, const floss_hid_callbacks_t *cb);

/**
 * Publish the newest input report (ISR / other-core safe, never blocks).
 * report_id 0 = descriptor without report IDs. len <= FLOSS_HID_MAX_REPORT.
 */
floss_err_t floss_hid_set_input(uint8_t report_id, const uint8_t *data, uint16_t len);

/**
 * Publish the newest BOOT protocol input report (ISR / other-core safe, never
 * blocks). boot_id = FLOSS_HID_BOOT_KEYBOARD (len 8) or FLOSS_HID_BOOT_MOUSE
 * (len 3..FLOSS_HID_MAX_REPORT), data WITHOUT the ID byte. It is sent only
 * while the host has selected boot protocol; in report protocol the slot is
 * just kept current for a later switch. FLOSS_ERR_UNSUPPORTED if the device
 * does not provide that boot report (see floss_hid_config_t.boot_device).
 * One writer per boot_id, as for floss_hid_set_input().
 */
floss_err_t floss_hid_set_boot_input(uint8_t boot_id, const uint8_t *data, uint16_t len);

/**
 * Send a report immediately, bypassing the mailbox (stack context only).
 * Returns FLOSS_ERR_BUSY if no controller credit is free right now.
 */
floss_err_t floss_hid_send_input_now(uint8_t report_id, const uint8_t *data, uint16_t len);

/**
 * True when a report handed over now (floss_hid_send_input_now()) would go
 * to the controller straight away, within the max_inflight cap. Lets an
 * application build a report only when it can leave (stack context only).
 */
bool floss_hid_can_send_now(void);

/** Reconnect to a host (NULL = most recently bonded). */
floss_err_t floss_hid_connect(const floss_addr_t *host);
/** Close both channels and the link. */
floss_err_t floss_hid_disconnect(void);
/** Send VIRTUAL_CABLE_UNPLUG, disconnect and forget the bond. */
floss_err_t floss_hid_unplug(void);

floss_hid_state_t floss_hid_state(void);
static inline bool floss_hid_connected(void) {
    return floss_hid_state() == FLOSS_HID_STATE_CONNECTED;
}
/** Current protocol mode (FLOSS_HID_PROTOCOL_*). */
uint8_t floss_hid_protocol(void);
/** ACL handle of the host link (0xFFFF if none). */
uint16_t floss_hid_handle(void);

/** Change the in-flight cap at runtime (see floss_hid_config_t.max_inflight). */
void floss_hid_set_max_inflight(uint8_t max_inflight);

/** Change pacing at runtime (e.g. host asked for a different rate). */
void floss_hid_set_report_interval(uint32_t interval_us);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_HID_H */
