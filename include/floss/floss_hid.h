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
    bool boot_device;                /**< supports boot protocol (keyboard/mouse) */
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
     * adds a full air-interface round of delay. Default 2.
     */
    uint8_t max_inflight;
    /**
     * If non-zero, request this poll latency from the host after connecting
     * (floss_gap_qos_setup). 1250 us asks for polling every 2 slots. Default 0.
     */
    uint32_t qos_latency_us;
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
    /** SET_PROTOCOL (boot/report). */
    void (*on_set_protocol)(uint8_t protocol);
    /** HID_CONTROL operations (suspend, exit suspend, unplug...). */
    void (*on_control)(uint8_t op);
    /**
     * Called just before a paced report is sent (report_interval_us > 0):
     * the last moment to call floss_hid_set_input() for this frame. Use it
     * to sample inputs "just in time" and minimise report age.
     */
    void (*on_report_tick)(void);
} floss_hid_callbacks_t;

/** Initialise. Call after floss_init(), before floss_start(). cfg and descriptor must stay valid. */
floss_err_t floss_hid_init(const floss_hid_config_t *cfg, const floss_hid_callbacks_t *cb);

/**
 * Publish the newest input report (ISR / other-core safe, never blocks).
 * report_id 0 = descriptor without report IDs. len <= FLOSS_HID_MAX_REPORT.
 */
floss_err_t floss_hid_set_input(uint8_t report_id, const uint8_t *data, uint16_t len);

/**
 * Send a report immediately, bypassing the mailbox (stack context only).
 * Returns FLOSS_ERR_BUSY if no controller credit is free right now.
 */
floss_err_t floss_hid_send_input_now(uint8_t report_id, const uint8_t *data, uint16_t len);

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
