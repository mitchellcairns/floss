/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_hogp.h - HID over GATT, device role (LE gamepads, keyboards, mice).
 *
 * What it does for you:
 *  - Builds and registers the HID Service (0x1812): Report Map (long read),
 *    HID Information, HID Control Point, one Report characteristic per
 *    report (with Report Reference and, for inputs, a CCCD), and Protocol
 *    Mode when the device supports boot protocol. Every HID characteristic
 *    requires an encrypted link, which is what makes hosts pair/bond.
 *  - Registers Device Information (PnP ID) and Battery services, which HOGP
 *    hosts expect next to the HID service.
 *  - Sends input reports as GATT notifications with the same "latest wins
 *    mailbox" as the Classic HID layer (floss_hid.h):
 *
 *  THE INPUT PATH
 *  --------------
 *  floss_hogp_set_input() may be called from the main loop, an ISR or the
 *  other core: it only copies the bytes into the report's seqlock slot. The
 *  stack notifies every subscribed host with the newest contents:
 *    - report_interval_us == 0: as soon as the link can take a packet
 *      (checked on every floss_poll() and on every freed ACL credit);
 *    - report_interval_us  > 0: on a drift-free timer, if the data changed
 *      (or always, with send_unchanged).
 *  At most `max_inflight` packets wait in the controller per link (default
 *  1), so a report never queues behind a stale one: whatever is newest when
 *  the controller frees a buffer is what goes out at the next connection
 *  event. On LE the report ID is NOT part of the notification; the host maps
 *  the characteristic to its ID through the Report Reference descriptor.
 *
 *  Single writer per report ID: the seqlock tolerates one producer per ID
 *  (no atomic read-modify-write is used, Cortex-M0+ has none).
 */
#ifndef FLOSS_HOGP_H
#define FLOSS_HOGP_H

#include "floss_types.h"
#include "floss_hid.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Max reports (input + output + feature) one HID service exposes. */
#ifndef FLOSS_HOGP_MAX_REPORTS
#define FLOSS_HOGP_MAX_REPORTS 6
#endif

/* HID Information flags */
#define FLOSS_HOGP_FLAG_REMOTE_WAKE          0x01
#define FLOSS_HOGP_FLAG_NORMALLY_CONNECTABLE 0x02

/* HID Control Point values */
#define FLOSS_HOGP_CTRL_SUSPEND      0x00
#define FLOSS_HOGP_CTRL_EXIT_SUSPEND 0x01

/** One report exposed as a Report characteristic. */
typedef struct {
    uint8_t id;    /**< report ID (0 when the report map uses no IDs) */
    uint8_t type;  /**< FLOSS_HID_REPORT_INPUT / OUTPUT / FEATURE */
    uint8_t len;   /**< payload length WITHOUT the ID byte (<= FLOSS_HID_MAX_REPORT) */
} floss_hogp_report_t;

typedef struct {
    /* --- HID service ------------------------------------------------------- */
    const uint8_t *report_map;       /**< HID report descriptor */
    uint16_t report_map_len;         /**< <= 512 (ATT attribute limit) */
    const floss_hogp_report_t *reports;
    uint8_t report_count;
    uint8_t country_code;            /**< 0 = not localised */
    bool remote_wake;
    bool normally_connectable;       /**< default true */
    bool boot_device;                /**< expose Protocol Mode (boot reports not provided) */

    /* --- companion services ------------------------------------------------ */
    const char *manufacturer;        /**< Device Information: Manufacturer Name (NULL = omit) */
    const char *model;               /**< Device Information: Model Number (NULL = omit) */
    uint8_t vendor_id_source;        /**< PnP ID: 1 or 2 (0 = omit PnP ID) */
    uint16_t vendor_id;
    uint16_t product_id;
    uint16_t version;
    bool battery_service;            /**< register Battery service (default true) */
    uint8_t battery_level;           /**< initial level (default 100) */

    /* --- behaviour --------------------------------------------------------- */
    /**
     * 0 = notify on change as fast as the link allows; >0 = paced period in us;
     * FLOSS_HOGP_INTERVAL_CONN = one report per connection interval, following
     * the link's current interval (recommended: LE delivers only at connection
     * events, so one fresh report per event is the most consistent schedule).
     */
    uint32_t report_interval_us;
    /** With pacing: also send when data did not change. */
    bool send_unchanged;
    /**
     * Max un-acknowledged ACL packets per link before reports wait (default 2,
     * multiplied by the chipset's completion_batch). LE acknowledges a
     * notification only at the next connection event, so 1 would skip every
     * other event when pacing at the connection interval.
     */
    uint8_t max_inflight;
} floss_hogp_config_t;

/** report_interval_us value: pace at the connection interval. */
#define FLOSS_HOGP_INTERVAL_CONN 1u

/** Fill a config with gamepad-friendly defaults (report map/reports still required). */
void floss_hogp_config_defaults(floss_hogp_config_t *cfg);

typedef struct {
    /** A host enabled notifications on an input report over an encrypted link: reports flow. */
    void (*on_connected)(uint16_t handle, const floss_addr_t *host);
    /** A host that was connected went away (reason = HCI reason). */
    void (*on_disconnected)(uint16_t handle, uint8_t reason);
    /** Output report written by the host (rumble, LEDs...). */
    void (*on_output_report)(uint8_t report_id, const uint8_t *data, uint16_t len);
    /**
     * Host reads a feature report (or an input report, if you want to serve
     * it yourself). Write up to cap bytes into out, set *len, return 0 or a
     * FLOSS_ATT_ERR_* code. NULL: inputs are answered from the mailbox,
     * features read as zeros.
     */
    uint8_t (*on_get_report)(uint8_t type, uint8_t report_id, uint8_t *out, uint16_t cap, uint16_t *len);
    /** Host writes a feature report. Return 0 or a FLOSS_ATT_ERR_* code. */
    uint8_t (*on_set_report)(uint8_t type, uint8_t report_id, const uint8_t *data, uint16_t len);
    /** Protocol Mode written (FLOSS_HID_PROTOCOL_BOOT / _REPORT). */
    void (*on_set_protocol)(uint8_t protocol);
    /** HID Control Point written (FLOSS_HOGP_CTRL_SUSPEND / _EXIT_SUSPEND). */
    void (*on_control)(uint8_t op);
    /** Paced mode: called right before each tick's send, the moment to sample inputs. */
    void (*on_report_tick)(void);
} floss_hogp_callbacks_t;

/**
 * Register the services. Call after floss_init(), before floss_start().
 * cfg, its arrays and the report map must stay valid.
 */
floss_err_t floss_hogp_init(const floss_hogp_config_t *cfg, const floss_hogp_callbacks_t *cb);

/**
 * Publish the newest input report (ISR / other-core safe, never blocks).
 * len <= the report's configured len. Returns FLOSS_ERR_NOT_FOUND for an
 * unknown input report ID.
 */
floss_err_t floss_hogp_set_input(uint8_t report_id, const uint8_t *data, uint16_t len);

/**
 * Notify a report immediately to every subscribed host, bypassing the
 * mailbox (stack context only). FLOSS_ERR_BUSY if no link could take it now.
 */
floss_err_t floss_hogp_send_input_now(uint8_t report_id, const uint8_t *data, uint16_t len);

/** True if at least one host is subscribed to input reports. */
bool floss_hogp_connected(void);
/** Link handle of the first subscribed host (0xFFFF if none). */
uint16_t floss_hogp_handle(void);
/** Current protocol mode (FLOSS_HID_PROTOCOL_*). */
uint8_t floss_hogp_protocol(void);
/** True after the host wrote SUSPEND to the HID Control Point. */
bool floss_hogp_suspended(void);
/** Change pacing at runtime (0 = send on change). */
void floss_hogp_set_report_interval(uint32_t interval_us);

/* ------------------------------------------------------------------------ */
/* Lifecycle (called by the LE module)                                      */
/* ------------------------------------------------------------------------ */

void floss_hogp_reset(void);
void floss_hogp_on_link_down(uint16_t handle, uint8_t reason);
void floss_hogp_on_security(uint16_t handle, uint8_t level);
/** ACL credits freed: pending reports may go out. */
void floss_hogp_on_credits(void);
/** Per-poll check for new mailbox contents (send-on-change mode). */
void floss_hogp_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_HOGP_H */
