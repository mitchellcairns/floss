/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * le_gamepad_app.c - LE (HID over GATT) gamepad demo application.
 *
 * Overview
 * --------
 * The LE twin of the Classic gamepad example. It shows the whole surface an
 * LE HID product needs:
 *   1. identity + advertising: name, gamepad appearance, HID service UUID;
 *   2. floss_hogp_init(): report map + report list; the stack builds the HID,
 *      Device Information and Battery services;
 *   3. a loop that samples inputs and calls floss_hogp_set_input() - here a
 *      synthetic pattern (walking button, rotating hat, triangle-wave axes,
 *      ramping triggers) so it works on a bare board.
 * Pairing is "just works": the HID characteristics require encryption, the
 * host pairs on first access and reconnects encrypted afterwards.
 *
 * Reports go out "on change" (report_interval_us = 0): every time the demo
 * publishes new state it is notified at the next connection event the
 * controller has room for, which with the default 7.5-15 ms interval is the
 * lowest latency LE offers.
 */
#include "le_gamepad_app.h"

#include "floss/floss_le.h"
#include "floss/floss_gatt.h"
#include "floss/floss_hogp.h"

#if FLOSS_ENABLE_LE

/* Demo cadence: new state every 4 ms (faster than any connection interval,
 * so the mailbox's latest-wins behaviour is exercised). */
#define DEMO_PERIOD_US 4000u

/* HID report descriptor: one gamepad application collection. */
static const uint8_t k_report_map[] = {
    0x05, 0x01,       /* Usage Page (Generic Desktop) */
    0x09, 0x05,       /* Usage (Game Pad) */
    0xA1, 0x01,       /* Collection (Application) */
    0x85, LE_GAMEPAD_INPUT_ID, /* Report ID (1) */
    /* 16 buttons */
    0x05, 0x09,       /*   Usage Page (Button) */
    0x19, 0x01,       /*   Usage Minimum (1) */
    0x29, 0x10,       /*   Usage Maximum (16) */
    0x15, 0x00,       /*   Logical Minimum (0) */
    0x25, 0x01,       /*   Logical Maximum (1) */
    0x75, 0x01,       /*   Report Size (1) */
    0x95, 0x10,       /*   Report Count (16) */
    0x81, 0x02,       /*   Input (Data, Variable, Absolute) */
    /* hat switch, 8 directions, out-of-range = centred */
    0x05, 0x01,       /*   Usage Page (Generic Desktop) */
    0x09, 0x39,       /*   Usage (Hat Switch) */
    0x15, 0x00,       /*   Logical Minimum (0) */
    0x25, 0x07,       /*   Logical Maximum (7) */
    0x35, 0x00,       /*   Physical Minimum (0) */
    0x46, 0x3B, 0x01, /*   Physical Maximum (315) */
    0x65, 0x14,       /*   Unit (Degrees) */
    0x75, 0x04,       /*   Report Size (4) */
    0x95, 0x01,       /*   Report Count (1) */
    0x81, 0x42,       /*   Input (Data, Variable, Absolute, Null State) */
    0x65, 0x00,       /*   Unit (None) */
    0x45, 0x00,       /*   Physical Maximum (0 = same as logical) */
    0x75, 0x04,       /*   Report Size (4) */
    0x95, 0x01,       /*   Report Count (1) */
    0x81, 0x03,       /*   Input (Constant) - padding */
    /* sticks: X, Y, Z, Rz as signed 16-bit */
    0x09, 0x30,       /*   Usage (X) */
    0x09, 0x31,       /*   Usage (Y) */
    0x09, 0x32,       /*   Usage (Z) */
    0x09, 0x35,       /*   Usage (Rz) */
    0x16, 0x00, 0x80, /*   Logical Minimum (-32768) */
    0x26, 0xFF, 0x7F, /*   Logical Maximum (32767) */
    0x75, 0x10,       /*   Report Size (16) */
    0x95, 0x04,       /*   Report Count (4) */
    0x81, 0x02,       /*   Input (Data, Variable, Absolute) */
    /* triggers: Rx, Ry as unsigned 8-bit */
    0x09, 0x33,       /*   Usage (Rx) */
    0x09, 0x34,       /*   Usage (Ry) */
    0x15, 0x00,       /*   Logical Minimum (0) */
    0x26, 0xFF, 0x00, /*   Logical Maximum (255) */
    0x75, 0x08,       /*   Report Size (8) */
    0x95, 0x02,       /*   Report Count (2) */
    0x81, 0x02,       /*   Input (Data, Variable, Absolute) */
    /* output: rumble left/right + LEDs (vendor usages) */
    0x85, LE_GAMEPAD_OUTPUT_ID, /* Report ID (2) */
    0x06, 0x00, 0xFF, /*   Usage Page (Vendor Defined 0xFF00) */
    0x09, 0x01,       /*   Usage (rumble left) */
    0x09, 0x02,       /*   Usage (rumble right) */
    0x09, 0x03,       /*   Usage (LEDs) */
    0x15, 0x00,       /*   Logical Minimum (0) */
    0x26, 0xFF, 0x00, /*   Logical Maximum (255) */
    0x75, 0x08,       /*   Report Size (8) */
    0x95, 0x03,       /*   Report Count (3) */
    0x91, 0x02,       /*   Output (Data, Variable, Absolute) */
    0xC0,             /* End Collection */
};

static const floss_hogp_report_t k_reports[] = {
    {.id = LE_GAMEPAD_INPUT_ID, .type = FLOSS_HID_REPORT_INPUT, .len = LE_GAMEPAD_INPUT_LEN},
    {.id = LE_GAMEPAD_OUTPUT_ID, .type = FLOSS_HID_REPORT_OUTPUT, .len = LE_GAMEPAD_OUTPUT_LEN},
};

static floss_hogp_config_t s_cfg;

static struct {
    uint32_t next_us;
    uint32_t frame;
} s_demo;

/* ------------------------------------------------------------------------ */
/* Callbacks                                                                */
/* ------------------------------------------------------------------------ */

static void on_connected(uint16_t handle, const floss_addr_t *host) {
    char str[18];
    floss_addr_to_str(host, str);
    FLOSS_LOGI(APP, "host %s listening (link 0x%04x)", str, (unsigned)handle);
}

static void on_disconnected(uint16_t handle, uint8_t reason) {
    FLOSS_LOGI(APP, "host gone: %s", floss_hci_status_str(reason));
}

static void on_output_report(uint8_t report_id, const uint8_t *data, uint16_t len) {
    if (report_id == LE_GAMEPAD_OUTPUT_ID && len >= 3) {
        FLOSS_LOGI(APP, "rumble L=%u R=%u leds=0x%02x", (unsigned)data[0], (unsigned)data[1], (unsigned)data[2]);
        /* Drive motors / LEDs here. */
    }
}

static void on_le_event(const floss_le_evt_t *evt, void *user) {
    switch (evt->type) {
    case FLOSS_LE_EVT_CONN_PARAMS:
        if (evt->status == 0) {
            FLOSS_LOGI(APP, "connection interval %u us", (unsigned)(evt->interval * 1250u));
        }
        break;
    case FLOSS_LE_EVT_SECURITY:
        FLOSS_LOGI(APP, "link security level %u", (unsigned)evt->level);
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------------ */
/* Demo pattern                                                             */
/* ------------------------------------------------------------------------ */

/** Triangle wave in [-32767, 32767] with a period of 512 frames. */
static int16_t triangle(uint32_t frame) {
    uint32_t p = frame & 511u;
    int32_t v = (p < 256u) ? (int32_t)p * 256 - 32767 : (int32_t)(511u - p) * 256 - 32767;
    return (int16_t)v;
}

static void build_report(uint32_t f, uint8_t out[LE_GAMEPAD_INPUT_LEN]) {
    uint16_t buttons = (uint16_t)(1u << ((f / 64u) & 15u)); /* one button walks every ~256 ms */
    uint8_t hat = (uint8_t)((f / 128u) % 9u);               /* 0..7 directions, 8 = centred */
    floss_put_le16(&out[0], buttons);
    out[2] = hat;
    floss_put_le16(&out[3], (uint16_t)triangle(f));
    floss_put_le16(&out[5], (uint16_t)triangle(f + 128u));
    floss_put_le16(&out[7], (uint16_t)triangle(f * 2u));
    floss_put_le16(&out[9], (uint16_t)triangle(f * 2u + 128u));
    out[11] = (uint8_t)f;
    out[12] = (uint8_t)(255u - (uint8_t)f);
}

/* ------------------------------------------------------------------------ */
/* Public                                                                   */
/* ------------------------------------------------------------------------ */

floss_err_t le_gamepad_app_init(void) {
    floss_le_set_name("FLOSS LE Gamepad");
    floss_le_set_appearance(FLOSS_LE_APPEARANCE_GAMEPAD);

    floss_hogp_config_defaults(&s_cfg);
    s_cfg.report_map = k_report_map;
    s_cfg.report_map_len = sizeof(k_report_map);
    s_cfg.reports = k_reports;
    s_cfg.report_count = FLOSS_ARRAY_SIZE(k_reports);
    s_cfg.manufacturer = "FLOSS";
    s_cfg.model = "LE Gamepad";
    s_cfg.vendor_id_source = 2;   /* USB-IF */
    s_cfg.vendor_id = 0x1209;     /* pid.codes open-source VID */
    s_cfg.product_id = 0x0001;    /* placeholder: request your own PID */
    s_cfg.version = 0x0100;
    s_cfg.report_interval_us = 0; /* send on change */

    static const floss_hogp_callbacks_t cb = {
        .on_connected = on_connected,
        .on_disconnected = on_disconnected,
        .on_output_report = on_output_report,
    };
    floss_err_t err = floss_hogp_init(&s_cfg, &cb);
    if (err != FLOSS_OK) {
        FLOSS_LOGE(APP, "hogp init failed: %s", floss_err_str(err));
        return err;
    }
    floss_le_add_listener(on_le_event, NULL);

    /* Advertising: flags + gamepad appearance + HID UUID + name. */
    static const uint16_t uuids[] = {FLOSS_GATT_SVC_HID};
    err = floss_le_adv_set_standard(uuids, FLOSS_ARRAY_SIZE(uuids));
    if (err != FLOSS_OK) {
        return err;
    }
    s_demo.next_us = floss_hal_time_us();
    return floss_le_adv_start();
}

void le_gamepad_app_task(void) {
    uint32_t now = floss_hal_time_us();
    if (floss_time_diff(now, s_demo.next_us) < 0) {
        return;
    }
    s_demo.next_us += DEMO_PERIOD_US;
    if (floss_time_diff(now, s_demo.next_us) > (int32_t)DEMO_PERIOD_US) {
        s_demo.next_us = now + DEMO_PERIOD_US; /* we were starved: skip, don't burst */
    }
    if (!floss_hogp_connected()) {
        return;
    }
    uint8_t report[LE_GAMEPAD_INPUT_LEN];
    build_report(s_demo.frame++, report);
    floss_hogp_set_input(LE_GAMEPAD_INPUT_ID, report, sizeof(report));
}

#else /* !FLOSS_ENABLE_LE */

floss_err_t le_gamepad_app_init(void) {
    return FLOSS_ERR_UNSUPPORTED;
}

void le_gamepad_app_task(void) {
}

#endif /* FLOSS_ENABLE_LE */
