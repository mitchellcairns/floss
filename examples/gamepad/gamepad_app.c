/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * gamepad_app.c - Portable Classic HID gamepad: descriptor, identity, demo.
 *
 * Design:
 *  - The report descriptor is a plain Generic Desktop "Game Pad" built only
 *    from usages every generic HID driver understands (Windows DirectInput /
 *    HID game controllers, Linux hid-generic + evdev, Android, macOS
 *    GameController/IOHID), so no host-side driver is needed.
 *  - Inputs live in a small state struct. gamepad_app_task() serialises it
 *    into the 13-byte input report and publishes it with
 *    floss_hid_set_input() only when the bytes differ from the last publish.
 *    The HID layer then sends it at the configured pace (1 kHz by default):
 *    the stack, not the app, decides when the radio is used.
 *  - Output reports (rumble/LED) arrive via on_output_report, from either
 *    the interrupt channel or SET_REPORT on control - the HID layer merges
 *    both paths.
 *  - Everything is integer math; no floats, no libm, no heap.
 */
#include "gamepad_app.h"

/* ------------------------------------------------------------------------ */
/* Report descriptor                                                        */
/* ------------------------------------------------------------------------ */

static const uint8_t gamepad_report_descriptor[] = {
    0x05, 0x01,             /* Usage Page (Generic Desktop)                    */
    0x09, 0x05,             /* Usage (Game Pad)                                */
    0xA1, 0x01,             /* Collection (Application)                        */

    /* ---- Input report 1 ------------------------------------------------- */
    0x85, GAMEPAD_REPORT_ID_INPUT, /* Report ID (1)                            */

    /* 16 buttons, 1 bit each */
    0x05, 0x09,             /*   Usage Page (Button)                           */
    0x19, 0x01,             /*   Usage Minimum (Button 1)                      */
    0x29, 0x10,             /*   Usage Maximum (Button 16)                     */
    0x15, 0x00,             /*   Logical Minimum (0)                           */
    0x25, 0x01,             /*   Logical Maximum (1)                           */
    0x75, 0x01,             /*   Report Size (1)                               */
    0x95, 0x10,             /*   Report Count (16)                             */
    0x81, 0x02,             /*   Input (Data, Var, Abs)                        */

    /* Hat switch: 8 directions, out-of-range value (8) = centred */
    0x05, 0x01,             /*   Usage Page (Generic Desktop)                  */
    0x09, 0x39,             /*   Usage (Hat Switch)                            */
    0x15, 0x00,             /*   Logical Minimum (0)                           */
    0x25, 0x07,             /*   Logical Maximum (7)                           */
    0x35, 0x00,             /*   Physical Minimum (0)                          */
    0x46, 0x3B, 0x01,       /*   Physical Maximum (315)                        */
    0x65, 0x14,             /*   Unit (English Rotation: Degrees)              */
    0x75, 0x04,             /*   Report Size (4)                               */
    0x95, 0x01,             /*   Report Count (1)                              */
    0x81, 0x42,             /*   Input (Data, Var, Abs, Null State)            */
    0x65, 0x00,             /*   Unit (None)                                   */
    0x45, 0x00,             /*   Physical Maximum (0): physical = logical again */
    0x75, 0x04,             /*   Report Size (4)                               */
    0x95, 0x01,             /*   Report Count (1)                              */
    0x81, 0x03,             /*   Input (Const, Var, Abs) - padding             */

    /* Sticks: X, Y (left), Z, Rz (right) as signed 16-bit */
    0x09, 0x30,             /*   Usage (X)                                     */
    0x09, 0x31,             /*   Usage (Y)                                     */
    0x09, 0x32,             /*   Usage (Z)                                     */
    0x09, 0x35,             /*   Usage (Rz)                                    */
    0x16, 0x01, 0x80,       /*   Logical Minimum (-32767)                      */
    0x26, 0xFF, 0x7F,       /*   Logical Maximum (32767)                       */
    0x75, 0x10,             /*   Report Size (16)                              */
    0x95, 0x04,             /*   Report Count (4)                              */
    0x81, 0x02,             /*   Input (Data, Var, Abs)                        */

    /* Triggers: Rx (left), Ry (right) as unsigned 8-bit. Rx/Ry are mapped
     * as analog axes by every generic driver; the Simulation page's
     * Brake/Accelerator are not on all of them. */
    0x09, 0x33,             /*   Usage (Rx)                                    */
    0x09, 0x34,             /*   Usage (Ry)                                    */
    0x15, 0x00,             /*   Logical Minimum (0)                           */
    0x26, 0xFF, 0x00,       /*   Logical Maximum (255)                         */
    0x75, 0x08,             /*   Report Size (8)                               */
    0x95, 0x02,             /*   Report Count (2)                              */
    0x81, 0x02,             /*   Input (Data, Var, Abs)                        */

    /* ---- Output report 2: rumble strong, rumble weak, LED --------------- */
    /* A vendor-defined page keeps generic drivers from misinterpreting the
     * bytes (there is no universal standard rumble usage outside the PID
     * force-feedback page, which needs a far larger descriptor). Hosts or
     * games that know this device write it with SET_REPORT / DATA. */
    0x85, GAMEPAD_REPORT_ID_OUTPUT, /* Report ID (2)                           */
    0x06, 0x00, 0xFF,       /*   Usage Page (Vendor Defined 0xFF00)            */
    0x09, 0x01,             /*   Usage (0x01: rumble strong)                   */
    0x09, 0x02,             /*   Usage (0x02: rumble weak)                     */
    0x09, 0x03,             /*   Usage (0x03: LED / player)                    */
    0x15, 0x00,             /*   Logical Minimum (0)                           */
    0x26, 0xFF, 0x00,       /*   Logical Maximum (255)                         */
    0x75, 0x08,             /*   Report Size (8)                               */
    0x95, 0x03,             /*   Report Count (3)                              */
    0x91, 0x02,             /*   Output (Data, Var, Abs)                       */

    0xC0,                   /* End Collection                                  */
};

/* ------------------------------------------------------------------------ */
/* State                                                                    */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint32_t buttons;
    uint8_t hat;
    int16_t lx, ly, rx, ry;
    uint8_t lt, rt;
} gamepad_state_t;

static gamepad_state_t app_in;      /* inputs fed by the port */
static bool app_demo = GAMEPAD_DEMO;
static uint8_t app_last[GAMEPAD_INPUT_REPORT_LEN];
static bool app_published;
static gamepad_output_t app_out;
static floss_hid_config_t app_cfg;

/* ------------------------------------------------------------------------ */
/* Demo animation (integer sine, 256 steps per turn)                        */
/* ------------------------------------------------------------------------ */

/* 32767 * sin(i * pi / 128), i = 0..64 (one quarter wave). */
static const int16_t quarter_sine[65] = {
    0,     804,   1608,  2410,  3212,  4011,  4808,  5602,  6393,
    7179,  7962,  8739,  9512,  10278, 11039, 11793, 12539, 13279,
    14010, 14732, 15446, 16151, 16846, 17530, 18204, 18868, 19519,
    20159, 20787, 21403, 22005, 22594, 23170, 23731, 24279, 24811,
    25329, 25832, 26319, 26790, 27245, 27683, 28105, 28510, 28898,
    29268, 29621, 29956, 30273, 30571, 30852, 31113, 31356, 31580,
    31785, 31971, 32137, 32285, 32412, 32521, 32609, 32678, 32728,
    32757, 32767,
};

static int16_t sin256(uint8_t a) {
    uint8_t q = a & 0x3F;
    switch (a >> 6) {
    case 0: return quarter_sine[q];
    case 1: return quarter_sine[64 - q];
    case 2: return (int16_t)-quarter_sine[q];
    default: return (int16_t)-quarter_sine[64 - q];
    }
}

static void demo_state(gamepad_state_t *s, uint32_t now_us) {
    /* 2 s per revolution, 80 % deflection. */
    uint8_t angle = (uint8_t)(((now_us % 2000000u) * 256u) / 2000000u);
    s->lx = (int16_t)((int32_t)sin256((uint8_t)(angle + 64)) * 4 / 5); /* cos */
    s->ly = (int16_t)((int32_t)sin256(angle) * 4 / 5);
    /* Triggers ramp up and down with the circle. */
    s->lt = (uint8_t)(angle < 128 ? angle * 2 : (255 - angle) * 2);
    s->rt = (uint8_t)(255 - s->lt);
    /*
     * Right stick X is a millisecond counter: it changes every 1 ms, so the
     * report stream exercises the full pacing rate and a host can measure
     * report interval / loss directly from the axis value.
     */
    s->rx = (int16_t)((int32_t)((now_us / 1000u) % 65535u) - 32767);
#if GAMEPAD_DEMO_BUTTONS
    /* One button at a time, half a second each. */
    s->buttons |= 1u << ((now_us / 500000u) % 16u);
#endif
}

/* ------------------------------------------------------------------------ */
/* Report building                                                          */
/* ------------------------------------------------------------------------ */

static void build_report(const gamepad_state_t *s, uint8_t out[GAMEPAD_INPUT_REPORT_LEN]) {
    floss_put_le16(&out[0], (uint16_t)(s->buttons & 0xFFFFu));
    out[2] = (uint8_t)(s->hat <= GAMEPAD_HAT_CENTERED ? s->hat : GAMEPAD_HAT_CENTERED);
    floss_put_le16(&out[3], (uint16_t)s->lx);
    floss_put_le16(&out[5], (uint16_t)s->ly);
    floss_put_le16(&out[7], (uint16_t)s->rx);
    floss_put_le16(&out[9], (uint16_t)s->ry);
    out[11] = s->lt;
    out[12] = s->rt;
}

/* -32768 is outside the declared logical range; clamp it. */
static int16_t clamp_axis(int16_t v) {
    return v < -32767 ? (int16_t)-32767 : v;
}

void gamepad_app_task(void) {
    gamepad_state_t s = app_in;
    if (app_demo) {
        demo_state(&s, floss_hal_time_us());
    }
    uint8_t report[GAMEPAD_INPUT_REPORT_LEN];
    build_report(&s, report);
    if (app_published && memcmp(report, app_last, sizeof(report)) == 0) {
        return; /* unchanged: nothing to publish */
    }
    if (floss_hid_set_input(GAMEPAD_REPORT_ID_INPUT, report, sizeof(report)) == FLOSS_OK) {
        memcpy(app_last, report, sizeof(report));
        app_published = true;
    }
}

void gamepad_app_set_demo(bool enabled) {
    app_demo = enabled;
}

void gamepad_app_set_buttons(uint32_t buttons) {
    app_in.buttons = buttons;
}

void gamepad_app_set_hat(uint8_t hat) {
    app_in.hat = hat;
}

void gamepad_app_set_sticks(int16_t lx, int16_t ly, int16_t rx, int16_t ry) {
    app_in.lx = clamp_axis(lx);
    app_in.ly = clamp_axis(ly);
    app_in.rx = clamp_axis(rx);
    app_in.ry = clamp_axis(ry);
}

void gamepad_app_set_triggers(uint8_t left, uint8_t right) {
    app_in.lt = left;
    app_in.rt = right;
}

const gamepad_output_t *gamepad_app_output(void) {
    return &app_out;
}

/* ------------------------------------------------------------------------ */
/* HID callbacks                                                            */
/* ------------------------------------------------------------------------ */

static void on_connected(const floss_addr_t *host) {
    char s[18];
    floss_addr_to_str(host, s);
    FLOSS_LOGI(APP, "gamepad connected to %s", s);
}

static void on_disconnected(uint8_t reason) {
    FLOSS_LOGI(APP, "gamepad disconnected (%s)", floss_hci_status_str(reason));
    /* Stop any rumble the host left running. */
    memset(&app_out, 0, sizeof(app_out));
}

static void on_output_report(uint8_t report_id, const uint8_t *data, uint16_t len) {
    if (report_id != GAMEPAD_REPORT_ID_OUTPUT || len < GAMEPAD_OUTPUT_REPORT_LEN) {
        FLOSS_LOGD(APP, "ignored output report id %u len %u", report_id, len);
        return;
    }
    gamepad_output_t o = {.rumble_strong = data[0], .rumble_weak = data[1], .led = data[2]};
    if (memcmp(&o, &app_out, sizeof(o)) != 0) {
        app_out = o;
        FLOSS_LOGI(APP, "output: rumble strong %u weak %u, led 0x%02X", o.rumble_strong, o.rumble_weak, o.led);
    }
}

static void on_control(uint8_t op) {
    if (op == FLOSS_HID_CTRL_VIRTUAL_CABLE_UNPLUG) {
        FLOSS_LOGI(APP, "host unpaired us");
    }
}

static const floss_hid_callbacks_t app_callbacks = {
    .on_connected = on_connected,
    .on_disconnected = on_disconnected,
    .on_output_report = on_output_report,
    .on_get_report = NULL,  /* input GET_REPORT answered from the mailbox */
    .on_set_report = NULL,  /* output SET_REPORT -> on_output_report */
    .on_set_protocol = NULL,
    .on_control = on_control,
    .on_report_tick = NULL,
};

floss_err_t gamepad_app_init(void) {
    floss_hid_config_defaults(&app_cfg);
    app_cfg.name = "FLOSS Gamepad";
    app_cfg.report_descriptor = gamepad_report_descriptor;
    app_cfg.report_descriptor_len = sizeof(gamepad_report_descriptor);
    app_cfg.subclass = FLOSS_HID_SUBCLASS_GAMEPAD;

    /* Device ID record. 0x1209 is the pid.codes open-source vendor ID;
     * product 0x0F10 is a PLACEHOLDER - a real product must register its
     * own product ID at https://pid.codes before shipping. */
    app_cfg.vendor_id_source = 2; /* USB-IF assigned vendor ID */
    app_cfg.vendor_id = 0x1209;
    app_cfg.product_id = 0x0F10;
    app_cfg.version = 0x0100;

    app_cfg.report_interval_us = GAMEPAD_REPORT_INTERVAL_US;
    app_cfg.flush_timeout_ms = GAMEPAD_FLUSH_TIMEOUT_MS;

    app_in.hat = GAMEPAD_HAT_CENTERED;
    memset(&app_out, 0, sizeof(app_out));
    app_published = false;

    floss_err_t err = floss_hid_init(&app_cfg, &app_callbacks);
    if (err != FLOSS_OK) {
        FLOSS_LOGE(APP, "floss_hid_init failed: %s", floss_err_str(err));
        return err;
    }
    /* Publish a neutral report right away so GET_REPORT has an answer. */
    gamepad_app_task();
    return FLOSS_OK;
}
