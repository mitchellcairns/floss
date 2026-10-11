/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * gamepad_app.c - Portable Classic HID gamepad: descriptor, identity, demo.
 *
 * Design:
 *  - The report descriptor, demo animation and report layout live in
 *    gamepad_state.c, shared with the pad <-> dongle link firmwares
 *    (ports/pico_w/link), so all of them speak the same reports.
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
/* State                                                                    */
/* ------------------------------------------------------------------------ */

static gamepad_state_t app_in;      /* inputs fed by the port */
static bool app_demo = GAMEPAD_DEMO;
static uint8_t app_last[GAMEPAD_INPUT_REPORT_LEN];
static bool app_published;
static gamepad_output_t app_out;
static floss_hid_config_t app_cfg;

/* ------------------------------------------------------------------------ */
/* Report publishing                                                        */
/* ------------------------------------------------------------------------ */

void gamepad_app_task(void) {
    gamepad_state_t s = app_in;
    if (app_demo) {
        gamepad_state_demo(&s, floss_hal_time_us());
    }
    uint8_t report[GAMEPAD_INPUT_REPORT_LEN];
    gamepad_state_build_report(&s, report);
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
    app_in.lx = gamepad_clamp_axis(lx);
    app_in.ly = gamepad_clamp_axis(ly);
    app_in.rx = gamepad_clamp_axis(rx);
    app_in.ry = gamepad_clamp_axis(ry);
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
    gamepad_output_t o;
    if (!gamepad_output_parse(report_id, data, len, &o)) {
        FLOSS_LOGD(APP, "ignored output report id %u len %u", report_id, len);
        return;
    }
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
    app_cfg.report_descriptor_len = GAMEPAD_REPORT_DESCRIPTOR_LEN;
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
    app_cfg.max_inflight = GAMEPAD_MAX_INFLIGHT;
    app_cfg.qos_latency_us = GAMEPAD_QOS_LATENCY_US;

    gamepad_state_init(&app_in);
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
