/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * keyboard_app.c - Portable Classic HID keyboard + mouse combo: descriptor,
 * identity, boot protocol, LEDs and a typing / pointer demo.
 *
 * Design:
 *  - One report descriptor with two application collections: a keyboard
 *    (report ID 1: input 8 bytes, LED output 1 byte) and a mouse (report
 *    ID 2: 3 buttons, X, Y, wheel). Both use the boot protocol layouts of
 *    the USB HID 1.11 boot appendix, so report and boot protocol carry the
 *    same bytes and every change is simply published twice
 *    (floss_hid_set_input + floss_hid_set_boot_input). The stack sends the
 *    format the host selected; no protocol bookkeeping here.
 *  - Send on change (report_interval_us = 0): a keyboard needs no steady
 *    stream, and a key press leaves as soon as the link has a credit.
 *  - LEDs arrive through on_output_report as report ID 1 in either
 *    protocol (boot protocol output reports always carry ID 1).
 *  - The demo is a small state machine driven by keyboard_app_task(): it
 *    holds each key state for KEYBOARD_KEY_HOLD_US so the latest-wins
 *    mailbox never drops a press, and ends every mouse movement with a
 *    zero-motion report.
 */
#include "keyboard_app.h"

/* ------------------------------------------------------------------------ */
/* Report descriptor                                                        */
/* ------------------------------------------------------------------------ */

static const uint8_t keyboard_report_descriptor[] = {
    /* ---- Keyboard: report ID 1 ------------------------------------------- */
    0x05, 0x01,             /* Usage Page (Generic Desktop)                    */
    0x09, 0x06,             /* Usage (Keyboard)                                */
    0xA1, 0x01,             /* Collection (Application)                        */
    0x85, KEYBOARD_REPORT_ID_KEYS, /* Report ID (1)                            */
    /* byte 0: modifier bits, Left Control .. Right GUI */
    0x05, 0x07,             /*   Usage Page (Keyboard/Keypad)                  */
    0x19, 0xE0,             /*   Usage Minimum (Left Control)                  */
    0x29, 0xE7,             /*   Usage Maximum (Right GUI)                     */
    0x15, 0x00,             /*   Logical Minimum (0)                           */
    0x25, 0x01,             /*   Logical Maximum (1)                           */
    0x75, 0x01,             /*   Report Size (1)                               */
    0x95, 0x08,             /*   Report Count (8)                              */
    0x81, 0x02,             /*   Input (Data, Var, Abs)                        */
    /* byte 1: reserved */
    0x75, 0x08,             /*   Report Size (8)                               */
    0x95, 0x01,             /*   Report Count (1)                              */
    0x81, 0x01,             /*   Input (Const)                                 */
    /* output byte: 5 LEDs + 3 bits padding */
    0x05, 0x08,             /*   Usage Page (LEDs)                             */
    0x19, 0x01,             /*   Usage Minimum (Num Lock)                      */
    0x29, 0x05,             /*   Usage Maximum (Kana)                          */
    0x75, 0x01,             /*   Report Size (1)                               */
    0x95, 0x05,             /*   Report Count (5)                              */
    0x91, 0x02,             /*   Output (Data, Var, Abs)                       */
    0x75, 0x03,             /*   Report Size (3)                               */
    0x95, 0x01,             /*   Report Count (1)                              */
    0x91, 0x01,             /*   Output (Const)                                */
    /* bytes 2..7: up to six pressed keys (array of usage codes) */
    0x05, 0x07,             /*   Usage Page (Keyboard/Keypad)                  */
    0x19, 0x00,             /*   Usage Minimum (0)                             */
    0x29, 0xFF,             /*   Usage Maximum (255)                           */
    0x15, 0x00,             /*   Logical Minimum (0)                           */
    0x26, 0xFF, 0x00,       /*   Logical Maximum (255)                         */
    0x75, 0x08,             /*   Report Size (8)                               */
    0x95, 0x06,             /*   Report Count (6)                              */
    0x81, 0x00,             /*   Input (Data, Array, Abs)                      */
    0xC0,                   /* End Collection                                  */

    /* ---- Mouse: report ID 2 ---------------------------------------------- */
    0x05, 0x01,             /* Usage Page (Generic Desktop)                    */
    0x09, 0x02,             /* Usage (Mouse)                                   */
    0xA1, 0x01,             /* Collection (Application)                        */
    0x85, KEYBOARD_REPORT_ID_MOUSE, /* Report ID (2)                           */
    0x09, 0x01,             /*   Usage (Pointer)                               */
    0xA1, 0x00,             /*   Collection (Physical)                         */
    /* byte 0: buttons 1..3 + 5 bits padding */
    0x05, 0x09,             /*     Usage Page (Button)                         */
    0x19, 0x01,             /*     Usage Minimum (1)                           */
    0x29, 0x03,             /*     Usage Maximum (3)                           */
    0x15, 0x00,             /*     Logical Minimum (0)                         */
    0x25, 0x01,             /*     Logical Maximum (1)                         */
    0x75, 0x01,             /*     Report Size (1)                             */
    0x95, 0x03,             /*     Report Count (3)                            */
    0x81, 0x02,             /*     Input (Data, Var, Abs)                      */
    0x75, 0x05,             /*     Report Size (5)                             */
    0x95, 0x01,             /*     Report Count (1)                            */
    0x81, 0x01,             /*     Input (Const)                               */
    /* bytes 1..3: X, Y, wheel, signed relative */
    0x05, 0x01,             /*     Usage Page (Generic Desktop)                */
    0x09, 0x30,             /*     Usage (X)                                   */
    0x09, 0x31,             /*     Usage (Y)                                   */
    0x09, 0x38,             /*     Usage (Wheel)                               */
    0x15, 0x81,             /*     Logical Minimum (-127)                      */
    0x25, 0x7F,             /*     Logical Maximum (127)                       */
    0x75, 0x08,             /*     Report Size (8)                             */
    0x95, 0x03,             /*     Report Count (3)                            */
    0x81, 0x06,             /*     Input (Data, Var, Rel)                      */
    0xC0,                   /*   End Collection                                */
    0xC0,                   /* End Collection                                  */
};

/* ------------------------------------------------------------------------ */
/* State                                                                    */
/* ------------------------------------------------------------------------ */

typedef enum {
    DEMO_IDLE = 0, /* waiting for the next cycle */
    DEMO_TYPING,
    DEMO_MOUSE,
} demo_phase_t;

/* Demo square: 4 sides of DEMO_SIDE_STEPS steps of DEMO_STEP_PX pixels. */
#define DEMO_SIDE_STEPS 20u
#define DEMO_STEP_PX    3
/* First demo cycle starts this long after the host connects. */
#define DEMO_START_DELAY_US 3000000u

static struct {
    demo_phase_t phase;
    bool armed;          /* connected and the first cycle is scheduled */
    bool key_down;       /* typing: a key is currently pressed */
    uint16_t pos;        /* typing: text index; mouse: step */
    uint32_t next_us;    /* next state change */
    uint32_t cycle_us;   /* start of the next cycle */
} demo;

static bool app_demo = KEYBOARD_DEMO;
static uint8_t app_leds;
static floss_hid_config_t app_cfg;

/* ------------------------------------------------------------------------ */
/* Publishing (both protocol formats)                                       */
/* ------------------------------------------------------------------------ */

void keyboard_app_set_keys(uint8_t modifiers, const uint8_t keys[6]) {
    uint8_t r[KEYBOARD_KEYS_LEN] = {modifiers, 0};
    if (keys) {
        memcpy(&r[2], keys, 6);
    }
    floss_hid_set_input(KEYBOARD_REPORT_ID_KEYS, r, sizeof(r));
    floss_hid_set_boot_input(FLOSS_HID_BOOT_KEYBOARD, r, sizeof(r));
}

void keyboard_app_mouse(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel) {
    /* -128 is outside the declared logical range. */
    uint8_t r[KEYBOARD_MOUSE_LEN] = {
        (uint8_t)(buttons & 0x07),
        (uint8_t)(dx == -128 ? -127 : dx),
        (uint8_t)(dy == -128 ? -127 : dy),
        (uint8_t)(wheel == -128 ? -127 : wheel),
    };
    floss_hid_set_input(KEYBOARD_REPORT_ID_MOUSE, r, sizeof(r));
    /* Boot mouse = buttons, X, Y; the wheel byte rides along (hosts in boot
     * protocol ignore what they do not know). */
    floss_hid_set_boot_input(FLOSS_HID_BOOT_MOUSE, r, sizeof(r));
}

/* ------------------------------------------------------------------------ */
/* Demo                                                                     */
/* ------------------------------------------------------------------------ */

/** ASCII -> keyboard usage + modifiers for the characters the demo uses. */
static bool ascii_to_key(char c, uint8_t *usage, uint8_t *mods) {
    *mods = 0;
    if (c >= 'a' && c <= 'z') {
        *usage = (uint8_t)(0x04 + (c - 'a'));
    } else if (c >= 'A' && c <= 'Z') {
        *usage = (uint8_t)(0x04 + (c - 'A'));
        *mods = FLOSS_HID_MOD_LSHIFT;
    } else if (c >= '1' && c <= '9') {
        *usage = (uint8_t)(0x1E + (c - '1'));
    } else if (c == '0') {
        *usage = 0x27;
    } else if (c == ' ') {
        *usage = 0x2C;
    } else if (c == '-') {
        *usage = 0x2D;
    } else if (c == ',') {
        *usage = 0x36;
    } else if (c == '.') {
        *usage = 0x37;
    } else if (c == '!') {
        *usage = 0x1E; /* shift + 1 (US layout) */
        *mods = FLOSS_HID_MOD_LSHIFT;
    } else {
        return false;
    }
    return true;
}

static void demo_type_step(uint32_t now) {
    static const char text[] = KEYBOARD_DEMO_TEXT;
    if (demo.key_down) {
        keyboard_app_set_keys(0, NULL); /* release: also separates repeated letters */
        demo.key_down = false;
        demo.pos++;
        demo.next_us = now + KEYBOARD_KEY_HOLD_US;
        return;
    }
    while (demo.pos < sizeof(text) - 1u) {
        uint8_t usage, mods;
        if (ascii_to_key(text[demo.pos], &usage, &mods)) {
            const uint8_t keys[6] = {usage, 0, 0, 0, 0, 0};
            keyboard_app_set_keys(mods, keys);
            demo.key_down = true;
            demo.next_us = now + KEYBOARD_KEY_HOLD_US;
            return;
        }
        demo.pos++; /* not in the table: skip */
    }
    demo.phase = DEMO_MOUSE;
    demo.pos = 0;
    demo.next_us = now;
}

static void demo_mouse_step(uint32_t now) {
    if (demo.pos >= 4u * DEMO_SIDE_STEPS) {
        keyboard_app_mouse(0, 0, 0, 0); /* neutral state for reconnects / protocol switches */
        demo.phase = DEMO_IDLE;
        return;
    }
    static const int8_t dir[4][2] = {
        {DEMO_STEP_PX, 0}, {0, DEMO_STEP_PX}, {-DEMO_STEP_PX, 0}, {0, -DEMO_STEP_PX},
    };
    const int8_t *d = dir[demo.pos / DEMO_SIDE_STEPS];
    keyboard_app_mouse(0, d[0], d[1], 0);
    demo.pos++;
    demo.next_us = now + KEYBOARD_MOUSE_STEP_US;
}

void keyboard_app_task(void) {
    if (!app_demo) {
        return;
    }
    if (!floss_hid_connected()) {
        demo.armed = false;
        return;
    }
    uint32_t now = floss_hal_time_us();
    if (!demo.armed) {
        demo.armed = true;
        demo.phase = DEMO_IDLE;
        demo.cycle_us = now + DEMO_START_DELAY_US;
    }
    switch (demo.phase) {
    case DEMO_IDLE:
        if (floss_time_diff(now, demo.cycle_us) >= 0) {
            demo.cycle_us += KEYBOARD_DEMO_PERIOD_US;
            if (floss_time_diff(now, demo.cycle_us) >= 0) {
                demo.cycle_us = now + KEYBOARD_DEMO_PERIOD_US; /* fell behind: resync */
            }
            demo.phase = DEMO_TYPING;
            demo.pos = 0;
            demo.key_down = false;
            demo.next_us = now;
        }
        break;
    case DEMO_TYPING:
        if (floss_time_diff(now, demo.next_us) >= 0) {
            demo_type_step(now);
        }
        break;
    case DEMO_MOUSE:
        if (floss_time_diff(now, demo.next_us) >= 0) {
            demo_mouse_step(now);
        }
        break;
    }
}

void keyboard_app_set_demo(bool enabled) {
    if (!enabled && app_demo) {
        /* Never leave a demo key held down or a movement in the slot. */
        keyboard_app_set_keys(0, NULL);
        keyboard_app_mouse(0, 0, 0, 0);
    }
    app_demo = enabled;
    demo.armed = false;
}

uint8_t keyboard_app_leds(void) {
    return app_leds;
}

/* ------------------------------------------------------------------------ */
/* HID callbacks                                                            */
/* ------------------------------------------------------------------------ */

static void on_connected(const floss_addr_t *host) {
    char s[18];
    floss_addr_to_str(host, s);
    FLOSS_LOGI(APP, "keyboard connected to %s", s);
}

static void on_disconnected(uint8_t reason) {
    FLOSS_LOGI(APP, "keyboard disconnected (%s)", floss_hci_status_str(reason));
    /* Release everything so the next host does not see a stuck key. */
    if (demo.phase != DEMO_IDLE) {
        keyboard_app_set_keys(0, NULL);
        keyboard_app_mouse(0, 0, 0, 0);
        demo.phase = DEMO_IDLE;
    }
    demo.armed = false;
}

/* LED output report: report ID 1 in report protocol and in boot protocol. */
static void on_output_report(uint8_t report_id, const uint8_t *data, uint16_t len) {
    if (report_id != KEYBOARD_REPORT_ID_KEYS || len < 1) {
        FLOSS_LOGD(APP, "ignored output report id %u len %u", report_id, len);
        return;
    }
    if (data[0] != app_leds) {
        app_leds = data[0];
        FLOSS_LOGI(APP, "LEDs: num %s, caps %s, scroll %s", (app_leds & FLOSS_HID_LED_NUM_LOCK) ? "on" : "off",
                   (app_leds & FLOSS_HID_LED_CAPS_LOCK) ? "on" : "off",
                   (app_leds & FLOSS_HID_LED_SCROLL_LOCK) ? "on" : "off");
    }
}

static void on_set_protocol(uint8_t protocol) {
    /* Nothing to do: both formats are always published. */
    FLOSS_LOGI(APP, "host selected %s protocol", protocol == FLOSS_HID_PROTOCOL_BOOT ? "boot" : "report");
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
    .on_get_report = NULL, /* input GET_REPORT answered from the mailboxes */
    .on_set_report = NULL, /* output SET_REPORT -> on_output_report */
    .on_set_protocol = on_set_protocol,
    .on_control = on_control,
    .on_report_tick = NULL,
};

floss_err_t keyboard_app_init(void) {
    floss_hid_config_defaults(&app_cfg);
    app_cfg.name = "FLOSS Keyboard";
    app_cfg.report_descriptor = keyboard_report_descriptor;
    app_cfg.report_descriptor_len = sizeof(keyboard_report_descriptor);
    app_cfg.subclass = FLOSS_HID_SUBCLASS_COMBO; /* keyboard + pointing device */
    app_cfg.boot_device = true;

    /* Device ID record. 0x1209 is the pid.codes open-source vendor ID;
     * product 0x0F11 is a PLACEHOLDER - a real product must register its
     * own product ID at https://pid.codes before shipping. */
    app_cfg.vendor_id_source = 2; /* USB-IF assigned vendor ID */
    app_cfg.vendor_id = 0x1209;
    app_cfg.product_id = 0x0F11;
    app_cfg.version = 0x0100;

    app_cfg.report_interval_us = 0; /* send on change */

    memset(&demo, 0, sizeof(demo));
    app_leds = 0;

    floss_err_t err = floss_hid_init(&app_cfg, &app_callbacks);
    if (err != FLOSS_OK) {
        FLOSS_LOGE(APP, "floss_hid_init failed: %s", floss_err_str(err));
        return err;
    }
    /* Neutral reports right away so GET_REPORT has an answer in either protocol. */
    keyboard_app_set_keys(0, NULL);
    keyboard_app_mouse(0, 0, 0, 0);
    return FLOSS_OK;
}
