/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * keyboard_app.h - Portable Classic HID keyboard + mouse combo example
 * (no platform code).
 *
 * A port wires this up like the gamepad example:
 *
 *     floss_init(...);
 *     keyboard_app_init();         // HID config, descriptor, SDP, callbacks
 *     floss_start();
 *     for (;;) { floss_poll(); keyboard_app_task(); }
 *
 * Demo (KEYBOARD_DEMO, on by default): while a host is connected, every
 * KEYBOARD_DEMO_PERIOD_US the device types KEYBOARD_DEMO_TEXT (no Enter)
 * and then moves the mouse pointer once around a small square. A port with
 * real keys calls keyboard_app_set_demo(false) and feeds
 * keyboard_app_set_keys() / keyboard_app_mouse().
 *
 * Reports (report protocol). Both are laid out exactly like the boot
 * protocol reports, so the same bytes serve both protocols:
 *   ID 1 input  (8 bytes): modifiers, reserved, 6 key usage codes
 *   ID 1 output (1 byte):  LEDs (num, caps, scroll, compose, kana)
 *   ID 2 input  (4 bytes): buttons (3), X, Y, wheel (signed, relative)
 *
 * The device declares itself a boot device: a PC's firmware set-up screen
 * can use it in boot protocol. Every change is published in both formats
 * (floss_hid_set_input + floss_hid_set_boot_input); the stack sends the one
 * the host selected.
 *
 * Note on the "latest wins" mailbox: a state that is replaced before the
 * link carried it is never sent. Keyboards therefore hold every key state
 * for KEYBOARD_KEY_HOLD_US, and relative mouse motion is published at most
 * every KEYBOARD_MOUSE_STEP_US, followed by a zero-motion report (so a
 * reconnect or protocol switch, which re-sends the current state, does not
 * replay a movement).
 */
#ifndef KEYBOARD_APP_H
#define KEYBOARD_APP_H

#include "floss/floss.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Start with the demo enabled. */
#ifndef KEYBOARD_DEMO
#define KEYBOARD_DEMO 1
#endif

/** Text the demo types (letters, digits, space and . , - ! only; no Enter). */
#ifndef KEYBOARD_DEMO_TEXT
#define KEYBOARD_DEMO_TEXT "Hello from FLOSS! "
#endif

/** One demo cycle (type the text, then draw a square with the mouse). */
#ifndef KEYBOARD_DEMO_PERIOD_US
#define KEYBOARD_DEMO_PERIOD_US 10000000u
#endif

/** How long each key press / release state stays published. */
#ifndef KEYBOARD_KEY_HOLD_US
#define KEYBOARD_KEY_HOLD_US 30000u
#endif

/** Pace of relative mouse reports in the demo. */
#ifndef KEYBOARD_MOUSE_STEP_US
#define KEYBOARD_MOUSE_STEP_US 10000u
#endif

#define KEYBOARD_REPORT_ID_KEYS  1 /**< keyboard input + LED output */
#define KEYBOARD_REPORT_ID_MOUSE 2
#define KEYBOARD_KEYS_LEN        8
#define KEYBOARD_MOUSE_LEN       4

/** Configure HID (descriptor, SDP, callbacks). Call after floss_init(), before floss_start(). */
floss_err_t keyboard_app_init(void);

/** Run once per main-loop iteration: advances the demo. */
void keyboard_app_task(void);

/** Enable/disable the demo (real inputs: disable). */
void keyboard_app_set_demo(bool enabled);

/**
 * Publish the keyboard state: modifier bits (FLOSS_HID_MOD_*) and up to six
 * pressed keys (HID keyboard usage codes, 0 = none).
 */
void keyboard_app_set_keys(uint8_t modifiers, const uint8_t keys[6]);

/** Publish one relative mouse report (FLOSS_HID_MOUSE_* buttons). */
void keyboard_app_mouse(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel);

/** Latest LED state from the host (FLOSS_HID_LED_* bits). */
uint8_t keyboard_app_leds(void);

#ifdef __cplusplus
}
#endif

#endif /* KEYBOARD_APP_H */
