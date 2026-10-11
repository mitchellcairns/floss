/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * gamepad_app.h - Portable Classic HID gamepad example (no platform code).
 *
 * A port wires this up in three lines:
 *
 *     floss_init(...);
 *     gamepad_app_init();          // HID config, descriptor, SDP, callbacks
 *     floss_start();
 *     for (;;) { floss_poll(); read_my_inputs(); gamepad_app_task(); }
 *
 * Inputs: by default a demo pattern animates the controller (left stick
 * circles every 2 s, buttons cycle) so a bare board shows something on the
 * host. A port with real hardware calls gamepad_app_set_demo(false) and
 * feeds its inputs with the setters below; buttons given with
 * gamepad_app_set_buttons() are OR-ed into the demo while it runs, so a
 * single real button (e.g. BOOTSEL) still works on top of the demo.
 *
 * The setters only update local state; gamepad_app_task() builds the HID
 * report and publishes it with floss_hid_set_input() only when it changed.
 * Call the setters and the task from the same context as floss_poll().
 *
 * Report layout, descriptor, demo pattern and the GAMEPAD_* report/hat
 * constants: gamepad_state.h (shared with the link pad and dongle).
 */
#ifndef GAMEPAD_APP_H
#define GAMEPAD_APP_H

#include "floss/floss.h"
#include "gamepad_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Input report pacing (us). 1000 = 1 kHz; 0 = send on change as fast as the link allows. */
#ifndef GAMEPAD_REPORT_INTERVAL_US
#define GAMEPAD_REPORT_INTERVAL_US 1000u
#endif

/**
 * Link flush timeout (ms). 0 keeps delivery reliable (default). A small value
 * (e.g. 10-20) lets the controller drop reports that missed their slot so a
 * fresh one is never stuck behind a stale one on a noisy link.
 */
#ifndef GAMEPAD_FLUSH_TIMEOUT_MS
#define GAMEPAD_FLUSH_TIMEOUT_MS 0
#endif

/** Max input reports queued in the controller (see floss_hid_config_t.max_inflight). */
#ifndef GAMEPAD_MAX_INFLIGHT
#define GAMEPAD_MAX_INFLIGHT 2
#endif

/** Poll latency requested from the host after connecting, us (0 = don't ask). */
#ifndef GAMEPAD_QOS_LATENCY_US
#define GAMEPAD_QOS_LATENCY_US 1250
#endif

/** Start with the demo animation enabled. */
#ifndef GAMEPAD_DEMO
#define GAMEPAD_DEMO 1
#endif

/** Configure HID (descriptor, SDP, callbacks). Call after floss_init(), before floss_start(). */
floss_err_t gamepad_app_init(void);

/** Run once per main-loop iteration: demo animation + publish the report if it changed. */
void gamepad_app_task(void);

/** Enable/disable the demo animation (real inputs: disable). */
void gamepad_app_set_demo(bool enabled);

/** Buttons 1..16 as a bitmask (bit 0 = button 1). */
void gamepad_app_set_buttons(uint32_t buttons);
/** Hat switch (GAMEPAD_HAT_*). */
void gamepad_app_set_hat(uint8_t hat);
/** Sticks, -32767..32767 (Y axes: positive = down). */
void gamepad_app_set_sticks(int16_t lx, int16_t ly, int16_t rx, int16_t ry);
/** Analog triggers, 0..255. */
void gamepad_app_set_triggers(uint8_t left, uint8_t right);

/** Latest rumble / LED state sent by the host. */
const gamepad_output_t *gamepad_app_output(void);

#ifdef __cplusplus
}
#endif

#endif /* GAMEPAD_APP_H */
