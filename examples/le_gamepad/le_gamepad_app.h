/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * le_gamepad_app.h - Portable LE (HID over GATT) gamepad demo.
 *
 * Platform-independent: a port's main() calls floss_init(), then
 * le_gamepad_app_init(), then floss_start(), and in its loop floss_poll()
 * followed by le_gamepad_app_task().
 *
 * Input report (ID 1, 13 bytes): 16 buttons (LE16), hat (low nibble,
 * 0..7 clockwise from north, 8 = centred), X/Y/Z/Rz as LE int16, two
 * 8-bit triggers (Rx, Ry).
 * Output report (ID 2, 3 bytes): rumble left, rumble right, LED mask.
 */
#ifndef LE_GAMEPAD_APP_H
#define LE_GAMEPAD_APP_H

#include "floss/floss.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Input/output report IDs and lengths (payload without the ID byte). */
#define LE_GAMEPAD_INPUT_ID   1
#define LE_GAMEPAD_INPUT_LEN  13
#define LE_GAMEPAD_OUTPUT_ID  2
#define LE_GAMEPAD_OUTPUT_LEN 3

/** Configure advertising and the HID-over-GATT services. After floss_init(), before floss_start(). */
floss_err_t le_gamepad_app_init(void);

/** Call from the main loop: advances the demo pattern and publishes input reports. */
void le_gamepad_app_task(void);

#ifdef __cplusplus
}
#endif

#endif /* LE_GAMEPAD_APP_H */
