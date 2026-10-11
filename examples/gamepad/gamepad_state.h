/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * gamepad_state.h - Gamepad controls, demo animation and report format,
 * shared by every firmware that speaks this gamepad's reports:
 *
 *   examples/gamepad      Classic HID gamepad (reports go to floss_hid)
 *   ports/pico_w/link     link pad (reports go to floss_link) and link
 *                         dongle (forwards the same reports over USB HID)
 *
 * Because all of them use the one descriptor and the one byte layout below,
 * the dongle can pass the pad's reports to the USB host unchanged.
 *
 * Report layout (input, report ID 1, 13 bytes after the ID):
 *   off 0  uint16  buttons 1..16 (bit 0 = button 1)
 *   off 2  uint8   hat switch in the low nibble: 0 = N, 1 = NE ... 7 = NW,
 *                  8 = centred (null state); high nibble padding
 *   off 3  int16   X  (left stick X,  -32767..32767)
 *   off 5  int16   Y  (left stick Y,  -32767..32767, + = down)
 *   off 7  int16   Z  (right stick X)
 *   off 9  int16   Rz (right stick Y)
 *   off 11 uint8   Rx (left trigger,  0..255)
 *   off 12 uint8   Ry (right trigger, 0..255)
 *
 * Output (report ID 2, 3 bytes, vendor page): rumble strong, rumble weak,
 * player LED / indicator byte.
 *
 * Demo: the left stick circles every 2 s, the triggers ramp, buttons cycle
 * (GAMEPAD_DEMO_BUTTONS) and the right stick X axis is a millisecond
 * counter, so a host can measure report interval and loss from the axis
 * value alone (tools/hostprobe).
 *
 * Plain integer code with no platform or stack dependencies beyond the
 * byte helpers in floss_types.h.
 */
#ifndef GAMEPAD_STATE_H
#define GAMEPAD_STATE_H

#include "floss/floss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Let the demo cycle through the buttons (set 0 if presses annoy the host). */
#ifndef GAMEPAD_DEMO_BUTTONS
#define GAMEPAD_DEMO_BUTTONS 1
#endif

#define GAMEPAD_REPORT_ID_INPUT  1
#define GAMEPAD_REPORT_ID_OUTPUT 2
#define GAMEPAD_INPUT_REPORT_LEN 13
#define GAMEPAD_OUTPUT_REPORT_LEN 3

/** Size of gamepad_report_descriptor[] (checked at compile time). */
#define GAMEPAD_REPORT_DESCRIPTOR_LEN 113

/** Hat switch values. */
#define GAMEPAD_HAT_UP         0
#define GAMEPAD_HAT_UP_RIGHT   1
#define GAMEPAD_HAT_RIGHT      2
#define GAMEPAD_HAT_DOWN_RIGHT 3
#define GAMEPAD_HAT_DOWN       4
#define GAMEPAD_HAT_DOWN_LEFT  5
#define GAMEPAD_HAT_LEFT       6
#define GAMEPAD_HAT_UP_LEFT    7
#define GAMEPAD_HAT_CENTERED   8

/** HID report descriptor: Generic Desktop Game Pad, input ID 1, output ID 2. */
extern const uint8_t gamepad_report_descriptor[];

/** Every control of the gamepad. */
typedef struct {
    uint32_t buttons;       /**< buttons 1..16 (bit 0 = button 1) */
    uint8_t hat;            /**< GAMEPAD_HAT_* */
    int16_t lx, ly, rx, ry; /**< sticks, -32767..32767 (Y: + = down) */
    uint8_t lt, rt;         /**< analog triggers, 0..255 */
} gamepad_state_t;

/** Latest output report from the host. */
typedef struct {
    uint8_t rumble_strong; /**< low-frequency motor, 0..255 */
    uint8_t rumble_weak;   /**< high-frequency motor, 0..255 */
    uint8_t led;           /**< player indicator / LED byte */
} gamepad_output_t;

/** Neutral state: nothing pressed, hat centred, sticks and triggers at rest. */
void gamepad_state_init(gamepad_state_t *s);

/**
 * Overlay the demo animation for time now_us (sticks, triggers, the
 * millisecond-counter axis; buttons are OR-ed in, so real buttons still work).
 */
void gamepad_state_demo(gamepad_state_t *s, uint32_t now_us);

/** Serialise s into the 13-byte input report (without the report ID). */
void gamepad_state_build_report(const gamepad_state_t *s, uint8_t out[GAMEPAD_INPUT_REPORT_LEN]);

/** Clamp an axis into the declared logical range (-32768 is outside it). */
int16_t gamepad_clamp_axis(int16_t v);

/**
 * Decode an output report. Returns false (out untouched) when the ID or
 * length does not match report 2.
 */
bool gamepad_output_parse(uint8_t report_id, const uint8_t *data, uint16_t len, gamepad_output_t *out);

#ifdef __cplusplus
}
#endif

#endif /* GAMEPAD_STATE_H */
