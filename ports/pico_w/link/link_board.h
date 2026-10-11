/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * link_board.h - Pico W board helpers shared by the pad <-> dongle link
 * firmwares (main_pad.c, main_dongle.c).
 *
 * What both ends have in common: the radio bring-up (exactly as the gamepad
 * firmware's main.c does it: HAL, CYW43 driver, FLOSS with
 * the CYW43 transport, public address from the board's WLAN MAC), the
 * link transport choice kept in flash, the LED that follows the link state,
 * the BOOTSEL button and the debug console keys that are not about the link.
 *
 * USB pump: the dongle runs its own TinyUSB device, which only makes
 * progress while tud_task() runs. Every helper here that waits (fatal
 * error loop, reboot) calls the pump given to link_board_init() so the
 * console keeps working; the pad passes NULL (the SDK's USB stdio runs in
 * the background).
 */
#ifndef LINK_BOARD_H
#define LINK_BOARD_H

#include "floss/floss.h"
#include "floss/floss_link.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Flash store key holding the selected transport (1 byte: floss_link_transport_t). */
#define LINK_STORE_KEY_TRANSPORT 0x8200u

/** Main-loop housekeeping callback (dongle: tud_task). */
typedef void (*link_board_pump_fn)(void);

/**
 * Bring up the HAL, the CYW43 driver and FLOSS (CYW43 transport + chipset
 * quirks, INFO log level, address override from the WLAN MAC, snoop hook if
 * FLOSS_PICO_SNOOP). Halts with a FATAL message on error. Call once; then
 * floss_link_init() and floss_start().
 */
void link_board_init(link_board_pump_fn pump);

/** CYW43 driver housekeeping: a no-op with the bare glue (driver work runs in floss_poll); async_context_poll otherwise. */
void link_board_poll_driver(void);

/** This board's radio address (valid after link_board_init). */
const floss_addr_t *link_board_addr(void);

/** Print "<label><address>". */
void link_board_print_addr(const char *label, const floss_addr_t *a);

/** Print a FATAL message every 2 s forever (keeps pumping USB). */
void link_board_fatal(const char *what, int err) __attribute__((noreturn));

/** Sample the BOOTSEL button (runs from RAM, ~10 us with interrupts off). */
bool link_board_bootsel_pressed(void);

/**
 * LED (on the radio chip, so only once the controller is up):
 *   fast blink  pairing
 *   slow blink  reconnecting to the bonded peer
 *   solid       connected
 *   short blip every 2 s  idle (no bond, not pairing)
 */
void link_board_led_task(uint32_t now);

/** Transport stored in flash (FLOSS_LINK_CLASSIC when nothing valid is stored). */
floss_link_transport_t link_board_transport_load(void);

/** "Classic" / "LE". */
const char *link_board_transport_name(floss_link_transport_t t);

/** "idle" / "pairing" / "connecting" / "connected". */
const char *link_board_state_name(floss_link_state_t s);

/**
 * Console 'c' / 'e': store transport `want` and reboot into it. Returns
 * (after printing a note) only when `want` is already the active one or the
 * flash write failed.
 */
void link_board_switch_transport(floss_link_transport_t active, floss_link_transport_t want);

/** Reboot through the watchdog after giving the console ~50 ms to drain. */
void link_board_reboot(void) __attribute__((noreturn));

/**
 * Debug keys common to both firmwares: 't' trace dump, 'v' / 'q' log level
 * DEBUG / INFO, 'p' toggle the HCI snoop log. Returns true if c was one.
 */
bool link_board_debug_key(int c);

/** Is the HCI snoop log on? */
bool link_board_snoop_on(void);

#ifdef __cplusplus
}
#endif

#endif /* LINK_BOARD_H */
