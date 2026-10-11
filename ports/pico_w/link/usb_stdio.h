/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * usb_stdio.h - The link dongle's USB device: TinyUSB bring-up and task,
 * and stdio (printf / getchar) over its own CDC interface.
 *
 * The SDK's pico_stdio_usb cannot be used here: it brings its own device
 * descriptors and runs tud_task() from a background interrupt, while the
 * dongle needs its own composite descriptors (usb_descriptors.c) and runs
 * TinyUSB from the main loop, next to floss_poll().
 *
 * Output never blocks for long: with no terminal attached (DTR low) it is
 * dropped; with one attached it goes into a 4 KB FIFO, and only when that
 * is full does printf keep USB running until the host takes data, giving
 * up after USB_STDIO_STALL_US without progress (host not reading).
 *
 * Opening the port at 1200 baud reboots into the ROM bootloader (BOOTSEL
 * drive), like the SDK's USB stdio - tools/pico_flash.ps1 relies on it.
 */
#ifndef USB_STDIO_H
#define USB_STDIO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Max time printf waits for the host to make room before dropping output (us). */
#ifndef USB_STDIO_STALL_US
#define USB_STDIO_STALL_US 2000u
#endif

/** Start the USB device (TinyUSB) and register the CDC stdio driver. */
void usb_stdio_init(void);

/** Run TinyUSB (tud_task). Call from the main loop; never blocks. */
void usb_stdio_task(void);

/** Is a terminal attached (CDC DTR set)? */
bool usb_stdio_connected(void);

/** Console bytes dropped because nobody was reading. */
uint32_t usb_stdio_dropped(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_STDIO_H */
