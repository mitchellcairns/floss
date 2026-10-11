/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * tusb_config.h - TinyUSB configuration for the link dongle (device only):
 * one CDC interface (console) + one HID interface (the gamepad).
 *
 * Found by the SDK's tinyusb_device library through the dongle target's
 * include path; the SDK's own pico_stdio_usb (and its tusb_config.h) is not
 * linked into the dongle.
 */
#ifndef FLOSS_LINK_TUSB_CONFIG_H
#define FLOSS_LINK_TUSB_CONFIG_H

/* Root hub port 0 in device mode, full speed (the RP2040/RP2350 controller). */
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE)

/* CFG_TUSB_MCU and CFG_TUSB_OS come from the SDK's TinyUSB integration. */

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0 /* TinyUSB must never print: printf runs over this very CDC */
#endif

#define CFG_TUD_ENDPOINT0_SIZE 64

/* ---- classes ------------------------------------------------------------- */
#define CFG_TUD_CDC    1
#define CFG_TUD_HID    1
#define CFG_TUD_MSC    0
#define CFG_TUD_MIDI   0
#define CFG_TUD_VENDOR 0

/* ---- CDC console --------------------------------------------------------- */
/* RX: console keys only; must be >= the endpoint buffer. */
#define CFG_TUD_CDC_RX_BUFSIZE 256
/* TX: large enough that a stats dump fits without waiting for the host. */
#define CFG_TUD_CDC_TX_BUFSIZE 4096
/* One full-speed bulk packet per transfer. */
#define CFG_TUD_CDC_EP_BUFSIZE 64

/* ---- HID gamepad --------------------------------------------------------- */
/* Largest report incl. ID: input 1 + 13 bytes, output 1 + 3 bytes. */
#define CFG_TUD_HID_EP_BUFSIZE 16

#endif /* FLOSS_LINK_TUSB_CONFIG_H */
