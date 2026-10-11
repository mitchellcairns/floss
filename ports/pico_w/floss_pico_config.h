/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_pico_config.h - FLOSS configuration for the Pico W gamepad.
 *
 * Included at the top of floss_config.h (FLOSS_USER_CONFIG_FILE), so only
 * override what differs from the defaults there. RAM is plentiful on the
 * RP2040 (264 KB) but the radio link is the bottleneck: buffers beyond what
 * the controller can have in flight only add latency.
 */
#ifndef FLOSS_PICO_CONFIG_H
#define FLOSS_PICO_CONFIG_H

/* One host link plus a spare for a second host trying to connect. */
#define FLOSS_MAX_CONNECTIONS 2

/* Packet pool: commands, ACL TX for both HID channels + SDP, reassembly. */
#define FLOSS_BUF_COUNT 16

/* The CYW43 shared-bus transport writes a 4-byte header in front of each packet. */
#define FLOSS_TRANSPORT_MAX_HEADROOM 4

/* The gamepad milestone is Classic HID only. Flip to 1 to build LE / RFCOMM in. */
#define FLOSS_ENABLE_LE 1
#define FLOSS_ENABLE_RFCOMM 1

/* Compile DEBUG messages in (runtime level starts at INFO; press 'v' on the
 * serial console to see them). Set to FLOSS_LOG_INFO to save flash. */
#define FLOSS_LOG_LEVEL FLOSS_LOG_DEBUG

#define FLOSS_TRACE_ENTRIES 256
#define FLOSS_ENABLE_STATS 1
#define FLOSS_ENABLE_SNOOP 1

/* Debug pins: logical 0..3 -> GPIO 2..5 (see floss_hal_pico.c). */
#define FLOSS_ENABLE_DEBUG_PINS 0

#endif /* FLOSS_PICO_CONFIG_H */
