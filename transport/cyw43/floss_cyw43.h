/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_cyw43.h - HCI transport for Infineon CYW43439 (Raspberry Pi Pico W,
 * Pico 2 W) over the shared WLAN/radio SPI bus.
 *
 * Wire format on the shared bus: [len0 len1 len2 type][HCI packet], written
 * through the Pico SDK's cyw43 driver (cyw43_bluetooth_hci_read/write), which
 * also downloads the radio firmware. Headroom = 4, so TX is zero-copy.
 *
 * NOTE: this file links against the Pico SDK's pico_cyw43_driver and
 * cybt_shared_bus libraries (BSD-3-Clause) and the cyw43-driver (licensed by
 * Raspberry Pi for use with RP-series chips). FLOSS itself stays Unlicense;
 * see docs/LEGAL.md.
 */
#ifndef FLOSS_CYW43_H
#define FLOSS_CYW43_H

#include "floss/floss_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Bare-metal driver glue (transport/cyw43/floss_cyw43_bare.c, define
 * FLOSS_CYW43_BARE=1): initialise the cyw43 driver with no async_context,
 * no RTOS and no interrupts. Driver work then runs inside the transport's
 * poll, i.e. inside floss_poll(). Call once before floss_init().
 */
bool floss_cyw43_driver_init(void);
/** Bare glue: run pending driver work now (called by the transport's poll). */
void floss_cyw43_driver_service(void);

/**
 * The transport singleton. Either floss_cyw43_driver_init() (bare) or the
 * SDK's cyw43_driver_init(async_context) must have been called.
 */
floss_transport_t *floss_cyw43_transport(void);

/** Chipset quirks: vendor set-address opcode 0xFC01. */
extern const floss_chipset_t floss_chipset_cyw43;

/**
 * Fill addr with the recommended public address: WLAN MAC + 1 in the last
 * byte (what the OTP would give). Use with floss_hci_set_addr_override().
 */
void floss_cyw43_default_addr(floss_addr_t *addr);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_CYW43_H */
