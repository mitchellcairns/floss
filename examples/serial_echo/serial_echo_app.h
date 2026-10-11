/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * serial_echo_app.h - Portable serial port echo demo.
 *
 * Publishes a serial port service "FLOSS Serial" on RFCOMM channel 1 and
 * echoes every received byte back, uppercased. Pair with it from a desktop
 * or phone, open the virtual serial port it creates, and type.
 *
 * Platform glue (a port's main) does:
 *
 *     floss_init(&(floss_init_t){ .transport = t });
 *     serial_echo_app_init();
 *     floss_start();
 *     for (;;) floss_poll();
 */
#ifndef SERIAL_ECHO_APP_H
#define SERIAL_ECHO_APP_H

#include "floss/floss.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Register the service and make the device discoverable. Call after floss_init(). */
floss_err_t serial_echo_app_init(void);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_ECHO_APP_H */
