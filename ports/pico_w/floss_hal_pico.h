/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_hal_pico.h - Pico W HAL extras (setup the generic HAL API lacks).
 *
 * The HAL contract itself (floss_hal.h) has no init function; this port
 * needs one to claim its spin lock, load the flash key/value store into RAM
 * and configure debug GPIOs. Call it once, before floss_init().
 */
#ifndef FLOSS_HAL_PICO_H
#define FLOSS_HAL_PICO_H

#include "floss/floss.h"

#ifdef __cplusplus
extern "C" {
#endif

/** First GPIO used for debug pins (logical 0..3 -> GPIO base..base+3). */
#ifndef FLOSS_PICO_DBG_PIN_BASE
#define FLOSS_PICO_DBG_PIN_BASE 2
#endif

/** One-time HAL setup. Call before floss_init(). */
void floss_pico_hal_init(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_HAL_PICO_H */
