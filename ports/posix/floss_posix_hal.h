/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_posix_hal.h - Extras of the desktop HAL (Windows + POSIX) that are
 * not part of the portable floss_hal.h contract.
 *
 * The desktop port runs the stack in a normal process: time comes from the
 * OS monotonic clock, persistent storage is a small file, logs go to stderr.
 * These helpers let a desktop program choose where that file lives, sleep
 * with sub-millisecond resolution between floss_poll() calls, and shift the
 * clock to exercise the 32-bit microsecond wrap without waiting 71 minutes.
 */
#ifndef FLOSS_POSIX_HAL_H
#define FLOSS_POSIX_HAL_H

#include "floss/floss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Path of the key/value store file (default "floss_store.bin" in the current
 * directory). Call before the stack first touches storage (i.e. before
 * floss_start()); the string is copied. The file is created on first write.
 */
void floss_posix_set_store_path(const char *path);

/**
 * Sleep for about `us` microseconds. Uses a high-resolution waitable timer on
 * Windows (falls back to Sleep(), ~1 ms granularity) and nanosleep() on
 * POSIX. 0 just yields the CPU.
 */
void floss_posix_sleep_us(uint32_t us);

/**
 * Add a fixed offset to floss_hal_time_us(). Setting it to e.g.
 * 0xFFFFFFFF - 5000000 makes the clock wrap five seconds after start, which
 * is how to test wrap handling in timers and tools.
 */
void floss_posix_set_time_offset(uint32_t offset_us);

/** Mirror log output into this file as well as stderr (NULL stops). Opened in append mode. */
floss_err_t floss_posix_set_log_file(const char *path);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_POSIX_HAL_H */
