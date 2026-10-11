/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_hal.h - Hardware Abstraction Layer (HAL).
 *
 * ===========================================================================
 *  THIS IS THE ENTIRE PLATFORM CONTRACT (besides the HCI transport, which is
 *  described separately in floss_transport.h).
 *
 *  A port provides one C file implementing every function below. They are
 *  plain link-time functions (no function-pointer tables) so the compiler can
 *  inline/optimise across them with LTO. See ports/posix/ and ports/pico_w/
 *  for complete reference implementations, and docs/PORTING.md.
 * ===========================================================================
 *
 * Threading model: the stack is single-threaded. Every floss_* API call must
 * come from the same execution context that calls floss_poll(), EXCEPT the
 * functions explicitly marked "ISR/other-core safe" (e.g.
 * floss_hid_set_input()). The HAL functions here are only called from the
 * stack's context, except floss_hal_time_us() which must be callable anywhere.
 */
#ifndef FLOSS_HAL_H
#define FLOSS_HAL_H

#include "floss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Time (REQUIRED)                                                          */
/* ------------------------------------------------------------------------ */

/**
 * Free-running monotonic microsecond counter. May wrap at 2^32.
 * Must be cheap (it is called on every hot path) and callable from any
 * context. Resolution of 1 us is expected for sub-millisecond scheduling.
 */
uint32_t floss_hal_time_us(void);

/* ------------------------------------------------------------------------ */
/* Persistent storage (REQUIRED, may be RAM-only on a test port)            */
/* ------------------------------------------------------------------------ */

/**
 * Tiny key/value store used for bonding data (link keys) and the identity of
 * the last connected host. Keys are 16-bit, values are small (< 64 bytes).
 *
 * @return number of bytes read (<= len), or a negative floss_err_t
 *         (FLOSS_ERR_NOT_FOUND when the key does not exist).
 */
int floss_hal_store_read(uint16_t key, void *buf, uint16_t len);

/** Write (create or replace) a value. Return FLOSS_OK or negative error. */
floss_err_t floss_hal_store_write(uint16_t key, const void *buf, uint16_t len);

/** Delete a value. Deleting a missing key is not an error. */
floss_err_t floss_hal_store_delete(uint16_t key);

/* Key ranges used by the stack. Application keys should use >= 0x8000. */
#define FLOSS_STORE_KEY_BOND_BASE 0x0100 /* + index (0..FLOSS_MAX_BONDS-1) */
#define FLOSS_STORE_KEY_LAST_HOST 0x0001
#define FLOSS_STORE_KEY_APP_BASE  0x8000

/* ------------------------------------------------------------------------ */
/* Entropy (REQUIRED)                                                       */
/* ------------------------------------------------------------------------ */

/** Fill buf with random bytes. Need not be cryptographic for Classic HID. */
void floss_hal_random(uint8_t *buf, size_t len);

/* ------------------------------------------------------------------------ */
/* Debug output (REQUIRED, may discard)                                     */
/* ------------------------------------------------------------------------ */

/**
 * Write a chunk of log text (already formatted, includes trailing newline).
 * Must not block for long: a slow log sink directly hurts radio latency.
 * Drop data rather than stall if the sink is busy.
 */
void floss_hal_log_write(const char *text, size_t len);

/**
 * Drive a debug GPIO high/low. Used only when FLOSS_ENABLE_DEBUG_PINS=1.
 * Pin numbers are logical (see FLOSS_DBG_PIN_* in floss_debug.h); map them
 * to real GPIOs however you like, or make this a no-op.
 */
void floss_hal_debug_pin(uint8_t pin, bool level);

/* ------------------------------------------------------------------------ */
/* Critical sections (REQUIRED)                                             */
/* ------------------------------------------------------------------------ */

/**
 * Enter/leave a short critical section. Only used to protect the tiny amount
 * of state shared with ISR/other-core-safe APIs. On a single-core MCU this is
 * "disable interrupts"; on multi-core use a spinlock; on desktop a mutex.
 */
uint32_t floss_hal_critical_enter(void);
void floss_hal_critical_exit(uint32_t state);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_HAL_H */
