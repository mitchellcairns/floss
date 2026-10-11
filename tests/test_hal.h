/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * test_hal.h - Controls for the test HAL (tests/test_hal.c).
 *
 * The test HAL implements floss_hal.h for unit and integration tests:
 *  - time:     a fake microsecond clock that only moves when a test (or
 *              floss_sim_pump()) advances it, so every run is identical;
 *  - storage:  a RAM key/value store that survives floss_init() (to test
 *              bonding across "reboots") until floss_test_store_reset();
 *  - entropy:  a seeded deterministic PRNG;
 *  - logs:     printed to stdout (silence with FLOSS_TEST_QUIET=1) and kept
 *              in a capture ring tests can search;
 *  - critical sections: no-ops that track nesting balance.
 */
#ifndef FLOSS_TEST_HAL_H
#define FLOSS_TEST_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Reset everything below to its initial state (the runner does this before each test). */
void floss_test_hal_reset(void);

/* Fake clock (starts at 1 s after reset). */
void floss_test_time_set(uint32_t us);
void floss_test_time_advance(uint32_t us);
uint32_t floss_test_time(void);

/* RAM store. */
#define FLOSS_TEST_STORE_SLOTS 32
#define FLOSS_TEST_STORE_VAL_MAX 128
void floss_test_store_reset(void);
/** Number of keys stored. */
unsigned floss_test_store_count(void);
/** Total successful writes since reset (wear / write-amplification checks). */
unsigned floss_test_store_writes(void);

/* PRNG. */
void floss_test_random_seed(uint32_t seed);

/* Logs. */
/** Force quiet (true) / verbose (false). Default comes from FLOSS_TEST_QUIET. */
void floss_test_log_quiet(bool quiet);
/** True if text appeared in the log since the last reset/clear. */
bool floss_test_log_contains(const char *text);
void floss_test_log_clear(void);

/* Critical sections: current nesting depth (0 when balanced). */
int floss_test_critical_depth(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_TEST_HAL_H */
