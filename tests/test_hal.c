/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * test_hal.c - Deterministic HAL for unit and integration tests.
 *
 * Every source of nondeterminism a port normally provides is replaced by
 * something a test controls: the clock moves only when told to, random bytes
 * come from a seeded xorshift, storage is a RAM table. That makes every
 * failure reproducible bit-for-bit, and lets tests cover hours of link time
 * (or a 32-bit clock wrap) in microseconds of CPU.
 *
 * The store deliberately outlives floss_init(): a test can run a session,
 * call floss_init() again and check that bonds come back - the RAM store
 * plays the role of flash across a reboot.
 */
#if defined(_WIN32) && !defined(_CRT_SECURE_NO_WARNINGS)
#define _CRT_SECURE_NO_WARNINGS /* getenv */
#endif

#include "test_hal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "floss/floss.h"

#define TEST_TIME_START 1000000u
#define TEST_LOG_CAPTURE 16384

static uint32_t g_time = TEST_TIME_START;

static struct {
    bool used;
    uint16_t key;
    uint16_t len;
    uint8_t val[FLOSS_TEST_STORE_VAL_MAX];
} g_store[FLOSS_TEST_STORE_SLOTS];
static unsigned g_store_writes;

static uint64_t g_rng = 0x9E3779B97F4A7C15ull;

static int g_quiet = -1; /* -1: not decided yet, read FLOSS_TEST_QUIET */
static char g_log[TEST_LOG_CAPTURE + 1];
static size_t g_log_len;

static int g_critical_depth;
static bool g_pins[8];

/* ------------------------------------------------------------------------ */
/* Controls                                                                 */
/* ------------------------------------------------------------------------ */

void floss_test_hal_reset(void) {
    g_time = TEST_TIME_START;
    floss_test_store_reset();
    floss_test_random_seed(0x12345678u);
    floss_test_log_clear();
    g_quiet = -1; /* back to the FLOSS_TEST_QUIET default */
    g_critical_depth = 0;
    memset(g_pins, 0, sizeof g_pins);
}

void floss_test_time_set(uint32_t us) {
    g_time = us;
}

void floss_test_time_advance(uint32_t us) {
    g_time += us;
}

uint32_t floss_test_time(void) {
    return g_time;
}

void floss_test_store_reset(void) {
    memset(g_store, 0, sizeof g_store);
    g_store_writes = 0;
}

unsigned floss_test_store_count(void) {
    unsigned n = 0;
    for (unsigned i = 0; i < FLOSS_TEST_STORE_SLOTS; i++) {
        n += g_store[i].used;
    }
    return n;
}

unsigned floss_test_store_writes(void) {
    return g_store_writes;
}

void floss_test_random_seed(uint32_t seed) {
    g_rng = 0x9E3779B97F4A7C15ull ^ seed;
    if (g_rng == 0) {
        g_rng = 1;
    }
}

void floss_test_log_quiet(bool quiet) {
    g_quiet = quiet;
}

bool floss_test_log_contains(const char *text) {
    return strstr(g_log, text) != NULL;
}

void floss_test_log_clear(void) {
    g_log_len = 0;
    g_log[0] = '\0';
}

int floss_test_critical_depth(void) {
    return g_critical_depth;
}

/* ------------------------------------------------------------------------ */
/* floss_hal.h implementation                                               */
/* ------------------------------------------------------------------------ */

uint32_t floss_hal_time_us(void) {
    return g_time;
}

static int store_find(uint16_t key) {
    for (int i = 0; i < FLOSS_TEST_STORE_SLOTS; i++) {
        if (g_store[i].used && g_store[i].key == key) {
            return i;
        }
    }
    return -1;
}

int floss_hal_store_read(uint16_t key, void *buf, uint16_t len) {
    int i = store_find(key);
    if (i < 0) {
        return FLOSS_ERR_NOT_FOUND;
    }
    uint16_t n = g_store[i].len < len ? g_store[i].len : len;
    memcpy(buf, g_store[i].val, n);
    return n;
}

floss_err_t floss_hal_store_write(uint16_t key, const void *buf, uint16_t len) {
    if (len > FLOSS_TEST_STORE_VAL_MAX) {
        return FLOSS_ERR_INVALID;
    }
    int i = store_find(key);
    if (i < 0) {
        for (i = 0; i < FLOSS_TEST_STORE_SLOTS && g_store[i].used; i++) {
        }
        if (i == FLOSS_TEST_STORE_SLOTS) {
            return FLOSS_ERR_NOMEM;
        }
    }
    g_store[i].used = true;
    g_store[i].key = key;
    g_store[i].len = len;
    memcpy(g_store[i].val, buf, len);
    g_store_writes++;
    return FLOSS_OK;
}

floss_err_t floss_hal_store_delete(uint16_t key) {
    int i = store_find(key);
    if (i >= 0) {
        g_store[i].used = false;
    }
    return FLOSS_OK;
}

/** xorshift64*: fast, deterministic, good enough for test randomness. */
void floss_hal_random(uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        g_rng ^= g_rng >> 12;
        g_rng ^= g_rng << 25;
        g_rng ^= g_rng >> 27;
        buf[i] = (uint8_t)((g_rng * 0x2545F4914F6CDD1Dull) >> 56);
    }
}

static void log_capture(const char *text, size_t len) {
    if (len > TEST_LOG_CAPTURE) {
        text += len - TEST_LOG_CAPTURE;
        len = TEST_LOG_CAPTURE;
    }
    if (g_log_len + len > TEST_LOG_CAPTURE) {
        /* Keep the newest half: tests ask about recent events. */
        size_t keep = TEST_LOG_CAPTURE / 2;
        if (keep > g_log_len) {
            keep = g_log_len;
        }
        memmove(g_log, g_log + g_log_len - keep, keep);
        g_log_len = keep;
        if (g_log_len + len > TEST_LOG_CAPTURE) {
            g_log_len = 0;
        }
    }
    memcpy(g_log + g_log_len, text, len);
    g_log_len += len;
    g_log[g_log_len] = '\0';
}

void floss_hal_log_write(const char *text, size_t len) {
    if (g_quiet < 0) {
        const char *env = getenv("FLOSS_TEST_QUIET");
        g_quiet = env && env[0] && env[0] != '0';
    }
    log_capture(text, len);
    if (!g_quiet) {
        fwrite(text, 1, len, stdout);
    }
}

void floss_hal_debug_pin(uint8_t pin, bool level) {
    if (pin < sizeof g_pins) {
        g_pins[pin] = level;
    }
}

uint32_t floss_hal_critical_enter(void) {
    return (uint32_t)g_critical_depth++;
}

void floss_hal_critical_exit(uint32_t state) {
    (void)state;
    g_critical_depth--;
}
