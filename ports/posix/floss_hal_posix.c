/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_hal_posix.c - Desktop HAL for Windows and POSIX (Linux, macOS, BSD).
 *
 * Overview
 * --------
 * Implements every function of include/floss/floss_hal.h on a desktop OS so
 * the stack can drive a real controller (e.g. a UART HCI dongle through
 * floss_serial.c) or a simulated one from an ordinary process.
 *
 *   time      Windows: QueryPerformanceCounter. POSIX: clock_gettime
 *             (CLOCK_MONOTONIC). Both start near 0 at the first call so log
 *             timestamps read as "seconds since start", plus an optional
 *             offset (floss_posix_set_time_offset) to test 32-bit wrap.
 *   storage   A RAM table loaded lazily from a small binary file and
 *             rewritten (via a temp file + rename, so a crash never leaves a
 *             half-written store) on every change. Writes are rare (bonding),
 *             so simplicity wins over incremental updates.
 *   entropy   rand_s() (Windows) or /dev/urandom, with a time-seeded
 *             xorshift fallback. Plenty for Classic HID; see floss_hal.h.
 *   log       stderr, optionally mirrored to a file.
 *   critical  A process-wide recursive mutex (CRITICAL_SECTION / pthread).
 *   pins      No-op (no GPIOs on a desktop).
 *
 * Store file format (all little-endian):
 *     "FLST" u8 version(=1) { u16 key, u16 len, u8 value[len] }*
 */
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0A00
#endif
#define _CRT_RAND_S /* exposes rand_s() in stdlib.h */
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS /* plain fopen() is fine here */
#endif
#include <windows.h>
#else
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#include <pthread.h>
#include <time.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "floss/floss_hal.h"
#include "floss_posix_hal.h"

/* ======================================================================== */
/* Time                                                                     */
/* ======================================================================== */

static uint32_t s_time_offset_us;

#ifdef _WIN32
static INIT_ONCE s_time_once = INIT_ONCE_STATIC_INIT;
static LONGLONG s_qpc_freq;
static LONGLONG s_qpc_base;

static BOOL CALLBACK time_init_once(PINIT_ONCE once, PVOID param, PVOID *ctx)
{
    LARGE_INTEGER f, c;

    (void)once;
    (void)param;
    (void)ctx;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    s_qpc_freq = f.QuadPart;
    s_qpc_base = c.QuadPart;
    return TRUE;
}

uint32_t floss_hal_time_us(void)
{
    LARGE_INTEGER c;
    uint64_t ticks, us;

    InitOnceExecuteOnce(&s_time_once, time_init_once, NULL, NULL);
    QueryPerformanceCounter(&c);
    ticks = (uint64_t)(c.QuadPart - s_qpc_base);
    /* Split the conversion so ticks * 1e6 cannot overflow 64 bits. */
    us = (ticks / (uint64_t)s_qpc_freq) * 1000000u + ((ticks % (uint64_t)s_qpc_freq) * 1000000u) / (uint64_t)s_qpc_freq;
    return (uint32_t)us + s_time_offset_us;
}
#else
static pthread_once_t s_time_once = PTHREAD_ONCE_INIT;
static uint64_t s_time_base_us;

static uint64_t mono_us(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

static void time_init_once(void)
{
    s_time_base_us = mono_us();
}

uint32_t floss_hal_time_us(void)
{
    pthread_once(&s_time_once, time_init_once);
    return (uint32_t)(mono_us() - s_time_base_us) + s_time_offset_us;
}
#endif

void floss_posix_set_time_offset(uint32_t offset_us)
{
    s_time_offset_us = offset_us;
}

void floss_posix_sleep_us(uint32_t us)
{
#ifdef _WIN32
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif
    static HANDLE s_timer;
    static bool s_timer_tried;
    LARGE_INTEGER due;

    if (us == 0) {
        SwitchToThread();
        return;
    }
    if (!s_timer_tried) {
        s_timer_tried = true;
        s_timer = CreateWaitableTimerExW(NULL, NULL, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
    }
    if (s_timer != NULL) {
        due.QuadPart = -(LONGLONG)us * 10; /* relative, 100 ns units */
        if (SetWaitableTimer(s_timer, &due, 0, NULL, NULL, FALSE)) {
            WaitForSingleObject(s_timer, INFINITE);
            return;
        }
    }
    Sleep((us + 999u) / 1000u); /* old Windows: ms granularity */
#else
    struct timespec ts;

    ts.tv_sec = (time_t)(us / 1000000u);
    ts.tv_nsec = (long)(us % 1000000u) * 1000L;
    nanosleep(&ts, NULL);
#endif
}

/* ======================================================================== */
/* Critical sections                                                        */
/* ======================================================================== */

#ifdef _WIN32
static INIT_ONCE s_cs_once = INIT_ONCE_STATIC_INIT;
static CRITICAL_SECTION s_cs;

static BOOL CALLBACK cs_init_once(PINIT_ONCE once, PVOID param, PVOID *ctx)
{
    (void)once;
    (void)param;
    (void)ctx;
    InitializeCriticalSection(&s_cs); /* recursive by design */
    return TRUE;
}

uint32_t floss_hal_critical_enter(void)
{
    InitOnceExecuteOnce(&s_cs_once, cs_init_once, NULL, NULL);
    EnterCriticalSection(&s_cs);
    return 0;
}

void floss_hal_critical_exit(uint32_t state)
{
    (void)state;
    LeaveCriticalSection(&s_cs);
}
#else
static pthread_once_t s_cs_once = PTHREAD_ONCE_INIT;
static pthread_mutex_t s_cs;

static void cs_init_once(void)
{
    pthread_mutexattr_t attr;

    /* Recursive: nesting enter/exit must not deadlock, as on an MCU where
     * the saved interrupt state makes nesting naturally safe. */
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&s_cs, &attr);
    pthread_mutexattr_destroy(&attr);
}

uint32_t floss_hal_critical_enter(void)
{
    pthread_once(&s_cs_once, cs_init_once);
    pthread_mutex_lock(&s_cs);
    return 0;
}

void floss_hal_critical_exit(uint32_t state)
{
    (void)state;
    pthread_mutex_unlock(&s_cs);
}
#endif

/* ======================================================================== */
/* Persistent storage                                                       */
/* ======================================================================== */

#ifndef FLOSS_POSIX_STORE_ENTRIES
#define FLOSS_POSIX_STORE_ENTRIES 64
#endif
#ifndef FLOSS_POSIX_STORE_VALUE_MAX
#define FLOSS_POSIX_STORE_VALUE_MAX 255
#endif

#define STORE_MAGIC   "FLST"
#define STORE_VERSION 1

typedef struct {
    bool used;
    uint16_t key;
    uint16_t len;
    uint8_t data[FLOSS_POSIX_STORE_VALUE_MAX];
} store_entry_t;

static store_entry_t s_store[FLOSS_POSIX_STORE_ENTRIES];
static bool s_store_loaded;
static char s_store_path[512] = "floss_store.bin";

void floss_posix_set_store_path(const char *path)
{
    if (path == NULL || path[0] == '\0') {
        return;
    }
    snprintf(s_store_path, sizeof(s_store_path), "%s", path);
    s_store_loaded = false; /* reload from the new location on next access */
}

/** Load the file into RAM once. A missing or damaged file means "empty". */
static void store_load(void)
{
    FILE *f;
    uint8_t hdr[5];
    size_t n = 0;

    if (s_store_loaded) {
        return;
    }
    s_store_loaded = true;
    memset(s_store, 0, sizeof(s_store));

    f = fopen(s_store_path, "rb");
    if (f == NULL) {
        return;
    }
    if (fread(hdr, 1, sizeof(hdr), f) != sizeof(hdr) || memcmp(hdr, STORE_MAGIC, 4) != 0 || hdr[4] != STORE_VERSION) {
        fclose(f);
        return;
    }
    while (n < FLOSS_POSIX_STORE_ENTRIES) {
        uint8_t rec[4];
        store_entry_t *e = &s_store[n];

        if (fread(rec, 1, sizeof(rec), f) != sizeof(rec)) {
            break;
        }
        e->key = floss_get_le16(&rec[0]);
        e->len = floss_get_le16(&rec[2]);
        if (e->len > FLOSS_POSIX_STORE_VALUE_MAX || fread(e->data, 1, e->len, f) != e->len) {
            memset(e, 0, sizeof(*e)); /* truncated tail: keep what was complete */
            break;
        }
        e->used = true;
        n++;
    }
    fclose(f);
}

/** Rewrite the whole file atomically (temp file + rename). */
static floss_err_t store_save(void)
{
    char tmp[sizeof(s_store_path) + 8];
    FILE *f;
    bool ok;

    snprintf(tmp, sizeof(tmp), "%s.tmp", s_store_path);
    f = fopen(tmp, "wb");
    if (f == NULL) {
        return FLOSS_ERR_IO;
    }
    ok = fwrite(STORE_MAGIC, 1, 4, f) == 4 && fputc(STORE_VERSION, f) != EOF;
    for (size_t i = 0; ok && i < FLOSS_POSIX_STORE_ENTRIES; i++) {
        const store_entry_t *e = &s_store[i];
        uint8_t rec[4];

        if (!e->used) {
            continue;
        }
        floss_put_le16(&rec[0], e->key);
        floss_put_le16(&rec[2], e->len);
        ok = fwrite(rec, 1, sizeof(rec), f) == sizeof(rec) && fwrite(e->data, 1, e->len, f) == e->len;
    }
    ok = (fclose(f) == 0) && ok;
    if (!ok) {
        remove(tmp);
        return FLOSS_ERR_IO;
    }
#ifdef _WIN32
    ok = MoveFileExA(tmp, s_store_path, MOVEFILE_REPLACE_EXISTING) != 0;
#else
    ok = rename(tmp, s_store_path) == 0;
#endif
    if (!ok) {
        remove(tmp);
        return FLOSS_ERR_IO;
    }
    return FLOSS_OK;
}

static store_entry_t *store_find(uint16_t key)
{
    for (size_t i = 0; i < FLOSS_POSIX_STORE_ENTRIES; i++) {
        if (s_store[i].used && s_store[i].key == key) {
            return &s_store[i];
        }
    }
    return NULL;
}

int floss_hal_store_read(uint16_t key, void *buf, uint16_t len)
{
    const store_entry_t *e;
    uint16_t n;

    store_load();
    e = store_find(key);
    if (e == NULL) {
        return FLOSS_ERR_NOT_FOUND;
    }
    n = (e->len < len) ? e->len : len;
    if (n > 0) {
        memcpy(buf, e->data, n);
    }
    return n;
}

floss_err_t floss_hal_store_write(uint16_t key, const void *buf, uint16_t len)
{
    store_entry_t *e;

    if (len > FLOSS_POSIX_STORE_VALUE_MAX || (len > 0 && buf == NULL)) {
        return FLOSS_ERR_INVALID;
    }
    store_load();
    e = store_find(key);
    if (e == NULL) {
        for (size_t i = 0; i < FLOSS_POSIX_STORE_ENTRIES && e == NULL; i++) {
            if (!s_store[i].used) {
                e = &s_store[i];
            }
        }
        if (e == NULL) {
            return FLOSS_ERR_NOMEM;
        }
    } else if (e->len == len && memcmp(e->data, buf, len) == 0) {
        return FLOSS_OK; /* unchanged: skip the file rewrite */
    }
    e->used = true;
    e->key = key;
    e->len = len;
    if (len > 0) {
        memcpy(e->data, buf, len);
    }
    return store_save();
}

floss_err_t floss_hal_store_delete(uint16_t key)
{
    store_entry_t *e;

    store_load();
    e = store_find(key);
    if (e == NULL) {
        return FLOSS_OK;
    }
    memset(e, 0, sizeof(*e));
    return store_save();
}

/* ======================================================================== */
/* Entropy                                                                  */
/* ======================================================================== */

/** Last resort if the OS source fails: xorshift32 seeded from the clock. */
static void random_fallback(uint8_t *buf, size_t len)
{
    static uint32_t s_state;

    if (s_state == 0) {
        s_state = floss_hal_time_us() ^ (uint32_t)(uintptr_t)buf ^ 0x9E3779B9u;
        if (s_state == 0) {
            s_state = 1;
        }
    }
    for (size_t i = 0; i < len; i++) {
        s_state ^= s_state << 13;
        s_state ^= s_state >> 17;
        s_state ^= s_state << 5;
        buf[i] = (uint8_t)s_state;
    }
}

void floss_hal_random(uint8_t *buf, size_t len)
{
#ifdef _WIN32
    size_t i = 0;

    while (i < len) {
        unsigned int r;
        if (rand_s(&r) != 0) {
            random_fallback(buf + i, len - i);
            return;
        }
        for (int k = 0; k < 4 && i < len; k++) {
            buf[i++] = (uint8_t)(r >> (8 * k));
        }
    }
#else
    FILE *f = fopen("/dev/urandom", "rb");
    size_t got = 0;

    if (f != NULL) {
        got = fread(buf, 1, len, f);
        fclose(f);
    }
    if (got < len) {
        random_fallback(buf + got, len - got);
    }
#endif
}

/* ======================================================================== */
/* Debug output                                                             */
/* ======================================================================== */

static FILE *s_log_file;

floss_err_t floss_posix_set_log_file(const char *path)
{
    if (s_log_file != NULL) {
        fclose(s_log_file);
        s_log_file = NULL;
    }
    if (path == NULL) {
        return FLOSS_OK;
    }
    s_log_file = fopen(path, "ab");
    return (s_log_file != NULL) ? FLOSS_OK : FLOSS_ERR_IO;
}

void floss_hal_log_write(const char *text, size_t len)
{
    /* stderr is unbuffered, so lines appear immediately and in order with
     * anything the application prints there itself. */
    fwrite(text, 1, len, stderr);
    if (s_log_file != NULL) {
        fwrite(text, 1, len, s_log_file);
        if (len > 0 && text[len - 1] == '\n') {
            fflush(s_log_file); /* keep the file usable while still running */
        }
    }
}

void floss_hal_debug_pin(uint8_t pin, bool level)
{
    (void)pin;
    (void)level;
}
