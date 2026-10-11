/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_test.h - A tiny dependency-free test framework.
 *
 * Design: every suite is one executable with an explicit table of test
 * functions (no constructor tricks, no linker sections - portable to any C11
 * compiler). A failed ASSERT longjmp()s back to the runner, so assertions
 * work at any call depth, including inside shared helpers, and the next test
 * still runs. The runner resets the test HAL (fake clock, RAM store, PRNG)
 * before each test.
 *
 *     TEST(buf_alloc_all) {
 *         ASSERT_EQ(floss_buf_free_count(), FLOSS_BUF_COUNT);
 *     }
 *     static const floss_test_case_t tests[] = {
 *         TEST_CASE(buf_alloc_all),
 *     };
 *     FLOSS_TEST_MAIN("core", tests)
 *
 * Run `test_core [substring]` to run only matching tests. Exit status is 0
 * when everything passed (ctest relies on it). Set FLOSS_TEST_QUIET=1 to
 * silence stack logs.
 */
#ifndef FLOSS_TEST_H
#define FLOSS_TEST_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "test_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*floss_test_fn_t)(void);

typedef struct {
    const char *name;
    floss_test_fn_t fn;
} floss_test_case_t;

#define TEST(name) static void name(void)
#define TEST_CASE(name) { #name, name }

/** Run the table. Returns the process exit code. */
int floss_test_main(const char *suite, const floss_test_case_t *cases, size_t count, int argc, char **argv);

#define FLOSS_TEST_MAIN(suite, table)                                                         \
    int main(int argc, char **argv) {                                                         \
        return floss_test_main((suite), (table), sizeof(table) / sizeof((table)[0]), argc, argv); \
    }

/** Report a failure and abandon the current test (never returns). */
_Noreturn void floss_test_fail(const char *file, int line, const char *fmt, ...)
#if defined(__GNUC__) || defined(__clang__)
    __attribute__((format(printf, 3, 4)))
#endif
    ;

/** Print a hex dump to stdout (diagnostics). */
void floss_test_hexdump(const char *label, const uint8_t *data, size_t len);

/** Find needle in haystack (portable memmem). NULL if absent. */
const uint8_t *floss_test_memfind(const uint8_t *hay, size_t hay_len, const uint8_t *needle, size_t needle_len);

/* ------------------------------------------------------------------------ */
/* Assertions                                                               */
/* ------------------------------------------------------------------------ */

#define ASSERT_TRUE(c)                                                         \
    do {                                                                       \
        if (!(c)) {                                                            \
            floss_test_fail(__FILE__, __LINE__, "ASSERT_TRUE(%s)", #c);        \
        }                                                                      \
    } while (0)

#define ASSERT_FALSE(c)                                                        \
    do {                                                                       \
        if (c) {                                                               \
            floss_test_fail(__FILE__, __LINE__, "ASSERT_FALSE(%s)", #c);       \
        }                                                                      \
    } while (0)

#define FLOSS_TEST__CMP(a, b, op, name)                                                          \
    do {                                                                                         \
        long long floss_t_a_ = (long long)(a);                                                   \
        long long floss_t_b_ = (long long)(b);                                                   \
        if (!(floss_t_a_ op floss_t_b_)) {                                                       \
            floss_test_fail(__FILE__, __LINE__, name "(%s, %s): %lld (0x%llx) vs %lld (0x%llx)", \
                            #a, #b, floss_t_a_, (unsigned long long)floss_t_a_, floss_t_b_,      \
                            (unsigned long long)floss_t_b_);                                     \
        }                                                                                        \
    } while (0)

#define ASSERT_EQ(a, b) FLOSS_TEST__CMP(a, b, ==, "ASSERT_EQ")
#define ASSERT_NE(a, b) FLOSS_TEST__CMP(a, b, !=, "ASSERT_NE")
#define ASSERT_LT(a, b) FLOSS_TEST__CMP(a, b, <, "ASSERT_LT")
#define ASSERT_LE(a, b) FLOSS_TEST__CMP(a, b, <=, "ASSERT_LE")
#define ASSERT_GT(a, b) FLOSS_TEST__CMP(a, b, >, "ASSERT_GT")
#define ASSERT_GE(a, b) FLOSS_TEST__CMP(a, b, >=, "ASSERT_GE")

#define ASSERT_NULL(p)                                                         \
    do {                                                                       \
        if ((p) != NULL) {                                                     \
            floss_test_fail(__FILE__, __LINE__, "ASSERT_NULL(%s)", #p);        \
        }                                                                      \
    } while (0)

#define ASSERT_NOT_NULL(p)                                                     \
    do {                                                                       \
        if ((p) == NULL) {                                                     \
            floss_test_fail(__FILE__, __LINE__, "ASSERT_NOT_NULL(%s)", #p);    \
        }                                                                      \
    } while (0)

#define ASSERT_MEM_EQ(a, b, n)                                                              \
    do {                                                                                    \
        const uint8_t *floss_t_pa_ = (const uint8_t *)(a);                                  \
        const uint8_t *floss_t_pb_ = (const uint8_t *)(b);                                  \
        size_t floss_t_n_ = (size_t)(n);                                                    \
        if (memcmp(floss_t_pa_, floss_t_pb_, floss_t_n_) != 0) {                            \
            floss_test_hexdump("  actual  ", floss_t_pa_, floss_t_n_);                      \
            floss_test_hexdump("  expected", floss_t_pb_, floss_t_n_);                      \
            floss_test_fail(__FILE__, __LINE__, "ASSERT_MEM_EQ(%s, %s, %s)", #a, #b, #n);   \
        }                                                                                   \
    } while (0)

#define ASSERT_STR_EQ(a, b)                                                                     \
    do {                                                                                        \
        const char *floss_t_sa_ = (a);                                                          \
        const char *floss_t_sb_ = (b);                                                          \
        if (strcmp(floss_t_sa_, floss_t_sb_) != 0) {                                            \
            floss_test_fail(__FILE__, __LINE__, "ASSERT_STR_EQ(%s, %s): \"%s\" vs \"%s\"", #a, #b, \
                            floss_t_sa_, floss_t_sb_);                                          \
        }                                                                                       \
    } while (0)

/** Unconditional failure with a message. */
#define FAIL(...) floss_test_fail(__FILE__, __LINE__, __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_TEST_H */
