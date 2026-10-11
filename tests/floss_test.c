/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_test.c - Runner for the tiny test framework (see floss_test.h).
 *
 * Each test runs between a setjmp() and the end of its function; a failing
 * assertion prints file:line plus the values and longjmp()s back here. State
 * touched across that jump lives in file-scope statics, never in locals, so
 * nothing is clobbered by the jump.
 */
#include "floss_test.h"

#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>

static jmp_buf g_jmp;
static const char *g_current;
static unsigned g_passed;
static unsigned g_failed;
static const char *g_failed_names[64];

_Noreturn void floss_test_fail(const char *file, int line, const char *fmt, ...) {
    va_list ap;
    fflush(stdout);
    printf("    %s:%d: FAILED in %s\n      ", file, line, g_current ? g_current : "?");
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
    fflush(stdout);
    longjmp(g_jmp, 1);
}

void floss_test_hexdump(const char *label, const uint8_t *data, size_t len) {
    printf("%s (%u):", label, (unsigned)len);
    for (size_t i = 0; i < len; i++) {
        if (i % 32 == 0 && i) {
            printf("\n%*s", (int)strlen(label) + 8, "");
        }
        printf(" %02x", data[i]);
    }
    printf("\n");
}

const uint8_t *floss_test_memfind(const uint8_t *hay, size_t hay_len, const uint8_t *needle, size_t needle_len) {
    if (needle_len == 0) {
        return hay;
    }
    for (size_t i = 0; i + needle_len <= hay_len; i++) {
        if (hay[i] == needle[0] && memcmp(hay + i, needle, needle_len) == 0) {
            return hay + i;
        }
    }
    return NULL;
}

static void run_one(const floss_test_case_t *tc) {
    g_current = tc->name;
    printf("[ RUN      ] %s\n", tc->name);
    fflush(stdout);
    floss_test_hal_reset();
    if (setjmp(g_jmp) == 0) {
        tc->fn();
        g_passed++;
        printf("[       OK ] %s\n", tc->name);
    } else {
        if (g_failed < sizeof g_failed_names / sizeof g_failed_names[0]) {
            g_failed_names[g_failed] = tc->name;
        }
        g_failed++;
        printf("[  FAILED  ] %s\n", tc->name);
    }
    fflush(stdout);
}

int floss_test_main(const char *suite, const floss_test_case_t *cases, size_t count, int argc, char **argv) {
    const char *filter = argc > 1 ? argv[1] : NULL;
    g_passed = 0;
    g_failed = 0;
    printf("=== suite %s: %u tests%s%s ===\n", suite, (unsigned)count, filter ? ", filter " : "",
           filter ? filter : "");
    for (size_t i = 0; i < count; i++) {
        if (filter && !strstr(cases[i].name, filter)) {
            continue;
        }
        run_one(&cases[i]);
    }
    printf("=== suite %s: %u passed, %u failed ===\n", suite, g_passed, g_failed);
    for (unsigned i = 0; i < g_failed && i < sizeof g_failed_names / sizeof g_failed_names[0]; i++) {
        printf("    failed: %s\n", g_failed_names[i]);
    }
    fflush(stdout);
    return g_failed ? 1 : 0;
}
