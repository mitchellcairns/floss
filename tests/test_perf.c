/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * test_perf.c - Desktop microbenchmarks of the two hottest paths.
 *
 *  1. Input reports: floss_hid_set_input() -> HID -> L2CAP -> HCI ->
 *     transport, 100k times, with a controller that completes packets
 *     instantly. The figure is pure host CPU per report (no air time): on a
 *     133 MHz Cortex-M0+ expect roughly 50-100x the desktop number.
 *  2. Idle floss_poll(): the cost the application pays for calling the
 *     stack when nothing happens.
 *
 * Wall-clock numbers vary per machine, so only sanity is asserted. Results
 * are printed for humans and CI logs.
 */
#include "test_common.h"

#include <stdio.h>
#include <time.h>

static floss_hid_config_t g_cfg;

static double wall_s(void) {
#ifdef TIME_UTC
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
#else
    return (double)clock() / CLOCKS_PER_SEC;
#endif
}

TEST(perf_input_reports) {
    const uint32_t n = 100000;
    floss_test_log_quiet(true);
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 0); /* send on change, as fast as credits allow */
    floss_tc_stack_start(t, &g_cfg);
    floss_sim_chan_t *ctrl, *intr;
    (void)floss_tc_hid_session(&ctrl, &intr);
    floss_log_set_level_all(FLOSS_LOG_ERROR);
    floss_stats_reset();

    uint32_t base = intr->rx_count;
    uint8_t in[FLOSS_TC_INPUT_LEN] = {0};
    double t0 = wall_s();
    for (uint32_t i = 0; i < n; i++) {
        in[0] = (uint8_t)i;
        in[1] = (uint8_t)(i >> 8);
        in[2] = (uint8_t)(i >> 16);
        (void)floss_hid_set_input(FLOSS_TC_INPUT_ID, in, sizeof in);
        (void)floss_poll();
        floss_test_time_advance(5);
    }
    floss_sim_pump(10);
    double t1 = wall_s();

    uint32_t got = intr->rx_count - base;
    double ns = (t1 - t0) * 1e9 / (double)n;
    const floss_stats_t *st = floss_stats();
    printf("perf: %u set_input calls -> %u reports delivered in %.1f ms: %.0f ns per set_input+poll\n",
           (unsigned)n, (unsigned)got, (t1 - t0) * 1e3, ns);
    printf("perf: hid_reports_sent %u, coalesced %u, deferred %u, report age p50 %u us p99 %u us\n",
           (unsigned)st->hid_reports_sent, (unsigned)st->hid_reports_coalesced, (unsigned)st->hid_send_deferred,
           (unsigned)floss_hist_percentile(&st->hid_report_age, 50),
           (unsigned)floss_hist_percentile(&st->hid_report_age, 99));

    ASSERT_GT(got, n / 10); /* the pipe kept moving */
    ASSERT_LE(got, n + 16);  /* never more reports than state changes */
    floss_tc_assert_sim_clean();
}

TEST(perf_idle_poll) {
    const uint32_t n = 100000;
    floss_test_log_quiet(true);
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 1000);
    floss_tc_stack_start(t, &g_cfg);
    floss_log_set_level_all(FLOSS_LOG_ERROR);

    uint32_t min_idle = UINT32_MAX;
    double t0 = wall_s();
    for (uint32_t i = 0; i < n; i++) {
        uint32_t idle = floss_poll();
        if (idle < min_idle) {
            min_idle = idle;
        }
        floss_test_time_advance(1);
    }
    double t1 = wall_s();
    printf("perf: %u idle floss_poll() in %.1f ms: %.0f ns per poll (min reported idle %u us)\n", (unsigned)n,
           (t1 - t0) * 1e3, (t1 - t0) * 1e9 / (double)n, (unsigned)min_idle);
    ASSERT_TRUE(t1 >= t0);
    floss_tc_assert_sim_clean();
}

static const floss_test_case_t tests[] = {
    TEST_CASE(perf_input_reports),
    TEST_CASE(perf_idle_poll),
};

FLOSS_TEST_MAIN("perf", tests)
