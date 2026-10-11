/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * test_core.c - Unit tests for src/core: buffer pool, queues, timers,
 * histograms, trace ring, address and error helpers.
 *
 * These are the primitives every hot path stands on, so the tests pin down
 * their exact contracts (headroom arithmetic, FIFO order on equal
 * deadlines, wrap-safe time comparisons) rather than just "it works".
 */
#include "floss/floss.h"
#include "floss_test.h"

/* ------------------------------------------------------------------------ */
/* Buffer pool                                                              */
/* ------------------------------------------------------------------------ */

TEST(buf_pool_alloc_all_and_exhaust) {
    static floss_buf_t *bufs[FLOSS_BUF_COUNT];
    floss_buf_pool_init();
    floss_stats_reset();
    ASSERT_EQ(floss_buf_free_count(), FLOSS_BUF_COUNT);
    ASSERT_EQ(floss_buf_low_watermark(), FLOSS_BUF_COUNT);

    for (unsigned i = 0; i < FLOSS_BUF_COUNT; i++) {
        bufs[i] = floss_buf_alloc();
        ASSERT_NOT_NULL(bufs[i]);
        /* Fresh buffers: empty, data placed after the full headroom. */
        ASSERT_EQ(bufs[i]->len, 0);
        ASSERT_TRUE(bufs[i]->data == (uint8_t *)bufs[i]->mem + FLOSS_BUF_HEADROOM);
        for (unsigned j = 0; j < i; j++) {
            ASSERT_TRUE(bufs[i] != bufs[j]);
        }
    }
    ASSERT_EQ(floss_buf_free_count(), 0);
    ASSERT_EQ(floss_buf_low_watermark(), 0);

    ASSERT_NULL(floss_buf_alloc());
#if FLOSS_ENABLE_STATS
    ASSERT_GE(floss_stats()->buf_alloc_failures, 1);
#endif

    for (unsigned i = 0; i < FLOSS_BUF_COUNT; i++) {
        floss_buf_free(bufs[i]);
    }
    floss_buf_free(NULL); /* ignored */
    ASSERT_EQ(floss_buf_free_count(), FLOSS_BUF_COUNT);
    ASSERT_EQ(floss_buf_low_watermark(), 0); /* lowest ever stays recorded */

    /* Reusable after exhaustion. */
    floss_buf_t *b = floss_buf_alloc();
    ASSERT_NOT_NULL(b);
    floss_buf_free(b);
}

TEST(buf_put_push_pull_bounds) {
    floss_buf_pool_init();
    floss_buf_t *b = floss_buf_alloc();
    ASSERT_NOT_NULL(b);

    uint16_t room = floss_buf_tailroom(b);
    ASSERT_EQ(room, sizeof(b->mem) - FLOSS_BUF_HEADROOM);
    ASSERT_GE(room, FLOSS_BUF_SIZE);

    uint8_t *p = floss_buf_put(b, 10);
    ASSERT_TRUE(p == b->data);
    ASSERT_EQ(b->len, 10);
    ASSERT_EQ(floss_buf_tailroom(b), room - 10);
    for (unsigned i = 0; i < 10; i++) {
        p[i] = (uint8_t)i;
    }

    /* One byte too many: refused, nothing changes. */
    ASSERT_NULL(floss_buf_put(b, (uint16_t)(floss_buf_tailroom(b) + 1)));
    ASSERT_EQ(b->len, 10);

    /* Exactly the remaining room fits. */
    ASSERT_NOT_NULL(floss_buf_put(b, floss_buf_tailroom(b)));
    ASSERT_EQ(floss_buf_tailroom(b), 0);
    ASSERT_EQ(b->len, room);
    ASSERT_NULL(floss_buf_put(b, 1));

    /* Headers: push the whole headroom, we land at the start of storage. */
    uint8_t *old = b->data;
    uint8_t *h = floss_buf_push(b, 4);
    ASSERT_TRUE(h == old - 4);
    ASSERT_TRUE(b->data == h);
    ASSERT_EQ(b->len, room + 4);
    h = floss_buf_push(b, FLOSS_BUF_HEADROOM - 4);
    ASSERT_TRUE(h == (uint8_t *)b->mem);
    ASSERT_EQ(b->len, room + FLOSS_BUF_HEADROOM);

    /* Pull them off again: pull returns the old front. */
    uint8_t *front = floss_buf_pull(b, FLOSS_BUF_HEADROOM);
    ASSERT_TRUE(front == (uint8_t *)b->mem);
    ASSERT_TRUE(b->data == old);
    ASSERT_EQ(b->data[3], 3);
    ASSERT_EQ(b->len, room);
    floss_buf_pull(b, b->len);
    ASSERT_EQ(b->len, 0);

    floss_buf_free(b);
    ASSERT_EQ(floss_buf_free_count(), FLOSS_BUF_COUNT);
}

TEST(bufq_fifo_order_and_drain) {
    floss_buf_pool_init();
    floss_bufq_t q;
    floss_bufq_init(&q);
    ASSERT_NULL(floss_bufq_pop(&q));
    ASSERT_EQ(q.count, 0);

    floss_buf_t *a = floss_buf_alloc();
    floss_buf_t *b = floss_buf_alloc();
    floss_buf_t *c = floss_buf_alloc();
    ASSERT_TRUE(a && b && c);

    floss_bufq_push_tail(&q, a);
    floss_bufq_push_tail(&q, b);
    floss_bufq_push_head(&q, c); /* priority insertion */
    ASSERT_EQ(q.count, 3);
    ASSERT_TRUE(floss_bufq_pop(&q) == c);
    ASSERT_TRUE(floss_bufq_pop(&q) == a);
    ASSERT_TRUE(floss_bufq_pop(&q) == b);
    ASSERT_NULL(floss_bufq_pop(&q));
    ASSERT_EQ(q.count, 0);

    /* push_head on an empty queue must also set the tail. */
    floss_bufq_push_head(&q, a);
    floss_bufq_push_tail(&q, b);
    ASSERT_TRUE(floss_bufq_pop(&q) == a);
    ASSERT_TRUE(floss_bufq_pop(&q) == b);

    floss_bufq_push_tail(&q, a);
    floss_bufq_push_tail(&q, b);
    floss_bufq_push_tail(&q, c);
    ASSERT_EQ(floss_buf_free_count(), FLOSS_BUF_COUNT - 3);
    floss_bufq_drain(&q);
    ASSERT_EQ(q.count, 0);
    ASSERT_NULL(q.head);
    ASSERT_EQ(floss_buf_free_count(), FLOSS_BUF_COUNT);
}

/* ------------------------------------------------------------------------ */
/* Timers                                                                   */
/* ------------------------------------------------------------------------ */

static int g_fired[32];
static unsigned g_fired_n;

static void rec_cb(floss_timer_t *t, void *arg) {
    (void)t;
    if (g_fired_n < 32) {
        g_fired[g_fired_n] = (int)(intptr_t)arg;
    }
    g_fired_n++;
}

static uint32_t run_at(uint32_t now) {
    floss_test_time_set(now);
    return floss_sched_run(now);
}

static void timers_reset(uint32_t now) {
    floss_sched_init();
    floss_test_time_set(now);
    g_fired_n = 0;
}

TEST(timer_ordering) {
    floss_timer_t a, b, c;
    timers_reset(1000);
    floss_timer_init(&a, rec_cb, (void *)1);
    floss_timer_init(&b, rec_cb, (void *)2);
    floss_timer_init(&c, rec_cb, (void *)3);
    ASSERT_FALSE(floss_timer_active(&a));

    floss_timer_start(&a, 300);
    floss_timer_start(&b, 100);
    floss_timer_start(&c, 200);
    ASSERT_TRUE(floss_timer_active(&a));

    ASSERT_EQ(run_at(1000), 100);   /* next deadline in 100 us */
    ASSERT_EQ(run_at(1099), 1);
    ASSERT_EQ(g_fired_n, 0);
    ASSERT_EQ(run_at(1250), 50);    /* b and c due, a in 50 us */
    ASSERT_EQ(g_fired_n, 2);
    ASSERT_EQ(g_fired[0], 2);
    ASSERT_EQ(g_fired[1], 3);
    ASSERT_FALSE(floss_timer_active(&b));
    ASSERT_EQ(run_at(1300), UINT32_MAX);
    ASSERT_EQ(g_fired_n, 3);
    ASSERT_EQ(g_fired[2], 1);
    ASSERT_FALSE(floss_timer_active(&a));
}

TEST(timer_same_deadline_fifo) {
    floss_timer_t t[5];
    timers_reset(5000);
    for (int i = 0; i < 5; i++) {
        floss_timer_init(&t[i], rec_cb, (void *)(intptr_t)(i + 1));
        floss_timer_start(&t[i], 100);
    }
    run_at(5100);
    ASSERT_EQ(g_fired_n, 5);
    for (int i = 0; i < 5; i++) {
        ASSERT_EQ(g_fired[i], i + 1); /* armed first, fires first */
    }
}

TEST(timer_wrap) {
    floss_timer_t x, y;
    timers_reset(0xFFFFFFF0u);
    floss_timer_init(&x, rec_cb, (void *)1);
    floss_timer_init(&y, rec_cb, (void *)2);
    floss_timer_start(&x, 0x20); /* deadline 0x00000010, after the wrap */
    floss_timer_start(&y, 0x08); /* deadline 0xFFFFFFF8, before it */

    ASSERT_EQ(run_at(0xFFFFFFF0u), 8);
    ASSERT_EQ(run_at(0xFFFFFFF9u), 0x17);
    ASSERT_EQ(g_fired_n, 1);
    ASSERT_EQ(g_fired[0], 2);
    ASSERT_EQ(run_at(0x0000000Fu), 1); /* past the wrap, x not yet due */
    ASSERT_EQ(g_fired_n, 1);
    ASSERT_EQ(run_at(0x00000010u), UINT32_MAX);
    ASSERT_EQ(g_fired_n, 2);
    ASSERT_EQ(g_fired[1], 1);
}

static unsigned g_rearm_count;
static void rearm_cb(floss_timer_t *t, void *arg) {
    (void)arg;
    g_rearm_count++;
    if (g_rearm_count < 3) {
        floss_timer_start(t, 100); /* periodic: re-arm itself */
    }
}

TEST(timer_rearm_in_callback) {
    floss_timer_t t;
    timers_reset(1000);
    g_rearm_count = 0;
    floss_timer_init(&t, rearm_cb, NULL);
    floss_timer_start(&t, 100);

    ASSERT_EQ(run_at(1100), 100); /* fired once, re-armed for 1200 */
    ASSERT_EQ(g_rearm_count, 1);
    ASSERT_TRUE(floss_timer_active(&t));
    run_at(1150);
    ASSERT_EQ(g_rearm_count, 1);
    run_at(1200);
    ASSERT_EQ(g_rearm_count, 2);
    ASSERT_EQ(run_at(1300), UINT32_MAX);
    ASSERT_EQ(g_rearm_count, 3);
    ASSERT_FALSE(floss_timer_active(&t));
}

TEST(timer_restart_moves_deadline) {
    floss_timer_t a;
    timers_reset(1000);
    floss_timer_init(&a, rec_cb, (void *)1);
    floss_timer_start(&a, 100);
    floss_timer_start(&a, 500); /* re-arm an active timer: only the new deadline counts */
    run_at(1100);
    ASSERT_EQ(g_fired_n, 0);
    run_at(1500);
    ASSERT_EQ(g_fired_n, 1);
    run_at(3000);
    ASSERT_EQ(g_fired_n, 1);
}

static floss_timer_t *g_victim;
static void stopper_cb(floss_timer_t *t, void *arg) {
    (void)t;
    (void)arg;
    floss_timer_stop(g_victim);
    g_fired_n++;
}

TEST(timer_stop) {
    floss_timer_t a, b, s;
    timers_reset(1000);
    floss_timer_init(&a, rec_cb, (void *)1);
    floss_timer_start(&a, 100);
    floss_timer_stop(&a);
    ASSERT_FALSE(floss_timer_active(&a));
    floss_timer_stop(&a); /* safe on an inactive timer */
    ASSERT_EQ(run_at(2000), UINT32_MAX);
    ASSERT_EQ(g_fired_n, 0);

    /* A callback stops another timer that is due in the same run. */
    floss_timer_init(&s, stopper_cb, NULL);
    floss_timer_init(&b, rec_cb, (void *)2);
    g_victim = &b;
    floss_timer_start(&s, 10);
    floss_timer_start(&b, 10);
    run_at(2010);
    ASSERT_EQ(g_fired_n, 1); /* only the stopper ran */
    ASSERT_FALSE(floss_timer_active(&b));
}

TEST(timer_start_at_absolute) {
    floss_timer_t a;
    timers_reset(1000);
    floss_timer_init(&a, rec_cb, (void *)7);
    floss_timer_start_at(&a, 5000);
    ASSERT_EQ(run_at(1000), 4000);
    run_at(4999);
    ASSERT_EQ(g_fired_n, 0);
    run_at(5003); /* late polls still fire it */
    ASSERT_EQ(g_fired_n, 1);
    ASSERT_EQ(g_fired[0], 7);
}

/* ------------------------------------------------------------------------ */
/* Histograms                                                               */
/* ------------------------------------------------------------------------ */

TEST(hist_buckets) {
    /* Layout (floss_debug.h): [0] < 8 us, [i] = [4<<i, 8<<i) us, last = overflow. */
    floss_hist_t h;
    memset(&h, 0, sizeof h);
    floss_hist_add(&h, 5);      /* < 8             -> 0 */
    floss_hist_add(&h, 7);      /*                 -> 0 */
    floss_hist_add(&h, 8);      /* [8, 16)         -> 1 */
    floss_hist_add(&h, 15);     /*                 -> 1 */
    floss_hist_add(&h, 16);     /* [16, 32)        -> 2 */
    floss_hist_add(&h, 1000);   /* [512, 1024)     -> 7 */
    floss_hist_add(&h, 131071); /* [65536, 131072) -> 14 */
    floss_hist_add(&h, 131072); /* overflow        -> 15 */
    floss_hist_add(&h, 5000000);

    ASSERT_EQ(h.count, 9);
    ASSERT_EQ(h.min_us, 5);
    ASSERT_EQ(h.max_us, 5000000);
    ASSERT_EQ(h.sum_us, 5ull + 7 + 8 + 15 + 16 + 1000 + 131071 + 131072 + 5000000);
    ASSERT_EQ(h.bucket[0], 2);
    ASSERT_EQ(h.bucket[1], 2);
    ASSERT_EQ(h.bucket[2], 1);
    ASSERT_EQ(h.bucket[7], 1);
    ASSERT_EQ(h.bucket[14], 1);
    ASSERT_EQ(h.bucket[FLOSS_HIST_BUCKETS - 1], 2);
}

TEST(hist_percentile) {
    floss_hist_t h;
    memset(&h, 0, sizeof h);
    for (int i = 0; i < 90; i++) {
        floss_hist_add(&h, 50);
    }
    for (int i = 0; i < 10; i++) {
        floss_hist_add(&h, 1000);
    }
    /* "Upper bucket edge": an estimate never below the real value. */
    uint32_t p50 = floss_hist_percentile(&h, 50);
    uint32_t p99 = floss_hist_percentile(&h, 99);
    ASSERT_GE(p50, 50);
    ASSERT_LE(p50, 128);
    ASSERT_GE(p99, 1000);
    ASSERT_LE(p99, 1024);
    ASSERT_GE(floss_hist_percentile(&h, 100), 1000);
}

/* ------------------------------------------------------------------------ */
/* Trace ring                                                               */
/* ------------------------------------------------------------------------ */

#if FLOSS_TRACE_ENTRIES > 0
TEST(trace_ring_wraps_and_freezes) {
    static floss_trace_entry_t out[FLOSS_TRACE_ENTRIES + 16];
    const uint32_t total = FLOSS_TRACE_ENTRIES + 10;
    floss_trace_freeze(false);
    for (uint32_t i = 0; i < total; i++) {
        floss_test_time_set(1000 + i);
        floss_trace(FLOSS_TR_USER, i);
    }
    uint16_t n = floss_trace_read(out, (uint16_t)FLOSS_ARRAY_SIZE(out));
    ASSERT_EQ(n, FLOSS_TRACE_ENTRIES); /* full ring, oldest overwritten */
    ASSERT_EQ(out[0].arg, total - FLOSS_TRACE_ENTRIES);
    for (uint16_t i = 0; i < n; i++) {
        ASSERT_EQ(out[i].id, FLOSS_TR_USER);
        ASSERT_EQ(out[i].arg, total - FLOSS_TRACE_ENTRIES + i); /* oldest first, no gaps */
        ASSERT_EQ(out[i].t_us, 1000 + out[i].arg);
    }

    /* Partial read returns the newest entries (what led up to "now"), oldest first. */
    ASSERT_EQ(floss_trace_read(out, 4), 4);
    ASSERT_EQ(out[0].arg, total - 4);
    ASSERT_EQ(out[3].arg, total - 1);

    floss_trace_freeze(true);
    floss_trace(FLOSS_TR_USER + 1, 0xDEAD);
    n = floss_trace_read(out, (uint16_t)FLOSS_ARRAY_SIZE(out));
    ASSERT_NE(out[n - 1].arg, 0xDEAD);
    floss_trace_freeze(false);
    floss_trace(FLOSS_TR_USER + 1, 0xBEEF);
    n = floss_trace_read(out, (uint16_t)FLOSS_ARRAY_SIZE(out));
    ASSERT_EQ(out[n - 1].arg, 0xBEEF);
    ASSERT_EQ(out[n - 1].id, FLOSS_TR_USER + 1);
}
#endif

/* ------------------------------------------------------------------------ */
/* Addresses and error strings                                              */
/* ------------------------------------------------------------------------ */

TEST(addr_roundtrip) {
    floss_addr_t a;
    char s[18];
    ASSERT_TRUE(floss_addr_from_str("12:34:56:78:9A:BC", &a));
    const uint8_t wire[6] = {0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12}; /* LSB first */
    ASSERT_MEM_EQ(a.b, wire, 6);
    floss_addr_to_str(&a, s);
    ASSERT_STR_EQ(s, "12:34:56:78:9A:BC");

    floss_addr_t b;
    ASSERT_TRUE(floss_addr_from_str("12:34:56:78:9a:bc", &b)); /* lower case accepted */
    ASSERT_TRUE(floss_addr_eq(&a, &b));
    ASSERT_FALSE(floss_addr_is_zero(&a));

    floss_addr_t z;
    ASSERT_TRUE(floss_addr_from_str("00:00:00:00:00:00", &z));
    ASSERT_TRUE(floss_addr_is_zero(&z));
    floss_addr_to_str(&z, s);
    ASSERT_STR_EQ(s, "00:00:00:00:00:00");
}

TEST(addr_rejects_malformed) {
    floss_addr_t a;
    ASSERT_FALSE(floss_addr_from_str("", &a));
    ASSERT_FALSE(floss_addr_from_str("12:34:56:78:9A", &a));
    ASSERT_FALSE(floss_addr_from_str("12:34:56:78:9A:BG", &a));
    ASSERT_FALSE(floss_addr_from_str("123:4:56:78:9A:BC", &a));
    ASSERT_FALSE(floss_addr_from_str("12:34:56:78:9A:BC:DE", &a));
    ASSERT_FALSE(floss_addr_from_str("12:34:56:78:9A BC", &a));
    /* '-' separators are accepted (documented in floss_types.h). */
    ASSERT_TRUE(floss_addr_from_str("12-34-56-78-9A-BC", &a));
}

TEST(err_str_all_codes) {
    static const floss_err_t codes[] = {
        FLOSS_OK, FLOSS_ERR_NOMEM, FLOSS_ERR_BUSY, FLOSS_ERR_INVALID, FLOSS_ERR_STATE,
        FLOSS_ERR_NOT_FOUND, FLOSS_ERR_TIMEOUT, FLOSS_ERR_IO, FLOSS_ERR_UNSUPPORTED, FLOSS_ERR_PROTOCOL,
    };
    for (unsigned i = 0; i < FLOSS_ARRAY_SIZE(codes); i++) {
        const char *s = floss_err_str(codes[i]);
        ASSERT_NOT_NULL(s);
        ASSERT_GT(strlen(s), 0);
        for (unsigned j = 0; j < i; j++) {
            ASSERT_TRUE(strcmp(s, floss_err_str(codes[j])) != 0); /* each code distinguishable */
        }
    }
    ASSERT_NOT_NULL(floss_err_str(-1000));
    ASSERT_NOT_NULL(floss_err_str(42));
}

static const floss_test_case_t tests[] = {
    TEST_CASE(buf_pool_alloc_all_and_exhaust),
    TEST_CASE(buf_put_push_pull_bounds),
    TEST_CASE(bufq_fifo_order_and_drain),
    TEST_CASE(timer_ordering),
    TEST_CASE(timer_same_deadline_fifo),
    TEST_CASE(timer_wrap),
    TEST_CASE(timer_rearm_in_callback),
    TEST_CASE(timer_restart_moves_deadline),
    TEST_CASE(timer_stop),
    TEST_CASE(timer_start_at_absolute),
    TEST_CASE(hist_buckets),
    TEST_CASE(hist_percentile),
#if FLOSS_TRACE_ENTRIES > 0
    TEST_CASE(trace_ring_wraps_and_freezes),
#endif
    TEST_CASE(addr_roundtrip),
    TEST_CASE(addr_rejects_malformed),
    TEST_CASE(err_str_all_codes),
};

FLOSS_TEST_MAIN("core", tests)
