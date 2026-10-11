/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * link_meter.c - Windowed arrival statistics for the dongle status line.
 *
 * Interval: binned as reports arrive. Jitter needs the whole window first
 * (the drift line is only known at the end), so arrivals are kept in a
 * fixed sample array - 4096 entries, ~5 s at the Classic link's 800
 * reports/s; later arrivals in a longer window still count for rate,
 * loss and interval but not for jitter.
 *
 * Drift removal: the latency floor (fastest deliveries) drifts linearly
 * with the crystal mismatch between pad and dongle (tens of ppm: up to a
 * few hundred us per 5 s window). The line through the minimum of each
 * half of the window follows that floor; residuals above it are the
 * jitter. Fixed point: x in ms since the window start, slope in us per ms
 * as Q16.
 */
#include <string.h>

#include "link_meter.h"

#define RX_SAMPLES        4096u
#define JITTER_MIN_SAMPLES 16u

/* ------------------------------------------------------------------------ */
/* Linear histogram                                                         */
/* ------------------------------------------------------------------------ */

void meter_hist_reset(meter_hist_t *h) {
    memset(h, 0, sizeof(*h));
}

void meter_hist_add(meter_hist_t *h, uint32_t us) {
    uint32_t b = us / METER_BIN_US;
    if (b >= METER_BINS) {
        b = METER_BINS - 1u;
    }
    if (h->bin[b] != UINT16_MAX) {
        h->bin[b]++;
    }
    h->count++;
    h->sum_us += us;
    if (us > h->max_us) {
        h->max_us = us;
    }
}

uint32_t meter_hist_percentile(const meter_hist_t *h, unsigned pct) {
    if (h->count == 0) {
        return 0;
    }
    if (pct > 100u) {
        pct = 100u;
    }
    uint64_t rank = ((uint64_t)h->count * pct + 99u) / 100u; /* ceil */
    uint64_t cum = 0;
    for (uint32_t i = 0; i < METER_BINS; i++) {
        cum += h->bin[i];
        if (cum >= rank && rank > 0) {
            uint32_t edge = (i == METER_BINS - 1u) ? h->max_us : (i + 1u) * METER_BIN_US;
            return edge < h->max_us ? edge : h->max_us;
        }
    }
    return h->max_us;
}

uint32_t meter_hist_mean(const meter_hist_t *h) {
    return h->count ? (uint32_t)(h->sum_us / h->count) : 0u;
}

/* ------------------------------------------------------------------------ */
/* Arrival window                                                           */
/* ------------------------------------------------------------------------ */

static struct {
    bool started;
    uint32_t start_us;
    uint32_t reports;
    uint32_t lost;
    meter_hist_t interval;
    bool have_prev;
    uint32_t prev_rx_us;
    /* jitter samples: x = ms since window start, y = latency - first latency */
    bool have_base;
    uint32_t base;
    uint32_t n;
    uint16_t x_ms[RX_SAMPLES];
    int32_t y_us[RX_SAMPLES];
} rx;

static meter_hist_t jitter;

void rx_meter_add(uint32_t t_pad_us, uint32_t t_rx_us, uint16_t lost) {
    if (!rx.started) {
        rx.started = true;
        rx.start_us = t_rx_us;
    }
    rx.reports++;
    rx.lost += lost;
    if (rx.have_prev) {
        meter_hist_add(&rx.interval, t_rx_us - rx.prev_rx_us);
    }
    rx.prev_rx_us = t_rx_us;
    rx.have_prev = true;

    /* Latency + clock offset; both clocks wrap, the difference is what counts. */
    uint32_t offset = t_rx_us - t_pad_us;
    if (!rx.have_base) {
        rx.base = offset;
        rx.have_base = true;
    }
    if (rx.n < RX_SAMPLES) {
        uint32_t ms = (t_rx_us - rx.start_us) / 1000u;
        rx.x_ms[rx.n] = (uint16_t)(ms > UINT16_MAX ? UINT16_MAX : ms);
        rx.y_us[rx.n] = (int32_t)(offset - rx.base);
        rx.n++;
    }
}

void rx_meter_link_reset(void) {
    rx.have_prev = false;
}

/* Jitter of the stored samples into `jitter`; false if too few. */
static bool compute_jitter(void) {
    meter_hist_reset(&jitter);
    uint32_t n = rx.n;
    if (n < JITTER_MIN_SAMPLES) {
        return false;
    }
    /* Lowest latency in each half of the window. */
    uint32_t half = n / 2u, ia = 0, ib = half;
    for (uint32_t i = 1; i < half; i++) {
        if (rx.y_us[i] < rx.y_us[ia]) {
            ia = i;
        }
    }
    for (uint32_t i = half + 1u; i < n; i++) {
        if (rx.y_us[i] < rx.y_us[ib]) {
            ib = i;
        }
    }
    int64_t slope_q16 = 0; /* us per ms, Q16 */
    int32_t dx = (int32_t)rx.x_ms[ib] - (int32_t)rx.x_ms[ia];
    if (dx > 0) {
        slope_q16 = ((int64_t)rx.y_us[ib] - rx.y_us[ia]) * 65536 / dx;
    }
    /* Residuals above the drift line; the smallest becomes 0. */
    int64_t rmin = INT64_MAX;
    for (uint32_t i = 0; i < n; i++) {
        int64_t line = rx.y_us[ia] + slope_q16 * ((int32_t)rx.x_ms[i] - (int32_t)rx.x_ms[ia]) / 65536;
        int64_t r = rx.y_us[i] - line;
        if (r < rmin) {
            rmin = r;
        }
    }
    for (uint32_t i = 0; i < n; i++) {
        int64_t line = rx.y_us[ia] + slope_q16 * ((int32_t)rx.x_ms[i] - (int32_t)rx.x_ms[ia]) / 65536;
        int64_t r = rx.y_us[i] - line - rmin;
        meter_hist_add(&jitter, r > (int64_t)UINT32_MAX ? UINT32_MAX : (uint32_t)r);
    }
    return true;
}

void rx_meter_finish(uint32_t now_us, rx_meter_result_t *r) {
    memset(r, 0, sizeof(*r));
    r->reports = rx.reports;
    r->lost = rx.lost;
    r->window_us = rx.started ? now_us - rx.start_us : 0u;
    r->interval_p50 = meter_hist_percentile(&rx.interval, 50);
    r->interval_p99 = meter_hist_percentile(&rx.interval, 99);
    r->interval_max = rx.interval.max_us;
    r->jitter_valid = compute_jitter();
    if (r->jitter_valid) {
        r->jitter_p50 = meter_hist_percentile(&jitter, 50);
        r->jitter_p99 = meter_hist_percentile(&jitter, 99);
        r->jitter_max = jitter.max_us;
    }

    /* Next window (the interval chain continues across windows). */
    rx.started = true;
    rx.start_us = now_us;
    rx.reports = 0;
    rx.lost = 0;
    meter_hist_reset(&rx.interval);
    rx.have_base = false;
    rx.n = 0;
}
