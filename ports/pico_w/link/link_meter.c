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
} rx;

/*
 * Two sample buffers: arrivals go into buf[active] while the previous
 * window's buffer is analysed a slice at a time by rx_meter_jitter_step(),
 * so closing a window never blocks the main loop (the drift fit over a few
 * thousand samples took ~10 ms on the Cortex-M0+ when done in one go, long
 * enough to stall the radio link).
 */
typedef struct {
    uint32_t n;
    uint16_t x_ms[RX_SAMPLES];
    int32_t y_us[RX_SAMPLES];
} sample_buf_t;
static sample_buf_t bufs[2];
static uint8_t active;

static meter_hist_t jitter;

/* Incremental jitter job over bufs[active ^ 1]. */
#define JOB_SLICE 256u
static struct {
    bool running;
    uint8_t phase;      /* 0 min first half, 1 min second half, 2 min residual, 3 histogram */
    uint32_t i, n, half, ia, ib;
    int32_t slope_q16;  /* us per ms, Q16, clamped to +-0.25 */
    int32_t rmin;
} job;

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
    sample_buf_t *b = &bufs[active];
    if (b->n < RX_SAMPLES) {
        uint32_t ms = (t_rx_us - rx.start_us) / 1000u;
        b->x_ms[b->n] = (uint16_t)(ms > UINT16_MAX ? UINT16_MAX : ms);
        b->y_us[b->n] = (int32_t)(offset - rx.base);
        b->n++;
    }
}

void rx_meter_link_reset(void) {
    rx.have_prev = false;
}

/* Residual of sample i above the drift line through sample ia (32-bit math only). */
static int32_t residual(const sample_buf_t *b, uint32_t i) {
    int32_t dx = (int32_t)b->x_ms[i] - (int32_t)b->x_ms[job.ia];
    return b->y_us[i] - (b->y_us[job.ia] + (int32_t)((job.slope_q16 * dx) >> 16));
}

bool rx_meter_jitter_step(rx_meter_result_t *r) {
    if (!job.running) {
        return true;
    }
    const sample_buf_t *b = &bufs[active ^ 1u];
    uint32_t end = job.i + JOB_SLICE;
    switch (job.phase) {
    case 0: /* lowest latency in the first half */
        for (; job.i < job.half && job.i < end; job.i++) {
            if (b->y_us[job.i] < b->y_us[job.ia]) {
                job.ia = job.i;
            }
        }
        if (job.i >= job.half) {
            job.phase = 1;
            job.ib = job.half;
        }
        return false;
    case 1: /* ... and in the second half: the two minima define the drift line */
        for (; job.i < job.n && job.i < end; job.i++) {
            if (b->y_us[job.i] < b->y_us[job.ib]) {
                job.ib = job.i;
            }
        }
        if (job.i >= job.n) {
            int32_t dx = (int32_t)b->x_ms[job.ib] - (int32_t)b->x_ms[job.ia];
            int32_t dy = b->y_us[job.ib] - b->y_us[job.ia];
            int32_t q = dx > 0 ? (int32_t)(((int64_t)dy << 16) / dx) : 0; /* once per window */
            /* Crystal drift is a few us per second; clamp so x * slope stays in 32 bits. */
            job.slope_q16 = q > 16384 ? 16384 : q < -16384 ? -16384 : q;
            job.phase = 2;
            job.i = 0;
            job.rmin = INT32_MAX;
        }
        return false;
    case 2: /* smallest residual becomes 0 */
        for (; job.i < job.n && job.i < end; job.i++) {
            int32_t res = residual(b, job.i);
            if (res < job.rmin) {
                job.rmin = res;
            }
        }
        if (job.i >= job.n) {
            job.phase = 3;
            job.i = 0;
        }
        return false;
    default: /* histogram of residuals */
        for (; job.i < job.n && job.i < end; job.i++) {
            meter_hist_add(&jitter, (uint32_t)(residual(b, job.i) - job.rmin));
        }
        if (job.i < job.n) {
            return false;
        }
        job.running = false;
        r->jitter_valid = true;
        r->jitter_p50 = meter_hist_percentile(&jitter, 50);
        r->jitter_p99 = meter_hist_percentile(&jitter, 99);
        r->jitter_max = jitter.max_us;
        return true;
    }
}

void rx_meter_finish(uint32_t now_us, rx_meter_result_t *r) {
    memset(r, 0, sizeof(*r));
    r->reports = rx.reports;
    r->lost = rx.lost;
    r->window_us = rx.started ? now_us - rx.start_us : 0u;
    r->interval_p50 = meter_hist_percentile(&rx.interval, 50);
    r->interval_p99 = meter_hist_percentile(&rx.interval, 99);
    r->interval_max = rx.interval.max_us;

    /* Freeze this window's samples and analyse them incrementally. */
    active ^= 1u;
    const sample_buf_t *b = &bufs[active ^ 1u];
    meter_hist_reset(&jitter);
    job.running = b->n >= JITTER_MIN_SAMPLES;
    job.phase = 0;
    job.i = 1;
    job.n = b->n;
    job.half = b->n / 2u;
    job.ia = 0;
    job.ib = job.half;
    bufs[active].n = 0;

    /* Next window (the interval chain continues across windows). */
    rx.started = true;
    rx.start_us = now_us;
    rx.reports = 0;
    rx.lost = 0;
    meter_hist_reset(&rx.interval);
    rx.have_base = false;
}
