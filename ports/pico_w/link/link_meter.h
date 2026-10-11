/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * link_meter.h - Windowed timing statistics for the dongle's status line.
 *
 * floss_link_stats() keeps cumulative log2 histograms: right for "has this
 * ever gone wrong", too coarse for the status line (a 1.25 ms and a 2 ms
 * report interval land in the same bucket). The dongle therefore measures
 * its own window (one status period) at 25 us resolution:
 *
 *   interval  time between consecutive INPUT arrivals at the dongle
 *   jitter    end-to-end latency variation: (dongle arrival - pad send
 *             time), with the drift between the two boards' crystals
 *             removed (a line through the lowest latency of each half of
 *             the window), offset so 0 = the fastest delivery in the window.
 *             Same idea as tools/hostprobe's "latency above best case".
 *
 * Plain integer code; no floats.
 */
#ifndef LINK_METER_H
#define LINK_METER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Histogram resolution and range: 1024 x 25 us = 25.6 ms, last bin = overflow. */
#define METER_BIN_US 25u
#define METER_BINS   1024u

/** Linear histogram of microsecond values (one window). */
typedef struct {
    uint16_t bin[METER_BINS]; /* saturating */
    uint32_t count;
    uint32_t max_us;
    uint64_t sum_us;
} meter_hist_t;

void meter_hist_reset(meter_hist_t *h);
void meter_hist_add(meter_hist_t *h, uint32_t us);
/** Percentile (0..100) as the upper edge of its bin (25 us resolution); 0 if empty. */
uint32_t meter_hist_percentile(const meter_hist_t *h, unsigned pct);
/** Mean, 0 if empty. */
uint32_t meter_hist_mean(const meter_hist_t *h);

/** Result of one arrival window. */
typedef struct {
    uint32_t reports;       /**< INPUTs received */
    uint32_t lost;          /**< sequence numbers skipped */
    uint32_t window_us;     /**< length of the window */
    uint32_t interval_p50, interval_p99, interval_max;
    uint32_t jitter_p50, jitter_p99, jitter_max;
    bool jitter_valid;      /**< enough samples for the drift fit */
} rx_meter_result_t;

/** Record one INPUT arrival (pad send time, dongle arrival time, sequence gap). */
void rx_meter_add(uint32_t t_pad_us, uint32_t t_rx_us, uint16_t lost);

/** The link dropped: the next arrival starts a new interval chain. */
void rx_meter_link_reset(void);

/**
 * Close the window that ends at now_us and start the next one. Counts and
 * interval percentiles are filled at once; jitter is computed afterwards in
 * small slices: call rx_meter_jitter_step() once per main-loop pass until it
 * returns true (jitter_valid stays false if the window had too few samples).
 */
void rx_meter_finish(uint32_t now_us, rx_meter_result_t *r);
bool rx_meter_jitter_step(rx_meter_result_t *r);

#ifdef __cplusplus
}
#endif

#endif /* LINK_METER_H */
