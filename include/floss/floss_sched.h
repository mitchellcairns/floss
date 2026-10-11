/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_sched.h - Microsecond timers for the cooperative run loop.
 *
 * Timers are intrusive (you own the floss_timer_t storage, no allocation) and
 * kept in a list sorted by deadline, so the scheduler only ever looks at the
 * head: firing is O(1), arming is O(n) with n = active timers (a handful).
 *
 * Precision: a timer fires on the first floss_poll() at/after its deadline.
 * floss_poll() returns the microseconds until the next deadline so the
 * application can sleep exactly that long (or spin) - this is how sub-ms
 * report pacing is achieved without a dedicated hardware timer.
 *
 * Callbacks run in the stack context and may (re)arm any timer, including
 * their own.
 */
#ifndef FLOSS_SCHED_H
#define FLOSS_SCHED_H

#include "floss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct floss_timer floss_timer_t;
typedef void (*floss_timer_fn)(floss_timer_t *timer, void *arg);

struct floss_timer {
    floss_timer_t *next;
    uint32_t deadline_us;
    floss_timer_fn fn;
    void *arg;
    bool active;
};

/** Prepare a timer (does not arm it). */
void floss_timer_init(floss_timer_t *t, floss_timer_fn fn, void *arg);

/** Arm (or re-arm) to fire delay_us from now. */
void floss_timer_start(floss_timer_t *t, uint32_t delay_us);

/** Arm (or re-arm) at an absolute floss_hal_time_us() deadline.
 *  Use this for drift-free periodic work: deadline += period. */
void floss_timer_start_at(floss_timer_t *t, uint32_t deadline_us);

/** Disarm. Safe on an inactive timer. */
void floss_timer_stop(floss_timer_t *t);

static inline bool floss_timer_active(const floss_timer_t *t) {
    return t->active;
}

/**
 * Fire all expired timers. Called by floss_poll().
 * @return microseconds until the next deadline, or UINT32_MAX if none.
 */
uint32_t floss_sched_run(uint32_t now_us);

/** Reset scheduler state (all timers forgotten). Called by floss_init(). */
void floss_sched_init(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_SCHED_H */
