/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_sched.c - Microsecond one-shot timers for the cooperative run loop.
 *
 * Overview
 * --------
 * Active timers form one intrusive singly linked list sorted by deadline
 * (earliest first). Timers with equal deadlines keep their arming order, so
 * two timers armed for the same instant fire first-armed-first.
 *
 * All comparisons use floss_time_diff(), so the 32-bit microsecond clock may
 * wrap (every ~71.6 minutes) without disturbing the order. The only rule is
 * that no deadline lies more than 2^31 us (~35 minutes) away from "now",
 * which floss_timer_start() enforces.
 *
 * Firing and re-entrancy
 * ----------------------
 * floss_sched_run() first detaches every timer that is due at `now_us` into
 * a private "due" list, then fires them one at a time. Callbacks may freely
 * start/stop any timer, including their own and ones still waiting in the
 * due list:
 *   - stopping a due timer removes it from the due list (it will not fire);
 *   - re-arming a due timer moves it back to the sorted list;
 *   - a timer armed during a callback never fires in the same run, even if
 *     its deadline has already passed. It fires on the next floss_poll(),
 *     and floss_sched_run() returns 0 so the caller knows not to sleep.
 * This bounds the work of one run and makes "re-arm with delay 0" a yield
 * instead of an infinite loop.
 */
#include "floss/floss_sched.h"
#include "floss/floss_debug.h"
#include "floss/floss_hal.h"

/** Deadlines must stay within half the clock range for wrap-safe ordering. */
#define FLOSS_SCHED_MAX_DELAY_US 0x7FFFFFFFu

static floss_timer_t *s_head; /* sorted, armed, not yet due */
static floss_timer_t *s_due;  /* detached by the current floss_sched_run() */
static bool s_running;        /* floss_sched_run() is not re-entrant */

/** Remove t from *list if present. O(n) over a handful of timers. */
static bool list_unlink(floss_timer_t **list, floss_timer_t *t)
{
    for (floss_timer_t **pp = list; *pp != NULL; pp = &(*pp)->next) {
        if (*pp == t) {
            *pp = t->next;
            t->next = NULL;
            return true;
        }
    }
    return false;
}

static void timer_detach(floss_timer_t *t)
{
    if (!t->active) {
        return;
    }
    if (!list_unlink(&s_head, t)) {
        (void)list_unlink(&s_due, t);
    }
    t->active = false;
}

void floss_sched_init(void)
{
    s_head = NULL;
    s_due = NULL;
    s_running = false;
}

void floss_timer_init(floss_timer_t *t, floss_timer_fn fn, void *arg)
{
    t->next = NULL;
    t->deadline_us = 0;
    t->fn = fn;
    t->arg = arg;
    t->active = false;
}

void floss_timer_start_at(floss_timer_t *t, uint32_t deadline_us)
{
    floss_timer_t **pp;

    FLOSS_ASSERT(t->fn != NULL);
    timer_detach(t);

    t->deadline_us = deadline_us;
    /* Insert after every timer whose deadline is <= ours (FIFO among equals). */
    pp = &s_head;
    while (*pp != NULL && floss_time_diff((*pp)->deadline_us, deadline_us) <= 0) {
        pp = &(*pp)->next;
    }
    t->next = *pp;
    *pp = t;
    t->active = true;
}

void floss_timer_start(floss_timer_t *t, uint32_t delay_us)
{
    if (delay_us > FLOSS_SCHED_MAX_DELAY_US) {
        delay_us = FLOSS_SCHED_MAX_DELAY_US;
    }
    floss_timer_start_at(t, floss_hal_time_us() + delay_us);
}

void floss_timer_stop(floss_timer_t *t)
{
    timer_detach(t);
}

uint32_t floss_sched_run(uint32_t now_us)
{
    floss_timer_t **tail;
    bool fired = false;
    int32_t until;

    FLOSS_ASSERT(!s_running); /* a timer callback must not call floss_poll() */
    s_running = true;

    /* Detach the due prefix of the sorted list in one go. */
    tail = &s_head;
    while (*tail != NULL && floss_time_diff(now_us, (*tail)->deadline_us) >= 0) {
        tail = &(*tail)->next;
    }
    if (tail != &s_head) {
        s_due = s_head;
        s_head = *tail;
        *tail = NULL;
    }

    while (s_due != NULL) {
        floss_timer_t *t = s_due;
        uint32_t t_fire = floss_hal_time_us();
        int32_t late = floss_time_diff(t_fire, t->deadline_us);
        uint32_t lateness = (late > 0) ? (uint32_t)late : 0u;

        s_due = t->next;
        t->next = NULL;
        t->active = false;
        fired = true;

        FLOSS_STAT_HIST(timer_lateness, lateness);
        FLOSS_TRACE(FLOSS_TR_TIMER, lateness);
        FLOSS_UNUSED(lateness); /* when stats and trace are compiled out */
        t->fn(t, t->arg);
    }
    s_running = false;

    if (s_head == NULL) {
        return UINT32_MAX;
    }
    /* Callbacks consume time: re-read the clock so the caller does not
     * oversleep the next deadline by the length of this run. */
    until = floss_time_diff(s_head->deadline_us, fired ? floss_hal_time_us() : now_us);
    return (until > 0) ? (uint32_t)until : 0u;
}
