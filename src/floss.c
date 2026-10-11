/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss.c - Top-level lifecycle: floss_init(), floss_start(), floss_poll().
 *
 * Overview
 * --------
 * floss_init() wires the layers together bottom-up (core services first, then
 * HCI, GAP, L2CAP, SDP and the optional LE / RFCOMM families) so that every
 * layer's init may rely on the ones below it being ready. No radio traffic
 * happens until floss_start(), which lets the application register HID/SDP
 * services and tweak settings in between.
 *
 * floss_poll() is the whole run loop of the stack:
 *
 *     poll_gap stats -> HCI (transport RX, event/ACL dispatch, TX flush)
 *                    -> timers due now -> return us until next timer
 *
 * HCI runs before the timers so that a timer callback (e.g. the HID pacing
 * tick) sees the freshest controller credits and link state.
 */
#include "floss_internal.h"

/**
 * Only polls that come at least this late are written to the trace ring.
 * A tight run loop polls millions of times per second; tracing every one
 * would flush all useful history out of the ring within microseconds, while
 * the poll_gap histogram still sees every gap. Set to 0 to trace them all.
 */
#ifndef FLOSS_TRACE_POLL_MIN_GAP_US
#define FLOSS_TRACE_POLL_MIN_GAP_US 500u
#endif

static struct {
    bool initialized;
    bool started;
    bool polled;           /* last_poll_us is valid */
    uint32_t last_poll_us; /* entry time of the previous floss_poll() */
} s_floss;

/** Reject transports that would crash or corrupt memory later on. */
static floss_err_t validate_transport(const floss_transport_t *t)
{
    if (t == NULL || t->ops == NULL) {
        FLOSS_LOGE(CORE, "init: no transport");
        return FLOSS_ERR_INVALID;
    }
    if (t->ops->send == NULL || t->ops->poll == NULL || t->ops->open == NULL) {
        FLOSS_LOGE(CORE, "init: transport '%s' lacks open/send/poll", t->ops->name ? t->ops->name : "?");
        return FLOSS_ERR_INVALID;
    }
    if (t->ops->headroom > FLOSS_TRANSPORT_MAX_HEADROOM) {
        /* Zero-copy TX reserves exactly FLOSS_TRANSPORT_MAX_HEADROOM bytes. */
        FLOSS_LOGE(CORE, "init: transport headroom %u > FLOSS_TRANSPORT_MAX_HEADROOM %u",
                   (unsigned)t->ops->headroom, (unsigned)FLOSS_TRANSPORT_MAX_HEADROOM);
        return FLOSS_ERR_INVALID;
    }
    return FLOSS_OK;
}

floss_err_t floss_init(const floss_init_t *init)
{
    floss_err_t err;

    floss_debug_init();
    if (init == NULL) {
        FLOSS_LOGE(CORE, "init: NULL config");
        return FLOSS_ERR_INVALID;
    }
    err = validate_transport(init->transport);
    if (err != FLOSS_OK) {
        return err;
    }

    s_floss.initialized = false;
    s_floss.started = false;
    s_floss.polled = false;

    floss_buf_pool_init();
    floss_sched_init();
    floss_hci_init(init->transport, init->chipset);
    floss_gap_init();
    floss_l2cap_init();
    floss_sdp_init();
#if FLOSS_ENABLE_LE
    floss_le_init();
    floss_smp_init();
#endif
#if FLOSS_ENABLE_RFCOMM
    floss_rfcomm_init();
#endif

    s_floss.initialized = true;
    FLOSS_LOGI(CORE, "FLOSS %s init: transport %s%s%s, %u bufs x %u B", FLOSS_VERSION_STRING,
               init->transport->ops->name ? init->transport->ops->name : "?",
               init->chipset ? ", chipset " : "",
               (init->chipset && init->chipset->name) ? init->chipset->name : "",
               (unsigned)FLOSS_BUF_COUNT, (unsigned)FLOSS_BUF_SIZE);
    return FLOSS_OK;
}

floss_err_t floss_start(void)
{
    floss_err_t err;

    if (!s_floss.initialized) {
        return FLOSS_ERR_STATE;
    }
    if (s_floss.started) {
        return FLOSS_OK;
    }
    err = floss_hci_start();
    if (err != FLOSS_OK) {
        FLOSS_LOGE(CORE, "start failed: %s", floss_err_str(err));
        return err;
    }
    s_floss.started = true;
    return FLOSS_OK;
}

uint32_t floss_poll(void)
{
    uint32_t now, next;

    if (!s_floss.initialized) {
        return UINT32_MAX;
    }
    FLOSS_DBG_PIN(FLOSS_DBG_PIN_POLL, 1);

    now = floss_hal_time_us();
    FLOSS_STAT_INC(polls);
    if (s_floss.polled) {
        uint32_t gap = now - s_floss.last_poll_us;
        FLOSS_STAT_HIST(poll_gap, gap);
        if (gap >= FLOSS_TRACE_POLL_MIN_GAP_US) {
            FLOSS_TRACE(FLOSS_TR_POLL, gap);
        }
    }
    s_floss.last_poll_us = now;
    s_floss.polled = true;

    if (s_floss.started) {
        floss_hci_poll();
        /* Profiles with ISR-fed mailboxes get a chance to send every poll. */
        floss_hid_poll();
#if FLOSS_ENABLE_LE
        floss_le_poll();
#endif
    }
    next = floss_sched_run(floss_hal_time_us());

    FLOSS_DBG_PIN(FLOSS_DBG_PIN_POLL, 0);
    return next;
}
