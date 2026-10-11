/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_cyw43_bare.c - Run the CYW43 driver with no async_context, no RTOS
 * and no interrupts: everything happens inside floss_poll().
 *
 * Overview
 * --------
 * The low-level cyw43-driver needs a few platform hooks. The Pico SDK
 * provides them in pico_cyw43_driver on top of async_context (a GPIO
 * interrupt queues a worker, timed workers handle bus sleep, a recursive lock
 * guards the bus). That works, but it is machinery the application did not
 * ask for and cannot see: work happens "somewhere else", at times the
 * application does not choose.
 *
 * This file replaces that glue with the simplest possible model, in the
 * spirit of TinyUSB's tud_task():
 *
 *   - The radio's host-wake line (WL_HOST_WAKE) is level-high while the
 *     chip has something for us. Instead of an interrupt we READ it on every
 *     transport poll - a single GPIO read. (The pin doubles as the SPI data
 *     line, but between our calls the bus is idle, so the level is valid.)
 *   - When the line is high, when the driver asked for a poll
 *     (cyw43_schedule_internal_poll_dispatch) or when its bus-sleep check is
 *     due, we call cyw43_poll() right there.
 *   - Locking is a nesting counter: there is exactly one execution context.
 *   - Delays are busy-waits; the driver's control path polls the bus itself.
 *
 * Cost and determinism are now fully visible: driver work runs inside
 * floss_poll() (measured by floss_stats transport_poll_time), never in an
 * interrupt or a hidden worker. Call floss_poll() often; that is the whole
 * contract.
 *
 * Use: link cyw43_driver_picow + pico_cyw43_driver_headers (NOT
 * pico_cyw43_driver or pico_cyw43_arch), call floss_cyw43_driver_init()
 * once, then floss_init/floss_start/floss_poll as usual.
 *
 * Constraint: all FLOSS and cyw43 calls must come from one core and never
 * from an interrupt handler (the same rule as the rest of FLOSS).
 */
#include <string.h>

#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "hardware/gpio.h"

#include "cyw43.h"
#include "floss_cyw43.h"
#include "floss/floss_debug.h"

/** How often the driver re-checks whether the bus may sleep (as in the SDK glue). */
#define BARE_SLEEP_CHECK_US 50000u

static bool s_initialised;
static volatile bool s_poll_requested; /* cyw43_schedule_internal_poll_dispatch() */
static bool s_sleep_check_armed;
static uint32_t s_sleep_check_at;
static uint8_t s_lock_depth;

bool floss_cyw43_driver_init(void) {
    if (s_initialised) {
        return true;
    }
    cyw43_init(&cyw43_state);
    s_initialised = true;
    return true;
}

/** Run cyw43_poll() if the chip or the driver wants it. Called by floss_cyw43's poll(). */
void floss_cyw43_driver_service(void) {
    if (!s_initialised || cyw43_poll == NULL) {
        return;
    }
    uint32_t now = time_us_32();
    bool due = s_poll_requested || (s_sleep_check_armed && (int32_t)(now - s_sleep_check_at) >= 0);
    /* The chip holds host-wake high while it has work for us (what the SDK glue's level IRQ waits for). */
    if (!due && gpio_get(CYW43_PIN_WL_HOST_WAKE)) {
        due = true;
    }
    if (!due) {
        return;
    }
    s_poll_requested = false;
    if (cyw43_sleep > 0) {
        cyw43_sleep--;
    }
    cyw43_poll();
    s_sleep_check_armed = cyw43_sleep != 0;
    s_sleep_check_at = now + BARE_SLEEP_CHECK_US;
}

/* ------------------------------------------------------------------------ */
/* Hooks the cyw43-driver expects from its platform                          */
/* ------------------------------------------------------------------------ */

void cyw43_schedule_internal_poll_dispatch(void (*func)(void)) {
    (void)func; /* always cyw43_poll */
    s_poll_requested = true;
}

void cyw43_post_poll_hook(void) {
    /* Nothing to re-arm: there is no interrupt. */
}

void cyw43_thread_enter(void) {
    s_lock_depth++;
}

void cyw43_thread_exit(void) {
    if (s_lock_depth) {
        s_lock_depth--;
    }
}

#ifndef NDEBUG
void cyw43_thread_lock_check(void) {
    /* Single context: nothing to check. */
}
#endif

void cyw43_await_background_or_timeout_us(uint32_t timeout_us) {
    /* Called by the control path while it waits for the chip; it polls the
     * bus itself after this returns, so a plain wait is all that is needed. */
    busy_wait_us_32(timeout_us);
}

void cyw43_delay_ms(uint32_t ms) {
    busy_wait_ms(ms);
}

void cyw43_delay_us(uint32_t us) {
    busy_wait_us_32(us);
}

/* Generate a MAC if the chip's OTP has none (same rule as the SDK glue). */
void __attribute__((weak)) cyw43_hal_generate_laa_mac(int idx, uint8_t buf[6]) {
    (void)idx;
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    memcpy(buf, &id.id[2], 6);
    buf[0] &= (uint8_t)~0x1u; /* unicast */
    buf[0] |= 0x2u;           /* locally administered */
}

void cyw43_hal_get_mac(int idx, uint8_t buf[6]) {
    (void)idx;
    memcpy(buf, cyw43_state.mac, 6); /* loaded from OTP when the chip comes up */
}

/* The driver references these; FLOSS has no use for them. */
uint32_t storage_read_blocks(uint8_t *dest, uint32_t block_num, uint32_t num_blocks) {
    (void)dest;
    (void)block_num;
    (void)num_blocks;
    return 0;
}

#if !CYW43_LWIP
void cyw43_cb_tcpip_init(cyw43_t *self, int itf) {
    (void)self;
    (void)itf;
}
void cyw43_cb_tcpip_deinit(cyw43_t *self, int itf) {
    (void)self;
    (void)itf;
}
void cyw43_cb_tcpip_set_link_up(cyw43_t *self, int itf) {
    (void)self;
    (void)itf;
}
void cyw43_cb_tcpip_set_link_down(cyw43_t *self, int itf) {
    (void)self;
    (void)itf;
}
void cyw43_cb_process_ethernet(void *cb_data, int itf, size_t len, const uint8_t *buf) {
    (void)cb_data;
    (void)itf;
    (void)len;
    (void)buf;
}
#endif
