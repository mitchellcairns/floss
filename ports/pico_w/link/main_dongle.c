/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * main_dongle.c - Pico W link dongle: the USB receiver end of the pad <->
 * dongle link (floss_link, DONGLE role).
 *
 * To the PC it is a composite USB device (usb_descriptors.c):
 *   - a CDC serial port: console and log (usb_stdio.c), 1200-baud reboot
 *     to BOOTSEL for tools/pico_flash.ps1;
 *   - an HID gamepad with the very report descriptor of examples/gamepad,
 *     polled every 1 ms (usb_gamepad.c).
 * The pad's input reports are forwarded unchanged; rumble output reports
 * from the PC go back to the pad.
 *
 * Structure:
 *
 *   boot:  TinyUSB + CDC stdio -> wait up to 1.5 s for a terminal (USB is
 *          serviced meanwhile) -> board bring-up (link_board_init; the radio
 *          firmware download blocks the loop for a moment) -> transport from
 *          flash -> floss_link_init (DONGLE role, PING every 100 ms) ->
 *          floss_start
 *
 *   loop:  (async_context_poll  only with FLOSS_PICO_BARE=OFF)
 *          floss_poll           the whole stack; on_input hands the report
 *                               to USB right away if the endpoint is free
 *          tud_task             USB device (enumeration, CDC, HID transfers)
 *          app                  pending report -> endpoint, rumble -> pad
 *          LED / console / status
 *
 * Nothing in the loop blocks; console output is dropped rather than
 * waited for (usb_stdio.h).
 *
 * Measuring: every 5 s while connected the status lines give the radio
 * side (reports/s, loss, arrival interval, latency jitter, round trip) and
 * the USB side (reports/s to the host, reports replaced before USB took
 * them, radio-arrival -> host-read wait). From the PC, tools/hostprobe
 * reads the dongle's HID gamepad like any other and measures the full
 * pad -> PC path from the demo's millisecond-counter axis.
 */
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "tusb.h"

#include "floss/floss.h"
#include "floss/floss_link.h"
#include "link_board.h"
#include "link_diag.h"
#include "link_meter.h"
#include "usb_gamepad.h"
#include "usb_stdio.h"

/** Round-trip probe period (ms): PING -> PONG, shown as "rtt" in the status. */
#ifndef LINK_DONGLE_PING_MS
#define LINK_DONGLE_PING_MS 0u
#endif

#define BOOT_USB_WAIT_MS  1500u
#define STATUS_PERIOD_US  5000000u
#define CONSOLE_PERIOD_US 10000u

static floss_link_transport_t transport;
static floss_link_config_t link_cfg;

/* ------------------------------------------------------------------------ */
/* Link callbacks                                                           */
/* ------------------------------------------------------------------------ */

static void on_connected(const floss_addr_t *peer) {
    char s[18];
    floss_addr_to_str(peer, s);
    FLOSS_LOGI(APP, "link up (%s): pad %s", link_board_transport_name(transport), s);
    rx_meter_link_reset();
}

static void on_disconnected(uint8_t reason) {
    FLOSS_LOGI(APP, "link down (%s)", floss_hci_status_str(reason));
    rx_meter_link_reset();
    usb_gamepad_neutral(); /* no stuck buttons on the PC */
}

/*
 * Gap catcher ('g' arms it for the next N gaps): when a report arrives more
 * than GAP_CATCH_US after the previous one, freeze the stack's trace ring so
 * the HCI-level events around the hole can be dumped from the main loop.
 */
#define GAP_CATCH_US 8000u
static uint8_t gap_armed;          /* gaps still to catch */
static volatile uint32_t gap_caught_us; /* 0 = none pending */
static uint32_t gap_last_rx;
static bool gap_log_all;            /* 'G': print every gap (no trace) */
static uint32_t gap_log_len[16], gap_log_at[16];
static volatile uint8_t gap_log_w, gap_log_r;

static void on_input(uint8_t report_id, const uint8_t *data, uint8_t len, const floss_link_rx_info_t *info) {
    usb_gamepad_input(report_id, data, len, info->t_rx_us); /* first: latency */
    rx_meter_add(info->t_pad_us, info->t_rx_us, info->lost);
    uint32_t gap = info->t_rx_us - gap_last_rx;
    gap_last_rx = info->t_rx_us;
    if (gap_log_all && gap >= GAP_CATCH_US && gap < 1000000u && (uint8_t)(gap_log_w - gap_log_r) < 16u) {
        gap_log_len[gap_log_w & 15u] = gap;
        gap_log_at[gap_log_w & 15u] = info->t_rx_us;
        gap_log_w++;
    }
    if (gap_armed && !gap_caught_us && gap >= GAP_CATCH_US && gap < 1000000u) {
        floss_trace_freeze(true);
        gap_caught_us = gap;
        gap_armed--;
    }
}

/* Print the last GAP_LINES trace entries one per loop pass (the USB console drops when flooded). */
#define GAP_LINES 48
static floss_trace_entry_t gap_entries[FLOSS_TRACE_ENTRIES];
static uint16_t gap_n, gap_i;

static void gap_task(void) {
    while (gap_log_r != gap_log_w) {
        printf("gap %lu us ending at %lu ms\n", (unsigned long)gap_log_len[gap_log_r & 15u],
               (unsigned long)(gap_log_at[gap_log_r & 15u] / 1000u));
        gap_log_r++;
    }
    if (gap_caught_us && gap_n == 0) {
        gap_n = floss_trace_read(gap_entries, FLOSS_TRACE_ENTRIES);
        floss_trace_freeze(false);
        gap_i = gap_n > GAP_LINES ? (uint16_t)(gap_n - GAP_LINES) : 0;
        printf("=== GAP %lu us ending t=%lu, last %u trace entries (t relative to the gap end) ===\n",
               (unsigned long)gap_caught_us, (unsigned long)gap_last_rx, (unsigned)(gap_n - gap_i));
        return;
    }
    if (gap_n == 0) {
        return;
    }
    if (gap_i < gap_n) {
        const floss_trace_entry_t *e = &gap_entries[gap_i];
        uint32_t prev = gap_i ? gap_entries[gap_i - 1].t_us : e->t_us;
        printf("  %8ld %7lu  id %2u  arg 0x%08lx\n", (long)(int32_t)(e->t_us - gap_entries[gap_n - 1].t_us),
               (unsigned long)(e->t_us - prev), (unsigned)e->id, (unsigned long)e->arg);
        gap_i++;
        return;
    }
    printf("=== end ===\n");
    gap_n = 0;
    gap_caught_us = 0;
}

static const floss_link_callbacks_t link_callbacks = {
    .on_connected = on_connected,
    .on_disconnected = on_disconnected,
    .on_input = on_input,
    .on_output = NULL, /* pad only */
    .on_report_tick = NULL,
};

/* ------------------------------------------------------------------------ */
/* USB state changes (logged from the loop: TinyUSB callbacks never print)  */
/* ------------------------------------------------------------------------ */

static void usb_state_task(void) {
    static int last = -1;
    int state = !tud_mounted() ? 0 : tud_suspended() ? 1 : 2;
    if (state != last) {
        static const char *const names[] = {"not configured", "suspended", "configured"};
        if (last >= 0) {
            FLOSS_LOGI(APP, "USB %s", names[state]);
        }
        last = state;
    }
}

/* ------------------------------------------------------------------------ */
/* Console                                                                  */
/* ------------------------------------------------------------------------ */

static void print_help(void) {
    printf("commands:\n"
           "  P  pair with a pad             x  forget the pad\n"
           "  c  transport Classic (reboot)  e  transport LE (reboot)\n"
           "  s  stats dump (incl. USB)      r  reset stats\n"
           "  t  trace dump                  p  toggle HCI snoop log\n"
           "  v  log level DEBUG             q  log level INFO\n"
           "  h  this help\n");
}

static void print_banner(void) {
    char serial[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    pico_get_unique_board_id_string(serial, sizeof(serial));
    printf("\n=== FLOSS %s - Pico W link dongle ===\n", FLOSS_VERSION_STRING);
    link_board_print_addr("address: ", link_board_addr());
    printf("USB: 1209:0F11 \"FLOSS Link Dongle\" serial %s, %s\n", serial,
           tud_mounted() ? "configured" : "not configured");
    printf("transport: %s, link: %s, snoop: %s\n", link_board_transport_name(transport),
           link_board_state_name(floss_link_state()), link_board_snoop_on() ? "on" : "off");
    print_help();
}

static void usb_stats_dump(void) {
    const usb_gamepad_stats_t *u = usb_gamepad_stats();
    printf("usb: %s%s, terminal %s\n", tud_mounted() ? "configured" : "not configured",
           tud_suspended() ? " (suspended)" : "", usb_stdio_connected() ? "attached" : "none");
    printf("usb: reports forwarded %lu, read by host %lu, replaced before USB took them %lu, bad %lu\n",
           (unsigned long)u->forwarded, (unsigned long)u->delivered, (unsigned long)u->replaced,
           (unsigned long)u->bad_input);
    printf("usb: outputs from host %lu, sent to pad %lu, dropped %lu; console bytes dropped %lu\n",
           (unsigned long)u->outputs_received, (unsigned long)u->outputs_forwarded,
           (unsigned long)u->outputs_dropped, (unsigned long)usb_stdio_dropped());
}

static void console_task(uint32_t now) {
    static uint32_t next;
    if (floss_time_diff(now, next) < 0) {
        return;
    }
    next = now + CONSOLE_PERIOD_US;
    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT || c < 0) {
        return;
    }
    if (link_diag_key(c) || link_board_debug_key(c)) {
        return;
    }
    floss_err_t err;
    switch (c) {
    case 'P':
        err = floss_link_pair();
        printf("searching for a pad: %s\n", floss_err_str(err));
        break;
    case 'x':
        err = floss_link_forget();
        printf("forget pad: %s (P to pair again)\n", floss_err_str(err));
        break;
    case 'c':
        link_board_switch_transport(transport, FLOSS_LINK_CLASSIC);
        break;
    case 'e':
        link_board_switch_transport(transport, FLOSS_LINK_LE);
        break;
    case 'R': {
        /* Experiment: hand the central role to the pad (or take it back). */
        for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
            floss_conn_t *lc = floss_hci_conn_at(i);
            if (lc->state == FLOSS_CONN_OPEN && !lc->le) {
                uint8_t want = lc->role == 0 ? 1 : 0;
                floss_gap_set_link_policy(lc->handle, FLOSS_LINK_POLICY_ROLE_SWITCH);
                floss_err_t e = floss_gap_switch_role(lc->handle, want);
                printf("role switch to %s: %s\n", want ? "peripheral" : "central", floss_err_str(e));
            }
        }
        break;
    }
    case 'L':
        printf("AFH relearn: %s\n", floss_err_str(floss_link_afh_relearn()));
        break;
    case 'G':
        gap_log_all = !gap_log_all;
        printf("gap log %s\n", gap_log_all ? "on" : "off");
        break;
    case 'g':
        gap_armed = 3;
        printf("gap catcher armed for 3 gaps >= %u us\n", (unsigned)GAP_CATCH_US);
        break;
    case 's':
        floss_stats_dump();
        floss_link_stats_dump();
        usb_stats_dump();
        break;
    case 'r':
        floss_stats_reset();
        floss_link_stats_reset();
        usb_gamepad_stats_reset();
        printf("stats reset\n");
        break;
    case 'h':
    case '?':
        print_banner();
        break;
    case '\r':
    case '\n':
        break;
    default:
        printf("unknown command '%c' (h = help)\n", c);
        break;
    }
}

/* ------------------------------------------------------------------------ */
/* Status lines + loop profiler                                             */
/* ------------------------------------------------------------------------ */

/*
 * Loop profiler: worst-case time spent in each main-loop section since the
 * last status line. When floss_stats() shows a large poll_gap or
 * timer_lateness, this says WHICH part of the loop is responsible.
 */
enum { PROF_ASYNC, PROF_FLOSS, PROF_USB, PROF_APP, PROF_LED, PROF_CONSOLE, PROF_COUNT };
static const char *const prof_names[PROF_COUNT] = {"cyw43", "floss", "usb", "app", "led", "console"};
static uint32_t prof_max[PROF_COUNT];

static inline uint32_t prof_mark(int section, uint32_t start) {
    uint32_t end = time_us_32();
    uint32_t d = end - start;
    if (d > prof_max[section]) {
        prof_max[section] = d;
    }
    return end;
}

/*
 * floss_link_stats() histograms are cumulative: return the part added
 * since *prev (min/max stay cumulative, which only clamps the extremes)
 * and remember the current state. A reset in between restarts the window.
 */
static floss_hist_t hist_window(const floss_hist_t *cur, floss_hist_t *prev) {
    floss_hist_t d = *cur;
    bool reset = cur->count < prev->count;
    for (unsigned i = 0; i < FLOSS_HIST_BUCKETS; i++) {
        reset = reset || cur->bucket[i] < prev->bucket[i];
    }
    if (!reset) {
        d.count -= prev->count;
        d.sum_us -= prev->sum_us;
        for (unsigned i = 0; i < FLOSS_HIST_BUCKETS; i++) {
            d.bucket[i] -= prev->bucket[i];
        }
    }
    *prev = *cur;
    return d;
}

static uint32_t per_second(uint32_t n, uint32_t window_us) {
    return (uint32_t)(((uint64_t)n * 1000000u) / (window_us ? window_us : 1u));
}

/* Every 5 s while connected: radio side, USB side, loop profile. */
static void status_task(uint32_t now) {
    static uint32_t last_t;
    static floss_hist_t rtt_prev;
    static uint32_t fwd_prev, repl_prev;
    /* One window's results, printed once the sliced jitter analysis is done. */
    static bool pending;
    static rx_meter_result_t rx;
    static uint32_t elapsed, waits, wait_p50, wait_p99, wait_max, fwd, repl;
    static floss_hist_t rtt;

    if (pending) {
        if (!rx_meter_jitter_step(&rx)) {
            return; /* a slice of analysis per loop pass: never stall the radio */
        }
        pending = false;
    } else {
        if (floss_time_diff(now, last_t) < (int32_t)STATUS_PERIOD_US) {
            return;
        }
        elapsed = now - last_t;
        last_t = now;
        /* Close every window, connected or not, so the next line covers 5 s. */
        rx_meter_finish(now, &rx);
        waits = usb_gamepad_wait_window(&wait_p50, &wait_p99, &wait_max);
        rtt = hist_window(&floss_link_stats()->rtt, &rtt_prev);
        const usb_gamepad_stats_t *u = usb_gamepad_stats();
        fwd = u->forwarded >= fwd_prev ? u->forwarded - fwd_prev : u->forwarded;
        repl = u->replaced >= repl_prev ? u->replaced - repl_prev : u->replaced;
        fwd_prev = u->forwarded;
        repl_prev = u->replaced;
        pending = true;
        return;
    }
    if (!floss_link_connected()) {
        return;
    }

    FLOSS_LOGI(APP, "status: connected (%s), %lu reports/s, lost %lu, rtt p50 %lu avg %lu us",
               link_board_transport_name(transport), (unsigned long)per_second(rx.reports, rx.window_us),
               (unsigned long)rx.lost, (unsigned long)floss_hist_percentile(&rtt, 50),
               (unsigned long)(rtt.count ? rtt.sum_us / rtt.count : 0u));
    if (rx.jitter_valid) {
        FLOSS_LOGI(APP, "rx interval p50 %lu p99 %lu max %lu us, jitter p50 %lu p99 %lu max %lu us",
                   (unsigned long)rx.interval_p50, (unsigned long)rx.interval_p99, (unsigned long)rx.interval_max,
                   (unsigned long)rx.jitter_p50, (unsigned long)rx.jitter_p99, (unsigned long)rx.jitter_max);
    } else {
        FLOSS_LOGI(APP, "rx interval p50 %lu p99 %lu max %lu us, jitter - (too few reports)",
                   (unsigned long)rx.interval_p50, (unsigned long)rx.interval_p99, (unsigned long)rx.interval_max);
    }
    FLOSS_LOGI(APP, "usb: %lu reports/s to host, replaced %lu, host wait p50 %lu p99 %lu max %lu us (%lu)",
               (unsigned long)per_second(fwd, elapsed), (unsigned long)repl, (unsigned long)wait_p50,
               (unsigned long)wait_p99, (unsigned long)wait_max, (unsigned long)waits);
    char line[96];
    int n = 0;
    for (int i = 0; i < PROF_COUNT && n < (int)sizeof(line); i++) {
        n += snprintf(line + n, sizeof(line) - (size_t)n, " %s %lu", prof_names[i], (unsigned long)prof_max[i]);
        prof_max[i] = 0;
    }
    FLOSS_LOGI(APP, "loop max us:%s", line);
}

/* ------------------------------------------------------------------------ */
/* Main                                                                     */
/* ------------------------------------------------------------------------ */

int main(void) {
    usb_stdio_init();
    usb_gamepad_init();

    /* Enumerate, and give a serial terminal a moment to attach so the boot
     * log is visible - but never wait for one. */
    absolute_time_t usb_deadline = make_timeout_time_ms(BOOT_USB_WAIT_MS);
    while (!usb_stdio_connected() && !time_reached(usb_deadline)) {
        usb_stdio_task();
    }

    link_board_init(usb_stdio_task);

    transport = link_board_transport_load();
    floss_link_config_defaults(&link_cfg, FLOSS_LINK_ROLE_DONGLE, transport);
    link_cfg.name = NULL; /* any FLOSS link pad */
    link_cfg.ping_period_ms = LINK_DONGLE_PING_MS;
    floss_err_t err = floss_link_init(&link_cfg, &link_callbacks);
    if (err != FLOSS_OK) {
        link_board_fatal("floss_link_init", err);
    }

    print_banner();

    err = floss_start();
    if (err != FLOSS_OK) {
        link_board_fatal("floss_start", err);
    }

    for (;;) {
        uint32_t t = time_us_32();
        link_board_poll_driver();
        t = prof_mark(PROF_ASYNC, t);
        floss_poll();
        t = prof_mark(PROF_FLOSS, t);
        usb_stdio_task();
        t = prof_mark(PROF_USB, t);

        uint32_t now = t;
        usb_gamepad_task();
        t = prof_mark(PROF_APP, t);

        link_board_led_task(now);
        link_diag_task(now);
        t = prof_mark(PROF_LED, t);
        console_task(now);
        prof_mark(PROF_CONSOLE, t);
        usb_state_task();
        status_task(now);
        gap_task();
    }
}
