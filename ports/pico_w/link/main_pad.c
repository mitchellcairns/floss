/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * main_pad.c - Pico W link pad: the controller end of the pad <-> dongle
 * link (floss_link, PAD role).
 *
 * Structure:
 *
 *   boot:  USB serial (SDK stdio) -> board bring-up (link_board_init: HAL,
 *          CYW43, FLOSS, address) -> transport from flash -> floss_link_init
 *          -> floss_start
 *
 *   loop:  (async_context_poll  only with FLOSS_PICO_BARE=OFF)
 *          floss_poll           the whole stack incl. the link's report mailbox
 *          bootsel              BOOTSEL as button 1 (sampled every 50 ms)
 *          pad                  demo + buttons -> input report, published
 *                               with floss_link_set_input() when it changed
 *          LED / console / status
 *
 * Reports: exactly the Classic HID gamepad's input report (ID 1, 13 bytes,
 * examples/gamepad/gamepad_state.h), so the dongle hands it to USB as is.
 * The demo's millisecond-counter axis (right stick X) therefore reaches the
 * PC unchanged and tools/hostprobe can measure the whole pad -> radio ->
 * dongle -> USB path. Output reports (ID 2: rumble strong, rumble weak,
 * LED) come back from the dongle and are logged.
 *
 * The loop spins instead of sleeping on floss_poll()'s idle hint: input
 * latency matters more than power on this demo.
 *
 * Pairing: an unbonded pad starts pairing by itself shortly after boot
 * (LINK_PAD_AUTO_PAIR); 'P' starts it again at any time.
 */
#include <stdio.h>

#include "pico/stdlib.h"
#if LIB_PICO_STDIO_USB
#include "pico/stdio_usb.h"
#endif

#include "floss/floss.h"
#include "floss/floss_link.h"
#include "gamepad_state.h"
#include "link_board.h"
#include "link_diag.h"

/** Enter pairing by itself when nothing is bonded (checked once after boot). */
#ifndef LINK_PAD_AUTO_PAIR
#define LINK_PAD_AUTO_PAIR 1
#endif

/** Name the pad shows to a searching dongle (Classic name / LE advertised name). */
#ifndef LINK_PAD_NAME
#define LINK_PAD_NAME "FLOSS Link Pad"
#endif

#define BOOT_USB_WAIT_MS   1500u
#define BOOTSEL_PERIOD_US  50000u
#define STATUS_PERIOD_US   5000000u
#define CONSOLE_PERIOD_US  10000u
#define AUTO_PAIR_DELAY_US 1500000u

static floss_link_transport_t transport;
static floss_link_config_t link_cfg;

/* ------------------------------------------------------------------------ */
/* Gamepad state -> link                                                    */
/* ------------------------------------------------------------------------ */

static gamepad_state_t pad_in; /* real inputs (BOOTSEL) */
static bool pad_demo = true;
static uint8_t pad_last[GAMEPAD_INPUT_REPORT_LEN];
static bool pad_published;
static gamepad_output_t pad_out;

/** Build the report; publish it only when the bytes changed (latest wins in the mailbox). */
static void pad_task(uint32_t now) {
    gamepad_state_t s = pad_in;
    if (pad_demo) {
        gamepad_state_demo(&s, now);
    }
    uint8_t report[GAMEPAD_INPUT_REPORT_LEN];
    gamepad_state_build_report(&s, report);
    if (pad_published && memcmp(report, pad_last, sizeof(report)) == 0) {
        return;
    }
    if (floss_link_set_input(GAMEPAD_REPORT_ID_INPUT, report, (uint8_t)sizeof(report)) == FLOSS_OK) {
        memcpy(pad_last, report, sizeof(report));
        pad_published = true;
    }
}

/* ------------------------------------------------------------------------ */
/* Link callbacks                                                           */
/* ------------------------------------------------------------------------ */

static void on_connected(const floss_addr_t *peer) {
    char s[18];
    floss_addr_to_str(peer, s);
    FLOSS_LOGI(APP, "link up (%s): dongle %s", link_board_transport_name(transport), s);
}

static void on_disconnected(uint8_t reason) {
    FLOSS_LOGI(APP, "link down (%s)", floss_hci_status_str(reason));
    memset(&pad_out, 0, sizeof(pad_out)); /* stop any rumble the host left running */
}

static void on_output(uint8_t report_id, const uint8_t *data, uint8_t len) {
    gamepad_output_t o;
    if (!gamepad_output_parse(report_id, data, len, &o)) {
        FLOSS_LOGD(APP, "ignored output report id %u len %u", report_id, len);
        return;
    }
    if (memcmp(&o, &pad_out, sizeof(o)) != 0) {
        pad_out = o;
        FLOSS_LOGI(APP, "output: rumble strong %u weak %u, led 0x%02X", o.rumble_strong, o.rumble_weak, o.led);
    }
}

static const floss_link_callbacks_t link_callbacks = {
    .on_connected = on_connected,
    .on_disconnected = on_disconnected,
    .on_input = NULL, /* dongle only */
    .on_output = on_output,
    .on_report_tick = NULL,
};

/* ------------------------------------------------------------------------ */
/* Pairing on first boot                                                    */
/* ------------------------------------------------------------------------ */

static void auto_pair_task(uint32_t now) {
#if LINK_PAD_AUTO_PAIR
    static uint32_t ready_at;
    static bool ready_seen, done;
    if (done || !floss_hci_ready()) {
        return;
    }
    if (!ready_seen) {
        ready_seen = true;
        ready_at = now;
        return;
    }
    /* Give the link a moment to start reconnecting to a bonded dongle. */
    if (floss_time_diff(now, ready_at) < (int32_t)AUTO_PAIR_DELAY_US) {
        return;
    }
    done = true;
    if (floss_link_state() == FLOSS_LINK_STATE_IDLE) {
        floss_err_t err = floss_link_pair();
        printf("no dongle bonded: pairing (%s) - press P on the dongle\n", floss_err_str(err));
    }
#else
    (void)now;
#endif
}

/* ------------------------------------------------------------------------ */
/* Console                                                                  */
/* ------------------------------------------------------------------------ */

static void print_help(void) {
    printf("commands:\n"
           "  P  pair with a dongle          x  forget the dongle\n"
           "  c  transport Classic (reboot)  e  transport LE (reboot)\n"
           "  s  stats dump                  r  reset stats\n"
           "  d  demo on/off                 t  trace dump\n"
           "  v  log level DEBUG             q  log level INFO\n"
           "  p  toggle HCI snoop log        h  this help\n");
}

static void print_banner(void) {
    printf("\n=== FLOSS %s - Pico W link pad ===\n", FLOSS_VERSION_STRING);
    link_board_print_addr("address: ", link_board_addr());
    printf("transport: %s, link: %s, demo: %s, snoop: %s\n", link_board_transport_name(transport),
           link_board_state_name(floss_link_state()), pad_demo ? "on" : "off",
           link_board_snoop_on() ? "on" : "off");
    print_help();
}

static void console_task(uint32_t now) {
    /* Polling USB CDC costs a few microseconds; 100 Hz is plenty for keys. */
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
        printf("pairing: %s\n", floss_err_str(err));
        break;
    case 'x':
        err = floss_link_forget();
        printf("forget dongle: %s (P to pair again)\n", floss_err_str(err));
        break;
    case 'c':
        link_board_switch_transport(transport, FLOSS_LINK_CLASSIC);
        break;
    case 'e':
        link_board_switch_transport(transport, FLOSS_LINK_LE);
        break;
    case 's':
        floss_stats_dump();
        floss_link_stats_dump();
        break;
    case 'r':
        floss_stats_reset();
        floss_link_stats_reset();
        printf("stats reset\n");
        break;
    case 'd':
        pad_demo = !pad_demo;
        printf("demo %s\n", pad_demo ? "on" : "off");
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
/* Status line + loop profiler                                              */
/* ------------------------------------------------------------------------ */

/*
 * Loop profiler: worst-case time spent in each main-loop section since the
 * last status line. When floss_stats() shows a large poll_gap or
 * timer_lateness, this says WHICH part of the loop is responsible.
 */
enum { PROF_ASYNC, PROF_FLOSS, PROF_BOOTSEL, PROF_PAD, PROF_LED, PROF_CONSOLE, PROF_COUNT };
static const char *const prof_names[PROF_COUNT] = {"cyw43", "floss", "bootsel", "pad", "led", "console"};
static uint32_t prof_max[PROF_COUNT];

static inline uint32_t prof_mark(int section, uint32_t start) {
    uint32_t end = time_us_32();
    uint32_t d = end - start;
    if (d > prof_max[section]) {
        prof_max[section] = d;
    }
    return end;
}

/* One line every few seconds while connected: is the link healthy? */
static void status_task(uint32_t now) {
    static uint32_t last_t;
    static uint32_t last_sent, last_out;
    if (floss_time_diff(now, last_t) < (int32_t)STATUS_PERIOD_US) {
        return;
    }
    const floss_link_stats_t *ls = floss_link_stats();
    uint32_t elapsed = now - last_t;
    /* Counters may have been reset ('r') since the last line. */
    uint32_t sent = ls->inputs_sent >= last_sent ? ls->inputs_sent - last_sent : ls->inputs_sent;
    uint32_t outs = ls->outputs_received >= last_out ? ls->outputs_received - last_out : ls->outputs_received;
    last_t = now;
    last_sent = ls->inputs_sent;
    last_out = ls->outputs_received;
    if (!floss_link_connected()) {
        return;
    }
    uint32_t rate = (uint32_t)(((uint64_t)sent * 1000000u) / (elapsed ? elapsed : 1u));
    FLOSS_LOGI(APP, "status: connected (%s), %lu reports/s sent, %lu outputs received",
               link_board_transport_name(transport), (unsigned long)rate, (unsigned long)outs);
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
    stdio_init_all();

#if LIB_PICO_STDIO_USB
    /* Give a serial terminal a moment to attach so the boot log is visible,
     * but never wait for one. */
    absolute_time_t usb_deadline = make_timeout_time_ms(BOOT_USB_WAIT_MS);
    while (!stdio_usb_connected() && !time_reached(usb_deadline)) {
        sleep_ms(10);
    }
#endif

    link_board_init(NULL);

    transport = link_board_transport_load();
    floss_link_config_defaults(&link_cfg, FLOSS_LINK_ROLE_PAD, transport);
    link_cfg.name = LINK_PAD_NAME;
    floss_err_t err = floss_link_init(&link_cfg, &link_callbacks);
    if (err != FLOSS_OK) {
        link_board_fatal("floss_link_init", err);
    }

    gamepad_state_init(&pad_in);
    pad_task(time_us_32()); /* a report is ready the moment the link comes up */

    print_banner();

    err = floss_start();
    if (err != FLOSS_OK) {
        link_board_fatal("floss_start", err);
    }

    uint32_t next_bootsel = time_us_32();
    for (;;) {
        uint32_t t = time_us_32();
        link_board_poll_driver();
        t = prof_mark(PROF_ASYNC, t);
        floss_poll();
        t = prof_mark(PROF_FLOSS, t);

        uint32_t now = t;
        if (floss_time_diff(now, next_bootsel) >= 0) {
            next_bootsel = now + BOOTSEL_PERIOD_US;
            pad_in.buttons = link_board_bootsel_pressed() ? 0x0001u : 0u;
            t = prof_mark(PROF_BOOTSEL, t);
        }
        pad_task(t);
        t = prof_mark(PROF_PAD, t);

        link_board_led_task(now);
        link_diag_task(now);
        t = prof_mark(PROF_LED, t);
        console_task(now);
        prof_mark(PROF_CONSOLE, t);
        auto_pair_task(now);
        status_task(now);
    }
}
