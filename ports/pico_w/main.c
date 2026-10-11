/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * main.c - Pico W gamepad firmware: board bring-up, main loop, console.
 *
 * Structure:
 *
 *   boot:  USB serial -> HAL (flash store, spin lock) -> async context +
 *          CYW43 driver -> floss_init (CYW43 transport + chipset quirks)
 *          -> address override -> gamepad app (HID) -> floss_start
 *
 *   loop:  async_context_poll   CYW43 driver housekeeping (bus IRQ work)
 *          floss_poll           the whole stack, never blocks
 *          inputs               BOOTSEL as button 1 (sampled every 50 ms)
 *          gamepad_app_task     publish the report if it changed
 *          LED / console / status
 *
 * The loop spins instead of sleeping on floss_poll()'s idle hint: input
 * latency and pacing precision matter more than power on this demo. A
 * battery product would __wfe() until the next deadline or radio IRQ.
 *
 * The CYW43 driver is used without pico_cyw43_arch: we own the
 * async_context and only need the shared-bus radio interface.
 *
 * LED (on the CYW43, so only driven once the controller is up):
 *   fast blink  no bonded host: discoverable, waiting to pair
 *   slow blink  bonded: connectable / reconnecting
 *   solid       host connected
 */
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/async_context_poll.h"
#include "pico/cyw43_driver.h"
#include "cyw43.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#if LIB_PICO_STDIO_USB
#include "pico/stdio_usb.h"
#endif

#include "floss/floss.h"
#include "floss_cyw43.h"
#include "floss_hal_pico.h"
#include "gamepad_app.h"

#ifndef FLOSS_PICO_SNOOP
#define FLOSS_PICO_SNOOP 0
#endif

#define BOOT_USB_WAIT_MS     1500u
#define BOOTSEL_PERIOD_US    50000u
#define STATUS_PERIOD_US     5000000u
#define CONSOLE_PERIOD_US    10000u
#define LED_FAST_HALF_US     125000u
#define LED_SLOW_HALF_US     500000u

static async_context_poll_t async_ctx;
static floss_addr_t local_addr;
static bool snoop_on;
static bool user_discoverable;

/*
 * The CYW43 driver calls this from cyw43_poll() when the radio core has
 * data. FLOSS pumps the radio from floss_poll() anyway, so the default is a
 * no-op; the CYW43 transport may provide a real (strong) definition.
 */
void __attribute__((weak)) cyw43_bluetooth_hci_process(void) {
}

/* ------------------------------------------------------------------------ */
/* BOOTSEL button                                                           */
/* ------------------------------------------------------------------------ */

/*
 * The BOOTSEL button pulls the flash chip-select (QSPI SS) low. To read it
 * we must stop driving SS, which makes flash unreadable for a moment - so
 * this function runs from RAM with interrupts off (no XIP access at all),
 * floats SS, waits for the pin to settle, samples it and restores it.
 */
static bool __no_inline_not_in_flash_func(bootsel_pressed)(void) {
    const uint ss_index = 1; /* QSPI pads: SCLK, SS, SD0..SD3 */
    uint32_t irq = save_and_disable_interrupts();

    hw_write_masked(&ioqspi_hw->io[ss_index].ctrl, GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);
    for (volatile int i = 0; i < 1000; ++i) {
        /* let the pull-up / button settle; no flash access allowed here */
    }
#if PICO_RP2040
    bool pressed = !(sio_hw->gpio_hi_in & (1u << ss_index));
#else
    bool pressed = !(sio_hw->gpio_hi_in & SIO_GPIO_HI_IN_QSPI_CSN_BITS);
#endif
    hw_write_masked(&ioqspi_hw->io[ss_index].ctrl, GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
                    IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    restore_interrupts(irq);
    return pressed;
}

/* ------------------------------------------------------------------------ */
/* LED                                                                      */
/* ------------------------------------------------------------------------ */

static void led_task(uint32_t now) {
    static int led = -1; /* unknown */
    if (!floss_hci_ready()) {
        return; /* the LED lives on the radio chip */
    }
    bool on;
    if (floss_hid_connected()) {
        on = true;
    } else {
        bool pairing = user_discoverable || floss_gap_bond_count() == 0;
        uint32_t half = pairing ? LED_FAST_HALF_US : LED_SLOW_HALF_US;
        on = ((now / half) & 1u) != 0;
    }
    if ((int)on != led) {
        led = on;
        cyw43_gpio_set(&cyw43_state, CYW43_WL_GPIO_LED_PIN, on);
    }
}

/* ------------------------------------------------------------------------ */
/* Console                                                                  */
/* ------------------------------------------------------------------------ */

static void print_addr(const char *label, const floss_addr_t *a) {
    char s[18];
    floss_addr_to_str(a, s);
    printf("%s%s\n", label, s);
}

static void print_help(void) {
    printf("commands:\n"
           "  s  stats dump            t  trace dump        r  reset stats\n"
           "  d  discoverable on       x  delete all bonds  u  HID unplug (forget host)\n"
           "  v  log level DEBUG       q  log level INFO    p  toggle HCI snoop log\n"
           "  h  this help\n");
}

static void print_banner(void) {
    printf("\n=== FLOSS %s - Pico W Classic HID gamepad ===\n", FLOSS_VERSION_STRING);
    print_addr("address: ", &local_addr);
    printf("bonds: %u, snoop: %s\n", floss_gap_bond_count(), snoop_on ? "on" : "off");
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
    switch (c) {
    case 's':
        floss_stats_dump();
        break;
    case 't':
        floss_trace_dump();
        break;
    case 'r':
        floss_stats_reset();
        printf("stats reset\n");
        break;
    case 'd':
        user_discoverable = true;
        floss_gap_set_scan(true, true);
        printf("discoverable\n");
        break;
    case 'x':
        floss_gap_bond_delete_all();
        printf("all bonds deleted\n");
        if (floss_hid_state() != FLOSS_HID_STATE_IDLE) {
            floss_hid_disconnect(); /* HID becomes discoverable when the link drops */
        } else {
            floss_gap_set_scan(true, true);
        }
        break;
    case 'u': {
        floss_err_t err = floss_hid_unplug();
        printf("unplug: %s\n", floss_err_str(err));
        break;
    }
    case 'v':
        floss_log_set_level_all(FLOSS_LOG_DEBUG);
        printf("log level DEBUG\n");
        break;
    case 'q':
        floss_log_set_level_all(FLOSS_LOG_INFO);
        printf("log level INFO\n");
        break;
    case 'p':
        snoop_on = !snoop_on;
        floss_snoop_set_hook(snoop_on ? floss_snoop_log_hook : NULL);
        printf("snoop %s\n", snoop_on ? "on" : "off");
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

/* One line every few seconds while connected: is the link healthy? */
static void status_task(uint32_t now) {
    static uint32_t last_t;
    static uint32_t last_sent;
    if (floss_time_diff(now, last_t) < (int32_t)STATUS_PERIOD_US) {
        return;
    }
    const floss_stats_t *st = floss_stats();
    uint32_t elapsed = now - last_t;
    uint32_t sent = st->hid_reports_sent;
    uint32_t delta = sent >= last_sent ? sent - last_sent : sent; /* stats may have been reset */
    last_t = now;
    last_sent = sent;
    if (!floss_hid_connected()) {
        return;
    }
    uint32_t rate = (uint32_t)(((uint64_t)delta * 1000000u) / (elapsed ? elapsed : 1u));
    FLOSS_LOGI(APP, "status: connected, %lu reports/s, ack p50 %lu us p99 %lu us, deferred %lu, coalesced %lu",
               (unsigned long)rate, (unsigned long)floss_hist_percentile(&st->acl_ack_latency, 50),
               (unsigned long)floss_hist_percentile(&st->acl_ack_latency, 99), (unsigned long)st->hid_send_deferred,
               (unsigned long)st->hid_reports_coalesced);
}

/* ------------------------------------------------------------------------ */
/* GAP events of interest to the board                                      */
/* ------------------------------------------------------------------------ */

static void board_gap_event(const floss_gap_evt_t *evt, void *user) {
    (void)user;
    switch (evt->type) {
    case FLOSS_GAP_EVT_READY:
        local_addr = floss_hci_controller()->addr;
        print_addr("controller ready, address ", &local_addr);
        break;
    case FLOSS_GAP_EVT_CONNECTED:
        user_discoverable = false;
        break;
    case FLOSS_GAP_EVT_HARDWARE_ERROR:
        printf("radio hardware error 0x%02X\n", evt->status);
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------------ */
/* Main                                                                     */
/* ------------------------------------------------------------------------ */

static void fatal(const char *what, int err) {
    for (;;) {
        printf("FATAL: %s (%d)\n", what, err);
        sleep_ms(2000);
    }
}

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

    floss_pico_hal_init();

    if (!async_context_poll_init_with_defaults(&async_ctx)) {
        fatal("async context init", 0);
    }
    if (!cyw43_driver_init(&async_ctx.core)) {
        fatal("cyw43 driver init", 0);
    }

    floss_init_t init = {
        .transport = floss_cyw43_transport(),
        .chipset = &floss_chipset_cyw43,
    };
    floss_err_t err = floss_init(&init);
    if (err != FLOSS_OK) {
        fatal("floss_init", err);
    }
    floss_log_set_level_all(FLOSS_LOG_INFO);

    floss_cyw43_default_addr(&local_addr);
    floss_hci_set_addr_override(&local_addr);

#if FLOSS_PICO_SNOOP
    snoop_on = true;
    floss_snoop_set_hook(floss_snoop_log_hook);
#endif

    floss_gap_add_listener(board_gap_event, NULL);

    err = gamepad_app_init();
    if (err != FLOSS_OK) {
        fatal("gamepad_app_init", err);
    }

    print_banner();

    err = floss_start();
    if (err != FLOSS_OK) {
        fatal("floss_start", err);
    }

    uint32_t next_bootsel = time_us_32();
    for (;;) {
        async_context_poll(&async_ctx.core);
        floss_poll();

        uint32_t now = time_us_32();
        if (floss_time_diff(now, next_bootsel) >= 0) {
            next_bootsel = now + BOOTSEL_PERIOD_US;
            gamepad_app_set_buttons(bootsel_pressed() ? 0x0001u : 0u);
        }
        gamepad_app_task();

        led_task(now);
        console_task(now);
        status_task(now);
    }
}
