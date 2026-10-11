/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * link_board.c - Pico W board helpers shared by the link pad and dongle.
 *
 * Bring-up is the same sequence as the gamepad firmware's main.c:
 *
 *   HAL (flash store, spin lock) -> async context + CYW43 driver ->
 *   floss_init (CYW43 transport + chipset quirks) -> address override
 *   (WLAN MAC + 1, so two boards always differ) -> [app] -> floss_start
 *
 * The CYW43 driver is used without pico_cyw43_arch: we own the
 * async_context and only need the shared-bus radio interface.
 */
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/async_context_poll.h"
#include "pico/cyw43_driver.h"
#include "cyw43.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"

#include "floss_cyw43.h"
#include "floss_hal_pico.h"
#include "link_board.h"

#ifndef FLOSS_PICO_SNOOP
#define FLOSS_PICO_SNOOP 0
#endif

#define LED_FAST_HALF_US   125000u
#define LED_SLOW_HALF_US   500000u
#define LED_IDLE_PERIOD_US 2000000u
#define LED_IDLE_BLIP_US   60000u
#define REBOOT_DELAY_MS    50u

static async_context_poll_t async_ctx;
static floss_addr_t local_addr;
static bool snoop_on;
static link_board_pump_fn board_pump;

/*
 * The CYW43 driver calls this from cyw43_poll() when the radio core has
 * data. FLOSS pumps the radio from floss_poll() anyway, so the default is a
 * no-op; the CYW43 transport may provide a real (strong) definition.
 */
void __attribute__((weak)) cyw43_bluetooth_hci_process(void) {
}

static void pump(void) {
    if (board_pump) {
        board_pump();
    }
}

/* ------------------------------------------------------------------------ */
/* Bring-up                                                                 */
/* ------------------------------------------------------------------------ */

void link_board_print_addr(const char *label, const floss_addr_t *a) {
    char s[18];
    floss_addr_to_str(a, s);
    printf("%s%s\n", label, s);
}

void link_board_fatal(const char *what, int err) {
    for (;;) {
        printf("FATAL: %s (%d)\n", what, err);
        uint32_t start = time_us_32();
        while (time_us_32() - start < 2000000u) {
            pump();
        }
    }
}

static void board_gap_event(const floss_gap_evt_t *evt, void *user) {
    (void)user;
    switch (evt->type) {
    case FLOSS_GAP_EVT_READY:
        local_addr = floss_hci_controller()->addr;
        link_board_print_addr("controller ready, address ", &local_addr);
        break;
    case FLOSS_GAP_EVT_HARDWARE_ERROR:
        printf("radio hardware error 0x%02X\n", evt->status);
        break;
    default:
        break;
    }
}

void link_board_init(link_board_pump_fn usb_pump) {
    board_pump = usb_pump;
    floss_pico_hal_init();

    if (!async_context_poll_init_with_defaults(&async_ctx)) {
        link_board_fatal("async context init", 0);
    }
    if (!cyw43_driver_init(&async_ctx.core)) {
        link_board_fatal("cyw43 driver init", 0);
    }

    floss_init_t init = {
        .transport = floss_cyw43_transport(),
        .chipset = &floss_chipset_cyw43,
    };
    floss_err_t err = floss_init(&init);
    if (err != FLOSS_OK) {
        link_board_fatal("floss_init", err);
    }
    floss_log_set_level_all(FLOSS_LOG_INFO);

    /* Each board's address comes from its own WLAN MAC, so a pad and a
     * dongle never collide. Powers the radio up once (firmware download). */
    floss_cyw43_default_addr(&local_addr);
    floss_hci_set_addr_override(&local_addr);

#if FLOSS_PICO_SNOOP
    snoop_on = true;
    floss_snoop_set_hook(floss_snoop_log_hook);
#endif

    floss_gap_add_listener(board_gap_event, NULL);
}

void link_board_poll_driver(void) {
    async_context_poll(&async_ctx.core);
}

const floss_addr_t *link_board_addr(void) {
    return &local_addr;
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
static bool __no_inline_not_in_flash_func(bootsel_sample)(void) {
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

bool link_board_bootsel_pressed(void) {
    return bootsel_sample();
}

/* ------------------------------------------------------------------------ */
/* LED                                                                      */
/* ------------------------------------------------------------------------ */

void link_board_led_task(uint32_t now) {
    static int led = -1; /* unknown */
    if (!floss_hci_ready()) {
        return; /* the LED lives on the radio chip */
    }
    bool on;
    switch (floss_link_state()) {
    case FLOSS_LINK_STATE_CONNECTED:
        on = true;
        break;
    case FLOSS_LINK_STATE_PAIRING:
        on = ((now / LED_FAST_HALF_US) & 1u) != 0;
        break;
    case FLOSS_LINK_STATE_CONNECTING:
        on = ((now / LED_SLOW_HALF_US) & 1u) != 0;
        break;
    default:
        on = (now % LED_IDLE_PERIOD_US) < LED_IDLE_BLIP_US;
        break;
    }
    if ((int)on != led) {
        led = on;
        cyw43_gpio_set(&cyw43_state, CYW43_WL_GPIO_LED_PIN, on);
    }
}

/* ------------------------------------------------------------------------ */
/* Transport selection                                                      */
/* ------------------------------------------------------------------------ */

floss_link_transport_t link_board_transport_load(void) {
    uint8_t v = 0;
    if (floss_hal_store_read(LINK_STORE_KEY_TRANSPORT, &v, 1) == 1 && v == (uint8_t)FLOSS_LINK_LE) {
        return FLOSS_LINK_LE;
    }
    return FLOSS_LINK_CLASSIC;
}

const char *link_board_transport_name(floss_link_transport_t t) {
    return t == FLOSS_LINK_LE ? "LE" : "Classic";
}

const char *link_board_state_name(floss_link_state_t s) {
    switch (s) {
    case FLOSS_LINK_STATE_IDLE: return "idle";
    case FLOSS_LINK_STATE_PAIRING: return "pairing";
    case FLOSS_LINK_STATE_CONNECTING: return "connecting";
    case FLOSS_LINK_STATE_CONNECTED: return "connected";
    default: return "?";
    }
}

void link_board_reboot(void) {
    watchdog_reboot(0, 0, REBOOT_DELAY_MS);
    for (;;) {
        pump(); /* let the console drain until the watchdog fires */
    }
}

void link_board_switch_transport(floss_link_transport_t active, floss_link_transport_t want) {
    if (want == active) {
        printf("transport is already %s\n", link_board_transport_name(want));
        return;
    }
    uint8_t v = (uint8_t)want;
    /* Rewrites the store sector (~50 ms, interrupts off): fine, we reboot. */
    floss_err_t err = floss_hal_store_write(LINK_STORE_KEY_TRANSPORT, &v, 1);
    if (err != FLOSS_OK) {
        printf("saving the transport failed: %s\n", floss_err_str(err));
        return;
    }
    printf("transport %s saved, rebooting\n", link_board_transport_name(want));
    link_board_reboot();
}

/* ------------------------------------------------------------------------ */
/* Debug console keys                                                       */
/* ------------------------------------------------------------------------ */

bool link_board_snoop_on(void) {
    return snoop_on;
}

bool link_board_debug_key(int c) {
    switch (c) {
    case 't':
        floss_trace_dump();
        return true;
    case 'v':
        floss_log_set_level_all(FLOSS_LOG_DEBUG);
        printf("log level DEBUG\n");
        return true;
    case 'q':
        floss_log_set_level_all(FLOSS_LOG_INFO);
        printf("log level INFO\n");
        return true;
    case 'p':
        snoop_on = !snoop_on;
        floss_snoop_set_hook(snoop_on ? floss_snoop_log_hook : NULL);
        printf("snoop %s\n", snoop_on ? "on" : "off");
        return true;
    default:
        return false;
    }
}
