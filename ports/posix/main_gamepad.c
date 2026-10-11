/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * main_gamepad.c - Desktop host for the gamepad example over an H4 UART.
 *
 * Overview
 * --------
 * Runs examples/gamepad on a PC, talking HCI over a serial port to any UART
 * controller (a USB HCI dongle with a UART bridge, a dev board running an
 * HCI-UART firmware, ...). Useful to develop and debug the upper layers with
 * a real radio but desktop tools (debugger, sanitizers, fast rebuilds).
 *
 *   serial port --floss_serial--> floss_stream_t --floss_h4--> transport
 *        --> floss_init() --> gamepad_app_init() --> floss_start()
 *        --> loop { floss_poll(); gamepad_app_task(); sleep <= 1 ms }
 *
 * The loop sleeps for whatever floss_poll() reports until the next timer,
 * capped at 1 ms because UART input does not wake us (no host-wake line).
 *
 * Keys while running: s = stats dump, t = trace dump, f = freeze/unfreeze
 * trace, q = quit. (On POSIX terminals press Enter after the key.)
 */
#ifndef _WIN32
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#endif

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <conio.h>
#else
#include <sys/select.h>
#include <unistd.h>
#endif

#include "floss/floss.h"
#include "floss_h4.h"
#include "floss_posix_hal.h"
#include "floss_serial.h"
#include "gamepad_app.h"

/** Longest we sleep between polls: bounds UART RX latency (no RX wake-up). */
#define MAX_IDLE_US 1000u

static volatile sig_atomic_t s_quit;

static void on_signal(int sig)
{
    (void)sig;
    s_quit = 1;
}

static void usage(const char *prog)
{
    fprintf(stderr,
            "usage: %s [options] PORT\n"
            "  PORT                 serial port (COM5, /dev/ttyUSB0, ...)\n"
            "  -b, --baud N         baud rate (default 115200)\n"
            "      --rtscts         enable RTS/CTS hardware flow control\n"
            "      --store FILE     bond storage file (default floss_store.bin)\n"
            "      --log FILE       also append the log to FILE\n"
            "      --snoop          log every HCI packet as SNOOP lines\n"
            "                       (convert with tools/floss_snoop.py)\n"
            "      --addr ADDR      override the controller address (needs chipset support)\n"
            "  -v, --verbose        debug-level logs (if compiled in)\n"
            "keys while running: s=stats  t=trace  f=freeze trace  q=quit\n",
            prog);
}

/** Non-blocking single key from the console, or -1. */
static int poll_key(void)
{
#ifdef _WIN32
    return _kbhit() ? _getch() : -1;
#else
    fd_set set;
    struct timeval tv = {0, 0};
    unsigned char c;

    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    if (select(STDIN_FILENO + 1, &set, NULL, NULL, &tv) > 0 && read(STDIN_FILENO, &c, 1) == 1) {
        return c;
    }
    return -1;
#endif
}

static void handle_key(int key)
{
    static bool s_frozen;

    switch (key) {
    case 's':
        floss_stats_dump();
        break;
    case 't':
        floss_trace_dump();
        break;
    case 'f':
        s_frozen = !s_frozen;
        floss_trace_freeze(s_frozen);
        FLOSS_LOGI(APP, "trace %s", s_frozen ? "frozen" : "running");
        break;
    case 'q':
        s_quit = 1;
        break;
    default:
        break;
    }
}

int main(int argc, char **argv)
{
    static floss_serial_t serial;
    static floss_h4_t h4;
    floss_serial_config_t cfg = {.baud = 115200, .rtscts = false};
    const char *port = NULL;
    const char *addr_str = NULL;
    bool snoop = false, verbose = false;
    floss_transport_t *transport;
    floss_err_t err;

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        bool has_next = (i + 1 < argc);

        if ((!strcmp(a, "-b") || !strcmp(a, "--baud")) && has_next) {
            cfg.baud = (uint32_t)strtoul(argv[++i], NULL, 10);
        } else if (!strcmp(a, "--rtscts")) {
            cfg.rtscts = true;
        } else if (!strcmp(a, "--store") && has_next) {
            floss_posix_set_store_path(argv[++i]);
        } else if (!strcmp(a, "--log") && has_next) {
            if (floss_posix_set_log_file(argv[++i]) != FLOSS_OK) {
                fprintf(stderr, "cannot open log file %s\n", argv[i]);
                return 1;
            }
        } else if (!strcmp(a, "--snoop")) {
            snoop = true;
        } else if (!strcmp(a, "--addr") && has_next) {
            addr_str = argv[++i];
        } else if (!strcmp(a, "-v") || !strcmp(a, "--verbose")) {
            verbose = true;
        } else if (!strcmp(a, "-h") || !strcmp(a, "--help")) {
            usage(argv[0]);
            return 0;
        } else if (a[0] != '-' && port == NULL) {
            port = a;
        } else {
            usage(argv[0]);
            return 2;
        }
    }
    if (port == NULL || cfg.baud == 0) {
        usage(argv[0]);
        return 2;
    }

    err = floss_serial_open(&serial, port, &cfg);
    if (err != FLOSS_OK) {
        fprintf(stderr, "cannot open %s at %lu baud: %s\n", port, (unsigned long)cfg.baud, floss_err_str(err));
        return 1;
    }

    floss_log_set_level_all(verbose ? FLOSS_LOG_DEBUG : FLOSS_LOG_INFO);
    if (snoop) {
        floss_snoop_set_hook(floss_snoop_log_hook);
    }

    transport = floss_h4_init(&h4, floss_serial_stream(&serial));
    err = floss_init(&(floss_init_t){.transport = transport, .chipset = NULL});
    if (err != FLOSS_OK) {
        fprintf(stderr, "floss_init failed: %s\n", floss_err_str(err));
        floss_serial_close(&serial);
        return 1;
    }
    if (addr_str != NULL) {
        floss_addr_t addr;
        if (!floss_addr_from_str(addr_str, &addr)) {
            fprintf(stderr, "bad address '%s' (want AA:BB:CC:DD:EE:FF)\n", addr_str);
            floss_serial_close(&serial);
            return 2;
        }
        floss_hci_set_addr_override(&addr);
    }

    err = gamepad_app_init();
    if (err != FLOSS_OK) {
        fprintf(stderr, "gamepad_app_init failed: %s\n", floss_err_str(err));
        floss_serial_close(&serial);
        return 1;
    }
    err = floss_start();
    if (err != FLOSS_OK) {
        fprintf(stderr, "floss_start failed: %s\n", floss_err_str(err));
        floss_serial_close(&serial);
        return 1;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    FLOSS_LOGI(APP, "running on %s @ %lu baud%s (s=stats t=trace q=quit)", port, (unsigned long)cfg.baud,
               cfg.rtscts ? " rtscts" : "");

    while (!s_quit) {
        uint32_t idle = floss_poll();
        int key;

        gamepad_app_task();
        while ((key = poll_key()) >= 0) {
            handle_key(key);
        }
        if (idle > 0) {
            floss_posix_sleep_us(idle < MAX_IDLE_US ? idle : MAX_IDLE_US);
        }
    }

    floss_stats_dump();
    floss_serial_close(&serial);
    return 0;
}
