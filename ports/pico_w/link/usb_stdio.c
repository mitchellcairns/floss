/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * usb_stdio.c - TinyUSB device bring-up and a stdio driver over its CDC.
 *
 * Design notes:
 *
 *  TASK      tud_task() runs from the main loop (usb_stdio_task) and, only
 *            when the console FIFO is full, from printf itself. A flag
 *            guards against re-entering it: TinyUSB callbacks must never
 *            print (and none of the dongle's do), and if one did, its
 *            output would simply not wait for room.
 *
 *  OUTPUT    Dropped while no terminal is attached (DTR low). Otherwise
 *            written into TinyUSB's TX FIFO (CFG_TUD_CDC_TX_BUFSIZE) and
 *            flushed; the host drains it at bulk speed. When the FIFO is
 *            full, printf runs TinyUSB until there is room, but at most
 *            USB_STDIO_STALL_US since the host last took data; after that
 *            output is dropped (counted) without waiting until the host
 *            reads again. So a terminal that is open but not read stalls
 *            the radio at most once by a couple of milliseconds.
 *
 *  INPUT     Straight from the CDC RX FIFO (getchar_timeout_us(0)).
 *
 *  BOOTSEL   SET_LINE_CODING at 1200 baud reboots into the ROM bootloader,
 *            the convention of the SDK's USB stdio and Arduino boards.
 */
#include "pico/stdlib.h"
#include "pico/stdio/driver.h"
#include "pico/bootrom.h"
#include "tusb.h"

#include "usb_stdio.h"

#define RESET_MAGIC_BAUD 1200u

static bool in_task;           /* inside tud_task(): do not re-enter */
static bool stalled;           /* host stopped taking data: drop without waiting */
static uint32_t last_progress; /* time_us_32() when output last went into the FIFO */
static uint32_t dropped;

void usb_stdio_task(void) {
    if (in_task) {
        return;
    }
    in_task = true;
    tud_task();
    in_task = false;
}

bool usb_stdio_connected(void) {
    return tud_cdc_connected();
}

uint32_t usb_stdio_dropped(void) {
    return dropped;
}

/* ------------------------------------------------------------------------ */
/* stdio driver                                                             */
/* ------------------------------------------------------------------------ */

static void usb_out_chars(const char *buf, int len) {
    if (len <= 0 || !tud_cdc_connected()) {
        return; /* no terminal: nobody to tell */
    }
    uint32_t n = (uint32_t)len;
    while (n) {
        uint32_t room = tud_cdc_write_available();
        if (room) {
            uint32_t k = tud_cdc_write(buf, n < room ? n : room);
            buf += k;
            n -= k;
            tud_cdc_write_flush();
            last_progress = time_us_32();
            stalled = false;
            continue;
        }
        /* FIFO full: let USB move data, but never for long and never re-entrantly. */
        if (in_task || stalled || time_us_32() - last_progress > USB_STDIO_STALL_US) {
            stalled = true;
            break;
        }
        usb_stdio_task();
        tud_cdc_write_flush();
        if (!tud_cdc_connected()) {
            break;
        }
    }
    dropped += n;
}

static void usb_out_flush(void) {
    tud_cdc_write_flush();
}

static int usb_in_chars(char *buf, int len) {
    if (len <= 0 || !tud_cdc_available()) {
        return PICO_ERROR_NO_DATA;
    }
    uint32_t n = tud_cdc_read(buf, (uint32_t)len);
    return n ? (int)n : PICO_ERROR_NO_DATA;
}

static stdio_driver_t usb_stdio_driver = {
    .out_chars = usb_out_chars,
    .out_flush = usb_out_flush,
    .in_chars = usb_in_chars,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = PICO_STDIO_DEFAULT_CRLF,
#endif
};

/* ------------------------------------------------------------------------ */
/* TinyUSB callbacks                                                        */
/* ------------------------------------------------------------------------ */

/* Host changed the serial settings: 1200 baud = "reboot to BOOTSEL". */
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *coding) {
    (void)itf;
    if (coding->bit_rate == RESET_MAGIC_BAUD) {
        reset_usb_boot(0, 0); /* no activity LED, both boot interfaces enabled */
    }
}

/* ------------------------------------------------------------------------ */
/* Init                                                                     */
/* ------------------------------------------------------------------------ */

void usb_stdio_init(void) {
    const tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO,
    };
    tusb_init(0, &dev_init);
    stdio_set_driver_enabled(&usb_stdio_driver, true);
}
