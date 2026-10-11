/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_serial.h - Non-blocking serial port as a floss_stream_t (desktop).
 *
 * Lets a desktop process talk H4 to a UART controller (a USB-serial HCI
 * dongle, a dev board running an HCI-over-UART firmware, ...):
 *
 *     static floss_serial_t port;
 *     static floss_h4_t h4;
 *     floss_serial_config_t cfg = { .baud = 115200, .rtscts = true };
 *     if (floss_serial_open(&port, "COM5", &cfg) == FLOSS_OK) {   // or "/dev/ttyUSB0"
 *         floss_init(&(floss_init_t){ .transport = floss_h4_init(&h4, floss_serial_stream(&port)) });
 *     }
 *
 * Reads return immediately with whatever bytes are available (0 = none),
 * which is what the H4 driver's poll expects. Raw 8N1, no software flow
 * control; hardware RTS/CTS is strongly recommended above 115200 baud.
 */
#ifndef FLOSS_SERIAL_H
#define FLOSS_SERIAL_H

#include "floss/floss_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t baud;  /**< e.g. 115200, 921600, 3000000 */
    bool rtscts;    /**< hardware flow control */
} floss_serial_config_t;

typedef struct {
#ifdef _WIN32
    void *handle;   /**< HANDLE */
#else
    int fd;
#endif
    bool open;
    floss_stream_t stream;
} floss_serial_t;

/**
 * Open and configure a port. Windows names: "COM3" (the "\\.\" prefix needed
 * for COM10+ is added automatically). POSIX: a device path.
 */
floss_err_t floss_serial_open(floss_serial_t *s, const char *port, const floss_serial_config_t *cfg);

/** Change the baud rate of an open port (e.g. after a vendor "set baud" HCI command). */
floss_err_t floss_serial_set_baud(floss_serial_t *s, uint32_t baud);

/** Close the port. Safe to call on a closed port. */
void floss_serial_close(floss_serial_t *s);

/** The stream to hand to floss_h4_init(). Valid while the port is open. */
const floss_stream_t *floss_serial_stream(floss_serial_t *s);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_SERIAL_H */
