/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_h4.h - H4 (UART-style) HCI transport over any floss_stream_t.
 *
 * Framing: [type byte][HCI packet]. RX is a byte-at-a-time state machine
 * that tolerates arbitrary chunking, so the stream may be a UART FIFO, a
 * USB CDC endpoint, a TCP socket or a pipe.
 *
 *     static floss_h4_t h4;
 *     floss_transport_t *t = floss_h4_init(&h4, &my_uart_stream);
 */
#ifndef FLOSS_H4_H
#define FLOSS_H4_H

#include "floss/floss_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/** H4 driver state (opaque to users; sized here to avoid allocation). */
typedef struct {
    floss_transport_t transport;
    const floss_stream_t *stream;
    uint8_t state;          /* RX state machine */
    uint8_t type;           /* current RX packet type */
    uint16_t need;          /* bytes still needed for current RX stage */
    uint16_t have;          /* bytes collected in rx[] */
    uint8_t rx[4 + FLOSS_BUF_SIZE]; /* current RX packet (header + payload) */
    uint32_t resyncs;       /* framing errors recovered */
} floss_h4_t;

/** Initialise and return the transport handle to pass to floss_init(). */
floss_transport_t *floss_h4_init(floss_h4_t *h4, const floss_stream_t *stream);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_H4_H */
