/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_transport.h - HCI transport driver interface.
 *
 * ===========================================================================
 *  A transport moves whole HCI packets between the host stack and a radio
 *  controller. It knows NOTHING about HCI semantics beyond the packet type.
 *
 *    +-----------------+      send(type, pkt, len)      +----------------+
 *    |   FLOSS HCI     | -----------------------------> |   transport    | --> wire
 *    |                 | <----------------------------- |   (driver)     | <-- wire
 *    +-----------------+   deliver(type, pkt, len)      +----------------+
 *                          (via floss_transport_t.rx)
 *
 *  Shipped drivers:
 *    transport/h4/     H4 framing over ANY byte stream (UART, USB-CDC, TCP,
 *                      pipe). You only implement floss_stream_t.
 *    transport/cyw43/  Infineon CYW43xxx shared-bus (Raspberry Pi Pico W).
 *    transport/sim/    In-memory virtual controller for tests.
 *
 *  Writing a new one (USB bulk endpoints, SPI, SDIO, a mailbox to a radio
 *  core, ...): fill in floss_transport_ops_t. See docs/PORTING.md.
 * ===========================================================================
 */
#ifndef FLOSS_TRANSPORT_H
#define FLOSS_TRANSPORT_H

#include "floss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** HCI packet types (the H4 indicator byte values, used by all transports). */
typedef enum {
    FLOSS_PKT_CMD = 0x01,
    FLOSS_PKT_ACL = 0x02,
    FLOSS_PKT_SCO = 0x03,
    FLOSS_PKT_EVT = 0x04,
    FLOSS_PKT_ISO = 0x05,
} floss_pkt_type_t;

typedef struct floss_transport floss_transport_t;

/** Called by the driver for every complete packet received from the controller. */
typedef void (*floss_transport_rx_fn)(void *user, uint8_t type, const uint8_t *pkt, uint16_t len);

/** Driver operations. All are called from the stack context only. */
typedef struct floss_transport_ops {
    /** Short name for logs, e.g. "h4", "cyw43". */
    const char *name;

    /**
     * Bytes the driver may write *in front of* pkt during send() (e.g. the H4
     * type byte, or the 4-byte CYW43 bus header). Must be
     * <= FLOSS_TRANSPORT_MAX_HEADROOM. This is what makes TX zero-copy.
     */
    uint8_t headroom;

    /** Bring the link up (power, firmware download, ...). Blocking is OK here. */
    floss_err_t (*open)(floss_transport_t *t);

    /** Shut the link down. */
    void (*close)(floss_transport_t *t);

    /**
     * Send one HCI packet (without any H4 type byte; pkt starts at the HCI
     * header). pkt[-headroom .. -1] is writable scratch space for the driver.
     *
     * Contract: synchronous ownership. When this returns FLOSS_OK the stack
     * may immediately reuse the buffer, so an asynchronous driver (DMA) must
     * copy or finish before returning. Return FLOSS_ERR_BUSY if the driver
     * cannot accept a packet right now; the stack will retry on a later poll.
     */
    floss_err_t (*send)(floss_transport_t *t, uint8_t type, uint8_t *pkt, uint16_t len);

    /**
     * Pump the driver: read whatever the controller has produced and deliver
     * each complete packet with floss_transport_deliver(). Called on every
     * floss_poll(). Must not block. Return number of packets delivered.
     */
    int (*poll)(floss_transport_t *t);
} floss_transport_ops_t;

/**
 * A transport instance. The driver owns `ctx`; the stack fills `rx`/`rx_user`
 * before calling open().
 */
struct floss_transport {
    const floss_transport_ops_t *ops;
    void *ctx;                    /**< driver private state */
    floss_transport_rx_fn rx;     /**< set by the stack */
    void *rx_user;                /**< set by the stack */
};

/** Helper for drivers: hand a received packet to the stack. */
static inline void floss_transport_deliver(floss_transport_t *t, uint8_t type, const uint8_t *pkt, uint16_t len) {
    if (t->rx) {
        t->rx(t->rx_user, type, pkt, len);
    }
}

/* ------------------------------------------------------------------------ */
/* Byte streams (for the H4 driver)                                          */
/* ------------------------------------------------------------------------ */

/**
 * A non-blocking byte pipe: a UART, a USB CDC port, a TCP socket, a
 * named pipe... The H4 driver (transport/h4) turns any of these into an
 * HCI transport. Implementing a stream is usually ~30 lines.
 */
typedef struct floss_stream {
    void *ctx;
    /** Write up to len bytes. Return bytes written (may be < len) or negative error. */
    int (*write)(void *ctx, const uint8_t *data, uint16_t len);
    /** Read up to max bytes that are available NOW. Return count (0 = none) or negative error. */
    int (*read)(void *ctx, uint8_t *buf, uint16_t max);
} floss_stream_t;

/* ------------------------------------------------------------------------ */
/* Chipset quirks                                                           */
/* ------------------------------------------------------------------------ */

/**
 * Controller-specific (vendor) knowledge that sits between the generic HCI
 * init sequence and the radio. All fields optional (0/NULL).
 */
typedef struct floss_chipset {
    const char *name;

    /** Vendor opcode that sets the public device address (6 bytes, wire order). 0 = unsupported. */
    uint16_t set_addr_opcode;

    /**
     * Optional extra commands sent right after HCI_Reset. A flat byte script:
     * repeated [opcode_lo, opcode_hi, param_len, params...].
     */
    const uint8_t *init_script;
    uint16_t init_script_len;
} floss_chipset_t;

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_TRANSPORT_H */
