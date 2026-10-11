/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_rfcomm.h - RFCOMM serial cable emulation (multiplexer + DLCs) and
 * the serial port service built on it.
 *
 * ===========================================================================
 *  MODEL
 *
 *  RFCOMM runs a TS 07.10 style multiplexer over ONE L2CAP channel (PSM
 *  0x0003) per baseband link. Inside it, DLCI 0 carries multiplexer control
 *  and every other DLCI is one "virtual serial cable" (a DLC). A DLCI is
 *  (server channel << 1) | direction bit, so the 30 server channels (1..30)
 *  exist independently on both devices.
 *
 *     L2CAP chan (PSM 3) --- mux (DLCI 0: SABM/UA, PN, MSC, RPN, RLS, ...)
 *                              |-- DLC  DLCI 2  (server channel 1)
 *                              `-- DLC  DLCI 5  (peer's channel 2) ...
 *
 *  Server:  floss_rfcomm_register_server(1, &ops, user);
 *           ... on_open(dlc, 0) -> floss_rfcomm_send(dlc, ...) / on_data ...
 *  Client:  floss_rfcomm_connect(handle, peer_channel, &ops, user, &dlc);
 *
 *  FLOW CONTROL
 *  ------------
 *  Credit based flow control (CFC) is requested on every DLC and is what all
 *  modern peers use: the receiver hands out credits, one credit = one frame
 *  of at most dlc->mtu bytes. floss_rfcomm_send() never sends without a
 *  credit; when it cannot send everything it returns what it sent and
 *  on_can_send() fires once more can go.
 *
 *  Receive credits are, by default, topped up automatically once on_data()
 *  returns (data handed to the callback counts as consumed). An application
 *  that buffers data and needs back-pressure calls
 *  floss_rfcomm_set_manual_credits(dlc, true) and then grants credits itself
 *  with floss_rfcomm_grant() as its buffer drains.
 *
 *  Peers that do not support CFC fall back to the multiplexer's FCon/FCoff
 *  and the MSC FC bit, which RFCOMM honours on transmit.
 *
 *  All calls must be made from the stack context (see floss_hal.h).
 * ===========================================================================
 */
#ifndef FLOSS_RFCOMM_H
#define FLOSS_RFCOMM_H

#include "floss_types.h"
#include "floss_buf.h"

#if FLOSS_ENABLE_RFCOMM

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Configuration (override like any floss_config.h value)                   */
/* ------------------------------------------------------------------------ */

/** Max registered server channels at once. */
#ifndef FLOSS_RFCOMM_MAX_SERVERS
#define FLOSS_RFCOMM_MAX_SERVERS 2
#endif

/**
 * Automatic credit mode: how many receive credits we keep handed out to the
 * peer per DLC. Each credit lets the peer send one frame, so this bounds the
 * frames in flight towards us. Topped up when it falls to half.
 */
#ifndef FLOSS_RFCOMM_RX_CREDITS
#define FLOSS_RFCOMM_RX_CREDITS 7
#endif

/** Control frames buffered per multiplexer while L2CAP is busy. */
#ifndef FLOSS_RFCOMM_CTRL_QUEUE_LEN
#define FLOSS_RFCOMM_CTRL_QUEUE_LEN 6
#endif

/** Number of serial port service records floss_spp_init() can register. */
#ifndef FLOSS_SPP_MAX_INSTANCES
#define FLOSS_SPP_MAX_INSTANCES 1
#endif

/* ------------------------------------------------------------------------ */
/* Constants                                                                */
/* ------------------------------------------------------------------------ */

#define FLOSS_RFCOMM_CHANNEL_MIN 1
#define FLOSS_RFCOMM_CHANNEL_MAX 30

/** Default max frame payload (N1) when the peer does not negotiate one. */
#define FLOSS_RFCOMM_DEFAULT_MTU 127

/**
 * Worst-case RFCOMM framing overhead per frame: address, control, 2-byte
 * length, credit byte and FCS. dlc->mtu never exceeds L2CAP MTU minus this.
 */
#define FLOSS_RFCOMM_FRAME_OVERHEAD 6

/** Bytes floss_rfcomm_buf_alloc() reserves in front of the payload. */
#define FLOSS_RFCOMM_HEADROOM 5

/** SDP UUIDs for the protocol and the serial port service class. */
#ifndef FLOSS_UUID_RFCOMM
#define FLOSS_UUID_RFCOMM 0x0003
#endif
#ifndef FLOSS_UUID_SERIAL_PORT
#define FLOSS_UUID_SERIAL_PORT 0x1101
#endif

/** V.24 signal bits of the modem status command (MSC). */
#define FLOSS_RFCOMM_V24_FC  0x02 /**< flow control: 1 = sender cannot accept frames */
#define FLOSS_RFCOMM_V24_RTC 0x04 /**< ready to communicate (DSR/DTR) */
#define FLOSS_RFCOMM_V24_RTR 0x08 /**< ready to receive (RTS/CTS) */
#define FLOSS_RFCOMM_V24_IC  0x40 /**< incoming call (RI) */
#define FLOSS_RFCOMM_V24_DV  0x80 /**< data valid (DCD) */
/** Signals we advertise by default: RTC | RTR | DV. */
#define FLOSS_RFCOMM_V24_DEFAULT (FLOSS_RFCOMM_V24_RTC | FLOSS_RFCOMM_V24_RTR | FLOSS_RFCOMM_V24_DV)

/** on_open() status values. */
#define FLOSS_RFCOMM_STATUS_OK        0
#define FLOSS_RFCOMM_STATUS_REFUSED   1 /**< peer answered DM: no such channel / rejected */
#define FLOSS_RFCOMM_STATUS_LINK_LOST 2 /**< L2CAP channel failed or closed */
#define FLOSS_RFCOMM_STATUS_NO_MUX    3 /**< multiplexer start-up refused */

/* ------------------------------------------------------------------------ */
/* Data link connections                                                    */
/* ------------------------------------------------------------------------ */

typedef enum {
    FLOSS_RFCOMM_DLC_FREE = 0,
    FLOSS_RFCOMM_DLC_NEGOTIATED, /**< responder: PN answered, waiting for SABM */
    FLOSS_RFCOMM_DLC_WAIT_MUX,   /**< initiator: waiting for the multiplexer */
    FLOSS_RFCOMM_DLC_WAIT_PN,    /**< initiator: PN sent */
    FLOSS_RFCOMM_DLC_WAIT_UA,    /**< initiator: SABM sent */
    FLOSS_RFCOMM_DLC_OPEN,
    FLOSS_RFCOMM_DLC_WAIT_DISC,  /**< DISC sent, waiting for UA/DM */
} floss_rfcomm_dlc_state_t;

typedef struct floss_rfcomm_dlc floss_rfcomm_dlc_t;
struct floss_rfcomm_mux;

/** DLC callbacks. All optional, but a useful channel has on_data. */
typedef struct {
    /**
     * DLC open (status 0) or failed to open (FLOSS_RFCOMM_STATUS_*). After a
     * failure the dlc is freed when the callback returns.
     */
    void (*on_open)(floss_rfcomm_dlc_t *dlc, uint8_t status);
    /** Payload received (at most dlc->mtu bytes). data valid only during the call. */
    void (*on_data)(floss_rfcomm_dlc_t *dlc, const uint8_t *data, uint16_t len);
    /** DLC closed (either side, or link loss). Freed after return. Only after on_open(0). */
    void (*on_close)(floss_rfcomm_dlc_t *dlc);
    /** floss_rfcomm_send() was short or BUSY and more can be sent now. */
    void (*on_can_send)(floss_rfcomm_dlc_t *dlc);
} floss_rfcomm_ops_t;

/**
 * One DLC. Fields are readable by the application; only the stack writes them.
 */
struct floss_rfcomm_dlc {
    floss_rfcomm_dlc_state_t state;
    uint16_t handle;        /**< ACL link */
    uint8_t dlci;           /**< (channel << 1) | direction */
    uint8_t channel;        /**< server channel (1..30) */
    uint16_t mtu;           /**< N1: max payload per frame, both directions */
    uint16_t tx_credits;    /**< frames we may still send (CFC) */
    uint16_t rx_credits;    /**< frames the peer may still send us (CFC) */
    uint16_t grant_pending; /**< manual mode: credits granted, not yet sent */
    uint8_t priority;       /**< PN priority (0..63) */
    uint8_t local_v24;      /**< our MSC signals (FLOSS_RFCOMM_V24_*) */
    uint8_t remote_v24;     /**< last MSC signals from the peer */
    uint8_t port[5];        /**< RPN settings: baud, format, flow, XON, XOFF */
    bool cfc;               /**< credit based flow control active */
    bool initiator;         /**< we opened this DLC */
    bool manual_credits;    /**< application grants rx credits */
    bool remote_fc;         /**< non-CFC: peer set FC in MSC (stop sending) */
    bool want_send;         /**< internal: on_can_send owed */
    bool open_reported;     /**< internal: on_open(0) delivered */
    bool closing;           /**< internal: final callback running, slot not reusable yet */
    struct floss_rfcomm_mux *mux; /**< internal */
    const floss_rfcomm_ops_t *ops;
    void *user;
};

/**
 * Register a server channel (1..30). Incoming DLCs for it are accepted and
 * reported via ops->on_open(dlc, 0); every DLC inherits ops and user.
 * Call after floss_init() (which resets the registry).
 */
floss_err_t floss_rfcomm_register_server(uint8_t channel, const floss_rfcomm_ops_t *ops, void *user);
floss_err_t floss_rfcomm_unregister_server(uint8_t channel);

/**
 * Open a DLC to `channel` on the peer at link `handle`. Starts the L2CAP
 * channel and the multiplexer if needed. *out is valid immediately; the
 * result arrives as ops->on_open().
 */
floss_err_t floss_rfcomm_connect(uint16_t handle, uint8_t channel, const floss_rfcomm_ops_t *ops, void *user,
                                 floss_rfcomm_dlc_t **out);

/**
 * Close a DLC (DISC). on_close fires when the peer confirms. A DLC that has
 * not opened yet is cancelled silently.
 */
floss_err_t floss_rfcomm_disconnect(floss_rfcomm_dlc_t *dlc);

/**
 * Copying send. Splits data into frames of at most dlc->mtu bytes, one
 * credit each.
 *
 * @return number of bytes accepted (1..len; may be short when credits or
 *         buffers run out), FLOSS_ERR_BUSY if nothing could be sent (then
 *         on_can_send fires later), or another negative error.
 */
int floss_rfcomm_send(floss_rfcomm_dlc_t *dlc, const uint8_t *data, uint16_t len);

/** True if floss_rfcomm_send() would accept at least one frame right now. */
bool floss_rfcomm_can_send_now(const floss_rfcomm_dlc_t *dlc);

/**
 * Zero-copy send, step 1: a pool buffer with room for the RFCOMM header in
 * front. Write up to dlc->mtu payload bytes at buf->data (floss_buf_put()).
 */
floss_buf_t *floss_rfcomm_buf_alloc(void);

/**
 * Zero-copy send, step 2: send one frame. Takes ownership of buf in all
 * cases (like floss_l2cap_send_buf()); check floss_rfcomm_can_send_now()
 * first, since a BUSY result means the payload was dropped.
 */
floss_err_t floss_rfcomm_send_buf(floss_rfcomm_dlc_t *dlc, floss_buf_t *buf);

/**
 * Credit mode. Automatic (default): rx credits are replenished after each
 * on_data(). Manual: only floss_rfcomm_grant() hands out credits.
 */
void floss_rfcomm_set_manual_credits(floss_rfcomm_dlc_t *dlc, bool manual);

/**
 * Give the peer `n` more frames' worth of credit (manual mode, or extra
 * credits in automatic mode). Sent immediately, or piggybacked on data.
 * No-op on DLCs without credit based flow control.
 */
floss_err_t floss_rfcomm_grant(floss_rfcomm_dlc_t *dlc, uint8_t n);

/** Update and send our V.24 signals (MSC), e.g. drop DV to signal "no data". */
floss_err_t floss_rfcomm_set_signals(floss_rfcomm_dlc_t *dlc, uint8_t v24);

/* ------------------------------------------------------------------------ */
/* Serial port service                                                      */
/* ------------------------------------------------------------------------ */

/**
 * Register server channel `channel` with `ops` and publish a serial port
 * service record (class 0x1101) named `service_name` in SDP. The name must
 * stay valid only during the call (the record keeps a copy).
 */
floss_err_t floss_spp_init(uint8_t channel, const char *service_name, const floss_rfcomm_ops_t *ops, void *user);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_ENABLE_RFCOMM */

#endif /* FLOSS_RFCOMM_H */
