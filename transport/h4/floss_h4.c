/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_h4.c - H4 HCI transport over any non-blocking byte stream.
 *
 * Overview
 * --------
 * H4 framing is one packet-type byte followed by the HCI packet. The packet
 * length is only known once the type-specific header has arrived:
 *
 *     type   header                      payload length
 *     0x04   EVT  code(1) len(1)         len
 *     0x02   ACL  handle(2) len(2)       len
 *     0x03   SCO  handle(2) len(1)       len
 *     0x05   ISO  handle(2) len(2)       len & 0x3FFF
 *
 * RX is a small state machine (TYPE -> HEADER -> PAYLOAD -> deliver). We ask
 * the stream for at most the bytes the current stage still needs, reading
 * the header and payload straight into h4->rx. That means:
 *   - no intermediate copy and no leftover bytes to remember between polls,
 *   - any chunking the stream produces (1 byte at a time, or whole bursts)
 *     works identically,
 *   - the per-poll packet cap is exact, which bounds the time one
 *     floss_poll() can spend here (latency for everything else).
 *
 * Framing errors: an unknown type byte is skipped (counted in resyncs) until
 * a valid type appears. A packet too large for our buffer is consumed and
 * discarded using its declared length, which keeps the framing intact.
 *
 * TX writes the type byte into the headroom in front of the packet
 * (pkt[-1]) and pushes the whole frame with one write loop: zero-copy.
 */
#include "floss_h4.h"
#include "floss/floss_debug.h"

enum {
    H4_RX_TYPE = 0,
    H4_RX_HEADER,
    H4_RX_PAYLOAD,
    H4_RX_DISCARD,
};

/** Max packets delivered by one poll (bounds poll duration). */
#define H4_MAX_PACKETS_PER_POLL 8
/** Max stream reads per poll (bounds work when the stream spews garbage). */
#define H4_MAX_READS_PER_POLL 64
/** Zero-byte writes tolerated before reporting BUSY, when nothing was written yet. */
#define H4_TX_IDLE_RETRIES 64
/** Once a frame is partly written we must finish it; give up only after this long. */
#define H4_TX_STALL_US 100000u

/** Header length for a packet type, 0 if the type is not valid from a controller. */
static uint8_t h4_header_len(uint8_t type) {
    switch (type) {
    case FLOSS_PKT_EVT:
        return 2;
    case FLOSS_PKT_ACL:
        return 4;
    case FLOSS_PKT_SCO:
        return 3;
    case FLOSS_PKT_ISO:
        return 4;
    default:
        return 0;
    }
}

static uint16_t h4_payload_len(uint8_t type, const uint8_t *hdr) {
    switch (type) {
    case FLOSS_PKT_EVT:
        return hdr[1];
    case FLOSS_PKT_ACL:
        return floss_get_le16(hdr + 2);
    case FLOSS_PKT_SCO:
        return hdr[2];
    case FLOSS_PKT_ISO:
        return floss_get_le16(hdr + 2) & 0x3FFF;
    default:
        return 0;
    }
}

static void h4_rx_reset(floss_h4_t *h4) {
    h4->state = H4_RX_TYPE;
    h4->need = 1;
    h4->have = 0;
}

static void h4_resync(floss_h4_t *h4, const char *what, uint16_t value) {
    h4->resyncs++;
    /* Log on the 1st, 2nd, 4th, 8th... error so a noisy line cannot flood the log. */
    if ((h4->resyncs & (h4->resyncs - 1)) == 0) {
        FLOSS_LOGW(TRANSPORT, "h4: %s 0x%04x, resyncing (%u framing errors)", what, value, (unsigned)h4->resyncs);
    }
}

/**
 * Advance the state machine by n bytes that were just read into the place
 * the current state asked for. Returns 1 if a packet was delivered.
 */
static int h4_rx_advance(floss_h4_t *h4, const uint8_t *data, uint16_t n) {
    switch (h4->state) {
    case H4_RX_TYPE: {
        uint8_t hl = h4_header_len(data[0]);
        if (hl == 0) {
            h4_resync(h4, "unknown packet type", data[0]);
            return 0;
        }
        h4->type = data[0];
        h4->state = H4_RX_HEADER;
        h4->need = hl;
        h4->have = 0;
        return 0;
    }
    case H4_RX_HEADER: {
        h4->have = (uint16_t)(h4->have + n);
        h4->need = (uint16_t)(h4->need - n);
        if (h4->need) {
            return 0;
        }
        uint16_t plen = h4_payload_len(h4->type, h4->rx);
        if ((uint32_t)h4->have + plen > sizeof(h4->rx)) {
            h4_resync(h4, "oversized packet, length", plen);
            h4->state = H4_RX_DISCARD;
            h4->need = plen;
            return 0;
        }
        if (plen) {
            h4->state = H4_RX_PAYLOAD;
            h4->need = plen;
            return 0;
        }
        break; /* header-only packet: deliver below */
    }
    case H4_RX_PAYLOAD:
        h4->have = (uint16_t)(h4->have + n);
        h4->need = (uint16_t)(h4->need - n);
        if (h4->need) {
            return 0;
        }
        break;
    case H4_RX_DISCARD:
    default:
        h4->need = (uint16_t)(h4->need - n);
        if (h4->need == 0) {
            h4_rx_reset(h4);
        }
        return 0;
    }

    uint8_t type = h4->type;
    uint16_t len = h4->have;
    h4_rx_reset(h4);
    floss_transport_deliver(&h4->transport, type, h4->rx, len);
    return 1;
}

static int h4_poll(floss_transport_t *t) {
    floss_h4_t *h4 = (floss_h4_t *)t->ctx;
    const floss_stream_t *s = h4->stream;
    uint8_t scratch[64]; /* type byte, or bytes being discarded */
    int delivered = 0;

    for (int reads = 0; reads < H4_MAX_READS_PER_POLL && delivered < H4_MAX_PACKETS_PER_POLL; reads++) {
        uint8_t *dst;
        uint16_t want;
        switch (h4->state) {
        case H4_RX_HEADER:
        case H4_RX_PAYLOAD:
            dst = h4->rx + h4->have;
            want = h4->need;
            break;
        case H4_RX_DISCARD:
            dst = scratch;
            want = (uint16_t)FLOSS_MIN(h4->need, sizeof(scratch));
            break;
        case H4_RX_TYPE:
        default:
            dst = scratch;
            want = 1;
            break;
        }

        int n = s->read(s->ctx, dst, want);
        if (n < 0) {
            FLOSS_STAT_INC(transport_errors);
            FLOSS_LOGE(TRANSPORT, "h4: stream read error %d", n);
            break;
        }
        if (n == 0) {
            break; /* nothing more right now */
        }
        if (n > want) {
            n = want; /* defensive: a stream must never return more than asked */
        }
        delivered += h4_rx_advance(h4, dst, (uint16_t)n);
    }
    return delivered;
}

/**
 * Write [type][packet]. A frame must never be left half-written: the
 * controller would misparse everything after it. So we may refuse a packet
 * (BUSY) only before its first byte went out; after that we keep pushing.
 */
static floss_err_t h4_send(floss_transport_t *t, uint8_t type, uint8_t *pkt, uint16_t len) {
    floss_h4_t *h4 = (floss_h4_t *)t->ctx;
    const floss_stream_t *s = h4->stream;
    uint8_t *frame = pkt - 1;
    uint32_t total = (uint32_t)len + 1;
    uint32_t written = 0;
    uint32_t idle = 0;
    uint32_t stall_start = 0;

    frame[0] = type;
    while (written < total) {
        uint16_t chunk = (uint16_t)FLOSS_MIN(total - written, 0xFFFFu);
        int n = s->write(s->ctx, frame + written, chunk);
        if (n < 0) {
            FLOSS_LOGE(TRANSPORT, "h4: stream write error %d after %u bytes", n, (unsigned)written);
            return FLOSS_ERR_IO;
        }
        if (n == 0) {
            if (written == 0) {
                if (++idle >= H4_TX_IDLE_RETRIES) {
                    return FLOSS_ERR_BUSY;
                }
                continue;
            }
            uint32_t now = floss_hal_time_us();
            if (idle++ == 0) {
                stall_start = now;
            } else if ((uint32_t)(now - stall_start) > H4_TX_STALL_US) {
                FLOSS_LOGE(TRANSPORT, "h4: stream stalled mid-frame (%u/%u bytes), link desynchronised",
                           (unsigned)written, (unsigned)total);
                return FLOSS_ERR_IO;
            }
            continue;
        }
        if ((uint32_t)n > chunk) {
            n = chunk;
        }
        written += (uint32_t)n;
        idle = 0;
    }
    return FLOSS_OK;
}

static floss_err_t h4_open(floss_transport_t *t) {
    h4_rx_reset((floss_h4_t *)t->ctx);
    return FLOSS_OK;
}

static void h4_close(floss_transport_t *t) {
    h4_rx_reset((floss_h4_t *)t->ctx);
}

static const floss_transport_ops_t k_h4_ops = {
    .name = "h4",
    .headroom = 1,
    .open = h4_open,
    .close = h4_close,
    .send = h4_send,
    .poll = h4_poll,
};

floss_transport_t *floss_h4_init(floss_h4_t *h4, const floss_stream_t *stream) {
    memset(h4, 0, sizeof(*h4));
    h4->stream = stream;
    h4->transport.ops = &k_h4_ops;
    h4->transport.ctx = h4;
    h4_rx_reset(h4);
    return &h4->transport;
}
