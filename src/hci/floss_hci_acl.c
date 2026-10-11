/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_hci_acl.c - ACL data: controller buffer credits, per-link TX queues,
 * fragmentation and RX reassembly.
 *
 * Overview
 * --------
 * Credits. The controller has a fixed number of ACL buffers (Read Buffer
 * Size; LE may have a separate pool from LE Read Buffer Size). Each ACL
 * packet we hand over consumes one; Number Of Completed Packets gives them
 * back. Sending without a credit overflows the controller, so data waits in
 * a per-link FIFO (floss_conn_t.txq) until credits return. The time between
 * hand-over and completion is recorded per packet: it is the best available
 * measure of air time + retransmissions + sniff latency (stat acl_ack_latency).
 *
 * TX path (zero-copy). Upper layers give us one L2CAP PDU per buffer with
 * buf->data at the L2CAP header. Each fragment of at most the controller's
 * ACL MTU gets a 4-byte ACL header written directly in front of it:
 *
 *    first:        [ACL hdr][L2CAP hdr | payload ............]
 *    continuation:                   [ACL hdr][.. rest of payload]
 *
 * For the first fragment the bytes in front are free headroom. For later
 * fragments they are the tail of the previous (already sent) fragment, so
 * we save them, write the header, send, and restore them. After a fragment
 * is sent the buffer is advanced (floss_buf_pull) and marked CONT, so the
 * queue head always describes exactly what remains to be sent.
 *
 * Fairness. floss_hci_acl_flush() walks the links round-robin, one fragment
 * per link per pass, so one link with a large PDU cannot starve another.
 * PRIORITY buffers (HID input) jump to the front of their link's queue, but
 * never into the middle of a PDU whose first fragment is already out.
 *
 * RX path. A start fragment that holds the whole L2CAP PDU (the common case
 * for HID/ATT) is delivered straight from the transport buffer with no copy.
 * Otherwise fragments are collected in a pool buffer until the length in the
 * L2CAP header is reached.
 */
#include "floss_hci_priv.h"

#define G floss_hci_g

/* ------------------------------------------------------------------------ */
/* Credit pools                                                             */
/* ------------------------------------------------------------------------ */

/** LE links use the dedicated LE pool when the controller has one, else the shared pool. */
static bool uses_le_pool(const floss_conn_t *c) {
    return c->le && G.info.le_acl_total != 0;
}

static uint16_t *pool_credits(const floss_conn_t *c) {
    return uses_le_pool(c) ? &G.le_credits : &G.acl_credits;
}

static uint16_t pool_total(const floss_conn_t *c) {
    return uses_le_pool(c) ? G.info.le_acl_total : G.info.acl_total;
}

static uint16_t link_mtu(const floss_conn_t *c) {
    return uses_le_pool(c) ? G.info.le_acl_mtu : G.info.acl_mtu;
}

/** Return n credits to c's pool, never exceeding what the controller owns. */
static void pool_give(const floss_conn_t *c, uint16_t n) {
    uint16_t *cr = pool_credits(c);
    uint32_t v = (uint32_t)*cr + n;
    uint16_t total = pool_total(c);
    *cr = (uint16_t)(v > total ? total : v);
}

void floss_hci_acl_reset_credits(void) {
    G.acl_credits = G.info.acl_total;
    G.le_credits = G.info.le_acl_total;
    G.acl_stalled = false;
}

uint16_t floss_hci_acl_credits(void) {
    return G.acl_credits;
}

/* ------------------------------------------------------------------------ */
/* In-flight timestamps                                                     */
/* ------------------------------------------------------------------------ */

/*
 * conn->inflight counts every packet the controller holds; inflight_t[] is a
 * ring with timestamps of the NEWEST min(inflight, FLOSS_ACL_INFLIGHT_MAX).
 * If more packets than that are in flight the oldest lose their stamp (we
 * overwrite), and completing an unstamped packet simply records nothing.
 */
static void inflight_push(floss_conn_t *c, uint32_t now) {
    if (c->inflight < FLOSS_ACL_INFLIGHT_MAX) {
        c->inflight_t[(c->inflight_head + c->inflight) % FLOSS_ACL_INFLIGHT_MAX] = now;
    } else {
        c->inflight_t[c->inflight_head] = now;
        c->inflight_head = (uint8_t)((c->inflight_head + 1) % FLOSS_ACL_INFLIGHT_MAX);
    }
    c->inflight++;
}

static void inflight_pop(floss_conn_t *c, uint32_t now) {
    if (c->inflight == 0) {
        return;
    }
    if (c->inflight <= FLOSS_ACL_INFLIGHT_MAX) {
        FLOSS_STAT_HIST(acl_ack_latency, (uint32_t)(now - c->inflight_t[c->inflight_head]));
        c->inflight_head = (uint8_t)((c->inflight_head + 1) % FLOSS_ACL_INFLIGHT_MAX);
    }
    c->inflight--;
}

/* ------------------------------------------------------------------------ */
/* TX                                                                       */
/* ------------------------------------------------------------------------ */

floss_err_t floss_hci_acl_send(floss_buf_t *buf) {
    if (!buf) {
        return FLOSS_ERR_INVALID;
    }
    floss_conn_t *c = floss_hci_conn_by_handle(buf->handle);
    if (!c || c->state != FLOSS_CONN_OPEN) {
        FLOSS_LOGE(HCI, "ACL send on unknown link 0x%03x", buf->handle);
        floss_buf_free(buf);
        return FLOSS_ERR_NOT_FOUND;
    }
    if (buf->len == 0) {
        floss_buf_free(buf);
        return FLOSS_ERR_INVALID;
    }
    if (c->txq.count >= FLOSS_ACL_TXQ_MAX) {
        /* The link is not draining (peer not acknowledging): keep the pool for others. */
        FLOSS_STAT_INC(acl_tx_dropped);
        FLOSS_LOGW(HCI, "link 0x%03x: TX queue full (%u), PDU dropped", c->handle, (unsigned)c->txq.count);
        floss_buf_free(buf);
        return FLOSS_ERR_BUSY;
    }
    buf->type = FLOSS_PKT_ACL;
    buf->flags &= (uint8_t)(FLOSS_BUF_F_FLUSHABLE | FLOSS_BUF_F_PRIORITY);

    if (buf->flags & FLOSS_BUF_F_PRIORITY) {
        floss_buf_t *head = c->txq.head;
        if (head && (head->flags & FLOSS_HCI_BUF_F_CONT)) {
            /* Head is mid-PDU: its remaining fragments must go next, so slot in behind it. */
            floss_bufq_pop(&c->txq);
            floss_bufq_push_head(&c->txq, buf);
            floss_bufq_push_head(&c->txq, head);
        } else {
            floss_bufq_push_head(&c->txq, buf);
        }
    } else {
        floss_bufq_push_tail(&c->txq, buf);
    }

    floss_hci_acl_flush();
    return FLOSS_OK;
}

/**
 * Hand the next fragment of c's head buffer to the transport.
 * @return FLOSS_ERR_BUSY if the transport refused (nothing changed), else FLOSS_OK.
 */
static floss_err_t send_fragment(floss_conn_t *c, uint16_t *credits) {
    floss_buf_t *b = c->txq.head;
    uint16_t mtu = link_mtu(c);
    uint16_t frag = FLOSS_MIN(b->len, mtu);
    bool first = !(b->flags & FLOSS_HCI_BUF_F_CONT);

    uint8_t pb;
    if (!first) {
        pb = FLOSS_ACL_PB_CONT;
    } else if (!c->le && (b->flags & FLOSS_BUF_F_FLUSHABLE)) {
        pb = FLOSS_ACL_PB_FIRST_FLUSH;
    } else {
        pb = FLOSS_ACL_PB_FIRST_NONFLUSH; /* LE always uses 0b00 for a start fragment */
    }

    /* The header plus the driver's own headroom must stay inside the buffer. */
    uint8_t headroom = G.transport->ops->headroom;
    uint8_t *hdr = b->data - FLOSS_HCI_ACL_HDR_LEN;
    uint8_t *front = hdr - headroom;
    FLOSS_ASSERT(front >= (uint8_t *)b->mem);

    uint8_t saved[FLOSS_TRANSPORT_MAX_HEADROOM + FLOSS_HCI_ACL_HDR_LEN];
    uint16_t save_len = (uint16_t)(headroom + FLOSS_HCI_ACL_HDR_LEN);
    if (!first) {
        memcpy(saved, front, save_len);
    }

    floss_put_le16(hdr, (uint16_t)(c->handle | ((uint16_t)pb << 12)));
    floss_put_le16(hdr + 2, frag);
    floss_err_t err = floss_hci_transport_send(FLOSS_PKT_ACL, hdr, (uint16_t)(frag + FLOSS_HCI_ACL_HDR_LEN));

    if (!first) {
        memcpy(front, saved, save_len);
    }
    if (err == FLOSS_ERR_BUSY) {
        return FLOSS_ERR_BUSY;
    }
    if (err != FLOSS_OK) {
        /* Transport failure: drop the whole PDU; the next start fragment resynchronises the peer. */
        floss_buf_free(floss_bufq_pop(&c->txq));
        return FLOSS_OK;
    }

    (*credits)--;
    inflight_push(c, floss_hal_time_us());
    FLOSS_STAT_INC(tx_acl);
    FLOSS_TRACE(FLOSS_TR_TX_ACL, ((uint32_t)c->handle << 16) | frag);

    if (frag == b->len) {
        floss_buf_free(floss_bufq_pop(&c->txq));
    } else {
        floss_buf_pull(b, frag);
        b->flags |= FLOSS_HCI_BUF_F_CONT;
    }
    return FLOSS_OK;
}

void floss_hci_acl_flush(void) {
    if (!G.started) {
        return;
    }
    bool progress;
    bool waiting;
    uint16_t stalled_handle = 0;
    do {
        progress = false;
        waiting = false;
        for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
            floss_conn_t *c = &G.conns[(G.acl_rr + i) % FLOSS_MAX_CONNECTIONS];
            if (c->state != FLOSS_CONN_OPEN || !c->txq.head) {
                continue;
            }
            uint16_t *credits = pool_credits(c);
            if (*credits == 0 || link_mtu(c) == 0) {
                waiting = true;
                stalled_handle = c->handle;
                continue;
            }
            if (send_fragment(c, credits) == FLOSS_ERR_BUSY) {
                return; /* transport back-pressure; floss_hci_poll() retries */
            }
            progress = true;
        }
        G.acl_rr = (uint8_t)((G.acl_rr + 1) % FLOSS_MAX_CONNECTIONS);
    } while (progress);

    if (waiting && !G.acl_stalled) {
        G.acl_stalled = true;
        FLOSS_STAT_INC(acl_credit_stalls);
        FLOSS_TRACE(FLOSS_TR_CREDIT_STALL, stalled_handle);
    }
    FLOSS_UNUSED(stalled_handle); /* when tracing is compiled out */
}

bool floss_hci_acl_can_send_now(uint16_t handle) {
    const floss_conn_t *c = floss_hci_conn_by_handle(handle);
    return c && c->state == FLOSS_CONN_OPEN && c->txq.count == 0 && *pool_credits(c) > 0 && link_mtu(c) > 0;
}

void floss_hci_acl_notify_credits(void) {
    floss_l2cap_on_credits();
#if FLOSS_ENABLE_LE
    floss_le_on_credits();
#endif
}

/*
 * Number Of Completed Packets:
 *   Num_Handles(1) then Num_Handles x { Connection_Handle(2) Num_Completed_Packets(2) }
 */
void floss_hci_acl_on_completed(const uint8_t *p, uint8_t len) {
    if (len < 1 || 1u + 4u * p[0] > len) {
        FLOSS_STAT_INC(rx_dropped);
        return;
    }
    uint8_t n = p[0];
    uint32_t now = floss_hal_time_us();
    bool freed = false;

    for (uint8_t i = 0; i < n; i++) {
        const uint8_t *e = p + 1 + 4 * i;
        uint16_t handle = floss_get_le16(e) & 0x0FFF;
        uint16_t count = floss_get_le16(e + 2);
        floss_conn_t *c = floss_hci_conn_by_handle(handle);
        if (!c || count == 0) {
            continue; /* link already gone: its credits were returned at disconnect */
        }
        if (count > c->inflight) {
            FLOSS_LOGW(HCI, "link 0x%03x: controller completed %u, only %u in flight", handle, count, c->inflight);
            count = c->inflight;
        }
        for (uint16_t k = 0; k < count; k++) {
            inflight_pop(c, now);
        }
        pool_give(c, count);
        FLOSS_TRACE(FLOSS_TR_ACL_DONE, ((uint32_t)handle << 16) | count);
        freed = freed || count > 0;
    }

    if (freed) {
        G.acl_stalled = false; /* end of the stall episode, if any */
        floss_hci_acl_flush();
        floss_hci_acl_notify_credits();
    }
}

uint16_t floss_hci_acl_link_down(floss_conn_t *c) {
    uint16_t returned = c->inflight;
    /* The controller discards a dead link's queued packets and frees their buffers. */
    pool_give(c, returned);
    if (returned) {
        G.acl_stalled = false;
    }
    c->inflight = 0;
    c->inflight_head = 0;
    floss_bufq_drain(&c->txq);
    floss_buf_free(c->rx_buf);
    c->rx_buf = NULL;
    c->rx_expected = 0;
    return returned;
}

/* ------------------------------------------------------------------------ */
/* RX                                                                       */
/* ------------------------------------------------------------------------ */

static void deliver(const floss_conn_t *c, const uint8_t *pdu, uint16_t len) {
    if (c->le) {
#if FLOSS_ENABLE_LE
        floss_le_on_pdu(c->handle, pdu, len);
#endif
    } else {
        floss_l2cap_on_pdu(c->handle, pdu, len);
    }
}

static void rx_abort(floss_conn_t *c, const char *why) {
    FLOSS_STAT_INC(rx_dropped);
    FLOSS_LOGW(HCI, "link 0x%03x: dropping RX PDU (%s)", c->handle, why);
    floss_buf_free(c->rx_buf);
    c->rx_buf = NULL;
    c->rx_expected = 0;
}

/** Append a fragment to the reassembly buffer; deliver when the PDU is complete. */
static void rx_append(floss_conn_t *c, const uint8_t *data, uint16_t n) {
    floss_buf_t *b = c->rx_buf;
    uint8_t *dst = floss_buf_put(b, n);
    if (!dst) {
        rx_abort(c, "PDU larger than a pool buffer");
        return;
    }
    memcpy(dst, data, n);

    /* The L2CAP length may only become known once 2 bytes have arrived. */
    if (c->rx_expected == 0 && b->len >= 2) {
        c->rx_expected = (uint16_t)(floss_get_le16(b->data) + FLOSS_L2CAP_HDR_LEN);
        if (c->rx_expected > b->len + floss_buf_tailroom(b)) {
            rx_abort(c, "PDU larger than a pool buffer");
            return;
        }
    }
    if (c->rx_expected == 0 || b->len < c->rx_expected) {
        return; /* more fragments to come */
    }
    if (b->len > c->rx_expected) {
        rx_abort(c, "fragments exceed L2CAP length");
        return;
    }
    c->rx_buf = NULL;
    c->rx_expected = 0;
    deliver(c, b->data, b->len);
    floss_buf_free(b);
}

void floss_hci_acl_on_rx(const uint8_t *pkt, uint16_t len) {
    if (len < FLOSS_HCI_ACL_HDR_LEN) {
        FLOSS_STAT_INC(rx_dropped);
        return;
    }
    uint16_t hf = floss_get_le16(pkt);
    uint16_t handle = hf & 0x0FFF;
    uint8_t pb = (uint8_t)((hf >> 12) & 0x3);
    uint16_t dlen = floss_get_le16(pkt + 2);
    const uint8_t *data = pkt + FLOSS_HCI_ACL_HDR_LEN;

    FLOSS_STAT_INC(rx_acl);
    FLOSS_TRACE(FLOSS_TR_RX_ACL, ((uint32_t)handle << 16) | dlen);

    if ((uint32_t)dlen + FLOSS_HCI_ACL_HDR_LEN > len) { /* trailing padding is tolerated */
        FLOSS_STAT_INC(rx_dropped);
        FLOSS_LOGW(HCI, "truncated ACL packet (%u of %u bytes)", len, dlen + FLOSS_HCI_ACL_HDR_LEN);
        return;
    }
    floss_conn_t *c = floss_hci_conn_by_handle(handle);
    if (!c) {
        FLOSS_STAT_INC(rx_dropped);
        FLOSS_LOGD(HCI, "ACL for unknown link 0x%03x", handle);
        return;
    }

    if (pb == FLOSS_ACL_PB_CONT) {
        if (!c->rx_buf) {
            FLOSS_STAT_INC(rx_dropped); /* rest of a PDU we already gave up on */
            return;
        }
        rx_append(c, data, dlen);
        return;
    }
    if (pb != FLOSS_ACL_PB_FIRST_FLUSH && pb != FLOSS_ACL_PB_FIRST_NONFLUSH) {
        FLOSS_STAT_INC(rx_dropped);
        return;
    }

    /* Start of a new PDU. */
    if (c->rx_buf) {
        rx_abort(c, "new PDU before previous one completed");
    }
    if (dlen >= 2) {
        uint32_t l2len = (uint32_t)floss_get_le16(data) + FLOSS_L2CAP_HDR_LEN;
        if (l2len == dlen) {
            deliver(c, data, dlen); /* fast path: whole PDU in one fragment, no copy */
            return;
        }
        if (l2len < dlen) {
            FLOSS_STAT_INC(rx_dropped);
            FLOSS_LOGW(HCI, "link 0x%03x: ACL longer than its L2CAP PDU", handle);
            return;
        }
    }
    c->rx_buf = floss_buf_alloc();
    if (!c->rx_buf) {
        FLOSS_STAT_INC(buf_alloc_failures);
        FLOSS_STAT_INC(rx_dropped);
        return;
    }
    c->rx_expected = 0;
    rx_append(c, data, dlen);
}
