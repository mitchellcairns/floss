/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_rfcomm.c - RFCOMM public API: server registry, DLC lifecycle, the
 * data path and credit based flow control.
 *
 * ===========================================================================
 *  OVERVIEW
 *
 *  Storage is static: FLOSS_MAX_CONNECTIONS sessions (one per link) and
 *  FLOSS_RFCOMM_MAX_DLC DLC slots shared by all sessions. A DLC slot points
 *  at its session; the session never lists its DLCs (they are found by a
 *  linear scan over a handful of slots, cheaper than keeping lists right).
 *
 *  DLC lifecycle
 *  -------------
 *    responder:  [PN -> NEGOTIATED] -> SABM -> UA sent -> OPEN
 *    initiator:  WAIT_MUX -> (session up) -> PN sent: WAIT_PN -> PN rsp ->
 *                SABM sent: WAIT_UA -> UA -> OPEN
 *    both:       OPEN -> DISC sent: WAIT_DISC -> UA/DM -> FREE
 *                OPEN -> DISC received -> UA sent -> FREE
 *  Opening sends our MSC straight away (RFCOMM requires each side to send
 *  MSC before data; the control FIFO keeps it ahead of any data).
 *
 *  Data path
 *  ---------
 *  floss_rfcomm_send() copies each fragment into a pool buffer reserved with
 *  FLOSS_RFCOMM_HEADROOM, pushes the 2-5 byte header in front, appends the
 *  FCS and hands the buffer to floss_l2cap_send_buf(): one copy, total. The
 *  zero-copy variant skips even that.
 *
 *  Credits
 *  -------
 *  tx_credits: frames we may send (initial k from PN, plus credit fields of
 *  incoming UIH frames). Every data frame costs one. rx_credits: frames the
 *  peer may still send us. We pay credits out piggybacked on data frames
 *  (free: the byte is there anyway) or in empty UIH frames when the peer is
 *  down to half of FLOSS_RFCOMM_RX_CREDITS and we have nothing to send.
 * ===========================================================================
 */
#include "floss_rfcomm_int.h"

#if FLOSS_ENABLE_RFCOMM

floss_rfcomm_ctx_t floss_rfcomm_ctx;
#define CTX floss_rfcomm_ctx

/** Pool was empty: how long to wait before letting blocked senders retry. */
#define RFC_RETRY_US 2000u

/* ------------------------------------------------------------------------ */
/* Sessions, servers, DLC slots                                             */
/* ------------------------------------------------------------------------ */

floss_rfcomm_mux_t *floss_rfcomm_mux_alloc(void) {
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        floss_rfcomm_mux_t *mux = &CTX.mux[i];
        if (mux->state == RFC_MUX_FREE) {
            memset(mux, 0, sizeof *mux);
            return mux;
        }
    }
    return NULL;
}

void floss_rfcomm_mux_free(floss_rfcomm_mux_t *mux) {
    memset(mux, 0, sizeof *mux);
}

floss_rfcomm_mux_t *floss_rfcomm_mux_by_handle(uint16_t handle) {
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        floss_rfcomm_mux_t *mux = &CTX.mux[i];
        if (mux->state != RFC_MUX_FREE && mux->handle == handle) {
            return mux;
        }
    }
    return NULL;
}

const floss_rfcomm_server_t *floss_rfcomm_server_find(uint8_t channel) {
    for (uint8_t i = 0; i < FLOSS_RFCOMM_MAX_SERVERS; i++) {
        if (CTX.server[i].channel == channel && channel != 0) {
            return &CTX.server[i];
        }
    }
    return NULL;
}

floss_rfcomm_dlc_t *floss_rfcomm_dlc_find(const floss_rfcomm_mux_t *mux, uint8_t dlci) {
    for (uint8_t i = 0; i < FLOSS_RFCOMM_MAX_DLC; i++) {
        floss_rfcomm_dlc_t *dlc = &CTX.dlc[i];
        if (dlc->state != FLOSS_RFCOMM_DLC_FREE && dlc->mux == mux && dlc->dlci == dlci) {
            return dlc;
        }
    }
    return NULL;
}

floss_rfcomm_dlc_t *floss_rfcomm_dlc_alloc(floss_rfcomm_mux_t *mux, uint8_t dlci) {
    static const uint8_t port_defaults[5] = {0x07, 0x03, 0x00, 0x11, 0x13}; /* 115200 8N1, XON/XOFF chars */

    for (uint8_t i = 0; i < FLOSS_RFCOMM_MAX_DLC; i++) {
        floss_rfcomm_dlc_t *dlc = &CTX.dlc[i];
        if (dlc->state != FLOSS_RFCOMM_DLC_FREE || dlc->closing) {
            continue;
        }
        memset(dlc, 0, sizeof *dlc);
        dlc->state = FLOSS_RFCOMM_DLC_NEGOTIATED;
        dlc->mux = mux;
        dlc->handle = mux->handle;
        dlc->dlci = dlci;
        dlc->channel = (uint8_t)(dlci >> 1);
        dlc->mtu = FLOSS_MIN(FLOSS_RFCOMM_DEFAULT_MTU, floss_rfcomm_mux_max_mtu(mux));
        dlc->cfc = mux->cfc;
        dlc->local_v24 = FLOSS_RFCOMM_V24_DEFAULT;
        memcpy(dlc->port, port_defaults, sizeof dlc->port);
        return dlc;
    }
    return NULL;
}

/*
 * The direction bit splits the DLCI space between the two devices: server
 * channels of the session responder use D = 0, those of the initiator D = 1.
 */
bool floss_rfcomm_dlci_is_local(const floss_rfcomm_mux_t *mux, uint8_t dlci) {
    return (dlci & 1u) == (mux->initiator ? 1u : 0u);
}

/* ------------------------------------------------------------------------ */
/* Credits                                                                  */
/* ------------------------------------------------------------------------ */

/** Credits to hand out at the next opportunity (fits the 8-bit credit field). */
static uint8_t credits_to_give(const floss_rfcomm_dlc_t *dlc) {
    uint32_t give = dlc->grant_pending;
    if (!dlc->manual_credits && dlc->rx_credits < FLOSS_RFCOMM_RX_CREDITS) {
        give += (uint32_t)(FLOSS_RFCOMM_RX_CREDITS - dlc->rx_credits);
    }
    return (uint8_t)FLOSS_MIN(give, 255u);
}

static void credits_given(floss_rfcomm_dlc_t *dlc, uint8_t give) {
    dlc->rx_credits = (uint16_t)(dlc->rx_credits + give);
    dlc->grant_pending = (uint16_t)(dlc->grant_pending - FLOSS_MIN(give, dlc->grant_pending));
}

void floss_rfcomm_dlc_replenish(floss_rfcomm_dlc_t *dlc) {
    if (!dlc->cfc || dlc->state != FLOSS_RFCOMM_DLC_OPEN) {
        return;
    }
    /* Automatic mode waits until half the window is used so that one credit
     * frame refills many data frames, but never lets the peer run dry. */
    bool due = dlc->grant_pending != 0 ||
               (!dlc->manual_credits && dlc->rx_credits <= FLOSS_RFCOMM_RX_CREDITS / 2);
    uint8_t give = due ? credits_to_give(dlc) : 0;
    if (give == 0) {
        return;
    }
    if (floss_rfcomm_mux_send_frame(dlc->mux, dlc->dlci, RFC_UIH, true, NULL, 0, give) == FLOSS_OK) {
        credits_given(dlc, give);
    }
}

/* ------------------------------------------------------------------------ */
/* DLC lifecycle                                                            */
/* ------------------------------------------------------------------------ */

void floss_rfcomm_dlc_opened(floss_rfcomm_dlc_t *dlc) {
    floss_rfcomm_mux_send_msc(dlc->mux, dlc);
    floss_rfcomm_dlc_replenish(dlc);
    dlc->open_reported = true;
    RFC_LOGI("DLC open: channel %u (DLCI %u) mtu %u %s", dlc->channel, dlc->dlci, dlc->mtu,
             dlc->cfc ? "credit flow" : "no credit flow");
    if (dlc->ops != NULL && dlc->ops->on_open != NULL) {
        dlc->ops->on_open(dlc, FLOSS_RFCOMM_STATUS_OK);
    }
}

void floss_rfcomm_dlc_closed(floss_rfcomm_dlc_t *dlc, uint8_t status) {
    floss_rfcomm_mux_t *mux = dlc->mux;
    const floss_rfcomm_ops_t *ops = dlc->ops;
    bool was_open = dlc->open_reported;

    /* The slot reads FREE to every API call during the final callback, but
     * cannot be reallocated until the callback has returned. */
    dlc->state = FLOSS_RFCOMM_DLC_FREE;
    dlc->closing = true;
    if (was_open) {
        RFC_LOGI("DLC closed: channel %u (DLCI %u)", dlc->channel, dlc->dlci);
        if (ops != NULL && ops->on_close != NULL) {
            ops->on_close(dlc);
        }
    } else if (dlc->initiator && status != FLOSS_RFCOMM_STATUS_OK) {
        RFC_LOGI("DLC to channel %u failed (%u)", dlc->channel, status);
        if (ops != NULL && ops->on_open != NULL) {
            ops->on_open(dlc, status);
        }
    }
    memset(dlc, 0, sizeof *dlc);
    floss_rfcomm_mux_maybe_shutdown(mux);
}

void floss_rfcomm_mux_close_all(floss_rfcomm_mux_t *mux, uint8_t status) {
    for (uint8_t i = 0; i < FLOSS_RFCOMM_MAX_DLC; i++) {
        floss_rfcomm_dlc_t *dlc = &CTX.dlc[i];
        if (dlc->state != FLOSS_RFCOMM_DLC_FREE && dlc->mux == mux) {
            floss_rfcomm_dlc_closed(dlc, status);
        }
    }
}

void floss_rfcomm_dlc_rx(floss_rfcomm_dlc_t *dlc, int credits, const uint8_t *data, uint16_t len) {
    if (credits > 0) {
        bool was_empty = dlc->tx_credits == 0;
        dlc->tx_credits = (uint16_t)FLOSS_MIN((uint32_t)dlc->tx_credits + (uint32_t)credits, 0xFFFFu);
        if (was_empty && dlc->want_send) {
            floss_rfcomm_mux_kick(dlc->mux);
        }
    }
    if (len == 0 || dlc->state == FLOSS_RFCOMM_DLC_FREE) {
        return;
    }
    if (dlc->cfc) {
        if (dlc->rx_credits > 0) {
            dlc->rx_credits--;
        } else {
            RFC_LOGW("DLCI %u: peer sent without credit", dlc->dlci);
        }
    }
    if (len > dlc->mtu) {
        RFC_LOGW("DLCI %u: frame of %u exceeds mtu %u", dlc->dlci, len, dlc->mtu);
    }
    if (dlc->ops != NULL && dlc->ops->on_data != NULL) {
        dlc->ops->on_data(dlc, data, len);
    }
    /* Delivered = consumed (automatic mode); the callback may have closed it. */
    if (dlc->state == FLOSS_RFCOMM_DLC_OPEN) {
        floss_rfcomm_dlc_replenish(dlc);
    }
}

void floss_rfcomm_mux_kick(floss_rfcomm_mux_t *mux) {
    if (!floss_rfcomm_mux_flush(mux)) {
        return;
    }
    for (uint8_t i = 0; i < FLOSS_RFCOMM_MAX_DLC; i++) {
        floss_rfcomm_dlc_t *dlc = &CTX.dlc[i];
        if (dlc->mux != mux || dlc->state != FLOSS_RFCOMM_DLC_OPEN) {
            continue;
        }
        /* Retry credit grants that could not be sent earlier. */
        floss_rfcomm_dlc_replenish(dlc);
        if (dlc->want_send && floss_rfcomm_can_send_now(dlc)) {
            dlc->want_send = false;
            if (dlc->ops != NULL && dlc->ops->on_can_send != NULL) {
                dlc->ops->on_can_send(dlc);
            }
        }
    }
}

static void retry_timer_fn(floss_timer_t *timer, void *arg) {
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        floss_rfcomm_mux_t *mux = &CTX.mux[i];
        if (mux->state != RFC_MUX_FREE && mux->chan != NULL) {
            floss_rfcomm_mux_kick(mux);
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Init and servers                                                         */
/* ------------------------------------------------------------------------ */

void floss_rfcomm_init(void) {
    floss_timer_stop(&CTX.retry_timer); /* re-init: never leave it linked in the scheduler */
    memset(&CTX, 0, sizeof CTX);
    floss_timer_init(&CTX.retry_timer, retry_timer_fn, NULL);
    floss_spp_reset();
    floss_err_t err = floss_l2cap_register_psm(FLOSS_PSM_RFCOMM, &floss_rfcomm_l2cap_ops, NULL, FLOSS_SEC_ENCRYPTED);
    if (err != FLOSS_OK) {
        RFC_LOGE("cannot register PSM 0x0003 (%s)", floss_err_str(err));
    }
}

floss_err_t floss_rfcomm_register_server(uint8_t channel, const floss_rfcomm_ops_t *ops, void *user) {
    if (channel < FLOSS_RFCOMM_CHANNEL_MIN || channel > FLOSS_RFCOMM_CHANNEL_MAX || ops == NULL) {
        return FLOSS_ERR_INVALID;
    }
    if (floss_rfcomm_server_find(channel) != NULL) {
        return FLOSS_ERR_STATE;
    }
    for (uint8_t i = 0; i < FLOSS_RFCOMM_MAX_SERVERS; i++) {
        floss_rfcomm_server_t *srv = &CTX.server[i];
        if (srv->channel == 0) {
            srv->channel = channel;
            srv->ops = ops;
            srv->user = user;
            return FLOSS_OK;
        }
    }
    return FLOSS_ERR_NOMEM;
}

floss_err_t floss_rfcomm_unregister_server(uint8_t channel) {
    floss_rfcomm_server_t *srv = (floss_rfcomm_server_t *)floss_rfcomm_server_find(channel);
    if (srv == NULL) {
        return FLOSS_ERR_NOT_FOUND;
    }
    memset(srv, 0, sizeof *srv);
    return FLOSS_OK;
}

/* ------------------------------------------------------------------------ */
/* Connect / disconnect                                                     */
/* ------------------------------------------------------------------------ */

floss_err_t floss_rfcomm_connect(uint16_t handle, uint8_t channel, const floss_rfcomm_ops_t *ops, void *user,
                                 floss_rfcomm_dlc_t **out) {
    if (channel < FLOSS_RFCOMM_CHANNEL_MIN || channel > FLOSS_RFCOMM_CHANNEL_MAX || ops == NULL || out == NULL) {
        return FLOSS_ERR_INVALID;
    }
    *out = NULL;

    floss_rfcomm_mux_t *mux = floss_rfcomm_mux_by_handle(handle);
    bool new_mux = false;
    if (mux == NULL) {
        mux = floss_rfcomm_mux_alloc();
        if (mux == NULL) {
            return FLOSS_ERR_NOMEM;
        }
        mux->handle = handle;
        mux->initiator = true;
        mux->state = RFC_MUX_WAIT_L2CAP;
        new_mux = true;
    } else if (mux->state == RFC_MUX_CLOSING || mux->state == RFC_MUX_IDLE) {
        /* Closing, or the peer opened L2CAP but has not started the session yet. */
        return FLOSS_ERR_BUSY;
    }

    /* The peer's server channels carry the direction bit of the far side. */
    uint8_t dlci = (uint8_t)((channel << 1) | (mux->initiator ? 0u : 1u));
    if (floss_rfcomm_dlc_find(mux, dlci) != NULL) {
        return FLOSS_ERR_STATE; /* already connected / connecting to that channel */
    }
    floss_rfcomm_dlc_t *dlc = floss_rfcomm_dlc_alloc(mux, dlci);
    if (dlc == NULL) {
        if (new_mux) {
            floss_rfcomm_mux_free(mux);
        }
        return FLOSS_ERR_NOMEM;
    }
    dlc->initiator = true;
    dlc->ops = ops;
    dlc->user = user;
    dlc->state = FLOSS_RFCOMM_DLC_WAIT_MUX;
    *out = dlc;

    if (new_mux) {
        floss_l2cap_chan_t *chan = NULL;
        floss_err_t err = floss_l2cap_connect(handle, FLOSS_PSM_RFCOMM, &floss_rfcomm_l2cap_ops, mux,
                                              FLOSS_SEC_ENCRYPTED, &chan);
        if (err != FLOSS_OK) {
            memset(dlc, 0, sizeof *dlc);
            floss_rfcomm_mux_free(mux);
            *out = NULL;
            return err;
        }
        if (mux->state == RFC_MUX_WAIT_L2CAP && mux->chan == NULL) {
            mux->chan = chan;
        }
    } else if (mux->state == RFC_MUX_OPEN) {
        dlc->mtu = floss_rfcomm_mux_max_mtu(mux);
        dlc->state = FLOSS_RFCOMM_DLC_WAIT_PN;
        floss_rfcomm_mux_send_pn(mux, dlc, true);
    }
    /* else: the session is starting; floss_rfcomm_mux_started() sends the PN. */
    return FLOSS_OK;
}

floss_err_t floss_rfcomm_disconnect(floss_rfcomm_dlc_t *dlc) {
    if (dlc == NULL || dlc->state == FLOSS_RFCOMM_DLC_FREE) {
        return FLOSS_ERR_INVALID;
    }
    switch (dlc->state) {
    case FLOSS_RFCOMM_DLC_WAIT_DISC:
        return FLOSS_OK;
    case FLOSS_RFCOMM_DLC_OPEN:
    case FLOSS_RFCOMM_DLC_WAIT_UA: {
        floss_err_t err = floss_rfcomm_mux_send_frame(dlc->mux, dlc->dlci, RFC_DISC, true, NULL, 0, -1);
        if (err != FLOSS_OK) {
            return err;
        }
        dlc->state = FLOSS_RFCOMM_DLC_WAIT_DISC;
        dlc->want_send = false;
        return FLOSS_OK;
    }
    default:
        /* Nothing on the wire for this DLC yet (or only a PN): just forget it. */
        floss_rfcomm_dlc_closed(dlc, FLOSS_RFCOMM_STATUS_OK);
        return FLOSS_OK;
    }
}

/* ------------------------------------------------------------------------ */
/* Data path                                                                */
/* ------------------------------------------------------------------------ */

bool floss_rfcomm_can_send_now(const floss_rfcomm_dlc_t *dlc) {
    if (dlc == NULL || dlc->state != FLOSS_RFCOMM_DLC_OPEN) {
        return false;
    }
    const floss_rfcomm_mux_t *mux = dlc->mux;
    if (mux->chan == NULL || mux->q_count != 0) {
        return false;
    }
    if (dlc->cfc) {
        return dlc->tx_credits > 0;
    }
    return !mux->peer_fcoff && !dlc->remote_fc;
}

floss_buf_t *floss_rfcomm_buf_alloc(void) {
    floss_buf_t *buf = floss_buf_alloc();
    if (buf != NULL) {
        buf->data += FLOSS_RFCOMM_HEADROOM; /* len is 0: just move the start */
    }
    return buf;
}

/** Frame and send one payload already at buf->data. Consumes buf. */
static floss_err_t dlc_tx(floss_rfcomm_dlc_t *dlc, floss_buf_t *buf) {
    floss_rfcomm_mux_t *mux = dlc->mux;
    uint8_t give = dlc->cfc ? credits_to_give(dlc) : 0;
    uint8_t hdr[FLOSS_RFCOMM_HEADROOM];
    uint8_t fcs;
    uint8_t hl = floss_rfcomm_build_hdr(hdr, mux, dlc->dlci, RFC_UIH, give != 0, buf->len, give ? give : -1, &fcs);

    /* Lower layers still need room for L2CAP, ACL and transport headers. */
    size_t headroom = (size_t)(buf->data - (uint8_t *)buf->mem);
    uint8_t *tail = floss_buf_put(buf, 1);
    if (tail == NULL || headroom < (size_t)hl + FLOSS_TRANSPORT_MAX_HEADROOM + 8) {
        floss_buf_free(buf);
        return FLOSS_ERR_INVALID;
    }
    *tail = fcs;
    memcpy(floss_buf_push(buf, hl), hdr, hl);

    floss_err_t err = floss_l2cap_send_buf(mux->chan, buf, 0);
    if (err != FLOSS_OK) {
        return err;
    }
    if (dlc->cfc) {
        dlc->tx_credits--;
        credits_given(dlc, give);
    }
    return FLOSS_OK;
}

/** Make sure queued control frames go first; true if data may follow now. */
static bool data_path_clear(floss_rfcomm_dlc_t *dlc) {
    if (dlc->mux->q_count != 0 && dlc->mux->chan != NULL) {
        floss_rfcomm_mux_flush(dlc->mux);
    }
    return floss_rfcomm_can_send_now(dlc);
}

int floss_rfcomm_send(floss_rfcomm_dlc_t *dlc, const uint8_t *data, uint16_t len) {
    if (dlc == NULL || (data == NULL && len != 0)) {
        return FLOSS_ERR_INVALID;
    }
    if (dlc->state != FLOSS_RFCOMM_DLC_OPEN) {
        return FLOSS_ERR_STATE;
    }
    uint16_t sent = 0;
    floss_err_t err = FLOSS_ERR_BUSY;

    while (sent < len) {
        if (!data_path_clear(dlc)) {
            err = FLOSS_ERR_BUSY;
            break;
        }
        uint16_t n = FLOSS_MIN((uint16_t)(len - sent), dlc->mtu);
        floss_buf_t *buf = floss_rfcomm_buf_alloc();
        if (buf == NULL) {
            /* Nothing will call us back when the pool refills: poll for it. */
            if (!floss_timer_active(&CTX.retry_timer)) {
                floss_timer_start(&CTX.retry_timer, RFC_RETRY_US);
            }
            err = FLOSS_ERR_BUSY;
            break;
        }
        uint8_t *p = floss_buf_put(buf, n);
        if (p == NULL) {
            floss_buf_free(buf);
            err = FLOSS_ERR_INVALID;
            break;
        }
        memcpy(p, data + sent, n);
        err = dlc_tx(dlc, buf);
        if (err != FLOSS_OK) {
            break;
        }
        sent = (uint16_t)(sent + n);
    }

    if (sent < len) {
        if (err == FLOSS_ERR_BUSY) {
            dlc->want_send = true;
        }
        if (sent == 0) {
            return err;
        }
    }
    return sent;
}

floss_err_t floss_rfcomm_send_buf(floss_rfcomm_dlc_t *dlc, floss_buf_t *buf) {
    if (buf == NULL) {
        return FLOSS_ERR_INVALID;
    }
    if (dlc == NULL || dlc->state != FLOSS_RFCOMM_DLC_OPEN) {
        floss_buf_free(buf);
        return FLOSS_ERR_STATE;
    }
    if (buf->len > dlc->mtu) {
        floss_buf_free(buf);
        return FLOSS_ERR_INVALID;
    }
    if (!data_path_clear(dlc)) {
        floss_buf_free(buf);
        dlc->want_send = true;
        return FLOSS_ERR_BUSY;
    }
    floss_err_t err = dlc_tx(dlc, buf);
    if (err == FLOSS_ERR_BUSY) {
        dlc->want_send = true;
    }
    return err;
}

/* ------------------------------------------------------------------------ */
/* Flow control and signals                                                 */
/* ------------------------------------------------------------------------ */

void floss_rfcomm_set_manual_credits(floss_rfcomm_dlc_t *dlc, bool manual) {
    if (dlc == NULL) {
        return;
    }
    dlc->manual_credits = manual;
    if (!manual) {
        floss_rfcomm_dlc_replenish(dlc);
    }
}

floss_err_t floss_rfcomm_grant(floss_rfcomm_dlc_t *dlc, uint8_t n) {
    if (dlc == NULL || dlc->state != FLOSS_RFCOMM_DLC_OPEN) {
        return FLOSS_ERR_STATE;
    }
    if (!dlc->cfc || n == 0) {
        return FLOSS_OK;
    }
    dlc->grant_pending = (uint16_t)FLOSS_MIN((uint32_t)dlc->grant_pending + n, 0xFFFFu);
    floss_rfcomm_dlc_replenish(dlc);
    return FLOSS_OK;
}

floss_err_t floss_rfcomm_set_signals(floss_rfcomm_dlc_t *dlc, uint8_t v24) {
    if (dlc == NULL || dlc->state != FLOSS_RFCOMM_DLC_OPEN) {
        return FLOSS_ERR_STATE;
    }
    dlc->local_v24 = (uint8_t)(v24 & ~RFC_EA);
    return floss_rfcomm_mux_send_msc(dlc->mux, dlc);
}

#endif /* FLOSS_ENABLE_RFCOMM */
