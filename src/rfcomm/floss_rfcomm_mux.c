/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_rfcomm_mux.c - RFCOMM framing and the TS 07.10 multiplexer.
 *
 * ===========================================================================
 *  OVERVIEW
 *
 *  This file owns everything that touches the wire:
 *
 *   - FCS (CRC-8, x^8 + x^2 + x + 1, LSB first) and frame headers.
 *   - The L2CAP callbacks of PSM 0x0003. One L2CAP channel = one multiplexer
 *     session (floss_rfcomm_mux_t). Incoming channels get a session when
 *     they open; outgoing ones are created by floss_rfcomm_connect().
 *   - The frame dispatcher: SABM / UA / DM / DISC / UIH, for DLCI 0 (the
 *     multiplexer itself) and for data DLCIs.
 *   - Multiplexer control commands carried in UIH frames on DLCI 0:
 *     PN, MSC, RPN, RLS, Test, FCon/FCoff, and NSC for everything else.
 *   - The control-frame FIFO. Control frames are tiny and must never be lost
 *     (a missing UA or MSC response stalls the peer), so when L2CAP reports
 *     BUSY they are copied into a small per-session queue that is flushed
 *     from L2CAP's on_can_send. Data is held back while the queue is not
 *     empty, which also guarantees the protocol ordering "MSC before data".
 *
 *  The DLC lifecycle, credits and the public API live in floss_rfcomm.c.
 *
 *  C/R BIT RULES (TS 07.10 5.2.1.2, adopted by RFCOMM)
 *  ---------------------------------------------------
 *  Address field: commands carry C/R = 1 when sent by the session initiator
 *  and 0 when sent by the responder; responses are the opposite. SABM, DISC
 *  and UIH are commands, UA and DM responses. Inside a UIH frame on DLCI 0
 *  the message type octet has its own C/R bit: 1 = command, 0 = response,
 *  independent of role. Received address C/R bits are not checked: the frame
 *  type already says what it is and some peers get the bit wrong.
 * ===========================================================================
 */
#include "floss_rfcomm_int.h"

#if FLOSS_ENABLE_RFCOMM

/* ------------------------------------------------------------------------ */
/* FCS                                                                      */
/* ------------------------------------------------------------------------ */

/*
 * CRC-8 table for the reflected polynomial 0xE0 (x^8 + x^2 + x + 1 bit
 * reversed): TS 07.10 transmits the FCS LSB first, so the reflected form lets
 * us process whole bytes without bit reversal.
 */
static const uint8_t s_fcs_table[256] = {
    0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B,
    0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69, 0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67,
    0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE, 0xDC, 0x4D, 0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43,
    0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 0x2A, 0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F,
    0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05, 0x7E, 0xEF, 0x9D, 0x0C, 0x79, 0xE8, 0x9A, 0x0B,
    0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19, 0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17,
    0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D, 0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33,
    0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21, 0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F,
    0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04, 0x95, 0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B,
    0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89, 0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87,
    0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD, 0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3,
    0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1, 0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF,
    0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5, 0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB,
    0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9, 0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7,
    0xA8, 0x39, 0x4B, 0xDA, 0xAF, 0x3E, 0x4C, 0xDD, 0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3,
    0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1, 0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF,
};

/** Running CRC register over len bytes, starting from the 0xFF preset. */
static uint8_t crc_run(const uint8_t *p, uint16_t len) {
    uint8_t crc = 0xFF;
    while (len--) {
        crc = s_fcs_table[crc ^ *p++];
    }
    return crc;
}

uint8_t floss_rfcomm_fcs(const uint8_t *p, uint16_t len) {
    /* The transmitted FCS is the ones complement of the register. */
    return (uint8_t)(0xFF - crc_run(p, len));
}

/** Receiver check: CRC over the covered bytes plus the FCS leaves 0xCF. */
static bool fcs_ok(const uint8_t *p, uint16_t len, uint8_t fcs) {
    return s_fcs_table[crc_run(p, len) ^ fcs] == 0xCF;
}

/* ------------------------------------------------------------------------ */
/* Frame building                                                           */
/* ------------------------------------------------------------------------ */

static bool is_command_frame(uint8_t type) {
    return type != RFC_UA && type != RFC_DM;
}

uint8_t floss_rfcomm_build_hdr(uint8_t *hdr, const floss_rfcomm_mux_t *mux, uint8_t dlci, uint8_t type, bool pf,
                               uint16_t info_len, int credits, uint8_t *fcs) {
    /* Commands from the initiator and responses from the responder carry C/R = 1. */
    bool cr = is_command_frame(type) ? mux->initiator : !mux->initiator;
    uint8_t n = 0;

    hdr[n++] = (uint8_t)(RFC_EA | (cr ? RFC_CR : 0) | (dlci << 2));
    hdr[n++] = (uint8_t)(type | (pf ? RFC_PF : 0));
    if (info_len <= 127) {
        hdr[n++] = (uint8_t)((info_len << 1) | RFC_EA);
    } else {
        hdr[n++] = (uint8_t)(info_len << 1); /* EA = 0: another length octet follows */
        hdr[n++] = (uint8_t)(info_len >> 7);
    }
    /* UIH frames protect only address + control so the FCS can be computed
     * before the (possibly long) payload is known; other frames include length. */
    *fcs = floss_rfcomm_fcs(hdr, type == RFC_UIH ? 2 : n);
    if (credits >= 0) {
        hdr[n++] = (uint8_t)credits;
    }
    return n;
}

uint16_t floss_rfcomm_mux_max_mtu(const floss_rfcomm_mux_t *mux) {
    uint16_t m = FLOSS_BUF_SIZE - 8; /* keep frames inside one pool buffer */
    if (mux->chan != NULL) {
        m = FLOSS_MIN(m, mux->chan->local_mtu);
        m = FLOSS_MIN(m, mux->chan->remote_mtu);
    } else {
        m = FLOSS_MIN(m, FLOSS_L2CAP_MTU);
    }
    return (uint16_t)(m > FLOSS_RFCOMM_FRAME_OVERHEAD ? m - FLOSS_RFCOMM_FRAME_OVERHEAD : 1);
}

/* ------------------------------------------------------------------------ */
/* Control path transmit                                                    */
/* ------------------------------------------------------------------------ */

static bool queue_push(floss_rfcomm_mux_t *mux, const uint8_t *frame, uint8_t len) {
    if (mux->q_count >= FLOSS_RFCOMM_CTRL_QUEUE_LEN) {
        return false;
    }
    uint8_t slot = (uint8_t)((mux->q_head + mux->q_count) % FLOSS_RFCOMM_CTRL_QUEUE_LEN);
    memcpy(mux->q_frame[slot], frame, len);
    mux->q_len[slot] = len;
    mux->q_count++;
    return true;
}

bool floss_rfcomm_mux_flush(floss_rfcomm_mux_t *mux) {
    while (mux->q_count > 0 && mux->chan != NULL) {
        uint8_t slot = mux->q_head;
        floss_err_t err = floss_l2cap_send(mux->chan, mux->q_frame[slot], mux->q_len[slot]);
        if (err == FLOSS_ERR_BUSY) {
            return false;
        }
        if (err != FLOSS_OK) {
            RFC_LOGW("control frame dropped (%s)", floss_err_str(err));
        }
        mux->q_head = (uint8_t)((slot + 1) % FLOSS_RFCOMM_CTRL_QUEUE_LEN);
        mux->q_count--;
    }
    return mux->q_count == 0;
}

/**
 * Send one frame whose information field is the concatenation a + b (lets
 * MCC messages be sent without first assembling header and value). Small
 * frames are built on the stack and queued on BUSY; large ones (only Test
 * echoes) go out through a pool buffer on a best-effort basis.
 */
static floss_err_t send_frame2(floss_rfcomm_mux_t *mux, uint8_t dlci, uint8_t type, bool pf, const uint8_t *a,
                               uint16_t alen, const uint8_t *b, uint16_t blen, int credits) {
    if (mux->chan == NULL) {
        return FLOSS_ERR_STATE;
    }
    uint8_t hdr[FLOSS_RFCOMM_HEADROOM];
    uint8_t fcs;
    uint16_t info_len = (uint16_t)(alen + blen);
    uint8_t hl = floss_rfcomm_build_hdr(hdr, mux, dlci, type, pf, info_len, credits, &fcs);
    uint32_t total = (uint32_t)hl + info_len + 1;

    if (total <= RFC_CTRL_FRAME_MAX) {
        uint8_t frame[RFC_CTRL_FRAME_MAX];
        memcpy(frame, hdr, hl);
        if (alen) {
            memcpy(frame + hl, a, alen);
        }
        if (blen) {
            memcpy(frame + hl + alen, b, blen);
        }
        frame[total - 1] = fcs;
        if (mux->q_count == 0) {
            floss_err_t err = floss_l2cap_send(mux->chan, frame, (uint16_t)total);
            if (err != FLOSS_ERR_BUSY) {
                return err;
            }
        }
        if (!queue_push(mux, frame, (uint8_t)total)) {
            RFC_LOGE("control queue full, frame 0x%02X on DLCI %u lost", type, dlci);
            return FLOSS_ERR_NOMEM;
        }
        return FLOSS_OK;
    }

    if (mux->q_count != 0 || total > (uint32_t)mux->chan->remote_mtu) {
        return FLOSS_ERR_BUSY;
    }
    floss_buf_t *buf = floss_buf_alloc();
    if (buf == NULL) {
        return FLOSS_ERR_NOMEM;
    }
    uint8_t *p = floss_buf_put(buf, (uint16_t)total);
    if (p == NULL) {
        floss_buf_free(buf);
        return FLOSS_ERR_INVALID;
    }
    memcpy(p, hdr, hl);
    if (alen) {
        memcpy(p + hl, a, alen);
    }
    if (blen) {
        memcpy(p + hl + alen, b, blen);
    }
    p[total - 1] = fcs;
    return floss_l2cap_send_buf(mux->chan, buf, 0);
}

floss_err_t floss_rfcomm_mux_send_frame(floss_rfcomm_mux_t *mux, uint8_t dlci, uint8_t type, bool pf,
                                        const uint8_t *info, uint16_t len, int credits) {
    return send_frame2(mux, dlci, type, pf, info, len, NULL, 0, credits);
}

floss_err_t floss_rfcomm_mux_send_mcc(floss_rfcomm_mux_t *mux, uint8_t type, bool command, const uint8_t *val,
                                      uint16_t len) {
    uint8_t h[3];
    uint16_t hl = 0;
    h[hl++] = (uint8_t)(type | (command ? RFC_CR : 0) | RFC_EA);
    if (len <= 127) {
        h[hl++] = (uint8_t)((len << 1) | RFC_EA);
    } else {
        h[hl++] = (uint8_t)(len << 1);
        h[hl++] = (uint8_t)(len >> 7);
    }
    /* All UIH frames are commands at the frame level, whatever the message is. */
    return send_frame2(mux, 0, RFC_UIH, false, h, hl, val, len, -1);
}

/** Unnumbered frame without information field (SABM/UA/DM/DISC). */
static floss_err_t send_u(floss_rfcomm_mux_t *mux, uint8_t dlci, uint8_t type, bool pf) {
    floss_err_t err = send_frame2(mux, dlci, type, pf, NULL, 0, NULL, 0, -1);
    if (err != FLOSS_OK) {
        RFC_LOGW("frame 0x%02X on DLCI %u not sent (%s)", type, dlci, floss_err_str(err));
    }
    return err;
}

/* ------------------------------------------------------------------------ */
/* Multiplexer control commands                                             */
/* ------------------------------------------------------------------------ */

floss_err_t floss_rfcomm_mux_send_pn(floss_rfcomm_mux_t *mux, floss_rfcomm_dlc_t *dlc, bool command) {
    uint8_t v[8];
    uint8_t k = (uint8_t)FLOSS_MIN(FLOSS_RFCOMM_RX_CREDITS, 7); /* k is a 3-bit field */

    v[0] = dlc->dlci;
    /* Frame type I = 0 (UIH) in the low nibble, convergence layer in the high
     * nibble: the initiator offers CFC (0xF), the responder accepts (0xE). */
    if (command) {
        v[1] = RFC_PN_CL_CFC_REQ;
    } else {
        v[1] = dlc->cfc ? RFC_PN_CL_CFC_RSP : 0;
    }
    v[2] = (uint8_t)(dlc->priority & 0x3F);
    v[3] = 0; /* T1: acknowledgement timer, unused by RFCOMM */
    floss_put_le16(&v[4], dlc->mtu);
    v[6] = 0; /* N2: retransmissions, unused */
    v[7] = (command || dlc->cfc) ? k : 0;

    floss_err_t err = floss_rfcomm_mux_send_mcc(mux, RFC_MCC_PN, command, v, sizeof v);
    if (err == FLOSS_OK && v[7] != 0) {
        /* The initial credits in a PN are granted as soon as it is sent;
         * if the peer then declines CFC they are simply forgotten. */
        dlc->rx_credits = k;
    }
    return err;
}

floss_err_t floss_rfcomm_mux_send_msc(floss_rfcomm_mux_t *mux, floss_rfcomm_dlc_t *dlc) {
    uint8_t v[2];
    /* The DLCI inside MSC is an address octet whose C/R bit is always 1. */
    v[0] = (uint8_t)(RFC_EA | RFC_CR | (dlc->dlci << 2));
    v[1] = (uint8_t)(dlc->local_v24 | RFC_EA);
    return floss_rfcomm_mux_send_mcc(mux, RFC_MCC_MSC, true, v, sizeof v);
}

/** PN command from the peer: negotiate the parameters of a DLC before its SABM. */
static void rx_pn_cmd(floss_rfcomm_mux_t *mux, const uint8_t *v, uint16_t len) {
    if (len < 8) {
        return;
    }
    uint8_t dlci = (uint8_t)(v[0] & 0x3F);
    if (dlci == 0) {
        return;
    }
    floss_rfcomm_dlc_t *dlc = floss_rfcomm_dlc_find(mux, dlci);
    if (dlc == NULL) {
        const floss_rfcomm_server_t *srv = NULL;
        if (floss_rfcomm_dlci_is_local(mux, dlci)) {
            srv = floss_rfcomm_server_find((uint8_t)(dlci >> 1));
        }
        if (srv != NULL) {
            dlc = floss_rfcomm_dlc_alloc(mux, dlci);
        }
        if (dlc == NULL) {
            /* No such service (or no room): DM rejects the DLC up front. */
            RFC_LOGI("PN for DLCI %u refused", dlci);
            send_u(mux, dlci, RFC_DM, false);
            return;
        }
        dlc->state = FLOSS_RFCOMM_DLC_NEGOTIATED;
        dlc->ops = srv->ops;
        dlc->user = srv->user;
    }

    if (dlc->state == FLOSS_RFCOMM_DLC_NEGOTIATED) {
        /* N1 may only shrink: never more than the peer asked for, or than our
         * L2CAP channel can carry (with worst-case framing overhead). */
        uint16_t n1 = floss_get_le16(&v[4]);
        uint16_t max = floss_rfcomm_mux_max_mtu(mux);
        dlc->mtu = n1 == 0 ? FLOSS_MIN(FLOSS_RFCOMM_DEFAULT_MTU, max) : FLOSS_MIN(n1, max);
        dlc->priority = (uint8_t)(v[2] & 0x3F);
        if ((v[1] & 0xF0) == RFC_PN_CL_CFC_REQ) {
            dlc->cfc = true;
            mux->cfc = true;
            dlc->tx_credits = (uint16_t)(v[7] & 0x07);
        } else {
            dlc->cfc = false;
            dlc->tx_credits = 0;
            dlc->rx_credits = 0;
        }
    }
    /* An open DLC keeps its parameters: the response just restates them. */
    RFC_LOGD("PN DLCI %u: mtu %u cfc %u credits %u", dlci, dlc->mtu, dlc->cfc, dlc->tx_credits);
    floss_rfcomm_mux_send_pn(mux, dlc, false);
}

/** PN response to our command: adopt the parameters and open the DLC. */
static void rx_pn_rsp(floss_rfcomm_mux_t *mux, const uint8_t *v, uint16_t len) {
    if (len < 8) {
        return;
    }
    floss_rfcomm_dlc_t *dlc = floss_rfcomm_dlc_find(mux, (uint8_t)(v[0] & 0x3F));
    if (dlc == NULL || dlc->state != FLOSS_RFCOMM_DLC_WAIT_PN) {
        return;
    }
    uint16_t n1 = floss_get_le16(&v[4]);
    if (n1 != 0 && n1 < dlc->mtu) {
        dlc->mtu = n1;
    }
    if ((v[1] & 0xF0) == RFC_PN_CL_CFC_RSP) {
        dlc->cfc = true;
        mux->cfc = true;
        dlc->tx_credits = (uint16_t)(v[7] & 0x07);
    } else {
        dlc->cfc = false;
        dlc->tx_credits = 0;
        dlc->rx_credits = 0;
    }
    dlc->state = FLOSS_RFCOMM_DLC_WAIT_UA;
    send_u(mux, dlc->dlci, RFC_SABM, true);
}

/** MSC command: answer it, remember the peer's signals, honour FC without CFC. */
static void rx_msc_cmd(floss_rfcomm_mux_t *mux, const uint8_t *v, uint16_t len) {
    if (len < 2) {
        return;
    }
    floss_rfcomm_mux_send_mcc(mux, RFC_MCC_MSC, false, v, len);

    floss_rfcomm_dlc_t *dlc = floss_rfcomm_dlc_find(mux, (uint8_t)(v[0] >> 2));
    if (dlc == NULL) {
        return;
    }
    dlc->remote_v24 = v[1];
    /* With CFC the FC bit has no meaning (credits do the job) and is ignored. */
    if (!dlc->cfc) {
        bool was_off = dlc->remote_fc;
        dlc->remote_fc = (v[1] & FLOSS_RFCOMM_V24_FC) != 0;
        if (was_off && !dlc->remote_fc) {
            floss_rfcomm_mux_kick(mux);
        }
    }
}

/**
 * RPN: a 1-byte value is a query (answer with the current settings), an
 * 8-byte value proposes settings, of which the mask selects the ones to
 * change. We are a virtual port, so every setting is accepted and echoed
 * back with the same mask.
 */
static void rx_rpn_cmd(floss_rfcomm_mux_t *mux, const uint8_t *v, uint16_t len) {
    static const uint8_t defaults[5] = {
        0x07, /* 115200 baud */
        0x03, /* 8 data bits, 1 stop bit, no parity */
        0x00, /* no flow control */
        0x11, /* XON (DC1) */
        0x13, /* XOFF (DC3) */
    };
    if (len < 1) {
        return;
    }
    floss_rfcomm_dlc_t *dlc = floss_rfcomm_dlc_find(mux, (uint8_t)(v[0] >> 2));
    uint8_t scratch[5];
    uint8_t *port = dlc != NULL ? dlc->port : scratch;
    uint16_t mask = 0x3F7F; /* every parameter */
    if (dlc == NULL) {
        memcpy(scratch, defaults, sizeof scratch);
    }

    if (len >= 8) {
        mask = (uint16_t)(floss_get_le16(&v[6]) & 0x3F7F);
        if (mask & 0x0001) {
            port[0] = v[1];
        }
        /* Format octet: data bits (bits 0-1), stop (2), parity (3), parity type (4-5). */
        uint8_t fmt_mask = (uint8_t)(((mask & 0x0002) ? 0x03 : 0) | ((mask & 0x0004) ? 0x04 : 0) |
                                     ((mask & 0x0008) ? 0x08 : 0) | ((mask & 0x0010) ? 0x30 : 0));
        port[1] = (uint8_t)((port[1] & ~fmt_mask) | (v[2] & fmt_mask));
        /* Flow control bits map one to one onto the second mask octet. */
        uint8_t fc_mask = (uint8_t)((mask >> 8) & 0x3F);
        port[2] = (uint8_t)((port[2] & ~fc_mask) | (v[3] & fc_mask));
        if (mask & 0x0020) {
            port[3] = v[4];
        }
        if (mask & 0x0040) {
            port[4] = v[5];
        }
    }

    uint8_t rsp[8];
    rsp[0] = v[0];
    memcpy(&rsp[1], port, 5);
    floss_put_le16(&rsp[6], mask);
    floss_rfcomm_mux_send_mcc(mux, RFC_MCC_RPN, false, rsp, sizeof rsp);
}

static void rx_mcc_cmd(floss_rfcomm_mux_t *mux, uint8_t raw_type, const uint8_t *v, uint16_t len) {
    uint8_t type = (uint8_t)(raw_type & 0xFC);

    switch (type) {
    case RFC_MCC_PN:
        rx_pn_cmd(mux, v, len);
        break;
    case RFC_MCC_MSC:
        rx_msc_cmd(mux, v, len);
        break;
    case RFC_MCC_RPN:
        rx_rpn_cmd(mux, v, len);
        break;
    case RFC_MCC_RLS:
        /* Line status (overrun / parity / framing) is informational for a
         * virtual port: log it and acknowledge by echoing. */
        if (len >= 2 && (v[1] & 0x01)) {
            RFC_LOGI("DLCI %u line status error 0x%02X", v[0] >> 2, v[1] >> 1);
        }
        floss_rfcomm_mux_send_mcc(mux, RFC_MCC_RLS, false, v, len);
        break;
    case RFC_MCC_TEST:
        floss_rfcomm_mux_send_mcc(mux, RFC_MCC_TEST, false, v, len);
        break;
    case RFC_MCC_FCON:
        mux->peer_fcoff = false;
        floss_rfcomm_mux_send_mcc(mux, RFC_MCC_FCON, false, NULL, 0);
        floss_rfcomm_mux_kick(mux);
        break;
    case RFC_MCC_FCOFF:
        mux->peer_fcoff = true;
        floss_rfcomm_mux_send_mcc(mux, RFC_MCC_FCOFF, false, NULL, 0);
        break;
    default: {
        /* NSC carries the offending type octet, C/R bit included. */
        RFC_LOGD("unsupported MCC 0x%02X", raw_type);
        floss_rfcomm_mux_send_mcc(mux, RFC_MCC_NSC, false, &raw_type, 1);
        break;
    }
    }
}

static void rx_mcc_rsp(floss_rfcomm_mux_t *mux, uint8_t raw_type, const uint8_t *v, uint16_t len) {
    switch (raw_type & 0xFC) {
    case RFC_MCC_PN:
        rx_pn_rsp(mux, v, len);
        break;
    case RFC_MCC_NSC:
        RFC_LOGW("peer does not support MCC 0x%02X", len ? v[0] : 0);
        break;
    default:
        /* MSC / RPN / RLS / Test / FC acknowledgements need no action. */
        break;
    }
}

/** UIH on DLCI 0: one or more multiplexer control messages back to back. */
static void rx_mcc(floss_rfcomm_mux_t *mux, const uint8_t *p, uint16_t len) {
    if (mux->state != RFC_MUX_OPEN) {
        return;
    }
    while (len >= 2) {
        uint8_t type = p[0];
        uint16_t vlen;
        uint16_t hl;
        if (!(type & RFC_EA)) {
            RFC_LOGW("multi-octet MCC type ignored");
            return;
        }
        if (p[1] & RFC_EA) {
            vlen = (uint16_t)(p[1] >> 1);
            hl = 2;
        } else {
            if (len < 3) {
                return;
            }
            vlen = (uint16_t)((p[1] >> 1) | (p[2] << 7));
            hl = 3;
        }
        if ((uint32_t)hl + vlen > len) {
            RFC_LOGW("truncated MCC 0x%02X", type);
            return;
        }
        if (type & RFC_CR) {
            rx_mcc_cmd(mux, type, p + hl, vlen);
        } else {
            rx_mcc_rsp(mux, type, p + hl, vlen);
        }
        /* A handler may have torn the session down. */
        if (mux->state != RFC_MUX_OPEN) {
            return;
        }
        p += hl + vlen;
        len = (uint16_t)(len - hl - vlen);
    }
}

/* ------------------------------------------------------------------------ */
/* Unnumbered frames                                                        */
/* ------------------------------------------------------------------------ */

static void rx_sabm(floss_rfcomm_mux_t *mux, uint8_t dlci) {
    if (dlci == 0) {
        if (mux->state == RFC_MUX_IDLE || mux->state == RFC_MUX_OPEN) {
            send_u(mux, 0, RFC_UA, true);
            if (mux->state != RFC_MUX_OPEN) {
                mux->state = RFC_MUX_OPEN;
                RFC_LOGI("session open on link 0x%03X (responder)", mux->handle);
            }
        } else {
            /* Start-up collision or closing session: keep our own role. */
            send_u(mux, 0, RFC_DM, true);
        }
        return;
    }
    if (mux->state != RFC_MUX_OPEN) {
        send_u(mux, dlci, RFC_DM, true);
        return;
    }

    floss_rfcomm_dlc_t *dlc = floss_rfcomm_dlc_find(mux, dlci);
    if (dlc != NULL && dlc->state == FLOSS_RFCOMM_DLC_OPEN) {
        send_u(mux, dlci, RFC_UA, true); /* repeated SABM: our UA was lost */
        return;
    }
    const floss_rfcomm_server_t *srv = NULL;
    if (floss_rfcomm_dlci_is_local(mux, dlci)) {
        srv = floss_rfcomm_server_find((uint8_t)(dlci >> 1));
    }
    if (srv == NULL || (dlc != NULL && dlc->state != FLOSS_RFCOMM_DLC_NEGOTIATED)) {
        RFC_LOGI("SABM for DLCI %u refused", dlci);
        send_u(mux, dlci, RFC_DM, true);
        return;
    }
    if (dlc == NULL) {
        /* No PN: TS 07.10 defaults, and credits if the session already uses them. */
        dlc = floss_rfcomm_dlc_alloc(mux, dlci);
        if (dlc == NULL) {
            RFC_LOGW("no free DLC for DLCI %u", dlci);
            send_u(mux, dlci, RFC_DM, true);
            return;
        }
    }
    dlc->ops = srv->ops;
    dlc->user = srv->user;
    if (send_u(mux, dlci, RFC_UA, true) != FLOSS_OK) {
        floss_rfcomm_dlc_closed(dlc, FLOSS_RFCOMM_STATUS_OK);
        return;
    }
    dlc->state = FLOSS_RFCOMM_DLC_OPEN;
    floss_rfcomm_dlc_opened(dlc);
}

static void rx_disc(floss_rfcomm_mux_t *mux, uint8_t dlci) {
    if (dlci == 0) {
        send_u(mux, 0, RFC_UA, true);
        if (mux->state == RFC_MUX_FREE) {
            return;
        }
        RFC_LOGI("session closed by peer on link 0x%03X", mux->handle);
        if (mux->initiator) {
            mux->state = RFC_MUX_CLOSING;
            floss_rfcomm_mux_close_all(mux, FLOSS_RFCOMM_STATUS_LINK_LOST);
            if (mux->chan != NULL) {
                floss_l2cap_disconnect(mux->chan);
            }
        } else {
            /* The initiator owns the L2CAP channel: wait for it to close it. */
            mux->state = RFC_MUX_IDLE;
            mux->cfc = false;
            mux->peer_fcoff = false;
            floss_rfcomm_mux_close_all(mux, FLOSS_RFCOMM_STATUS_LINK_LOST);
        }
        return;
    }
    floss_rfcomm_dlc_t *dlc = floss_rfcomm_dlc_find(mux, dlci);
    if (dlc == NULL || dlc->state == FLOSS_RFCOMM_DLC_NEGOTIATED) {
        send_u(mux, dlci, RFC_DM, true);
        if (dlc != NULL) {
            floss_rfcomm_dlc_closed(dlc, FLOSS_RFCOMM_STATUS_OK);
        }
        return;
    }
    send_u(mux, dlci, RFC_UA, true);
    floss_rfcomm_dlc_closed(dlc, dlc->state == FLOSS_RFCOMM_DLC_WAIT_DISC ? FLOSS_RFCOMM_STATUS_OK
                                                                         : FLOSS_RFCOMM_STATUS_REFUSED);
}

static void rx_ua(floss_rfcomm_mux_t *mux, uint8_t dlci) {
    if (dlci == 0) {
        if (mux->state == RFC_MUX_WAIT_UA) {
            mux->state = RFC_MUX_OPEN;
            RFC_LOGI("session open on link 0x%03X (initiator)", mux->handle);
            floss_rfcomm_mux_started(mux);
        } else if (mux->state == RFC_MUX_CLOSING && mux->chan != NULL) {
            floss_l2cap_disconnect(mux->chan);
        }
        return;
    }
    floss_rfcomm_dlc_t *dlc = floss_rfcomm_dlc_find(mux, dlci);
    if (dlc == NULL) {
        return;
    }
    if (dlc->state == FLOSS_RFCOMM_DLC_WAIT_UA) {
        dlc->state = FLOSS_RFCOMM_DLC_OPEN;
        floss_rfcomm_dlc_opened(dlc);
    } else if (dlc->state == FLOSS_RFCOMM_DLC_WAIT_DISC) {
        floss_rfcomm_dlc_closed(dlc, FLOSS_RFCOMM_STATUS_OK);
    }
}

static void rx_dm(floss_rfcomm_mux_t *mux, uint8_t dlci) {
    if (dlci == 0) {
        if (mux->state == RFC_MUX_WAIT_UA || mux->state == RFC_MUX_CLOSING) {
            RFC_LOGW("session start refused on link 0x%03X", mux->handle);
            mux->state = RFC_MUX_CLOSING;
            floss_rfcomm_mux_close_all(mux, FLOSS_RFCOMM_STATUS_NO_MUX);
            if (mux->chan != NULL) {
                floss_l2cap_disconnect(mux->chan);
            }
        }
        return;
    }
    floss_rfcomm_dlc_t *dlc = floss_rfcomm_dlc_find(mux, dlci);
    if (dlc == NULL) {
        return;
    }
    floss_rfcomm_dlc_closed(dlc, dlc->state == FLOSS_RFCOMM_DLC_WAIT_DISC ? FLOSS_RFCOMM_STATUS_OK
                                                                         : FLOSS_RFCOMM_STATUS_REFUSED);
}

/* ------------------------------------------------------------------------ */
/* Frame receive                                                            */
/* ------------------------------------------------------------------------ */

/** Parse and dispatch one RFCOMM frame (= one L2CAP SDU). */
static void rx_frame(floss_rfcomm_mux_t *mux, const uint8_t *p, uint16_t len) {
    if (len < 4 || !(p[0] & RFC_EA)) {
        goto malformed;
    }
    uint8_t dlci = (uint8_t)(p[0] >> 2);
    uint8_t type = (uint8_t)(p[1] & ~RFC_PF);
    bool pf = (p[1] & RFC_PF) != 0;
    uint16_t info_len;
    uint16_t hl;

    if (p[2] & RFC_EA) {
        info_len = (uint16_t)(p[2] >> 1);
        hl = 3;
    } else {
        if (len < 5) {
            goto malformed;
        }
        info_len = (uint16_t)((p[2] >> 1) | (p[3] << 7));
        hl = 4;
    }
    uint16_t fcs_cover = type == RFC_UIH ? 2 : hl;

    /* Credit byte: UIH with P/F set on a DLC that uses credit based flow control. */
    int credits = -1;
    if (type == RFC_UIH && pf && dlci != 0) {
        const floss_rfcomm_dlc_t *dlc = floss_rfcomm_dlc_find(mux, dlci);
        if (dlc != NULL ? dlc->cfc : mux->cfc) {
            if (hl >= len - 1) {
                goto malformed;
            }
            credits = p[hl++];
        }
    }
    if ((uint32_t)hl + info_len + 1 > len) {
        goto malformed;
    }
    if (!fcs_ok(p, fcs_cover, p[hl + info_len])) {
        RFC_LOGW("FCS error on DLCI %u", dlci);
        FLOSS_STAT_INC(rx_dropped);
        return;
    }
    const uint8_t *info = p + hl;

    switch (type) {
    case RFC_SABM:
        rx_sabm(mux, dlci);
        break;
    case RFC_DISC:
        rx_disc(mux, dlci);
        break;
    case RFC_UA:
        rx_ua(mux, dlci);
        break;
    case RFC_DM:
        rx_dm(mux, dlci);
        break;
    case RFC_UIH:
        if (dlci == 0) {
            rx_mcc(mux, info, info_len);
        } else {
            floss_rfcomm_dlc_t *dlc = floss_rfcomm_dlc_find(mux, dlci);
            if (dlc != NULL &&
                (dlc->state == FLOSS_RFCOMM_DLC_OPEN || dlc->state == FLOSS_RFCOMM_DLC_WAIT_DISC)) {
                floss_rfcomm_dlc_rx(dlc, credits, info, info_len);
            } else if (info_len != 0) {
                RFC_LOGD("UIH on closed DLCI %u dropped", dlci);
            }
        }
        break;
    default:
        RFC_LOGW("unknown frame type 0x%02X", p[1]);
        break;
    }
    return;

malformed:
    RFC_LOGW("malformed frame (%u bytes)", len);
    FLOSS_STAT_INC(rx_dropped);
}

/* ------------------------------------------------------------------------ */
/* L2CAP glue                                                               */
/* ------------------------------------------------------------------------ */

/*
 * chan->user is the session: set by floss_rfcomm_connect() for outgoing
 * channels, and on open for incoming ones (registered with user = NULL).
 */
static void l2_on_open(floss_l2cap_chan_t *chan, uint16_t status) {
    floss_rfcomm_mux_t *mux = (floss_rfcomm_mux_t *)chan->user;

    if (mux == NULL) {
        if (status != 0) {
            return;
        }
        /* Incoming. One session per link: a second channel is a peer bug. */
        if (floss_rfcomm_mux_by_handle(chan->handle) != NULL || (mux = floss_rfcomm_mux_alloc()) == NULL) {
            RFC_LOGW("rejecting extra RFCOMM channel on link 0x%03X", chan->handle);
            floss_l2cap_disconnect(chan);
            return;
        }
        mux->chan = chan;
        mux->handle = chan->handle;
        mux->initiator = false;
        mux->state = RFC_MUX_IDLE;
        chan->user = mux;
        return;
    }

    if (status != 0) {
        RFC_LOGW("L2CAP connect failed (0x%04X)", status);
        chan->user = NULL;
        mux->state = RFC_MUX_CLOSING;
        floss_rfcomm_mux_close_all(mux, FLOSS_RFCOMM_STATUS_LINK_LOST);
        floss_rfcomm_mux_free(mux);
        return;
    }
    mux->chan = chan;
    mux->state = RFC_MUX_WAIT_UA;
    send_u(mux, 0, RFC_SABM, true);
}

static void l2_on_close(floss_l2cap_chan_t *chan) {
    floss_rfcomm_mux_t *mux = (floss_rfcomm_mux_t *)chan->user;
    if (mux == NULL) {
        return;
    }
    chan->user = NULL;
    mux->chan = NULL;
    mux->state = RFC_MUX_CLOSING; /* blocks new connects while DLCs are torn down */
    floss_rfcomm_mux_close_all(mux, FLOSS_RFCOMM_STATUS_LINK_LOST);
    floss_rfcomm_mux_free(mux);
}

static void l2_on_data(floss_l2cap_chan_t *chan, const uint8_t *data, uint16_t len) {
    floss_rfcomm_mux_t *mux = (floss_rfcomm_mux_t *)chan->user;
    if (mux != NULL) {
        rx_frame(mux, data, len);
    }
}

static void l2_on_can_send(floss_l2cap_chan_t *chan) {
    floss_rfcomm_mux_t *mux = (floss_rfcomm_mux_t *)chan->user;
    if (mux != NULL) {
        floss_rfcomm_mux_kick(mux);
    }
}

const floss_l2cap_ops_t floss_rfcomm_l2cap_ops = {
    .on_open = l2_on_open,
    .on_close = l2_on_close,
    .on_data = l2_on_data,
    .on_can_send = l2_on_can_send,
};

/** Initiator: the session is up, send SABM-preceding PNs for waiting DLCs. */
void floss_rfcomm_mux_started(floss_rfcomm_mux_t *mux) {
    for (uint8_t i = 0; i < FLOSS_RFCOMM_MAX_DLC; i++) {
        floss_rfcomm_dlc_t *dlc = &floss_rfcomm_ctx.dlc[i];
        if (dlc->mux == mux && dlc->state == FLOSS_RFCOMM_DLC_WAIT_MUX) {
            dlc->mtu = floss_rfcomm_mux_max_mtu(mux);
            dlc->state = FLOSS_RFCOMM_DLC_WAIT_PN;
            floss_rfcomm_mux_send_pn(mux, dlc, true);
        }
    }
}

/** Initiator: the last DLC is gone, so close DLCI 0 (then the L2CAP channel). */
void floss_rfcomm_mux_maybe_shutdown(floss_rfcomm_mux_t *mux) {
    if (mux == NULL || !mux->initiator || mux->state != RFC_MUX_OPEN) {
        return;
    }
    for (uint8_t i = 0; i < FLOSS_RFCOMM_MAX_DLC; i++) {
        if (floss_rfcomm_ctx.dlc[i].mux == mux) {
            return;
        }
    }
    mux->state = RFC_MUX_CLOSING;
    send_u(mux, 0, RFC_DISC, true);
}

#endif /* FLOSS_ENABLE_RFCOMM */
