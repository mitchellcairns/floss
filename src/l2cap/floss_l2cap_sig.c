/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_l2cap_sig.c - Classic L2CAP signaling channel and channel state machine.
 *
 * Overview
 * --------
 * Signaling travels on CID 0x0001 in C-frames. One C-frame may carry several
 * commands back to back, each with a 4-byte header: code, identifier, length.
 * The identifier pairs a response with its request; we pick ours from a
 * rolling 1..255 counter (0 is reserved by the protocol) and store it in
 * chan->sig_id while a request is outstanding.
 *
 * Channel lifecycle (Basic mode only):
 *
 *   incoming                             outgoing (floss_l2cap_connect)
 *   Connection Request                     |
 *     | PSM needs encryption,          WAIT_SECURITY (if PSM needs encryption)
 *     | link not encrypted                 | link encrypted
 *     v -> rsp "pending", WAIT_SECURITY    v
 *     | link encrypted                 Connection Request -> WAIT_CONN_RSP
 *     v                                    | rsp success (pending: keep waiting)
 *   rsp success ----------> CONFIG <-------+
 *                             | send our Configuration Request at once;
 *                             | answer theirs. OUT_DONE + IN_DONE ->
 *                             v
 *                            OPEN --(Disconnection Req/Rsp)--> closed
 *
 * Configuration is two independent half-handshakes: each side states what it
 * can RECEIVE (MTU) and the other side accepts or counter-proposes. The
 * channel is open once our request was accepted (CFG_OUT_DONE) and we
 * accepted theirs (CFG_IN_DONE), in whichever order those happen.
 *
 * Feature claims are deliberately honest: Basic mode only, no connectionless
 * data, no enhanced retransmission/streaming. A peer asking for another mode
 * is told "unacceptable, use Basic", which every conforming peer accepts.
 *
 * Timers: one per channel, armed for whatever we currently wait on (response
 * RTX, extended ERTX after "pending", security, the peer's config request).
 */
#include "floss_l2cap_priv.h"

static uint8_t sig_next_id;

/** Max data we echo back: keeps the whole C-frame within the minimum 48-byte signaling MTU. */
#define SIG_ECHO_MAX 44

void floss_l2cap_sig_init(void)
{
    sig_next_id = 1;
}

static uint8_t sig_alloc_id(void)
{
    uint8_t id = sig_next_id;
    sig_next_id = (uint8_t)(sig_next_id == 0xFF ? 1 : sig_next_id + 1);
    return id;
}

/* ------------------------------------------------------------------------ */
/* Sending                                                                  */
/* ------------------------------------------------------------------------ */

/**
 * Build one signaling command in a pool buffer and hand it to HCI.
 * Signaling is never marked flushable: losing a command stalls the channel
 * until RTX expires, which is far worse than a little latency.
 */
static floss_err_t sig_send(uint16_t handle, uint8_t code, uint8_t id, const uint8_t *data, uint16_t len)
{
    floss_buf_t *buf = floss_buf_alloc();
    if (!buf) {
        FLOSS_STAT_INC(buf_alloc_failures);
        FLOSS_LOGW(L2CAP, "no buffer for signaling code 0x%02x", code);
        return FLOSS_ERR_NOMEM;
    }
    uint8_t *p = floss_buf_put(buf, (uint16_t)(FLOSS_L2CAP_HDR_LEN + 4 + len));
    if (!p) {
        floss_buf_free(buf);
        return FLOSS_ERR_INVALID;
    }
    floss_put_le16(p, (uint16_t)(4 + len));
    floss_put_le16(p + 2, FLOSS_CID_SIGNALING);
    p[4] = code;
    p[5] = id;
    floss_put_le16(p + 6, len);
    if (len) {
        memcpy(p + 8, data, len);
    }
    buf->handle = handle;
    buf->flags = 0;
    FLOSS_TRACE(FLOSS_TR_L2CAP_SIG, ((uint32_t)code << 16) | id);
    FLOSS_LOGD(L2CAP, "sig tx code 0x%02x id %u len %u", code, id, len);
    return floss_hci_acl_send(buf);
}

static void sig_send_reject(uint16_t handle, uint8_t id, uint16_t reason, uint16_t cid_a, uint16_t cid_b)
{
    uint8_t d[6];
    uint16_t n = 2;
    floss_put_le16(d, reason);
    if (reason == FLOSS_L2CAP_REJ_INVALID_CID) {
        floss_put_le16(d + 2, cid_a);
        floss_put_le16(d + 4, cid_b);
        n = 6;
    }
    sig_send(handle, FLOSS_L2CAP_SIG_CMD_REJECT, id, d, n);
}

/** Connection Response: dcid = our CID (0 on refusal), scid = the requester's CID. */
static void sig_send_conn_rsp(uint16_t handle, uint8_t id, uint16_t dcid, uint16_t scid, uint16_t result,
                              uint16_t status)
{
    uint8_t d[8];
    floss_put_le16(d, dcid);
    floss_put_le16(d + 2, scid);
    floss_put_le16(d + 4, result);
    floss_put_le16(d + 6, status);
    sig_send(handle, FLOSS_L2CAP_SIG_CONN_RSP, id, d, sizeof(d));
}

static void sig_send_config_rsp(uint16_t handle, uint8_t id, uint16_t scid, uint16_t flags, uint16_t result,
                                const uint8_t *opts, uint8_t opts_len)
{
    uint8_t d[6 + FLOSS_L2CAP_CFG_RSP_OPTS_MAX];
    floss_put_le16(d, scid);
    floss_put_le16(d + 2, flags);
    floss_put_le16(d + 4, result);
    if (opts_len) {
        memcpy(d + 6, opts, opts_len);
    }
    sig_send(handle, FLOSS_L2CAP_SIG_CONFIG_RSP, id, d, (uint16_t)(6 + opts_len));
}

/**
 * Our Configuration Request. We only state our receive MTU; everything else
 * (flush timeout infinite, best-effort QoS, Basic mode) is the default, and
 * omitting defaults keeps old peers happy.
 */
static void sig_send_config_req(floss_l2cap_slot_t *s)
{
    floss_l2cap_chan_t *chan = &s->chan;
    uint8_t d[8];
    floss_put_le16(d, chan->remote_cid);
    floss_put_le16(d + 2, 0); /* flags: no continuation */
    d[4] = FLOSS_L2CAP_OPT_MTU;
    d[5] = 2;
    floss_put_le16(d + 6, chan->local_mtu);
    chan->sig_id = sig_alloc_id();
    floss_timer_start(&s->timer, FLOSS_L2CAP_RTX_US);
    sig_send(chan->handle, FLOSS_L2CAP_SIG_CONFIG_REQ, chan->sig_id, d, sizeof(d));
}

/** Disconnection Request without any state change (used when abandoning a channel). */
static void sig_send_disc_req_raw(floss_l2cap_slot_t *s)
{
    uint8_t d[4];
    floss_put_le16(d, s->chan.remote_cid);
    floss_put_le16(d + 2, s->chan.local_cid);
    s->chan.sig_id = sig_alloc_id();
    sig_send(s->chan.handle, FLOSS_L2CAP_SIG_DISC_REQ, s->chan.sig_id, d, sizeof(d));
}

void floss_l2cap_sig_disconnect(floss_l2cap_slot_t *s)
{
    s->chan.state = FLOSS_L2CAP_WAIT_DISC_RSP;
    /* Arm before sending: if no buffer is available the RTX expiry still
     * completes the teardown locally. */
    floss_timer_start(&s->timer, FLOSS_L2CAP_RTX_US);
    sig_send_disc_req_raw(s);
}

floss_err_t floss_l2cap_sig_connect(floss_l2cap_slot_t *s)
{
    floss_l2cap_chan_t *chan = &s->chan;
    uint8_t d[4];
    floss_put_le16(d, chan->psm);
    floss_put_le16(d + 2, chan->local_cid);
    chan->state = FLOSS_L2CAP_WAIT_CONN_RSP;
    chan->sig_id = sig_alloc_id();
    floss_err_t err = sig_send(chan->handle, FLOSS_L2CAP_SIG_CONN_REQ, chan->sig_id, d, sizeof(d));
    if (err == FLOSS_OK) {
        floss_timer_start(&s->timer, FLOSS_L2CAP_RTX_US);
    }
    FLOSS_LOGD(L2CAP, "connect psm 0x%04x lcid 0x%04x", chan->psm, chan->local_cid);
    return err;
}

void floss_l2cap_sig_refuse_pending(floss_l2cap_slot_t *s, uint16_t result)
{
    sig_send_conn_rsp(s->chan.handle, s->peer_req_id, 0, s->chan.remote_cid, result, FLOSS_L2CAP_CONN_STATUS_NONE);
}

/* ------------------------------------------------------------------------ */
/* State machine helpers                                                    */
/* ------------------------------------------------------------------------ */

static void sig_enter_config(floss_l2cap_slot_t *s)
{
    s->chan.state = FLOSS_L2CAP_CONFIG;
    s->chan.cfg_flags = 0;
    s->cfg_retries = 0;
    sig_send_config_req(s);
}

/** Incoming channel accepted (security satisfied): confirm, then configure. */
static void sig_accept_incoming(floss_l2cap_slot_t *s)
{
    sig_send_conn_rsp(s->chan.handle, s->peer_req_id, s->chan.local_cid, s->chan.remote_cid,
                      FLOSS_L2CAP_CONN_SUCCESS, FLOSS_L2CAP_CONN_STATUS_NONE);
    sig_enter_config(s);
}

static void sig_check_open(floss_l2cap_slot_t *s)
{
    floss_l2cap_chan_t *chan = &s->chan;
    const uint8_t both = FLOSS_L2CAP_CFG_OUT_DONE | FLOSS_L2CAP_CFG_IN_DONE;
    if (chan->state != FLOSS_L2CAP_CONFIG || (chan->cfg_flags & both) != both) {
        return;
    }
    floss_timer_stop(&s->timer);
    chan->state = FLOSS_L2CAP_OPEN;
    if (s->opened) {
        return; /* reconfiguration of an already open channel */
    }
    s->opened = true;
    FLOSS_LOGI(L2CAP, "open psm 0x%04x lcid 0x%04x rcid 0x%04x mtu in %u out %u%s", chan->psm, chan->local_cid,
               chan->remote_cid, chan->local_mtu, chan->remote_mtu, chan->initiator ? " (out)" : " (in)");
    if (chan->ops && chan->ops->on_open) {
        chan->ops->on_open(chan, 0);
    }
}

/** Configuration cannot converge: disconnect politely, report `status` once done. */
static void sig_fail_config(floss_l2cap_slot_t *s, uint16_t status)
{
    FLOSS_LOGW(L2CAP, "config failed lcid 0x%04x status 0x%04x", s->chan.local_cid, status);
    s->fail_status = status;
    floss_l2cap_sig_disconnect(s);
}

/* ------------------------------------------------------------------------ */
/* Configuration option parsing                                             */
/* ------------------------------------------------------------------------ */

/*
 * Results for the peer's request accumulate across continuation segments.
 * Precedence: unknown options > rejected > unacceptable > success. Only the
 * options relevant to the winning result are echoed back.
 */
static void cfg_note_unknown(floss_l2cap_slot_t *s, uint8_t type)
{
    if (s->cfg_in_result != FLOSS_L2CAP_CFG_UNKNOWN) {
        s->cfg_in_result = FLOSS_L2CAP_CFG_UNKNOWN;
        s->cfg_in_opts_len = 0;
    }
    /* The unknown-options response lists the offending option types. */
    if (s->cfg_in_opts_len < FLOSS_L2CAP_CFG_RSP_OPTS_MAX) {
        s->cfg_in_opts[s->cfg_in_opts_len++] = type;
    }
}

static void cfg_note_rejected(floss_l2cap_slot_t *s)
{
    if (s->cfg_in_result != FLOSS_L2CAP_CFG_UNKNOWN) {
        s->cfg_in_result = FLOSS_L2CAP_CFG_REJECTED;
        s->cfg_in_opts_len = 0;
    }
}

/** Counter-propose an option value (type/len/value as it must appear on the wire). */
static void cfg_note_unacceptable(floss_l2cap_slot_t *s, const uint8_t *opt, uint8_t opt_len)
{
    if (s->cfg_in_result == FLOSS_L2CAP_CFG_SUCCESS) {
        s->cfg_in_result = FLOSS_L2CAP_CFG_UNACCEPTABLE;
        s->cfg_in_opts_len = 0;
    }
    if (s->cfg_in_result != FLOSS_L2CAP_CFG_UNACCEPTABLE) {
        return;
    }
    if (s->cfg_in_opts_len + opt_len <= FLOSS_L2CAP_CFG_RSP_OPTS_MAX) {
        memcpy(&s->cfg_in_opts[s->cfg_in_opts_len], opt, opt_len);
        s->cfg_in_opts_len = (uint8_t)(s->cfg_in_opts_len + opt_len);
    }
}

/** Walk the options of one Configuration Request segment from the peer. */
static void cfg_parse_peer_options(floss_l2cap_slot_t *s, const uint8_t *p, uint16_t len)
{
    floss_l2cap_chan_t *chan = &s->chan;
    uint16_t off = 0;
    while (off < len) {
        if (len - off < 2 || (uint16_t)(off + 2 + p[off + 1]) > len) {
            FLOSS_LOGD(L2CAP, "malformed config option at %u", off);
            cfg_note_rejected(s);
            return;
        }
        uint8_t type = p[off];
        uint8_t olen = p[off + 1];
        const uint8_t *v = p + off + 2;
        off = (uint16_t)(off + 2 + olen);

        switch (type & (uint8_t)~FLOSS_L2CAP_OPT_HINT) {
        case FLOSS_L2CAP_OPT_MTU: {
            if (olen != 2) {
                cfg_note_rejected(s);
                break;
            }
            uint16_t mtu = floss_get_le16(v);
            if (mtu < FLOSS_L2CAP_MIN_MTU) {
                uint8_t opt[4] = {FLOSS_L2CAP_OPT_MTU, 2, 0, 0};
                floss_put_le16(opt + 2, FLOSS_L2CAP_MIN_MTU);
                cfg_note_unacceptable(s, opt, sizeof(opt));
            } else {
                chan->remote_mtu = mtu;
            }
            break;
        }
        case FLOSS_L2CAP_OPT_FLUSH_TO:
            if (olen != 2) {
                cfg_note_rejected(s);
                break;
            }
            chan->flush_timeout = floss_get_le16(v);
            break;
        case FLOSS_L2CAP_OPT_QOS:
            /* Best effort is all a baseband link gives us anyway; accept. */
            break;
        case FLOSS_L2CAP_OPT_RFC:
            if (olen < 1) {
                cfg_note_rejected(s);
                break;
            }
            if (v[0] != 0x00) {
                /* Only Basic mode: counter-propose it with all other fields zero. */
                uint8_t opt[11] = {FLOSS_L2CAP_OPT_RFC, 9, 0x00};
                cfg_note_unacceptable(s, opt, sizeof(opt));
            }
            break;
        case FLOSS_L2CAP_OPT_FCS:
        case FLOSS_L2CAP_OPT_EFS:
        case FLOSS_L2CAP_OPT_EWS:
            /* Understood, but they only apply to the retransmission modes. In
             * Basic mode they have no effect, so accepting them is correct and
             * friendlier than rejecting a peer that sends them unconditionally. */
            break;
        default:
            if (!(type & FLOSS_L2CAP_OPT_HINT)) {
                cfg_note_unknown(s, type);
            }
            break;
        }
    }
}

/**
 * The peer answered our request with "unacceptable parameters" and its own
 * proposals. Adopt them if we can live with them; false = give up.
 */
static bool cfg_adopt_peer_proposal(floss_l2cap_slot_t *s, const uint8_t *p, uint16_t len)
{
    uint16_t off = 0;
    while (off + 2 <= len) {
        uint8_t type = p[off] & (uint8_t)~FLOSS_L2CAP_OPT_HINT;
        uint8_t olen = p[off + 1];
        const uint8_t *v = p + off + 2;
        if ((uint16_t)(off + 2 + olen) > len) {
            return false;
        }
        off = (uint16_t)(off + 2 + olen);
        if (type == FLOSS_L2CAP_OPT_MTU && olen == 2) {
            uint16_t mtu = floss_get_le16(v);
            if (mtu < FLOSS_L2CAP_MIN_MTU || mtu > FLOSS_L2CAP_MTU) {
                return false;
            }
            s->chan.local_mtu = mtu;
        } else if (type == FLOSS_L2CAP_OPT_RFC && olen >= 1) {
            if (v[0] != 0x00) {
                return false; /* we cannot do anything but Basic mode */
            }
        } else if (type == FLOSS_L2CAP_OPT_FLUSH_TO || type == FLOSS_L2CAP_OPT_QOS) {
            /* We never requested these; nothing to adopt. */
        } else {
            return false;
        }
    }
    return true;
}

/* ------------------------------------------------------------------------ */
/* Command handlers                                                         */
/* ------------------------------------------------------------------------ */

static void sig_on_conn_req(uint16_t handle, uint8_t id, const uint8_t *d, uint16_t len)
{
    if (len < 4) {
        sig_send_reject(handle, id, FLOSS_L2CAP_REJ_NOT_UNDERSTOOD, 0, 0);
        return;
    }
    uint16_t psm = floss_get_le16(d);
    uint16_t scid = floss_get_le16(d + 2);
    const floss_l2cap_server_t *srv = floss_l2cap_server_find(psm);
    if (!srv) {
        FLOSS_LOGI(L2CAP, "incoming psm 0x%04x not registered", psm);
        sig_send_conn_rsp(handle, id, 0, scid, FLOSS_L2CAP_CONN_PSM_UNSUPP, 0);
        return;
    }
    if (scid < FLOSS_CID_DYNAMIC_FIRST) {
        sig_send_conn_rsp(handle, id, 0, scid, FLOSS_L2CAP_CONN_INVALID_SCID, 0);
        return;
    }
    if (floss_l2cap_slot_by_remote_cid(handle, scid)) {
        sig_send_conn_rsp(handle, id, 0, scid, FLOSS_L2CAP_CONN_SCID_IN_USE, 0);
        return;
    }
    floss_l2cap_slot_t *s = floss_l2cap_slot_alloc(handle);
    if (!s) {
        FLOSS_LOGW(L2CAP, "no free channel for psm 0x%04x", psm);
        sig_send_conn_rsp(handle, id, 0, scid, FLOSS_L2CAP_CONN_NO_RESOURCES, 0);
        return;
    }
    floss_l2cap_chan_t *chan = &s->chan;
    chan->psm = psm;
    chan->remote_cid = scid;
    chan->ops = srv->ops;
    chan->user = srv->user;
    chan->initiator = false;
    s->sec = srv->sec;
    s->peer_req_id = id;

    const floss_conn_t *conn = floss_hci_conn_by_handle(handle);
    if (srv->sec == FLOSS_SEC_ENCRYPTED && !(conn && conn->encrypted)) {
        /* Park the request: tell the peer we are authenticating so it extends
         * its own timeout, and ask GAP to secure the link. */
        chan->state = FLOSS_L2CAP_WAIT_SECURITY;
        floss_timer_start(&s->timer, FLOSS_L2CAP_SECURITY_TIMEOUT_US);
        sig_send_conn_rsp(handle, id, chan->local_cid, scid, FLOSS_L2CAP_CONN_PENDING,
                          FLOSS_L2CAP_CONN_STATUS_AUTH_PEND);
        FLOSS_LOGD(L2CAP, "incoming psm 0x%04x lcid 0x%04x: waiting for security", psm, chan->local_cid);
        floss_gap_require_security(handle);
        return;
    }
    sig_accept_incoming(s);
}

static void sig_on_conn_rsp(uint16_t handle, uint8_t id, const uint8_t *d, uint16_t len)
{
    if (len < 8) {
        return;
    }
    uint16_t dcid = floss_get_le16(d);
    uint16_t scid = floss_get_le16(d + 2);
    uint16_t result = floss_get_le16(d + 4);
    uint16_t status = floss_get_le16(d + 6);

    floss_l2cap_slot_t *s = NULL;
    for (unsigned i = 0; i < FLOSS_MAX_L2CAP_CHANNELS; i++) {
        floss_l2cap_slot_t *c = &floss_l2cap_slots[i];
        /* Some peers put 0 in the SCID field of a refusal: match on id then. */
        if (c->in_use && !c->finishing && c->chan.handle == handle && c->chan.state == FLOSS_L2CAP_WAIT_CONN_RSP &&
            c->chan.sig_id == id && (scid == c->chan.local_cid || scid == 0)) {
            s = c;
            break;
        }
    }
    if (!s) {
        FLOSS_LOGD(L2CAP, "unexpected connection response id %u", id);
        return;
    }
    switch (result) {
    case FLOSS_L2CAP_CONN_SUCCESS:
        if (dcid < FLOSS_CID_DYNAMIC_FIRST) {
            floss_l2cap_slot_finish(s, FLOSS_L2CAP_STATUS_PROTOCOL);
            return;
        }
        s->chan.remote_cid = dcid;
        s->chan.sig_id = 0;
        if (s->user_closing) {
            floss_l2cap_sig_disconnect(s);
        } else {
            sig_enter_config(s);
        }
        break;
    case FLOSS_L2CAP_CONN_PENDING:
        /* Peer is authenticating/authorising: switch to the extended timeout. */
        FLOSS_LOGD(L2CAP, "connection pending lcid 0x%04x status %u", s->chan.local_cid, status);
        floss_timer_start(&s->timer, FLOSS_L2CAP_ERTX_US);
        break;
    default:
        FLOSS_LOGI(L2CAP, "connection refused psm 0x%04x result 0x%04x", s->chan.psm, result);
        floss_l2cap_slot_finish(s, result);
        break;
    }
}

static void sig_on_config_req(uint16_t handle, uint8_t id, const uint8_t *d, uint16_t len)
{
    if (len < 4) {
        sig_send_reject(handle, id, FLOSS_L2CAP_REJ_NOT_UNDERSTOOD, 0, 0);
        return;
    }
    uint16_t dcid = floss_get_le16(d);
    uint16_t flags = floss_get_le16(d + 2);
    floss_l2cap_slot_t *s = floss_l2cap_slot_by_local_cid(handle, dcid);
    if (!s || (s->chan.state != FLOSS_L2CAP_CONFIG && s->chan.state != FLOSS_L2CAP_OPEN)) {
        sig_send_reject(handle, id, FLOSS_L2CAP_REJ_INVALID_CID, dcid, 0);
        return;
    }
    floss_l2cap_chan_t *chan = &s->chan;
    if (!(chan->cfg_flags & FLOSS_L2CAP_CFG_IN_CONT)) {
        s->cfg_in_result = FLOSS_L2CAP_CFG_SUCCESS;
        s->cfg_in_opts_len = 0;
    }
    cfg_parse_peer_options(s, d + 4, (uint16_t)(len - 4));

    if (flags & 0x0001) {
        /* Continuation: acknowledge this segment, judge the whole at the end. */
        chan->cfg_flags |= FLOSS_L2CAP_CFG_IN_CONT;
        sig_send_config_rsp(handle, id, chan->remote_cid, 0x0001, FLOSS_L2CAP_CFG_SUCCESS, NULL, 0);
        return;
    }
    chan->cfg_flags &= (uint8_t)~FLOSS_L2CAP_CFG_IN_CONT;
    sig_send_config_rsp(handle, id, chan->remote_cid, 0, s->cfg_in_result, s->cfg_in_opts, s->cfg_in_opts_len);
    if (s->cfg_in_result == FLOSS_L2CAP_CFG_SUCCESS) {
        chan->cfg_flags |= FLOSS_L2CAP_CFG_IN_DONE;
        sig_check_open(s);
    } else {
        FLOSS_LOGD(L2CAP, "config req lcid 0x%04x answered with result %u", chan->local_cid, s->cfg_in_result);
    }
}

static void sig_on_config_rsp(uint16_t handle, uint8_t id, const uint8_t *d, uint16_t len)
{
    if (len < 6) {
        return;
    }
    uint16_t scid = floss_get_le16(d);
    uint16_t result = floss_get_le16(d + 4);
    floss_l2cap_slot_t *s = floss_l2cap_slot_by_local_cid(handle, scid);
    if (!s || s->chan.state != FLOSS_L2CAP_CONFIG || s->chan.sig_id != id) {
        FLOSS_LOGD(L2CAP, "unexpected config response id %u cid 0x%04x", id, scid);
        return;
    }
    floss_l2cap_chan_t *chan = &s->chan;
    switch (result) {
    case FLOSS_L2CAP_CFG_SUCCESS:
        chan->sig_id = 0;
        chan->cfg_flags |= FLOSS_L2CAP_CFG_OUT_DONE;
        if (!(chan->cfg_flags & FLOSS_L2CAP_CFG_IN_DONE)) {
            floss_timer_start(&s->timer, FLOSS_L2CAP_CONFIG_TIMEOUT_US);
        }
        sig_check_open(s);
        break;
    case FLOSS_L2CAP_CFG_PENDING:
        floss_timer_start(&s->timer, FLOSS_L2CAP_ERTX_US);
        break;
    case FLOSS_L2CAP_CFG_UNACCEPTABLE:
        if (++s->cfg_retries <= 2 && cfg_adopt_peer_proposal(s, d + 6, (uint16_t)(len - 6))) {
            sig_send_config_req(s);
        } else {
            sig_fail_config(s, FLOSS_L2CAP_STATUS_CONFIG_FAILED);
        }
        break;
    default:
        sig_fail_config(s, FLOSS_L2CAP_STATUS_CONFIG_FAILED);
        break;
    }
}

static void sig_on_disc_req(uint16_t handle, uint8_t id, const uint8_t *d, uint16_t len)
{
    if (len < 4) {
        sig_send_reject(handle, id, FLOSS_L2CAP_REJ_NOT_UNDERSTOOD, 0, 0);
        return;
    }
    uint16_t dcid = floss_get_le16(d); /* ours */
    uint16_t scid = floss_get_le16(d + 2); /* theirs */
    floss_l2cap_slot_t *s = floss_l2cap_slot_by_local_cid(handle, dcid);
    if (!s || s->chan.remote_cid != scid || s->chan.state == FLOSS_L2CAP_WAIT_SECURITY ||
        s->chan.state == FLOSS_L2CAP_WAIT_CONN_RSP) {
        sig_send_reject(handle, id, FLOSS_L2CAP_REJ_INVALID_CID, dcid, scid);
        return;
    }
    uint8_t rsp[4];
    floss_put_le16(rsp, dcid);
    floss_put_le16(rsp + 2, scid);
    sig_send(handle, FLOSS_L2CAP_SIG_DISC_RSP, id, rsp, sizeof(rsp));
    floss_l2cap_slot_finish(s, s->fail_status ? s->fail_status : FLOSS_L2CAP_STATUS_REMOTE_CLOSED);
}

static void sig_on_disc_rsp(uint16_t handle, uint8_t id, const uint8_t *d, uint16_t len)
{
    if (len < 4) {
        return;
    }
    uint16_t scid = floss_get_le16(d + 2); /* ours */
    floss_l2cap_slot_t *s = floss_l2cap_slot_by_local_cid(handle, scid);
    if (!s || s->chan.state != FLOSS_L2CAP_WAIT_DISC_RSP) {
        FLOSS_LOGD(L2CAP, "unexpected disconnection response cid 0x%04x", scid);
        return;
    }
    floss_l2cap_slot_finish(s, s->fail_status);
}

static void sig_on_cmd_reject(uint16_t handle, uint8_t id, const uint8_t *d, uint16_t len)
{
    uint16_t reason = len >= 2 ? floss_get_le16(d) : 0xFFFF;
    for (unsigned i = 0; i < FLOSS_MAX_L2CAP_CHANNELS; i++) {
        floss_l2cap_slot_t *s = &floss_l2cap_slots[i];
        if (!s->in_use || s->finishing || s->chan.handle != handle || s->chan.sig_id != id) {
            continue;
        }
        FLOSS_LOGW(L2CAP, "command rejected lcid 0x%04x reason %u", s->chan.local_cid, reason);
        switch (s->chan.state) {
        case FLOSS_L2CAP_WAIT_CONN_RSP:
            floss_l2cap_slot_finish(s, FLOSS_L2CAP_STATUS_REJECTED);
            break;
        case FLOSS_L2CAP_CONFIG:
            sig_fail_config(s, FLOSS_L2CAP_STATUS_REJECTED);
            break;
        case FLOSS_L2CAP_WAIT_DISC_RSP:
            /* Peer does not know the channel any more: it is gone either way. */
            floss_l2cap_slot_finish(s, s->fail_status);
            break;
        default:
            break;
        }
        return;
    }
    FLOSS_LOGD(L2CAP, "command reject id %u reason %u (no pending request)", id, reason);
}

static void sig_on_echo_req(uint16_t handle, uint8_t id, const uint8_t *d, uint16_t len)
{
    sig_send(handle, FLOSS_L2CAP_SIG_ECHO_RSP, id, d, FLOSS_MIN(len, SIG_ECHO_MAX));
}

static void sig_on_info_req(uint16_t handle, uint8_t id, const uint8_t *d, uint16_t len)
{
    if (len < 2) {
        sig_send_reject(handle, id, FLOSS_L2CAP_REJ_NOT_UNDERSTOOD, 0, 0);
        return;
    }
    uint16_t type = floss_get_le16(d);
    uint8_t rsp[12];
    uint16_t n = 4;
    floss_put_le16(rsp, type);
    floss_put_le16(rsp + 2, FLOSS_L2CAP_INFO_SUCCESS);
    switch (type) {
    case FLOSS_L2CAP_INFO_EXT_FEATURES:
        /* Only "fixed channels supported" (bit 7): no ERTM, streaming, FCS
         * option, extended flow spec or extended window. */
        floss_put_le32(rsp + 4, 0x00000080u);
        n = 8;
        break;
    case FLOSS_L2CAP_INFO_FIXED_CHANS:
        /* Bit 1: the signaling channel. No connectionless channel. */
        memset(rsp + 4, 0, 8);
        rsp[4] = 0x02;
        n = 12;
        break;
    default: /* includes connectionless MTU: we have no connectionless channel */
        floss_put_le16(rsp + 2, FLOSS_L2CAP_INFO_NOT_SUPPORTED);
        break;
    }
    sig_send(handle, FLOSS_L2CAP_SIG_INFO_RSP, id, rsp, n);
}

/* ------------------------------------------------------------------------ */
/* Entry points                                                             */
/* ------------------------------------------------------------------------ */

void floss_l2cap_sig_on_frame(uint16_t handle, const uint8_t *data, uint16_t len)
{
    uint16_t off = 0;
    while (len - off >= 4) {
        uint8_t code = data[off];
        uint8_t id = data[off + 1];
        uint16_t clen = floss_get_le16(data + off + 2);
        if (clen > len - off - 4) {
            FLOSS_LOGD(L2CAP, "truncated signaling command code 0x%02x", code);
            FLOSS_STAT_INC(rx_dropped);
            return;
        }
        const uint8_t *d = data + off + 4;
        off = (uint16_t)(off + 4 + clen);

        FLOSS_TRACE(FLOSS_TR_L2CAP_SIG, ((uint32_t)code << 16) | id);
        FLOSS_LOGD(L2CAP, "sig rx code 0x%02x id %u len %u", code, id, clen);
        if (id == 0) {
            continue; /* identifier 0 is illegal: ignore the command */
        }
        switch (code) {
        case FLOSS_L2CAP_SIG_CMD_REJECT: sig_on_cmd_reject(handle, id, d, clen); break;
        case FLOSS_L2CAP_SIG_CONN_REQ:   sig_on_conn_req(handle, id, d, clen); break;
        case FLOSS_L2CAP_SIG_CONN_RSP:   sig_on_conn_rsp(handle, id, d, clen); break;
        case FLOSS_L2CAP_SIG_CONFIG_REQ: sig_on_config_req(handle, id, d, clen); break;
        case FLOSS_L2CAP_SIG_CONFIG_RSP: sig_on_config_rsp(handle, id, d, clen); break;
        case FLOSS_L2CAP_SIG_DISC_REQ:   sig_on_disc_req(handle, id, d, clen); break;
        case FLOSS_L2CAP_SIG_DISC_RSP:   sig_on_disc_rsp(handle, id, d, clen); break;
        case FLOSS_L2CAP_SIG_ECHO_REQ:   sig_on_echo_req(handle, id, d, clen); break;
        case FLOSS_L2CAP_SIG_ECHO_RSP:   break;
        case FLOSS_L2CAP_SIG_INFO_REQ:   sig_on_info_req(handle, id, d, clen); break;
        case FLOSS_L2CAP_SIG_INFO_RSP:   break; /* we never ask */
        default:
            sig_send_reject(handle, id, FLOSS_L2CAP_REJ_NOT_UNDERSTOOD, 0, 0);
            break;
        }
    }
}

void floss_l2cap_on_link_security(uint16_t handle, uint8_t status, bool encrypted)
{
    for (unsigned i = 0; i < FLOSS_MAX_L2CAP_CHANNELS; i++) {
        floss_l2cap_slot_t *s = &floss_l2cap_slots[i];
        if (!s->in_use || s->finishing || s->chan.handle != handle || s->chan.state != FLOSS_L2CAP_WAIT_SECURITY) {
            continue;
        }
        if (status == 0 && encrypted) {
            floss_timer_stop(&s->timer);
            if (s->chan.initiator) {
                if (floss_l2cap_sig_connect(s) != FLOSS_OK) {
                    floss_l2cap_slot_finish(s, FLOSS_L2CAP_CONN_NO_RESOURCES);
                }
            } else {
                sig_accept_incoming(s);
            }
        } else if (status != 0) {
            FLOSS_LOGW(L2CAP, "security failed (0x%02x) for psm 0x%04x", status, s->chan.psm);
            if (!s->chan.initiator) {
                floss_l2cap_sig_refuse_pending(s, FLOSS_L2CAP_CONN_SECURITY_BLOCK);
            }
            floss_l2cap_slot_finish(s, FLOSS_L2CAP_CONN_SECURITY_BLOCK);
        }
        /* status 0 but not (yet) encrypted: authentication done, keep waiting. */
    }
}

void floss_l2cap_sig_timeout(floss_timer_t *t, void *arg)
{
    floss_l2cap_slot_t *s = (floss_l2cap_slot_t *)arg;
    if (!s->in_use || s->finishing) {
        return;
    }
    FLOSS_LOGW(L2CAP, "timeout lcid 0x%04x state %u", s->chan.local_cid, (unsigned)s->chan.state);
    switch (s->chan.state) {
    case FLOSS_L2CAP_WAIT_SECURITY:
        if (!s->chan.initiator) {
            floss_l2cap_sig_refuse_pending(s, FLOSS_L2CAP_CONN_SECURITY_BLOCK);
        }
        floss_l2cap_slot_finish(s, FLOSS_L2CAP_STATUS_TIMEOUT);
        break;
    case FLOSS_L2CAP_WAIT_CONN_RSP:
        floss_l2cap_slot_finish(s, FLOSS_L2CAP_STATUS_TIMEOUT);
        break;
    case FLOSS_L2CAP_CONFIG:
        /* Peer stopped answering: tell it (best effort) and give up now. */
        sig_send_disc_req_raw(s);
        floss_l2cap_slot_finish(s, FLOSS_L2CAP_STATUS_TIMEOUT);
        break;
    case FLOSS_L2CAP_WAIT_DISC_RSP:
        floss_l2cap_slot_finish(s, s->fail_status ? s->fail_status : FLOSS_L2CAP_STATUS_TIMEOUT);
        break;
    default:
        break;
    }
}
