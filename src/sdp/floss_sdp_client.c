/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_sdp_client.c - SDP client: ServiceSearchAttribute queries and a
 * walker for their results.
 *
 * Overview
 * --------
 * One query runs at a time, on an L2CAP channel the client opens to the
 * peer's PSM 0x0001 on an existing link (SDP needs no security):
 *
 *   floss_sdp_query_ssa()
 *     -> L2CAP connect (PSM 0x0001)
 *     -> on_open: ServiceSearchAttributeRequest (pattern, max bytes, range)
 *     -> on_data: ServiceSearchAttributeResponse
 *          AttributeListsByteCount | AttributeLists chunk | continuation
 *        append the chunk; non-empty continuation -> repeat the identical
 *        request carrying it; empty -> done
 *     -> close the channel, cb(handle, 0, lists, len, user)
 *
 * The request (all big-endian):
 *
 *   0x06 | tid(2) | plen(2) | DES{UUID} | MaximumAttributeByteCount(2)
 *        | DES{uint32 attr_lo << 16 | attr_hi} | cont_len(1) | cont(0..16)
 *
 * MaximumAttributeByteCount is sized so a full response fits our L2CAP
 * receive MTU (5 header + 2 count + 1 + 16 continuation bytes around it).
 * The chunks are concatenated into a fixed buffer (FLOSS_SDP_CLIENT_BUF);
 * a larger answer fails with FLOSS_ERR_NOMEM rather than being truncated.
 *
 * Every way a query can end funnels through query_finish(), which clears
 * the state BEFORE calling back, so the callback fires exactly once and may
 * start the next query. Callbacks of a channel that no longer belongs to
 * the running query (one we closed after finishing) are ignored by
 * comparing channel pointers.
 *
 * The walker (floss_sdp_iter_*, floss_sdp_find_attr, floss_sdp_rfcomm_channel)
 * only uses floss_de_header(), which bounds-checks every element against
 * the bytes that remain, so hostile results cannot make it read past len.
 */
#include "floss_sdp_priv.h"

#if FLOSS_ENABLE_SDP_CLIENT

/* ------------------------------------------------------------------------ */
/* Protocol constants                                                       */
/* ------------------------------------------------------------------------ */

#define SDPC_PDU_ERROR_RSP       0x01
#define SDPC_PDU_SEARCH_ATTR_REQ 0x06
#define SDPC_PDU_SEARCH_ATTR_RSP 0x07

#define SDPC_HDR_LEN        5  /* id + tid + param length */
#define SDPC_CONT_MAX       16 /* protocol limit on continuation state bytes */
#define SDPC_RSP_OVERHEAD   (SDPC_HDR_LEN + 2 + 1 + SDPC_CONT_MAX)
#define SDPC_MIN_ATTR_BYTES 7  /* smallest legal MaximumAttributeByteCount */
#define SDPC_MAX_EMPTY      4  /* responses in a row with no bytes but a continuation */
#define SDPC_REQ_MAX        (SDPC_HDR_LEN + 2 + 17 + 2 + 7 + 1 + SDPC_CONT_MAX)

/* ------------------------------------------------------------------------ */
/* State                                                                    */
/* ------------------------------------------------------------------------ */

static struct {
    bool busy;
    bool tx_pending;              /**< request built but L2CAP said BUSY */
    uint16_t handle;
    floss_l2cap_chan_t *chan;     /**< channel of the running query */
    floss_sdp_attr_fn cb;
    void *user;
    floss_sdp_rfcomm_fn rfcomm_cb; /**< floss_sdp_query_rfcomm_channel() caller */
    void *rfcomm_user;
    uint8_t uuid[16];
    uint8_t uuid_len;
    uint16_t attr_lo;
    uint16_t attr_hi;
    uint16_t max_bytes;           /**< MaximumAttributeByteCount */
    uint16_t tid;
    uint8_t cont[SDPC_CONT_MAX];
    uint8_t cont_len;
    uint8_t empty_rounds;
    uint8_t req[SDPC_REQ_MAX];
    uint16_t req_len;
    uint16_t len;                 /**< bytes reassembled in buf */
    uint8_t buf[FLOSS_SDP_CLIENT_BUF];
    floss_timer_t timer;
} cl;

/* ------------------------------------------------------------------------ */
/* Query lifecycle                                                          */
/* ------------------------------------------------------------------------ */

/**
 * End the running query: stop the timer, optionally close the channel,
 * clear the state, then report. `close` is false when the channel is
 * already going away (link loss, peer close, failed open).
 */
static void query_finish(int status, bool close)
{
    floss_l2cap_chan_t *chan = cl.chan;
    floss_sdp_attr_fn cb = cl.cb;
    void *user = cl.user;
    uint16_t handle = cl.handle;

    floss_timer_stop(&cl.timer);
    cl.busy = false;
    cl.tx_pending = false;
    cl.chan = NULL;
    cl.cb = NULL;
    cl.user = NULL;
    if (close && chan != NULL) {
        (void)floss_l2cap_disconnect(chan);
    }
    if (status == 0) {
        FLOSS_LOGD(SDP, "client: query on 0x%03x done, %u bytes", handle, cl.len);
    } else {
        FLOSS_LOGW(SDP, "client: query on 0x%03x failed: %d", handle, status);
    }
    if (cb != NULL) {
        cb(handle, status, status == 0 ? cl.buf : NULL, status == 0 ? cl.len : 0, user);
    }
}

static void query_timeout(floss_timer_t *timer, void *arg)
{
    (void)timer;
    (void)arg;
    if (cl.busy) {
        query_finish(FLOSS_ERR_TIMEOUT, true);
    }
}

static void timer_restart(void)
{
    floss_timer_start(&cl.timer, (uint32_t)FLOSS_SDP_CLIENT_TIMEOUT_MS * 1000u);
}

/** Build the request for the current continuation state into cl.req. */
static void build_request(void)
{
    uint8_t *p = cl.req;
    uint16_t n = 0;

    p[n++] = SDPC_PDU_SEARCH_ATTR_REQ;
    floss_put_be16(p + n, cl.tid);
    n = (uint16_t)(n + 4); /* + parameter length, patched below */

    /* ServiceSearchPattern: a sequence of one UUID. */
    p[n++] = FLOSS_DE_HDR(FLOSS_DE_TYPE_SEQ, FLOSS_DE_SIZE_LEN8);
    p[n++] = (uint8_t)(1 + cl.uuid_len);
    p[n++] = FLOSS_DE_HDR(FLOSS_DE_TYPE_UUID, cl.uuid_len == 2 ? FLOSS_DE_SIZE_2
                                              : cl.uuid_len == 4 ? FLOSS_DE_SIZE_4
                                                                 : FLOSS_DE_SIZE_16);
    memcpy(p + n, cl.uuid, cl.uuid_len);
    n = (uint16_t)(n + cl.uuid_len);

    floss_put_be16(p + n, cl.max_bytes);
    n = (uint16_t)(n + 2);

    /* AttributeIDList: one uint32 range. */
    p[n++] = FLOSS_DE_HDR(FLOSS_DE_TYPE_SEQ, FLOSS_DE_SIZE_LEN8);
    p[n++] = 5;
    p[n++] = FLOSS_DE_HDR_UINT32;
    floss_put_be32(p + n, ((uint32_t)cl.attr_lo << 16) | cl.attr_hi);
    n = (uint16_t)(n + 4);

    p[n++] = cl.cont_len;
    memcpy(p + n, cl.cont, cl.cont_len);
    n = (uint16_t)(n + cl.cont_len);

    floss_put_be16(p + 3, (uint16_t)(n - SDPC_HDR_LEN));
    cl.req_len = n;
}

/** Send cl.req; BUSY waits for on_can_send, anything else ends the query. */
static void send_request(void)
{
    floss_err_t rc = floss_l2cap_send(cl.chan, cl.req, cl.req_len);
    if (rc == FLOSS_ERR_BUSY) {
        cl.tx_pending = true;
        return;
    }
    cl.tx_pending = false;
    if (rc != FLOSS_OK) {
        FLOSS_LOGW(SDP, "client: send failed: %s", floss_err_str(rc));
        query_finish(FLOSS_ERR_IO, true);
    }
}

static void next_request(void)
{
    cl.tid++;
    build_request();
    timer_restart();
    send_request();
}

/* ------------------------------------------------------------------------ */
/* L2CAP callbacks                                                          */
/* ------------------------------------------------------------------------ */

static void cl_on_open(floss_l2cap_chan_t *chan, uint16_t status)
{
    if (!cl.busy || chan != cl.chan) {
        return; /* a channel we already gave up on */
    }
    if (status != 0) {
        query_finish(status == FLOSS_L2CAP_CONN_PSM_UNSUPP ? FLOSS_ERR_UNSUPPORTED : FLOSS_ERR_IO, false);
        return;
    }
    uint16_t mtu = chan->local_mtu;
    uint16_t max = mtu > SDPC_RSP_OVERHEAD ? (uint16_t)(mtu - SDPC_RSP_OVERHEAD) : 0;
    cl.max_bytes = FLOSS_MAX(max, (uint16_t)SDPC_MIN_ATTR_BYTES);
    next_request();
}

static void cl_on_close(floss_l2cap_chan_t *chan)
{
    if (cl.busy && chan == cl.chan) {
        query_finish(FLOSS_ERR_IO, false);
    }
}

static void cl_on_can_send(floss_l2cap_chan_t *chan)
{
    if (cl.busy && chan == cl.chan && cl.tx_pending) {
        send_request();
    }
}

/** The complete result must be exactly one data element sequence. */
static bool result_well_formed(void)
{
    uint8_t type;
    uint32_t n;
    uint16_t h = floss_de_header(cl.buf, cl.len, &type, &n);
    return h != 0 && type == FLOSS_DE_TYPE_SEQ && (uint32_t)h + n == cl.len;
}

static void on_attr_rsp(const uint8_t *p, uint16_t plen)
{
    if (plen < 3) {
        query_finish(FLOSS_ERR_PROTOCOL, true);
        return;
    }
    uint16_t count = floss_get_be16(p);
    if ((uint32_t)count + 3 > plen) {
        query_finish(FLOSS_ERR_PROTOCOL, true);
        return;
    }
    uint8_t cont_len = p[2 + count];
    if (cont_len > SDPC_CONT_MAX || (uint32_t)count + 3 + cont_len > plen) {
        query_finish(FLOSS_ERR_PROTOCOL, true);
        return;
    }
    if ((uint32_t)cl.len + count > sizeof cl.buf) {
        FLOSS_LOGW(SDP, "client: result exceeds FLOSS_SDP_CLIENT_BUF (%u)", (unsigned)sizeof cl.buf);
        query_finish(FLOSS_ERR_NOMEM, true);
        return;
    }
    memcpy(cl.buf + cl.len, p + 2, count);
    cl.len = (uint16_t)(cl.len + count);

    if (cont_len == 0) {
        query_finish(result_well_formed() ? 0 : FLOSS_ERR_PROTOCOL, true);
        return;
    }
    /* A server that keeps answering "more" without data would loop forever. */
    cl.empty_rounds = count == 0 ? (uint8_t)(cl.empty_rounds + 1) : 0;
    if (cl.empty_rounds >= SDPC_MAX_EMPTY) {
        query_finish(FLOSS_ERR_PROTOCOL, true);
        return;
    }
    memcpy(cl.cont, p + 3 + count, cont_len);
    cl.cont_len = cont_len;
    next_request();
}

static void cl_on_data(floss_l2cap_chan_t *chan, const uint8_t *data, uint16_t len)
{
    if (!cl.busy || chan != cl.chan) {
        return;
    }
    if (len < SDPC_HDR_LEN) {
        query_finish(FLOSS_ERR_PROTOCOL, true);
        return;
    }
    uint8_t pdu = data[0];
    uint16_t tid = floss_get_be16(data + 1);
    uint16_t plen = floss_get_be16(data + 3);
    if ((uint32_t)plen + SDPC_HDR_LEN > len || tid != cl.tid || cl.tx_pending) {
        query_finish(FLOSS_ERR_PROTOCOL, true);
        return;
    }
    const uint8_t *p = data + SDPC_HDR_LEN;
    switch (pdu) {
    case SDPC_PDU_SEARCH_ATTR_RSP:
        on_attr_rsp(p, plen);
        break;
    case SDPC_PDU_ERROR_RSP: {
        uint16_t code = plen >= 2 ? floss_get_be16(p) : 0;
        FLOSS_LOGW(SDP, "client: peer answered error 0x%04x", code);
        query_finish(code != 0 ? (int)code : FLOSS_ERR_PROTOCOL, true);
        break;
    }
    default:
        query_finish(FLOSS_ERR_PROTOCOL, true);
        break;
    }
}

static const floss_l2cap_ops_t cl_ops = {
    .on_open = cl_on_open,
    .on_close = cl_on_close,
    .on_data = cl_on_data,
    .on_can_send = cl_on_can_send,
};

/* ------------------------------------------------------------------------ */
/* Public API: queries                                                      */
/* ------------------------------------------------------------------------ */

floss_err_t floss_sdp_query_ssa(uint16_t handle, const uint8_t *uuid_be, uint8_t uuid_len, uint16_t attr_lo,
                                uint16_t attr_hi, floss_sdp_attr_fn cb, void *user)
{
    if (cb == NULL || uuid_be == NULL || (uuid_len != 2 && uuid_len != 4 && uuid_len != 16) ||
        attr_lo > attr_hi) {
        return FLOSS_ERR_INVALID;
    }
    if (cl.busy) {
        return FLOSS_ERR_BUSY;
    }
    floss_l2cap_chan_t *chan = NULL;
    floss_err_t err = floss_l2cap_connect(handle, FLOSS_PSM_SDP, &cl_ops, NULL, FLOSS_SEC_NONE, &chan);
    if (err != FLOSS_OK) {
        return err;
    }
    cl.busy = true;
    cl.tx_pending = false;
    cl.handle = handle;
    cl.chan = chan;
    cl.cb = cb;
    cl.user = user;
    memcpy(cl.uuid, uuid_be, uuid_len);
    cl.uuid_len = uuid_len;
    cl.attr_lo = attr_lo;
    cl.attr_hi = attr_hi;
    cl.cont_len = 0;
    cl.empty_rounds = 0;
    cl.len = 0;
    timer_restart();
    FLOSS_LOGD(SDP, "client: query on 0x%03x, attributes 0x%04x-0x%04x", handle, attr_lo, attr_hi);
    return FLOSS_OK;
}

bool floss_sdp_query_busy(void)
{
    return cl.busy;
}

static void rfcomm_done(uint16_t handle, int status, const uint8_t *lists, uint16_t len, void *user)
{
    (void)user;
    floss_sdp_rfcomm_fn cb = cl.rfcomm_cb;
    void *cb_user = cl.rfcomm_user;
    uint8_t channel = 0;

    cl.rfcomm_cb = NULL;
    cl.rfcomm_user = NULL;
    if (status == 0) {
        status = floss_sdp_rfcomm_channel(lists, len, &channel);
        if (status != FLOSS_OK) {
            channel = 0;
        }
    }
    if (cb != NULL) {
        cb(handle, status, channel, cb_user);
    }
}

floss_err_t floss_sdp_query_rfcomm_channel(uint16_t handle, const uint8_t uuid128_be[16], floss_sdp_rfcomm_fn cb,
                                           void *user)
{
    if (cb == NULL || uuid128_be == NULL) {
        return FLOSS_ERR_INVALID;
    }
    floss_err_t err = floss_sdp_query_ssa(handle, uuid128_be, 16, FLOSS_ATTR_PROTOCOL_DESCRIPTOR_LIST,
                                          FLOSS_ATTR_PROTOCOL_DESCRIPTOR_LIST, rfcomm_done, NULL);
    if (err == FLOSS_OK) {
        cl.rfcomm_cb = cb;
        cl.rfcomm_user = user;
    }
    return err;
}

/* ------------------------------------------------------------------------ */
/* Result walker                                                            */
/* ------------------------------------------------------------------------ */

/**
 * Decode the element at p (avail bytes). Sets *type, *payload, *payload_len
 * and returns the element's total size, or 0 when it is malformed.
 */
static uint16_t elem(const uint8_t *p, const uint8_t *end, uint8_t *type, const uint8_t **payload,
                     uint16_t *payload_len)
{
    if (p >= end) {
        return 0;
    }
    uint32_t n;
    uint16_t h = floss_de_header(p, (uint16_t)(end - p), type, &n);
    if (h == 0) {
        return 0;
    }
    *payload = p + h;
    *payload_len = (uint16_t)n;
    return (uint16_t)(h + n);
}

floss_err_t floss_sdp_iter_init(floss_sdp_iter_t *it, const uint8_t *data, uint16_t len)
{
    uint8_t type;
    const uint8_t *body;
    uint16_t body_len;

    if (it == NULL) {
        return FLOSS_ERR_INVALID;
    }
    memset(it, 0, sizeof *it);
    if (data == NULL || elem(data, data + len, &type, &body, &body_len) == 0 || type != FLOSS_DE_TYPE_SEQ) {
        return FLOSS_ERR_PROTOCOL;
    }
    /* An attribute list starts with a uint16 attribute ID, a list of them with a sequence. */
    it->single = body_len > 0 && (body[0] >> 3) == FLOSS_DE_TYPE_UINT;
    if (it->single) {
        it->p = data;
        it->end = body + body_len;
    } else {
        it->p = body;
        it->end = body + body_len;
    }
    return FLOSS_OK;
}

floss_err_t floss_sdp_iter_next(floss_sdp_iter_t *it, const uint8_t **record, uint16_t *record_len)
{
    uint8_t type;
    const uint8_t *body;
    uint16_t body_len;

    if (it == NULL || it->p == NULL || it->p >= it->end) {
        return FLOSS_ERR_NOT_FOUND;
    }
    uint16_t n = elem(it->p, it->end, &type, &body, &body_len);
    if (n == 0 || type != FLOSS_DE_TYPE_SEQ) {
        it->p = it->end; /* stop: nothing after a broken element can be trusted */
        return FLOSS_ERR_PROTOCOL;
    }
    if (record != NULL) {
        *record = it->p;
    }
    if (record_len != NULL) {
        *record_len = n;
    }
    it->p += n;
    return FLOSS_OK;
}

/** Look up attr_id in one attribute list (record = the whole sequence element). */
static floss_err_t record_attr(const uint8_t *record, uint16_t record_len, uint16_t attr_id, const uint8_t **value,
                               uint16_t *value_len)
{
    uint8_t type;
    const uint8_t *p;
    uint16_t plen;

    if (elem(record, record + record_len, &type, &p, &plen) == 0) {
        return FLOSS_ERR_PROTOCOL;
    }
    const uint8_t *end = p + plen;
    while (p < end) {
        const uint8_t *idp;
        uint16_t idlen;
        uint16_t n = elem(p, end, &type, &idp, &idlen);
        if (n == 0 || type != FLOSS_DE_TYPE_UINT || idlen != 2) {
            return FLOSS_ERR_PROTOCOL;
        }
        uint16_t id = floss_get_be16(idp);
        p += n;
        const uint8_t *vp;
        uint16_t vlen;
        n = elem(p, end, &type, &vp, &vlen);
        if (n == 0) {
            return FLOSS_ERR_PROTOCOL;
        }
        if (id == attr_id) {
            *value = p;
            *value_len = n;
            return FLOSS_OK;
        }
        p += n;
    }
    return FLOSS_ERR_NOT_FOUND;
}

floss_err_t floss_sdp_find_attr(const uint8_t *data, uint16_t len, uint16_t attr_id, const uint8_t **value,
                                uint16_t *value_len)
{
    floss_sdp_iter_t it;
    const uint8_t *rec;
    uint16_t rec_len;

    if (value == NULL || value_len == NULL) {
        return FLOSS_ERR_INVALID;
    }
    floss_err_t err = floss_sdp_iter_init(&it, data, len);
    if (err != FLOSS_OK) {
        return err;
    }
    while ((err = floss_sdp_iter_next(&it, &rec, &rec_len)) == FLOSS_OK) {
        err = record_attr(rec, rec_len, attr_id, value, value_len);
        if (err != FLOSS_ERR_NOT_FOUND) {
            return err;
        }
    }
    return err;
}

/**
 * One protocol descriptor (payload of its sequence): UUID first, then
 * protocol parameters. RFCOMM's single parameter is the server channel.
 */
static floss_err_t descriptor_channel(const uint8_t *p, const uint8_t *end, uint8_t *channel)
{
    static const uint8_t rfcomm_be[2] = {(uint8_t)(FLOSS_UUID_RFCOMM >> 8), (uint8_t)FLOSS_UUID_RFCOMM};
    uint8_t want[16];
    uint8_t got[16];
    uint8_t type;
    const uint8_t *v;
    uint16_t vlen;

    uint16_t n = elem(p, end, &type, &v, &vlen);
    if (n == 0 || type != FLOSS_DE_TYPE_UUID) {
        return FLOSS_ERR_PROTOCOL;
    }
    floss_sdp_uuid_promote(rfcomm_be, 2, want);
    floss_sdp_uuid_promote(v, vlen, got);
    if (memcmp(want, got, 16) != 0) {
        return FLOSS_ERR_NOT_FOUND;
    }
    p += n;
    n = elem(p, end, &type, &v, &vlen);
    /* The channel is a uint8; tolerate a wider unsigned integer holding it. */
    if (n == 0 || type != FLOSS_DE_TYPE_UINT || vlen > 2) {
        return FLOSS_ERR_PROTOCOL;
    }
    uint16_t ch = vlen == 1 ? v[0] : floss_get_be16(v);
    if (ch < 1 || ch > 30) {
        return FLOSS_ERR_PROTOCOL;
    }
    *channel = (uint8_t)ch;
    return FLOSS_OK;
}

/**
 * A ProtocolDescriptorList value (whole element): a sequence of protocol
 * descriptors, or an alternative of such sequences (depth 1).
 */
static floss_err_t pdl_channel(const uint8_t *e, const uint8_t *end, uint8_t *channel, bool allow_alt)
{
    uint8_t type;
    const uint8_t *p;
    uint16_t plen;

    if (elem(e, end, &type, &p, &plen) == 0) {
        return FLOSS_ERR_PROTOCOL;
    }
    if (type != FLOSS_DE_TYPE_SEQ && !(allow_alt && type == FLOSS_DE_TYPE_ALT)) {
        return FLOSS_ERR_PROTOCOL;
    }
    const uint8_t *lend = p + plen;
    floss_err_t result = FLOSS_ERR_NOT_FOUND;
    while (p < lend) {
        const uint8_t *d;
        uint16_t dlen;
        uint8_t dtype;
        uint16_t n = elem(p, lend, &dtype, &d, &dlen);
        if (n == 0 || dtype != FLOSS_DE_TYPE_SEQ) {
            return FLOSS_ERR_PROTOCOL;
        }
        floss_err_t err = type == FLOSS_DE_TYPE_ALT ? pdl_channel(p, lend, channel, false)
                                                    : descriptor_channel(d, d + dlen, channel);
        if (err == FLOSS_OK) {
            return FLOSS_OK;
        }
        if (err == FLOSS_ERR_PROTOCOL) {
            result = err;
        }
        p += n;
    }
    return result;
}

floss_err_t floss_sdp_rfcomm_channel(const uint8_t *data, uint16_t len, uint8_t *channel)
{
    floss_sdp_iter_t it;
    const uint8_t *rec;
    uint16_t rec_len;

    if (channel == NULL) {
        return FLOSS_ERR_INVALID;
    }
    *channel = 0;
    floss_err_t err = floss_sdp_iter_init(&it, data, len);
    if (err != FLOSS_OK) {
        return err;
    }
    floss_err_t result = FLOSS_ERR_NOT_FOUND;
    while ((err = floss_sdp_iter_next(&it, &rec, &rec_len)) == FLOSS_OK) {
        const uint8_t *v;
        uint16_t vlen;
        err = record_attr(rec, rec_len, FLOSS_ATTR_PROTOCOL_DESCRIPTOR_LIST, &v, &vlen);
        if (err == FLOSS_OK) {
            err = pdl_channel(v, v + vlen, channel, true);
            if (err == FLOSS_OK) {
                return FLOSS_OK;
            }
        }
        if (err == FLOSS_ERR_PROTOCOL) {
            result = err;
        }
    }
    if (err == FLOSS_ERR_PROTOCOL) {
        result = err;
    }
    *channel = 0;
    return result;
}

/* ------------------------------------------------------------------------ */
/* Lifecycle                                                                */
/* ------------------------------------------------------------------------ */

void floss_sdp_client_init(void)
{
    memset(&cl, 0, sizeof cl);
    floss_timer_init(&cl.timer, query_timeout, NULL);
}

#endif /* FLOSS_ENABLE_SDP_CLIENT */
