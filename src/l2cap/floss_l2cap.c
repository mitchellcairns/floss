/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_l2cap.c - Classic L2CAP channels: table, PSM registry, data path.
 *
 * Overview
 * --------
 * Every L2CAP PDU starts with a 4-byte basic header: payload length and
 * destination channel ID (CID). CID 0x0001 carries signaling commands, which
 * floss_l2cap_sig.c handles; CIDs from 0x0040 up are dynamic channels created
 * by a Connection Request and owned by a profile (HID, SDP, RFCOMM).
 *
 * Memory: a static table of FLOSS_MAX_L2CAP_CHANNELS slots and a small PSM
 * registry. Nothing is allocated at runtime except pool buffers for TX.
 *
 * CIDs: local CIDs come from a rolling counter starting at 0x0040 that skips
 * CIDs still in use. Rolling (instead of always taking the lowest free CID)
 * means a just-closed CID is not immediately handed out again, so a stale
 * packet for the old channel cannot land on a new one.
 *
 * RX hot path (floss_l2cap_on_pdu): parse header, find the OPEN channel by
 * (handle, local CID), enforce our MTU, call on_data with the payload. The
 * table is tiny (a handful of entries), so a linear scan beats any index.
 *
 * TX hot path (floss_l2cap_send_buf): push the basic header into the buffer's
 * headroom and hand the buffer to HCI - zero copies. When HCI has no credit
 * the copying floss_l2cap_send() returns BUSY and flags the channel; the next
 * floss_l2cap_on_credits() wakes it through on_can_send.
 *
 * Lifecycle guarantee: every channel ends with exactly ONE terminal callback,
 * see floss_l2cap_slot_finish().
 */
#include "floss_l2cap_priv.h"

floss_l2cap_slot_t floss_l2cap_slots[FLOSS_MAX_L2CAP_CHANNELS];

static floss_l2cap_server_t l2cap_servers[FLOSS_L2CAP_MAX_SERVERS];
static uint16_t l2cap_next_cid;

/* ------------------------------------------------------------------------ */
/* Slots                                                                    */
/* ------------------------------------------------------------------------ */

static bool l2cap_cid_in_use(uint16_t cid)
{
    for (unsigned i = 0; i < FLOSS_MAX_L2CAP_CHANNELS; i++) {
        if (floss_l2cap_slots[i].in_use && floss_l2cap_slots[i].chan.local_cid == cid) {
            return true;
        }
    }
    return false;
}

/**
 * Next free dynamic CID. CIDs are kept unique across all links (stricter than
 * the per-link uniqueness the protocol requires), which keeps lookups simple.
 * At most FLOSS_MAX_L2CAP_CHANNELS - 1 CIDs can be live while we allocate, so
 * the loop always finds one within FLOSS_MAX_L2CAP_CHANNELS + 1 steps.
 */
static uint16_t l2cap_alloc_cid(void)
{
    for (unsigned tries = 0; tries <= FLOSS_MAX_L2CAP_CHANNELS; tries++) {
        uint16_t cid = l2cap_next_cid;
        l2cap_next_cid = (l2cap_next_cid == 0xFFFF) ? FLOSS_CID_DYNAMIC_FIRST : (uint16_t)(l2cap_next_cid + 1);
        if (!l2cap_cid_in_use(cid)) {
            return cid;
        }
    }
    return 0; /* unreachable with a consistent table */
}

floss_l2cap_slot_t *floss_l2cap_slot_alloc(uint16_t handle)
{
    for (unsigned i = 0; i < FLOSS_MAX_L2CAP_CHANNELS; i++) {
        floss_l2cap_slot_t *s = &floss_l2cap_slots[i];
        if (s->in_use) {
            continue;
        }
        uint16_t cid = l2cap_alloc_cid();
        if (cid == 0) {
            return NULL;
        }
        memset(s, 0, sizeof(*s));
        s->in_use = true;
        s->chan.state = FLOSS_L2CAP_CLOSED;
        s->chan.handle = handle;
        s->chan.local_cid = cid;
        s->chan.local_mtu = FLOSS_L2CAP_MTU;
        s->chan.remote_mtu = FLOSS_L2CAP_DEFAULT_MTU;
        s->chan.flush_timeout = 0xFFFF;
        floss_timer_init(&s->timer, floss_l2cap_sig_timeout, s);
        return s;
    }
    return NULL;
}

floss_l2cap_slot_t *floss_l2cap_slot_by_local_cid(uint16_t handle, uint16_t cid)
{
    for (unsigned i = 0; i < FLOSS_MAX_L2CAP_CHANNELS; i++) {
        floss_l2cap_slot_t *s = &floss_l2cap_slots[i];
        if (s->in_use && !s->finishing && s->chan.handle == handle && s->chan.local_cid == cid) {
            return s;
        }
    }
    return NULL;
}

floss_l2cap_slot_t *floss_l2cap_slot_by_remote_cid(uint16_t handle, uint16_t cid)
{
    for (unsigned i = 0; i < FLOSS_MAX_L2CAP_CHANNELS; i++) {
        floss_l2cap_slot_t *s = &floss_l2cap_slots[i];
        if (s->in_use && !s->finishing && s->chan.handle == handle && s->chan.remote_cid == cid) {
            return s;
        }
    }
    return NULL;
}

/** Map an application channel pointer back to a live slot (NULL if bogus/stale). */
static floss_l2cap_slot_t *l2cap_slot_checked(const floss_l2cap_chan_t *chan)
{
    for (unsigned i = 0; i < FLOSS_MAX_L2CAP_CHANNELS; i++) {
        if (&floss_l2cap_slots[i].chan == chan) {
            return floss_l2cap_slots[i].in_use ? &floss_l2cap_slots[i] : NULL;
        }
    }
    return NULL;
}

void floss_l2cap_slot_finish(floss_l2cap_slot_t *s, uint16_t status)
{
    if (!s->in_use || s->finishing) {
        return;
    }
    floss_l2cap_chan_t *chan = &s->chan;
    floss_timer_stop(&s->timer);
    s->finishing = true;
    s->wants_can_send = false;
    chan->state = FLOSS_L2CAP_CLOSED;

    const floss_l2cap_ops_t *ops = chan->ops;
    if (s->opened || s->user_closing) {
        FLOSS_LOGI(L2CAP, "closed psm 0x%04x lcid 0x%04x rcid 0x%04x handle 0x%03x",
                   chan->psm, chan->local_cid, chan->remote_cid, chan->handle);
        if (ops && ops->on_close) {
            ops->on_close(chan);
        }
    } else {
        if (status == 0) {
            status = FLOSS_L2CAP_STATUS_ABORTED;
        }
        FLOSS_LOGI(L2CAP, "open failed psm 0x%04x lcid 0x%04x status 0x%04x", chan->psm, chan->local_cid, status);
        if (ops && ops->on_open) {
            ops->on_open(chan, status);
        }
    }
    /* Freed only after the callback returned: the app may still read chan. */
    memset(s, 0, sizeof(*s));
}

/* ------------------------------------------------------------------------ */
/* PSM registry                                                             */
/* ------------------------------------------------------------------------ */

const floss_l2cap_server_t *floss_l2cap_server_find(uint16_t psm)
{
    for (unsigned i = 0; i < FLOSS_L2CAP_MAX_SERVERS; i++) {
        if (l2cap_servers[i].ops && l2cap_servers[i].psm == psm) {
            return &l2cap_servers[i];
        }
    }
    return NULL;
}

/** A PSM is odd, and the low bit of its upper octet is 0 (protocol rule). */
static bool l2cap_psm_valid(uint16_t psm)
{
    return (psm & 0x0101) == 0x0001;
}

floss_err_t floss_l2cap_register_psm(uint16_t psm, const floss_l2cap_ops_t *ops, void *user, floss_sec_level_t sec)
{
    if (!ops || !l2cap_psm_valid(psm)) {
        return FLOSS_ERR_INVALID;
    }
    floss_l2cap_server_t *free_entry = NULL;
    for (unsigned i = 0; i < FLOSS_L2CAP_MAX_SERVERS; i++) {
        floss_l2cap_server_t *e = &l2cap_servers[i];
        if (e->ops && e->psm == psm) {
            free_entry = e; /* re-registration replaces the handlers */
            break;
        }
        if (!e->ops && !free_entry) {
            free_entry = e;
        }
    }
    if (!free_entry) {
        return FLOSS_ERR_NOMEM;
    }
    free_entry->psm = psm;
    free_entry->ops = ops;
    free_entry->user = user;
    free_entry->sec = (uint8_t)sec;
    FLOSS_LOGD(L2CAP, "registered psm 0x%04x sec %u", psm, (unsigned)sec);
    return FLOSS_OK;
}

/* ------------------------------------------------------------------------ */
/* Channel API                                                              */
/* ------------------------------------------------------------------------ */

floss_err_t floss_l2cap_connect(uint16_t handle, uint16_t psm, const floss_l2cap_ops_t *ops, void *user,
                                floss_sec_level_t sec, floss_l2cap_chan_t **out)
{
    if (!ops || !l2cap_psm_valid(psm)) {
        return FLOSS_ERR_INVALID;
    }
    const floss_conn_t *conn = floss_hci_conn_by_handle(handle);
    if (!conn) {
        return FLOSS_ERR_NOT_FOUND;
    }
    if (conn->state != FLOSS_CONN_OPEN || conn->le) {
        return FLOSS_ERR_STATE;
    }
    floss_l2cap_slot_t *s = floss_l2cap_slot_alloc(handle);
    if (!s) {
        return FLOSS_ERR_NOMEM;
    }
    floss_l2cap_chan_t *chan = &s->chan;
    chan->psm = psm;
    chan->ops = ops;
    chan->user = user;
    chan->initiator = true;
    s->sec = (uint8_t)sec;

    if (sec == FLOSS_SEC_ENCRYPTED && !conn->encrypted) {
        /* The Connection Request goes out once GAP reports an encrypted link
         * (floss_l2cap_on_link_security). */
        chan->state = FLOSS_L2CAP_WAIT_SECURITY;
        floss_timer_start(&s->timer, FLOSS_L2CAP_SECURITY_TIMEOUT_US);
        if (out) {
            *out = chan;
        }
        FLOSS_LOGD(L2CAP, "connect psm 0x%04x lcid 0x%04x: waiting for security", psm, chan->local_cid);
        floss_gap_require_security(handle);
        return FLOSS_OK;
    }

    floss_err_t err = floss_l2cap_sig_connect(s);
    if (err != FLOSS_OK) {
        memset(s, 0, sizeof(*s)); /* never announced: free silently */
        return err;
    }
    if (out) {
        *out = chan;
    }
    return FLOSS_OK;
}

floss_err_t floss_l2cap_disconnect(floss_l2cap_chan_t *chan)
{
    floss_l2cap_slot_t *s = l2cap_slot_checked(chan);
    if (!s || s->finishing) {
        return FLOSS_ERR_INVALID;
    }
    switch (chan->state) {
    case FLOSS_L2CAP_OPEN:
    case FLOSS_L2CAP_CONFIG:
        s->user_closing = true;
        floss_l2cap_sig_disconnect(s);
        return FLOSS_OK;
    case FLOSS_L2CAP_WAIT_CONN_RSP:
        /* We do not know the remote CID yet; the Connection Response handler
         * sees user_closing and disconnects instead of configuring. */
        s->user_closing = true;
        return FLOSS_OK;
    case FLOSS_L2CAP_WAIT_SECURITY:
        s->user_closing = true;
        if (!chan->initiator) {
            floss_l2cap_sig_refuse_pending(s, FLOSS_L2CAP_CONN_NO_RESOURCES);
        }
        floss_l2cap_slot_finish(s, 0);
        return FLOSS_OK;
    case FLOSS_L2CAP_WAIT_DISC_RSP:
        s->user_closing = true;
        return FLOSS_OK;
    default:
        return FLOSS_ERR_STATE;
    }
}

floss_err_t floss_l2cap_send_buf(floss_l2cap_chan_t *chan, floss_buf_t *buf, uint8_t flags)
{
    if (!buf) {
        return FLOSS_ERR_INVALID;
    }
    if (!chan || chan->state != FLOSS_L2CAP_OPEN) {
        floss_buf_free(buf);
        return FLOSS_ERR_STATE;
    }
    uint16_t len = buf->len;
    if (len > chan->remote_mtu) {
        FLOSS_LOGW(L2CAP, "tx %u bytes exceeds peer MTU %u on lcid 0x%04x", len, chan->remote_mtu, chan->local_cid);
        floss_buf_free(buf);
        return FLOSS_ERR_INVALID;
    }
    uint8_t *hdr = floss_buf_push(buf, FLOSS_L2CAP_HDR_LEN);
    floss_put_le16(hdr, len);
    floss_put_le16(hdr + 2, chan->remote_cid);
    buf->handle = chan->handle;
    buf->flags = flags;
    return floss_hci_acl_send(buf);
}

floss_err_t floss_l2cap_send(floss_l2cap_chan_t *chan, const uint8_t *data, uint16_t len)
{
    if (!chan || chan->state != FLOSS_L2CAP_OPEN) {
        return FLOSS_ERR_STATE;
    }
    if ((len && !data) || len > chan->remote_mtu) {
        return FLOSS_ERR_INVALID;
    }
    floss_l2cap_slot_t *s = floss_l2cap_slot_of(chan);
    if (!floss_hci_acl_can_send_now(chan->handle)) {
        s->wants_can_send = true;
        return FLOSS_ERR_BUSY;
    }
    floss_buf_t *buf = floss_buf_alloc();
    if (!buf) {
        FLOSS_STAT_INC(buf_alloc_failures);
        s->wants_can_send = true;
        return FLOSS_ERR_BUSY;
    }
    uint8_t *p = floss_buf_put(buf, len);
    if (!p) {
        floss_buf_free(buf);
        return FLOSS_ERR_INVALID;
    }
    if (len) {
        memcpy(p, data, len);
    }
    return floss_l2cap_send_buf(chan, buf, 0);
}

bool floss_l2cap_can_send_now(const floss_l2cap_chan_t *chan)
{
    return chan && chan->state == FLOSS_L2CAP_OPEN && floss_hci_acl_can_send_now(chan->handle);
}

floss_l2cap_chan_t *floss_l2cap_find(uint16_t handle, uint16_t psm)
{
    for (unsigned i = 0; i < FLOSS_MAX_L2CAP_CHANNELS; i++) {
        floss_l2cap_slot_t *s = &floss_l2cap_slots[i];
        if (s->in_use && !s->finishing && s->chan.state == FLOSS_L2CAP_OPEN && s->chan.handle == handle &&
            s->chan.psm == psm) {
            return &s->chan;
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------------ */
/* Hooks from HCI / GAP                                                     */
/* ------------------------------------------------------------------------ */

void floss_l2cap_on_pdu(uint16_t handle, const uint8_t *pdu, uint16_t len)
{
    if (len < FLOSS_L2CAP_HDR_LEN) {
        FLOSS_STAT_INC(rx_dropped);
        return;
    }
    uint16_t plen = floss_get_le16(pdu);
    uint16_t cid = floss_get_le16(pdu + 2);
    if (plen > len - FLOSS_L2CAP_HDR_LEN) {
        FLOSS_LOGD(L2CAP, "truncated pdu: hdr %u, have %u", plen, (unsigned)(len - FLOSS_L2CAP_HDR_LEN));
        FLOSS_STAT_INC(rx_dropped);
        return;
    }
    const uint8_t *payload = pdu + FLOSS_L2CAP_HDR_LEN;

    if (cid >= FLOSS_CID_DYNAMIC_FIRST) {
        floss_l2cap_slot_t *s = floss_l2cap_slot_by_local_cid(handle, cid);
        if (!s || s->chan.state != FLOSS_L2CAP_OPEN) {
            FLOSS_LOGD(L2CAP, "rx on unknown/closed cid 0x%04x dropped", cid);
            FLOSS_STAT_INC(rx_dropped);
            return;
        }
        if (plen > s->chan.local_mtu) {
            FLOSS_LOGW(L2CAP, "rx %u bytes exceeds our MTU %u on cid 0x%04x", plen, s->chan.local_mtu, cid);
            FLOSS_STAT_INC(rx_dropped);
            return;
        }
        if (s->chan.ops && s->chan.ops->on_data) {
            s->chan.ops->on_data(&s->chan, payload, plen);
        }
        return;
    }
    if (cid == FLOSS_CID_SIGNALING) {
        floss_l2cap_sig_on_frame(handle, payload, plen);
        return;
    }
    /* Connectionless (0x0002) and other fixed channels: not supported. */
    FLOSS_LOGD(L2CAP, "rx on fixed cid 0x%04x dropped", cid);
    FLOSS_STAT_INC(rx_dropped);
}

void floss_l2cap_on_credits(void)
{
    for (unsigned i = 0; i < FLOSS_MAX_L2CAP_CHANNELS; i++) {
        floss_l2cap_slot_t *s = &floss_l2cap_slots[i];
        if (!s->in_use || !s->wants_can_send) {
            continue;
        }
        s->wants_can_send = false;
        if (s->chan.state == FLOSS_L2CAP_OPEN && s->chan.ops && s->chan.ops->on_can_send) {
            s->chan.ops->on_can_send(&s->chan);
        }
    }
}

void floss_l2cap_on_link_up(uint16_t handle)
{
    /* Nothing to set up: channels are created on demand by either side. */
    FLOSS_LOGD(L2CAP, "link up 0x%03x", handle);
}

void floss_l2cap_on_link_down(uint16_t handle, uint8_t reason)
{
    /* The link is gone, so no signaling: just tear everything down locally. */
    for (unsigned i = 0; i < FLOSS_MAX_L2CAP_CHANNELS; i++) {
        floss_l2cap_slot_t *s = &floss_l2cap_slots[i];
        if (s->in_use && !s->finishing && s->chan.handle == handle) {
            floss_l2cap_slot_finish(s, reason ? reason : FLOSS_HCI_ERR_CONN_TIMEOUT);
        }
    }
}

void floss_l2cap_init(void)
{
    for (unsigned i = 0; i < FLOSS_MAX_L2CAP_CHANNELS; i++) {
        memset(&floss_l2cap_slots[i], 0, sizeof(floss_l2cap_slots[i]));
    }
    memset(l2cap_servers, 0, sizeof(l2cap_servers));
    l2cap_next_cid = FLOSS_CID_DYNAMIC_FIRST;
    floss_l2cap_sig_init();
}
