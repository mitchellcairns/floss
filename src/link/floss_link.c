/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_link.c - Pad <-> dongle link profile: the transport-independent core.
 *
 * ===========================================================================
 *  OVERVIEW
 * ===========================================================================
 *
 * Two FLOSS devices, a pad (controller) and a dongle (USB receiver), keep one
 * private link. The dongle is always the central (it pages / initiates), the
 * pad the peripheral. One data path per link: a custom L2CAP channel on
 * Classic, the link GATT service on LE (floss_link_classic.c,
 * floss_link_le.c). This file holds everything that does not depend on the
 * radio: the state machine glue, the pad's input mailbox and pacing, the
 * PDU codec, the dongle's statistics, PING/PONG and HELLO, and the stored
 * identity of the peer.
 *
 * States (floss_link_state()):
 *
 *   IDLE        not started, or bonded with auto_reconnect off
 *   PAIRING     pad: discoverable / advertising as pairable;
 *               dongle: inquiry / scanning for a pad
 *   CONNECTING  pad: connectable for its dongle; dongle: paging / initiating
 *   CONNECTED   data path up: reports flow
 *
 * Resting state (floss_link_rest_state()) when not connected: PAIRING when
 * there is no peer yet (out of the box both sides pair without a button),
 * otherwise CONNECTING with auto_reconnect, else IDLE. floss_link_pair()
 * opens a pairing window; with a peer to go back to it closes after
 * FLOSS_LINK_PAIR_TIMEOUT_US.
 *
 * ===========================================================================
 *  PAD INPUT PATH  (performance critical, same technique as floss_hid.c)
 * ===========================================================================
 *
 * One mailbox slot per report ID, latest wins, guarded by a seqlock:
 *
 *   producer (app / ISR / other core)       consumer (stack context)
 *   floss_link_set_input()                  slot changed and the radio can
 *     seq++ (odd), fence, copy, fence,      take a packet without queuing:
 *     seq++ (even), pending = 1               alloc buffer, write the INPUT
 *                                             header, copy the slot straight
 *                                             into it, retry if seq moved,
 *                                             hand it to L2CAP / ATT
 *
 * No locks, no atomic read-modify-write (Cortex-M0+ has none): one writer
 * per report ID. A slot overwritten before it could be sent counts in
 * floss_stats().hid_reports_coalesced. The input-path statistics of
 * floss_stats() (hid_reports_sent, hid_report_age, hid_send_interval,
 * hid_send_deferred) are shared with every input profile, as HOGP does.
 *
 * When reports leave:
 *   report_interval_us == 0   send on change, as soon as the link can take
 *                             it (floss_link_poll on every floss_poll).
 *   report_interval_us  > 0   paced: drift-free timer (deadline += period,
 *                             resync when more than a period late) calls
 *                             on_report_tick, then sends changed slots.
 *   FLOSS_LINK_INTERVAL_CONN  paced at the transport's natural period: the
 *                             LE connection interval (follows updates);
 *                             1.25 ms (one poll pair) on Classic.
 *
 * "The link can take it" = a controller buffer is free AND fewer than
 * max_inflight x floss_hci_completion_batch() of our packets are still in
 * the controller: a report queued in the radio can no longer be replaced
 * by newer state (docs/PERFORMANCE.md findings 1 and 2).
 *
 * ===========================================================================
 *  WIRE FORMAT  (see floss_link.h; little endian; one PDU per SDU / value)
 * ===========================================================================
 *
 *   INPUT  01 seq(2) t_pad_us(4) id len data   pad -> dongle
 *   OUTPUT 02 seq(2) id len data               dongle -> pad
 *   PING   03 t_dongle_us(4)                   dongle -> pad
 *   PONG   04 t_dongle_us(4) t_pad_us(4)       pad -> dongle
 *   HELLO  05 version role max_report flags    both, once per connection
 *
 * Decoding is strict: INPUT/OUTPUT/PING/PONG must have exactly their
 * length, HELLO at least its length (later versions may append fields),
 * report lengths at most FLOSS_LINK_MAX_REPORT, and each type only in its
 * direction. Anything else is dropped and counted (floss_stats().rx_dropped
 * and the "malformed" line of floss_link_stats_dump()). A version mismatch
 * in HELLO is logged, not fatal.
 *
 * ===========================================================================
 *  DONGLE STATISTICS
 * ===========================================================================
 *
 *   inputs_lost  sequence numbers skipped (16-bit, wrap-safe; a jump
 *                backwards means the pad restarted counting: no loss)
 *   rx_interval  time between consecutive INPUT arrivals
 *   rx_jitter    (t_rx - t_pad) minus its running minimum: how much later
 *                than the luckiest report this one arrived. The clocks are
 *                independent, so only differences of this offset mean
 *                anything; its minimum is taken over a sliding window
 *                (FLOSS_LINK_JITTER_WINDOW_US, see floss_link_priv.h) so
 *                that crystal drift does not accumulate into fake jitter.
 *   rtt          PING -> PONG round trip (ping_period_ms > 0)
 */
#include <stdatomic.h>

#include "floss_link_priv.h"

#if FLOSS_ENABLE_LINK

/** Seqlock read attempts before deferring to the next poll. */
#define LINK_SEQLOCK_TRIES 3
/** Peer record: version, transport, addr type, addr(6). */
#define LINK_PEER_REC_VERSION 1u
#define LINK_PEER_REC_LEN 9
/** PONGs echoing a timestamp older than this are ignored (stale). */
#define LINK_RTT_MAX_US 10000000u

/* f1055000-7a3c-4b8e-9d2f-5c6a1e0b7d10 and its two characteristics. */
const uint8_t floss_link_uuid_service[16] = {0xF1, 0x05, 0x50, 0x00, 0x7A, 0x3C, 0x4B, 0x8E,
                                             0x9D, 0x2F, 0x5C, 0x6A, 0x1E, 0x0B, 0x7D, 0x10};
const uint8_t floss_link_uuid_input[16] = {0xF1, 0x05, 0x50, 0x01, 0x7A, 0x3C, 0x4B, 0x8E,
                                           0x9D, 0x2F, 0x5C, 0x6A, 0x1E, 0x0B, 0x7D, 0x10};
const uint8_t floss_link_uuid_output[16] = {0xF1, 0x05, 0x50, 0x02, 0x7A, 0x3C, 0x4B, 0x8E,
                                            0x9D, 0x2F, 0x5C, 0x6A, 0x1E, 0x0B, 0x7D, 0x10};

/* ------------------------------------------------------------------------ */
/* State                                                                    */
/* ------------------------------------------------------------------------ */

/**
 * One "latest wins" input mailbox slot. Fields above the divider are shared
 * with the producer and protected by the seqlock; below, stack only.
 */
typedef struct {
    volatile uint32_t seq;  /**< even = stable, odd = producer mid-write */
    uint8_t len;
    uint32_t t_set_us;      /**< floss_hal_time_us() when published */
    uint8_t data[FLOSS_LINK_MAX_REPORT];
    volatile uint8_t used;  /**< claimed (constant afterwards) */
    uint8_t id;
    /* ---- stack private ---- */
    uint32_t sent_seq;      /**< seq of the version last handed to the radio */
    bool count_coalesced;   /**< sent_seq belongs to this session */
    uint8_t last_len;       /**< content last sent (idle detection only) */
    uint8_t last[FLOSS_LINK_MAX_REPORT];
} link_slot_t;

static link_slot_t s_slots[FLOSS_LINK_MAX_INPUT_IDS];

/** Set by producers after publishing; cleared by the stack before scanning. */
static volatile uint8_t s_pending;

static struct {
    const floss_link_config_t *cfg;
    const floss_link_callbacks_t *cb;
    const floss_link_tp_t *tp;
    bool pad;                 /**< cfg->role == PAD (hot path shortcut) */
    floss_link_state_t state;
    bool ready;               /**< controller up */
    bool pair_request;        /**< floss_link_pair() before the controller was ready */
    bool session;             /**< data path up (CONNECTED) */
    floss_addr_t session_peer;
    bool have_peer;
    floss_link_peer_t peer;

    /* pad transmit */
    uint16_t tx_seq;          /**< sequence number of the next INPUT */
    uint32_t tick_deadline;
    uint32_t due_mask;        /**< paced: slots this tick still has to send */
    uint32_t force_mask;      /**< slots to send once regardless of change (new session) */
    bool defer_counted;
    bool have_last_send;
    uint32_t last_send_us;
    bool track_changes;       /**< report content changes to the transport (idle power) */
    bool hello_pending;
    bool pong_pending;
    uint32_t pong_t_dongle;

    /* dongle receive */
    bool rx_have_seq;
    uint16_t rx_last_seq;
    bool rx_have_last;
    uint32_t rx_last_us;
    bool jit_have_cur, jit_have_prev;
    uint32_t jit_min_cur, jit_min_prev; /* modular offsets t_rx - t_pad */
    uint32_t jit_window_start;
    uint16_t out_seq;

    /* the peer's HELLO */
    bool peer_hello;
    uint8_t peer_version;
    uint8_t peer_flags;

    floss_timer_t tick_timer;
    floss_timer_t ping_timer;
    floss_timer_t pair_timer;

    floss_link_stats_t stats;
    uint32_t rx_malformed;
    uint32_t rx_ignored;      /**< valid PDUs that arrived while no session was up */
} L;

static const floss_link_callbacks_t s_no_callbacks;

static void link_input_service(void);

/* ------------------------------------------------------------------------ */
/* Small helpers                                                            */
/* ------------------------------------------------------------------------ */

static inline void link_fence(void) {
    /* Full barrier: orders the plain data copy against the seq stores/loads,
     * for the compiler and (DMB) for the other core. */
    atomic_thread_fence(memory_order_seq_cst);
}

static const char *state_name(floss_link_state_t st) {
    switch (st) {
    case FLOSS_LINK_STATE_IDLE:       return "IDLE";
    case FLOSS_LINK_STATE_PAIRING:    return "PAIRING";
    case FLOSS_LINK_STATE_CONNECTING: return "CONNECTING";
    case FLOSS_LINK_STATE_CONNECTED:  return "CONNECTED";
    default:                          return "?";
    }
}

static const char *role_name(uint8_t role) {
    return role == FLOSS_LINK_ROLE_PAD ? "pad" : role == FLOSS_LINK_ROLE_DONGLE ? "dongle" : "?";
}

const floss_link_config_t *floss_link_cfg(void) {
    return L.cfg;
}

uint32_t floss_link_backoff_us(uint8_t attempt) {
    uint32_t d = FLOSS_LINK_RECONNECT_MIN_US;
    while (attempt-- > 0 && d < FLOSS_LINK_RECONNECT_MAX_US) {
        d *= 2u;
    }
    return FLOSS_MIN(d, FLOSS_LINK_RECONNECT_MAX_US);
}

bool floss_link_inflight_ok(uint16_t handle) {
    uint8_t m = L.cfg->max_inflight;
    if (m == 0) {
        return true;
    }
    /* Scaled by the controller's completion batching: the cap is about packets
     * truly queued in the radio, not packets whose completion is not reported yet. */
    const floss_conn_t *c = floss_hci_conn_by_handle(handle);
    return c == NULL || c->inflight < (uint16_t)(m * floss_hci_completion_batch());
}

/* ------------------------------------------------------------------------ */
/* Peer record                                                              */
/* ------------------------------------------------------------------------ */

static void peer_load(void) {
    uint8_t rec[LINK_PEER_REC_LEN + 1]; /* +1 detects oversized records */
    int n = floss_hal_store_read(FLOSS_LINK_STORE_KEY_PEER, rec, sizeof(rec));
    L.have_peer = false;
    if (n <= 0) {
        return;
    }
    if (n != LINK_PEER_REC_LEN || rec[0] != LINK_PEER_REC_VERSION) {
        FLOSS_LOGW(LINK, "peer record unreadable (len %d), ignored", n);
        return;
    }
    if (rec[1] != (uint8_t)L.cfg->transport) {
        /* Paired over the other technology: keep the record, but it is not ours. */
        FLOSS_LOGI(LINK, "stored peer belongs to the %s transport, ignored", rec[1] ? "LE" : "Classic");
        return;
    }
    L.peer.transport = rec[1];
    L.peer.addr_type = rec[2];
    memcpy(L.peer.addr.b, &rec[3], 6);
    if (floss_addr_is_zero(&L.peer.addr)) {
        return;
    }
    L.have_peer = true;
}

bool floss_link_peer_get(floss_link_peer_t *out) {
    if (L.have_peer && out) {
        *out = L.peer;
    }
    return L.have_peer;
}

void floss_link_peer_set(uint8_t addr_type, const floss_addr_t *addr) {
    if (L.have_peer && L.peer.addr_type == addr_type && floss_addr_eq(&L.peer.addr, addr)) {
        return; /* unchanged: no flash write */
    }
    L.peer.transport = (uint8_t)L.cfg->transport;
    L.peer.addr_type = addr_type;
    L.peer.addr = *addr;
    L.have_peer = true;

    uint8_t rec[LINK_PEER_REC_LEN];
    rec[0] = LINK_PEER_REC_VERSION;
    rec[1] = L.peer.transport;
    rec[2] = addr_type;
    memcpy(&rec[3], addr->b, 6);
    floss_err_t err = floss_hal_store_write(FLOSS_LINK_STORE_KEY_PEER, rec, sizeof(rec));
    char s[18];
    floss_addr_to_str(addr, s);
    if (err != FLOSS_OK) {
        FLOSS_LOGW(LINK, "peer %s: store write failed (%s)", s, floss_err_str(err));
    } else {
        FLOSS_LOGI(LINK, "paired with %s %s", L.pad ? "dongle" : "pad", s);
    }
}

void floss_link_peer_clear(void) {
    L.have_peer = false;
    memset(&L.peer, 0, sizeof(L.peer));
    floss_hal_store_delete(FLOSS_LINK_STORE_KEY_PEER);
}

/* ------------------------------------------------------------------------ */
/* State                                                                    */
/* ------------------------------------------------------------------------ */

void floss_link_set_state(floss_link_state_t st) {
    if (L.state == st) {
        return;
    }
    FLOSS_LOGI(LINK, "%s -> %s", state_name(L.state), state_name(st));
    L.state = st;
    if (st == FLOSS_LINK_STATE_PAIRING && L.have_peer) {
        floss_timer_start(&L.pair_timer, FLOSS_LINK_PAIR_TIMEOUT_US);
    } else if (st != FLOSS_LINK_STATE_PAIRING) {
        floss_timer_stop(&L.pair_timer);
    }
}

floss_link_state_t floss_link_rest_state(void) {
    if (!L.have_peer) {
        return FLOSS_LINK_STATE_PAIRING;
    }
    return L.cfg->auto_reconnect ? FLOSS_LINK_STATE_CONNECTING : FLOSS_LINK_STATE_IDLE;
}

static void pair_timer_fired(floss_timer_t *t, void *arg) {
    (void)t;
    (void)arg;
    if (L.state == FLOSS_LINK_STATE_PAIRING && L.have_peer) {
        FLOSS_LOGI(LINK, "pairing window closed, back to the bonded peer");
        L.tp->on_pair_timeout();
    }
}

bool floss_link_session_active(void) {
    return L.session;
}

/* ------------------------------------------------------------------------ */
/* Mailbox slots (seqlock)                                                  */
/* ------------------------------------------------------------------------ */

static link_slot_t *slot_find(uint8_t id) {
    for (unsigned i = 0; i < FLOSS_LINK_MAX_INPUT_IDS; i++) {
        link_slot_t *s = &s_slots[i];
        if (s->used) {
            link_fence(); /* id was written before used was set */
            if (s->id == id) {
                return s;
            }
        }
    }
    return NULL;
}

/** First publish of a report ID: claim a free slot (rare: takes the HAL critical section). */
static link_slot_t *slot_claim(uint8_t id) {
    uint32_t cs = floss_hal_critical_enter();
    link_slot_t *found = slot_find(id);
    for (unsigned i = 0; !found && i < FLOSS_LINK_MAX_INPUT_IDS; i++) {
        link_slot_t *s = &s_slots[i];
        if (!s->used) {
            s->id = id;
            s->seq = 0;
            s->sent_seq = 0;
            s->count_coalesced = false;
            s->last_len = 0;
            link_fence();
            s->used = 1;
            found = s;
        }
    }
    floss_hal_critical_exit(cs);
    return found;
}

/**
 * Seqlock read: copy a consistent snapshot of the slot into out (room for
 * FLOSS_LINK_MAX_REPORT bytes). false if the producer kept changing it
 * (only possible from the other core); the caller defers.
 */
static bool slot_read(const link_slot_t *s, uint8_t *out, uint8_t *len, uint32_t *t_set, uint32_t *seq_out) {
    for (int tries = 0; tries < LINK_SEQLOCK_TRIES; tries++) {
        uint32_t s1 = s->seq;
        if (s1 & 1u) {
            continue; /* write in progress on the other core */
        }
        link_fence();
        uint8_t n = s->len;
        if (n > FLOSS_LINK_MAX_REPORT) {
            n = FLOSS_LINK_MAX_REPORT; /* only a torn len can exceed it; the seq check rejects it */
        }
        memcpy(out, s->data, n);
        uint32_t t = s->t_set_us;
        link_fence();
        if (s->seq == s1) {
            *len = n;
            *t_set = t;
            *seq_out = s1;
            return true;
        }
    }
    return false;
}

/** Slots whose content was published since it was last sent. */
static uint32_t changed_mask(void) {
    uint32_t mask = 0;
    for (unsigned i = 0; i < FLOSS_LINK_MAX_INPUT_IDS; i++) {
        const link_slot_t *s = &s_slots[i];
        if (s->used && s->seq != s->sent_seq) {
            mask |= 1u << i;
        }
    }
    return mask;
}

/** Slots that hold a report (published at least once). */
static uint32_t published_mask(void) {
    uint32_t mask = 0;
    for (unsigned i = 0; i < FLOSS_LINK_MAX_INPUT_IDS; i++) {
        const link_slot_t *s = &s_slots[i];
        if (s->used && s->seq != 0) {
            mask |= 1u << i;
        }
    }
    return mask;
}

floss_err_t floss_link_set_input(uint8_t report_id, const uint8_t *data, uint8_t len) {
    if (len > FLOSS_LINK_MAX_REPORT || (len && !data)) {
        return FLOSS_ERR_INVALID;
    }
    link_slot_t *s = slot_find(report_id);
    if (!s) {
        s = slot_claim(report_id);
        if (!s) {
            return FLOSS_ERR_NOMEM; /* raise FLOSS_LINK_MAX_INPUT_IDS */
        }
    }
    /* Seqlock write. Single writer per slot: plain load/store of seq. */
    uint32_t seq = s->seq;
    s->seq = seq + 1u;
    link_fence();
    s->len = len;
    if (len) {
        memcpy(s->data, data, len);
    }
    s->t_set_us = floss_hal_time_us();
    link_fence();
    s->seq = seq + 2u;

    s_pending = 1;
    FLOSS_TRACE(FLOSS_TR_HID_SET_INPUT, report_id);
    return FLOSS_OK;
}

/* ------------------------------------------------------------------------ */
/* Pad: input transmission (stack context)                                  */
/* ------------------------------------------------------------------------ */

static void note_deferred(void) {
    if (!L.defer_counted) {
        L.defer_counted = true;
        FLOSS_STAT_INC(hid_send_deferred);
        FLOSS_TRACE(FLOSS_TR_HID_DEFER, 0);
    }
}

/**
 * Build one INPUT straight from a mailbox slot into a radio buffer and send
 * it. The caller checked can_send().
 * @return FLOSS_OK, FLOSS_ERR_BUSY (try later), or another error when the
 *         report can never be sent (it is then marked sent).
 */
static floss_err_t send_slot(link_slot_t *s) {
    uint8_t *p;
    uint16_t cap;
    floss_buf_t *b = L.tp->input_alloc(&p, &cap);
    if (!b) {
        return FLOSS_ERR_BUSY;
    }
    uint8_t len;
    uint32_t t_set, seq;
    if (!slot_read(s, p + FLOSS_LINK_INPUT_HDR, &len, &t_set, &seq)) {
        floss_buf_free(b);
        return FLOSS_ERR_BUSY;
    }
    uint16_t total = (uint16_t)(FLOSS_LINK_INPUT_HDR + len);
    if (total > cap) {
        FLOSS_LOGE(LINK, "input report %u (%u bytes) does not fit the link (%u)", s->id, total, cap);
        floss_buf_free(b);
        s->sent_seq = seq;
        return FLOSS_ERR_INVALID;
    }
    if (L.track_changes) {
        /* Idle power: only a different content counts as activity. */
        if (!s->count_coalesced || len != s->last_len || memcmp(s->last, p + FLOSS_LINK_INPUT_HDR, len) != 0) {
            memcpy(s->last, p + FLOSS_LINK_INPUT_HDR, len);
            s->last_len = len;
            L.tp->on_input_change();
        }
    }
    uint32_t now = floss_hal_time_us();
    p[0] = FLOSS_LINK_PDU_INPUT;
    floss_put_le16(&p[1], L.tx_seq);
    floss_put_le32(&p[3], now); /* t_pad: handed to the radio now */
    p[7] = s->id;
    p[8] = len;

    FLOSS_DBG_PIN(FLOSS_DBG_PIN_HID_TX, 1);
    floss_err_t err = L.tp->input_send(b, total);
    FLOSS_DBG_PIN(FLOSS_DBG_PIN_HID_TX, 0);
    if (err != FLOSS_OK) {
        return err; /* buffer consumed either way; the seq number is reused */
    }
    L.tx_seq++;
    L.stats.inputs_sent++;

    /* Each publish advances seq by 2: more than one version since the last
     * send means intermediate states were superseded. */
    uint32_t versions = (seq - s->sent_seq) / 2u;
    if (s->count_coalesced && versions > 1) {
        FLOSS_STAT_ADD(hid_reports_coalesced, versions - 1);
    }
    s->count_coalesced = true;
    s->sent_seq = seq;

    uint32_t age = now - t_set;
    FLOSS_STAT_INC(hid_reports_sent);
    FLOSS_STAT_HIST(hid_report_age, age);
    FLOSS_TRACE(FLOSS_TR_HID_SEND, age);
    if (L.have_last_send) {
        FLOSS_STAT_HIST(hid_send_interval, now - L.last_send_us);
    }
    L.have_last_send = true;
    L.last_send_us = now;
    return FLOSS_OK;
}

/**
 * Send whatever the mailbox says is due, while the link can take packets
 * without queuing. Cheap when idle: one flag (on change) or one mask (paced).
 */
static void link_input_service(void) {
    uint32_t want;
    if (L.cfg->report_interval_us == 0) {
        if (!s_pending && !L.force_mask) {
            return;
        }
        s_pending = 0;
        link_fence(); /* clear before scanning: a publish after this re-arms it */
        want = changed_mask() | L.force_mask;
    } else {
        want = L.due_mask;
    }
    for (unsigned i = 0; want && i < FLOSS_LINK_MAX_INPUT_IDS; i++) {
        uint32_t bit = 1u << i;
        if (!(want & bit)) {
            continue;
        }
        if (!L.tp->can_send()) {
            /* Saturated: the rest stays due; the newest data goes when room returns. */
            note_deferred();
            s_pending = 1;
            return;
        }
        floss_err_t err = send_slot(&s_slots[i]);
        if (err == FLOSS_ERR_BUSY) {
            s_pending = 1; /* torn read or no buffer: retry next poll */
            continue;
        }
        want &= ~bit;
        L.due_mask &= ~bit;
        L.force_mask &= ~bit;
        if (err == FLOSS_OK && L.cfg->report_interval_us == 0) {
            L.defer_counted = false;
        }
    }
}

/** Current pacing period (us). */
static uint32_t pacing_period_us(void) {
    uint32_t iv = L.cfg->report_interval_us;
    if (iv == FLOSS_LINK_INTERVAL_CONN) {
        uint32_t p = L.tp->conn_period_us();
        return p ? p : 7500u;
    }
    return iv;
}

/** Pacing timer: drift-free period, then sample + send. */
static void tick_fired(floss_timer_t *t, void *arg) {
    (void)arg;
    uint32_t period = pacing_period_us();
    uint32_t now = floss_hal_time_us();
    L.tick_deadline += period;
    if (floss_time_diff(now, L.tick_deadline) >= 0) {
        /* More than one period behind (stack starved): resync, no burst. */
        L.tick_deadline = now + period;
    }
    floss_timer_start_at(t, L.tick_deadline);

    if (!L.session) {
        return;
    }
    if (L.cb->on_report_tick) {
        L.cb->on_report_tick();
    }
    L.due_mask |= changed_mask() | L.force_mask;
    L.defer_counted = false; /* each tick may count one deferral */
    link_input_service();
}

floss_err_t floss_link_shift_phase(int32_t delta_us) {
    if (!L.cfg || !L.pad || !L.session || L.cfg->report_interval_us == 0 ||
        L.cfg->report_interval_us == FLOSS_LINK_INTERVAL_CONN) {
        return FLOSS_ERR_STATE;
    }
    L.tick_deadline += (uint32_t)delta_us;
    floss_timer_start_at(&L.tick_timer, L.tick_deadline);
    return FLOSS_OK;
}

static void start_pacing(void) {
    floss_timer_stop(&L.tick_timer);
    L.due_mask = 0;
    if (L.session && L.pad && L.cfg->report_interval_us > 0) {
        L.tick_deadline = floss_hal_time_us() + pacing_period_us();
        floss_timer_start_at(&L.tick_timer, L.tick_deadline);
    }
}

/* ------------------------------------------------------------------------ */
/* Control PDUs                                                             */
/* ------------------------------------------------------------------------ */

static void send_hello(void) {
    uint8_t flags = 0;
    if (L.pad && L.cfg->transport == FLOSS_LINK_CLASSIC && L.cfg->classic_idle_ms > 0) {
        flags |= FLOSS_LINK_HELLO_F_IDLE_SNIFF;
    }
    uint8_t p[FLOSS_LINK_HELLO_LEN] = {FLOSS_LINK_PDU_HELLO, FLOSS_LINK_VERSION, (uint8_t)L.cfg->role,
                                       (uint8_t)FLOSS_LINK_MAX_REPORT, flags};
    floss_err_t err = L.tp->send_ctrl(p, sizeof(p));
    L.hello_pending = err == FLOSS_ERR_BUSY;
    if (err != FLOSS_OK && err != FLOSS_ERR_BUSY) {
        FLOSS_LOGW(LINK, "HELLO not sent: %s", floss_err_str(err));
    }
}

static void send_pong(void) {
    uint8_t p[FLOSS_LINK_PONG_LEN];
    p[0] = FLOSS_LINK_PDU_PONG;
    floss_put_le32(&p[1], L.pong_t_dongle);
    floss_put_le32(&p[5], floss_hal_time_us());
    floss_err_t err = L.tp->send_ctrl(p, sizeof(p));
    L.pong_pending = err == FLOSS_ERR_BUSY;
}

static void ping_fired(floss_timer_t *t, void *arg) {
    (void)arg;
    if (!L.session || L.cfg->ping_period_ms == 0) {
        return;
    }
    floss_timer_start(t, L.cfg->ping_period_ms * 1000u);
    uint8_t p[FLOSS_LINK_PING_LEN];
    p[0] = FLOSS_LINK_PDU_PING;
    floss_put_le32(&p[1], floss_hal_time_us());
    (void)L.tp->send_ctrl(p, sizeof(p)); /* busy: skip this round */
}

floss_err_t floss_link_send_output(uint8_t report_id, const uint8_t *data, uint8_t len) {
    if (!L.cfg) {
        return FLOSS_ERR_STATE;
    }
    if (L.pad) {
        return FLOSS_ERR_UNSUPPORTED;
    }
    if (len > FLOSS_LINK_MAX_REPORT || (len && !data)) {
        return FLOSS_ERR_INVALID;
    }
    if (!L.session) {
        return FLOSS_ERR_STATE;
    }
    uint8_t p[FLOSS_LINK_OUTPUT_HDR + FLOSS_LINK_MAX_REPORT];
    p[0] = FLOSS_LINK_PDU_OUTPUT;
    floss_put_le16(&p[1], L.out_seq);
    p[3] = report_id;
    p[4] = len;
    if (len) {
        memcpy(&p[FLOSS_LINK_OUTPUT_HDR], data, len);
    }
    floss_err_t err = L.tp->send_ctrl(p, (uint16_t)(FLOSS_LINK_OUTPUT_HDR + len));
    if (err == FLOSS_OK) {
        L.out_seq++;
        L.stats.outputs_sent++;
    }
    return err;
}

/* ------------------------------------------------------------------------ */
/* Receive                                                                  */
/* ------------------------------------------------------------------------ */

static void rx_bad(const uint8_t *pdu, uint16_t len, const char *why) {
    L.rx_malformed++;
    FLOSS_STAT_INC(rx_dropped);
    FLOSS_LOGD(LINK, "dropped PDU 0x%02X (%u bytes): %s", len ? pdu[0] : 0, len, why);
}

/** Running minimum of the clock offset over the current + previous window. */
static uint32_t jitter_sample(uint32_t offset, uint32_t now) {
    if (L.jit_have_cur && (uint32_t)(now - L.jit_window_start) >= FLOSS_LINK_JITTER_WINDOW_US) {
        L.jit_min_prev = L.jit_min_cur;
        L.jit_have_prev = true;
        L.jit_have_cur = false;
    }
    if (!L.jit_have_cur) {
        L.jit_min_cur = offset;
        L.jit_have_cur = true;
        L.jit_window_start = now;
    } else if ((int32_t)(offset - L.jit_min_cur) < 0) {
        L.jit_min_cur = offset;
    }
    uint32_t base = L.jit_min_cur;
    if (L.jit_have_prev && (int32_t)(L.jit_min_prev - base) < 0) {
        base = L.jit_min_prev;
    }
    return offset - base; /* >= 0: base <= min_cur <= offset (modular, small) */
}

static void rx_input(const uint8_t *pdu, uint16_t len) {
    if (len < FLOSS_LINK_INPUT_HDR) {
        rx_bad(pdu, len, "short INPUT");
        return;
    }
    uint8_t dlen = pdu[8];
    if (dlen > FLOSS_LINK_MAX_REPORT || len != FLOSS_LINK_INPUT_HDR + dlen) {
        rx_bad(pdu, len, "INPUT length mismatch");
        return;
    }
    uint32_t now = floss_hal_time_us();
    floss_link_rx_info_t info;
    info.seq = floss_get_le16(&pdu[1]);
    info.t_pad_us = floss_get_le32(&pdu[3]);
    info.t_rx_us = now;
    info.lost = 0;
    if (L.rx_have_seq) {
        uint16_t d = (uint16_t)(info.seq - L.rx_last_seq);
        if (d != 0 && d < 0x8000u) {
            info.lost = (uint16_t)(d - 1u);
        }
        /* d == 0 (repeat) or a jump backwards (the pad restarted counting): no loss. */
    }
    L.rx_have_seq = true;
    L.rx_last_seq = info.seq;

    L.stats.inputs_received++;
    L.stats.inputs_lost += info.lost;
    if (L.rx_have_last) {
        floss_hist_add(&L.stats.rx_interval, now - L.rx_last_us);
    }
    L.rx_have_last = true;
    L.rx_last_us = now;
    floss_hist_add(&L.stats.rx_jitter, jitter_sample(now - info.t_pad_us, now));

    if (L.cb->on_input) {
        L.cb->on_input(pdu[7], &pdu[FLOSS_LINK_INPUT_HDR], dlen, &info);
    }
}

static void rx_output(const uint8_t *pdu, uint16_t len) {
    if (len < FLOSS_LINK_OUTPUT_HDR) {
        rx_bad(pdu, len, "short OUTPUT");
        return;
    }
    uint8_t dlen = pdu[4];
    if (dlen > FLOSS_LINK_MAX_REPORT || len != FLOSS_LINK_OUTPUT_HDR + dlen) {
        rx_bad(pdu, len, "OUTPUT length mismatch");
        return;
    }
    L.stats.outputs_received++;
    FLOSS_STAT_INC(hid_output_reports);
    FLOSS_TRACE(FLOSS_TR_HID_OUTPUT, pdu[3]);
    if (L.cb->on_output) {
        L.cb->on_output(pdu[3], &pdu[FLOSS_LINK_OUTPUT_HDR], dlen);
    }
}

static void rx_hello(const uint8_t *pdu, uint16_t len) {
    if (len < FLOSS_LINK_HELLO_LEN) {
        rx_bad(pdu, len, "short HELLO");
        return;
    }
    L.peer_hello = true;
    L.peer_version = pdu[1];
    L.peer_flags = pdu[4];
    if (pdu[1] != FLOSS_LINK_VERSION) {
        FLOSS_LOGW(LINK, "peer speaks link protocol v%u, we speak v%u", pdu[1], FLOSS_LINK_VERSION);
    }
    if (pdu[2] == (uint8_t)L.cfg->role) {
        FLOSS_LOGE(LINK, "peer is a %s too: check the firmwares", role_name(pdu[2]));
    }
    if (L.pad && pdu[3] < FLOSS_LINK_MAX_REPORT) {
        FLOSS_LOGW(LINK, "dongle accepts reports up to %u bytes only", pdu[3]);
    }
    FLOSS_LOGI(LINK, "peer HELLO: v%u %s, max report %u, flags 0x%02X", pdu[1], role_name(pdu[2]), pdu[3], pdu[4]);
    if (L.tp->on_peer_hello) {
        L.tp->on_peer_hello(pdu[4]);
    }
}

void floss_link_rx_pdu(const uint8_t *pdu, uint16_t len) {
    if (!L.cfg) {
        return;
    }
    if (len == 0 || !pdu) {
        rx_bad(pdu, 0, "empty");
        return;
    }
    if (!L.session) {
        L.rx_ignored++;
        return;
    }
    switch (pdu[0]) {
    case FLOSS_LINK_PDU_INPUT:
        if (!L.pad) {
            rx_input(pdu, len);
            return;
        }
        break;
    case FLOSS_LINK_PDU_OUTPUT:
        if (L.pad) {
            rx_output(pdu, len);
            return;
        }
        break;
    case FLOSS_LINK_PDU_PING:
        if (L.pad) {
            if (len != FLOSS_LINK_PING_LEN) {
                rx_bad(pdu, len, "PING length");
                return;
            }
            L.pong_t_dongle = floss_get_le32(&pdu[1]); /* newest wins if an echo is still pending */
            send_pong();
            return;
        }
        break;
    case FLOSS_LINK_PDU_PONG:
        if (!L.pad) {
            if (len != FLOSS_LINK_PONG_LEN) {
                rx_bad(pdu, len, "PONG length");
                return;
            }
            uint32_t rtt = floss_hal_time_us() - floss_get_le32(&pdu[1]);
            if (rtt <= LINK_RTT_MAX_US) {
                floss_hist_add(&L.stats.rtt, rtt);
            }
            return;
        }
        break;
    case FLOSS_LINK_PDU_HELLO:
        rx_hello(pdu, len);
        return;
    default:
        rx_bad(pdu, len, "unknown type");
        return;
    }
    rx_bad(pdu, len, "wrong direction");
}

/* ------------------------------------------------------------------------ */
/* Session                                                                  */
/* ------------------------------------------------------------------------ */

void floss_link_session_up(const floss_addr_t *peer) {
    L.session = true;
    L.session_peer = *peer;
    L.rx_have_seq = false;
    L.rx_have_last = false;
    L.jit_have_cur = false;
    L.jit_have_prev = false;
    L.peer_hello = false;
    L.pong_pending = false;
    floss_link_set_state(FLOSS_LINK_STATE_CONNECTED);

    if (L.pad) {
        /* Send the current state of every report once: the dongle should
         * not wait for the next change to know where the sticks are. */
        L.force_mask = published_mask();
        for (unsigned i = 0; i < FLOSS_LINK_MAX_INPUT_IDS; i++) {
            s_slots[i].count_coalesced = false;
        }
        L.defer_counted = false;
        L.have_last_send = false;
        start_pacing();
    } else if (L.cfg->ping_period_ms) {
        floss_timer_start(&L.ping_timer, L.cfg->ping_period_ms * 1000u);
    }

    char s[18];
    floss_addr_to_str(peer, s);
    FLOSS_LOGI(LINK, "connected to %s %s (%s)", L.pad ? "dongle" : "pad", s, L.tp->name);
    send_hello();
    if (L.cb->on_connected) {
        L.cb->on_connected(peer);
    }
    if (L.pad) {
        s_pending = 1;
        link_input_service();
    }
}

void floss_link_session_down(uint8_t reason) {
    if (!L.session) {
        return;
    }
    L.session = false;
    floss_timer_stop(&L.tick_timer);
    floss_timer_stop(&L.ping_timer);
    L.due_mask = 0;
    L.force_mask = 0;
    L.hello_pending = false;
    L.pong_pending = false;
    FLOSS_LOGI(LINK, "disconnected (%s)", floss_hci_status_str(reason));
    if (L.cb->on_disconnected) {
        L.cb->on_disconnected(reason);
    }
}

/* ------------------------------------------------------------------------ */
/* Run loop, GAP glue                                                       */
/* ------------------------------------------------------------------------ */

/** floss_link_poll() target: a handful of loads when there is nothing to do. */
static void link_poll(void) {
    if (L.session) {
        if (L.hello_pending) {
            send_hello();
        }
        if (L.pong_pending) {
            send_pong();
        }
        if (L.pad) {
            link_input_service();
        }
    }
    if (L.tp->poll) {
        L.tp->poll();
    }
}

static void link_gap_event(const floss_gap_evt_t *evt, void *user) {
    (void)user;
    if (evt->type == FLOSS_GAP_EVT_READY) {
        /* (Re)started controller: no link survived. */
        floss_link_session_down(FLOSS_HCI_ERR_UNSPECIFIED);
        L.ready = true;
        L.tp->on_ready();
        if (L.pair_request) {
            L.pair_request = false;
            L.tp->pair();
        }
        return;
    }
    if (L.tp->on_gap_event) {
        L.tp->on_gap_event(evt);
    }
}

/* ------------------------------------------------------------------------ */
/* Public API                                                               */
/* ------------------------------------------------------------------------ */

void floss_link_config_defaults(floss_link_config_t *cfg, floss_link_role_t role, floss_link_transport_t transport) {
    if (!cfg) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->role = role;
    cfg->transport = transport;
    cfg->name = role == FLOSS_LINK_ROLE_PAD ? "FLOSS Pad" : NULL;
    if (role == FLOSS_LINK_ROLE_PAD) {
        /* Classic: paced at 1 kHz gives the most even delivery
         * (docs/PERFORMANCE.md finding 6). LE: one fresh report per
         * connection event (HOGP measurement in the same document). */
        cfg->report_interval_us = transport == FLOSS_LINK_LE ? FLOSS_LINK_INTERVAL_CONN : 1000u;
    }
    cfg->max_inflight = 2;            /* x completion_batch: findings 1, 2 */
    cfg->classic_poll_us = 1250;      /* one poll per slot pair: 800 Hz */
    cfg->classic_idle_ms = 0;
    cfg->classic_idle_sniff_slots = 16; /* 10 ms */
    cfg->classic_afh = FLOSS_LINK_AFH_FREEZE;
    cfg->classic_afh_learn_ms = 4000;
    cfg->classic_afh_refresh_s = 0;
    cfg->le_interval_min = 6;         /* 7.5 ms */
    cfg->le_interval_max = 6;
    cfg->le_peripheral_latency = 0;
    cfg->le_supervision_timeout = 200; /* 2 s */
    cfg->le_2m_phy = true;
    cfg->ping_period_ms = 0;
    cfg->auto_reconnect = true;
    cfg->classic_store_keys = true;
}

static floss_err_t validate(const floss_link_config_t *cfg) {
    if (cfg->role != FLOSS_LINK_ROLE_PAD && cfg->role != FLOSS_LINK_ROLE_DONGLE) {
        return FLOSS_ERR_INVALID;
    }
    if (cfg->transport != FLOSS_LINK_CLASSIC && cfg->transport != FLOSS_LINK_LE) {
        return FLOSS_ERR_INVALID;
    }
    if (cfg->transport == FLOSS_LINK_LE) {
#if !FLOSS_ENABLE_LE
        FLOSS_LOGE(LINK, "LE transport needs FLOSS_ENABLE_LE");
        return FLOSS_ERR_UNSUPPORTED;
#endif
        if (cfg->le_interval_min < 6 || cfg->le_interval_min > cfg->le_interval_max ||
            cfg->le_interval_max > 3200 || cfg->le_peripheral_latency > 499 ||
            cfg->le_supervision_timeout < 10 || cfg->le_supervision_timeout > 3200) {
            FLOSS_LOGE(LINK, "invalid LE connection parameters");
            return FLOSS_ERR_INVALID;
        }
    }
    if (cfg->classic_idle_ms > 0 &&
        (cfg->classic_idle_sniff_slots < 2 || (cfg->classic_idle_sniff_slots & 1u))) {
        FLOSS_LOGE(LINK, "classic_idle_sniff_slots must be even and >= 2");
        return FLOSS_ERR_INVALID;
    }
    return FLOSS_OK;
}

floss_err_t floss_link_init(const floss_link_config_t *cfg, const floss_link_callbacks_t *cb) {
    if (!cfg) {
        return FLOSS_ERR_INVALID;
    }
    floss_err_t err = validate(cfg);
    if (err != FLOSS_OK) {
        return err;
    }
    floss_link_poll_hook = NULL;
    memset(&L, 0, sizeof(L));
    memset(s_slots, 0, sizeof(s_slots));
    s_pending = 0;
    L.cfg = cfg;
    L.cb = cb ? cb : &s_no_callbacks;
    L.pad = cfg->role == FLOSS_LINK_ROLE_PAD;
    L.tp = cfg->transport == FLOSS_LINK_LE ? &floss_link_le : &floss_link_classic;
    L.state = FLOSS_LINK_STATE_IDLE;
    floss_timer_init(&L.tick_timer, tick_fired, NULL);
    floss_timer_init(&L.ping_timer, ping_fired, NULL);
    floss_timer_init(&L.pair_timer, pair_timer_fired, NULL);
    L.track_changes = L.pad && L.tp->on_input_change && cfg->transport == FLOSS_LINK_CLASSIC &&
                      cfg->classic_idle_ms > 0;
    peer_load();

    err = floss_gap_add_listener(link_gap_event, NULL);
    if (err == FLOSS_OK) {
        err = L.tp->init();
    }
    if (err != FLOSS_OK) {
        FLOSS_LOGE(LINK, "init failed: %s", floss_err_str(err));
        L.cfg = NULL;
        return err;
    }
    floss_link_poll_hook = link_poll;

    char s[18] = "none";
    if (L.have_peer) {
        floss_addr_to_str(&L.peer.addr, s);
    }
    FLOSS_LOGI(LINK, "%s over %s, peer %s, %s", role_name((uint8_t)cfg->role), L.tp->name, s,
               !L.pad ? "receiving"
               : cfg->report_interval_us == 0 ? "send on change"
               : cfg->report_interval_us == FLOSS_LINK_INTERVAL_CONN ? "paced per connection event"
                                                                     : "paced");
    return FLOSS_OK;
}

floss_err_t floss_link_pair(void) {
    if (!L.cfg) {
        return FLOSS_ERR_STATE;
    }
    if (!L.ready) {
        L.pair_request = true; /* applied once the controller is up */
        return FLOSS_OK;
    }
    if (L.state == FLOSS_LINK_STATE_PAIRING && L.have_peer) {
        floss_timer_start(&L.pair_timer, FLOSS_LINK_PAIR_TIMEOUT_US); /* re-open the window */
    }
    return L.tp->pair();
}

floss_err_t floss_link_forget(void) {
    if (!L.cfg) {
        return FLOSS_ERR_STATE;
    }
    floss_link_peer_t old;
    bool had = floss_link_peer_get(&old);
    floss_link_peer_clear();
    FLOSS_LOGI(LINK, "peer forgotten");
    if (L.ready) {
        L.tp->forget(had ? &old : NULL);
    }
    return FLOSS_OK;
}

floss_link_state_t floss_link_state(void) {
    return L.cfg ? L.state : FLOSS_LINK_STATE_IDLE;
}

const floss_link_stats_t *floss_link_stats(void) {
    return &L.stats;
}

void floss_link_stats_reset(void) {
    memset(&L.stats, 0, sizeof(L.stats));
    L.rx_malformed = 0;
    L.rx_ignored = 0;
}

uint32_t floss_link_rx_malformed(void) {
    return L.rx_malformed;
}

static void dump_hist(const char *name, const floss_hist_t *h) {
    if (h->count == 0) {
        FLOSS_LOGI(LINK, "  %-11s -", name);
        return;
    }
    FLOSS_LOGI(LINK, "  %-11s n %lu  min %lu  avg %lu  p50 %lu  p90 %lu  p99 %lu  max %lu us", name,
               (unsigned long)h->count, (unsigned long)h->min_us, (unsigned long)(h->sum_us / h->count),
               (unsigned long)floss_hist_percentile(h, 50), (unsigned long)floss_hist_percentile(h, 90),
               (unsigned long)floss_hist_percentile(h, 99), (unsigned long)h->max_us);
}

void floss_link_stats_dump(void) {
    if (!L.cfg) {
        FLOSS_LOGI(LINK, "link: not initialised");
        return;
    }
    const floss_link_stats_t *st = &L.stats;
    char s[18] = "none";
    if (L.have_peer) {
        floss_addr_to_str(&L.peer.addr, s);
    }
    FLOSS_LOGI(LINK, "link %s over %s: %s, peer %s", role_name((uint8_t)L.cfg->role), L.tp->name,
               state_name(L.state), s);
    if (L.pad) {
        FLOSS_LOGI(LINK, "  inputs sent %lu, outputs received %lu (next seq %u)", (unsigned long)st->inputs_sent,
                   (unsigned long)st->outputs_received, L.tx_seq);
    } else {
        uint32_t total = st->inputs_received + st->inputs_lost;
        uint32_t pct100 = total ? (uint32_t)(((uint64_t)st->inputs_lost * 10000u) / total) : 0;
        FLOSS_LOGI(LINK, "  inputs received %lu, lost %lu (%lu.%02lu %%), outputs sent %lu",
                   (unsigned long)st->inputs_received, (unsigned long)st->inputs_lost, (unsigned long)(pct100 / 100u),
                   (unsigned long)(pct100 % 100u), (unsigned long)st->outputs_sent);
        dump_hist("rx interval", &st->rx_interval);
        dump_hist("rx jitter", &st->rx_jitter);
        dump_hist("rtt", &st->rtt);
    }
    FLOSS_LOGI(LINK, "  PDUs dropped: %lu malformed, %lu outside a session", (unsigned long)L.rx_malformed,
               (unsigned long)L.rx_ignored);
    if (L.peer_hello) {
        FLOSS_LOGI(LINK, "  peer protocol v%u, flags 0x%02X", L.peer_version, L.peer_flags);
    }
}

#endif /* FLOSS_ENABLE_LINK */
