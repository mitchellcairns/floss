/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_hid.c - HID device role over L2CAP (HIDP): session management,
 * control-channel transactions, reconnection and the low-latency input path.
 *
 * ===========================================================================
 *  OVERVIEW
 * ===========================================================================
 *
 * A Classic HID session is two L2CAP channels on one baseband link:
 *
 *   control   (PSM 0x11)  request/response transactions (GET_REPORT, ...)
 *   interrupt (PSM 0x13)  unsolicited DATA: input reports up, output down
 *
 * Whoever created the baseband link opens the channels, control first. When
 * a new host pairs with us, the host does it; when we reconnect to a bonded
 * host (virtual cable, "reconnect initiate"), we page the host and open them
 * ourselves. The session is CONNECTED once both channels are open.
 *
 *   IDLE --(host opens ctrl / we page host)--> CONNECTING
 *   CONNECTING --(ctrl + intr open)--> CONNECTED   (on_connected)
 *   CONNECTED --(a channel closes / floss_hid_disconnect)--> DISCONNECTING
 *   any --(baseband link gone)--> IDLE             (on_disconnected)
 *
 * Teardown is always interrupt -> control -> link, which is the order the
 * HID profile expects; a watchdog timer forces the link down if the peer
 * stops answering. When a link disappears we make ourselves connectable
 * again (and discoverable if we have nobody to go back to).
 *
 * Reconnection: page the most recent bond; on a page failure retry a few
 * times with a backoff; if the host rejects our key (it forgot us) give up
 * immediately and become discoverable so the user can pair again.
 *
 * ===========================================================================
 *  CONTROL CHANNEL
 * ===========================================================================
 *
 * Every HIDP message starts with one header byte: (type << 4) | param. Each
 * request the host sends on control gets exactly one answer: either DATA
 * (GET_REPORT / GET_PROTOCOL / GET_IDLE) or a HANDSHAKE carrying a result
 * code. HID_CONTROL operations (suspend, unplug, ...) are not answered. The
 * host never has more than one transaction outstanding, so a single response
 * buffer suffices; if L2CAP cannot take it right now it is kept and retried
 * from floss_hid_poll() / on_can_send.
 *
 * ===========================================================================
 *  THE INPUT PATH  (performance critical)
 * ===========================================================================
 *
 * Goal: the report on the air is always the newest state the application
 * published, and it leaves as soon as the link can carry it. A queue would
 * be wrong for a game controller: under back-pressure it delivers stale
 * state late. Instead each report ID has ONE mailbox slot, latest wins.
 *
 *   producer (app main loop, ISR or the other core)
 *        floss_hid_set_input(id, data, len)
 *            slot.seq++            (odd: write in progress)
 *            fence
 *            copy data, len, t_set_us
 *            fence
 *            slot.seq++            (even: stable)
 *            pending = 1
 *
 *   consumer (stack context, floss_hid_poll / pacing timer / on_can_send)
 *        if interrupt channel has a controller credit:
 *            alloc ACL buffer (headroom for L2CAP + ACL + transport headers)
 *            write HIDP header 0xA1 (DATA|INPUT) and optional report ID
 *            s1 = seq; fence; copy slot straight into the buffer; fence;
 *            if seq != s1 or s1 odd: torn read -> retry (bounded), else defer
 *            floss_l2cap_send_buf(FLUSHABLE | PRIORITY)
 *            sent_seq = s1
 *
 * This is a seqlock. The writer never waits and never takes a lock, so it
 * is safe from interrupt handlers and from the second core. Its only
 * requirement is ONE writer per report ID (two contexts must not publish
 * the same ID concurrently). Only plain 32-bit volatile loads/stores and
 * fences are used - no atomic read-modify-write - because the Cortex-M0+
 * (RP2040) has no exclusive-access instructions. The reader copies the
 * slot directly into the outgoing ACL buffer: one copy from the app's
 * struct to the radio buffer, no intermediate staging.
 *
 * A slot whose seq advanced by more than one version since the last send
 * was overwritten before it could be sent; that is counted in
 * hid_reports_coalesced (normal when the app updates faster than the link).
 *
 * When reports leave:
 *   report_interval_us == 0  "send on change": any changed slot is sent as
 *       soon as the interrupt channel can send without queuing. Driven by
 *       floss_hid_poll() (called on every floss_poll()) and by credits
 *       returning (on_can_send).
 *   report_interval_us  > 0  paced: a drift-free timer (deadline += period,
 *       resync if more than one period late) calls on_report_tick() - the
 *       app's last chance to sample inputs - then sends changed slots (all
 *       slots with send_unchanged). If no credit is free the tick is counted
 *       in hid_send_deferred and the report leaves the moment a credit
 *       returns, still carrying the newest data.
 *
 * Packets are FLUSHABLE (with a link flush timeout the controller drops
 * reports that missed their window instead of delaying fresh ones) and
 * PRIORITY (they overtake queued signalling/SDP traffic on the link).
 *
 * Instrumentation: hid_reports_sent, hid_report_age (set_input -> handed to
 * HCI), hid_send_interval, hid_send_deferred, hid_reports_coalesced;
 * FLOSS_TR_HID_* trace events; debug pin FLOSS_DBG_PIN_HID_TX.
 */
#include <stdatomic.h>

#include "floss_internal.h"

/* ------------------------------------------------------------------------ */
/* Tunables (override with -D or the user config file)                      */
/* ------------------------------------------------------------------------ */

/** Largest report descriptor the SDP record buffer can hold. */
#ifndef FLOSS_HID_MAX_DESCRIPTOR
#define FLOSS_HID_MAX_DESCRIPTOR 512
#endif

/** Page retries after the first failed reconnect attempt. */
#ifndef FLOSS_HID_RECONNECT_RETRIES
#define FLOSS_HID_RECONNECT_RETRIES 3
#endif

/** Delay between reconnect attempts. */
#ifndef FLOSS_HID_RECONNECT_BACKOFF_US
#define FLOSS_HID_RECONNECT_BACKOFF_US 1000000u
#endif

/**
 * Stay page-scannable while a host is connected. Lets a second host (or the
 * same host after a crash) reach us, but page scan windows steal radio time
 * from the link: set 0 for the most consistent report timing.
 */
#ifndef FLOSS_HID_CONNECTABLE_WHILE_CONNECTED
#define FLOSS_HID_CONNECTABLE_WHILE_CONNECTED 1
#endif

/** After the host closes one channel, how long we wait for it to finish before tearing down ourselves. */
#ifndef FLOSS_HID_REMOTE_CLOSE_GRACE_US
#define FLOSS_HID_REMOTE_CLOSE_GRACE_US 1000000u
#endif

/** Local teardown watchdog: force the link down if channel disconnects stall. */
#ifndef FLOSS_HID_TEARDOWN_TIMEOUT_US
#define FLOSS_HID_TEARDOWN_TIMEOUT_US 2000000u
#endif

/** Sniff max interval (slots) requested while the host has us suspended. 0 = leave the link alone. */
#ifndef FLOSS_HID_SUSPEND_SNIFF_INTERVAL
#define FLOSS_HID_SUSPEND_SNIFF_INTERVAL 0
#endif

/** Seqlock read attempts before deferring to the next poll. */
#define HID_SEQLOCK_TRIES 3

#if FLOSS_HID_MAX_INPUT_IDS > 32
#error "FLOSS_HID_MAX_INPUT_IDS must be <= 32 (slot bitmasks are uint32_t)"
#endif

/* ------------------------------------------------------------------------ */
/* HIDP wire format                                                         */
/* ------------------------------------------------------------------------ */

/* Transaction types (high nibble of the header byte). */
#define HIDP_HANDSHAKE    0x0
#define HIDP_HID_CONTROL  0x1
#define HIDP_GET_REPORT   0x4
#define HIDP_SET_REPORT   0x5
#define HIDP_GET_PROTOCOL 0x6
#define HIDP_SET_PROTOCOL 0x7
#define HIDP_GET_IDLE     0x8
#define HIDP_SET_IDLE     0x9
#define HIDP_DATA         0xA
#define HIDP_DATC         0xB

#define HIDP_HDR(type, param) ((uint8_t)(((type) << 4) | ((param) & 0x0F)))

/** GET_REPORT param bit 3: a 2-byte BufferSize follows (after the report ID). */
#define HIDP_GET_REPORT_SIZE_PRESENT 0x08
#define HIDP_REPORT_TYPE_MASK        0x03

#define HID_NO_HANDLE 0xFFFFu

/* ------------------------------------------------------------------------ */
/* State                                                                    */
/* ------------------------------------------------------------------------ */

/**
 * One "latest wins" input mailbox slot (see THE INPUT PATH above).
 * Fields above the divider are shared with the producer and protected by
 * the seqlock; fields below are touched only by the stack.
 */
typedef struct {
    volatile uint32_t seq;  /**< even = stable, odd = producer mid-write */
    uint16_t len;
    uint32_t t_set_us;      /**< floss_hal_time_us() when published */
    uint8_t data[FLOSS_HID_MAX_REPORT];
    volatile uint8_t used;  /**< claimed (constant afterwards) */
    uint8_t id;             /**< report ID (0 = descriptor without IDs) */
    /* ---- stack private ---- */
    uint32_t sent_seq;      /**< seq of the version last handed to L2CAP */
    bool count_coalesced;   /**< sent_seq belongs to this session */
} hid_slot_t;

static hid_slot_t hid_slots[FLOSS_HID_MAX_INPUT_IDS];

/** Set by producers after publishing; cleared by the stack before scanning. */
static volatile uint8_t hid_pending;

static struct {
    const floss_hid_config_t *cfg;
    const floss_hid_callbacks_t *cb;

    floss_hid_state_t state;
    uint16_t handle;          /**< ACL handle of the host link, HID_NO_HANDLE if none */
    floss_addr_t host;
    floss_l2cap_chan_t *ctrl; /**< control channel (pending or open) */
    floss_l2cap_chan_t *intr; /**< interrupt channel (pending or open) */
    bool ctrl_open;
    bool intr_open;
    bool app_connected;       /**< on_connected delivered for this link */

    bool reconnecting;        /**< we are paging the host / opening its channels */
    uint8_t reconnect_retries;
    bool auth_failed;         /**< host rejected our key: be discoverable again */
    bool local_teardown;      /**< we are closing the session */
    bool link_disc_sent;

    uint8_t protocol;         /**< FLOSS_HID_PROTOCOL_* */
    uint8_t idle_rate;        /**< SET_IDLE value (4 ms units), reported back only */
    bool uses_report_ids;     /**< report ID byte present in reports */

    uint32_t interval_us;     /**< 0 = send on change */
    uint32_t tick_deadline;
    uint32_t due_mask;        /**< paced: slots this tick still has to send */
    uint32_t force_mask;      /**< slots to send once regardless of change (new session) */
    bool defer_counted;       /**< hid_send_deferred already counted for this wait */
    bool have_last_send;
    uint32_t last_send_us;

    floss_timer_t tick_timer;
    floss_timer_t reconnect_timer;
    floss_timer_t teardown_timer;

    /** Control-channel response under construction / waiting for L2CAP. */
    uint8_t ctrl_tx[2 + FLOSS_HID_MAX_REPORT];
    uint16_t ctrl_tx_len;     /**< non-zero = response waiting to be sent */
} hid;

static const floss_hid_callbacks_t hid_no_callbacks;

static uint8_t hid_sdp_buf[512 + FLOSS_HID_MAX_DESCRIPTOR];
static floss_sdp_record_t hid_sdp_rec;
static uint8_t did_sdp_buf[96];
static floss_sdp_record_t did_sdp_rec;

/* Forward declarations ---------------------------------------------------- */
static void hid_input_service(void);
static void hid_teardown_step(void);
static void hid_become_visible(void);
static const floss_l2cap_ops_t hid_ctrl_ops;
static const floss_l2cap_ops_t hid_intr_ops;

/* ------------------------------------------------------------------------ */
/* Small helpers                                                            */
/* ------------------------------------------------------------------------ */

static inline void hid_fence(void) {
    /* Full barrier: orders the plain data copy against the seq stores/loads,
     * for the compiler and (DMB) for the other core. */
    atomic_thread_fence(memory_order_seq_cst);
}

static void hid_log_addr(const char *what, const floss_addr_t *addr) {
    char s[18];
    floss_addr_to_str(addr, s);
    FLOSS_LOGI(HID, "%s %s", what, s);
}

static bool hid_is_key_failure(uint8_t status) {
    return status == FLOSS_HCI_ERR_AUTH_FAILURE || status == FLOSS_HCI_ERR_PIN_OR_KEY_MISSING;
}

/**
 * Does the report descriptor declare report IDs? Walks the short items
 * (prefix byte: tag/type in bits 7..2, size code in bits 1..0) looking for
 * a Global "Report ID" item (tag 0x84). Long items (0xFE) are skipped.
 */
static bool hid_descriptor_uses_ids(const uint8_t *d, uint16_t len) {
    uint16_t i = 0;
    while (i < len) {
        uint8_t prefix = d[i];
        if (prefix == 0xFE) {
            if (i + 1u >= len) {
                break;
            }
            i = (uint16_t)(i + 3u + d[i + 1]);
            continue;
        }
        uint8_t size = prefix & 0x03;
        if (size == 3) {
            size = 4;
        }
        if ((prefix & 0xFC) == 0x84) {
            return true;
        }
        i = (uint16_t)(i + 1u + size);
    }
    return false;
}

static uint32_t hid_cod_for_subclass(uint8_t subclass) {
    uint32_t cod = FLOSS_COD_MAJOR_PERIPHERAL;
    switch (subclass) {
    case FLOSS_HID_SUBCLASS_GAMEPAD:  cod |= FLOSS_COD_MINOR_GAMEPAD; break;
    case FLOSS_HID_SUBCLASS_JOYSTICK: cod |= FLOSS_COD_MINOR_JOYSTICK; break;
    case FLOSS_HID_SUBCLASS_KEYBOARD: cod |= FLOSS_COD_MINOR_KEYBOARD; break;
    case FLOSS_HID_SUBCLASS_MOUSE:    cod |= FLOSS_COD_MINOR_POINTING; break;
    case FLOSS_HID_SUBCLASS_COMBO:    cod |= FLOSS_COD_MINOR_KEYBOARD | FLOSS_COD_MINOR_POINTING; break;
    default: break;
    }
    return cod;
}

/* ------------------------------------------------------------------------ */
/* Mailbox slots (seqlock)                                                  */
/* ------------------------------------------------------------------------ */

static hid_slot_t *hid_slot_find(uint8_t id) {
    for (unsigned i = 0; i < FLOSS_HID_MAX_INPUT_IDS; i++) {
        hid_slot_t *s = &hid_slots[i];
        if (s->used) {
            hid_fence(); /* id was written before used was set */
            if (s->id == id) {
                return s;
            }
        }
    }
    return NULL;
}

/**
 * First publish of a report ID: claim a free slot. Rare, so it simply takes
 * the HAL critical section (two producers might claim concurrently).
 */
static hid_slot_t *hid_slot_claim(uint8_t id) {
    hid_slot_t *found = NULL;
    uint32_t cs = floss_hal_critical_enter();
    found = hid_slot_find(id);
    for (unsigned i = 0; !found && i < FLOSS_HID_MAX_INPUT_IDS; i++) {
        hid_slot_t *s = &hid_slots[i];
        if (!s->used) {
            s->id = id;
            s->seq = 0;
            s->sent_seq = 0;
            s->count_coalesced = false;
            hid_fence();
            s->used = 1;
            found = s;
        }
    }
    if (found && id != 0) {
        hid.uses_report_ids = true;
    }
    floss_hal_critical_exit(cs);
    return found;
}

/**
 * Seqlock read: copy a consistent snapshot of the slot into out (cap bytes).
 * Returns false if the producer kept changing it (only possible when the
 * producer runs on the other core); the caller defers to the next poll.
 */
static bool hid_slot_read(const hid_slot_t *s, uint8_t *out, uint16_t cap, uint16_t *len, uint32_t *t_set,
                          uint32_t *seq_out) {
    for (int tries = 0; tries < HID_SEQLOCK_TRIES; tries++) {
        uint32_t s1 = s->seq;
        if (s1 & 1u) {
            continue; /* write in progress on the other core */
        }
        hid_fence();
        uint16_t n = s->len;
        if (n > cap) {
            n = cap; /* only a torn len can exceed cap; the seq check rejects it */
        }
        memcpy(out, s->data, n);
        uint32_t t = s->t_set_us;
        hid_fence();
        if (s->seq == s1) {
            *len = n;
            *t_set = t;
            *seq_out = s1;
            return true;
        }
    }
    return false;
}

/** Bitmask of slots whose content changed since it was last sent. */
static uint32_t hid_changed_mask(void) {
    uint32_t mask = 0;
    for (unsigned i = 0; i < FLOSS_HID_MAX_INPUT_IDS; i++) {
        const hid_slot_t *s = &hid_slots[i];
        if (s->used && s->seq != s->sent_seq) {
            mask |= 1u << i;
        }
    }
    return mask;
}

/** Bitmask of slots that hold a report (published at least once). */
static uint32_t hid_published_mask(void) {
    uint32_t mask = 0;
    for (unsigned i = 0; i < FLOSS_HID_MAX_INPUT_IDS; i++) {
        const hid_slot_t *s = &hid_slots[i];
        if (s->used && s->seq != 0) {
            mask |= 1u << i;
        }
    }
    return mask;
}

floss_err_t floss_hid_set_input(uint8_t report_id, const uint8_t *data, uint16_t len) {
    if (len > FLOSS_HID_MAX_REPORT || (len && !data)) {
        return FLOSS_ERR_INVALID;
    }
    hid_slot_t *s = hid_slot_find(report_id);
    if (!s) {
        s = hid_slot_claim(report_id);
        if (!s) {
            return FLOSS_ERR_NOMEM; /* raise FLOSS_HID_MAX_INPUT_IDS */
        }
    }

    /* Seqlock write. Single writer per slot, so plain load/store of seq is
     * enough (no atomic RMW, which the Cortex-M0+ lacks). */
    uint32_t seq = s->seq;
    s->seq = seq + 1u;
    hid_fence();
    s->len = len;
    if (len) {
        memcpy(s->data, data, len);
    }
    s->t_set_us = floss_hal_time_us();
    hid_fence();
    s->seq = seq + 2u;

    hid_pending = 1;
    FLOSS_TRACE(FLOSS_TR_HID_SET_INPUT, report_id);
    return FLOSS_OK;
}

/* ------------------------------------------------------------------------ */
/* Input transmission (stack context)                                       */
/* ------------------------------------------------------------------------ */

static void hid_note_deferred(void) {
    if (!hid.defer_counted) {
        hid.defer_counted = true;
        FLOSS_STAT_INC(hid_send_deferred);
        FLOSS_TRACE(FLOSS_TR_HID_DEFER, 0);
    }
}

static void hid_note_sent(uint32_t now) {
    FLOSS_STAT_INC(hid_reports_sent);
    if (hid.have_last_send) {
        FLOSS_STAT_HIST(hid_send_interval, now - hid.last_send_us);
    }
    hid.have_last_send = true;
    hid.last_send_us = now;
}

/**
 * Build an input packet straight from a mailbox slot into a pool buffer and
 * hand it to L2CAP. The caller has checked that a credit is available.
 * @return FLOSS_OK, FLOSS_ERR_BUSY (try again later), or another error if
 *         the report can never be sent (it is then marked as sent).
 */
static floss_err_t hid_send_slot(hid_slot_t *s) {
    floss_buf_t *b = floss_buf_alloc();
    if (!b) {
        return FLOSS_ERR_BUSY;
    }
    uint8_t *p = b->data;
    uint16_t hdr = 0;
    p[hdr++] = HIDP_HDR(HIDP_DATA, FLOSS_HID_REPORT_INPUT); /* 0xA1 */
    if (s->id != 0) {
        p[hdr++] = s->id;
    }

    uint16_t len;
    uint32_t t_set, seq;
    if (!hid_slot_read(s, p + hdr, FLOSS_HID_MAX_REPORT, &len, &t_set, &seq)) {
        floss_buf_free(b);
        return FLOSS_ERR_BUSY;
    }
    uint16_t total = (uint16_t)(hdr + len);
    if (total > hid.intr->remote_mtu) {
        FLOSS_LOGE(HID, "input report %u (%u bytes) exceeds host MTU %u", s->id, total, hid.intr->remote_mtu);
        floss_buf_free(b);
        s->sent_seq = seq;
        return FLOSS_ERR_INVALID;
    }
    b->len = total;

    FLOSS_DBG_PIN(FLOSS_DBG_PIN_HID_TX, 1);
    floss_err_t err = floss_l2cap_send_buf(hid.intr, b, FLOSS_BUF_F_FLUSHABLE | FLOSS_BUF_F_PRIORITY);
    FLOSS_DBG_PIN(FLOSS_DBG_PIN_HID_TX, 0);
    if (err != FLOSS_OK) {
        return err; /* buf consumed by L2CAP either way */
    }

    /* Each publish advances seq by 2: more than one version since the last
     * send means intermediate states were superseded before the link could
     * carry them. */
    uint32_t versions = (seq - s->sent_seq) / 2u;
    if (s->count_coalesced && versions > 1) {
        FLOSS_STAT_ADD(hid_reports_coalesced, versions - 1);
    }
    s->count_coalesced = true;
    s->sent_seq = seq;

    uint32_t now = floss_hal_time_us();
    uint32_t age = now - t_set;
    FLOSS_STAT_HIST(hid_report_age, age);
    FLOSS_TRACE(FLOSS_TR_HID_SEND, age);
    hid_note_sent(now);
    return FLOSS_OK;
}

/**
 * Send whatever the mailbox says is due, as long as the interrupt channel
 * can transmit without queuing. Cheap when there is nothing to do: one state
 * check and one flag (unpaced) or one mask (paced).
 */
static void hid_input_service(void) {
    if (hid.state != FLOSS_HID_STATE_CONNECTED || !hid.intr_open) {
        return;
    }

    uint32_t want;
    if (hid.interval_us == 0) {
        if (!hid_pending && !hid.force_mask) {
            return;
        }
        hid_pending = 0;
        hid_fence(); /* clear before scanning: a publish after this re-arms it */
        want = hid_changed_mask() | hid.force_mask;
    } else {
        want = hid.due_mask;
    }

    for (unsigned i = 0; want && i < FLOSS_HID_MAX_INPUT_IDS; i++) {
        uint32_t bit = 1u << i;
        if (!(want & bit)) {
            continue;
        }
        if (!floss_l2cap_can_send_now(hid.intr)) {
            /* Link saturated: keep the rest due; the newest data goes out
             * when a credit returns (on_can_send / next poll). */
            hid_note_deferred();
            hid_pending = 1;
            return;
        }
        floss_err_t err = hid_send_slot(&hid_slots[i]);
        if (err == FLOSS_ERR_BUSY) {
            hid_pending = 1; /* torn read or no buffer: retry next poll */
            continue;
        }
        want &= ~bit;
        hid.due_mask &= ~bit;
        hid.force_mask &= ~bit;
        if (err == FLOSS_OK && hid.interval_us == 0) {
            hid.defer_counted = false;
        }
    }
}

/** Pacing timer: drift-free period, then sample + send. */
static void hid_tick(floss_timer_t *t, void *arg) {
    (void)arg;
    uint32_t now = floss_hal_time_us();
    hid.tick_deadline += hid.interval_us;
    if (floss_time_diff(now, hid.tick_deadline) >= 0) {
        /* More than one period behind (stack starved): resync instead of
         * firing a burst of catch-up ticks. */
        hid.tick_deadline = now + hid.interval_us;
    }
    floss_timer_start_at(t, hid.tick_deadline);

    if (hid.state != FLOSS_HID_STATE_CONNECTED) {
        return;
    }
    if (hid.cb->on_report_tick) {
        hid.cb->on_report_tick();
    }
    uint32_t due = hid_changed_mask() | hid.force_mask;
    if (hid.cfg->send_unchanged) {
        due |= hid_published_mask();
    }
    hid.due_mask |= due;
    hid.defer_counted = false; /* each tick may count one deferral */
    hid_input_service();
}

static void hid_start_pacing(void) {
    floss_timer_stop(&hid.tick_timer);
    hid.due_mask = 0;
    if (hid.state == FLOSS_HID_STATE_CONNECTED && hid.interval_us > 0) {
        hid.tick_deadline = floss_hal_time_us() + hid.interval_us;
        floss_timer_start_at(&hid.tick_timer, hid.tick_deadline);
    }
}

floss_err_t floss_hid_send_input_now(uint8_t report_id, const uint8_t *data, uint16_t len) {
    if (len > FLOSS_HID_MAX_REPORT || (len && !data)) {
        return FLOSS_ERR_INVALID;
    }
    if (hid.state != FLOSS_HID_STATE_CONNECTED || !hid.intr_open) {
        return FLOSS_ERR_STATE;
    }
    uint16_t total = (uint16_t)(1u + (report_id ? 1u : 0u) + len);
    if (total > hid.intr->remote_mtu) {
        return FLOSS_ERR_INVALID;
    }
    if (!floss_l2cap_can_send_now(hid.intr)) {
        return FLOSS_ERR_BUSY;
    }
    floss_buf_t *b = floss_buf_alloc();
    if (!b) {
        return FLOSS_ERR_BUSY;
    }
    uint8_t *p = b->data;
    *p++ = HIDP_HDR(HIDP_DATA, FLOSS_HID_REPORT_INPUT);
    if (report_id) {
        *p++ = report_id;
    }
    if (len) {
        memcpy(p, data, len);
    }
    b->len = total;

    FLOSS_DBG_PIN(FLOSS_DBG_PIN_HID_TX, 1);
    floss_err_t err = floss_l2cap_send_buf(hid.intr, b, FLOSS_BUF_F_FLUSHABLE | FLOSS_BUF_F_PRIORITY);
    FLOSS_DBG_PIN(FLOSS_DBG_PIN_HID_TX, 0);
    if (err == FLOSS_OK) {
        uint32_t now = floss_hal_time_us();
        FLOSS_STAT_HIST(hid_report_age, 0);
        FLOSS_TRACE(FLOSS_TR_HID_SEND, 0);
        hid_note_sent(now);
    }
    return err;
}

void floss_hid_set_report_interval(uint32_t interval_us) {
    hid.interval_us = interval_us;
    hid_start_pacing();
    hid_pending = 1; /* switching to send-on-change: flush anything changed */
}

/* ------------------------------------------------------------------------ */
/* Control channel                                                          */
/* ------------------------------------------------------------------------ */

/** Try to hand the pending control response to L2CAP. */
static void hid_ctrl_flush(void) {
    if (!hid.ctrl_tx_len) {
        return;
    }
    if (!hid.ctrl || !hid.ctrl_open) {
        hid.ctrl_tx_len = 0;
        return;
    }
    floss_err_t err = floss_l2cap_send(hid.ctrl, hid.ctrl_tx, hid.ctrl_tx_len);
    if (err == FLOSS_ERR_BUSY) {
        return; /* keep it; retried from floss_hid_poll / on_can_send */
    }
    if (err != FLOSS_OK) {
        FLOSS_LOGW(HID, "control response dropped: %s", floss_err_str(err));
    }
    hid.ctrl_tx_len = 0;
}

static void hid_ctrl_respond(uint16_t len) {
    if (hid.ctrl_tx_len) {
        /* The host broke the one-transaction-at-a-time rule while our last
         * answer is still waiting for a buffer; the newest request wins. */
        FLOSS_LOGW(HID, "control response overrun");
    }
    hid.ctrl_tx_len = len;
    hid_ctrl_flush();
}

static void hid_handshake(uint8_t result) {
    hid.ctrl_tx_len = 0; /* the buffer is about to be rewritten */
    hid.ctrl_tx[0] = HIDP_HDR(HIDP_HANDSHAKE, result);
    hid_ctrl_respond(1);
}

static void hid_deliver_output(uint8_t id, const uint8_t *data, uint16_t len) {
    FLOSS_STAT_INC(hid_output_reports);
    FLOSS_TRACE(FLOSS_TR_HID_OUTPUT, id);
    if (hid.cb->on_output_report) {
        hid.cb->on_output_report(id, data, len);
    }
}

/** Answer GET_REPORT(INPUT) from the mailbox. */
static uint8_t hid_mailbox_get(uint8_t id, uint8_t *out, uint16_t cap, uint16_t *len) {
    const hid_slot_t *s = hid_slot_find(id);
    if (!s) {
        return FLOSS_HID_HS_ERR_INVALID_REPORT_ID;
    }
    uint32_t t, seq;
    if (!hid_slot_read(s, out, cap, len, &t, &seq)) {
        return FLOSS_HID_HS_NOT_READY;
    }
    return FLOSS_HID_HS_SUCCESSFUL;
}

/*
 * GET_REPORT: [hdr][report ID if the descriptor uses IDs][BufferSize le16 if
 * param bit 3]. Answer DATA|type with the report, truncated to BufferSize
 * (which counts the report ID + report bytes) and to the channel MTU.
 */
static void hid_on_get_report(uint8_t param, const uint8_t *p, uint16_t len) {
    uint8_t type = param & HIDP_REPORT_TYPE_MASK;
    bool size_present = (param & HIDP_GET_REPORT_SIZE_PRESENT) != 0;
    uint16_t need = (uint16_t)((hid.uses_report_ids ? 1u : 0u) + (size_present ? 2u : 0u));
    if (type == FLOSS_HID_REPORT_OTHER || len < need) {
        hid_handshake(FLOSS_HID_HS_ERR_INVALID_PARAMETER);
        return;
    }
    uint8_t id = hid.uses_report_ids ? p[0] : 0;
    uint32_t max = UINT32_MAX;
    if (size_present) {
        max = floss_get_le16(p + (hid.uses_report_ids ? 1 : 0));
    }

    hid.ctrl_tx_len = 0;
    uint8_t *out = hid.ctrl_tx;
    uint16_t hdr = 0;
    out[hdr++] = HIDP_HDR(HIDP_DATA, type);
    if (hid.uses_report_ids) {
        out[hdr++] = id;
    }

    uint16_t rlen = 0;
    uint8_t hs = FLOSS_HID_HS_ERR_UNSUPPORTED_REQUEST;
    if (hid.cb->on_get_report) {
        hs = hid.cb->on_get_report(type, id, out + hdr, FLOSS_HID_MAX_REPORT, &rlen);
    }
    if (hs == FLOSS_HID_HS_ERR_UNSUPPORTED_REQUEST && type == FLOSS_HID_REPORT_INPUT) {
        /* No (or declining) callback: the mailbox holds the current input state. */
        hs = hid_mailbox_get(id, out + hdr, FLOSS_HID_MAX_REPORT, &rlen);
    }
    if (hs != FLOSS_HID_HS_SUCCESSFUL) {
        hid_handshake(hs);
        return;
    }
    if (rlen > FLOSS_HID_MAX_REPORT) {
        rlen = FLOSS_HID_MAX_REPORT;
    }
    uint32_t total = (uint32_t)hdr + rlen;
    if (total - 1u > max) {
        total = 1u + max;
    }
    if (total > hid.ctrl->remote_mtu) {
        total = hid.ctrl->remote_mtu;
    }
    hid_ctrl_respond((uint16_t)total);
}

/* SET_REPORT: [hdr][report ID if IDs used][report]. Answer with a HANDSHAKE. */
static void hid_on_set_report(uint8_t param, const uint8_t *p, uint16_t len) {
    uint8_t type = param & HIDP_REPORT_TYPE_MASK;
    if (type == FLOSS_HID_REPORT_OTHER) {
        hid_handshake(FLOSS_HID_HS_ERR_INVALID_PARAMETER);
        return;
    }
    uint8_t id = 0;
    if (hid.uses_report_ids) {
        if (len < 1) {
            hid_handshake(FLOSS_HID_HS_ERR_INVALID_PARAMETER);
            return;
        }
        id = p[0];
        p++;
        len--;
    }
    uint8_t hs;
    if (hid.cb->on_set_report) {
        hs = hid.cb->on_set_report(type, id, p, len);
    } else {
        hs = type == FLOSS_HID_REPORT_OUTPUT ? FLOSS_HID_HS_SUCCESSFUL : FLOSS_HID_HS_ERR_UNSUPPORTED_REQUEST;
    }
    if (hs == FLOSS_HID_HS_SUCCESSFUL && type == FLOSS_HID_REPORT_OUTPUT) {
        hid_deliver_output(id, p, len);
    }
    hid_handshake(hs);
}

static void hid_on_hid_control(uint8_t op) {
    switch (op) {
    case FLOSS_HID_CTRL_NOP:
        return;
    case FLOSS_HID_CTRL_HARD_RESET:
    case FLOSS_HID_CTRL_SOFT_RESET:
        FLOSS_LOGI(HID, "host requested %s reset", op == FLOSS_HID_CTRL_HARD_RESET ? "hard" : "soft");
        break;
    case FLOSS_HID_CTRL_SUSPEND:
        FLOSS_LOGI(HID, "host suspended");
#if FLOSS_HID_SUSPEND_SNIFF_INTERVAL
        floss_gap_sniff(hid.handle, FLOSS_HID_SUSPEND_SNIFF_INTERVAL / 2u, FLOSS_HID_SUSPEND_SNIFF_INTERVAL, 1, 0);
#endif
        break;
    case FLOSS_HID_CTRL_EXIT_SUSPEND:
        FLOSS_LOGI(HID, "host resumed");
#if FLOSS_HID_SUSPEND_SNIFF_INTERVAL
        if (hid.cfg->sniff_max_interval) {
            uint16_t max = hid.cfg->sniff_max_interval;
            uint16_t min = (uint16_t)((max / 2u) & ~1u);
            floss_gap_sniff(hid.handle, min ? min : max, max, 1, 0);
        } else {
            floss_gap_sniff_exit(hid.handle);
        }
#endif
        break;
    case FLOSS_HID_CTRL_VIRTUAL_CABLE_UNPLUG:
        /* The host removed us: forget it and go. Not acknowledged. */
        hid_log_addr("virtual cable unplugged by", &hid.host);
        floss_gap_bond_delete(&hid.host);
        if (hid.cb->on_control) {
            hid.cb->on_control(op);
        }
        floss_hid_disconnect();
        return;
    default:
        hid_handshake(FLOSS_HID_HS_ERR_UNSUPPORTED_REQUEST);
        return;
    }
    if (hid.cb->on_control) {
        hid.cb->on_control(op);
    }
}

static void hid_ctrl_on_data(floss_l2cap_chan_t *ch, const uint8_t *data, uint16_t len) {
    if (ch != hid.ctrl || len == 0) {
        return;
    }
    uint8_t type = data[0] >> 4;
    uint8_t param = data[0] & 0x0F;
    const uint8_t *p = data + 1;
    uint16_t n = (uint16_t)(len - 1);
    FLOSS_HEXDUMP(HID, "ctrl rx", data, len);

    switch (type) {
    case HIDP_HID_CONTROL:
        hid_on_hid_control(param);
        break;
    case HIDP_GET_REPORT:
        hid_on_get_report(param, p, n);
        break;
    case HIDP_SET_REPORT:
        hid_on_set_report(param, p, n);
        break;
    case HIDP_GET_PROTOCOL:
        hid.ctrl_tx_len = 0;
        hid.ctrl_tx[0] = HIDP_HDR(HIDP_DATA, FLOSS_HID_REPORT_OTHER);
        hid.ctrl_tx[1] = hid.protocol;
        hid_ctrl_respond(2);
        break;
    case HIDP_SET_PROTOCOL: {
        uint8_t proto = param & 0x01;
        if (proto == FLOSS_HID_PROTOCOL_BOOT && !hid.cfg->boot_device) {
            hid_handshake(FLOSS_HID_HS_ERR_UNSUPPORTED_REQUEST);
            break;
        }
        hid.protocol = proto;
        FLOSS_LOGI(HID, "protocol -> %s", proto == FLOSS_HID_PROTOCOL_BOOT ? "boot" : "report");
        if (hid.cb->on_set_protocol) {
            hid.cb->on_set_protocol(proto);
        }
        hid_handshake(FLOSS_HID_HS_SUCCESSFUL);
        break;
    }
    case HIDP_GET_IDLE:
        hid.ctrl_tx_len = 0;
        hid.ctrl_tx[0] = HIDP_HDR(HIDP_DATA, FLOSS_HID_REPORT_OTHER);
        hid.ctrl_tx[1] = hid.idle_rate;
        hid_ctrl_respond(2);
        break;
    case HIDP_SET_IDLE:
        if (n < 1) {
            hid_handshake(FLOSS_HID_HS_ERR_INVALID_PARAMETER);
            break;
        }
        hid.idle_rate = p[0];
        hid_handshake(FLOSS_HID_HS_SUCCESSFUL);
        break;
    default:
        /* HANDSHAKE/DATA/DATC from the host on control, reserved types. */
        FLOSS_LOGD(HID, "unsupported control transaction 0x%02X", data[0]);
        hid_handshake(FLOSS_HID_HS_ERR_UNSUPPORTED_REQUEST);
        break;
    }
}

/* Interrupt channel: the host sends DATA|OUTPUT (0xA2) reports here. */
static void hid_intr_on_data(floss_l2cap_chan_t *ch, const uint8_t *data, uint16_t len) {
    if (ch != hid.intr || len == 0) {
        return;
    }
    if (data[0] != HIDP_HDR(HIDP_DATA, FLOSS_HID_REPORT_OUTPUT)) {
        FLOSS_LOGD(HID, "ignored interrupt PDU 0x%02X", data[0]);
        return;
    }
    const uint8_t *p = data + 1;
    uint16_t n = (uint16_t)(len - 1);
    uint8_t id = 0;
    if (hid.uses_report_ids) {
        if (n < 1) {
            return;
        }
        id = p[0];
        p++;
        n--;
    }
    hid_deliver_output(id, p, n);
}

/* ------------------------------------------------------------------------ */
/* Session lifecycle                                                        */
/* ------------------------------------------------------------------------ */

/** Forget everything about the current link (it is gone or never came). */
static void hid_end_session(void) {
    floss_timer_stop(&hid.tick_timer);
    floss_timer_stop(&hid.reconnect_timer);
    floss_timer_stop(&hid.teardown_timer);
    hid.state = FLOSS_HID_STATE_IDLE;
    hid.handle = HID_NO_HANDLE;
    hid.ctrl = NULL;
    hid.intr = NULL;
    hid.ctrl_open = false;
    hid.intr_open = false;
    hid.app_connected = false;
    hid.reconnecting = false;
    hid.local_teardown = false;
    hid.link_disc_sent = false;
    hid.due_mask = 0;
    hid.force_mask = 0;
    hid.ctrl_tx_len = 0;
    hid.protocol = FLOSS_HID_PROTOCOL_REPORT;
}

/** Connectable always; discoverable only if there is nobody to go back to. */
static void hid_become_visible(void) {
    bool discoverable = hid.cfg->auto_discoverable && (floss_gap_bond_count() == 0 || hid.auth_failed);
    floss_gap_set_scan(discoverable, true);
    FLOSS_LOGI(HID, "waiting for host (%s)", discoverable ? "discoverable" : "connectable");
}

/** Both channels are open: the session is live. */
static void hid_session_up(void) {
    hid.state = FLOSS_HID_STATE_CONNECTED;
    hid.app_connected = true;
    hid.reconnecting = false;
    hid.reconnect_retries = 0;
    hid.auth_failed = false;
    hid.local_teardown = false;
    hid.protocol = FLOSS_HID_PROTOCOL_REPORT;
    floss_timer_stop(&hid.reconnect_timer);
    floss_timer_stop(&hid.teardown_timer);

    /* Link tuning for latency (see floss_hid_config_t). */
    if (hid.cfg->flush_timeout_ms) {
        floss_gap_set_flush_timeout(hid.handle, hid.cfg->flush_timeout_ms);
    }
    if (hid.cfg->sniff_max_interval) {
        uint16_t max = hid.cfg->sniff_max_interval;
        uint16_t min = (uint16_t)((max / 2u) & ~1u); /* sniff intervals are even slot counts */
        floss_gap_sniff(hid.handle, min ? min : max, max, 1, 0);
    }
    floss_gap_set_scan(false, FLOSS_HID_CONNECTABLE_WHILE_CONNECTED);

    /* Send the current state of every report once, so the host does not
     * have to wait for the next change to know where the sticks are. */
    hid.force_mask = hid_published_mask();
    for (unsigned i = 0; i < FLOSS_HID_MAX_INPUT_IDS; i++) {
        hid_slots[i].count_coalesced = false;
    }
    hid.defer_counted = false;
    hid.have_last_send = false;
    hid_start_pacing();

    hid_log_addr("connected to", &hid.host);
    if (hid.cb->on_connected) {
        hid.cb->on_connected(&hid.host);
    }
    hid_pending = 1;
    hid_input_service();
}

static floss_err_t hid_open_interrupt(void) {
    floss_l2cap_chan_t *ch = NULL;
    floss_err_t err =
        floss_l2cap_connect(hid.handle, FLOSS_PSM_HID_INTERRUPT, &hid_intr_ops, NULL, FLOSS_SEC_ENCRYPTED, &ch);
    if (err == FLOSS_OK) {
        hid.intr = ch;
    }
    return err;
}

/* Begin a local teardown: interrupt, then control, then the link. */
static void hid_start_local_teardown(void) {
    hid.state = FLOSS_HID_STATE_DISCONNECTING;
    hid.local_teardown = true;
    floss_timer_stop(&hid.tick_timer);
    floss_timer_start(&hid.teardown_timer, FLOSS_HID_TEARDOWN_TIMEOUT_US);
    hid_teardown_step();
}

static void hid_disconnect_link(void) {
    if (hid.handle != HID_NO_HANDLE && !hid.link_disc_sent) {
        hid.link_disc_sent = true;
        floss_gap_disconnect(hid.handle, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED);
    }
}

/** Advance the teardown by one step; called again from each on_close. */
static void hid_teardown_step(void) {
    floss_l2cap_chan_t **order[2] = {&hid.intr, &hid.ctrl};
    for (unsigned i = 0; i < 2; i++) {
        floss_l2cap_chan_t *ch = *order[i];
        if (!ch) {
            continue;
        }
        if (ch->state == FLOSS_L2CAP_WAIT_DISC_RSP) {
            return; /* wait for its on_close */
        }
        if (floss_l2cap_disconnect(ch) == FLOSS_OK) {
            return;
        }
        /* Could not disconnect (never opened): just forget it. */
        *order[i] = NULL;
        if (i == 0) {
            hid.intr_open = false;
        } else {
            hid.ctrl_open = false;
        }
    }
    hid_disconnect_link();
}

static void hid_teardown_timeout(floss_timer_t *t, void *arg) {
    (void)arg;
    if (!hid.local_teardown) {
        /* The host closed a channel and then went quiet: finish the job. */
        FLOSS_LOGI(HID, "host closed the session, tearing down");
        hid.local_teardown = true;
        hid.state = FLOSS_HID_STATE_DISCONNECTING;
        floss_timer_start(t, FLOSS_HID_TEARDOWN_TIMEOUT_US);
        hid_teardown_step();
        return;
    }
    FLOSS_LOGW(HID, "teardown stalled, dropping link");
    hid.link_disc_sent = false;
    hid_disconnect_link();
}

/* ---- reconnection -------------------------------------------------------- */

static void hid_reconnect_give_up(void) {
    hid.reconnecting = false;
    hid.state = FLOSS_HID_STATE_IDLE;
    hid.handle = HID_NO_HANDLE;
    hid_become_visible();
}

/** A reconnect attempt failed with HCI status/reason `status`. */
static void hid_reconnect_failed(uint8_t status) {
    if (!hid.reconnecting) {
        return;
    }
    hid.handle = HID_NO_HANDLE;
    if (hid_is_key_failure(status) || hid.auth_failed) {
        hid.auth_failed = true;
        FLOSS_LOGW(HID, "host rejected our key; pairing required");
        hid_reconnect_give_up();
        return;
    }
    if (hid.reconnect_retries < FLOSS_HID_RECONNECT_RETRIES) {
        hid.reconnect_retries++;
        FLOSS_LOGI(HID, "reconnect failed (%s), retry %u/%u", floss_hci_status_str(status), hid.reconnect_retries,
                   (unsigned)FLOSS_HID_RECONNECT_RETRIES);
        floss_timer_start(&hid.reconnect_timer, FLOSS_HID_RECONNECT_BACKOFF_US);
        return;
    }
    FLOSS_LOGI(HID, "host unreachable, giving up");
    hid_reconnect_give_up();
}

static void hid_open_control(void) {
    floss_l2cap_chan_t *ch = NULL;
    floss_err_t err =
        floss_l2cap_connect(hid.handle, FLOSS_PSM_HID_CONTROL, &hid_ctrl_ops, NULL, FLOSS_SEC_ENCRYPTED, &ch);
    if (err != FLOSS_OK) {
        FLOSS_LOGW(HID, "cannot open control channel: %s", floss_err_str(err));
        hid_disconnect_link(); /* link down -> reconnect logic */
        return;
    }
    hid.ctrl = ch;
}

/** Page the host (or reuse an existing link to it). */
static void hid_page_host(void) {
    floss_conn_t *c = floss_hci_conn_by_addr(&hid.host);
    if (c && c->state == FLOSS_CONN_OPEN && !c->le) {
        hid.handle = c->handle;
        hid_open_control();
        return;
    }
    hid.handle = HID_NO_HANDLE;
    floss_err_t err = floss_gap_connect(&hid.host);
    if (err != FLOSS_OK) {
        FLOSS_LOGW(HID, "page failed to start: %s", floss_err_str(err));
        hid_reconnect_failed(FLOSS_HCI_ERR_UNSPECIFIED);
    }
}

static void hid_reconnect_timeout(floss_timer_t *t, void *arg) {
    (void)t;
    (void)arg;
    if (hid.reconnecting && hid.state == FLOSS_HID_STATE_CONNECTING) {
        hid_page_host();
    }
}

floss_err_t floss_hid_connect(const floss_addr_t *host) {
    if (!hid.cfg) {
        return FLOSS_ERR_STATE;
    }
    if (hid.state != FLOSS_HID_STATE_IDLE) {
        return FLOSS_ERR_STATE;
    }
    if (host) {
        hid.host = *host;
    } else {
        const floss_bond_t *b = floss_gap_bond_last();
        if (!b) {
            return FLOSS_ERR_NOT_FOUND;
        }
        hid.host = b->addr;
    }
    hid.state = FLOSS_HID_STATE_CONNECTING;
    hid.reconnecting = true;
    hid.reconnect_retries = 0;
    hid.auth_failed = false;
    hid_log_addr("reconnecting to", &hid.host);
    hid_page_host();
    return FLOSS_OK;
}

floss_err_t floss_hid_disconnect(void) {
    if (!hid.cfg) {
        return FLOSS_ERR_STATE;
    }
    switch (hid.state) {
    case FLOSS_HID_STATE_IDLE:
        return FLOSS_ERR_STATE;
    case FLOSS_HID_STATE_DISCONNECTING:
        if (!hid.local_teardown) {
            hid_start_local_teardown();
        }
        return FLOSS_OK;
    case FLOSS_HID_STATE_CONNECTING:
        if (hid.handle == HID_NO_HANDLE) {
            /* Still paging (or waiting to retry): cancel. */
            floss_timer_stop(&hid.reconnect_timer);
            floss_gap_connect_cancel(&hid.host);
            hid.reconnecting = false;
            hid.state = FLOSS_HID_STATE_IDLE;
            hid_become_visible();
            return FLOSS_OK;
        }
        hid.reconnecting = false;
        hid_start_local_teardown();
        return FLOSS_OK;
    case FLOSS_HID_STATE_CONNECTED:
    default:
        hid_start_local_teardown();
        return FLOSS_OK;
    }
}

floss_err_t floss_hid_unplug(void) {
    if (!hid.cfg) {
        return FLOSS_ERR_STATE;
    }
    floss_addr_t addr;
    if (hid.state != FLOSS_HID_STATE_IDLE) {
        addr = hid.host;
        if (hid.ctrl && hid.ctrl_open) {
            /* Tell the host to drop its bond too. Not acknowledged by the host. */
            uint8_t msg = HIDP_HDR(HIDP_HID_CONTROL, FLOSS_HID_CTRL_VIRTUAL_CABLE_UNPLUG);
            floss_err_t err = floss_l2cap_send(hid.ctrl, &msg, 1);
            if (err != FLOSS_OK) {
                FLOSS_LOGW(HID, "unplug message not sent: %s", floss_err_str(err));
            }
        }
    } else {
        const floss_bond_t *b = floss_gap_bond_last();
        if (!b) {
            return FLOSS_ERR_NOT_FOUND;
        }
        addr = b->addr;
    }
    hid_log_addr("virtual cable unplug", &addr);
    floss_gap_bond_delete(&addr);
    if (hid.state != FLOSS_HID_STATE_IDLE) {
        return floss_hid_disconnect();
    }
    hid_become_visible();
    return FLOSS_OK;
}

/* ---- L2CAP channel callbacks ------------------------------------------- */

static void hid_chan_open(floss_l2cap_chan_t *ch, uint16_t status, bool is_ctrl) {
    floss_l2cap_chan_t **slot = is_ctrl ? &hid.ctrl : &hid.intr;
    const char *name = is_ctrl ? "control" : "interrupt";

    if (status != 0) {
        if (*slot == ch) {
            *slot = NULL;
        }
        FLOSS_LOGW(HID, "%s channel failed to open (0x%04X)", name, status);
        if (hid.state == FLOSS_HID_STATE_CONNECTING) {
            hid_start_local_teardown(); /* link drop feeds the reconnect retry logic */
        } else if (hid.state == FLOSS_HID_STATE_DISCONNECTING && hid.local_teardown) {
            hid_teardown_step();
        }
        return;
    }

    /* One session, one link: refuse channels from anywhere else. */
    if (hid.handle != HID_NO_HANDLE && ch->handle != hid.handle) {
        FLOSS_LOGW(HID, "%s channel from a second link refused", name);
        floss_l2cap_disconnect(ch);
        return;
    }
    if (*slot && *slot != ch) {
        FLOSS_LOGW(HID, "duplicate %s channel refused", name);
        floss_l2cap_disconnect(ch);
        return;
    }
    if (hid.state == FLOSS_HID_STATE_DISCONNECTING) {
        floss_l2cap_disconnect(ch);
        return;
    }

    *slot = ch;
    if (is_ctrl) {
        hid.ctrl_open = true;
    } else {
        hid.intr_open = true;
    }
    if (hid.state == FLOSS_HID_STATE_IDLE) {
        /* A host is connecting to us. */
        hid.state = FLOSS_HID_STATE_CONNECTING;
        hid.reconnecting = false;
    }
    if (hid.handle == HID_NO_HANDLE) {
        floss_conn_t *c = floss_hci_conn_by_handle(ch->handle);
        if (hid.reconnecting && c && !floss_addr_eq(&c->addr, &hid.host)) {
            /* Another host connected while we were paging ours: serve it. */
            floss_timer_stop(&hid.reconnect_timer);
            floss_gap_connect_cancel(&hid.host);
            hid.reconnecting = false;
        }
        hid.handle = ch->handle;
        if (c) {
            hid.host = c->addr;
        }
    }
    FLOSS_LOGD(HID, "%s channel open (cid 0x%04X, mtu %u)", name, ch->local_cid, ch->remote_mtu);

    /* We initiated: control is up, now open interrupt. */
    if (is_ctrl && ch->initiator && !hid.intr) {
        floss_err_t err = hid_open_interrupt();
        if (err != FLOSS_OK) {
            FLOSS_LOGW(HID, "cannot open interrupt channel: %s", floss_err_str(err));
            hid_start_local_teardown();
        }
        return;
    }
    if (hid.ctrl_open && hid.intr_open) {
        hid_session_up();
    }
}

static void hid_chan_close(floss_l2cap_chan_t *ch, bool is_ctrl) {
    floss_l2cap_chan_t **slot = is_ctrl ? &hid.ctrl : &hid.intr;
    if (*slot != ch) {
        return; /* not ours (refused duplicate) or session already ended */
    }
    *slot = NULL;
    if (is_ctrl) {
        hid.ctrl_open = false;
        hid.ctrl_tx_len = 0;
    } else {
        hid.intr_open = false;
    }
    FLOSS_LOGD(HID, "%s channel closed", is_ctrl ? "control" : "interrupt");

    if (hid.state == FLOSS_HID_STATE_CONNECTED || hid.state == FLOSS_HID_STATE_CONNECTING) {
        if (!hid.local_teardown) {
            /* The host is closing the session. Give it time to close the
             * other channel and the link itself, then finish for it. */
            hid.state = FLOSS_HID_STATE_DISCONNECTING;
            floss_timer_stop(&hid.tick_timer);
            floss_timer_start(&hid.teardown_timer, FLOSS_HID_REMOTE_CLOSE_GRACE_US);
            return;
        }
    }
    if (hid.state == FLOSS_HID_STATE_DISCONNECTING && hid.local_teardown) {
        hid_teardown_step();
    }
}

static void hid_ctrl_on_open(floss_l2cap_chan_t *ch, uint16_t status) {
    hid_chan_open(ch, status, true);
}
static void hid_intr_on_open(floss_l2cap_chan_t *ch, uint16_t status) {
    hid_chan_open(ch, status, false);
}
static void hid_ctrl_on_close(floss_l2cap_chan_t *ch) {
    hid_chan_close(ch, true);
}
static void hid_intr_on_close(floss_l2cap_chan_t *ch) {
    hid_chan_close(ch, false);
}
static void hid_ctrl_on_can_send(floss_l2cap_chan_t *ch) {
    (void)ch;
    hid_ctrl_flush();
}
static void hid_intr_on_can_send(floss_l2cap_chan_t *ch) {
    (void)ch;
    hid_input_service();
}

static const floss_l2cap_ops_t hid_ctrl_ops = {
    .on_open = hid_ctrl_on_open,
    .on_close = hid_ctrl_on_close,
    .on_data = hid_ctrl_on_data,
    .on_can_send = hid_ctrl_on_can_send,
};

static const floss_l2cap_ops_t hid_intr_ops = {
    .on_open = hid_intr_on_open,
    .on_close = hid_intr_on_close,
    .on_data = hid_intr_on_data,
    .on_can_send = hid_intr_on_can_send,
};

/* ---- GAP events ----------------------------------------------------------- */

static void hid_on_ready(void) {
    /* Controller (re)started: any previous link is gone. */
    hid_end_session();
    const floss_bond_t *b = floss_gap_bond_last();
    if (hid.cfg->auto_reconnect && b) {
        floss_gap_set_scan(false, true);
        if (floss_hid_connect(&b->addr) == FLOSS_OK) {
            return;
        }
    }
    hid_become_visible();
}

static void hid_on_link_up(const floss_gap_evt_t *evt) {
    floss_conn_t *c = floss_hci_conn_by_handle(evt->handle);
    if (c && c->le) {
        return;
    }
    if (hid.state == FLOSS_HID_STATE_CONNECTING && hid.handle == HID_NO_HANDLE &&
        floss_addr_eq(&evt->addr, &hid.host)) {
        floss_timer_stop(&hid.reconnect_timer);
        hid.handle = evt->handle;
        if (hid.reconnecting && c && c->initiator) {
            hid_open_control(); /* L2CAP secures the link first (FLOSS_SEC_ENCRYPTED) */
        } else {
            /* The host paged us at the same time: it opens the channels. */
            hid.reconnecting = false;
        }
        return;
    }
    if (hid.state == FLOSS_HID_STATE_IDLE && c && c->initiator && floss_addr_eq(&evt->addr, &hid.host)) {
        /* A page we cancelled completed anyway: nobody wants this link. */
        floss_gap_disconnect(evt->handle, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED);
    }
}

static void hid_on_connect_failed(const floss_gap_evt_t *evt) {
    if (hid.reconnecting && hid.state == FLOSS_HID_STATE_CONNECTING && floss_addr_eq(&evt->addr, &hid.host)) {
        hid_reconnect_failed(evt->status);
    }
}

static void hid_on_link_down(const floss_gap_evt_t *evt) {
    if (hid.handle == HID_NO_HANDLE || evt->handle != hid.handle) {
        return;
    }
    bool was_connected = hid.app_connected;
    bool was_reconnecting = hid.reconnecting && !was_connected;
    uint8_t reason = evt->status;
    floss_addr_t host = hid.host;

    hid_end_session();
    if (hid_is_key_failure(reason)) {
        hid.auth_failed = true;
    }
    FLOSS_LOGI(HID, "link down (%s)", floss_hci_status_str(reason));

    if (was_connected && hid.cb->on_disconnected) {
        hid.cb->on_disconnected(reason);
    }
    if (was_reconnecting) {
        /* The link came up but the session did not (security, channel refused). */
        hid.reconnecting = true;
        hid.state = FLOSS_HID_STATE_CONNECTING;
        hid_reconnect_failed(reason);
        return;
    }
    if (was_connected && reason == FLOSS_HCI_ERR_CONN_TIMEOUT && hid.cfg->auto_reconnect &&
        floss_gap_bond_find(&host)) {
        /* Supervision timeout = we drifted out of range: try to get back. */
        if (floss_hid_connect(&host) == FLOSS_OK) {
            floss_gap_set_scan(false, true);
            return;
        }
    }
    hid_become_visible();
}

static void hid_gap_event(const floss_gap_evt_t *evt, void *user) {
    (void)user;
    switch (evt->type) {
    case FLOSS_GAP_EVT_READY:
        hid_on_ready();
        break;
    case FLOSS_GAP_EVT_CONNECTED:
        hid_on_link_up(evt);
        break;
    case FLOSS_GAP_EVT_CONNECT_FAILED:
        hid_on_connect_failed(evt);
        break;
    case FLOSS_GAP_EVT_DISCONNECTED:
        hid_on_link_down(evt);
        break;
    case FLOSS_GAP_EVT_AUTHENTICATED:
    case FLOSS_GAP_EVT_PAIRING_FAILED:
        if (evt->handle == hid.handle && hid.handle != HID_NO_HANDLE && hid_is_key_failure(evt->status)) {
            hid.auth_failed = true;
        }
        break;
    default:
        break;
    }
}

/* ------------------------------------------------------------------------ */
/* Init / public accessors                                                  */
/* ------------------------------------------------------------------------ */

void floss_hid_config_defaults(floss_hid_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->subclass = FLOSS_HID_SUBCLASS_GAMEPAD;
    cfg->virtual_cable = true;
    cfg->reconnect_initiate = true;
    cfg->normally_connectable = false;
    cfg->battery_power = true;
    cfg->report_interval_us = 0;
    cfg->flush_timeout_ms = 0;
    cfg->auto_discoverable = true;
    cfg->auto_reconnect = true;
    cfg->sniff_max_interval = 0;
}

floss_err_t floss_hid_init(const floss_hid_config_t *cfg, const floss_hid_callbacks_t *cb) {
    if (!cfg || !cfg->report_descriptor || cfg->report_descriptor_len == 0) {
        return FLOSS_ERR_INVALID;
    }
    if (cfg->report_descriptor_len > FLOSS_HID_MAX_DESCRIPTOR) {
        FLOSS_LOGE(HID, "report descriptor too large (%u > FLOSS_HID_MAX_DESCRIPTOR %u)", cfg->report_descriptor_len,
                   (unsigned)FLOSS_HID_MAX_DESCRIPTOR);
        return FLOSS_ERR_INVALID;
    }

    memset(&hid, 0, sizeof(hid));
    memset(hid_slots, 0, sizeof(hid_slots));
    hid_pending = 0;
    hid.cfg = cfg;
    hid.cb = cb ? cb : &hid_no_callbacks;
    hid.handle = HID_NO_HANDLE;
    hid.protocol = FLOSS_HID_PROTOCOL_REPORT;
    hid.interval_us = cfg->report_interval_us;
    hid.uses_report_ids = hid_descriptor_uses_ids(cfg->report_descriptor, cfg->report_descriptor_len);
    floss_timer_init(&hid.tick_timer, hid_tick, NULL);
    floss_timer_init(&hid.reconnect_timer, hid_reconnect_timeout, NULL);
    floss_timer_init(&hid.teardown_timer, hid_teardown_timeout, NULL);

    /* SDP: the HID record (with the report descriptor) and Device ID. */
    const char *name = cfg->name ? cfg->name : "FLOSS HID";
    floss_sdp_hid_params_t sp = {
        .service_name = name,
        .service_description = NULL,
        .provider_name = "FLOSS",
        .subclass = cfg->subclass,
        .country_code = 0,
        .virtual_cable = cfg->virtual_cable,
        .reconnect_initiate = cfg->reconnect_initiate,
        .battery_power = cfg->battery_power,
        .remote_wake = cfg->remote_wake,
        .normally_connectable = cfg->normally_connectable,
        .boot_device = cfg->boot_device,
        .report_descriptor = cfg->report_descriptor,
        .report_descriptor_len = cfg->report_descriptor_len,
    };
    uint16_t len = floss_sdp_build_hid(hid_sdp_buf, sizeof(hid_sdp_buf), &sp);
    if (len == 0) {
        FLOSS_LOGE(HID, "HID SDP record does not fit");
        return FLOSS_ERR_NOMEM;
    }
    hid_sdp_rec.data = hid_sdp_buf;
    hid_sdp_rec.len = len;
    floss_err_t err = floss_sdp_register(&hid_sdp_rec);
    if (err != FLOSS_OK) {
        return err;
    }

    if (cfg->vendor_id_source != 0) {
        floss_sdp_did_params_t dp = {
            .vendor_id_source = cfg->vendor_id_source,
            .vendor_id = cfg->vendor_id,
            .product_id = cfg->product_id,
            .version = cfg->version,
            .primary = true,
        };
        len = floss_sdp_build_did(did_sdp_buf, sizeof(did_sdp_buf), &dp);
        if (len == 0) {
            return FLOSS_ERR_NOMEM;
        }
        did_sdp_rec.data = did_sdp_buf;
        did_sdp_rec.len = len;
        err = floss_sdp_register(&did_sdp_rec);
        if (err != FLOSS_OK) {
            return err;
        }
    }

    floss_gap_set_name(name);
    floss_gap_set_class(cfg->class_of_device ? cfg->class_of_device : hid_cod_for_subclass(cfg->subclass));

    err = floss_l2cap_register_psm(FLOSS_PSM_HID_CONTROL, &hid_ctrl_ops, NULL, FLOSS_SEC_ENCRYPTED);
    if (err == FLOSS_OK) {
        err = floss_l2cap_register_psm(FLOSS_PSM_HID_INTERRUPT, &hid_intr_ops, NULL, FLOSS_SEC_ENCRYPTED);
    }
    if (err == FLOSS_OK) {
        err = floss_gap_add_listener(hid_gap_event, NULL);
    }
    if (err != FLOSS_OK) {
        return err;
    }
    FLOSS_LOGI(HID, "\"%s\" ready: %s, report IDs %s", name,
               hid.interval_us ? "paced" : "send on change", hid.uses_report_ids ? "yes" : "no");
    return FLOSS_OK;
}

void floss_hid_poll(void) {
    if (hid.ctrl_tx_len) {
        hid_ctrl_flush();
    }
    hid_input_service();
}

floss_hid_state_t floss_hid_state(void) {
    return hid.state;
}

uint8_t floss_hid_protocol(void) {
    return hid.protocol;
}

uint16_t floss_hid_handle(void) {
    return hid.cfg ? hid.handle : HID_NO_HANDLE;
}
