/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_hogp.c - HID over GATT device: HID service and the input mailbox.
 *
 * Overview
 * --------
 * floss_hogp_init() builds the HID service table in RAM (its shape depends
 * on how many reports the application declares), registers it together with
 * Device Information and Battery, and remembers the value handle of every
 * Report characteristic.
 *
 * Input path (the hot path):
 *
 *   app/ISR/core1                 stack context (floss_poll)
 *   floss_hogp_set_input()  --->  slot.seq changed?  --->  link can take a
 *   seqlock write, no locks       (poll / credits /        packet? notify:
 *                                 pacing tick)             alloc buf, copy
 *                                                          slot straight into
 *                                                          it, push headers
 *
 * Each input report has one slot guarded by a sequence counter: the writer
 * makes it odd, copies, makes it even again; the reader copies and retries
 * if the counter moved. Plain loads/stores plus fences only - Cortex-M0+
 * has no atomic read-modify-write, and none is needed with one writer.
 *
 * Latest wins: a report is sent only while fewer than `max_inflight`
 * packets of that link wait in the controller. Otherwise it stays in the
 * slot and gets overwritten by newer data; when Number Of Completed Packets
 * frees a buffer (floss_le_on_credits) the then-newest data goes out. So the
 * age of a delivered report is bounded by one connection interval plus the
 * poll gap, never by a queue.
 *
 * "Connected" for the application means: link encrypted (every HID
 * characteristic requires it) AND the host enabled notifications on at
 * least one input report. That is the moment hosts start listening.
 *
 * Boot protocol: the boot characteristics (keyboard input/output, mouse
 * input) are ordinary entries of s_hogp.reports flagged `boot`, appended
 * after the application's reports, so they share the mailbox, CCCD and
 * send machinery. report_active() decides which set the current Protocol
 * Mode notifies; find_report() (the application's report IDs) never sees
 * them. The table is still built once at init and never changes.
 */
#include <stdatomic.h>

#include "floss_internal.h"
#include "floss/floss_le.h"
#include "floss/floss_gatt.h"
#include "floss/floss_hogp.h"

#if FLOSS_ENABLE_LE

/** Boot characteristics: keyboard input, keyboard output, mouse input. */
#define HOGP_BOOT_REPORTS 3
#define HOGP_REPORT_SLOTS (FLOSS_HOGP_MAX_REPORTS + HOGP_BOOT_REPORTS)

/* service + protocol mode (2) + 4 per report + report map, info, control point (6)
 * + boot keyboard input (3) + boot keyboard output (2) + boot mouse input (3) */
#define HOGP_MAX_ATTRS (1 + 2 + 4 * FLOSS_HOGP_MAX_REPORTS + 6 + 8)

#if FLOSS_HID_MAX_REPORT < FLOSS_HOGP_BOOT_MOUSE_MAX_LEN
#error "FLOSS_HID_MAX_REPORT must hold a boot report (8 bytes)"
#endif

typedef struct {
    uint8_t id;                  /* report ID; boot reports: FLOSS_HID_BOOT_KEYBOARD / _MOUSE */
    uint8_t type;
    uint8_t len;                 /* configured payload length (boot mouse: maximum) */
    uint8_t index;               /* position in s_hogp.reports (bit in due masks) */
    bool boot;                   /* a boot protocol characteristic, not a Report */
    uint16_t value_handle;
    uint8_t ref[2];              /* Report Reference value: {id, type} */
    floss_gatt_cccd_t cccd;      /* inputs only */

    /* Mailbox (inputs) or stored value (outputs/features). For inputs the
     * fields below `seq` are only touched inside the seqlock protocol. */
    volatile uint32_t seq;       /* odd while the writer is copying */
    volatile uint32_t t_us;      /* floss_hogp_set_input() timestamp */
    volatile uint8_t cur_len;
    uint8_t data[FLOSS_HID_MAX_REPORT];

    uint32_t sent_seq[FLOSS_MAX_CONNECTIONS]; /* stack side: last seq notified per link */
} hogp_report_t;

typedef struct {
    bool connected;       /* on_connected delivered */
    uint16_t due;         /* paced mode: reports owed by the last tick (bit = report index) */
    uint16_t force;       /* send current state even if unchanged (just subscribed) */
} hogp_link_t;

static struct {
    const floss_hogp_config_t *cfg;
    floss_hogp_callbacks_t cb;
    hogp_report_t reports[HOGP_REPORT_SLOTS];
    uint8_t report_count;            /* application reports + boot characteristics */
    hogp_report_t *boot_in[2];       /* boot keyboard / mouse input (NULL = not offered) */
    hogp_link_t links[FLOSS_MAX_CONNECTIONS];

    floss_gatt_attr_t attrs[HOGP_MAX_ATTRS];
    uint8_t hid_info[4];
    uint8_t protocol;
    bool suspended;

    uint32_t interval_us;
    floss_timer_t tick;
    uint32_t next_tick_us;
    uint32_t last_send_us;
    bool have_last_send;
} s_hogp;

_Static_assert(HOGP_REPORT_SLOTS <= 16, "due/force masks are 16 bits");

/* ------------------------------------------------------------------------ */
/* Mailbox                                                                  */
/* ------------------------------------------------------------------------ */

/** An application report (Report characteristic) by type and ID. */
static hogp_report_t *find_report(uint8_t type, uint8_t id) {
    for (uint8_t i = 0; i < s_hogp.report_count; i++) {
        hogp_report_t *r = &s_hogp.reports[i];
        if (!r->boot && r->type == type && r->id == id) {
            return r;
        }
    }
    return NULL;
}

/** Is this report's characteristic the one the current Protocol Mode notifies? */
static inline bool report_active(const hogp_report_t *r) {
    return r->boot == (s_hogp.protocol == FLOSS_HID_PROTOCOL_BOOT);
}

static void mailbox_write(hogp_report_t *r, const uint8_t *data, uint16_t len);

floss_err_t floss_hogp_set_input(uint8_t report_id, const uint8_t *data, uint16_t len) {
    hogp_report_t *r = find_report(FLOSS_HID_REPORT_INPUT, report_id);
    if (r == NULL) {
        return FLOSS_ERR_NOT_FOUND;
    }
    if (len > r->len || (len && data == NULL)) {
        return FLOSS_ERR_INVALID;
    }
    mailbox_write(r, data, len);
    return FLOSS_OK;
}

floss_err_t floss_hogp_set_boot_input(uint8_t boot_id, const uint8_t *data, uint16_t len) {
    if (boot_id != FLOSS_HID_BOOT_KEYBOARD && boot_id != FLOSS_HID_BOOT_MOUSE) {
        return FLOSS_ERR_INVALID;
    }
    hogp_report_t *r = s_hogp.boot_in[boot_id - 1u];
    if (r == NULL) {
        return FLOSS_ERR_UNSUPPORTED;
    }
    uint16_t min = boot_id == FLOSS_HID_BOOT_KEYBOARD ? FLOSS_HID_BOOT_KEYBOARD_LEN : FLOSS_HID_BOOT_MOUSE_MIN_LEN;
    if (data == NULL || len < min || len > r->len) {
        return FLOSS_ERR_INVALID;
    }
    mailbox_write(r, data, len);
    return FLOSS_OK;
}

/** Seqlock write (one writer per report). */
static void mailbox_write(hogp_report_t *r, const uint8_t *data, uint16_t len) {
    uint32_t s = r->seq;
    r->seq = s + 1; /* odd: readers will retry */
    /* Full fence: the odd counter must be visible before any data byte. */
    atomic_thread_fence(memory_order_seq_cst);
    if (len) {
        memcpy(r->data, data, len);
    }
    r->cur_len = (uint8_t)len;
    r->t_us = floss_hal_time_us();
    atomic_thread_fence(memory_order_release);
    r->seq = s + 2;
}

/**
 * Seqlock read into out. Returns false if the writer kept interfering
 * (another core mid-write); the caller simply tries again later.
 */
static bool mailbox_read(hogp_report_t *r, uint8_t *out, uint16_t cap, uint16_t *len, uint32_t *seq,
                         uint32_t *t_us) {
    for (uint8_t tries = 0; tries < 4; tries++) {
        uint32_t s1 = r->seq;
        if (s1 & 1u) {
            continue;
        }
        atomic_thread_fence(memory_order_acquire);
        uint16_t n = FLOSS_MIN((uint16_t)r->cur_len, cap);
        uint32_t t = r->t_us;
        memcpy(out, r->data, n);
        atomic_thread_fence(memory_order_acquire);
        if (r->seq == s1) {
            *len = n;
            *seq = s1;
            *t_us = t;
            return true;
        }
    }
    return false;
}

/* ------------------------------------------------------------------------ */
/* Sending                                                                  */
/* ------------------------------------------------------------------------ */

/** Encrypted link with at least one input subscription? */
static bool link_subscribed(uint8_t i, uint16_t handle) {
    const floss_le_link_t *l = floss_le_link(handle);
    if (l == NULL || l->security_level < 2) {
        return false;
    }
    for (uint8_t k = 0; k < s_hogp.report_count; k++) {
        const hogp_report_t *r = &s_hogp.reports[k];
        if (r->type == FLOSS_HID_REPORT_INPUT && (r->cccd.v[i] & FLOSS_GATT_CCCD_NOTIFY)) {
            return true;
        }
    }
    return false;
}

/** Room for a report now, without queueing behind an older one? */
static bool can_send(uint16_t handle) {
    if (!floss_hci_acl_can_send_now(handle)) {
        return false;
    }
    const floss_conn_t *c = floss_hci_conn_by_handle(handle);
    uint8_t max = s_hogp.cfg && s_hogp.cfg->max_inflight ? s_hogp.cfg->max_inflight : 1;
    /* Controllers that report completions in batches (floss_chipset_t
     * .completion_batch) would otherwise leave the link idle: the CYW43439
     * holds back a lone completion for ~250 ms. */
    uint16_t cap = (uint16_t)(max * floss_hci_completion_batch());
    return c == NULL || c->inflight < cap;
}

static floss_err_t send_report(uint8_t i, uint16_t handle, hogp_report_t *r) {
    uint8_t *value;
    uint16_t cap;
    floss_buf_t *b = floss_gatt_notify_alloc(handle, r->value_handle, &value, &cap);
    if (b == NULL) {
        return FLOSS_ERR_BUSY;
    }
    uint16_t len;
    uint32_t seq, t_set;
    if (!mailbox_read(r, value, cap, &len, &seq, &t_set)) {
        floss_buf_free(b);
        return FLOSS_ERR_BUSY;
    }
    FLOSS_DBG_PIN(FLOSS_DBG_PIN_HID_TX, 1);
    floss_err_t err = floss_gatt_notify_send(b, len, FLOSS_BUF_F_PRIORITY);
    FLOSS_DBG_PIN(FLOSS_DBG_PIN_HID_TX, 0);
    if (err != FLOSS_OK) {
        return err;
    }

    uint32_t now = floss_hal_time_us();
    uint32_t prev = r->sent_seq[i];
    r->sent_seq[i] = seq;
    s_hogp.links[i].force &= (uint16_t)~(1u << r->index);
    FLOSS_STAT_INC(hid_reports_sent);
    if (seq != prev && seq - prev > 2u && prev != 0) {
        /* Every +2 is one floss_hogp_set_input(); all but the newest were superseded. */
        FLOSS_STAT_ADD(hid_reports_coalesced, (seq - prev) / 2u - 1u);
    }
    /* Age = app publish -> handed to HCI (0 for the never-set initial state). */
    uint32_t age = seq != 0 ? now - t_set : 0;
    if (seq != 0) {
        FLOSS_STAT_HIST(hid_report_age, age);
    }
    FLOSS_TRACE(FLOSS_TR_HID_SEND, age);
    (void)age;
    if (s_hogp.have_last_send) {
        FLOSS_STAT_HIST(hid_send_interval, now - s_hogp.last_send_us);
    }
    s_hogp.last_send_us = now;
    s_hogp.have_last_send = true;
    return FLOSS_OK;
}

/**
 * Bitmask of subscribed input reports of the current protocol whose slot
 * changed (or were forced) since sent on link i.
 */
static uint16_t changed_mask(uint8_t i) {
    uint16_t m = 0;
    for (uint8_t k = 0; k < s_hogp.report_count; k++) {
        hogp_report_t *r = &s_hogp.reports[k];
        if (r->type != FLOSS_HID_REPORT_INPUT || !(r->cccd.v[i] & FLOSS_GATT_CCCD_NOTIFY) || !report_active(r)) {
            continue;
        }
        if (r->seq != r->sent_seq[i] || (s_hogp.links[i].force & (1u << k))) {
            m |= (uint16_t)(1u << k);
        }
    }
    return m;
}

/** Bitmask of subscribed input reports of the current protocol on link i. */
static uint16_t subscribed_mask(uint8_t i) {
    uint16_t m = 0;
    for (uint8_t k = 0; k < s_hogp.report_count; k++) {
        const hogp_report_t *r = &s_hogp.reports[k];
        if (r->type == FLOSS_HID_REPORT_INPUT && (r->cccd.v[i] & FLOSS_GATT_CCCD_NOTIFY) && report_active(r)) {
            m |= (uint16_t)(1u << k);
        }
    }
    return m;
}

/**
 * Send what link i is owed. tick: a pacing period elapsed. In paced mode
 * reports are owed only by ticks (`due`); in send-on-change mode whatever
 * changed is owed right away.
 */
static void service_link(uint8_t i, bool tick) {
    hogp_link_t *hl = &s_hogp.links[i];
    uint16_t handle = floss_le_link_handle_at(i);
    if (!hl->connected) {
        hl->due = 0;
        return;
    }
    uint16_t due;
    if (s_hogp.interval_us) {
        if (tick) {
            hl->due |= (uint16_t)(changed_mask(i) | (s_hogp.cfg->send_unchanged ? subscribed_mask(i) : 0));
        }
        due = hl->due;
    } else {
        due = changed_mask(i);
    }
    for (uint8_t k = 0; due && k < s_hogp.report_count; k++) {
        if (!(due & (1u << k))) {
            continue;
        }
        if (!can_send(handle)) {
            if (tick) {
                FLOSS_STAT_INC(hid_send_deferred);
                FLOSS_TRACE(FLOSS_TR_HID_DEFER, 0);
            }
            return; /* the next credit brings us back here */
        }
        if (send_report(i, handle, &s_hogp.reports[k]) != FLOSS_OK) {
            return;
        }
        due &= (uint16_t)~(1u << k);
        hl->due &= (uint16_t)~(1u << k);
    }
}

static void service_all(bool tick) {
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        service_link(i, tick);
    }
}

/** Current pacing period: fixed, or the first connected link's interval. */
static uint32_t period_us(void) {
    if (s_hogp.interval_us != FLOSS_HOGP_INTERVAL_CONN) {
        return s_hogp.interval_us;
    }
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        uint16_t h = floss_le_link_handle_at(i);
        const floss_le_link_t *l = h != 0xFFFF ? floss_le_link(h) : NULL;
        if (l && l->interval) {
            return (uint32_t)l->interval * 1250u;
        }
    }
    return 7500u;
}

/** Drift-free pacing: deadlines advance by exactly one period. */
static void tick_fired(floss_timer_t *t, void *arg) {
    if (s_hogp.cb.on_report_tick) {
        s_hogp.cb.on_report_tick(); /* last chance to sample inputs for this frame */
    }
    service_all(true);
    if (s_hogp.interval_us == 0) {
        return;
    }
    uint32_t period = period_us();
    s_hogp.next_tick_us += period;
    uint32_t now = floss_hal_time_us();
    if (floss_time_diff(now, s_hogp.next_tick_us) > (int32_t)period) {
        s_hogp.next_tick_us = now + period; /* fell far behind: resync, don't burst */
    }
    floss_timer_start_at(&s_hogp.tick, s_hogp.next_tick_us);
}

static bool any_connected(void) {
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        if (s_hogp.links[i].connected) {
            return true;
        }
    }
    return false;
}

/** Run the pacing timer only while some host listens. */
static void update_tick(void) {
    bool run = s_hogp.interval_us > 0 && any_connected();
    if (run && !floss_timer_active(&s_hogp.tick)) {
        s_hogp.next_tick_us = floss_hal_time_us() + period_us();
        floss_timer_start_at(&s_hogp.tick, s_hogp.next_tick_us);
    } else if (!run) {
        floss_timer_stop(&s_hogp.tick);
    }
}

/** Track the app-level "connected" state of every link. */
static void update_connections(void) {
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        hogp_link_t *hl = &s_hogp.links[i];
        uint16_t handle = floss_le_link_handle_at(i);
        bool sub = handle != 0xFFFF && link_subscribed(i, handle);
        if (sub && !hl->connected) {
            hl->connected = true;
            const floss_le_link_t *l = floss_le_link(handle);
            FLOSS_LOGI(LE, "hogp: host 0x%04x subscribed, reports flowing", (unsigned)handle);
            if (s_hogp.cb.on_connected) {
                s_hogp.cb.on_connected(handle, &l->peer);
            }
            update_tick();
        } else if (!sub && hl->connected && handle != 0xFFFF) {
            hl->connected = false; /* unsubscribed, link still up */
            update_tick();
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Attribute callbacks                                                      */
/* ------------------------------------------------------------------------ */

static int copy_out(const uint8_t *src, uint16_t len, uint16_t offset, uint8_t *out, uint16_t cap) {
    if (offset > len) {
        return -FLOSS_ATT_ERR_INVALID_OFFSET;
    }
    uint16_t n = FLOSS_MIN((uint16_t)(len - offset), cap);
    if (n) {
        memcpy(out, src + offset, n);
    }
    return n;
}

static int report_read(uint16_t conn, uint16_t h, const floss_gatt_attr_t *a, uint16_t offset, uint8_t *out,
                       uint16_t cap) {
    hogp_report_t *r = (hogp_report_t *)a->user;
    uint8_t tmp[FLOSS_HID_MAX_REPORT];
    uint16_t len = 0;
    if (s_hogp.cb.on_get_report && r->type != FLOSS_HID_REPORT_OUTPUT && !r->boot) {
        uint8_t err = s_hogp.cb.on_get_report(r->type, r->id, tmp, r->len, &len);
        if (err) {
            return -(int)err;
        }
    } else if (r->type == FLOSS_HID_REPORT_INPUT) {
        uint32_t seq, t;
        if (!mailbox_read(r, tmp, sizeof(tmp), &len, &seq, &t)) {
            return -FLOSS_ATT_ERR_UNLIKELY;
        }
    } else {
        len = r->cur_len;
        memcpy(tmp, r->data, len);
    }
    return copy_out(tmp, FLOSS_MIN(len, (uint16_t)r->len), offset, out, cap);
}

static uint8_t report_write(uint16_t conn, uint16_t h, const floss_gatt_attr_t *a, uint16_t offset,
                            const uint8_t *data, uint16_t len, uint8_t flags) {
    hogp_report_t *r = (hogp_report_t *)a->user;
    if (offset != 0) {
        return FLOSS_ATT_ERR_INVALID_OFFSET;
    }
    if (len > r->len) {
        return FLOSS_ATT_ERR_INVALID_VALUE_LEN;
    }
    if (r->type == FLOSS_HID_REPORT_FEATURE && s_hogp.cb.on_set_report) {
        uint8_t err = s_hogp.cb.on_set_report(r->type, r->id, data, len);
        if (err) {
            return err;
        }
    }
    memcpy(r->data, data, len);
    r->cur_len = (uint8_t)len;
    if (r->type == FLOSS_HID_REPORT_OUTPUT) {
        FLOSS_STAT_INC(hid_output_reports);
        FLOSS_TRACE(FLOSS_TR_HID_OUTPUT, r->id);
        if (s_hogp.cb.on_output_report) {
            s_hogp.cb.on_output_report(r->id, data, len);
        }
    }
    return 0;
}

/** CCCD of an input report: a fresh subscription gets the current state at once. */
static uint8_t report_cccd_write(uint16_t conn, uint16_t h, const floss_gatt_attr_t *a, uint16_t offset,
                                 const uint8_t *data, uint16_t len, uint8_t flags) {
    hogp_report_t *r = (hogp_report_t *)a->user;
    int i = floss_le_link_index(conn);
    if (i >= 0 && (floss_get_le16(data) & FLOSS_GATT_CCCD_NOTIFY)) {
        s_hogp.links[i].force |= (uint16_t)(1u << r->index);
    }
    return 0;
}

static int protocol_read(uint16_t conn, uint16_t h, const floss_gatt_attr_t *a, uint16_t offset, uint8_t *out,
                         uint16_t cap) {
    return copy_out(&s_hogp.protocol, 1, offset, out, cap);
}

static uint8_t protocol_write(uint16_t conn, uint16_t h, const floss_gatt_attr_t *a, uint16_t offset,
                              const uint8_t *data, uint16_t len, uint8_t flags) {
    if (offset != 0 || len != 1) {
        return FLOSS_ATT_ERR_INVALID_VALUE_LEN;
    }
    if (data[0] > FLOSS_HID_PROTOCOL_REPORT) {
        return FLOSS_ATT_ERR_VALUE_NOT_ALLOWED;
    }
    if (s_hogp.protocol != data[0]) {
        s_hogp.protocol = data[0];
        FLOSS_LOGI(LE, "hogp: protocol %s", data[0] ? "report" : "boot");
        /* The other characteristic set is live now: start it from the
         * newest state instead of waiting for the next change. */
        for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
            s_hogp.links[i].due = 0;
            s_hogp.links[i].force = subscribed_mask(i);
        }
        if (s_hogp.cb.on_set_protocol) {
            s_hogp.cb.on_set_protocol(data[0]);
        }
    }
    return 0;
}

static uint8_t control_write(uint16_t conn, uint16_t h, const floss_gatt_attr_t *a, uint16_t offset,
                             const uint8_t *data, uint16_t len, uint8_t flags) {
    if (offset != 0 || len != 1) {
        return FLOSS_ATT_ERR_INVALID_VALUE_LEN;
    }
    if (data[0] > FLOSS_HOGP_CTRL_EXIT_SUSPEND) {
        return FLOSS_ATT_ERR_VALUE_NOT_ALLOWED;
    }
    s_hogp.suspended = data[0] == FLOSS_HOGP_CTRL_SUSPEND;
    FLOSS_LOGI(LE, "hogp: host %s", s_hogp.suspended ? "suspended" : "resumed");
    if (s_hogp.cb.on_control) {
        s_hogp.cb.on_control(data[0]);
    }
    return 0;
}

/* ------------------------------------------------------------------------ */
/* Public API                                                               */
/* ------------------------------------------------------------------------ */

void floss_hogp_config_defaults(floss_hogp_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->normally_connectable = true;
    cfg->battery_service = true;
    cfg->battery_level = 100;
    cfg->max_inflight = 2; /* an LE notification is acknowledged one connection event later */
}

/**
 * Append one boot characteristic (declaration, value, and a CCCD for inputs;
 * boot characteristics carry no Report Reference) and its report slot.
 * len = maximum value length, cur = length reads return before the first
 * update (zeros). Returns the slot.
 */
static hogp_report_t *add_boot_report(floss_gatt_attr_t *t, uint16_t *n, uint16_t *value_at, uint16_t uuid,
                                      uint8_t boot_id, uint8_t type, uint8_t len, uint8_t cur) {
    uint8_t k = s_hogp.report_count++;
    hogp_report_t *r = &s_hogp.reports[k];
    memset(r, 0, sizeof(*r));
    r->id = boot_id;
    r->type = type;
    r->len = len;
    r->cur_len = cur;
    r->index = k;
    r->boot = true;
    if (type == FLOSS_HID_REPORT_INPUT) {
        t[(*n)++] = (floss_gatt_attr_t)FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_READ | FLOSS_GATT_PROP_NOTIFY);
        value_at[k] = *n;
        t[(*n)++] = (floss_gatt_attr_t)FLOSS_GATT_VALUE16_CB(uuid, FLOSS_GATT_P_RD_ENC, report_read, NULL, len, r);
        t[*n] = (floss_gatt_attr_t)FLOSS_GATT_CCCD_P(r->cccd, FLOSS_GATT_P_READ | FLOSS_GATT_P_WR_ENC);
        t[*n].write = report_cccd_write;
        t[*n].user = r;
        (*n)++;
    } else {
        t[(*n)++] = (floss_gatt_attr_t)FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_READ | FLOSS_GATT_PROP_WRITE |
                                                                 FLOSS_GATT_PROP_WRITE_NO_RSP);
        value_at[k] = *n;
        t[(*n)++] =
            (floss_gatt_attr_t)FLOSS_GATT_VALUE16_CB(uuid, FLOSS_GATT_P_RW_ENC, report_read, report_write, len, r);
    }
    return r;
}

floss_err_t floss_hogp_init(const floss_hogp_config_t *cfg, const floss_hogp_callbacks_t *cb) {
    if (cfg == NULL || cfg->report_map == NULL || cfg->report_map_len == 0 ||
        cfg->report_map_len > FLOSS_ATT_MAX_VALUE || cfg->report_count == 0 ||
        cfg->report_count > FLOSS_HOGP_MAX_REPORTS || cfg->reports == NULL) {
        return FLOSS_ERR_INVALID;
    }
    if (s_hogp.cfg) {
        return FLOSS_ERR_STATE; /* the database cannot grow a second HID service here */
    }
    uint8_t inputs = 0;
    for (uint8_t k = 0; k < cfg->report_count; k++) {
        const floss_hogp_report_t *rc = &cfg->reports[k];
        if (rc->len == 0 || rc->len > FLOSS_HID_MAX_REPORT || rc->type < FLOSS_HID_REPORT_INPUT ||
            rc->type > FLOSS_HID_REPORT_FEATURE) {
            return FLOSS_ERR_INVALID;
        }
        inputs = (uint8_t)(inputs + (rc->type == FLOSS_HID_REPORT_INPUT));
    }
    if (inputs > FLOSS_HID_MAX_INPUT_IDS) {
        return FLOSS_ERR_INVALID;
    }

    /* Companion services first, so the HID service is last in discovery. */
    if (cfg->manufacturer || cfg->model || cfg->vendor_id_source) {
        floss_gatt_dis_t dis = {
            .manufacturer = cfg->manufacturer,
            .model = cfg->model,
            .vendor_id_source = cfg->vendor_id_source,
            .vendor_id = cfg->vendor_id,
            .product_id = cfg->product_id,
            .version = cfg->version,
        };
        floss_err_t err = floss_gatt_dis_register(&dis);
        if (err != FLOSS_OK && err != FLOSS_ERR_STATE) {
            return err;
        }
    }
    if (cfg->battery_service) {
        floss_err_t err = floss_gatt_battery_register(cfg->battery_level);
        if (err != FLOSS_OK && err != FLOSS_ERR_STATE) {
            return err;
        }
    }

    s_hogp.cfg = cfg;
    s_hogp.cb = cb ? *cb : (floss_hogp_callbacks_t){0};
    s_hogp.protocol = FLOSS_HID_PROTOCOL_REPORT;
    s_hogp.interval_us = cfg->report_interval_us;
    s_hogp.hid_info[0] = 0x11; /* bcdHID 1.11 */
    s_hogp.hid_info[1] = 0x01;
    s_hogp.hid_info[2] = cfg->country_code;
    s_hogp.hid_info[3] = (uint8_t)((cfg->remote_wake ? FLOSS_HOGP_FLAG_REMOTE_WAKE : 0) |
                                   (cfg->normally_connectable ? FLOSS_HOGP_FLAG_NORMALLY_CONNECTABLE : 0));

    /* Build the HID service table. value_at[k] = index of report k's value. */
    floss_gatt_attr_t *t = s_hogp.attrs;
    uint16_t n = 0;
    uint16_t value_at[HOGP_REPORT_SLOTS];
    t[n++] = (floss_gatt_attr_t)FLOSS_GATT_PRIMARY_SERVICE16(FLOSS_GATT_SVC_HID);
    if (cfg->boot_device || cfg->boot_keyboard || cfg->boot_mouse) {
        t[n++] = (floss_gatt_attr_t)FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_READ | FLOSS_GATT_PROP_WRITE_NO_RSP);
        t[n++] = (floss_gatt_attr_t)FLOSS_GATT_VALUE16_CB(FLOSS_GATT_CHR_PROTOCOL_MODE, FLOSS_GATT_P_RW_ENC,
                                                          protocol_read, protocol_write, 1, NULL);
    }
    s_hogp.report_count = cfg->report_count;
    for (uint8_t k = 0; k < cfg->report_count; k++) {
        hogp_report_t *r = &s_hogp.reports[k];
        memset(r, 0, sizeof(*r));
        r->id = cfg->reports[k].id;
        r->type = cfg->reports[k].type;
        r->len = cfg->reports[k].len;
        r->index = k;
        r->cur_len = r->len; /* reads before the first update return zeros */
        r->ref[0] = r->id;
        r->ref[1] = r->type;

        uint8_t props;
        uint16_t perms;
        if (r->type == FLOSS_HID_REPORT_INPUT) {
            props = FLOSS_GATT_PROP_READ | FLOSS_GATT_PROP_NOTIFY;
            perms = FLOSS_GATT_P_RD_ENC;
        } else if (r->type == FLOSS_HID_REPORT_OUTPUT) {
            props = FLOSS_GATT_PROP_READ | FLOSS_GATT_PROP_WRITE | FLOSS_GATT_PROP_WRITE_NO_RSP;
            perms = FLOSS_GATT_P_RW_ENC;
        } else {
            props = FLOSS_GATT_PROP_READ | FLOSS_GATT_PROP_WRITE;
            perms = FLOSS_GATT_P_RW_ENC;
        }
        t[n++] = (floss_gatt_attr_t)FLOSS_GATT_CHARACTERISTIC(props);
        value_at[k] = n;
        t[n++] = (floss_gatt_attr_t)FLOSS_GATT_VALUE16_CB(FLOSS_GATT_CHR_REPORT, perms, report_read,
                                                          r->type == FLOSS_HID_REPORT_INPUT ? NULL : report_write,
                                                          r->len, r);
        if (r->type == FLOSS_HID_REPORT_INPUT) {
            t[n] = (floss_gatt_attr_t)FLOSS_GATT_CCCD_P(r->cccd, FLOSS_GATT_P_READ | FLOSS_GATT_P_WR_ENC);
            t[n].write = report_cccd_write;
            t[n].user = r;
            n++;
        }
        t[n++] = (floss_gatt_attr_t)FLOSS_GATT_DESCRIPTOR16(FLOSS_GATT_UUID_REPORT_REF, FLOSS_GATT_P_RD_ENC, r->ref,
                                                            2);
    }
    s_hogp.boot_in[0] = NULL;
    s_hogp.boot_in[1] = NULL;
    if (cfg->boot_keyboard) {
        s_hogp.boot_in[0] = add_boot_report(t, &n, value_at, FLOSS_GATT_CHR_BOOT_KB_INPUT, FLOSS_HID_BOOT_KEYBOARD,
                                            FLOSS_HID_REPORT_INPUT, FLOSS_HID_BOOT_KEYBOARD_LEN,
                                            FLOSS_HID_BOOT_KEYBOARD_LEN);
        (void)add_boot_report(t, &n, value_at, FLOSS_GATT_CHR_BOOT_KB_OUTPUT, FLOSS_HID_BOOT_KEYBOARD,
                              FLOSS_HID_REPORT_OUTPUT, 1, 1);
    }
    if (cfg->boot_mouse) {
        s_hogp.boot_in[1] = add_boot_report(t, &n, value_at, FLOSS_GATT_CHR_BOOT_MOUSE_INPUT, FLOSS_HID_BOOT_MOUSE,
                                            FLOSS_HID_REPORT_INPUT, FLOSS_HOGP_BOOT_MOUSE_MAX_LEN,
                                            FLOSS_HID_BOOT_MOUSE_MIN_LEN);
    }
    t[n++] = (floss_gatt_attr_t)FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_READ);
    t[n++] = (floss_gatt_attr_t)FLOSS_GATT_VALUE16(FLOSS_GATT_CHR_REPORT_MAP, FLOSS_GATT_P_RD_ENC, cfg->report_map,
                                                   cfg->report_map_len, NULL);
    t[n++] = (floss_gatt_attr_t)FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_READ);
    t[n++] = (floss_gatt_attr_t)FLOSS_GATT_VALUE16(FLOSS_GATT_CHR_HID_INFORMATION, FLOSS_GATT_P_RD_ENC,
                                                   s_hogp.hid_info, 4, NULL);
    t[n++] = (floss_gatt_attr_t)FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_WRITE_NO_RSP);
    t[n++] = (floss_gatt_attr_t)FLOSS_GATT_VALUE16_CB(FLOSS_GATT_CHR_HID_CONTROL_POINT, FLOSS_GATT_P_WR_ENC, NULL,
                                                      control_write, 1, NULL);

    uint16_t base = floss_gatt_register(t, n);
    if (base == 0) {
        s_hogp.cfg = NULL;
        return FLOSS_ERR_NOMEM;
    }
    for (uint8_t k = 0; k < s_hogp.report_count; k++) {
        s_hogp.reports[k].value_handle = (uint16_t)(base + value_at[k]);
    }
    FLOSS_LOGI(LE, "hogp: HID service at 0x%04x-0x%04x, %u reports%s%s", (unsigned)base, (unsigned)(base + n - 1),
               (unsigned)cfg->report_count, cfg->boot_keyboard ? ", boot keyboard" : "",
               cfg->boot_mouse ? ", boot mouse" : "");
    return FLOSS_OK;
}

floss_err_t floss_hogp_send_input_now(uint8_t report_id, const uint8_t *data, uint16_t len) {
    hogp_report_t *r = find_report(FLOSS_HID_REPORT_INPUT, report_id);
    if (r == NULL) {
        return FLOSS_ERR_NOT_FOUND;
    }
    if (len > r->len) {
        return FLOSS_ERR_INVALID;
    }
    floss_err_t res = FLOSS_ERR_BUSY;
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        uint16_t handle = floss_le_link_handle_at(i);
        if (!s_hogp.links[i].connected || !(r->cccd.v[i] & FLOSS_GATT_CCCD_NOTIFY) || !can_send(handle)) {
            continue;
        }
        uint8_t *value;
        uint16_t cap;
        floss_buf_t *b = floss_gatt_notify_alloc(handle, r->value_handle, &value, &cap);
        if (b == NULL) {
            continue;
        }
        uint16_t n = FLOSS_MIN(len, cap);
        memcpy(value, data, n);
        if (floss_gatt_notify_send(b, n, FLOSS_BUF_F_PRIORITY) == FLOSS_OK) {
            FLOSS_STAT_INC(hid_reports_sent);
            res = FLOSS_OK;
        }
    }
    return res;
}

bool floss_hogp_connected(void) {
    return any_connected();
}

uint16_t floss_hogp_handle(void) {
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        if (s_hogp.links[i].connected) {
            return floss_le_link_handle_at(i);
        }
    }
    return 0xFFFF;
}

uint8_t floss_hogp_protocol(void) {
    return s_hogp.protocol;
}

bool floss_hogp_suspended(void) {
    return s_hogp.suspended;
}

void floss_hogp_set_report_interval(uint32_t interval_us) {
    s_hogp.interval_us = interval_us;
    floss_timer_stop(&s_hogp.tick);
    update_tick();
}

/* ------------------------------------------------------------------------ */
/* Lifecycle                                                                */
/* ------------------------------------------------------------------------ */

void floss_hogp_reset(void) {
    memset(&s_hogp, 0, sizeof(s_hogp));
    s_hogp.protocol = FLOSS_HID_PROTOCOL_REPORT;
    floss_timer_init(&s_hogp.tick, tick_fired, NULL);
}

void floss_hogp_on_link_down(uint16_t handle, uint8_t reason) {
    int i = floss_le_link_index(handle);
    if (i < 0 || s_hogp.cfg == NULL) {
        return;
    }
    hogp_link_t *hl = &s_hogp.links[i];
    bool was = hl->connected;
    memset(hl, 0, sizeof(*hl));
    for (uint8_t k = 0; k < s_hogp.report_count; k++) {
        s_hogp.reports[k].sent_seq[i] = 0;
    }
    s_hogp.suspended = false;
    s_hogp.protocol = FLOSS_HID_PROTOCOL_REPORT; /* every connection starts in report protocol */
    if (was) {
        FLOSS_LOGI(LE, "hogp: host 0x%04x gone", (unsigned)handle);
        if (s_hogp.cb.on_disconnected) {
            s_hogp.cb.on_disconnected(handle, reason);
        }
    }
    /* The link is still in the LE table during this call: recompute the
     * timer from the remaining links only. */
    update_tick();
}

void floss_hogp_on_security(uint16_t handle, uint8_t level) {
    if (s_hogp.cfg == NULL) {
        return;
    }
    update_connections();
    service_all(false);
}

void floss_hogp_on_credits(void) {
    if (s_hogp.cfg == NULL) {
        return;
    }
    service_all(false);
}

void floss_hogp_poll(void) {
    if (s_hogp.cfg == NULL) {
        return;
    }
    update_connections();
    service_all(false);
}

#endif /* FLOSS_ENABLE_LE */
