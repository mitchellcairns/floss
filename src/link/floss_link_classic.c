/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_link_classic.c - Link profile over Classic (BR/EDR): one L2CAP
 * channel on FLOSS_LINK_PSM between a pad and its dongle.
 *
 * ===========================================================================
 *  ROLES
 * ===========================================================================
 *
 *   pad (peripheral)                       dongle (central)
 *   ----------------                       ----------------
 *   serves FLOSS_LINK_PSM (encrypted)      PAIRING: inquiry; the first answer
 *   EIR: name + link service UUID            whose EIR lists the link service
 *   CoD: peripheral / gamepad                (and starts with cfg->name, if
 *   PAIRING: discoverable + connectable      set) -> cancel inquiry -> page it
 *   CONNECTING: connectable, fast page     CONNECTING: page the stored pad,
 *     scan (FLOSS_LINK_PAD_PSCAN_*)          backoff 0.2 s doubling to 5 s
 *   CONNECTED: page scan OFF               link up: link policy without role
 *     (docs/PERFORMANCE.md finding 3)        switch (we stay central), sniff
 *                                            only if wanted, QoS poll 1.25 ms,
 *                                            supervision 2 s, authenticate
 *                                            (SSP just works), open the PSM
 *
 * The data path is the channel; the ACL link exists for it. When the
 * channel closes the link is dropped, and when the link drops everything
 * ends. Input PDUs go out zero-copy (floss_l2cap_send_buf, written at
 * buf->data) as FLUSHABLE | PRIORITY, exactly like HID input reports; the
 * flush timeout stays infinite (finding 7), the in-flight cap bounds age.
 *
 * Who may connect: the stored peer always; anybody else only while
 * PAIRING, and then becomes the peer (its link key is in GAP's bond
 * database, the record of who it is under FLOSS_LINK_STORE_KEY_PEER; the
 * previous peer's link key is deleted). Others are refused at channel open.
 *
 * ===========================================================================
 *  IDLE POWER (pad, classic_idle_ms > 0)
 * ===========================================================================
 *
 * After classic_idle_ms without a change in report CONTENT (republishing
 * the same bytes does not count) the pad asks for sniff mode with an
 * interval of classic_idle_sniff_slots; the first changed report requests
 * sniff exit. The pad announces this in HELLO (FLOSS_LINK_HELLO_F_IDLE_SNIFF)
 * so the dongle's link policy allows sniff even if its own config does not.
 *
 * Wake latency, to be measured on hardware: the first changed report waits
 * for the next sniff anchor (up to classic_idle_sniff_slots x 0.625 ms,
 * 10 ms by default) and goes out there; the sniff exit negotiation (LMP,
 * a few ms) then restores 1.25 ms polling for the following reports.
 */
#include "floss_link_priv.h"

#if FLOSS_ENABLE_LINK

#define CL_NO_HANDLE 0xFFFFu
/** Retry a sniff request the command queue could not take. */
#define CL_SNIFF_RETRY_US 100000u

static struct {
    uint16_t handle;          /**< ACL link of the data path, CL_NO_HANDLE if none */
    floss_l2cap_chan_t *chan; /**< the channel (pending or open) */
    bool open;

    /* dongle */
    floss_addr_t target;      /**< address being paged */
    bool paging;
    bool have_candidate;      /**< PAIRING: a pad answered the inquiry */
    floss_addr_t candidate;
    uint8_t attempts;         /**< consecutive failures (backoff) */
    bool peer_idle_sniff;     /**< the pad announced idle sniff in HELLO */
    floss_timer_t retry_timer;

    floss_timer_t close_timer; /**< channel closed but the link stayed: drop it */

    /* pad idle power */
    floss_timer_t idle_timer;
    uint32_t last_change_us;
    uint8_t mode;             /**< FLOSS_LINK_MODE_* of our link */
    bool sniff_req;
    bool exit_req;
} C;

static const floss_l2cap_ops_t s_chan_ops;

static const floss_link_config_t *cfg(void) {
    return floss_link_cfg();
}

static bool is_pad(void) {
    return floss_link_is_pad(cfg());
}

static void log_addr(const char *what, const floss_addr_t *addr) {
    char s[18];
    floss_addr_to_str(addr, s);
    FLOSS_LOGI(LINK, "%s %s", what, s);
}

/** Drop the ACL link of the data path (link down ends the rest). */
static void drop_link(void) {
    if (C.handle != CL_NO_HANDLE) {
        floss_gap_disconnect(C.handle, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED);
    }
}

/**
 * Channel open from/to `addr`: is it our peer? While PAIRING a new device
 * becomes the peer (and the previous one's link key goes).
 */
static bool accept_peer(const floss_addr_t *addr) {
    floss_link_peer_t p;
    bool have = floss_link_peer_get(&p);
    if (have && floss_addr_eq(&p.addr, addr)) {
        return true;
    }
    if (floss_link_state() != FLOSS_LINK_STATE_PAIRING) {
        log_addr("refusing (not paired, not pairing):", addr);
        return false;
    }
    if (have) {
        floss_gap_bond_delete(&p.addr);
    }
    floss_link_peer_set(0, addr);
    return true;
}

/* ------------------------------------------------------------------------ */
/* Pad                                                                      */
/* ------------------------------------------------------------------------ */

static void pad_apply_scan(floss_link_state_t st) {
    bool pairing = st == FLOSS_LINK_STATE_PAIRING;
    floss_gap_set_scan(pairing, pairing || st == FLOSS_LINK_STATE_CONNECTING);
}

static void pad_rest(void) {
    floss_link_state_t st = floss_link_rest_state();
    floss_link_set_state(st);
    pad_apply_scan(st);
}

/** Back to active mode (fast polling) if we are in, or asked for, sniff. */
static void pad_wake(void) {
    if ((C.mode == FLOSS_LINK_MODE_SNIFF || C.sniff_req) && !C.exit_req && C.handle != CL_NO_HANDLE) {
        if (floss_gap_sniff_exit(C.handle) == FLOSS_OK) {
            C.exit_req = true;
            C.sniff_req = false;
            FLOSS_LOGD(LINK, "input changed: leaving sniff");
        }
    }
}

/** Core hook (hot path, only with classic_idle_ms): a report with new content is going out. */
static void pad_on_input_change(void) {
    C.last_change_us = floss_hal_time_us();
    pad_wake();
}

static void idle_fired(floss_timer_t *t, void *arg) {
    (void)arg;
    if (!floss_link_session_active() || C.handle == CL_NO_HANDLE || cfg()->classic_idle_ms == 0) {
        return;
    }
    uint32_t idle_us = cfg()->classic_idle_ms * 1000u;
    uint32_t since = floss_hal_time_us() - C.last_change_us;
    if (since < idle_us) {
        floss_timer_start(t, idle_us - since);
        return;
    }
    if (C.mode != FLOSS_LINK_MODE_ACTIVE || C.sniff_req || C.exit_req) {
        return; /* MODE_CHANGE back to active restarts the timer */
    }
    uint16_t iv = cfg()->classic_idle_sniff_slots;
    if (floss_gap_sniff(C.handle, iv, iv, 1, 0) == FLOSS_OK) {
        C.sniff_req = true;
        FLOSS_LOGI(LINK, "idle %lu ms: requesting sniff (%u slots)", (unsigned long)cfg()->classic_idle_ms, iv);
    } else {
        floss_timer_start(t, CL_SNIFF_RETRY_US);
    }
}

static void pad_on_mode_change(const floss_gap_evt_t *evt) {
    if (evt->status != FLOSS_HCI_SUCCESS) {
        C.sniff_req = false; /* refused: the idle timer tries again later */
        C.exit_req = false;
        if (C.mode == FLOSS_LINK_MODE_ACTIVE && floss_link_session_active() && cfg()->classic_idle_ms) {
            floss_timer_start(&C.idle_timer, cfg()->classic_idle_ms * 1000u);
        }
        return;
    }
    C.mode = evt->mode;
    if (evt->mode == FLOSS_LINK_MODE_ACTIVE) {
        C.sniff_req = false;
        C.exit_req = false;
        if (floss_link_session_active() && cfg()->classic_idle_ms) {
            floss_timer_start(&C.idle_timer, cfg()->classic_idle_ms * 1000u);
        }
        return;
    }
    if (evt->mode == FLOSS_LINK_MODE_SNIFF) {
        C.sniff_req = false;
        FLOSS_LOGI(LINK, "sniff (%u slots)", evt->interval);
        /* Input changed while the request was in flight: back to active. */
        if ((uint32_t)(floss_hal_time_us() - C.last_change_us) < cfg()->classic_idle_ms * 1000u) {
            pad_wake();
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Dongle                                                                   */
/* ------------------------------------------------------------------------ */

static void dongle_search(void);

/** Does this inquiry answer come from a FLOSS link pad we want? */
static bool dongle_match(const floss_gap_evt_t *evt) {
    if (!floss_gap_eir_has_uuid128(evt->eir, evt->eir_len, floss_link_uuid_service)) {
        return false;
    }
    const char *want = cfg()->name;
    if (!want) {
        return true;
    }
    size_t wlen = strlen(want);
    uint8_t nlen = 0;
    const uint8_t *name = floss_gap_eir_find(evt->eir, evt->eir_len, FLOSS_EIR_NAME_COMPLETE, &nlen);
    if (!name) {
        name = floss_gap_eir_find(evt->eir, evt->eir_len, FLOSS_EIR_NAME_SHORT, &nlen);
    }
    return name && nlen >= wlen && memcmp(name, want, wlen) == 0;
}

static void dongle_retry_later(void) {
    floss_timer_start(&C.retry_timer, floss_link_backoff_us(C.attempts));
    if (C.attempts < 16) {
        C.attempts++;
    }
}

static void dongle_page(const floss_addr_t *addr) {
    C.target = *addr;
    floss_err_t err = floss_gap_connect(addr);
    if (err == FLOSS_OK) {
        C.paging = true;
        return;
    }
    FLOSS_LOGD(LINK, "page not started: %s", floss_err_str(err));
    if (err == FLOSS_ERR_BUSY && floss_gap_inquiry_active()) {
        floss_gap_inquiry_cancel();
    }
    dongle_retry_later();
}

/** PAIRING: run inquiry rounds until a pad answers. */
static void dongle_search(void) {
    floss_link_set_state(FLOSS_LINK_STATE_PAIRING);
    C.have_candidate = false;
    if (floss_gap_inquiry_active() || C.paging || C.handle != CL_NO_HANDLE) {
        return; /* INQUIRY_COMPLETE / the link's end brings us back */
    }
    floss_err_t err = floss_gap_inquiry_start(FLOSS_LINK_INQUIRY_LENGTH, 0);
    if (err != FLOSS_OK) {
        FLOSS_LOGD(LINK, "inquiry not started: %s", floss_err_str(err));
        dongle_retry_later();
    }
}

static void dongle_rest(void) {
    floss_link_state_t st = floss_link_rest_state();
    if (st == FLOSS_LINK_STATE_PAIRING) {
        dongle_search();
        return;
    }
    floss_link_set_state(st);
    floss_link_peer_t p;
    if (st == FLOSS_LINK_STATE_CONNECTING && floss_link_peer_get(&p) && !C.paging && C.handle == CL_NO_HANDLE) {
        log_addr("reconnecting to pad", &p.addr);
        dongle_page(&p.addr);
    }
}

/** A page / link attempt failed or the link was lost. */
static void dongle_after_failure(void) {
    if (floss_link_state() == FLOSS_LINK_STATE_PAIRING) {
        C.have_candidate = false;
        dongle_search();
        return;
    }
    floss_link_state_t st = floss_link_rest_state();
    if (st == FLOSS_LINK_STATE_PAIRING) {
        dongle_search();
        return;
    }
    floss_link_set_state(st);
    if (st == FLOSS_LINK_STATE_CONNECTING) {
        dongle_retry_later();
    }
}

static void retry_fired(floss_timer_t *t, void *arg) {
    (void)t;
    (void)arg;
    if (C.paging || C.handle != CL_NO_HANDLE) {
        return;
    }
    floss_link_state_t st = floss_link_state();
    floss_link_peer_t p;
    if (st == FLOSS_LINK_STATE_PAIRING) {
        if (C.have_candidate && !floss_gap_inquiry_active()) {
            dongle_page(&C.candidate);
        } else if (!C.have_candidate) {
            dongle_search();
        }
    } else if (st == FLOSS_LINK_STATE_CONNECTING && floss_link_peer_get(&p)) {
        dongle_page(&p.addr);
    }
}

/** Fresh link to a pad: tune it, secure it, open the data channel. */
static void dongle_link_up(uint16_t handle, const floss_addr_t *addr) {
    C.handle = handle;
    C.paging = false;
    floss_timer_stop(&C.retry_timer);
    log_addr("link up to pad", addr);

    /* Stay central (no role switch); sniff only if somebody wants idle power. */
    uint16_t policy = (cfg()->classic_idle_ms > 0 || C.peer_idle_sniff) ? FLOSS_LINK_POLICY_SNIFF : 0;
    floss_gap_set_link_policy(handle, policy);
    if (cfg()->classic_poll_us) {
        floss_gap_qos_setup(handle, cfg()->classic_poll_us, 0);
    }
    floss_gap_set_supervision_timeout(handle, FLOSS_LINK_CLASSIC_SUPERVISION_SLOTS);

    floss_gap_authenticate(handle); /* SSP just works; L2CAP also waits for encryption */
    floss_l2cap_chan_t *ch = NULL;
    floss_err_t err = floss_l2cap_connect(handle, FLOSS_LINK_PSM, &s_chan_ops, NULL, FLOSS_SEC_ENCRYPTED, &ch);
    if (err != FLOSS_OK) {
        FLOSS_LOGW(LINK, "cannot open the link channel: %s", floss_err_str(err));
        drop_link();
        return;
    }
    C.chan = ch;
}

static void cl_on_peer_hello(uint8_t flags) {
    if (is_pad()) {
        return;
    }
    if ((flags & FLOSS_LINK_HELLO_F_IDLE_SNIFF) && !C.peer_idle_sniff) {
        C.peer_idle_sniff = true;
        if (cfg()->classic_idle_ms == 0 && C.handle != CL_NO_HANDLE) {
            floss_gap_set_link_policy(C.handle, FLOSS_LINK_POLICY_SNIFF); /* the pad will ask for sniff */
        }
    }
}

static void dongle_on_inquiry(const floss_gap_evt_t *evt) {
    bool searching = floss_link_state() == FLOSS_LINK_STATE_PAIRING && C.handle == CL_NO_HANDLE && !C.paging;
    if (evt->type == FLOSS_GAP_EVT_INQUIRY_RESULT) {
        if (searching && !C.have_candidate && dongle_match(evt)) {
            C.have_candidate = true;
            C.candidate = evt->addr;
            log_addr("found pad", &evt->addr);
            if (floss_gap_inquiry_cancel() != FLOSS_OK && !floss_gap_inquiry_active()) {
                dongle_page(&C.candidate);
            }
        }
        return;
    }
    /* INQUIRY_COMPLETE */
    if (!searching) {
        return;
    }
    if (C.have_candidate) {
        dongle_page(&C.candidate);
    } else if (evt->status != FLOSS_HCI_SUCCESS) {
        dongle_retry_later(); /* controller refused the inquiry: do not spin */
    } else {
        dongle_search(); /* nobody yet: next round */
    }
}

/* ------------------------------------------------------------------------ */
/* Channel                                                                  */
/* ------------------------------------------------------------------------ */

static void chan_on_open(floss_l2cap_chan_t *ch, uint16_t status) {
    if (status != 0) {
        if (ch == C.chan) {
            C.chan = NULL;
            FLOSS_LOGW(LINK, "link channel failed to open (0x%04X)", status);
            drop_link();
        }
        return;
    }
    if (C.chan && C.chan != ch) {
        FLOSS_LOGW(LINK, "second link channel refused");
        floss_l2cap_disconnect(ch);
        return;
    }
    floss_conn_t *c = floss_hci_conn_by_handle(ch->handle);
    if (!c || (C.handle != CL_NO_HANDLE && ch->handle != C.handle)) {
        floss_l2cap_disconnect(ch);
        return;
    }
    if (!accept_peer(&c->addr)) {
        C.chan = NULL;
        floss_l2cap_disconnect(ch);
        floss_gap_disconnect(ch->handle, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED);
        return;
    }
    C.chan = ch;
    C.open = true;
    C.handle = ch->handle;
    C.attempts = 0;
    C.mode = c->mode;
    if (is_pad()) {
        floss_gap_set_scan(false, false); /* page scan steals radio time from the link */
        if (cfg()->classic_idle_ms) {
            floss_gap_set_link_policy(C.handle, FLOSS_LINK_POLICY_SNIFF);
            C.last_change_us = floss_hal_time_us();
            C.sniff_req = false;
            C.exit_req = false;
            floss_timer_start(&C.idle_timer, cfg()->classic_idle_ms * 1000u);
        }
    }
    floss_link_session_up(&c->addr);
}

static void chan_on_close(floss_l2cap_chan_t *ch) {
    if (ch != C.chan) {
        return;
    }
    C.chan = NULL;
    C.open = false;
    /* If the link itself is going down, its DISCONNECTED (with the real
     * reason) follows right away and cancels this; otherwise drop it. */
    floss_timer_start(&C.close_timer, 0);
}

static void chan_on_data(floss_l2cap_chan_t *ch, const uint8_t *data, uint16_t len) {
    if (ch == C.chan && C.open) {
        floss_link_rx_pdu(data, len);
    }
}

static const floss_l2cap_ops_t s_chan_ops = {
    .on_open = chan_on_open,
    .on_close = chan_on_close,
    .on_data = chan_on_data,
    .on_can_send = NULL, /* floss_link_poll() services the input path every loop */
};

static void close_fired(floss_timer_t *t, void *arg) {
    (void)t;
    (void)arg;
    if (C.handle != CL_NO_HANDLE) {
        floss_link_session_down(FLOSS_HCI_ERR_REMOTE_USER_TERMINATED);
        drop_link();
    }
}

/* ------------------------------------------------------------------------ */
/* Link events                                                              */
/* ------------------------------------------------------------------------ */

static void link_down(uint8_t reason) {
    floss_timer_stop(&C.close_timer);
    floss_timer_stop(&C.idle_timer);
    C.handle = CL_NO_HANDLE;
    C.chan = NULL;
    C.open = false;
    C.mode = FLOSS_LINK_MODE_ACTIVE;
    C.sniff_req = false;
    C.exit_req = false;
    C.peer_idle_sniff = false;
    floss_link_session_down(reason);
    if (is_pad()) {
        if (floss_link_state() == FLOSS_LINK_STATE_PAIRING) {
            pad_apply_scan(FLOSS_LINK_STATE_PAIRING);
        } else {
            pad_rest();
        }
    } else {
        dongle_after_failure();
    }
}

static void cl_on_gap_event(const floss_gap_evt_t *evt) {
    switch (evt->type) {
    case FLOSS_GAP_EVT_CONNECTED: {
        if (is_pad()) {
            return; /* the pad learns its link at channel open */
        }
        floss_conn_t *c = floss_hci_conn_by_handle(evt->handle);
        if (C.paging && floss_addr_eq(&evt->addr, &C.target)) {
            dongle_link_up(evt->handle, &evt->addr);
        } else if (c && c->initiator && C.handle != evt->handle && floss_addr_eq(&evt->addr, &C.target)) {
            /* A page we gave up on (cancelled) completed anyway. */
            floss_gap_disconnect(evt->handle, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED);
        }
        return;
    }
    case FLOSS_GAP_EVT_CONNECT_FAILED:
        if (!is_pad() && C.paging && floss_addr_eq(&evt->addr, &C.target)) {
            C.paging = false;
            FLOSS_LOGI(LINK, "page failed: %s", floss_hci_status_str(evt->status));
            dongle_after_failure();
        }
        return;
    case FLOSS_GAP_EVT_DISCONNECTED:
        if (C.handle != CL_NO_HANDLE && evt->handle == C.handle) {
            link_down(evt->status);
        }
        return;
    case FLOSS_GAP_EVT_INQUIRY_RESULT:
    case FLOSS_GAP_EVT_INQUIRY_COMPLETE:
        if (!is_pad()) {
            dongle_on_inquiry(evt);
        }
        return;
    case FLOSS_GAP_EVT_MODE_CHANGE:
        if (evt->handle == C.handle && C.handle != CL_NO_HANDLE) {
            if (is_pad()) {
                pad_on_mode_change(evt);
            } else if (evt->status == FLOSS_HCI_SUCCESS) {
                C.mode = evt->mode;
            }
        }
        return;
    case FLOSS_GAP_EVT_ROLE_CHANGE:
#ifndef FLOSS_LINK_DONGLE_KEEP_CENTRAL
#define FLOSS_LINK_DONGLE_KEEP_CENTRAL 1
#endif
        if (FLOSS_LINK_DONGLE_KEEP_CENTRAL && !is_pad() && evt->handle == C.handle && C.handle != CL_NO_HANDLE &&
            evt->status == 0 && evt->mode == 1) {
            FLOSS_LOGW(LINK, "pad took the central role: switching back");
            floss_gap_switch_role(C.handle, 0);
        }
        return;
    default:
        return;
    }
}

/* ------------------------------------------------------------------------ */
/* Transport operations                                                     */
/* ------------------------------------------------------------------------ */

static floss_err_t cl_init(void) {
    memset(&C, 0, sizeof(C));
    C.handle = CL_NO_HANDLE;
    floss_timer_init(&C.retry_timer, retry_fired, NULL);
    floss_timer_init(&C.close_timer, close_fired, NULL);
    floss_timer_init(&C.idle_timer, idle_fired, NULL);

    floss_err_t err = FLOSS_OK;
    if (is_pad()) {
        err = floss_l2cap_register_psm(FLOSS_LINK_PSM, &s_chan_ops, NULL, FLOSS_SEC_ENCRYPTED);
        if (err == FLOSS_OK) {
            err = floss_gap_set_name(cfg()->name ? cfg()->name : "FLOSS Pad");
        }
        if (err == FLOSS_OK) {
            err = floss_gap_set_class(FLOSS_COD_GAMEPAD);
        }
        if (err == FLOSS_OK) {
            err = floss_gap_eir_add_uuid128(floss_link_uuid_service);
        }
        if (err == FLOSS_OK) {
            err = floss_gap_set_page_scan(FLOSS_LINK_PAD_PSCAN_INTERVAL, FLOSS_LINK_PAD_PSCAN_WINDOW, true);
        }
    }
    floss_gap_set_scan(false, false); /* the rest state is applied once the controller is up */
    return err;
}

static void cl_on_ready(void) {
    /* A (re)started controller has no links, pages or inquiries. */
    floss_timer_stop(&C.retry_timer);
    floss_timer_stop(&C.close_timer);
    floss_timer_stop(&C.idle_timer);
    C.handle = CL_NO_HANDLE;
    C.chan = NULL;
    C.open = false;
    C.paging = false;
    C.have_candidate = false;
    C.attempts = 0;
    C.mode = FLOSS_LINK_MODE_ACTIVE;
    if (is_pad()) {
        pad_rest();
    } else {
        floss_gap_set_scan(false, false);
        dongle_rest();
    }
}

static floss_err_t cl_pair(void) {
    floss_link_set_state(FLOSS_LINK_STATE_PAIRING);
    if (is_pad()) {
        pad_apply_scan(FLOSS_LINK_STATE_PAIRING);
        drop_link(); /* link down keeps us PAIRING */
        return FLOSS_OK;
    }
    C.attempts = 0;
    C.have_candidate = false;
    floss_timer_stop(&C.retry_timer);
    if (C.handle != CL_NO_HANDLE) {
        drop_link(); /* link down sees PAIRING and starts searching */
        return FLOSS_OK;
    }
    if (C.paging) {
        floss_gap_connect_cancel(&C.target); /* CONNECT_FAILED is ignored once paging is false */
        C.paging = false;
    }
    dongle_search();
    return FLOSS_OK;
}

static void cl_on_pair_timeout(void) {
    if (is_pad()) {
        pad_rest();
        return;
    }
    C.have_candidate = false;
    if (floss_gap_inquiry_active()) {
        floss_gap_inquiry_cancel();
    }
    if (C.paging) {
        floss_gap_connect_cancel(&C.target);
        C.paging = false;
    }
    dongle_rest(); /* paging waits (retry) until the inquiry has stopped */
}

static void cl_forget(const floss_link_peer_t *old) {
    if (old) {
        floss_gap_bond_delete(&old->addr);
    }
    cl_pair(); /* no peer: pairing is the rest state */
}

static bool cl_can_send(void) {
    return C.open && floss_l2cap_can_send_now(C.chan) && floss_link_inflight_ok(C.handle);
}

static floss_buf_t *cl_input_alloc(uint8_t **pdu, uint16_t *cap) {
    if (!C.open) {
        return NULL;
    }
    floss_buf_t *b = floss_buf_alloc();
    if (!b) {
        FLOSS_STAT_INC(buf_alloc_failures);
        return NULL;
    }
    *pdu = b->data;
    *cap = FLOSS_MIN(C.chan->remote_mtu, floss_buf_tailroom(b));
    return b;
}

static floss_err_t cl_input_send(floss_buf_t *b, uint16_t len) {
    b->len = len;
    return floss_l2cap_send_buf(C.chan, b, FLOSS_BUF_F_FLUSHABLE | FLOSS_BUF_F_PRIORITY);
}

static floss_err_t cl_send_ctrl(const uint8_t *pdu, uint16_t len) {
    if (!C.open) {
        return FLOSS_ERR_STATE;
    }
    return floss_l2cap_send(C.chan, pdu, len);
}

static uint32_t cl_conn_period_us(void) {
    return 1250u; /* one poll per slot pair */
}

const floss_link_tp_t floss_link_classic = {
    .name = "Classic",
    .init = cl_init,
    .on_ready = cl_on_ready,
    .on_gap_event = cl_on_gap_event,
    .pair = cl_pair,
    .on_pair_timeout = cl_on_pair_timeout,
    .forget = cl_forget,
    .can_send = cl_can_send,
    .input_alloc = cl_input_alloc,
    .input_send = cl_input_send,
    .send_ctrl = cl_send_ctrl,
    .conn_period_us = cl_conn_period_us,
    .on_input_change = pad_on_input_change,
    .on_peer_hello = cl_on_peer_hello,
    .poll = NULL,
};

#endif /* FLOSS_ENABLE_LINK */
