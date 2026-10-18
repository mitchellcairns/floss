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
 * One side waits to be connected, the other connects. By default the pad
 * waits and the dongle connects; with classic_pad_connects it is the other
 * way round. Connecting is independent of the data roles (the pad sends
 * INPUT, the dongle OUTPUT/PING), so the code below talks about the
 * WAITING and the CONNECTING side:
 *
 *   waiting (peripheral)                   connecting (central)
 *   --------------------                   --------------------
 *   serves FLOSS_LINK_PSM (encrypted)      PAIRING: inquiry; the first answer
 *   EIR: name + link service UUID            whose EIR lists the link service
 *   CoD: gamepad (pad)                       (and starts with the peer name,
 *   PAIRING: discoverable + connectable      if set) -> cancel inquiry -> page
 *   CONNECTING: connectable, fast page     CONNECTING: page the stored peer,
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
 * Without stored keys (classic_store_keys false) nothing is kept: every
 * connection pairs again and the key lives with the link. The waiting side
 * remembers nobody, so it rests in PAIRING (discoverable + connectable) and
 * accepts any link device of the other role. The connecting side remembers
 * its last peer's address only: it pages that peer first and, when it does
 * not answer, searches. A search round then prefers the remembered peer and
 * otherwise takes the strongest answer of the round.
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

    /* connecting side */
    floss_addr_t target;      /**< address being paged */
    bool paging;
    bool have_candidate;      /**< PAIRING: a peer answered the inquiry */
    floss_addr_t candidate;
    int8_t candidate_rssi;    /**< without stored keys: the strongest answer of the round */
    uint8_t attempts;         /**< consecutive failures (backoff) */
    bool peer_idle_sniff;     /**< the pad announced idle sniff in HELLO */
    floss_timer_t retry_timer;

    floss_timer_t close_timer; /**< channel closed but the link stayed: drop it */

    /* adaptive hopping: dongle learn-then-freeze (FLOSS_LINK_AFH_FREEZE) */
    floss_timer_t afh_timer;
    bool afh_learning;

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

/** The side that waits to be connected: the pad, or the dongle with classic_pad_connects. */
static bool is_waiting(void) {
    return is_pad() != cfg()->classic_pad_connects;
}

static bool store_keys(void) {
    return cfg()->classic_store_keys;
}

/** What the peer is called in logs. */
static const char *peer_word(void) {
    return is_pad() ? "dongle" : "pad";
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
    if (!store_keys()) {
        if (!is_waiting()) {
            floss_link_peer_set(0, addr); /* whom to try first next time */
        }
        return true;
    }
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
/* Waiting side                                                             */
/* ------------------------------------------------------------------------ */

static void wait_apply_scan(floss_link_state_t st) {
    if (floss_link_held()) {
        floss_gap_set_scan(false, false); /* hold: not connectable either */
        return;
    }
    bool pairing = st == FLOSS_LINK_STATE_PAIRING;
    floss_gap_set_scan(pairing, pairing || st == FLOSS_LINK_STATE_CONNECTING);
}

static void wait_rest(void) {
    floss_link_state_t st = floss_link_rest_state();
    floss_link_set_state(st);
    wait_apply_scan(st);
}

/* ------------------------------------------------------------------------ */
/* Pad idle power                                                           */
/* ------------------------------------------------------------------------ */

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
/* Connecting side                                                          */
/* ------------------------------------------------------------------------ */

static void conn_search(void);

/** Does this inquiry answer come from a FLOSS link device we want? */
static bool conn_match(const floss_gap_evt_t *evt) {
    if (!floss_gap_eir_has_uuid128(evt->eir, evt->eir_len, floss_link_uuid_service)) {
        return false;
    }
    /* A dongle's `name` is the pad name it looks for (unless peer_name says otherwise). */
    const char *want = cfg()->peer_name ? cfg()->peer_name : (is_pad() ? NULL : cfg()->name);
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

static void conn_retry_later(void) {
    if (floss_link_held()) {
        return; /* releasing the hold starts over */
    }
    floss_timer_start(&C.retry_timer, floss_link_backoff_us(C.attempts));
    if (C.attempts < 16) {
        C.attempts++;
    }
}

static void conn_page(const floss_addr_t *addr) {
    if (floss_link_held()) {
        return;
    }
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
    conn_retry_later();
}

/** PAIRING: run inquiry rounds until a peer answers. */
static void conn_search(void) {
    floss_link_set_state(FLOSS_LINK_STATE_PAIRING);
    C.have_candidate = false;
    if (floss_link_held() || floss_gap_inquiry_active() || C.paging || C.handle != CL_NO_HANDLE) {
        return; /* INQUIRY_COMPLETE / the link's end / the release brings us back */
    }
    floss_err_t err = floss_gap_inquiry_start(FLOSS_LINK_INQUIRY_LENGTH, 0);
    if (err != FLOSS_OK) {
        FLOSS_LOGD(LINK, "inquiry not started: %s", floss_err_str(err));
        conn_retry_later();
    }
}

static void conn_rest(void) {
    floss_link_state_t st = floss_link_rest_state();
    if (st == FLOSS_LINK_STATE_PAIRING) {
        conn_search();
        return;
    }
    floss_link_set_state(st);
    floss_link_peer_t p;
    if (st == FLOSS_LINK_STATE_CONNECTING && floss_link_peer_get(&p) && !C.paging && C.handle == CL_NO_HANDLE) {
        FLOSS_LOGI(LINK, "reconnecting to the %s", peer_word());
        log_addr("  at", &p.addr);
        conn_page(&p.addr);
    }
}

/** A page failed (page_failed) or a link / its channel was lost. */
static void conn_after_failure(bool page_failed) {
    if (floss_link_state() == FLOSS_LINK_STATE_PAIRING) {
        C.have_candidate = false;
        conn_search();
        return;
    }
    floss_link_state_t st = floss_link_rest_state();
    /* Without stored keys, once the remembered peer does not answer a page,
     * any free peer will do: look for one (the pairing window brings us back
     * to the remembered peer when nobody answers). */
    if (st == FLOSS_LINK_STATE_PAIRING || (!store_keys() && page_failed && st == FLOSS_LINK_STATE_CONNECTING)) {
        conn_search();
        return;
    }
    floss_link_set_state(st);
    if (st == FLOSS_LINK_STATE_CONNECTING) {
        conn_retry_later();
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
            conn_page(&C.candidate);
        } else if (!C.have_candidate) {
            conn_search();
        }
    } else if (st == FLOSS_LINK_STATE_CONNECTING && floss_link_peer_get(&p)) {
        conn_page(&p.addr);
    }
}

/* ------------------------------------------------------------------------ */
/* Adaptive hopping without periodic scans                                  */
/* ------------------------------------------------------------------------ */
/*
 * Measured on two CYW43439 radios: with controller channel assessment on,
 * the link goes silent for ~25 ms every few seconds (two independent
 * schedules, ~4 s and ~3 s) while the radio measures all 79 channels.
 * Turning assessment off removes the gaps (max report interval 3.8 ms) but
 * then the link also hops onto channels that are busy (e.g. Wi-Fi).
 *
 * FREEZE gets both: the dongle (central) assesses for a short window after
 * connecting, reads the resulting hopping map, gives it back to its
 * controller as the host classification (bad channels stay excluded) and
 * turns assessment off. The pad never assesses. Re-learning can be
 * periodic (classic_afh_refresh_s) or on demand (floss_link_afh_relearn).
 */
static const uint8_t k_all_channels[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F};

static unsigned afh_count(const uint8_t map[10]) {
    unsigned n = 0;
    for (int i = 0; i < 10; i++) {
        for (uint8_t b = map[i]; b; b &= (uint8_t)(b - 1u)) {
            n++;
        }
    }
    return n;
}

/** Controller-wide setting applied whenever the controller (re)starts. */
static void afh_apply_rest(void) {
    if (cfg()->classic_afh == FLOSS_LINK_AFH_CONTROLLER) {
        return; /* leave the controller default alone */
    }
    floss_gap_set_afh_assessment(false);
    floss_gap_set_afh_host_channels(k_all_channels);
}

static void afh_start_learning(void) {
    if (is_waiting() || cfg()->classic_afh != FLOSS_LINK_AFH_FREEZE || C.handle == CL_NO_HANDLE) {
        return;
    }
    C.afh_learning = true;
    floss_gap_set_afh_host_channels(k_all_channels); /* forget the old map: re-admit recovered channels */
    floss_gap_set_afh_assessment(true);
    floss_timer_start(&C.afh_timer, cfg()->classic_afh_learn_ms * 1000u);
    FLOSS_LOGI(LINK, "AFH: learning channels for %lu ms", (unsigned long)cfg()->classic_afh_learn_ms);
}

static void afh_on_map(uint16_t handle, uint8_t status, uint8_t afh_mode, const uint8_t map[10], void *user) {
    FLOSS_UNUSED(user);
    if (handle != C.handle) {
        return; /* link changed meanwhile */
    }
    unsigned n = status == 0 ? afh_count(map) : 0u;
    /* Keep the learned map only if hopping was on and enough channels remain. */
    floss_gap_set_afh_host_channels(status == 0 && afh_mode == 1 && n >= 20 ? map : k_all_channels);
    floss_gap_set_afh_assessment(false);
    C.afh_learning = false;
    FLOSS_LOGI(LINK, "AFH: frozen on %u of 79 channels, assessment off", n >= 20 ? n : 79u);
    if (cfg()->classic_afh_refresh_s) {
        floss_timer_start(&C.afh_timer, cfg()->classic_afh_refresh_s * 1000000u);
    }
}

static void afh_fired(floss_timer_t *t, void *arg) {
    FLOSS_UNUSED(t);
    FLOSS_UNUSED(arg);
    if (C.handle == CL_NO_HANDLE) {
        return;
    }
    if (C.afh_learning) {
        if (floss_gap_read_afh_map(C.handle, afh_on_map, NULL) != FLOSS_OK) {
            floss_timer_start(&C.afh_timer, 100000u); /* command queue busy: try again shortly */
        }
    } else {
        afh_start_learning(); /* periodic refresh */
    }
}

floss_err_t floss_link_afh_relearn(void) {
    if (is_waiting() || cfg()->classic_afh != FLOSS_LINK_AFH_FREEZE) {
        return FLOSS_ERR_STATE;
    }
    if (C.handle == CL_NO_HANDLE) {
        return FLOSS_ERR_NOT_FOUND;
    }
    floss_timer_stop(&C.afh_timer);
    afh_start_learning();
    return FLOSS_OK;
}

/** Fresh link to our peer: tune it, secure it, open the data channel. */
static void conn_link_up(uint16_t handle, const floss_addr_t *addr) {
    C.handle = handle;
    C.paging = false;
    floss_timer_stop(&C.retry_timer);
    FLOSS_LOGI(LINK, "link up to the %s", peer_word());
    log_addr("  at", addr);

    /* Stay central (no role switch); sniff only if somebody wants idle power. */
    uint16_t policy = (cfg()->classic_idle_ms > 0 || C.peer_idle_sniff) ? FLOSS_LINK_POLICY_SNIFF : 0;
    floss_gap_set_link_policy(handle, policy);
    if (cfg()->classic_poll_us) {
        floss_gap_qos_setup(handle, cfg()->classic_poll_us, 0);
    }
    floss_gap_set_supervision_timeout(handle, FLOSS_LINK_CLASSIC_SUPERVISION_SLOTS);
    afh_start_learning();

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

static void conn_on_inquiry(const floss_gap_evt_t *evt) {
    bool searching = floss_link_state() == FLOSS_LINK_STATE_PAIRING && C.handle == CL_NO_HANDLE && !C.paging;
    if (evt->type == FLOSS_GAP_EVT_INQUIRY_RESULT) {
        if (!searching || !conn_match(evt)) {
            return;
        }
        bool take_now;
        if (store_keys()) {
            if (C.have_candidate) {
                return;
            }
            take_now = true; /* first answer wins */
        } else {
            /* Remembered peer: at once. Others: the strongest of the round. */
            floss_link_peer_t p;
            take_now = floss_link_peer_get(&p) && floss_addr_eq(&p.addr, &evt->addr);
            if (!take_now && C.have_candidate && evt->rssi <= C.candidate_rssi) {
                return;
            }
        }
        C.have_candidate = true;
        C.candidate = evt->addr;
        C.candidate_rssi = evt->rssi;
        FLOSS_LOGI(LINK, "found a %s (rssi %d)", peer_word(), evt->rssi);
        log_addr("  at", &evt->addr);
        if (take_now && floss_gap_inquiry_cancel() != FLOSS_OK && !floss_gap_inquiry_active()) {
            conn_page(&C.candidate);
        }
        return;
    }
    /* INQUIRY_COMPLETE */
    if (!searching) {
        return;
    }
    if (C.have_candidate) {
        conn_page(&C.candidate);
    } else if (evt->status != FLOSS_HCI_SUCCESS) {
        conn_retry_later(); /* controller refused the inquiry: do not spin */
    } else {
        conn_search(); /* nobody yet: next round */
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
    if (is_waiting()) {
        floss_gap_set_scan(false, false); /* page scan steals radio time from the link */
    }
    if (is_pad()) {
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
    floss_timer_stop(&C.afh_timer);
    C.afh_learning = false;
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
    if (is_waiting()) {
        if (floss_link_state() == FLOSS_LINK_STATE_PAIRING) {
            wait_apply_scan(FLOSS_LINK_STATE_PAIRING);
        } else {
            wait_rest();
        }
    } else {
        conn_after_failure(false);
    }
}

static void cl_on_gap_event(const floss_gap_evt_t *evt) {
    switch (evt->type) {
    case FLOSS_GAP_EVT_CONNECTED: {
        if (is_waiting()) {
            return; /* the waiting side learns its link at channel open */
        }
        floss_conn_t *c = floss_hci_conn_by_handle(evt->handle);
        if (C.paging && floss_addr_eq(&evt->addr, &C.target)) {
            conn_link_up(evt->handle, &evt->addr);
        } else if (c && c->initiator && C.handle != evt->handle && floss_addr_eq(&evt->addr, &C.target)) {
            /* A page we gave up on (cancelled) completed anyway. */
            floss_gap_disconnect(evt->handle, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED);
        }
        return;
    }
    case FLOSS_GAP_EVT_CONNECT_FAILED:
        if (!is_waiting() && C.paging && floss_addr_eq(&evt->addr, &C.target)) {
            C.paging = false;
            FLOSS_LOGI(LINK, "page failed: %s", floss_hci_status_str(evt->status));
            conn_after_failure(true);
        }
        return;
    case FLOSS_GAP_EVT_DISCONNECTED:
        if (C.handle != CL_NO_HANDLE && evt->handle == C.handle) {
            link_down(evt->status);
        }
        return;
    case FLOSS_GAP_EVT_INQUIRY_RESULT:
    case FLOSS_GAP_EVT_INQUIRY_COMPLETE:
        if (!is_waiting()) {
            conn_on_inquiry(evt);
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
        /* The connecting side (the dongle by default) stays central. */
        if (FLOSS_LINK_DONGLE_KEEP_CENTRAL && !is_waiting() && evt->handle == C.handle && C.handle != CL_NO_HANDLE &&
            evt->status == 0 && evt->mode == 1) {
            FLOSS_LOGW(LINK, "the %s took the central role: switching back", peer_word());
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
    floss_timer_init(&C.afh_timer, afh_fired, NULL);

    floss_err_t err = FLOSS_OK;
    if (!store_keys()) {
        /* Pair on every connection, keep the key with the link only. */
        floss_gap_security_t sec;
        floss_gap_get_security(&sec);
        sec.bondable = false;
        err = floss_gap_set_security(&sec);
        if (is_waiting() && floss_link_peer_get(NULL)) {
            floss_link_peer_clear(); /* the waiting side remembers nobody */
        }
    }
    if (err == FLOSS_OK && !is_waiting() && cfg()->classic_page_timeout_ms) {
        err = floss_gap_set_page_timeout(cfg()->classic_page_timeout_ms);
    }
    if (err == FLOSS_OK && is_waiting()) {
        err = floss_l2cap_register_psm(FLOSS_LINK_PSM, &s_chan_ops, NULL, FLOSS_SEC_ENCRYPTED);
        if (err == FLOSS_OK) {
            err = floss_gap_set_name(cfg()->name ? cfg()->name : (is_pad() ? "FLOSS Pad" : "FLOSS Dongle"));
        }
        if (err == FLOSS_OK && is_pad()) {
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
    afh_apply_rest();
    floss_timer_stop(&C.afh_timer);
    C.afh_learning = false;
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
    if (is_waiting()) {
        wait_rest();
    } else {
        floss_gap_set_scan(false, false);
        conn_rest();
    }
}

static floss_err_t cl_pair(void) {
    floss_link_set_state(FLOSS_LINK_STATE_PAIRING);
    if (is_waiting()) {
        wait_apply_scan(FLOSS_LINK_STATE_PAIRING);
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
    conn_search();
    return FLOSS_OK;
}

static void cl_on_pair_timeout(void) {
    if (is_waiting()) {
        wait_rest();
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
    conn_rest(); /* paging waits (retry) until the inquiry has stopped */
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

/** floss_link_hold(): stop paging and searching (a link already up stays), or carry on. */
static void cl_hold(bool hold) {
    if (is_waiting()) {
        if (hold) {
            floss_gap_set_scan(false, false);
        } else if (C.handle == CL_NO_HANDLE) {
            wait_rest();
        }
        return;
    }
    if (hold) {
        floss_timer_stop(&C.retry_timer);
        C.have_candidate = false;
        if (floss_gap_inquiry_active()) {
            floss_gap_inquiry_cancel();
        }
        if (C.paging) {
            floss_gap_connect_cancel(&C.target); /* CONNECT_FAILED is ignored once paging is false */
            C.paging = false;
        }
        return;
    }
    if (C.handle == CL_NO_HANDLE) {
        C.attempts = 0;
        if (floss_link_state() == FLOSS_LINK_STATE_PAIRING) {
            conn_search();
        } else {
            conn_rest();
        }
    }
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
    .hold = cl_hold,
};

#endif /* FLOSS_ENABLE_LINK */
