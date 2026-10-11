/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_link_le.c - Link profile over LE: the link GATT service.
 *
 * ===========================================================================
 *  ROLES
 * ===========================================================================
 *
 *   pad (peripheral, GATT server)           dongle (central, GATT client)
 *   -----------------------------           -----------------------------
 *   service floss_link_uuid_service:        PAIRING: active scan; the first
 *     input  notify + CCCD (write needs       connectable advertiser with the
 *            encryption)                      link service UUID, the
 *     output write / write w/o response       discoverable flag and (if set)
 *            (needs encryption)               a name starting with cfg->name
 *   advertising: flags + 128-bit service    CONNECTING: connect directly to
 *     UUID (+ name), scan response = name     the stored identity address
 *     PAIRING: general discoverable flag    link up: request security (SMP
 *     otherwise: not discoverable (only       just works / stored bond), MTU,
 *     the bonded dongle connects directly)    2M PHY, find input (+CCCD) and
 *   connection parameters = cfg (link         output, subscribe -> CONNECTED
 *     policy; the dongle asks for them)     PDUs: notifications in, Write
 *   CONNECTED = encrypted AND subscribed      Command out
 *
 * INPUT PDUs leave as notifications built in place (floss_gatt_notify_alloc,
 * zero copy), paced per connection event by default, with the same
 * in-flight cap as the Classic path. Peripheral latency (cfg) lets an idle
 * pad skip events; it still sends at the very next event after a change.
 *
 * Who may connect: a central that has not encrypted the link after
 * FLOSS_LINK_LE_SECURE_TIMEOUT_US is dropped. Once encrypted, its identity
 * (resolved through SMP's bonds, else the connection address) must be the
 * stored peer, unless we are PAIRING: then it becomes the peer (and the
 * previous peer's LE bond is deleted).
 */
#include "floss_link_priv.h"

#if FLOSS_ENABLE_LINK && FLOSS_ENABLE_LE

#define LE_NO_HANDLE 0xFFFFu

/** Dongle connection steps. */
enum {
    STEP_NONE = 0,
    STEP_SECURE,     /* waiting for encryption */
    STEP_MTU,
    STEP_FIND_INPUT,
    STEP_FIND_OUTPUT,
    STEP_SUBSCRIBE,
    STEP_READY,
};

static struct {
    uint16_t handle;          /**< LE link of the data path, LE_NO_HANDLE if none */
    int8_t idx;               /**< floss_le_link_index() of it */
    uint8_t conn_type;
    floss_addr_t conn_addr;   /**< over-the-air address */
    bool secured;             /**< encrypted and identity accepted */
    bool adopted;             /**< this link became our peer while pairing */
    floss_timer_t secure_timer;

    /* pad */
    uint16_t in_vh;           /**< value handle of the input characteristic */

    /* dongle */
    uint8_t step;
    uint16_t d_in_vh, d_in_cccd, d_out_vh;
    bool connecting;
    uint8_t target_type;
    floss_addr_t target;
    uint8_t attempts;
    floss_timer_t retry_timer;
    struct {
        bool valid;
        uint8_t type;
        floss_addr_t addr;
        bool uuid;            /* advertises the link service */
        bool discoverable;    /* in pairing mode */
        bool name_ok;
    } cand;
} E;

/* UUIDs in wire order (little endian), as the GATT table wants them. */
static uint8_t s_uuid_svc_le[16];
static uint8_t s_uuid_in_le[16];
static uint8_t s_uuid_out_le[16];
static floss_gatt_cccd_t s_in_cccd;

static uint8_t le_output_write(uint16_t conn, uint16_t attr_handle, const floss_gatt_attr_t *attr, uint16_t offset,
                               const uint8_t *data, uint16_t len, uint8_t flags);

static const floss_gatt_attr_t s_link_service[] = {
    FLOSS_GATT_PRIMARY_SERVICE128(s_uuid_svc_le),
    FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_NOTIFY),
    {.uuid128 = s_uuid_in_le, .perms = 0}, /* notify only: not readable */
    FLOSS_GATT_CCCD_P(s_in_cccd, FLOSS_GATT_P_READ | FLOSS_GATT_P_WR_ENC),
    FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_WRITE_NO_RSP | FLOSS_GATT_PROP_WRITE),
    {.uuid128 = s_uuid_out_le, .perms = FLOSS_GATT_P_WR_ENC, .write = le_output_write,
     .max_len = FLOSS_LINK_PDU_MAX},
};

static const floss_link_config_t *cfg(void) {
    return floss_link_cfg();
}

static bool is_pad(void) {
    return floss_link_is_pad(cfg());
}

static void reverse16(uint8_t out[16], const uint8_t in[16]) {
    for (unsigned i = 0; i < 16; i++) {
        out[i] = in[15 - i];
    }
}

static void drop_link(void) {
    if (E.handle != LE_NO_HANDLE) {
        floss_le_disconnect(E.handle, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED);
    }
}

static void reset_link(void) {
    floss_timer_stop(&E.secure_timer);
    E.handle = LE_NO_HANDLE;
    E.idx = -1;
    E.secured = false;
    E.adopted = false;
    E.step = STEP_NONE;
    E.d_in_vh = E.d_in_cccd = E.d_out_vh = 0;
}

/** Link up (either role): remember it, insist on encryption soon. */
static void link_up(const floss_le_evt_t *evt) {
    E.handle = evt->handle;
    E.idx = (int8_t)floss_le_link_index(evt->handle);
    E.conn_type = evt->addr_type;
    E.conn_addr = evt->addr;
    E.secured = false;
    E.adopted = false;
    floss_timer_start(&E.secure_timer, FLOSS_LINK_LE_SECURE_TIMEOUT_US);
}

static void secure_timeout(floss_timer_t *t, void *arg) {
    (void)t;
    (void)arg;
    if (E.handle != LE_NO_HANDLE && !E.secured) {
        FLOSS_LOGW(LINK, "LE link not secured in time, dropping it");
        drop_link();
    }
}

/** Is this (identity) address our peer? While PAIRING it becomes the peer. */
static bool check_peer(uint8_t type, const floss_addr_t *addr) {
    floss_link_peer_t p;
    bool have = floss_link_peer_get(&p);
    if (have && p.addr_type == type && floss_addr_eq(&p.addr, addr)) {
        return true;
    }
    if (floss_link_state() != FLOSS_LINK_STATE_PAIRING) {
        char s[18];
        floss_addr_to_str(addr, s);
        FLOSS_LOGI(LINK, "refusing %s: not paired, not pairing", s);
        return false;
    }
    if (have) {
        floss_smp_bond_delete(p.addr_type, &p.addr);
    }
    floss_link_peer_set(type, addr);
    E.adopted = true;
    return true;
}

/** Encrypted: identify the peer (bond identity if SMP knows it, else the connection address). */
static bool identify(void) {
    const floss_smp_bond_t *b = floss_smp_resolve(E.conn_type, &E.conn_addr);
    if (b) {
        return check_peer(b->addr_type, &b->addr);
    }
    return check_peer((uint8_t)(E.conn_type & 0x01u), &E.conn_addr);
}

/* ------------------------------------------------------------------------ */
/* Pad                                                                      */
/* ------------------------------------------------------------------------ */

static void pad_apply_adv(bool pairing) {
    const char *name = cfg()->name ? cfg()->name : "FLOSS Pad";
    floss_le_ad_t ad, sr;
    floss_le_ad_init(&ad);
    floss_le_ad_flags(&ad, (uint8_t)((pairing ? FLOSS_AD_FLAG_GENERAL_DISC : 0) | FLOSS_AD_FLAG_NO_BREDR));
    floss_le_ad_add(&ad, FLOSS_AD_UUID128_ALL, s_uuid_svc_le, 16);
    if (FLOSS_LE_ADV_DATA_MAX - ad.len >= 3) {
        floss_le_ad_name(&ad, name); /* shortened if tight; the scan response has it all */
    }
    floss_le_ad_init(&sr);
    floss_le_ad_name(&sr, name);
    floss_le_adv_set_data(ad.data, ad.len);
    floss_le_adv_set_scan_rsp(sr.data, sr.len);
}

static void pad_rest(void) {
    floss_link_state_t st = floss_link_rest_state();
    floss_link_set_state(st);
    pad_apply_adv(st == FLOSS_LINK_STATE_PAIRING);
    if (st == FLOSS_LINK_STATE_IDLE) {
        floss_le_adv_stop();
    } else {
        floss_le_adv_start();
    }
}

/** Per poll: CONNECTED = encrypted and subscribed. A few loads. */
static void pad_poll(void) {
    if (E.handle == LE_NO_HANDLE || !E.secured || E.idx < 0) {
        return;
    }
    bool sub = (s_in_cccd.v[E.idx] & FLOSS_GATT_CCCD_NOTIFY) != 0;
    bool up = floss_link_session_active();
    if (sub && !up) {
        floss_link_session_up(&E.conn_addr);
    } else if (!sub && up) {
        floss_link_session_down(FLOSS_HCI_SUCCESS); /* unsubscribed; the link stays */
    }
}

static uint8_t le_output_write(uint16_t conn, uint16_t attr_handle, const floss_gatt_attr_t *attr, uint16_t offset,
                               const uint8_t *data, uint16_t len, uint8_t flags) {
    (void)attr_handle;
    (void)attr;
    (void)flags;
    if (offset != 0) {
        return FLOSS_ATT_ERR_INVALID_OFFSET;
    }
    if (conn == E.handle && E.secured) {
        floss_link_rx_pdu(data, len); /* malformed PDUs are counted there, never an ATT error */
    }
    return 0;
}

/* ------------------------------------------------------------------------ */
/* Dongle                                                                   */
/* ------------------------------------------------------------------------ */

static void dongle_search(void);

static void retry_later(void) {
    floss_timer_start(&E.retry_timer, floss_link_backoff_us(E.attempts));
    if (E.attempts < 16) {
        E.attempts++;
    }
}

static void dongle_connect(uint8_t type, const floss_addr_t *addr) {
    floss_le_connect_params_t p;
    floss_le_connect_params_defaults(&p);
    p.conn.interval_min = cfg()->le_interval_min;
    p.conn.interval_max = cfg()->le_interval_max;
    p.conn.latency = cfg()->le_peripheral_latency;
    p.conn.timeout = cfg()->le_supervision_timeout;
    E.target_type = type;
    E.target = *addr;
    floss_err_t err = floss_le_connect(type, addr, &p);
    if (err == FLOSS_OK) {
        E.connecting = true;
        return;
    }
    FLOSS_LOGD(LINK, "LE connect not started: %s", floss_err_str(err));
    retry_later();
}

static bool name_matches(const uint8_t *data, uint8_t len, bool *decided) {
    const char *want = cfg()->name;
    *decided = true;
    if (!want) {
        return true;
    }
    size_t wlen = strlen(want);
    uint8_t n = 0;
    const uint8_t *name = floss_le_ad_find(data, len, FLOSS_AD_NAME_COMPLETE, &n);
    if (name) {
        return n >= wlen && memcmp(name, want, wlen) == 0;
    }
    name = floss_le_ad_find(data, len, FLOSS_AD_NAME_SHORT, &n);
    if (name && n >= wlen) {
        return memcmp(name, want, wlen) == 0;
    }
    if (name && memcmp(name, want, n) != 0) {
        return false; /* even the shortened name differs */
    }
    *decided = false; /* wait for the scan response */
    return false;
}

static void scan_report(const floss_le_adv_report_t *r, void *user) {
    (void)user;
    if (floss_link_state() != FLOSS_LINK_STATE_PAIRING || E.connecting || E.handle != LE_NO_HANDLE) {
        return;
    }
    if (r->evt_type == 1 || r->evt_type == 3) {
        return; /* directed or non-connectable */
    }
    bool same = E.cand.valid && E.cand.type == r->addr_type && floss_addr_eq(&E.cand.addr, &r->addr);
    if (!same) {
        if (r->evt_type == 4) {
            return; /* scan response of someone we are not considering */
        }
        memset(&E.cand, 0, sizeof(E.cand));
        E.cand.valid = true;
        E.cand.type = r->addr_type;
        E.cand.addr = r->addr;
    }
    if (r->evt_type != 4) {
        E.cand.uuid = floss_le_ad_has_uuid128(r->data, r->len, floss_link_uuid_service);
        uint8_t n = 0;
        const uint8_t *flags = floss_le_ad_find(r->data, r->len, FLOSS_AD_FLAGS, &n);
        E.cand.discoverable =
            flags && n >= 1 && (flags[0] & (FLOSS_AD_FLAG_GENERAL_DISC | FLOSS_AD_FLAG_LIMITED_DISC)) != 0;
    }
    bool decided;
    bool ok = name_matches(r->data, r->len, &decided);
    if (decided) {
        E.cand.name_ok = ok;
    }
    if (E.cand.uuid && E.cand.discoverable && E.cand.name_ok) {
        char s[18];
        floss_addr_to_str(&E.cand.addr, s);
        FLOSS_LOGI(LINK, "found pad %s (rssi %d)", s, r->rssi);
        dongle_connect(E.cand.type, &E.cand.addr); /* stops the scan */
    }
}

static void dongle_search(void) {
    floss_link_set_state(FLOSS_LINK_STATE_PAIRING);
    memset(&E.cand, 0, sizeof(E.cand));
    if (E.connecting || E.handle != LE_NO_HANDLE || floss_le_scanning()) {
        return;
    }
    floss_le_scan_params_t sp;
    floss_le_scan_params_defaults(&sp);
    sp.active = true;            /* the full name is in the scan response */
    sp.filter_duplicates = false;
    floss_err_t err = floss_le_scan_start(&sp, scan_report, NULL);
    if (err != FLOSS_OK) {
        FLOSS_LOGD(LINK, "scan not started: %s", floss_err_str(err));
        retry_later();
    }
}

static void dongle_rest(void) {
    floss_link_state_t st = floss_link_rest_state();
    if (st == FLOSS_LINK_STATE_PAIRING) {
        dongle_search();
        return;
    }
    if (floss_le_scanning()) {
        floss_le_scan_stop();
    }
    floss_link_set_state(st);
    floss_link_peer_t p;
    if (st == FLOSS_LINK_STATE_CONNECTING && floss_link_peer_get(&p) && !E.connecting && E.handle == LE_NO_HANDLE) {
        char s[18];
        floss_addr_to_str(&p.addr, s);
        FLOSS_LOGI(LINK, "reconnecting to pad %s", s);
        dongle_connect(p.addr_type, &p.addr);
    }
}

static void dongle_after_failure(void) {
    if (floss_link_state() == FLOSS_LINK_STATE_PAIRING) {
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
        retry_later();
    }
}

static void retry_fired(floss_timer_t *t, void *arg) {
    (void)t;
    (void)arg;
    if (E.connecting || E.handle != LE_NO_HANDLE) {
        return;
    }
    if (floss_link_state() == FLOSS_LINK_STATE_PAIRING) {
        dongle_search();
    } else {
        dongle_rest();
    }
}

static void dongle_fail(const char *what, uint8_t err) {
    FLOSS_LOGW(LINK, "pad setup failed: %s (0x%02X)", what, err);
    drop_link();
}

static void sub_done(uint16_t conn, uint8_t att_err, void *user) {
    (void)user;
    if (conn != E.handle || E.step != STEP_SUBSCRIBE) {
        return;
    }
    if (att_err) {
        dongle_fail("subscribe", att_err);
        return;
    }
    E.step = STEP_READY;
    E.attempts = 0;
    floss_link_session_up(&E.conn_addr);
}

static void output_found(uint16_t conn, uint8_t att_err, uint8_t props, uint16_t vh, uint16_t cccd, void *user) {
    (void)props;
    (void)cccd;
    (void)user;
    if (conn != E.handle || E.step != STEP_FIND_OUTPUT) {
        return;
    }
    if (att_err || vh == 0) {
        dongle_fail("no output characteristic", att_err);
        return;
    }
    E.d_out_vh = vh;
    E.step = STEP_SUBSCRIBE;
    floss_err_t err = floss_gattc_subscribe(conn, E.d_in_cccd, FLOSS_GATT_CCCD_NOTIFY, sub_done, NULL);
    if (err != FLOSS_OK) {
        dongle_fail("subscribe", 0);
    }
}

static void input_found(uint16_t conn, uint8_t att_err, uint8_t props, uint16_t vh, uint16_t cccd, void *user) {
    (void)props;
    (void)user;
    if (conn != E.handle || E.step != STEP_FIND_INPUT) {
        return;
    }
    if (att_err || vh == 0 || cccd == 0) {
        dongle_fail("no input characteristic", att_err);
        return;
    }
    E.d_in_vh = vh;
    E.d_in_cccd = cccd;
    E.step = STEP_FIND_OUTPUT;
    if (floss_gattc_find_char128(conn, floss_link_uuid_output, output_found, NULL) != FLOSS_OK) {
        dongle_fail("discovery", 0);
    }
}

static void mtu_done(uint16_t conn, uint8_t att_err, void *user) {
    (void)user;
    if (conn != E.handle || E.step != STEP_MTU) {
        return;
    }
    if (att_err) {
        FLOSS_LOGW(LINK, "MTU exchange failed (0x%02X): reports over 11 bytes will not fit", att_err);
    }
    if (cfg()->le_2m_phy && floss_le_local_feature(FLOSS_LE_FEAT_2M_PHY)) {
        floss_le_set_phy(conn, 0x02, 0x02); /* shorter air time; result is FLOSS_LE_EVT_PHY */
    }
    E.step = STEP_FIND_INPUT;
    if (floss_gattc_find_char128(conn, floss_link_uuid_input, input_found, NULL) != FLOSS_OK) {
        dongle_fail("discovery", 0);
    }
}

/** Encrypted link to the pad: discover and subscribe. */
static void dongle_secured(void) {
    E.step = STEP_MTU;
    if (floss_gattc_exchange_mtu(E.handle, mtu_done, NULL) != FLOSS_OK) {
        dongle_fail("MTU exchange", 0);
    }
}

static void dongle_notify(uint16_t conn, uint16_t handle, const uint8_t *data, uint16_t len, bool indication,
                          void *user) {
    (void)user;
    if (conn == E.handle && handle == E.d_in_vh && E.d_in_vh != 0 && !indication) {
        floss_link_rx_pdu(data, len);
    }
}

/* ------------------------------------------------------------------------ */
/* Events                                                                   */
/* ------------------------------------------------------------------------ */

static void le_event(const floss_le_evt_t *evt, void *user) {
    (void)user;
    if (!floss_link_cfg() || floss_link_cfg()->transport != FLOSS_LINK_LE) {
        return;
    }
    switch (evt->type) {
    case FLOSS_LE_EVT_CONNECTED:
        if (is_pad()) {
            if (evt->role != 1) {
                return;
            }
            if (E.handle != LE_NO_HANDLE) {
                floss_le_disconnect(evt->handle, FLOSS_HCI_ERR_REJECTED_LIMITED_RESOURCES);
                return;
            }
            link_up(evt);
            return;
        }
        if (evt->role == 0 && E.connecting && floss_addr_eq(&evt->addr, &E.target)) {
            E.connecting = false;
            floss_timer_stop(&E.retry_timer);
            link_up(evt);
            E.step = STEP_SECURE;
            if (floss_smp_request_security(evt->handle) != FLOSS_OK) {
                dongle_fail("security", 0);
            }
        }
        return;
    case FLOSS_LE_EVT_CONNECT_FAILED:
        if (!is_pad() && E.connecting) {
            E.connecting = false;
            FLOSS_LOGI(LINK, "LE connect failed: %s", floss_hci_status_str(evt->status));
            dongle_after_failure();
        }
        return;
    case FLOSS_LE_EVT_DISCONNECTED:
        if (E.handle == LE_NO_HANDLE || evt->handle != E.handle) {
            return;
        }
        reset_link();
        floss_link_session_down(evt->status);
        if (is_pad()) {
            if (floss_link_state() == FLOSS_LINK_STATE_PAIRING) {
                pad_apply_adv(true); /* advertising restarts by itself */
            } else {
                pad_rest();
            }
        } else {
            dongle_after_failure();
        }
        return;
    case FLOSS_LE_EVT_SECURITY:
        if (evt->handle != E.handle || E.handle == LE_NO_HANDLE || evt->level < 2 || E.secured) {
            return;
        }
        if (!identify()) {
            drop_link();
            return;
        }
        E.secured = true;
        floss_timer_stop(&E.secure_timer);
        if (!is_pad() && E.step == STEP_SECURE) {
            dongle_secured();
        }
        return;
    default:
        return;
    }
}

static void smp_event(const floss_smp_evt_t *evt, void *user) {
    (void)user;
    if (!floss_link_cfg() || floss_link_cfg()->transport != FLOSS_LINK_LE || evt->handle != E.handle ||
        E.handle == LE_NO_HANDLE) {
        return;
    }
    if (evt->type == FLOSS_SMP_EVT_PAIRING_COMPLETE && evt->bonded && E.adopted) {
        floss_link_peer_set(evt->addr_type, &evt->addr); /* now we know its identity address */
    } else if (evt->type == FLOSS_SMP_EVT_PAIRING_FAILED) {
        FLOSS_LOGW(LINK, "LE pairing failed (reason 0x%02X)", evt->reason);
        drop_link();
    }
}

/* ------------------------------------------------------------------------ */
/* Transport operations                                                     */
/* ------------------------------------------------------------------------ */

static floss_err_t le_init(void) {
    memset(&E, 0, sizeof(E));
    reset_link();
    floss_timer_init(&E.secure_timer, secure_timeout, NULL);
    floss_timer_init(&E.retry_timer, retry_fired, NULL);
    reverse16(s_uuid_svc_le, floss_link_uuid_service);
    reverse16(s_uuid_in_le, floss_link_uuid_input);
    reverse16(s_uuid_out_le, floss_link_uuid_output);
    memset(&s_in_cccd, 0, sizeof(s_in_cccd));

    floss_le_link_policy_t pol;
    floss_le_link_policy_defaults(&pol);
    pol.preferred.interval_min = cfg()->le_interval_min;
    pol.preferred.interval_max = cfg()->le_interval_max;
    pol.preferred.latency = cfg()->le_peripheral_latency;
    pol.preferred.timeout = cfg()->le_supervision_timeout;
    /* The dongle asks for 2M itself once the link is set up. */
    pol.prefer_2m_phy = is_pad() && cfg()->le_2m_phy;
    floss_err_t err = floss_le_set_link_policy(&pol);

    if (err == FLOSS_OK) {
        err = floss_le_add_listener(le_event, NULL);
    }
    if (err == FLOSS_OK && floss_smp_add_listener(smp_event, NULL) != FLOSS_OK) {
        FLOSS_LOGW(LINK, "no SMP listener slot: peer identity from connection addresses only");
    }
    if (err != FLOSS_OK) {
        return err;
    }
    if (is_pad()) {
        uint16_t base = floss_gatt_register(s_link_service, (uint16_t)FLOSS_ARRAY_SIZE(s_link_service));
        if (base == 0) {
            return FLOSS_ERR_NOMEM;
        }
        E.in_vh = (uint16_t)(base + 2);
        floss_le_set_name(cfg()->name ? cfg()->name : "FLOSS Pad");
        floss_le_set_appearance(FLOSS_LE_APPEARANCE_GAMEPAD);
        pad_apply_adv(false);
    } else {
        floss_gattc_set_notify_handler(dongle_notify, NULL);
    }
    return FLOSS_OK;
}

static void le_on_ready(void) {
    floss_timer_stop(&E.retry_timer);
    reset_link();
    E.connecting = false;
    E.attempts = 0;
    if (is_pad()) {
        pad_rest();
    } else {
        floss_gattc_set_notify_handler(dongle_notify, NULL);
        /* LE finishes its own bring-up right after GAP READY: start on the
         * next timer pass (retry_fired -> dongle_rest). */
        floss_timer_start(&E.retry_timer, 0);
    }
}

static floss_err_t le_pair(void) {
    floss_link_set_state(FLOSS_LINK_STATE_PAIRING);
    if (is_pad()) {
        pad_apply_adv(true);
        floss_le_adv_start();
        drop_link();
        return FLOSS_OK;
    }
    E.attempts = 0;
    floss_timer_stop(&E.retry_timer);
    if (E.handle != LE_NO_HANDLE) {
        drop_link(); /* DISCONNECTED sees PAIRING and scans */
    } else if (E.connecting) {
        floss_le_connect_cancel(); /* CONNECT_FAILED sees PAIRING and scans */
    } else {
        dongle_search();
    }
    return FLOSS_OK;
}

static void le_on_pair_timeout(void) {
    if (is_pad()) {
        pad_rest();
        return;
    }
    if (E.connecting) {
        floss_le_connect_cancel();
        E.connecting = false;
    }
    dongle_rest();
}

static void le_forget(const floss_link_peer_t *old) {
    if (old) {
        floss_smp_bond_delete(old->addr_type, &old->addr);
    }
    le_pair(); /* no peer: pairing is the rest state */
}

static bool le_can_send(void) {
    return E.handle != LE_NO_HANDLE && floss_hci_acl_can_send_now(E.handle) && floss_link_inflight_ok(E.handle);
}

static floss_buf_t *le_input_alloc(uint8_t **pdu, uint16_t *cap) {
    if (E.handle == LE_NO_HANDLE) {
        return NULL;
    }
    return floss_gatt_notify_alloc(E.handle, E.in_vh, pdu, cap);
}

static floss_err_t le_input_send(floss_buf_t *b, uint16_t len) {
    return floss_gatt_notify_send(b, len, FLOSS_BUF_F_PRIORITY);
}

static floss_err_t le_send_ctrl(const uint8_t *pdu, uint16_t len) {
    if (E.handle == LE_NO_HANDLE) {
        return FLOSS_ERR_STATE;
    }
    if (is_pad()) {
        return floss_gatt_notify(E.handle, E.in_vh, pdu, len);
    }
    if (E.d_out_vh == 0) {
        return FLOSS_ERR_STATE;
    }
    return floss_gattc_write(E.handle, E.d_out_vh, pdu, len, false, NULL, NULL);
}

static uint32_t le_conn_period_us(void) {
    const floss_le_link_t *l = E.handle != LE_NO_HANDLE ? floss_le_link(E.handle) : NULL;
    return (l && l->interval) ? (uint32_t)l->interval * 1250u : 0u;
}

static void le_poll(void) {
    if (is_pad()) {
        pad_poll();
    }
}

const floss_link_tp_t floss_link_le = {
    .name = "LE",
    .init = le_init,
    .on_ready = le_on_ready,
    .on_gap_event = NULL,
    .pair = le_pair,
    .on_pair_timeout = le_on_pair_timeout,
    .forget = le_forget,
    .can_send = le_can_send,
    .input_alloc = le_input_alloc,
    .input_send = le_input_send,
    .send_ctrl = le_send_ctrl,
    .conn_period_us = le_conn_period_us,
    .on_input_change = NULL,
    .on_peer_hello = NULL,
    .poll = le_poll,
};

#elif FLOSS_ENABLE_LINK

/* LE disabled: floss_link_init() refuses the LE transport before using this. */
static floss_err_t le_unsupported(void) {
    return FLOSS_ERR_UNSUPPORTED;
}
const floss_link_tp_t floss_link_le = {
    .name = "LE (disabled)",
    .init = le_unsupported,
};

#endif
