/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_le.c - LE GAP: advertising, LE link table, connection parameter
 * policy, LE signaling channel and fixed-channel routing.
 *
 * Overview
 * --------
 * HCI owns the controller and the generic connection table; this module owns
 * everything that is specific to LE links:
 *
 *  - ADVERTISING. Legacy connectable undirected advertising (ADV_IND). The
 *    app's settings are kept here and (re)applied whenever the controller is
 *    ready, so the API can be used before floss_start(). Because the HCI
 *    command queue is FIFO, a reconfiguration is simply "disable, set
 *    address/params/data, enable" queued back to back - no state machine.
 *    The controller stops legacy advertising by itself when a central
 *    connects; we restart it after the link drops (auto_restart).
 *
 *  - LINKS. One le_slot_t per LE link (index = floss_le_link_index), holding
 *    the public floss_le_link_t snapshot (interval, MTU, security, PHY...)
 *    plus parameter-update bookkeeping. ATT, GATT CCCDs and HOGP index their
 *    own per-link tables with the same index.
 *
 *  - LATENCY POLICY. A gamepad wants a short connection interval and no
 *    peripheral latency, but centrals usually connect with 30-50 ms. As a
 *    peripheral we wait ~1 s (centrals dislike parameter changes in the
 *    middle of service discovery), then ask for the preferred parameters:
 *    with the LL Connection Parameters Request procedure when our controller
 *    supports it (LE Connection Update), else - or if the peer's controller
 *    does not - with the L2CAP Connection Parameter Update Request on CID 5.
 *    2M PHY and 251-byte data length are requested right after connecting.
 *
 *  - ROUTING. LE L2CAP has no channel setup: PDUs arrive on fixed CIDs and
 *    are dispatched here (ATT 4, signaling 5, SMP 6).
 */
#include "floss_internal.h"
#include "floss/floss_le.h"
#include "floss/floss_gatt.h"
#include "floss/floss_hogp.h"

#if FLOSS_ENABLE_LE

/* HCI reason/status codes used locally */
#define LE_ERR_UNSUPPORTED_REMOTE_FEATURE 0x1A
#define LE_ERR_UNACCEPTABLE_CONN_PARAMS   0x3B
#define LE_ERR_ADV_TIMEOUT                0x3C

/* Parameter update progress */
enum { PARAM_IDLE = 0, PARAM_HCI_PENDING, PARAM_L2CAP_PENDING };

typedef struct {
    bool used;
    floss_le_link_t info;
    uint8_t sig_id;            /* identifier of our last LE signaling request */
    uint8_t param_state;       /* PARAM_* */
    floss_le_conn_params_t param_req; /* what we asked for */
    floss_timer_t update_timer;
} le_slot_t;

typedef struct {
    floss_le_evt_fn fn;
    void *user;
} le_listener_t;

static struct {
    bool ready;                     /* controller up, commands may be sent */

    /* identity */
    char name[FLOSS_LE_NAME_MAX + 1];
    uint16_t appearance;
    floss_addr_t static_addr;
    bool static_addr_valid;

    /* advertising */
    floss_le_adv_params_t adv;
    uint8_t adv_data[FLOSS_LE_ADV_DATA_MAX];
    uint8_t adv_len;
    uint8_t scan_rsp[FLOSS_LE_ADV_DATA_MAX];
    uint8_t scan_rsp_len;
    bool adv_data_set;              /* app provided advertising data */
    bool adv_wanted;                /* app wants us to advertise */
    bool adv_target;                /* last enable value we sent */
    bool adv_on;                    /* controller confirmed */

    floss_le_link_policy_t policy;
    le_listener_t listeners[FLOSS_LE_MAX_LISTENERS];
    le_slot_t links[FLOSS_MAX_CONNECTIONS];
} s_le;

/* ------------------------------------------------------------------------ */
/* Helpers                                                                  */
/* ------------------------------------------------------------------------ */

static le_slot_t *slot_by_handle(uint16_t handle) {
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        if (s_le.links[i].used && s_le.links[i].info.handle == handle) {
            return &s_le.links[i];
        }
    }
    return NULL;
}

void floss_le_emit(const floss_le_evt_t *evt) {
    for (uint8_t i = 0; i < FLOSS_LE_MAX_LISTENERS; i++) {
        if (s_le.listeners[i].fn) {
            s_le.listeners[i].fn(evt, s_le.listeners[i].user);
        }
    }
}

static void emit_link_evt(floss_le_evt_type_t type, const le_slot_t *s, uint8_t status) {
    floss_le_evt_t e;
    memset(&e, 0, sizeof(e));
    e.type = type;
    e.status = status;
    if (s) {
        e.handle = s->info.handle;
        e.addr = s->info.peer;
        e.addr_type = s->info.peer_addr_type;
        e.role = s->info.role;
        e.level = s->info.security_level;
        e.interval = s->info.interval;
        e.latency = s->info.latency;
        e.timeout = s->info.timeout;
        e.mtu = s->info.att_mtu;
        e.tx_octets = s->info.tx_octets;
        e.tx_phy = s->info.tx_phy;
        e.rx_phy = s->info.rx_phy;
    }
    floss_le_emit(&e);
}

bool floss_le_local_feature(uint8_t bit) {
    return (floss_hci_controller()->le_features[bit >> 3] >> (bit & 7)) & 1u;
}

static void log_cmd_status(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len) {
    if (status) {
        FLOSS_LOGW(GAP, "le: cmd 0x%04x failed: %s", (unsigned)opcode, floss_hci_status_str(status));
    }
}

/* ------------------------------------------------------------------------ */
/* Identity                                                                 */
/* ------------------------------------------------------------------------ */

floss_err_t floss_le_set_name(const char *name) {
    if (name == NULL) {
        return FLOSS_ERR_INVALID;
    }
    size_t n = strlen(name);
    if (n > FLOSS_LE_NAME_MAX) {
        n = FLOSS_LE_NAME_MAX;
    }
    memcpy(s_le.name, name, n);
    s_le.name[n] = '\0';
    return FLOSS_OK;
}

const char *floss_le_name(void) {
    return s_le.name;
}

void floss_le_set_appearance(uint16_t appearance) {
    s_le.appearance = appearance;
}

uint16_t floss_le_appearance(void) {
    return s_le.appearance;
}

/** Random static address: two MSBs set, the other 46 bits neither all 0 nor all 1. */
static bool static_addr_ok(const floss_addr_t *a) {
    if ((a->b[5] & 0xC0) != 0xC0) {
        return false;
    }
    bool zeros = (a->b[5] & 0x3F) == 0;
    bool ones = (a->b[5] & 0x3F) == 0x3F;
    for (uint8_t i = 0; i < 5; i++) {
        zeros = zeros && a->b[i] == 0x00;
        ones = ones && a->b[i] == 0xFF;
    }
    return !zeros && !ones;
}

floss_err_t floss_le_set_static_address(const floss_addr_t *addr) {
    if (addr && !static_addr_ok(addr)) {
        return FLOSS_ERR_INVALID;
    }
    if (addr) {
        s_le.static_addr = *addr;
        s_le.static_addr_valid = true;
    } else {
        s_le.static_addr_valid = false;
    }
    return FLOSS_OK;
}

/**
 * The static address must not change between power cycles, or every bonded
 * host would see a new device: load it from storage, generate it only once.
 */
static void ensure_static_addr(void) {
    if (s_le.static_addr_valid) {
        return;
    }
    floss_addr_t a;
    if (floss_hal_store_read(FLOSS_LE_STORE_KEY_STATIC_ADDR, a.b, 6) == 6 && static_addr_ok(&a)) {
        s_le.static_addr = a;
    } else {
        do {
            floss_hal_random(a.b, 6);
            a.b[5] |= 0xC0;
        } while (!static_addr_ok(&a));
        s_le.static_addr = a;
        floss_hal_store_write(FLOSS_LE_STORE_KEY_STATIC_ADDR, a.b, 6);
    }
    s_le.static_addr_valid = true;
    char str[18];
    floss_addr_to_str(&s_le.static_addr, str);
    FLOSS_LOGI(GAP, "le: static random address %s", str);
}

void floss_le_own_address(floss_addr_t *addr, uint8_t *addr_type) {
    if (s_le.adv.own_addr_type == FLOSS_LE_ADDR_RANDOM) {
        ensure_static_addr();
        *addr = s_le.static_addr;
        *addr_type = FLOSS_LE_ADDR_RANDOM;
    } else {
        *addr = floss_hci_controller()->addr;
        *addr_type = FLOSS_LE_ADDR_PUBLIC;
    }
}

/* ------------------------------------------------------------------------ */
/* Advertising data builder                                                 */
/* ------------------------------------------------------------------------ */

void floss_le_ad_init(floss_le_ad_t *ad) {
    memset(ad, 0, sizeof(*ad));
}

bool floss_le_ad_add(floss_le_ad_t *ad, uint8_t type, const uint8_t *data, uint8_t len) {
    if ((uint16_t)ad->len + 2 + len > FLOSS_LE_ADV_DATA_MAX) {
        ad->overflow = true;
        return false;
    }
    ad->data[ad->len++] = (uint8_t)(len + 1);
    ad->data[ad->len++] = type;
    if (len) {
        memcpy(&ad->data[ad->len], data, len);
    }
    ad->len = (uint8_t)(ad->len + len);
    return true;
}

bool floss_le_ad_flags(floss_le_ad_t *ad, uint8_t flags) {
    return floss_le_ad_add(ad, FLOSS_AD_FLAGS, &flags, 1);
}

bool floss_le_ad_name(floss_le_ad_t *ad, const char *name) {
    size_t n = strlen(name);
    uint8_t room = (uint8_t)(FLOSS_LE_ADV_DATA_MAX - ad->len);
    if (room < 3) {
        ad->overflow = true;
        return false;
    }
    if (n + 2 <= room) {
        return floss_le_ad_add(ad, FLOSS_AD_NAME_COMPLETE, (const uint8_t *)name, (uint8_t)n);
    }
    /* Shortened Local Name: the scanner learns the full one from GAP/scan response. */
    return floss_le_ad_add(ad, FLOSS_AD_NAME_SHORT, (const uint8_t *)name, (uint8_t)(room - 2));
}

bool floss_le_ad_uuid16(floss_le_ad_t *ad, const uint16_t *uuids, uint8_t count) {
    uint8_t tmp[FLOSS_LE_ADV_DATA_MAX];
    if (count > (FLOSS_LE_ADV_DATA_MAX - 2) / 2) {
        ad->overflow = true;
        return false;
    }
    for (uint8_t i = 0; i < count; i++) {
        floss_put_le16(&tmp[2 * i], uuids[i]);
    }
    return floss_le_ad_add(ad, FLOSS_AD_UUID16_ALL, tmp, (uint8_t)(2 * count));
}

bool floss_le_ad_appearance(floss_le_ad_t *ad, uint16_t appearance) {
    uint8_t v[2];
    floss_put_le16(v, appearance);
    return floss_le_ad_add(ad, FLOSS_AD_APPEARANCE, v, 2);
}

bool floss_le_ad_tx_power(floss_le_ad_t *ad, int8_t dbm) {
    uint8_t v = (uint8_t)dbm;
    return floss_le_ad_add(ad, FLOSS_AD_TX_POWER, &v, 1);
}

/* ------------------------------------------------------------------------ */
/* Advertising                                                              */
/* ------------------------------------------------------------------------ */

void floss_le_adv_params_defaults(floss_le_adv_params_t *p) {
    memset(p, 0, sizeof(*p));
    /* 30-60 ms: hosts find the device within a few hundred ms without
     * burning much power. Apple recommends 20 ms for the first 30 s. */
    p->interval_min = 0x0030;
    p->interval_max = 0x0060;
    p->own_addr_type = FLOSS_LE_ADDR_PUBLIC;
    p->channel_map = 0x07;
    p->filter_policy = 0;
    p->auto_restart = true;
    p->while_connected = false;
}

static void adv_enable_done(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len) {
    bool enable = ctx != NULL;
    if (status) {
        FLOSS_LOGW(GAP, "le: advertising %s failed: %s", enable ? "enable" : "disable",
                   floss_hci_status_str(status));
        s_le.adv_target = s_le.adv_on;
        return;
    }
    if (s_le.adv_on != enable) {
        s_le.adv_on = enable;
        FLOSS_LOGI(GAP, "le: advertising %s", enable ? "on" : "off");
        emit_link_evt(enable ? FLOSS_LE_EVT_ADV_STARTED : FLOSS_LE_EVT_ADV_STOPPED, NULL, 0);
    }
}

static void adv_send_enable(bool enable) {
    uint8_t v = enable ? 1 : 0;
    s_le.adv_target = enable;
    floss_hci_cmd(FLOSS_HCI_LE_SET_ADV_ENABLE, &v, 1, adv_enable_done, enable ? (void *)1 : NULL);
}

static void adv_send_data(void) {
    uint8_t p[1 + FLOSS_LE_ADV_DATA_MAX] = {0};
    p[0] = s_le.adv_len;
    memcpy(&p[1], s_le.adv_data, s_le.adv_len);
    floss_hci_cmd(FLOSS_HCI_LE_SET_ADV_DATA, p, sizeof(p), log_cmd_status, NULL);
}

static void adv_send_scan_rsp(void) {
    uint8_t p[1 + FLOSS_LE_ADV_DATA_MAX] = {0};
    p[0] = s_le.scan_rsp_len;
    memcpy(&p[1], s_le.scan_rsp, s_le.scan_rsp_len);
    floss_hci_cmd(FLOSS_HCI_LE_SET_SCAN_RSP_DATA, p, sizeof(p), log_cmd_status, NULL);
}

static uint8_t le_link_count(void) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        n = (uint8_t)(n + (s_le.links[i].used ? 1 : 0));
    }
    return n;
}

/** Bring the controller's advertising state in line with what we want. */
static void adv_update(void) {
    if (!s_le.ready) {
        return;
    }
    uint8_t links = le_link_count();
    bool want = s_le.adv_wanted && links < FLOSS_MAX_CONNECTIONS && (links == 0 || s_le.adv.while_connected);
    if (want != s_le.adv_target) {
        adv_send_enable(want);
    }
}

/** Standard payloads (see floss_le_adv_set_standard). False if the UUIDs do not fit. */
static bool build_standard(const uint16_t *uuids16, uint8_t count, floss_le_ad_t *ad, floss_le_ad_t *sr) {
    floss_le_ad_init(ad);
    floss_le_ad_init(sr);
    /* LE only: without "BR/EDR not supported" dual-mode hosts may try to
     * reach the device over Classic with the same address. */
    floss_le_ad_flags(ad, FLOSS_AD_FLAG_GENERAL_DISC | FLOSS_AD_FLAG_NO_BREDR);
    if (s_le.appearance) {
        floss_le_ad_appearance(ad, s_le.appearance);
    }
    if (count) {
        floss_le_ad_uuid16(ad, uuids16, count);
    }
    if (ad->overflow) {
        return false;
    }
    if (s_le.name[0]) {
        if (FLOSS_LE_ADV_DATA_MAX - ad->len >= 3) {
            floss_le_ad_name(ad, s_le.name); /* complete, or shortened if tight */
        }
        floss_le_ad_name(sr, s_le.name);
    }
    return true;
}

/**
 * (Re)apply everything. Address and parameters may only change while
 * advertising is off, so stop first; adv_update() restarts it.
 */
static void adv_reconfigure(void) {
    if (!s_le.ready) {
        return;
    }
    if (s_le.adv_target) {
        adv_send_enable(false);
    }
    if (!s_le.adv_data_set) {
        /* App gave no payload: flags + appearance + name, rebuilt each time
         * so a later floss_le_set_name() is picked up. */
        floss_le_ad_t ad, sr;
        build_standard(NULL, 0, &ad, &sr);
        memcpy(s_le.adv_data, ad.data, ad.len);
        s_le.adv_len = ad.len;
        memcpy(s_le.scan_rsp, sr.data, sr.len);
        s_le.scan_rsp_len = sr.len;
    }
    if (s_le.adv.own_addr_type == FLOSS_LE_ADDR_RANDOM) {
        ensure_static_addr();
        floss_hci_cmd(FLOSS_HCI_LE_SET_RANDOM_ADDRESS, s_le.static_addr.b, 6, log_cmd_status, NULL);
    }
    uint8_t p[15] = {0};
    floss_put_le16(&p[0], s_le.adv.interval_min);
    floss_put_le16(&p[2], s_le.adv.interval_max);
    p[4] = 0x00; /* ADV_IND: connectable, scannable, undirected */
    p[5] = s_le.adv.own_addr_type;
    /* p[6..12]: peer address type/address, unused for undirected */
    p[13] = s_le.adv.channel_map ? s_le.adv.channel_map : 0x07;
    p[14] = s_le.adv.filter_policy;
    floss_hci_cmd(FLOSS_HCI_LE_SET_ADV_PARAMS, p, sizeof(p), log_cmd_status, NULL);
    adv_send_data();
    adv_send_scan_rsp();
    adv_update();
}

floss_err_t floss_le_adv_set_params(const floss_le_adv_params_t *p) {
    if (p == NULL || p->interval_min < 0x20 || p->interval_min > p->interval_max || p->interval_max > 0x4000 ||
        p->own_addr_type > FLOSS_LE_ADDR_RANDOM) {
        return FLOSS_ERR_INVALID;
    }
    s_le.adv = *p;
    adv_reconfigure();
    return FLOSS_OK;
}

floss_err_t floss_le_adv_set_data(const uint8_t *data, uint8_t len) {
    if (len > FLOSS_LE_ADV_DATA_MAX || (len && data == NULL)) {
        return FLOSS_ERR_INVALID;
    }
    if (len) {
        memcpy(s_le.adv_data, data, len);
    }
    s_le.adv_len = len;
    s_le.adv_data_set = true;
    if (s_le.ready) {
        adv_send_data(); /* allowed while advertising */
    }
    return FLOSS_OK;
}

floss_err_t floss_le_adv_set_scan_rsp(const uint8_t *data, uint8_t len) {
    if (len > FLOSS_LE_ADV_DATA_MAX || (len && data == NULL)) {
        return FLOSS_ERR_INVALID;
    }
    if (len) {
        memcpy(s_le.scan_rsp, data, len);
    }
    s_le.scan_rsp_len = len;
    if (s_le.ready) {
        adv_send_scan_rsp();
    }
    return FLOSS_OK;
}

floss_err_t floss_le_adv_set_standard(const uint16_t *uuids16, uint8_t count) {
    floss_le_ad_t ad, sr;
    if (!build_standard(uuids16, count, &ad, &sr)) {
        return FLOSS_ERR_INVALID;
    }
    floss_le_adv_set_data(ad.data, ad.len);
    floss_le_adv_set_scan_rsp(sr.data, sr.len);
    return FLOSS_OK;
}

floss_err_t floss_le_adv_start(void) {
    s_le.adv_wanted = true;
    adv_update();
    return FLOSS_OK;
}

floss_err_t floss_le_adv_stop(void) {
    s_le.adv_wanted = false;
    adv_update();
    return FLOSS_OK;
}

bool floss_le_adv_active(void) {
    return s_le.adv_on;
}

/* ------------------------------------------------------------------------ */
/* Connection parameters                                                    */
/* ------------------------------------------------------------------------ */

void floss_le_link_policy_defaults(floss_le_link_policy_t *p) {
    memset(p, 0, sizeof(*p));
    p->preferred.interval_min = 6;  /* 7.5 ms */
    p->preferred.interval_max = 12; /* 15 ms */
    p->preferred.latency = 0;       /* every event: lowest input latency */
    p->preferred.timeout = 300;     /* 3 s */
    p->update_delay_us = 1000000u;
    p->prefer_2m_phy = true;
    p->max_data_length = true;
}

floss_err_t floss_le_set_link_policy(const floss_le_link_policy_t *p) {
    if (p == NULL) {
        return FLOSS_ERR_INVALID;
    }
    s_le.policy = *p;
    return FLOSS_OK;
}

const floss_le_link_policy_t *floss_le_link_policy(void) {
    return &s_le.policy;
}

/** Spec limits, including "timeout must outlast (1 + latency) intervals twice". */
static bool conn_params_valid(const floss_le_conn_params_t *p) {
    return p->interval_min >= 6 && p->interval_max <= 3200 && p->interval_min <= p->interval_max &&
           p->latency <= 499 && p->timeout >= 10 && p->timeout <= 3200 &&
           (uint32_t)p->timeout * 4u > (uint32_t)(1u + p->latency) * p->interval_max;
}

static void send_hci_conn_update(uint16_t handle, const floss_le_conn_params_t *p, floss_hci_cmd_cb cb) {
    uint8_t c[14] = {0};
    floss_put_le16(&c[0], handle);
    floss_put_le16(&c[2], p->interval_min);
    floss_put_le16(&c[4], p->interval_max);
    floss_put_le16(&c[6], p->latency);
    floss_put_le16(&c[8], p->timeout);
    /* c[10..13]: min/max CE length 0 = controller decides */
    floss_hci_cmd(FLOSS_HCI_LE_CONN_UPDATE, c, sizeof(c), cb, (void *)(uintptr_t)handle);
}

static void send_l2cap_param_req(le_slot_t *s) {
    uint8_t p[12];
    s->sig_id = (uint8_t)(s->sig_id + 1);
    if (s->sig_id == 0) {
        s->sig_id = 1; /* identifier 0 is invalid */
    }
    p[0] = FLOSS_LE_SIG_CONN_PARAM_REQ;
    p[1] = s->sig_id;
    floss_put_le16(&p[2], 8);
    floss_put_le16(&p[4], s->param_req.interval_min);
    floss_put_le16(&p[6], s->param_req.interval_max);
    floss_put_le16(&p[8], s->param_req.latency);
    floss_put_le16(&p[10], s->param_req.timeout);
    s->param_state = PARAM_L2CAP_PENDING;
    FLOSS_TRACE(FLOSS_TR_L2CAP_SIG, ((uint32_t)FLOSS_LE_SIG_CONN_PARAM_REQ << 16) | s->sig_id);
    if (floss_le_send_fixed(s->info.handle, FLOSS_CID_LE_SIGNALING, p, sizeof(p)) != FLOSS_OK) {
        s->param_state = PARAM_IDLE;
        floss_timer_start(&s->update_timer, 100000u); /* no buffer: retry shortly */
    }
}

/** LE Connection Update rejected by our controller: use L2CAP instead (peripheral). */
static void conn_update_status(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len) {
    le_slot_t *s = slot_by_handle((uint16_t)(uintptr_t)ctx);
    if (s == NULL || status == 0) {
        return;
    }
    FLOSS_LOGW(GAP, "le: connection update refused (%s)", floss_hci_status_str(status));
    if (s->param_state == PARAM_HCI_PENDING && s->info.role == 1) {
        send_l2cap_param_req(s);
    } else {
        s->param_state = PARAM_IDLE;
        emit_link_evt(FLOSS_LE_EVT_CONN_PARAMS, s, status);
    }
}

floss_err_t floss_le_request_conn_params(uint16_t handle, const floss_le_conn_params_t *p) {
    le_slot_t *s = slot_by_handle(handle);
    if (s == NULL) {
        return FLOSS_ERR_NOT_FOUND;
    }
    if (p == NULL || !conn_params_valid(p)) {
        return FLOSS_ERR_INVALID;
    }
    s->param_req = *p;
    FLOSS_LOGI(GAP, "le: requesting interval %u-%u latency %u timeout %u", (unsigned)p->interval_min,
               (unsigned)p->interval_max, (unsigned)p->latency, (unsigned)p->timeout);
    if (s->info.role == 0 || floss_le_local_feature(FLOSS_LE_FEAT_CONN_PARAM_REQ)) {
        s->param_state = PARAM_HCI_PENDING;
        send_hci_conn_update(handle, p, conn_update_status);
    } else {
        send_l2cap_param_req(s);
    }
    return FLOSS_OK;
}

/** Fires update_delay_us after connecting (peripheral): apply the latency policy. */
static void update_timer_fired(floss_timer_t *t, void *arg) {
    le_slot_t *s = (le_slot_t *)arg;
    if (!s->used || s->param_state != PARAM_IDLE) {
        return;
    }
    const floss_le_conn_params_t *want = &s_le.policy.preferred;
    if (s->info.interval >= want->interval_min && s->info.interval <= want->interval_max &&
        s->info.latency == want->latency) {
        return; /* the central already picked something we like */
    }
    floss_le_request_conn_params(s->info.handle, want);
}

/* ------------------------------------------------------------------------ */
/* Links                                                                    */
/* ------------------------------------------------------------------------ */

const floss_le_link_t *floss_le_link(uint16_t handle) {
    le_slot_t *s = slot_by_handle(handle);
    return s ? &s->info : NULL;
}

int floss_le_link_index(uint16_t handle) {
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        if (s_le.links[i].used && s_le.links[i].info.handle == handle) {
            return i;
        }
    }
    return -1;
}

uint16_t floss_le_link_handle_at(uint8_t i) {
    if (i >= FLOSS_MAX_CONNECTIONS || !s_le.links[i].used) {
        return 0xFFFF;
    }
    return s_le.links[i].info.handle;
}

uint8_t floss_le_link_count(void) {
    return le_link_count();
}

floss_err_t floss_le_disconnect(uint16_t handle, uint8_t reason) {
    if (slot_by_handle(handle) == NULL) {
        return FLOSS_ERR_NOT_FOUND;
    }
    uint8_t p[3];
    floss_put_le16(p, handle);
    p[2] = reason ? reason : FLOSS_HCI_ERR_REMOTE_USER_TERMINATED;
    return floss_hci_cmd(FLOSS_HCI_DISCONNECT, p, sizeof(p), log_cmd_status, NULL);
}

void floss_le_set_att_mtu(uint16_t handle, uint16_t mtu) {
    le_slot_t *s = slot_by_handle(handle);
    if (s == NULL || s->info.att_mtu == mtu) {
        return;
    }
    s->info.att_mtu = mtu;
    FLOSS_LOGI(GAP, "le: ATT MTU %u", (unsigned)mtu);
    emit_link_evt(FLOSS_LE_EVT_MTU, s, 0);
}

floss_err_t floss_le_add_listener(floss_le_evt_fn fn, void *user) {
    for (uint8_t i = 0; i < FLOSS_LE_MAX_LISTENERS; i++) {
        if (s_le.listeners[i].fn == NULL) {
            s_le.listeners[i].fn = fn;
            s_le.listeners[i].user = user;
            return FLOSS_OK;
        }
    }
    return FLOSS_ERR_NOMEM;
}

/** Ask for 2M PHY and long LL packets: both shorten air time per report. */
static void link_tune(le_slot_t *s) {
    uint16_t h = s->info.handle;
    if (s_le.policy.max_data_length && floss_le_local_feature(FLOSS_LE_FEAT_DATA_LENGTH_EXT)) {
        uint8_t p[6];
        floss_put_le16(&p[0], h);
        floss_put_le16(&p[2], 251);  /* max LL payload */
        floss_put_le16(&p[4], 2120); /* us to send 251 bytes on 1M */
        floss_hci_cmd(FLOSS_HCI_LE_SET_DATA_LENGTH, p, sizeof(p), log_cmd_status, NULL);
    }
    if (s_le.policy.prefer_2m_phy && floss_le_local_feature(FLOSS_LE_FEAT_2M_PHY)) {
        uint8_t p[7] = {0};
        floss_put_le16(&p[0], h);
        p[2] = 0x00; /* we have TX and RX preferences */
        p[3] = 0x02; /* TX: 2M */
        p[4] = 0x02; /* RX: 2M */
        /* p[5..6]: coded PHY options, none */
        floss_hci_cmd(FLOSS_HCI_LE_SET_PHY, p, sizeof(p), log_cmd_status, NULL);
    }
}

/**
 * LE Connection Complete (legacy layout, 18 bytes) and Enhanced Connection
 * Complete (30 bytes: adds local and peer resolvable private addresses).
 */
static void on_conn_complete(const uint8_t *p, uint8_t len, bool enhanced) {
    if (len < (enhanced ? 30 : 18)) {
        return;
    }
    uint8_t status = p[0];
    uint16_t handle = (uint16_t)(floss_get_le16(&p[1]) & 0x0FFF);
    uint8_t role = p[3];
    const uint8_t *tail = &p[enhanced ? 23 : 11];

    if (role == 1 || status == LE_ERR_ADV_TIMEOUT) {
        /* Legacy advertising ends when a link is created from it. */
        bool was_on = s_le.adv_on;
        s_le.adv_on = false;
        s_le.adv_target = false;
        if (was_on) {
            emit_link_evt(FLOSS_LE_EVT_ADV_STOPPED, NULL, status);
        }
        if (status == 0 && !s_le.adv.auto_restart) {
            s_le.adv_wanted = false;
        }
    }
    if (status) {
        FLOSS_LOGW(GAP, "le: connection failed: %s", floss_hci_status_str(status));
        emit_link_evt(FLOSS_LE_EVT_CONNECT_FAILED, NULL, status);
        adv_update();
        return;
    }

    le_slot_t *s = NULL;
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        if (!s_le.links[i].used) {
            s = &s_le.links[i];
            break;
        }
    }
    if (s == NULL) {
        FLOSS_LOGE(GAP, "le: no slot for link 0x%04x", (unsigned)handle);
        uint8_t d[3];
        floss_put_le16(d, handle);
        d[2] = FLOSS_HCI_ERR_REJECTED_LIMITED_RESOURCES;
        floss_hci_cmd(FLOSS_HCI_DISCONNECT, d, sizeof(d), NULL, NULL);
        return;
    }
    memset(s, 0, sizeof(*s));
    s->used = true;
    s->info.handle = handle;
    s->info.role = role;
    s->info.peer_addr_type = p[4];
    memcpy(s->info.peer.b, &p[5], 6);
    s->info.interval = floss_get_le16(&tail[0]);
    s->info.latency = floss_get_le16(&tail[2]);
    s->info.timeout = floss_get_le16(&tail[4]);
    s->info.att_mtu = FLOSS_ATT_DEFAULT_MTU;
    s->info.security_level = 1;
    s->info.tx_octets = 27;
    s->info.tx_phy = s->info.rx_phy = 1;
    floss_timer_init(&s->update_timer, update_timer_fired, s);

    char str[18];
    floss_addr_to_str(&s->info.peer, str);
    FLOSS_LOGI(GAP, "le: connected 0x%04x %s as %s, interval %u.%02u ms", (unsigned)handle, str,
               role ? "peripheral" : "central", (unsigned)(s->info.interval * 125u / 100u),
               (unsigned)(s->info.interval * 125u % 100u));

    floss_att_on_link_up(handle);
    floss_smp_on_link_up(handle, role, s->info.peer_addr_type, &s->info.peer);
    emit_link_evt(FLOSS_LE_EVT_CONNECTED, s, 0);

    link_tune(s);
    if (role == 1 && s_le.policy.update_delay_us) {
        floss_timer_start(&s->update_timer, s_le.policy.update_delay_us);
    }
    adv_update(); /* second connectable set, if configured */
}

static void on_conn_update_complete(const uint8_t *p, uint8_t len) {
    if (len < 9) {
        return;
    }
    le_slot_t *s = slot_by_handle((uint16_t)(floss_get_le16(&p[1]) & 0x0FFF));
    if (s == NULL) {
        return;
    }
    uint8_t status = p[0];
    if (status == LE_ERR_UNSUPPORTED_REMOTE_FEATURE && s->param_state == PARAM_HCI_PENDING && s->info.role == 1) {
        /* Peer controller lacks the LL procedure: ask its host over L2CAP. */
        send_l2cap_param_req(s);
        return;
    }
    s->param_state = PARAM_IDLE;
    if (status == 0) {
        s->info.interval = floss_get_le16(&p[3]);
        s->info.latency = floss_get_le16(&p[5]);
        s->info.timeout = floss_get_le16(&p[7]);
        FLOSS_LOGI(GAP, "le: interval %u.%02u ms latency %u timeout %u ms", (unsigned)(s->info.interval * 125u / 100u),
                   (unsigned)(s->info.interval * 125u % 100u), (unsigned)s->info.latency,
                   (unsigned)(s->info.timeout * 10u));
    }
    emit_link_evt(FLOSS_LE_EVT_CONN_PARAMS, s, status);
}

/** Central wants new parameters (LL procedure): we accept whatever it proposes. */
static void on_remote_conn_param_req(const uint8_t *p, uint8_t len) {
    if (len < 10) {
        return;
    }
    uint8_t c[14] = {0};
    memcpy(c, p, 10); /* handle, interval min/max, latency, timeout */
    floss_hci_cmd(FLOSS_HCI_LE_REMOTE_CONN_PARAM_REPLY, c, sizeof(c), log_cmd_status, NULL);
}

static void on_data_length_change(const uint8_t *p, uint8_t len) {
    if (len < 10) {
        return;
    }
    le_slot_t *s = slot_by_handle((uint16_t)(floss_get_le16(&p[0]) & 0x0FFF));
    if (s == NULL) {
        return;
    }
    s->info.tx_octets = floss_get_le16(&p[2]);
    FLOSS_LOGI(GAP, "le: data length tx %u rx %u", (unsigned)s->info.tx_octets, (unsigned)floss_get_le16(&p[6]));
    emit_link_evt(FLOSS_LE_EVT_DATA_LENGTH, s, 0);
}

static void on_phy_update(const uint8_t *p, uint8_t len) {
    if (len < 5) {
        return;
    }
    le_slot_t *s = slot_by_handle((uint16_t)(floss_get_le16(&p[1]) & 0x0FFF));
    if (s == NULL) {
        return;
    }
    if (p[0] == 0) {
        s->info.tx_phy = p[3];
        s->info.rx_phy = p[4];
        FLOSS_LOGI(GAP, "le: PHY tx %u rx %u", (unsigned)p[3], (unsigned)p[4]);
    }
    emit_link_evt(FLOSS_LE_EVT_PHY, s, p[0]);
}

/* ------------------------------------------------------------------------ */
/* LE signaling channel (CID 0x0005)                                        */
/* ------------------------------------------------------------------------ */

static void sig_send(uint16_t handle, uint8_t code, uint8_t id, const uint8_t *data, uint16_t len) {
    uint8_t p[4 + 10];
    if (len > sizeof(p) - 4) {
        return;
    }
    p[0] = code;
    p[1] = id;
    floss_put_le16(&p[2], len);
    if (len) {
        memcpy(&p[4], data, len);
    }
    floss_le_send_fixed(handle, FLOSS_CID_LE_SIGNALING, p, (uint16_t)(4 + len));
}

static void sig_reject(uint16_t handle, uint8_t id) {
    uint8_t reason[2] = {0x00, 0x00}; /* command not understood */
    sig_send(handle, FLOSS_LE_SIG_COMMAND_REJECT, id, reason, 2);
}

static void le_signaling(le_slot_t *s, const uint8_t *p, uint16_t len) {
    if (len < 4) {
        return;
    }
    uint8_t code = p[0];
    uint8_t id = p[1];
    uint16_t dlen = floss_get_le16(&p[2]);
    const uint8_t *d = &p[4];
    if (dlen > len - 4) {
        return;
    }
    FLOSS_TRACE(FLOSS_TR_L2CAP_SIG, ((uint32_t)code << 16) | id);
    uint16_t h = s->info.handle;

    switch (code) {
    case FLOSS_LE_SIG_COMMAND_REJECT:
        if (s->param_state == PARAM_L2CAP_PENDING && id == s->sig_id) {
            FLOSS_LOGW(GAP, "le: peer rejected the parameter update request");
            s->param_state = PARAM_IDLE;
            emit_link_evt(FLOSS_LE_EVT_CONN_PARAMS, s, LE_ERR_UNACCEPTABLE_CONN_PARAMS);
        }
        break;

    case FLOSS_LE_SIG_CONN_PARAM_RSP:
        if (dlen >= 2 && s->param_state == PARAM_L2CAP_PENDING && id == s->sig_id) {
            uint16_t result = floss_get_le16(d);
            FLOSS_LOGI(GAP, "le: parameter update %s", result == 0 ? "accepted" : "rejected");
            s->param_state = PARAM_IDLE;
            /* Accepted: the central now runs the LL procedure and we get
             * Connection Update Complete. Rejected: report it now. */
            if (result != 0) {
                emit_link_evt(FLOSS_LE_EVT_CONN_PARAMS, s, LE_ERR_UNACCEPTABLE_CONN_PARAMS);
            }
        }
        break;

    case FLOSS_LE_SIG_CONN_PARAM_REQ: {
        if (s->info.role != 0 || dlen < 8) {
            sig_reject(h, id); /* only a central may receive this request */
            break;
        }
        floss_le_conn_params_t cp = {
            .interval_min = floss_get_le16(&d[0]),
            .interval_max = floss_get_le16(&d[2]),
            .latency = floss_get_le16(&d[4]),
            .timeout = floss_get_le16(&d[6]),
        };
        bool ok = conn_params_valid(&cp);
        uint8_t rsp[2];
        floss_put_le16(rsp, ok ? 0 : 1);
        sig_send(h, FLOSS_LE_SIG_CONN_PARAM_RSP, id, rsp, 2);
        if (ok) {
            send_hci_conn_update(h, &cp, log_cmd_status);
        }
        break;
    }

    case FLOSS_LE_SIG_CREDIT_CONN_REQ: {
        /* No LE credit-based channels: "SPSM not supported". */
        uint8_t rsp[10] = {0};
        floss_put_le16(&rsp[8], 0x0002);
        sig_send(h, FLOSS_LE_SIG_CREDIT_CONN_RSP, id, rsp, sizeof(rsp));
        break;
    }

    default:
        sig_reject(h, id);
        break;
    }
}

/* ------------------------------------------------------------------------ */
/* Hooks from HCI                                                           */
/* ------------------------------------------------------------------------ */

void floss_le_init(void) {
    memset(&s_le, 0, sizeof(s_le));
    floss_le_adv_params_defaults(&s_le.adv);
    floss_le_link_policy_defaults(&s_le.policy);
    floss_le_set_name("FLOSS");
    floss_gatt_init();
    floss_att_init();
    floss_gatt_services_register_core();
    floss_hogp_reset();
    floss_smp_init();
}

void floss_le_on_hci_ready(void) {
    s_le.ready = true;
    s_le.adv_on = false;
    s_le.adv_target = false;
    adv_reconfigure();
}

void floss_le_on_hci_event(uint8_t subevent, const uint8_t *params, uint8_t len) {
    switch (subevent) {
    case FLOSS_LE_SUBEVT_CONN_COMPLETE:
        on_conn_complete(params, len, false);
        break;
    case FLOSS_LE_SUBEVT_ENH_CONN_COMPLETE:
        on_conn_complete(params, len, true);
        break;
    case FLOSS_LE_SUBEVT_CONN_UPDATE_COMPLETE:
        on_conn_update_complete(params, len);
        break;
    case FLOSS_LE_SUBEVT_REMOTE_CONN_PARAM_REQ:
        on_remote_conn_param_req(params, len);
        break;
    case FLOSS_LE_SUBEVT_DATA_LENGTH_CHANGE:
        on_data_length_change(params, len);
        break;
    case FLOSS_LE_SUBEVT_PHY_UPDATE_COMPLETE:
        on_phy_update(params, len);
        break;
    case FLOSS_LE_SUBEVT_LTK_REQUEST:
    case FLOSS_LE_SUBEVT_P256_KEY_COMPLETE:
    case FLOSS_LE_SUBEVT_DHKEY_COMPLETE:
        floss_smp_on_le_event(subevent, params, len);
        break;
    default:
        FLOSS_LOGD(GAP, "le: subevent 0x%02x ignored", (unsigned)subevent);
        break;
    }
}

/** Security level bookkeeping; emits SECURITY and tells HOGP. */
static void set_security_level(le_slot_t *s, uint8_t level) {
    if (s->info.security_level == level) {
        return;
    }
    s->info.security_level = level;
    FLOSS_LOGI(GAP, "le: link 0x%04x security level %u", (unsigned)s->info.handle, (unsigned)level);
    emit_link_evt(FLOSS_LE_EVT_SECURITY, s, 0);
    floss_hogp_on_security(s->info.handle, level);
}

void floss_le_on_encryption(uint16_t handle, uint8_t status, bool encrypted) {
    le_slot_t *s = slot_by_handle(handle);
    if (s && status == 0) {
        s->info.encrypted = encrypted;
        /* Encryption alone proves level >= 2; SMP refines it (3/4) through
         * floss_le_on_security_changed() when the key was MITM-protected. */
        if (!encrypted) {
            set_security_level(s, 1);
        } else if (s->info.security_level < 2) {
            set_security_level(s, 2);
        }
    }
    floss_smp_on_encryption(handle, status, encrypted);
}

void floss_le_on_security_changed(uint16_t handle, uint8_t level) {
    le_slot_t *s = slot_by_handle(handle);
    if (s && level >= 1 && level <= 4) {
        set_security_level(s, level);
    }
}

void floss_le_on_pdu(uint16_t handle, const uint8_t *pdu, uint16_t len) {
    le_slot_t *s = slot_by_handle(handle);
    if (s == NULL || len < FLOSS_L2CAP_HDR_LEN) {
        FLOSS_STAT_INC(rx_dropped);
        return;
    }
    uint16_t plen = floss_get_le16(&pdu[0]);
    uint16_t cid = floss_get_le16(&pdu[2]);
    if (plen != len - FLOSS_L2CAP_HDR_LEN) {
        FLOSS_STAT_INC(rx_dropped);
        return;
    }
    const uint8_t *payload = &pdu[FLOSS_L2CAP_HDR_LEN];
    switch (cid) {
    case FLOSS_CID_ATT:
        floss_att_on_pdu(handle, payload, plen);
        break;
    case FLOSS_CID_LE_SIGNALING:
        le_signaling(s, payload, plen);
        break;
    case FLOSS_CID_SMP:
        floss_smp_on_pdu(handle, payload, plen);
        break;
    default:
        FLOSS_LOGD(GAP, "le: PDU on unknown CID 0x%04x dropped", (unsigned)cid);
        FLOSS_STAT_INC(rx_dropped);
        break;
    }
}

void floss_le_on_link_down(uint16_t handle, uint8_t reason) {
    int idx = floss_le_link_index(handle);
    if (idx < 0) {
        return;
    }
    le_slot_t *s = &s_le.links[idx];
    char str[18];
    floss_addr_to_str(&s->info.peer, str);
    FLOSS_LOGI(GAP, "le: disconnected 0x%04x %s: %s", (unsigned)handle, str, floss_hci_status_str(reason));
    floss_timer_stop(&s->update_timer);
    floss_smp_on_link_down(handle);
    floss_att_on_link_down(handle);
    floss_gatt_on_link_down((uint8_t)idx);
    floss_hogp_on_link_down(handle, reason);
    emit_link_evt(FLOSS_LE_EVT_DISCONNECTED, s, reason);
    s->used = false;
    adv_update(); /* auto-restart */
}

void floss_le_on_credits(void) {
    floss_att_poll();
    floss_hogp_on_credits();
}

void floss_le_poll(void) {
    floss_att_poll();
    floss_hogp_poll();
}

floss_err_t floss_le_send_fixed(uint16_t handle, uint16_t cid, const uint8_t *data, uint16_t len) {
    floss_buf_t *b = floss_buf_alloc();
    if (b == NULL) {
        FLOSS_STAT_INC(buf_alloc_failures);
        return FLOSS_ERR_BUSY;
    }
    uint8_t *p = floss_buf_put(b, len);
    if (p == NULL) {
        floss_buf_free(b);
        return FLOSS_ERR_INVALID;
    }
    if (len) {
        memcpy(p, data, len);
    }
    uint8_t *hdr = floss_buf_push(b, FLOSS_L2CAP_HDR_LEN);
    floss_put_le16(&hdr[0], len);
    floss_put_le16(&hdr[2], cid);
    b->handle = handle;
    b->type = FLOSS_PKT_ACL;
    b->flags = 0;
    return floss_hci_acl_send(b);
}

#endif /* FLOSS_ENABLE_LE */
