/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_le_central.c - LE central role: scanning, advertising data parsing,
 * initiating connections, PHY requests.
 *
 * Overview
 * --------
 * Like advertising in floss_le.c this module has no state machine of its
 * own: the HCI command queue is strict FIFO with one command outstanding, so
 * a sequence such as "stop scanning, then create the connection" is simply
 * queued back to back and the controller sees the commands in that order.
 * That matters because controllers refuse LE Set Scan Parameters while
 * scanning and (most of them) LE Create Connection while scanning.
 *
 *  - SCANNING. floss_le_scan_start() queues [Scan Enable 0 if we were
 *    scanning], Set Scan Parameters, Scan Enable 1. LE Advertising Reports
 *    (LE meta subevent 0x02) are forwarded by floss_le.c to
 *    floss_le_central_on_le_event(); every report is bounds-checked before
 *    anything is handed to the application, and an event that is
 *    inconsistent anywhere is dropped as a whole.
 *
 *  - CONNECTING. floss_le_connect() stops scanning, then queues LE Create
 *    Connection. The result arrives as LE (Enhanced) Connection Complete,
 *    which HCI turns into a connection-table entry and floss_le.c into a
 *    link (role 0 = central) and FLOSS_LE_EVT_CONNECTED, or
 *    FLOSS_LE_EVT_CONNECT_FAILED when the status is non-zero (also after a
 *    cancel: status 0x02 Unknown Connection Identifier). A Create Connection
 *    the controller refuses outright (Command Status != 0) never produces a
 *    Connection Complete, so CONNECT_FAILED is emitted here instead.
 *
 *  - ADDRESSES. Scanning and initiating use the same own address type as
 *    advertising (floss_le_own_address): public, or the random static
 *    address floss_le.c has already written to the controller.
 *
 * AD data: UUIDs inside advertising data are little-endian (wire order).
 * The API takes 128-bit UUIDs big-endian, the way they are written
 * ("6e400001-b5a3-..." -> {0x6e, 0x40, ...}); floss_le_ad_has_uuid128()
 * reverses them before comparing.
 */
#include "floss_gatt_client.h"
#include "floss/floss_le.h"
#include "floss/floss_le_central.h"

#if FLOSS_ENABLE_LE

/* LE controller commands only this module sends (OGF 0x08) */
#define CEN_OP_SET_SCAN_PARAMS    FLOSS_OPCODE(0x08, 0x000B)
#define CEN_OP_SET_SCAN_ENABLE    FLOSS_OPCODE(0x08, 0x000C)
#define CEN_OP_CREATE_CONN        FLOSS_OPCODE(0x08, 0x000D)
#define CEN_OP_CREATE_CONN_CANCEL FLOSS_OPCODE(0x08, 0x000E)

/* AD types for 128-bit service UUID lists */
#define CEN_AD_UUID128_SOME 0x06
#define CEN_AD_UUID128_ALL  0x07

/* Advertising-timeout status: belongs to directed advertising, never to us */
#define CEN_ERR_ADV_TIMEOUT 0x3C

/* LE Advertising Report: Num_Reports is 1..0x19 */
#define CEN_MAX_REPORTS 0x19
/* Fixed part of one report: event type, address type, address, data length */
#define CEN_REPORT_HDR 9

static struct {
    floss_le_scan_fn fn;
    void *user;
    bool target;           /* scanning wanted: last Scan Enable value we queued */
    bool on;               /* controller confirmed scanning */
    uint8_t enable_inflight; /* Scan Enable commands queued, not yet completed */
    bool connecting;       /* Create Connection issued, no Connection Complete yet */
    bool cancelling;       /* Create Connection Cancel queued */
    floss_addr_t peer;     /* address of the connection attempt */
    uint8_t peer_type;
} s_cen;

/* ------------------------------------------------------------------------ */
/* Parameter checks                                                         */
/* ------------------------------------------------------------------------ */

/** Scan interval/window: 0x0004..0x4000 (2.5 ms .. 10.24 s), window <= interval. */
static bool scan_timing_ok(uint16_t interval, uint16_t window) {
    return interval >= 0x0004 && interval <= 0x4000 && window >= 0x0004 && window <= interval;
}

/** Connection parameter limits, including "timeout outlasts (1 + latency) intervals twice". */
static bool conn_params_ok(const floss_le_conn_params_t *p) {
    return p->interval_min >= 6 && p->interval_max <= 3200 && p->interval_min <= p->interval_max &&
           p->latency <= 499 && p->timeout >= 10 && p->timeout <= 3200 &&
           (uint32_t)p->timeout * 4u > (uint32_t)(1u + p->latency) * p->interval_max;
}

/* ------------------------------------------------------------------------ */
/* Scanning                                                                 */
/* ------------------------------------------------------------------------ */

void floss_le_scan_params_defaults(floss_le_scan_params_t *p) {
    memset(p, 0, sizeof(*p));
    p->interval = 0x0030; /* 30 ms */
    p->window = 0x0030;   /* = interval: listen continuously */
    p->active = true;
    p->filter_duplicates = false;
}

static void scan_enable_done(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len) {
    bool enable = ctx != NULL;
    FLOSS_UNUSED(opcode);
    FLOSS_UNUSED(ret);
    FLOSS_UNUSED(ret_len);
    if (s_cen.enable_inflight) {
        s_cen.enable_inflight--;
    }
    if (status) {
        FLOSS_LOGW(LE, "le: scan %s failed: %s", enable ? "enable" : "disable", floss_hci_status_str(status));
        if (!enable) {
            s_cen.on = false; /* already off, or the controller is broken either way */
        } else if (s_cen.enable_inflight == 0) {
            s_cen.target = s_cen.on; /* the latest request failed: what we want is moot */
        }
        return;
    }
    if (s_cen.on != enable) {
        s_cen.on = enable;
        FLOSS_LOGI(LE, "le: scanning %s", enable ? "on" : "off");
    }
}

static floss_err_t scan_send_enable(bool enable, bool filter_duplicates) {
    uint8_t p[2] = {enable ? 1 : 0, (enable && filter_duplicates) ? 1 : 0};
    if (floss_hci_cmd(CEN_OP_SET_SCAN_ENABLE, p, sizeof(p), scan_enable_done, enable ? (void *)1 : NULL) !=
        FLOSS_OK) {
        return FLOSS_ERR_BUSY;
    }
    s_cen.enable_inflight++;
    s_cen.target = enable;
    return FLOSS_OK;
}

static void log_cmd_status(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len) {
    FLOSS_UNUSED(ctx);
    FLOSS_UNUSED(ret);
    FLOSS_UNUSED(ret_len);
    if (status) {
        FLOSS_LOGW(LE, "le: cmd 0x%04x failed: %s", (unsigned)opcode, floss_hci_status_str(status));
    }
}

floss_err_t floss_le_scan_start(const floss_le_scan_params_t *p, floss_le_scan_fn fn, void *user) {
    floss_le_scan_params_t def;
    if (p == NULL) {
        floss_le_scan_params_defaults(&def);
        p = &def;
    }
    if (fn == NULL || !scan_timing_ok(p->interval, p->window)) {
        return FLOSS_ERR_INVALID;
    }
    if (s_cen.connecting) {
        return FLOSS_ERR_BUSY; /* initiating: the controller is busy scanning for that peer */
    }
    /* Parameters may only change while scanning is off. */
    if (s_cen.target && scan_send_enable(false, false) != FLOSS_OK) {
        return FLOSS_ERR_BUSY;
    }
    floss_addr_t own;
    uint8_t c[7];
    c[0] = p->active ? 0x01 : 0x00;
    floss_put_le16(&c[1], p->interval);
    floss_put_le16(&c[3], p->window);
    floss_le_own_address(&own, &c[5]);
    c[6] = 0x00; /* filter policy: accept all advertisers */
    if (floss_hci_cmd(CEN_OP_SET_SCAN_PARAMS, c, sizeof(c), log_cmd_status, NULL) != FLOSS_OK) {
        return FLOSS_ERR_BUSY;
    }
    s_cen.fn = fn;
    s_cen.user = user;
    if (scan_send_enable(true, p->filter_duplicates) != FLOSS_OK) {
        return FLOSS_ERR_BUSY;
    }
    FLOSS_LOGI(LE, "le: %s scan, interval %u window %u", p->active ? "active" : "passive", (unsigned)p->interval,
               (unsigned)p->window);
    return FLOSS_OK;
}

floss_err_t floss_le_scan_stop(void) {
    if (!s_cen.target) {
        return FLOSS_OK;
    }
    return scan_send_enable(false, false);
}

bool floss_le_scanning(void) {
    return s_cen.target;
}

/**
 * LE Advertising Report: Num_Reports, then per report Event_Type,
 * Address_Type, Address(6), Data_Length, Data, RSSI. The whole event is
 * validated first: if any report does not fit, none is trusted.
 */
static void on_adv_report(const uint8_t *p, uint8_t len) {
    if (len < 1 || p[0] == 0 || p[0] > CEN_MAX_REPORTS) {
        FLOSS_STAT_INC(rx_dropped);
        return;
    }
    uint8_t n = p[0];
    uint16_t off = 1;
    for (uint8_t i = 0; i < n; i++) {
        if (off + CEN_REPORT_HDR > len) {
            FLOSS_STAT_INC(rx_dropped);
            return;
        }
        uint8_t dlen = p[off + 8];
        if (dlen > FLOSS_LE_ADV_DATA_MAX || off + CEN_REPORT_HDR + dlen + 1u > len) {
            FLOSS_STAT_INC(rx_dropped);
            return;
        }
        off = (uint16_t)(off + CEN_REPORT_HDR + dlen + 1);
    }
    off = 1;
    /* Re-check every time: the callback may stop scanning or start connecting. */
    for (uint8_t i = 0; i < n && s_cen.target && s_cen.fn; i++) {
        floss_le_adv_report_t r;
        r.evt_type = p[off];
        r.addr_type = p[off + 1];
        memcpy(r.addr.b, &p[off + 2], 6);
        r.len = p[off + 8];
        r.data = &p[off + CEN_REPORT_HDR];
        r.rssi = (int8_t)p[off + CEN_REPORT_HDR + r.len];
        s_cen.fn(&r, s_cen.user);
        off = (uint16_t)(off + CEN_REPORT_HDR + r.len + 1);
    }
}

/* ------------------------------------------------------------------------ */
/* Advertising data                                                         */
/* ------------------------------------------------------------------------ */

/**
 * Walk AD structures [len][type][data(len-1)]. Calls back with each payload
 * of `type`; stops early when visit() returns true. A zero length ends the
 * significant part; a structure running past the end stops the walk.
 */
static const uint8_t *ad_walk(const uint8_t *data, uint8_t len, uint8_t type, uint8_t *out_len,
                              bool (*visit)(const uint8_t *pl, uint8_t pl_len, const void *arg), const void *arg) {
    if (data == NULL) {
        return NULL;
    }
    uint16_t off = 0;
    while (off < len) {
        uint8_t l = data[off];
        if (l == 0 || off + 1u + l > len) {
            return NULL; /* end of significant part, or malformed */
        }
        if (data[off + 1] == type) {
            const uint8_t *pl = &data[off + 2];
            uint8_t pl_len = (uint8_t)(l - 1);
            if (visit == NULL || visit(pl, pl_len, arg)) {
                if (out_len) {
                    *out_len = pl_len;
                }
                return pl;
            }
        }
        off = (uint16_t)(off + 1u + l);
    }
    return NULL;
}

const uint8_t *floss_le_ad_find(const uint8_t *data, uint8_t len, uint8_t ad_type, uint8_t *out_len) {
    return ad_walk(data, len, ad_type, out_len, NULL, NULL);
}

/** Does a 128-bit UUID list payload contain `arg` (16 bytes, little-endian)? */
static bool ad_list_has(const uint8_t *pl, uint8_t pl_len, const void *arg) {
    for (uint16_t i = 0; i + 16u <= pl_len; i = (uint16_t)(i + 16)) {
        if (memcmp(&pl[i], arg, 16) == 0) {
            return true;
        }
    }
    return false;
}

bool floss_le_ad_has_uuid128(const uint8_t *data, uint8_t len, const uint8_t uuid_be[16]) {
    if (uuid_be == NULL) {
        return false;
    }
    uint8_t le[16];
    for (uint8_t i = 0; i < 16; i++) {
        le[i] = uuid_be[15 - i]; /* written form is big-endian; AD data carries it little-endian */
    }
    return ad_walk(data, len, CEN_AD_UUID128_ALL, NULL, ad_list_has, le) != NULL ||
           ad_walk(data, len, CEN_AD_UUID128_SOME, NULL, ad_list_has, le) != NULL;
}

/* ------------------------------------------------------------------------ */
/* Connecting                                                               */
/* ------------------------------------------------------------------------ */

void floss_le_connect_params_defaults(floss_le_connect_params_t *p) {
    memset(p, 0, sizeof(*p));
    p->scan_interval = 0x0030;
    p->scan_window = 0x0030;
    p->conn.interval_min = 6; /* 7.5 ms */
    p->conn.interval_max = 6;
    p->conn.latency = 0;
    p->conn.timeout = 200;    /* 2 s */
}

static void emit_connect_failed(uint8_t status) {
    floss_le_evt_t e;
    memset(&e, 0, sizeof(e));
    e.type = FLOSS_LE_EVT_CONNECT_FAILED;
    e.status = status;
    e.addr = s_cen.peer;
    e.addr_type = s_cen.peer_type;
    floss_le_emit(&e);
}

/** Command Status of LE Create Connection: non-zero means no Connection Complete will follow. */
static void create_conn_status(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len) {
    FLOSS_UNUSED(ctx);
    FLOSS_UNUSED(opcode);
    FLOSS_UNUSED(ret);
    FLOSS_UNUSED(ret_len);
    if (status == 0 || !s_cen.connecting) {
        return;
    }
    FLOSS_LOGW(LE, "le: create connection refused: %s", floss_hci_status_str(status));
    s_cen.connecting = false;
    emit_connect_failed(status);
}

floss_err_t floss_le_connect(uint8_t addr_type, const floss_addr_t *addr, const floss_le_connect_params_t *p) {
    floss_le_connect_params_t def;
    if (p == NULL) {
        floss_le_connect_params_defaults(&def);
        p = &def;
    }
    if (addr == NULL || addr_type > 0x03 || !scan_timing_ok(p->scan_interval, p->scan_window) ||
        !conn_params_ok(&p->conn)) {
        return FLOSS_ERR_INVALID;
    }
    if (s_cen.connecting) {
        return FLOSS_ERR_BUSY;
    }
    if (floss_le_link_count() >= FLOSS_MAX_CONNECTIONS) {
        return FLOSS_ERR_NOMEM;
    }
    /* Controllers refuse to initiate while scanning: stop first (FIFO queue). */
    if (s_cen.target && scan_send_enable(false, false) != FLOSS_OK) {
        return FLOSS_ERR_BUSY;
    }
    floss_addr_t own;
    uint8_t c[25];
    memset(c, 0, sizeof(c));
    floss_put_le16(&c[0], p->scan_interval);
    floss_put_le16(&c[2], p->scan_window);
    c[4] = 0x00; /* initiator filter policy: use the peer address below */
    c[5] = addr_type;
    memcpy(&c[6], addr->b, 6);
    floss_le_own_address(&own, &c[12]);
    floss_put_le16(&c[13], p->conn.interval_min);
    floss_put_le16(&c[15], p->conn.interval_max);
    floss_put_le16(&c[17], p->conn.latency);
    floss_put_le16(&c[19], p->conn.timeout);
    /* c[21..24]: minimum/maximum CE length 0 = controller decides */
    if (floss_hci_cmd(CEN_OP_CREATE_CONN, c, sizeof(c), create_conn_status, NULL) != FLOSS_OK) {
        return FLOSS_ERR_BUSY;
    }
    s_cen.connecting = true;
    s_cen.cancelling = false;
    s_cen.peer = *addr;
    s_cen.peer_type = addr_type;
    char str[18];
    floss_addr_to_str(addr, str);
    FLOSS_LOGI(LE, "le: connecting to %s (type %u)", str, (unsigned)addr_type);
    return FLOSS_OK;
}

static void cancel_done(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len) {
    FLOSS_UNUSED(ctx);
    FLOSS_UNUSED(opcode);
    FLOSS_UNUSED(ret);
    FLOSS_UNUSED(ret_len);
    s_cen.cancelling = false;
    if (status) {
        /* Usually a race: the connection completed (or failed) first. */
        FLOSS_LOGD(LE, "le: create connection cancel: %s", floss_hci_status_str(status));
    }
}

floss_err_t floss_le_connect_cancel(void) {
    if (!s_cen.connecting) {
        return FLOSS_ERR_STATE;
    }
    if (s_cen.cancelling) {
        return FLOSS_OK;
    }
    if (floss_hci_cmd(CEN_OP_CREATE_CONN_CANCEL, NULL, 0, cancel_done, NULL) != FLOSS_OK) {
        return FLOSS_ERR_BUSY;
    }
    s_cen.cancelling = true;
    return FLOSS_OK;
}

/** LE Set PHY refused by our controller: report it like a failed PHY update. */
static void set_phy_status(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len) {
    FLOSS_UNUSED(opcode);
    FLOSS_UNUSED(ret);
    FLOSS_UNUSED(ret_len);
    uint16_t handle = (uint16_t)(uintptr_t)ctx;
    const floss_le_link_t *l = floss_le_link(handle);
    if (status == 0 || l == NULL) {
        return;
    }
    FLOSS_LOGW(LE, "le: set PHY refused: %s", floss_hci_status_str(status));
    floss_le_evt_t e;
    memset(&e, 0, sizeof(e));
    e.type = FLOSS_LE_EVT_PHY;
    e.handle = handle;
    e.addr = l->peer;
    e.addr_type = l->peer_addr_type;
    e.role = l->role;
    e.status = status;
    e.tx_phy = l->tx_phy;
    e.rx_phy = l->rx_phy;
    floss_le_emit(&e);
}

floss_err_t floss_le_set_phy(uint16_t handle, uint8_t tx_phys, uint8_t rx_phys) {
    if (floss_le_link(handle) == NULL) {
        return FLOSS_ERR_NOT_FOUND;
    }
    if (((tx_phys | rx_phys) & ~0x07u) != 0) {
        return FLOSS_ERR_INVALID;
    }
    uint8_t c[7] = {0};
    floss_put_le16(&c[0], handle);
    /* ALL_PHYS: bit 0 = no TX preference, bit 1 = no RX preference */
    c[2] = (uint8_t)((tx_phys == 0 ? 0x01 : 0x00) | (rx_phys == 0 ? 0x02 : 0x00));
    c[3] = tx_phys;
    c[4] = rx_phys;
    /* c[5..6]: PHY options (Coded PHY coding), no preference */
    if (floss_hci_cmd(FLOSS_HCI_LE_SET_PHY, c, sizeof(c), set_phy_status, (void *)(uintptr_t)handle) != FLOSS_OK) {
        return FLOSS_ERR_BUSY;
    }
    return FLOSS_OK;
}

/* ------------------------------------------------------------------------ */
/* Hooks                                                                    */
/* ------------------------------------------------------------------------ */

void floss_le_central_init(void) {
    memset(&s_cen, 0, sizeof(s_cen));
    floss_gattc_init();
}

bool floss_le_central_on_le_event(uint8_t subevent, const uint8_t *params, uint8_t len) {
    switch (subevent) {
    case FLOSS_LE_SUBEVT_ADV_REPORT:
        on_adv_report(params, len);
        return true;
    case FLOSS_LE_SUBEVT_CONN_COMPLETE:
    case FLOSS_LE_SUBEVT_ENH_CONN_COMPLETE:
        /* Status(1) Handle(2) Role(1) ...: our attempt is over when a central
         * link opens or any attempt fails (advertising timeouts aside). */
        if (len >= 4 && s_cen.connecting &&
            (params[0] == 0 ? params[3] == 0 : params[0] != CEN_ERR_ADV_TIMEOUT)) {
            s_cen.connecting = false;
            s_cen.cancelling = false;
        }
        return false;
    default:
        return false;
    }
}

#endif /* FLOSS_ENABLE_LE */
