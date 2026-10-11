/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * test_le_central.c - LE central role: scanning, advertising data helpers,
 * connecting as central, PHY requests and the GATT client, all against the
 * simulator (remote advertisers + a scripted GATT server on the far side).
 *
 * The scripted server's database (handles):
 *
 *    1  primary service 0x1800
 *    2  characteristic decl  READ                -> 3 (0x2A00)
 *    3  value 0x2A00 "PAD"
 *    4  characteristic decl  READ|NOTIFY         -> 5 (0x2A19)
 *    5  value 0x2A19 {100}
 *    6  CCCD
 *    7  primary service (128-bit)
 *    8  characteristic decl  WRITE|WNR|NOTIFY|INDICATE -> 9 (UUID X)
 *    9  value X
 *   10  user description "rx"
 *   11  CCCD
 *   12  characteristic decl  READ                -> 13 (UUID Y)
 *   13  value Y "hello"
 *   14  value 0x2A50 that always fails with Insufficient Authentication
 *
 * At ATT MTU 23 the 0x2803 declarations come back in two Read By Type rounds
 * (the 16-bit ones, then the first 128-bit one: pairs in one response must
 * have the same length), which exercises the client's continuation logic.
 */
#include "test_common.h"
#include "floss/floss_le_central.h"

#if FLOSS_ENABLE_LE

/* ------------------------------------------------------------------------ */
/* Fixtures                                                                 */
/* ------------------------------------------------------------------------ */

/** The remote pad: random static address C6:55:44:33:22:11. */
static const floss_addr_t k_pad = {{0x11, 0x22, 0x33, 0x44, 0x55, 0xC6}};

/* UUIDs as written (big-endian) */
static const uint8_t k_uuid_x[16] = {0xF1, 0x05, 0x50, 0x01, 0x7A, 0x2B, 0x4C, 0x3D,
                                     0x8E, 0x4F, 0x50, 0x61, 0x72, 0x83, 0x94, 0xA5};
static const uint8_t k_uuid_y[16] = {0xF1, 0x05, 0x50, 0x02, 0x7A, 0x2B, 0x4C, 0x3D,
                                     0x8E, 0x4F, 0x50, 0x61, 0x72, 0x83, 0x94, 0xA5};
static const uint8_t k_uuid_svc[16] = {0xF1, 0x05, 0x50, 0x00, 0x7A, 0x2B, 0x4C, 0x3D,
                                       0x8E, 0x4F, 0x50, 0x61, 0x72, 0x83, 0x94, 0xA5};

static void be_to_le(const uint8_t be[16], uint8_t le[16]) {
    for (int i = 0; i < 16; i++) {
        le[i] = be[15 - i];
    }
}

/* LE listener log */
#define LE_LOG 48
static floss_le_evt_t g_le[LE_LOG];
static unsigned g_le_count;

static void le_listener(const floss_le_evt_t *e, void *user) {
    (void)user;
    if (g_le_count < LE_LOG) {
        g_le[g_le_count++] = *e;
    }
}

static const floss_le_evt_t *le_last(floss_le_evt_type_t type) {
    for (unsigned i = g_le_count; i > 0; i--) {
        if (g_le[i - 1].type == type) {
            return &g_le[i - 1];
        }
    }
    return NULL;
}

static unsigned le_count(floss_le_evt_type_t type) {
    unsigned n = 0;
    for (unsigned i = 0; i < g_le_count; i++) {
        n += g_le[i].type == type;
    }
    return n;
}

/* Scan callback log (the data pointer is only valid during the callback: copy). */
typedef struct {
    floss_le_adv_report_t r;
    uint8_t data[64];
} rep_t;
static rep_t g_rep[32];
static unsigned g_rep_count;

static void scan_cb(const floss_le_adv_report_t *r, void *user) {
    (void)user;
    if (g_rep_count < FLOSS_ARRAY_SIZE(g_rep)) {
        rep_t *x = &g_rep[g_rep_count];
        x->r = *r;
        memcpy(x->data, r->data, r->len);
        x->r.data = x->data;
    }
    g_rep_count++;
}

/* GATT client callback logs */
static struct {
    unsigned calls;
    uint16_t conn;
    uint8_t err;
} g_done;
static struct {
    unsigned calls;
    uint8_t err, props;
    uint16_t vh, cccd;
} g_char;
static struct {
    unsigned calls;
    uint8_t err;
    uint8_t data[200];
    uint16_t len;
} g_read;
static struct {
    unsigned calls, indications;
    uint16_t conn, handle;
    uint8_t data[200];
    uint16_t len;
} g_notify;

static void done_cb(uint16_t conn, uint8_t att_err, void *user) {
    (void)user;
    g_done.calls++;
    g_done.conn = conn;
    g_done.err = att_err;
}

static void char_cb(uint16_t conn, uint8_t att_err, uint8_t properties, uint16_t value_handle, uint16_t cccd_handle,
                    void *user) {
    (void)conn;
    (void)user;
    g_char.calls++;
    g_char.err = att_err;
    g_char.props = properties;
    g_char.vh = value_handle;
    g_char.cccd = cccd_handle;
}

static void read_cb(uint16_t conn, uint8_t att_err, const uint8_t *data, uint16_t len, void *user) {
    (void)conn;
    (void)user;
    g_read.calls++;
    g_read.err = att_err;
    g_read.len = len;
    if (len) {
        memcpy(g_read.data, data, FLOSS_MIN(len, (uint16_t)sizeof g_read.data));
    }
}

static void notify_cb(uint16_t conn, uint16_t handle, const uint8_t *data, uint16_t len, bool indication, void *user) {
    (void)user;
    g_notify.calls++;
    g_notify.indications += indication;
    g_notify.conn = conn;
    g_notify.handle = handle;
    g_notify.len = len;
    memcpy(g_notify.data, data, FLOSS_MIN(len, (uint16_t)sizeof g_notify.data));
}

static bool wait_for(const unsigned *ctr, unsigned target) {
    for (unsigned i = 0; i < 3000 && *ctr < target; i++) {
        floss_sim_pump(1);
    }
    return *ctr >= target;
}

/** Stack up (no HID), LE listener + notification handler installed, logs cleared. */
static void central_start(void) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_stack_start(t, NULL);
    g_le_count = 0;
    g_rep_count = 0;
    memset(&g_done, 0, sizeof g_done);
    memset(&g_char, 0, sizeof g_char);
    memset(&g_read, 0, sizeof g_read);
    memset(&g_notify, 0, sizeof g_notify);
    ASSERT_EQ(floss_le_add_listener(le_listener, NULL), FLOSS_OK);
    floss_gattc_set_notify_handler(notify_cb, NULL);
}

/** Advertising data of the pad: flags, complete 128-bit service list, short name. */
static uint8_t pad_adv(uint8_t *out) {
    uint8_t n = 0;
    out[n++] = 2;
    out[n++] = FLOSS_AD_FLAGS;
    out[n++] = FLOSS_AD_FLAG_GENERAL_DISC | FLOSS_AD_FLAG_NO_BREDR;
    out[n++] = 17;
    out[n++] = FLOSS_AD_UUID128_ALL;
    be_to_le(k_uuid_svc, &out[n]);
    n = (uint8_t)(n + 16);
    out[n++] = 4;
    out[n++] = FLOSS_AD_NAME_SHORT;
    out[n++] = 'P';
    out[n++] = 'a';
    out[n++] = 'd';
    return n;
}

static floss_sim_advertiser_t *add_pad(void) {
    uint8_t adv[31];
    uint8_t n = pad_adv(adv);
    static const uint8_t rsp[] = {10, FLOSS_AD_NAME_COMPLETE, 'F', 'L', 'O', 'S', 'S', ' ', 'P', 'a', 'd'};
    floss_sim_advertiser_t *a = floss_sim_le_advertiser_add(&k_pad, FLOSS_LE_ADDR_RANDOM, adv, n, rsp, sizeof rsp);
    ASSERT_NOT_NULL(a);
    return a;
}

/** Connect to the pad as central; returns the handle. */
static uint16_t central_connect(void) {
    add_pad();
    ASSERT_EQ(floss_le_connect(FLOSS_LE_ADDR_RANDOM, &k_pad, NULL), FLOSS_OK);
    unsigned target = le_count(FLOSS_LE_EVT_CONNECTED) + 1;
    for (unsigned i = 0; i < 200 && le_count(FLOSS_LE_EVT_CONNECTED) < target; i++) {
        floss_sim_pump(1);
    }
    const floss_le_evt_t *e = le_last(FLOSS_LE_EVT_CONNECTED);
    ASSERT_NOT_NULL(e);
    ASSERT_EQ(e->role, 0);
    floss_sim_pump(20); /* data length / key pre-warm traffic settles */
    return e->handle;
}

/* The scripted server database (see the file header). */
static uint8_t g_decl2[5], g_decl4[5], g_decl8[19], g_decl12[19];
static uint8_t g_svc7[16], g_x_le[16], g_y_le[16];
static floss_sim_gatt_attr_t g_db[14];

static void build_db(void) {
    static const uint8_t svc1800[2] = {0x00, 0x18};
    static const uint8_t pad[3] = {'P', 'A', 'D'};
    static const uint8_t battery[1] = {100};
    static const uint8_t cccd0[2] = {0, 0};
    static const uint8_t desc[2] = {'r', 'x'};
    static const uint8_t hello[5] = {'h', 'e', 'l', 'l', 'o'};
    be_to_le(k_uuid_svc, g_svc7);
    be_to_le(k_uuid_x, g_x_le);
    be_to_le(k_uuid_y, g_y_le);
    g_decl2[0] = FLOSS_GATT_PROP_READ;
    floss_put_le16(&g_decl2[1], 3);
    floss_put_le16(&g_decl2[3], 0x2A00);
    g_decl4[0] = FLOSS_GATT_PROP_READ | FLOSS_GATT_PROP_NOTIFY;
    floss_put_le16(&g_decl4[1], 5);
    floss_put_le16(&g_decl4[3], 0x2A19);
    g_decl8[0] = FLOSS_GATT_PROP_WRITE | FLOSS_GATT_PROP_WRITE_NO_RSP | FLOSS_GATT_PROP_NOTIFY |
                 FLOSS_GATT_PROP_INDICATE;
    floss_put_le16(&g_decl8[1], 9);
    memcpy(&g_decl8[3], g_x_le, 16);
    g_decl12[0] = FLOSS_GATT_PROP_READ;
    floss_put_le16(&g_decl12[1], 13);
    memcpy(&g_decl12[3], g_y_le, 16);

    const floss_sim_gatt_attr_t db[14] = {
        {.handle = 1, .type16 = 0x2800, .value = svc1800, .len = 2},
        {.handle = 2, .type16 = 0x2803, .value = g_decl2, .len = 5},
        {.handle = 3, .type16 = 0x2A00, .value = pad, .len = 3},
        {.handle = 4, .type16 = 0x2803, .value = g_decl4, .len = 5},
        {.handle = 5, .type16 = 0x2A19, .value = battery, .len = 1},
        {.handle = 6, .type16 = 0x2902, .value = cccd0, .len = 2},
        {.handle = 7, .type16 = 0x2800, .value = g_svc7, .len = 16},
        {.handle = 8, .type16 = 0x2803, .value = g_decl8, .len = 19},
        {.handle = 9, .type128 = g_x_le, .len = 0},
        {.handle = 10, .type16 = 0x2901, .value = desc, .len = 2},
        {.handle = 11, .type16 = 0x2902, .value = cccd0, .len = 2},
        {.handle = 12, .type16 = 0x2803, .value = g_decl12, .len = 19},
        {.handle = 13, .type128 = g_y_le, .value = hello, .len = 5},
        {.handle = 14, .type16 = 0x2A50, .error = FLOSS_ATT_ERR_INSUFFICIENT_AUTHEN},
    };
    memcpy(g_db, db, sizeof db);
}

/** Central link to the pad with the scripted server on it. */
static uint16_t gatt_session(uint16_t server_mtu) {
    central_start();
    uint16_t h = central_connect();
    build_db();
    ASSERT_EQ(floss_sim_gatt_serve(h, g_db, (uint8_t)FLOSS_ARRAY_SIZE(g_db), server_mtu), FLOSS_OK);
    return h;
}

static void drain_att(uint16_t h) {
    uint8_t tmp[256];
    while (floss_sim_att_recv(h, tmp, sizeof tmp) >= 0) {
    }
}

/* ------------------------------------------------------------------------ */
/* Advertising data helpers                                                 */
/* ------------------------------------------------------------------------ */

TEST(ad_find_and_uuid128) {
    uint8_t adv[31];
    uint8_t n = pad_adv(adv);
    uint8_t len = 0xEE;
    const uint8_t *name = floss_le_ad_find(adv, n, FLOSS_AD_NAME_SHORT, &len);
    ASSERT_NOT_NULL(name);
    ASSERT_EQ(len, 3);
    ASSERT_MEM_EQ(name, "Pad", 3);
    const uint8_t *flags = floss_le_ad_find(adv, n, FLOSS_AD_FLAGS, &len);
    ASSERT_NOT_NULL(flags);
    ASSERT_EQ(len, 1);
    ASSERT_EQ(flags[0], FLOSS_AD_FLAG_GENERAL_DISC | FLOSS_AD_FLAG_NO_BREDR);
    ASSERT_NULL(floss_le_ad_find(adv, n, FLOSS_AD_APPEARANCE, &len));
    ASSERT_NOT_NULL(floss_le_ad_find(adv, n, FLOSS_AD_FLAGS, NULL)); /* out_len optional */

    /* The API takes the UUID as written; AD data carries it reversed. */
    ASSERT_TRUE(floss_le_ad_has_uuid128(adv, n, k_uuid_svc));
    ASSERT_FALSE(floss_le_ad_has_uuid128(adv, n, k_uuid_x));
    uint8_t le[16];
    be_to_le(k_uuid_svc, le);
    ASSERT_FALSE(floss_le_ad_has_uuid128(adv, n, le)); /* byte order matters */

    /* Incomplete list (0x06) with two entries: the second one matches. */
    uint8_t two[2 + 32];
    two[0] = 33;
    two[1] = 0x06;
    be_to_le(k_uuid_y, &two[2]);
    be_to_le(k_uuid_x, &two[18]);
    ASSERT_TRUE(floss_le_ad_has_uuid128(two, sizeof two, k_uuid_x));
    ASSERT_TRUE(floss_le_ad_has_uuid128(two, sizeof two, k_uuid_y));
    ASSERT_FALSE(floss_le_ad_has_uuid128(two, sizeof two, k_uuid_svc));
}

TEST(ad_find_malformed) {
    /* Second structure claims 9 bytes but only 2 remain: never read past len. */
    const uint8_t bad[] = {2, 0x01, 0x06, 9, 0x09, 'x', 'y'};
    uint8_t len;
    ASSERT_NOT_NULL(floss_le_ad_find(bad, sizeof bad, 0x01, &len));
    ASSERT_NULL(floss_le_ad_find(bad, sizeof bad, 0x09, &len));
    /* Truncated buffer view of valid data: the name no longer fits. */
    uint8_t adv[31];
    uint8_t n = pad_adv(adv);
    ASSERT_NULL(floss_le_ad_find(adv, (uint8_t)(n - 1), FLOSS_AD_NAME_SHORT, &len));
    /* UUID list cut short: no false positive from a partial entry. */
    ASSERT_FALSE(floss_le_ad_has_uuid128(adv, 20, k_uuid_svc));
    /* Zero length ends the significant part (zero padding after it). */
    const uint8_t padded[] = {2, 0x01, 0x06, 0, 0, 0, 0};
    ASSERT_NULL(floss_le_ad_find(padded, sizeof padded, 0x00, &len));
    ASSERT_NULL(floss_le_ad_find(NULL, 5, 0x01, &len));
    ASSERT_FALSE(floss_le_ad_has_uuid128(NULL, 0, k_uuid_svc));
}

/* ------------------------------------------------------------------------ */
/* Scanning                                                                 */
/* ------------------------------------------------------------------------ */

TEST(scan_reports_parsed) {
    central_start();
    add_pad();
    ASSERT_FALSE(floss_le_scanning());
    ASSERT_EQ(floss_le_scan_start(NULL, scan_cb, NULL), FLOSS_OK);
    ASSERT_TRUE(floss_le_scanning());
    ASSERT_TRUE(wait_for(&g_rep_count, 2));

    const floss_sim_le_scan_t *s = floss_sim_le_scan();
    ASSERT_TRUE(s->enabled);
    ASSERT_TRUE(s->active);
    ASSERT_FALSE(s->filter_duplicates);
    ASSERT_EQ(s->interval, 0x0030);
    ASSERT_EQ(s->window, 0x0030);
    ASSERT_EQ(s->own_addr_type, FLOSS_LE_ADDR_PUBLIC);
    ASSERT_EQ(s->filter_policy, 0);

    /* ADV_IND first, then the scan response (active scan). */
    const floss_le_adv_report_t *r = &g_rep[0].r;
    ASSERT_EQ(r->evt_type, 0);
    ASSERT_EQ(r->addr_type, FLOSS_LE_ADDR_RANDOM);
    ASSERT_TRUE(floss_addr_eq(&r->addr, &k_pad));
    ASSERT_EQ(r->rssi, -50);
    uint8_t adv[31];
    uint8_t n = pad_adv(adv);
    ASSERT_EQ(r->len, n);
    ASSERT_MEM_EQ(r->data, adv, n);
    ASSERT_TRUE(floss_le_ad_has_uuid128(r->data, r->len, k_uuid_svc));

    r = &g_rep[1].r;
    ASSERT_EQ(r->evt_type, 4);
    uint8_t len;
    const uint8_t *name = floss_le_ad_find(r->data, r->len, FLOSS_AD_NAME_COMPLETE, &len);
    ASSERT_NOT_NULL(name);
    ASSERT_EQ(len, 9);
    ASSERT_MEM_EQ(name, "FLOSS Pad", 9);

    /* Without duplicate filtering the pad keeps being reported. */
    floss_sim_pump_us(100000);
    ASSERT_GT(g_rep_count, 4);

    /* Restarting with other parameters: disable, parameters, enable (never refused). */
    floss_sim_log_clear();
    floss_le_scan_params_t p;
    floss_le_scan_params_defaults(&p);
    p.interval = 0x0060;
    p.window = 0x0020;
    ASSERT_EQ(floss_le_scan_start(&p, scan_cb, NULL), FLOSS_OK);
    floss_sim_pump(20);
    ASSERT_EQ(floss_sim_cmd_total(), 3);
    ASSERT_EQ(floss_sim_cmd_at(0)->opcode, FLOSS_OPCODE(0x08, 0x000C));
    ASSERT_EQ(floss_sim_cmd_at(0)->params[0], 0);
    ASSERT_EQ(floss_sim_cmd_at(1)->opcode, FLOSS_OPCODE(0x08, 0x000B));
    ASSERT_EQ(floss_sim_cmd_at(2)->opcode, FLOSS_OPCODE(0x08, 0x000C));
    ASSERT_EQ(floss_sim_cmd_at(2)->params[0], 1);
    ASSERT_EQ(s->interval, 0x0060);
    ASSERT_EQ(s->window, 0x0020);
    ASSERT_TRUE(s->enabled);

    /* Stop: no reports afterwards. */
    ASSERT_EQ(floss_le_scan_stop(), FLOSS_OK);
    ASSERT_FALSE(floss_le_scanning());
    floss_sim_pump(20);
    ASSERT_FALSE(s->enabled);
    unsigned before = g_rep_count;
    floss_sim_pump_us(100000);
    ASSERT_EQ(g_rep_count, before);
    ASSERT_EQ(floss_le_scan_stop(), FLOSS_OK); /* idempotent */

    /* Bad parameters are refused up front. */
    p.window = 0x0070; /* window > interval */
    ASSERT_EQ(floss_le_scan_start(&p, scan_cb, NULL), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_le_scan_start(NULL, NULL, NULL), FLOSS_ERR_INVALID);
    floss_tc_assert_sim_clean();
}

TEST(scan_passive_filter_duplicates) {
    central_start();
    add_pad();
    floss_le_scan_params_t p;
    floss_le_scan_params_defaults(&p);
    p.active = false;
    p.filter_duplicates = true;
    ASSERT_EQ(floss_le_scan_start(&p, scan_cb, NULL), FLOSS_OK);
    floss_sim_pump_us(300000);
    ASSERT_EQ(g_rep_count, 1); /* one ADV_IND, no scan response (passive) */
    ASSERT_EQ(g_rep[0].r.evt_type, 0);
    ASSERT_FALSE(floss_sim_le_scan()->active);
    ASSERT_TRUE(floss_sim_le_scan()->filter_duplicates);
    floss_tc_assert_sim_clean();
}

/** LE Advertising Report as raw LE meta parameters. */
static void inject(const uint8_t *params, uint8_t len) {
    ASSERT_EQ(floss_sim_inject_event(FLOSS_EVT_LE_META, params, len), FLOSS_OK);
    floss_sim_pump(3);
}

TEST(scan_malformed_reports_dropped) {
    central_start();
    ASSERT_EQ(floss_le_scan_start(NULL, scan_cb, NULL), FLOSS_OK);
    floss_sim_pump(20);
    ASSERT_EQ(g_rep_count, 0);

    /* Two well-formed reports in one event: both delivered, in order. */
    const uint8_t two[] = {0x02, 2, 0x00, 0x00, 1, 2, 3, 4, 5, 6, 2, 0x01, 0x06, (uint8_t)-40,
                           0x03, 0x01, 7, 8, 9, 10, 11, 0xC0, 0, (uint8_t)-70};
    inject(two, sizeof two);
    ASSERT_EQ(g_rep_count, 2);
    ASSERT_EQ(g_rep[0].r.len, 2);
    ASSERT_EQ(g_rep[0].r.rssi, -40);
    ASSERT_EQ(g_rep[1].r.evt_type, 3);
    ASSERT_EQ(g_rep[1].r.addr_type, 1);
    ASSERT_EQ(g_rep[1].r.len, 0);
    ASSERT_EQ(g_rep[1].r.rssi, -70);

    /* Data length runs past the event. */
    const uint8_t overrun[] = {0x02, 1, 0x00, 0x00, 1, 2, 3, 4, 5, 6, 20, 0x01, 0x06, 0};
    inject(overrun, sizeof overrun);
    /* Second report missing entirely: the first is not trusted either. */
    const uint8_t short2[] = {0x02, 2, 0x00, 0x00, 1, 2, 3, 4, 5, 6, 0, 0x10};
    inject(short2, sizeof short2);
    /* RSSI byte missing. */
    const uint8_t nor[] = {0x02, 1, 0x00, 0x00, 1, 2, 3, 4, 5, 6, 1, 0x02};
    inject(nor, sizeof nor);
    /* Num_Reports 0 and > 0x19; header cut short; legacy data longer than 31 bytes. */
    const uint8_t zero[] = {0x02, 0};
    inject(zero, sizeof zero);
    const uint8_t many[] = {0x02, 0x1A, 0, 0, 1, 2, 3, 4, 5, 6, 0, 0};
    inject(many, sizeof many);
    const uint8_t cut[] = {0x02, 1, 0x00, 0x00, 1, 2, 3};
    inject(cut, sizeof cut);
    uint8_t big[2 + 9 + 32 + 1] = {0x02, 1, 0x00, 0x00, 1, 2, 3, 4, 5, 6, 32};
    inject(big, sizeof big);
    const uint8_t empty[] = {0x02};
    inject(empty, sizeof empty);
    ASSERT_EQ(g_rep_count, 2);
    floss_tc_assert_sim_clean();
}

/* ------------------------------------------------------------------------ */
/* Connecting                                                               */
/* ------------------------------------------------------------------------ */

TEST(connect_as_central) {
    central_start();
    add_pad();
    ASSERT_EQ(floss_le_scan_start(NULL, scan_cb, NULL), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_rep_count, 1));

    /* Connecting stops scanning first: the sim refuses Create Connection while scanning. */
    floss_sim_log_clear();
    floss_le_connect_params_t cp;
    floss_le_connect_params_defaults(&cp);
    ASSERT_EQ(cp.conn.interval_min, 6);
    ASSERT_EQ(cp.conn.timeout, 200);
    ASSERT_EQ(floss_le_connect(g_rep[0].r.addr_type, &g_rep[0].r.addr, &cp), FLOSS_OK);
    ASSERT_FALSE(floss_le_scanning());
    ASSERT_EQ(floss_le_connect(FLOSS_LE_ADDR_RANDOM, &k_pad, NULL), FLOSS_ERR_BUSY);
    for (unsigned i = 0; i < 100 && !le_last(FLOSS_LE_EVT_CONNECTED); i++) {
        floss_sim_pump(1);
    }
    const floss_le_evt_t *e = le_last(FLOSS_LE_EVT_CONNECTED);
    ASSERT_NOT_NULL(e);
    ASSERT_EQ(e->role, 0);
    ASSERT_TRUE(floss_addr_eq(&e->addr, &k_pad));
    ASSERT_EQ(e->addr_type, FLOSS_LE_ADDR_RANDOM);
    ASSERT_EQ(le_count(FLOSS_LE_EVT_CONNECT_FAILED), 0);

    ASSERT_EQ(floss_sim_cmd_at(0)->opcode, FLOSS_OPCODE(0x08, 0x000C)); /* scan off ... */
    ASSERT_EQ(floss_sim_cmd_at(0)->params[0], 0);
    const floss_sim_cmd_t *cc = floss_sim_cmd_at(1); /* ... then create connection */
    ASSERT_EQ(cc->opcode, FLOSS_OPCODE(0x08, 0x000D));
    ASSERT_EQ(cc->len, 25);
    ASSERT_EQ(floss_get_le16(&cc->params[0]), 0x0030); /* scan interval */
    ASSERT_EQ(floss_get_le16(&cc->params[2]), 0x0030); /* scan window */
    ASSERT_EQ(cc->params[4], 0);                       /* filter policy: peer address */
    ASSERT_EQ(cc->params[5], FLOSS_LE_ADDR_RANDOM);
    ASSERT_MEM_EQ(&cc->params[6], k_pad.b, 6);
    ASSERT_EQ(cc->params[12], FLOSS_LE_ADDR_PUBLIC); /* own address type */
    ASSERT_EQ(floss_get_le16(&cc->params[13]), 6);
    ASSERT_EQ(floss_get_le16(&cc->params[15]), 6);
    ASSERT_EQ(floss_get_le16(&cc->params[17]), 0);
    ASSERT_EQ(floss_get_le16(&cc->params[19]), 200);

    /* The link as the LE module sees it: central, default MTU, HCI table agrees. */
    const floss_le_link_t *l = floss_le_link(e->handle);
    ASSERT_NOT_NULL(l);
    ASSERT_EQ(l->role, 0);
    ASSERT_EQ(l->att_mtu, FLOSS_ATT_DEFAULT_MTU);
    ASSERT_EQ(l->interval, 6);
    const floss_conn_t *c = floss_hci_conn_by_handle(e->handle);
    ASSERT_NOT_NULL(c);
    ASSERT_TRUE(c->le);
    ASSERT_EQ(c->role, 0);
    ASSERT_TRUE(c->initiator);
    const floss_sim_link_t *sl = floss_sim_link(e->handle);
    ASSERT_NOT_NULL(sl);
    ASSERT_EQ(sl->role, 0);

    /* As central the stack never asks the peer for parameters (peripheral latency policy). */
    floss_sim_pump_us(2500000);
    uint8_t tmp[64];
    ASSERT_EQ(floss_sim_fixed_recv(e->handle, FLOSS_SIM_CID_LE_SIG, tmp, sizeof tmp), -1);
    ASSERT_EQ(floss_sim_cmd_count(FLOSS_HCI_LE_CONN_UPDATE), 0);
    ASSERT_NOT_NULL(floss_le_link(e->handle));

    /* Changing parameters as central: LE Connection Update directly. */
    floss_le_conn_params_t np = {.interval_min = 12, .interval_max = 12, .latency = 0, .timeout = 300};
    ASSERT_EQ(floss_le_request_conn_params(e->handle, &np), FLOSS_OK);
    floss_sim_pump(10);
    ASSERT_EQ(floss_le_link(e->handle)->interval, 12);

    /* Scanning again while connected works; the pad stopped advertising when the link was created. */
    unsigned seen = g_rep_count;
    ASSERT_EQ(floss_le_scan_start(NULL, scan_cb, NULL), FLOSS_OK);
    floss_sim_pump_us(100000);
    ASSERT_TRUE(floss_sim_le_scan()->enabled);
    ASSERT_EQ(g_rep_count, seen);
    floss_tc_assert_sim_clean();
}

TEST(connect_cancel_and_refusal) {
    central_start();
    static const floss_addr_t ghost = {{0x01, 0x02, 0x03, 0x04, 0x05, 0x06}};
    ASSERT_EQ(floss_le_connect_cancel(), FLOSS_ERR_STATE);
    ASSERT_EQ(floss_le_connect(FLOSS_LE_ADDR_PUBLIC, &ghost, NULL), FLOSS_OK);
    floss_sim_pump(10);
    ASSERT_TRUE(floss_sim_le_initiating());
    ASSERT_EQ(floss_le_scan_start(NULL, scan_cb, NULL), FLOSS_ERR_BUSY);
    ASSERT_EQ(floss_le_connect_cancel(), FLOSS_OK);
    floss_sim_pump(10);
    ASSERT_FALSE(floss_sim_le_initiating());
    const floss_le_evt_t *e = le_last(FLOSS_LE_EVT_CONNECT_FAILED);
    ASSERT_NOT_NULL(e);
    ASSERT_EQ(e->status, FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
    ASSERT_EQ(le_count(FLOSS_LE_EVT_CONNECTED), 0);

    /* Not busy any more; a controller refusal (Command Status) reports CONNECT_FAILED too.
     * Scanning switched on behind the module's back makes the sim refuse. */
    const uint8_t on[2] = {1, 0};
    ASSERT_EQ(floss_hci_cmd(FLOSS_OPCODE(0x08, 0x000C), on, 2, NULL, NULL), FLOSS_OK);
    floss_sim_pump(5);
    ASSERT_TRUE(floss_sim_le_scan()->enabled);
    ASSERT_EQ(floss_le_connect(FLOSS_LE_ADDR_PUBLIC, &ghost, NULL), FLOSS_OK);
    floss_sim_pump(10);
    ASSERT_EQ(le_count(FLOSS_LE_EVT_CONNECT_FAILED), 2);
    e = le_last(FLOSS_LE_EVT_CONNECT_FAILED);
    ASSERT_EQ(e->status, 0x0C); /* Command Disallowed */
    ASSERT_TRUE(floss_addr_eq(&e->addr, &ghost));
    ASSERT_EQ(floss_le_connect_cancel(), FLOSS_ERR_STATE);

    /* Parameter validation. */
    floss_le_connect_params_t cp;
    floss_le_connect_params_defaults(&cp);
    cp.conn.timeout = 1; /* below the minimum and shorter than the interval allows */
    ASSERT_EQ(floss_le_connect(FLOSS_LE_ADDR_PUBLIC, &ghost, &cp), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_le_connect(4, &ghost, NULL), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_le_connect(0, NULL, NULL), FLOSS_ERR_INVALID);
    floss_tc_assert_sim_clean();
}

TEST(set_phy_as_central) {
    central_start();
    uint16_t h = central_connect();
    ASSERT_EQ(floss_le_set_phy(0x0EEE, 0x02, 0x02), FLOSS_ERR_NOT_FOUND);
    ASSERT_EQ(floss_le_set_phy(h, 0x08, 0x02), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_le_set_phy(h, 0x02, 0x02), FLOSS_OK);
    floss_sim_pump(10);
    const floss_le_evt_t *e = le_last(FLOSS_LE_EVT_PHY);
    ASSERT_NOT_NULL(e);
    ASSERT_EQ(e->status, 0);
    ASSERT_EQ(e->tx_phy, 2);
    ASSERT_EQ(e->rx_phy, 2);
    ASSERT_EQ(floss_le_link(h)->tx_phy, 2);
    const floss_sim_cmd_t *c = floss_sim_cmd_last(FLOSS_HCI_LE_SET_PHY);
    ASSERT_NOT_NULL(c);
    ASSERT_EQ(c->params[2], 0); /* both directions have a preference */
    /* tx_phys 0 = no preference (ALL_PHYS bit 0). */
    ASSERT_EQ(floss_le_set_phy(h, 0, 0x01), FLOSS_OK);
    floss_sim_pump(10);
    c = floss_sim_cmd_last(FLOSS_HCI_LE_SET_PHY);
    ASSERT_EQ(c->params[2], 0x01);
    ASSERT_EQ(floss_le_link(h)->rx_phy, 1);
    floss_tc_assert_sim_clean();
}

/* ------------------------------------------------------------------------ */
/* GATT client                                                              */
/* ------------------------------------------------------------------------ */

TEST(gattc_find_two_rounds_at_default_mtu) {
    uint16_t h = gatt_session(247);
    ASSERT_EQ(floss_gattc_find_char128(h, k_uuid_x, char_cb, NULL), FLOSS_OK);
    ASSERT_TRUE(floss_gattc_busy(h));
    ASSERT_EQ(floss_gattc_find_char128(h, k_uuid_x, char_cb, NULL), FLOSS_ERR_BUSY);
    ASSERT_EQ(floss_gattc_read(h, 3, read_cb, NULL), FLOSS_ERR_BUSY);
    ASSERT_TRUE(wait_for(&g_char.calls, 1));
    ASSERT_FALSE(floss_gattc_busy(h));
    ASSERT_EQ(g_char.err, 0);
    ASSERT_EQ(g_char.vh, 9);
    ASSERT_EQ(g_char.cccd, 11);
    ASSERT_EQ(g_char.props, g_decl8[0]);
    /* Two Read By Type rounds (16-bit declarations, then the 128-bit one) + one Find Information. */
    ASSERT_EQ(floss_sim_gatt_stats()->requests, 3);
    ASSERT_EQ(floss_sim_gatt_stats()->mtu, 0); /* never exchanged: 23 */

    /* A characteristic that cannot notify: no CCCD search. */
    ASSERT_EQ(floss_gattc_find_char128(h, k_uuid_y, char_cb, NULL), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_char.calls, 2));
    ASSERT_EQ(g_char.err, 0);
    ASSERT_EQ(g_char.vh, 13);
    ASSERT_EQ(g_char.cccd, 0);
    ASSERT_EQ(g_char.props, FLOSS_GATT_PROP_READ);

    /* 16-bit UUID: found in the first response, CCCD right after the value. */
    ASSERT_EQ(floss_gattc_find_char16(h, 0x2A19, char_cb, NULL), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_char.calls, 3));
    ASSERT_EQ(g_char.err, 0);
    ASSERT_EQ(g_char.vh, 5);
    ASSERT_EQ(g_char.cccd, 6);

    /* Absent UUID: Attribute Not Found after walking the whole server. */
    ASSERT_EQ(floss_gattc_find_char16(h, 0x1234, char_cb, NULL), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_char.calls, 4));
    ASSERT_EQ(g_char.err, FLOSS_ATT_ERR_ATTR_NOT_FOUND);
    ASSERT_EQ(g_char.vh, 0);
    ASSERT_EQ(floss_sim_gatt_stats()->pipelined, 0);
    floss_tc_assert_sim_clean();
}

TEST(gattc_mtu_subscribe_notify_indicate_write) {
    uint16_t h = gatt_session(247);
    ASSERT_EQ(floss_gattc_exchange_mtu(h, done_cb, NULL), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_done.calls, 1));
    ASSERT_EQ(g_done.err, 0);
    ASSERT_EQ(g_done.conn, h);
    ASSERT_EQ(floss_le_link(h)->att_mtu, FLOSS_MIN(247, FLOSS_ATT_MTU));
    ASSERT_EQ(floss_sim_gatt_stats()->mtu, FLOSS_MIN(247, FLOSS_ATT_MTU));
    const floss_le_evt_t *m = le_last(FLOSS_LE_EVT_MTU);
    ASSERT_NOT_NULL(m);
    ASSERT_EQ(m->mtu, FLOSS_MIN(247, FLOSS_ATT_MTU));
    ASSERT_EQ(floss_gattc_exchange_mtu(h, done_cb, NULL), FLOSS_ERR_STATE); /* once per connection */

    /* With the larger MTU both 128-bit declarations share a response: the
     * next one bounds the descriptor search. */
    unsigned reqs = floss_sim_gatt_stats()->requests;
    ASSERT_EQ(floss_gattc_find_char128(h, k_uuid_x, char_cb, NULL), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_char.calls, 1));
    ASSERT_EQ(g_char.err, 0);
    ASSERT_EQ(g_char.vh, 9);
    ASSERT_EQ(g_char.cccd, 11);
    ASSERT_EQ(floss_sim_gatt_stats()->requests - reqs, 3);

    /* Subscribe = Write Request of the CCCD. */
    ASSERT_EQ(floss_gattc_subscribe(h, g_char.cccd, FLOSS_GATT_CCCD_NOTIFY | FLOSS_GATT_CCCD_INDICATE, done_cb, NULL),
              FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_done.calls, 2));
    ASSERT_EQ(g_done.err, 0);
    uint8_t v[8];
    ASSERT_EQ(floss_sim_gatt_value(11, v, sizeof v), 2);
    ASSERT_EQ(v[0], 0x03);
    ASSERT_EQ(v[1], 0x00);

    /* Notification -> handler. */
    const uint8_t report[] = {0xDE, 0xAD, 0xBE, 0xEF};
    ASSERT_EQ(floss_sim_gatt_notify(9, report, sizeof report, false), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_notify.calls, 1));
    ASSERT_EQ(g_notify.conn, h);
    ASSERT_EQ(g_notify.handle, 9);
    ASSERT_EQ(g_notify.len, 4);
    ASSERT_MEM_EQ(g_notify.data, report, 4);
    ASSERT_EQ(g_notify.indications, 0);
    ASSERT_EQ(floss_sim_gatt_stats()->confirmations, 0);

    /* Indication -> handler + Handle Value Confirmation. */
    const uint8_t ind[] = {0x42, 0x43};
    ASSERT_EQ(floss_sim_gatt_notify(9, ind, sizeof ind, true), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_notify.calls, 2));
    ASSERT_EQ(g_notify.indications, 1);
    ASSERT_EQ(g_notify.len, 2);
    ASSERT_MEM_EQ(g_notify.data, ind, 2);
    floss_sim_pump(5);
    ASSERT_EQ(floss_sim_gatt_stats()->confirmations, 1);

    /* Write Command: exact bytes on the wire, no callback wait, allowed while busy. */
    drain_att(h);
    ASSERT_EQ(floss_gattc_read(h, 13, read_cb, NULL), FLOSS_OK); /* a request in flight */
    const uint8_t cmd[] = {1, 2, 3, 4, 5};
    unsigned done_before = g_done.calls;
    ASSERT_EQ(floss_gattc_write(h, 9, cmd, sizeof cmd, false, done_cb, NULL), FLOSS_OK);
    ASSERT_EQ(g_done.calls, done_before + 1); /* handed to HCI */
    ASSERT_TRUE(wait_for(&g_read.calls, 1));
    uint8_t pdu[64];
    ASSERT_EQ(floss_sim_att_recv(h, pdu, sizeof pdu), 3); /* Read Request */
    ASSERT_EQ(pdu[0], FLOSS_ATT_READ_REQ);
    ASSERT_EQ(floss_sim_att_recv(h, pdu, sizeof pdu), 8);
    const uint8_t want[] = {0x52, 0x09, 0x00, 1, 2, 3, 4, 5};
    ASSERT_MEM_EQ(pdu, want, sizeof want);
    ASSERT_EQ(floss_sim_gatt_stats()->write_cmds, 1);
    ASSERT_EQ(floss_sim_gatt_value(9, v, sizeof v), 5);
    ASSERT_MEM_EQ(v, cmd, 5);
    ASSERT_EQ(g_read.err, 0);
    ASSERT_EQ(g_read.len, 5);
    ASSERT_MEM_EQ(g_read.data, "hello", 5);

    /* Write Request with a completion. */
    const uint8_t wr[] = {9, 8, 7};
    ASSERT_EQ(floss_gattc_write(h, 9, wr, sizeof wr, true, done_cb, NULL), FLOSS_OK);
    unsigned target = g_done.calls + 1;
    ASSERT_TRUE(wait_for(&g_done.calls, target));
    ASSERT_EQ(g_done.err, 0);
    ASSERT_EQ(floss_sim_gatt_stats()->writes, 2); /* CCCD + this */
    ASSERT_EQ(floss_sim_gatt_stats()->last_write_handle, 9);

    /* Larger than one PDU at the negotiated MTU: refused. */
    static uint8_t huge[FLOSS_ATT_MTU];
    ASSERT_EQ(floss_gattc_write(h, 9, huge, (uint16_t)(FLOSS_MIN(247, FLOSS_ATT_MTU) - 2), true, done_cb, NULL),
              FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_sim_gatt_stats()->pipelined, 0);
    floss_tc_assert_sim_clean();
}

TEST(gattc_error_responses) {
    uint16_t h = gatt_session(23);
    /* Server rejects the write: callback carries the ATT error. */
    const uint8_t v[] = {1};
    ASSERT_EQ(floss_gattc_write(h, 14, v, 1, true, done_cb, NULL), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_done.calls, 1));
    ASSERT_EQ(g_done.err, FLOSS_ATT_ERR_INSUFFICIENT_AUTHEN);
    ASSERT_EQ(floss_gattc_read(h, 14, read_cb, NULL), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_read.calls, 1));
    ASSERT_EQ(g_read.err, FLOSS_ATT_ERR_INSUFFICIENT_AUTHEN);
    ASSERT_EQ(g_read.len, 0);
    ASSERT_EQ(floss_gattc_read(h, 0x0050, read_cb, NULL), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_read.calls, 2));
    ASSERT_EQ(g_read.err, FLOSS_ATT_ERR_INVALID_HANDLE);

    /* Scripted by hand: an Error Response for another request is ignored, the right response completes. */
    floss_sim_gatt_silence(true);
    ASSERT_EQ(floss_gattc_read(h, 3, read_cb, NULL), FLOSS_OK);
    floss_sim_pump(5);
    const uint8_t stray[] = {0x01, 0x12, 0x03, 0x00, 0x03};
    ASSERT_EQ(floss_sim_att_send(h, stray, sizeof stray), FLOSS_OK);
    floss_sim_pump(5);
    ASSERT_EQ(g_read.calls, 2);
    ASSERT_TRUE(floss_gattc_busy(h));
    const uint8_t rsp[] = {0x0B, 'P', 'A', 'D'};
    ASSERT_EQ(floss_sim_att_send(h, rsp, sizeof rsp), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_read.calls, 3));
    ASSERT_EQ(g_read.err, 0);
    ASSERT_EQ(g_read.len, 3);
    /* A response nobody asked for is dropped (and not answered by our server). */
    ASSERT_EQ(floss_sim_att_send(h, rsp, sizeof rsp), FLOSS_OK);
    floss_sim_pump(5);
    ASSERT_EQ(g_read.calls, 3);

    /* Malformed discovery response: pair length 9 is neither 7 nor 21. */
    ASSERT_EQ(floss_gattc_find_char16(h, 0x2A19, char_cb, NULL), FLOSS_OK);
    floss_sim_pump(5);
    const uint8_t bad[] = {0x09, 0x09, 0x02, 0x00, 0x10, 0x03, 0x00, 0x19, 0x2A, 0x00, 0x00};
    ASSERT_EQ(floss_sim_att_send(h, bad, sizeof bad), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_char.calls, 1));
    ASSERT_EQ(g_char.err, FLOSS_ATT_ERR_INVALID_PDU);

    /* Handles going backwards are refused too (no endless discovery loops). */
    ASSERT_EQ(floss_gattc_find_char16(h, 0x2A19, char_cb, NULL), FLOSS_OK);
    floss_sim_pump(5);
    const uint8_t first[] = {0x09, 0x07, 0x02, 0x00, 0x02, 0x03, 0x00, 0x00, 0x2A};
    ASSERT_EQ(floss_sim_att_send(h, first, sizeof first), FLOSS_OK);
    floss_sim_pump(5);
    const uint8_t back[] = {0x09, 0x07, 0x02, 0x00, 0x02, 0x03, 0x00, 0x00, 0x2A}; /* handle 2 again */
    ASSERT_EQ(floss_sim_att_send(h, back, sizeof back), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_char.calls, 2));
    ASSERT_EQ(g_char.err, FLOSS_ATT_ERR_INVALID_PDU);

    /* Notifications too short to carry a handle are dropped; the link stays healthy. */
    const uint8_t runt[] = {0x1B, 0x09};
    ASSERT_EQ(floss_sim_att_send(h, runt, sizeof runt), FLOSS_OK);
    floss_sim_pump(5);
    ASSERT_EQ(g_notify.calls, 0);
    floss_sim_gatt_silence(false);
    ASSERT_EQ(floss_gattc_read(h, 5, read_cb, NULL), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_read.calls, 4));
    ASSERT_EQ(g_read.err, 0);
    ASSERT_EQ(g_read.data[0], 100);
    ASSERT_EQ(floss_sim_gatt_stats()->pipelined, 0);
    floss_tc_assert_sim_clean();
}

TEST(gattc_timeout_closes_link) {
    uint16_t h = gatt_session(23);
    floss_sim_gatt_silence(true);
    ASSERT_EQ(floss_gattc_read(h, 3, read_cb, NULL), FLOSS_OK);
    floss_sim_pump_us(29000000u / 1000u); /* well before the deadline: nothing yet */
    ASSERT_EQ(g_read.calls, 0);
    floss_sim_config_t *cfg = floss_sim_cfg();
    uint32_t step = cfg->pump_step_us;
    cfg->pump_step_us = 100000; /* 100 ms per poll to cover 30 s quickly */
    for (unsigned i = 0; i < 400 && g_read.calls == 0; i++) {
        floss_sim_pump(1);
    }
    cfg->pump_step_us = step;
    ASSERT_EQ(g_read.calls, 1);
    ASSERT_EQ(g_read.err, 0xFF);
    /* No more ATT on that bearer: the link is closed. */
    floss_sim_pump(20);
    const floss_le_evt_t *d = le_last(FLOSS_LE_EVT_DISCONNECTED);
    ASSERT_NOT_NULL(d);
    ASSERT_EQ(d->handle, h);
    ASSERT_NULL(floss_le_link(h));
    ASSERT_EQ(floss_gattc_read(h, 3, read_cb, NULL), FLOSS_ERR_NOT_FOUND);
    ASSERT_EQ(g_read.calls, 1);
    floss_tc_assert_sim_clean();
}

TEST(gattc_link_loss_fails_procedure) {
    uint16_t h = gatt_session(23);
    floss_sim_gatt_silence(true);
    ASSERT_EQ(floss_gattc_find_char128(h, k_uuid_x, char_cb, NULL), FLOSS_OK);
    floss_sim_pump(5);
    ASSERT_EQ(floss_sim_remote_disconnect(h, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_char.calls, 1));
    ASSERT_EQ(g_char.err, 0xFF);
    ASSERT_FALSE(floss_gattc_busy(h));
    ASSERT_NOT_NULL(le_last(FLOSS_LE_EVT_DISCONNECTED));

    /* A new central link to the same peer starts clean. */
    uint16_t h2 = central_connect();
    ASSERT_NE(h2, h);
    build_db();
    ASSERT_EQ(floss_sim_gatt_serve(h2, g_db, (uint8_t)FLOSS_ARRAY_SIZE(g_db), 64), FLOSS_OK);
    ASSERT_EQ(floss_gattc_exchange_mtu(h2, done_cb, NULL), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_done.calls, 1));
    ASSERT_EQ(g_done.err, 0);
    ASSERT_EQ(floss_le_link(h2)->att_mtu, 64);
    floss_tc_assert_sim_clean();
}

TEST(gattc_and_server_share_a_link) {
    uint16_t h = gatt_session(23);
    /* Client request outstanding while the peer queries OUR server. */
    floss_sim_gatt_silence(true);
    ASSERT_EQ(floss_gattc_read(h, 13, read_cb, NULL), FLOSS_OK);
    floss_sim_pump(5);
    drain_att(h);
    const uint8_t req[] = {0x0A, 0x01, 0x00}; /* Read our handle 1: GAP primary service declaration */
    ASSERT_EQ(floss_sim_att_send(h, req, sizeof req), FLOSS_OK);
    floss_sim_pump(10);
    uint8_t rsp[32];
    ASSERT_EQ(floss_sim_att_recv(h, rsp, sizeof rsp), 3);
    ASSERT_EQ(rsp[0], FLOSS_ATT_READ_RSP);
    ASSERT_EQ(floss_get_le16(&rsp[1]), FLOSS_GATT_SVC_GAP);
    ASSERT_TRUE(floss_gattc_busy(h)); /* the server exchange did not disturb the client */
    const uint8_t answer[] = {0x0B, 'o', 'k'};
    ASSERT_EQ(floss_sim_att_send(h, answer, sizeof answer), FLOSS_OK);
    ASSERT_TRUE(wait_for(&g_read.calls, 1));
    ASSERT_EQ(g_read.err, 0);
    ASSERT_EQ(g_read.len, 2);
    floss_tc_assert_sim_clean();
}

TEST(gattc_argument_checks) {
    central_start();
    ASSERT_EQ(floss_gattc_read(0x0123, 3, read_cb, NULL), FLOSS_ERR_NOT_FOUND);
    ASSERT_EQ(floss_gattc_exchange_mtu(0x0123, done_cb, NULL), FLOSS_ERR_NOT_FOUND);
    ASSERT_EQ(floss_gattc_write(0x0123, 3, NULL, 0, true, done_cb, NULL), FLOSS_ERR_NOT_FOUND);
    ASSERT_FALSE(floss_gattc_busy(0x0123));
    uint16_t h = central_connect();
    ASSERT_EQ(floss_gattc_read(h, 0, read_cb, NULL), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_gattc_read(h, 3, NULL, NULL), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_gattc_find_char16(h, 0x2A19, NULL, NULL), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_gattc_find_char128(h, NULL, char_cb, NULL), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_gattc_write(h, 3, NULL, 2, true, done_cb, NULL), FLOSS_ERR_INVALID);
    /* At MTU 23 a value of 21 bytes no longer fits one PDU. */
    uint8_t big[21] = {0};
    ASSERT_EQ(floss_gattc_write(h, 3, big, 21, false, NULL, NULL), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_gattc_write(h, 3, big, 20, false, NULL, NULL), FLOSS_OK);
    floss_tc_assert_sim_clean();
}

static const floss_test_case_t tests[] = {
    TEST_CASE(ad_find_and_uuid128),
    TEST_CASE(ad_find_malformed),
    TEST_CASE(scan_reports_parsed),
    TEST_CASE(scan_passive_filter_duplicates),
    TEST_CASE(scan_malformed_reports_dropped),
    TEST_CASE(connect_as_central),
    TEST_CASE(connect_cancel_and_refusal),
    TEST_CASE(set_phy_as_central),
    TEST_CASE(gattc_find_two_rounds_at_default_mtu),
    TEST_CASE(gattc_mtu_subscribe_notify_indicate_write),
    TEST_CASE(gattc_error_responses),
    TEST_CASE(gattc_timeout_closes_link),
    TEST_CASE(gattc_link_loss_fails_procedure),
    TEST_CASE(gattc_and_server_share_a_link),
    TEST_CASE(gattc_argument_checks),
};

#else /* !FLOSS_ENABLE_LE */

TEST(le_disabled) {
}

static const floss_test_case_t tests[] = {
    TEST_CASE(le_disabled),
};

#endif

FLOSS_TEST_MAIN("le_central", tests)
