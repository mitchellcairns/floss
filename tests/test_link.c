/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * test_link.c - The pad <-> dongle link profile (floss_link.h) against the
 * simulator, plus the GAP discovery pieces it relies on.
 *
 * The far side is scripted by the test: when the stack is the pad, the sim
 * remote plays the dongle (pages, pairs, opens PSM 0x1001, sends OUTPUT /
 * PING PDUs); when the stack is the dongle, the sim plays the pad (answers
 * the inquiry with an EIR, accepts the page, serves the PSM, sends INPUT
 * PDUs). The LE cases use the sim's LE link, its advertisers and scripted
 * GATT server; encryption is injected (the sim has no SMP peer).
 */
#include "test_common.h"
#include "link/floss_link_priv.h"
#include "gap/floss_gap_internal.h"

#include <stdio.h>

/* ------------------------------------------------------------------------ */
/* Fixtures                                                                 */
/* ------------------------------------------------------------------------ */

/** The remote pad / dongle. */
static const floss_addr_t k_peer = {{0x01, 0x00, 0x00, 0x5D, 0x05, 0xF1}};
/** A second remote that is not our peer. */
static const floss_addr_t k_stranger = {{0x02, 0x00, 0x00, 0x5D, 0x05, 0xF1}};

static floss_link_config_t g_cfg;
static floss_transport_t *g_transport;

#define GAP_LOG 128
static struct {
    floss_gap_evt_t evt[GAP_LOG];
    unsigned count;
} g_gap;

static void gap_rec(const floss_gap_evt_t *evt, void *user) {
    (void)user;
    if (g_gap.count < GAP_LOG) {
        g_gap.evt[g_gap.count] = *evt;
        g_gap.evt[g_gap.count].eir = NULL; /* only valid during the callback */
    }
    g_gap.count++;
}

static unsigned gap_count(floss_gap_evt_type_t type) {
    unsigned n = 0;
    for (unsigned i = 0; i < FLOSS_MIN(g_gap.count, (unsigned)GAP_LOG); i++) {
        n += g_gap.evt[i].type == type;
    }
    return n;
}

/* What the link callbacks saw. */
static struct {
    unsigned connected, disconnected, ticks;
    uint8_t reason;
    floss_addr_t peer;
    unsigned inputs;
    uint8_t in_id, in_len, in_data[64];
    floss_link_rx_info_t in_info;
    uint32_t lost_sum;
    unsigned outputs;
    uint8_t out_id, out_len, out_data[64];
} g_cb;

static void cb_connected(const floss_addr_t *peer) {
    g_cb.connected++;
    g_cb.peer = *peer;
}
static void cb_disconnected(uint8_t reason) {
    g_cb.disconnected++;
    g_cb.reason = reason;
}
static void cb_input(uint8_t id, const uint8_t *data, uint8_t len, const floss_link_rx_info_t *info) {
    g_cb.inputs++;
    g_cb.in_id = id;
    g_cb.in_len = len;
    memcpy(g_cb.in_data, data, len);
    g_cb.in_info = *info;
    g_cb.lost_sum += info->lost;
}
static void cb_output(uint8_t id, const uint8_t *data, uint8_t len) {
    g_cb.outputs++;
    g_cb.out_id = id;
    g_cb.out_len = len;
    memcpy(g_cb.out_data, data, len);
}
static uint32_t g_tick_us[64];

static void cb_tick(void) {
    if (g_cb.ticks < 64) {
        g_tick_us[g_cb.ticks] = floss_hal_time_us();
    }
    g_cb.ticks++;
}

static const floss_link_callbacks_t k_cb = {
    .on_connected = cb_connected,
    .on_disconnected = cb_disconnected,
    .on_input = cb_input,
    .on_output = cb_output,
    .on_report_tick = cb_tick,
};

static bool cond_ready(void *arg) {
    (void)arg;
    return gap_count(FLOSS_GAP_EVT_READY) > 0;
}

/** floss_init + link init (tweak may adjust the config) + start, until READY. */
static void start_link(floss_link_role_t role, floss_link_transport_t tp, void (*tweak)(floss_link_config_t *)) {
    memset(&g_gap, 0, sizeof g_gap);
    memset(&g_cb, 0, sizeof g_cb);
    g_transport = floss_tc_sim_init(NULL);
    ASSERT_EQ(floss_init(&(floss_init_t){.transport = g_transport}), FLOSS_OK);
    ASSERT_EQ(floss_gap_add_listener(gap_rec, NULL), FLOSS_OK);
    floss_link_config_defaults(&g_cfg, role, tp);
    if (tweak) {
        tweak(&g_cfg);
    }
    ASSERT_EQ(floss_link_init(&g_cfg, &k_cb), FLOSS_OK);
    ASSERT_EQ(floss_start(), FLOSS_OK);
    ASSERT_TRUE(floss_sim_pump_until(cond_ready, NULL, 5000));
    floss_sim_pump(20);
}

static bool cond_connected(void *arg) {
    (void)arg;
    return floss_link_connected();
}

static bool cond_state(void *arg) {
    return floss_link_state() == (floss_link_state_t)(uintptr_t)arg;
}

static bool wait_state(floss_link_state_t st, uint32_t polls) {
    return floss_sim_pump_until(cond_state, (void *)(uintptr_t)st, polls);
}

static bool cond_link_open(void *arg) {
    const floss_sim_link_t *l = floss_sim_link((uint16_t)(uintptr_t)arg);
    return l && l->state == FLOSS_SIM_LINK_OPEN;
}

static bool cond_encrypted(void *arg) {
    const floss_sim_link_t *l = floss_sim_link((uint16_t)(uintptr_t)arg);
    return l && l->encrypted;
}

/** Pad side: the sim dongle pages us, pairs and opens the link PSM. */
static uint16_t dongle_connects(const floss_addr_t *who, floss_sim_chan_t **chan) {
    uint16_t h = floss_sim_remote_connect(who, 0x000104);
    ASSERT_NE(h, FLOSS_SIM_NO_HANDLE);
    ASSERT_TRUE(floss_sim_pump_until(cond_link_open, (void *)(uintptr_t)h, 500));
    floss_sim_pump(5);
    ASSERT_EQ(floss_sim_remote_pair(h), FLOSS_OK);
    ASSERT_TRUE(floss_sim_pump_until(cond_encrypted, (void *)(uintptr_t)h, 1000));
    floss_sim_pump(5);
    *chan = floss_sim_l2cap_connect(h, FLOSS_LINK_PSM);
    ASSERT_NOT_NULL(*chan);
    return h;
}

/** Receive one PDU from the stack (fails the test on timeout). */
static int recv_pdu(floss_sim_chan_t *ch, uint8_t *out, uint16_t cap) {
    int n = floss_sim_l2cap_recv_wait(ch, out, cap);
    if (n < 0) {
        FAIL("no PDU from the stack");
    }
    return n;
}

/** Receive PDUs until one of `type` arrives. */
static int recv_type(floss_sim_chan_t *ch, uint8_t type, uint8_t *out, uint16_t cap) {
    for (int i = 0; i < 64; i++) {
        int n = recv_pdu(ch, out, cap);
        if (n > 0 && out[0] == type) {
            return n;
        }
    }
    FAIL("no PDU of type 0x%02x", type);
}

static void build_input(uint8_t *p, uint16_t seq, uint32_t t_pad, uint8_t id, const uint8_t *d, uint8_t len) {
    p[0] = FLOSS_LINK_PDU_INPUT;
    floss_put_le16(&p[1], seq);
    floss_put_le32(&p[3], t_pad);
    p[7] = id;
    p[8] = len;
    memcpy(&p[9], d, len);
}

/* ------------------------------------------------------------------------ */
/* Configuration                                                            */
/* ------------------------------------------------------------------------ */

TEST(config_defaults) {
    floss_link_config_t c;
    floss_link_config_defaults(&c, FLOSS_LINK_ROLE_PAD, FLOSS_LINK_CLASSIC);
    ASSERT_EQ(c.report_interval_us, 1000);
    ASSERT_EQ(c.max_inflight, 2);
    ASSERT_EQ(c.classic_poll_us, 1250);
    ASSERT_EQ(c.classic_idle_ms, 0);
    ASSERT_EQ(c.classic_idle_sniff_slots, 16);
    ASSERT_EQ(c.le_interval_min, 6);
    ASSERT_EQ(c.le_interval_max, 6);
    ASSERT_EQ(c.le_peripheral_latency, 0);
    ASSERT_EQ(c.le_supervision_timeout, 200);
    ASSERT_TRUE(c.le_2m_phy);
    ASSERT_EQ(c.ping_period_ms, 0);
    ASSERT_TRUE(c.auto_reconnect);
    ASSERT_FALSE(c.classic_pad_connects);
    ASSERT_TRUE(c.classic_store_keys);
    ASSERT_NULL(c.peer_name);
    ASSERT_NOT_NULL(c.name);
    floss_link_config_defaults(&c, FLOSS_LINK_ROLE_PAD, FLOSS_LINK_LE);
    ASSERT_EQ(c.report_interval_us, FLOSS_LINK_INTERVAL_CONN);
    floss_link_config_defaults(&c, FLOSS_LINK_ROLE_DONGLE, FLOSS_LINK_CLASSIC);
    ASSERT_NULL(c.name);
    ASSERT_EQ(c.report_interval_us, 0);

    /* Invalid configurations are refused. */
    floss_transport_t *t = floss_tc_sim_init(NULL);
    ASSERT_EQ(floss_init(&(floss_init_t){.transport = t}), FLOSS_OK);
    floss_link_config_defaults(&g_cfg, FLOSS_LINK_ROLE_PAD, FLOSS_LINK_CLASSIC);
    g_cfg.classic_idle_ms = 100;
    g_cfg.classic_idle_sniff_slots = 15; /* sniff intervals are even */
    ASSERT_EQ(floss_link_init(&g_cfg, NULL), FLOSS_ERR_INVALID);
    floss_link_config_defaults(&g_cfg, FLOSS_LINK_ROLE_DONGLE, FLOSS_LINK_LE);
    g_cfg.le_interval_min = 5;
    ASSERT_EQ(floss_link_init(&g_cfg, NULL), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_link_init(NULL, NULL), FLOSS_ERR_INVALID);
}

/* ------------------------------------------------------------------------ */
/* GAP discovery                                                            */
/* ------------------------------------------------------------------------ */

TEST(eir_build_and_parse) {
    static const uint8_t uuid[16] = {0xF1, 0x05, 0x50, 0x00, 0x7A, 0x3C, 0x4B, 0x8E,
                                     0x9D, 0x2F, 0x5C, 0x6A, 0x1E, 0x0B, 0x7D, 0x10};
    uint8_t le[16];
    for (int i = 0; i < 16; i++) {
        le[i] = uuid[15 - i];
    }
    uint8_t eir[240];
    const uint16_t u16[1] = {0x1124};
    uint16_t n = floss_gap_eir_build(eir, sizeof eir, (const uint8_t *)"Pad", 3, u16, 1, le, 1);
    ASSERT_EQ(n, 5 + 4 + 18);
    uint8_t len = 0;
    const uint8_t *name = floss_gap_eir_find(eir, (uint8_t)n, FLOSS_EIR_NAME_COMPLETE, &len);
    ASSERT_NOT_NULL(name);
    ASSERT_EQ(len, 3);
    ASSERT_MEM_EQ(name, "Pad", 3);
    ASSERT_TRUE(floss_gap_eir_has_uuid128(eir, (uint8_t)n, uuid));
    ASSERT_TRUE(floss_gap_eir_has_uuid128(eir, 240, uuid)); /* zero padding ends the search */
    ASSERT_FALSE(floss_gap_eir_has_uuid128(eir, (uint8_t)(n - 1), uuid)); /* truncated list */

    /* Malformed: a structure running past the end is never read. */
    const uint8_t bad[] = {0x02, 0x09, 'A', 0x20, 0x07, 0x01};
    ASSERT_NOT_NULL(floss_gap_eir_find(bad, sizeof bad, FLOSS_EIR_NAME_COMPLETE, &len));
    ASSERT_NULL(floss_gap_eir_find(bad, sizeof bad, FLOSS_EIR_UUID128_ALL, &len));
    ASSERT_NULL(floss_gap_eir_find(NULL, 0, FLOSS_EIR_NAME_COMPLETE, &len));
}

TEST(inquiry_result_formats) {
    start_link(FLOSS_LINK_ROLE_PAD, FLOSS_LINK_CLASSIC, NULL); /* pad: no inquiry of its own */
    static const floss_addr_t far = {{0x09, 0, 0, 0, 0, 0x0A}};
    floss_sim_inquiry_add(&far, 0x000104, -90, NULL, 0); /* keeps the sim inquiry running */
    ASSERT_EQ(floss_gap_inquiry_start(0, 0), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_gap_inquiry_start(2, 0), FLOSS_OK);
    ASSERT_EQ(floss_gap_inquiry_start(2, 0), FLOSS_ERR_BUSY);
    floss_sim_pump(5);
    ASSERT_TRUE(floss_gap_inquiry_active());

    /* Inquiry Result with RSSI carrying two responses, field-major. */
    uint8_t p[1 + 14 * 2];
    memset(p, 0, sizeof p);
    p[0] = 2;
    memcpy(&p[1], k_peer.b, 6);
    memcpy(&p[7], k_stranger.b, 6);
    p[13] = 1;                    /* psrm[0] */
    p[14] = 2;                    /* psrm[1] */
    floss_put_le24(&p[17], 0x002508); /* cod[0] */
    floss_put_le24(&p[20], 0x000104); /* cod[1] */
    floss_put_le16(&p[23], 0x0100);
    floss_put_le16(&p[25], 0x0200);
    p[27] = (uint8_t)-40;
    p[28] = (uint8_t)-70;
    unsigned before = gap_count(FLOSS_GAP_EVT_INQUIRY_RESULT);
    ASSERT_EQ(floss_sim_inject_event(FLOSS_EVT_INQUIRY_RESULT_WITH_RSSI, p, sizeof p), FLOSS_OK);
    floss_sim_pump(3);
    ASSERT_EQ(gap_count(FLOSS_GAP_EVT_INQUIRY_RESULT), before + 2);
    const floss_gap_evt_t *e = &g_gap.evt[g_gap.count - 2];
    ASSERT_TRUE(floss_addr_eq(&e->addr, &k_peer));
    ASSERT_EQ(e->cod, 0x002508);
    ASSERT_EQ(e->rssi, -40);
    e = &g_gap.evt[g_gap.count - 1];
    ASSERT_TRUE(floss_addr_eq(&e->addr, &k_stranger));
    ASSERT_EQ(e->cod, 0x000104);
    ASSERT_EQ(e->rssi, -70);

    /* Truncated: dropped. */
    ASSERT_EQ(floss_sim_inject_event(FLOSS_EVT_INQUIRY_RESULT, p, 10), FLOSS_OK);
    floss_sim_pump(3);
    ASSERT_EQ(gap_count(FLOSS_GAP_EVT_INQUIRY_RESULT), before + 2);

    /* Page hint from the answer: psrm + clock offset with the valid bit. */
    floss_sim_peer_set_present(&k_stranger, false);
    floss_sim_log_clear();
    ASSERT_EQ(floss_gap_connect(&k_stranger), FLOSS_ERR_BUSY); /* not while inquiring */
    ASSERT_EQ(floss_gap_inquiry_cancel(), FLOSS_OK);
    floss_sim_pump(5);
    ASSERT_FALSE(floss_gap_inquiry_active());
    ASSERT_EQ(gap_count(FLOSS_GAP_EVT_INQUIRY_COMPLETE), 1);
    ASSERT_EQ(floss_gap_connect(&k_stranger), FLOSS_OK);
    const floss_sim_cmd_t *cc = floss_sim_expect_cmd(FLOSS_HCI_CREATE_CONNECTION);
    ASSERT_NOT_NULL(cc);
    ASSERT_EQ(cc->params[8], 2);
    ASSERT_EQ(floss_get_le16(&cc->params[10]), 0x8200);
    /* No second INQUIRY_COMPLETE for the same inquiry. */
    floss_sim_pump_us(3000000);
    ASSERT_EQ(gap_count(FLOSS_GAP_EVT_INQUIRY_COMPLETE), 1);
    floss_tc_assert_sim_clean();
}

/* ------------------------------------------------------------------------ */
/* Classic pad                                                              */
/* ------------------------------------------------------------------------ */

TEST(classic_pad_flow) {
    start_link(FLOSS_LINK_ROLE_PAD, FLOSS_LINK_CLASSIC, NULL);
    /* No peer yet: pairing mode = discoverable + connectable, gamepad CoD, EIR with the link UUID. */
    ASSERT_EQ(floss_link_state(), FLOSS_LINK_STATE_PAIRING);
    ASSERT_FALSE(floss_link_stored_peer(FLOSS_LINK_CLASSIC, NULL));
    ASSERT_EQ(floss_sim_scan_enable(), 0x03);
    ASSERT_EQ(floss_sim_class_of_device(), FLOSS_COD_GAMEPAD);
    ASSERT_STR_EQ(floss_sim_local_name(), "FLOSS Pad");
    const floss_sim_cmd_t *eir = floss_sim_cmd_last(FLOSS_HCI_WRITE_EXTENDED_INQUIRY_RESPONSE);
    ASSERT_NOT_NULL(eir); /* (params truncated in the log; full check below via the builder) */

    floss_sim_chan_t *ch;
    uint16_t h = dongle_connects(&k_peer, &ch);
    ASSERT_EQ(ch->state, FLOSS_SIM_CHAN_OPEN);
    ASSERT_TRUE(floss_sim_pump_until(cond_connected, NULL, 100));
    ASSERT_EQ(g_cb.connected, 1);
    ASSERT_TRUE(floss_addr_eq(&g_cb.peer, &k_peer));
    floss_sim_pump(5);
    ASSERT_EQ(floss_sim_scan_enable(), 0x00); /* page scan off while connected */

    /* The peer is remembered. */
    uint8_t rec[16];
    ASSERT_EQ(floss_hal_store_read(FLOSS_LINK_STORE_KEY_PEER, rec, sizeof rec), 9);
    ASSERT_MEM_EQ(&rec[3], k_peer.b, 6);
    floss_addr_t stored;
    ASSERT_TRUE(floss_link_stored_peer(FLOSS_LINK_CLASSIC, &stored));
    ASSERT_TRUE(floss_addr_eq(&stored, &k_peer));
    ASSERT_FALSE(floss_link_stored_peer(FLOSS_LINK_LE, NULL)); /* paired over Classic only */
    /* Remembering the same peer again changes nothing; another one replaces it, and back */
    ASSERT_EQ(floss_link_remember_peer(FLOSS_LINK_CLASSIC, &k_peer), FLOSS_OK);
    ASSERT_EQ(floss_link_remember_peer(FLOSS_LINK_CLASSIC, &k_stranger), FLOSS_OK);
    ASSERT_TRUE(floss_link_stored_peer(FLOSS_LINK_CLASSIC, &stored));
    ASSERT_TRUE(floss_addr_eq(&stored, &k_stranger));
    ASSERT_EQ(floss_link_remember_peer(FLOSS_LINK_CLASSIC, &k_peer), FLOSS_OK);

    /* HELLO first. */
    uint8_t pdu[128];
    int n = recv_pdu(ch, pdu, sizeof pdu);
    const uint8_t hello[] = {FLOSS_LINK_PDU_HELLO, FLOSS_LINK_VERSION, FLOSS_LINK_ROLE_PAD, FLOSS_LINK_MAX_REPORT, 0};
    ASSERT_EQ(n, 5);
    ASSERT_MEM_EQ(pdu, hello, 5);

    /* Inputs: paced, increasing seq, exact layout. */
    uint16_t first_seq = 0;
    for (int i = 0; i < 5; i++) {
        uint8_t r[13];
        memset(r, i, sizeof r);
        ASSERT_EQ(floss_link_set_input(1, r, sizeof r), FLOSS_OK);
        n = recv_type(ch, FLOSS_LINK_PDU_INPUT, pdu, sizeof pdu);
        ASSERT_EQ(n, 22); /* 13-byte report = 22-byte PDU */
        uint16_t seq = floss_get_le16(&pdu[1]);
        if (i == 0) {
            first_seq = seq;
        } else {
            ASSERT_EQ(seq, (uint16_t)(first_seq + i));
        }
        ASSERT_EQ(pdu[7], 1);
        ASSERT_EQ(pdu[8], 13);
        ASSERT_MEM_EQ(&pdu[9], r, 13);
        uint32_t t_pad = floss_get_le32(&pdu[3]);
        ASSERT_LE((uint32_t)(floss_hal_time_us() - t_pad), 20000u);
    }
    ASSERT_GT(g_cb.ticks, 0);
    ASSERT_EQ(floss_link_stats()->inputs_sent, 5);
    ASSERT_EQ(floss_link_set_input(1, pdu, FLOSS_LINK_MAX_REPORT + 1), FLOSS_ERR_INVALID);

    /* OUTPUT -> on_output. */
    const uint8_t out[] = {FLOSS_LINK_PDU_OUTPUT, 7, 0, 2, 3, 0xAA, 0xBB, 0xCC};
    ASSERT_EQ(floss_sim_l2cap_send(ch, out, sizeof out), FLOSS_OK);
    floss_sim_pump(5);
    ASSERT_EQ(g_cb.outputs, 1);
    ASSERT_EQ(g_cb.out_id, 2);
    ASSERT_EQ(g_cb.out_len, 3);
    ASSERT_MEM_EQ(g_cb.out_data, &out[5], 3);
    ASSERT_EQ(floss_link_stats()->outputs_received, 1);

    /* PING -> PONG echoing the dongle's timestamp. */
    const uint8_t ping[] = {FLOSS_LINK_PDU_PING, 0x78, 0x56, 0x34, 0x12};
    ASSERT_EQ(floss_sim_l2cap_send(ch, ping, sizeof ping), FLOSS_OK);
    n = recv_type(ch, FLOSS_LINK_PDU_PONG, pdu, sizeof pdu);
    ASSERT_EQ(n, 9);
    ASSERT_EQ(floss_get_le32(&pdu[1]), 0x12345678u);

    /* Link loss: back to connectable for our dongle only. */
    ASSERT_EQ(floss_sim_remote_disconnect(h, FLOSS_HCI_ERR_CONN_TIMEOUT), FLOSS_OK);
    ASSERT_TRUE(wait_state(FLOSS_LINK_STATE_CONNECTING, 100));
    ASSERT_EQ(g_cb.disconnected, 1);
    ASSERT_EQ(g_cb.reason, FLOSS_HCI_ERR_CONN_TIMEOUT);
    floss_sim_pump(5);
    ASSERT_EQ(floss_sim_scan_enable(), 0x02);

    floss_link_stats_dump();
    floss_tc_assert_sim_clean();
}

TEST(classic_pad_refuses_stranger) {
    start_link(FLOSS_LINK_ROLE_PAD, FLOSS_LINK_CLASSIC, NULL);
    floss_sim_chan_t *ch;
    uint16_t h = dongle_connects(&k_peer, &ch);
    ASSERT_TRUE(floss_sim_pump_until(cond_connected, NULL, 100));
    ASSERT_EQ(floss_sim_remote_disconnect(h, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED), FLOSS_OK);
    ASSERT_TRUE(wait_state(FLOSS_LINK_STATE_CONNECTING, 100));

    /* Someone else, not pairing: the channel is refused and the link dropped. */
    floss_sim_chan_t *sch;
    dongle_connects(&k_stranger, &sch);
    floss_sim_pump(20);
    ASSERT_NE(sch->state, FLOSS_SIM_CHAN_OPEN);
    ASSERT_NULL(floss_sim_link_by_addr(&k_stranger));
    ASSERT_EQ(g_cb.connected, 1);
    ASSERT_EQ(floss_link_state(), FLOSS_LINK_STATE_CONNECTING);

    /* Our dongle comes back with its stored key. */
    h = dongle_connects(&k_peer, &ch);
    ASSERT_TRUE(floss_sim_pump_until(cond_connected, NULL, 100));
    ASSERT_EQ(g_cb.connected, 2);

    /* Pairing mode: the current link is dropped, discoverable again, a new dongle is adopted. */
    ASSERT_EQ(floss_link_pair(), FLOSS_OK);
    ASSERT_TRUE(wait_state(FLOSS_LINK_STATE_PAIRING, 100));
    floss_sim_pump(10);
    ASSERT_EQ(floss_sim_scan_enable(), 0x03);
    dongle_connects(&k_stranger, &sch);
    ASSERT_TRUE(floss_sim_pump_until(cond_connected, NULL, 100));
    floss_link_peer_t p;
    ASSERT_TRUE(floss_link_peer_get(&p));
    ASSERT_TRUE(floss_addr_eq(&p.addr, &k_stranger));
    ASSERT_NULL(floss_gap_bond_find(&k_peer)); /* the old dongle's key is gone */
    floss_tc_assert_sim_clean();
}

static void tweak_idle(floss_link_config_t *c) {
    c->classic_idle_ms = 50;
}

TEST(classic_pad_idle_sniff) {
    start_link(FLOSS_LINK_ROLE_PAD, FLOSS_LINK_CLASSIC, tweak_idle);
    floss_sim_chan_t *ch;
    uint16_t h = dongle_connects(&k_peer, &ch);
    ASSERT_TRUE(floss_sim_pump_until(cond_connected, NULL, 100));
    uint8_t pdu[64];
    ASSERT_EQ(recv_pdu(ch, pdu, sizeof pdu), 5);
    ASSERT_EQ(pdu[4], FLOSS_LINK_HELLO_F_IDLE_SNIFF); /* announced to the dongle */
    ASSERT_TRUE((floss_sim_link(h)->link_policy & FLOSS_LINK_POLICY_SNIFF) != 0);

    const uint8_t a[4] = {1, 2, 3, 4};
    floss_link_set_input(1, a, 4);
    recv_type(ch, FLOSS_LINK_PDU_INPUT, pdu, sizeof pdu);
    /* Republishing the same content is not activity. */
    for (int i = 0; i < 6; i++) {
        floss_link_set_input(1, a, 4);
        floss_sim_pump_us(10000);
    }
    ASSERT_EQ(floss_sim_link(h)->mode, FLOSS_LINK_MODE_SNIFF);
    ASSERT_EQ(floss_sim_link(h)->interval, 16);

    /* A change wakes the link up. */
    floss_sim_log_clear();
    const uint8_t b[4] = {9, 9, 9, 9};
    floss_link_set_input(1, b, 4);
    ASSERT_NOT_NULL(floss_sim_expect_cmd(FLOSS_HCI_EXIT_SNIFF_MODE));
    floss_sim_pump(10);
    ASSERT_EQ(floss_sim_link(h)->mode, FLOSS_LINK_MODE_ACTIVE);
    floss_tc_assert_sim_clean();
}

TEST(classic_pad_malformed_pdus) {
    start_link(FLOSS_LINK_ROLE_PAD, FLOSS_LINK_CLASSIC, NULL);
    floss_sim_chan_t *ch;
    dongle_connects(&k_peer, &ch);
    ASSERT_TRUE(floss_sim_pump_until(cond_connected, NULL, 100));
    uint32_t before = floss_link_rx_malformed();

    static const struct {
        uint8_t len;
        uint8_t b[48];
    } bad[] = {
        {1, {0x00}},                                   /* unknown type */
        {1, {0xFF}},                                   /* unknown type */
        {4, {FLOSS_LINK_PDU_OUTPUT, 0, 0, 1}},         /* short OUTPUT */
        {7, {FLOSS_LINK_PDU_OUTPUT, 0, 0, 1, 5, 1, 2}}, /* len says 5, has 2 */
        {8, {FLOSS_LINK_PDU_OUTPUT, 0, 0, 1, 1, 1, 2, 3}}, /* trailing bytes */
        {6, {FLOSS_LINK_PDU_OUTPUT, 0, 0, 1, 0xFF, 0}},  /* len > max */
        {4, {FLOSS_LINK_PDU_PING, 1, 2, 3}},           /* short PING */
        {6, {FLOSS_LINK_PDU_PING, 1, 2, 3, 4, 5}},     /* long PING */
        {10, {FLOSS_LINK_PDU_INPUT, 0, 0, 0, 0, 0, 0, 1, 1, 0}}, /* wrong direction */
        {9, {FLOSS_LINK_PDU_PONG, 0, 0, 0, 0, 0, 0, 0, 0}},      /* wrong direction */
        {3, {FLOSS_LINK_PDU_HELLO, 1, 0}},             /* short HELLO */
    };
    for (unsigned i = 0; i < FLOSS_ARRAY_SIZE(bad); i++) {
        ASSERT_EQ(floss_sim_l2cap_send(ch, bad[i].b, bad[i].len), FLOSS_OK);
        floss_sim_pump(2);
    }
    /* An empty SDU too. */
    ASSERT_EQ(floss_sim_l2cap_send(ch, NULL, 0), FLOSS_OK);
    floss_sim_pump(5);
    ASSERT_EQ(floss_link_rx_malformed() - before, FLOSS_ARRAY_SIZE(bad) + 1);
    ASSERT_EQ(g_cb.outputs, 0);
    ASSERT_TRUE(floss_link_connected());

    /* Still works afterwards; a newer HELLO with extra fields is accepted. */
    const uint8_t hello[] = {FLOSS_LINK_PDU_HELLO, 2, FLOSS_LINK_ROLE_DONGLE, 32, 0, 0xEE};
    ASSERT_EQ(floss_sim_l2cap_send(ch, hello, sizeof hello), FLOSS_OK);
    const uint8_t out[] = {FLOSS_LINK_PDU_OUTPUT, 0, 0, 4, 1, 0x55};
    ASSERT_EQ(floss_sim_l2cap_send(ch, out, sizeof out), FLOSS_OK);
    floss_sim_pump(5);
    ASSERT_EQ(g_cb.outputs, 1);
    ASSERT_EQ(g_cb.out_id, 4);
    ASSERT_EQ(floss_link_rx_malformed() - before, FLOSS_ARRAY_SIZE(bad) + 1);
    ASSERT_TRUE(floss_test_log_contains("v2"));
    floss_tc_assert_sim_clean();
}

/* ------------------------------------------------------------------------ */
/* Classic dongle                                                           */
/* ------------------------------------------------------------------------ */

/** The sim pad answering inquiries: EIR = name + link service UUID. */
static void add_sim_pad(const floss_addr_t *addr, const char *name, bool with_uuid) {
    uint8_t eir[64];
    uint8_t n = 0;
    size_t nl = strlen(name);
    eir[n++] = (uint8_t)(nl + 1);
    eir[n++] = FLOSS_EIR_NAME_COMPLETE;
    memcpy(&eir[n], name, nl);
    n = (uint8_t)(n + nl);
    if (with_uuid) {
        eir[n++] = 17;
        eir[n++] = FLOSS_EIR_UUID128_ALL;
        for (int i = 0; i < 16; i++) {
            eir[n++] = floss_link_uuid_service[15 - i];
        }
    }
    ASSERT_NOT_NULL(floss_sim_inquiry_add(addr, FLOSS_COD_GAMEPAD, -45, eir, n));
}

static floss_sim_chan_t *g_dch;
static uint16_t g_dh;

static bool cond_dongle_chan(void *arg) {
    (void)arg;
    const floss_sim_link_t *l = floss_sim_link_by_addr(&k_peer);
    if (!l || l->state != FLOSS_SIM_LINK_OPEN) {
        return false;
    }
    floss_sim_chan_t *c = floss_sim_chan_find(l->handle, FLOSS_LINK_PSM);
    if (c && c->state == FLOSS_SIM_CHAN_OPEN) {
        g_dch = c;
        g_dh = l->handle;
        return true;
    }
    return false;
}

/** Dongle: wait until the stack paged the sim pad and opened the PSM. */
static void dongle_wait_session(void) {
    /* Up to a page timeout (5.12 s) plus the reconnect backoff of fake time. */
    ASSERT_TRUE(floss_sim_pump_until(cond_dongle_chan, NULL, 150000));
    ASSERT_TRUE(floss_sim_pump_until(cond_connected, NULL, 200));
}

TEST(classic_dongle_flow) {
    start_link(FLOSS_LINK_ROLE_DONGLE, FLOSS_LINK_CLASSIC, NULL);
    add_sim_pad(&k_stranger, "Some Headset", false); /* not a link pad: ignored */
    add_sim_pad(&k_peer, "FLOSS Pad", true);
    ASSERT_EQ(floss_link_pair(), FLOSS_OK);
    ASSERT_EQ(floss_link_state(), FLOSS_LINK_STATE_PAIRING);
    ASSERT_NOT_NULL(floss_sim_expect_cmd(FLOSS_HCI_INQUIRY));
    dongle_wait_session();
    ASSERT_GE(floss_sim_cmd_count(FLOSS_HCI_INQUIRY_CANCEL), 1);
    ASSERT_NULL(floss_sim_link_by_addr(&k_stranger));

    /* Page used the inquiry hint; we are central, sniff not allowed, QoS 1.25 ms, 2 s supervision. */
    const floss_sim_cmd_t *cc = floss_sim_cmd_last(FLOSS_HCI_CREATE_CONNECTION);
    ASSERT_NOT_NULL(cc);
    ASSERT_EQ(floss_get_le16(&cc->params[10]), 0x1234 | 0x8000);
    const floss_sim_link_t *l = floss_sim_link(g_dh);
    ASSERT_EQ(l->role, 0);
    ASSERT_EQ(l->link_policy & FLOSS_LINK_POLICY_ROLE_SWITCH, 0);
    ASSERT_EQ(l->link_policy & FLOSS_LINK_POLICY_SNIFF, 0);
    const floss_sim_cmd_t *qos = floss_sim_cmd_last(FLOSS_HCI_QOS_SETUP);
    ASSERT_NOT_NULL(qos);
    ASSERT_EQ(floss_get_le32(&qos->params[12]), 1250);
    const floss_sim_cmd_t *sto = floss_sim_cmd_last(FLOSS_HCI_WRITE_LINK_SUPERVISION_TIMEOUT);
    ASSERT_NOT_NULL(sto);
    ASSERT_EQ(floss_get_le16(&sto->params[2]), FLOSS_LINK_CLASSIC_SUPERVISION_SLOTS);
    ASSERT_TRUE(l->authenticated);
    ASSERT_NE(l->encrypted, 0);
    ASSERT_EQ(g_cb.connected, 1);
    ASSERT_TRUE(floss_addr_eq(&g_cb.peer, &k_peer));
    floss_link_peer_t p;
    ASSERT_TRUE(floss_link_peer_get(&p));
    ASSERT_TRUE(floss_addr_eq(&p.addr, &k_peer));
    ASSERT_NOT_NULL(floss_gap_bond_find(&k_peer));

    /* The dongle's HELLO. */
    uint8_t pdu[64];
    ASSERT_EQ(recv_pdu(g_dch, pdu, sizeof pdu), 5);
    ASSERT_EQ(pdu[0], FLOSS_LINK_PDU_HELLO);
    ASSERT_EQ(pdu[2], FLOSS_LINK_ROLE_DONGLE);

    /* INPUT PDUs from the pad: seq, loss, data. */
    const uint8_t r[6] = {1, 2, 3, 4, 5, 6};
    uint16_t seqs[] = {100, 101, 103, 104, 110};
    uint16_t lost[] = {0, 0, 1, 0, 5};
    for (unsigned i = 0; i < FLOSS_ARRAY_SIZE(seqs); i++) {
        build_input(pdu, seqs[i], 1000u * i, 3, r, sizeof r);
        ASSERT_EQ(floss_sim_l2cap_send(g_dch, pdu, 9 + sizeof r), FLOSS_OK);
        floss_sim_pump(10);
        ASSERT_EQ(g_cb.inputs, i + 1);
        ASSERT_EQ(g_cb.in_info.seq, seqs[i]);
        ASSERT_EQ(g_cb.in_info.lost, lost[i]);
        ASSERT_EQ(g_cb.in_info.t_pad_us, 1000u * i);
    }
    ASSERT_EQ(g_cb.in_id, 3);
    ASSERT_EQ(g_cb.in_len, 6);
    ASSERT_MEM_EQ(g_cb.in_data, r, 6);
    const floss_link_stats_t *st = floss_link_stats();
    ASSERT_EQ(st->inputs_received, 5);
    ASSERT_EQ(st->inputs_lost, 6);
    ASSERT_EQ(st->rx_interval.count, 4);
    ASSERT_EQ(st->rx_jitter.count, 5);

    /* OUTPUT to the pad. */
    const uint8_t rumble[2] = {0x80, 0x40};
    ASSERT_EQ(floss_link_send_output(2, rumble, 2), FLOSS_OK);
    floss_sim_pump(5);
    ASSERT_EQ(recv_pdu(g_dch, pdu, sizeof pdu), 7);
    const uint8_t want[] = {FLOSS_LINK_PDU_OUTPUT, 0, 0, 2, 2, 0x80, 0x40};
    ASSERT_MEM_EQ(pdu, want, sizeof want);
    ASSERT_EQ(floss_link_stats()->outputs_sent, 1);
    ASSERT_EQ(floss_link_set_input(1, r, 1), FLOSS_OK); /* harmless on a dongle */
    floss_link_stats_dump();
    floss_tc_assert_sim_clean();
}

TEST(classic_dongle_jitter_and_drift) {
    start_link(FLOSS_LINK_ROLE_DONGLE, FLOSS_LINK_CLASSIC, NULL);
    add_sim_pad(&k_peer, "FLOSS Pad", true);
    dongle_wait_session();
    uint8_t pdu[32];
    const uint8_t r[1] = {0};
    /* Pad clock 50 ppm slower than ours plus a 2 ms late report every 50:
     * the drift must not accumulate (window baseline), the late ones show. */
    uint32_t base = floss_hal_time_us();
    for (unsigned i = 0; i < 2000; i++) {
        uint32_t now = floss_hal_time_us();
        uint32_t elapsed = now - base;
        uint32_t t_pad = 5000000u + elapsed - elapsed / 20000u; /* -50 ppm */
        if (i % 50 == 25) {
            floss_sim_pump_us(2000); /* this one arrives 2 ms late */
        }
        build_input(pdu, (uint16_t)i, t_pad, 1, r, 1);
        ASSERT_EQ(floss_sim_l2cap_send(g_dch, pdu, 10), FLOSS_OK);
        floss_sim_pump(10); /* 1 ms */
    }
    const floss_link_stats_t *st = floss_link_stats();
    ASSERT_EQ(st->inputs_received, 2000);
    ASSERT_EQ(st->inputs_lost, 0);
    /* Typical reports sit near 0 jitter even after seconds of drift. */
    ASSERT_LE(floss_hist_percentile(&st->rx_jitter, 50), 512u);
    ASSERT_GE(st->rx_jitter.max_us, 1900u);
    ASSERT_LE(st->rx_jitter.max_us, 2600u);
}

TEST(classic_dongle_reconnects) {
    start_link(FLOSS_LINK_ROLE_DONGLE, FLOSS_LINK_CLASSIC, NULL);
    add_sim_pad(&k_peer, "FLOSS Pad", true);
    dongle_wait_session();
    unsigned inquiries = floss_sim_inquiry_count();

    /* Out of range for a while: page timeouts, then it comes back. */
    floss_sim_peer_set_present(&k_peer, false);
    ASSERT_EQ(floss_sim_remote_disconnect(g_dh, FLOSS_HCI_ERR_CONN_TIMEOUT), FLOSS_OK);
    ASSERT_TRUE(wait_state(FLOSS_LINK_STATE_CONNECTING, 100));
    ASSERT_EQ(g_cb.disconnected, 1);
    floss_sim_log_clear();
    floss_sim_pump_us(12000000);
    ASSERT_GE(floss_sim_cmd_count(FLOSS_HCI_CREATE_CONNECTION), 2); /* retried with backoff */
    floss_sim_peer_set_present(&k_peer, true);
    dongle_wait_session();
    ASSERT_EQ(g_cb.connected, 2);
    ASSERT_EQ(floss_sim_inquiry_count(), inquiries); /* reconnect = page, no search */
    ASSERT_NE(floss_sim_link(g_dh)->encrypted, 0);   /* with the stored key */

    /* Reboot of the dongle (same radio world, same flash): it reconnects at
     * start from the stored peer, without searching. */
    memset(&g_gap, 0, sizeof g_gap);
    ASSERT_EQ(floss_init(&(floss_init_t){.transport = g_transport}), FLOSS_OK);
    ASSERT_EQ(floss_gap_add_listener(gap_rec, NULL), FLOSS_OK);
    ASSERT_EQ(floss_link_init(&g_cfg, &k_cb), FLOSS_OK);
    ASSERT_EQ(floss_start(), FLOSS_OK);
    ASSERT_TRUE(floss_sim_pump_until(cond_ready, NULL, 5000));
    ASSERT_NE(floss_link_state(), FLOSS_LINK_STATE_PAIRING); /* paging (or already back) */
    inquiries = floss_sim_inquiry_count();
    dongle_wait_session();
    ASSERT_EQ(floss_sim_inquiry_count(), inquiries);
    ASSERT_EQ(g_cb.connected, 3);

    /* Forget: back to searching. */
    ASSERT_EQ(floss_link_forget(), FLOSS_OK);
    ASSERT_TRUE(wait_state(FLOSS_LINK_STATE_PAIRING, 200));
    ASSERT_FALSE(floss_link_peer_get(NULL));
    ASSERT_NULL(floss_gap_bond_find(&k_peer));
    floss_tc_assert_sim_clean();
}

static void tweak_ping(floss_link_config_t *c) {
    c->ping_period_ms = 5;
}

TEST(classic_dongle_ping_rtt_and_malformed) {
    start_link(FLOSS_LINK_ROLE_DONGLE, FLOSS_LINK_CLASSIC, tweak_ping);
    add_sim_pad(&k_peer, "FLOSS Pad", true);
    dongle_wait_session();
    uint8_t pdu[64];
    int n = recv_type(g_dch, FLOSS_LINK_PDU_PING, pdu, sizeof pdu);
    ASSERT_EQ(n, 5);
    floss_sim_pump_us(700); /* the pad's answer takes 0.7 ms */
    uint8_t pong[9];
    pong[0] = FLOSS_LINK_PDU_PONG;
    memcpy(&pong[1], &pdu[1], 4);
    floss_put_le32(&pong[5], 42);
    ASSERT_EQ(floss_sim_l2cap_send(g_dch, pong, sizeof pong), FLOSS_OK);
    floss_sim_pump(3);
    ASSERT_EQ(floss_link_stats()->rtt.count, 1);
    ASSERT_GE(floss_link_stats()->rtt.min_us, 700u);
    ASSERT_LE(floss_link_stats()->rtt.min_us, 1100u);

    /* Malformed INPUTs never reach on_input. */
    uint32_t before = floss_link_rx_malformed();
    const uint8_t r[4] = {1, 2, 3, 4};
    build_input(pdu, 1, 0, 1, r, 4);
    ASSERT_EQ(floss_sim_l2cap_send(g_dch, pdu, 12), FLOSS_OK); /* one byte short */
    ASSERT_EQ(floss_sim_l2cap_send(g_dch, pdu, 8), FLOSS_OK);  /* header cut */
    pdu[8] = FLOSS_LINK_MAX_REPORT + 1;
    static uint8_t big[9 + FLOSS_LINK_MAX_REPORT + 1];
    memcpy(big, pdu, 9);
    ASSERT_EQ(floss_sim_l2cap_send(g_dch, big, sizeof big), FLOSS_OK); /* report too long */
    const uint8_t outp[] = {FLOSS_LINK_PDU_OUTPUT, 0, 0, 1, 0};
    ASSERT_EQ(floss_sim_l2cap_send(g_dch, outp, sizeof outp), FLOSS_OK); /* wrong direction */
    const uint8_t pong_short[] = {FLOSS_LINK_PDU_PONG, 1, 2};
    ASSERT_EQ(floss_sim_l2cap_send(g_dch, pong_short, sizeof pong_short), FLOSS_OK);
    floss_sim_pump(10);
    ASSERT_EQ(g_cb.inputs, 0);
    ASSERT_EQ(floss_link_rx_malformed() - before, 5);
    ASSERT_TRUE(floss_link_connected());
    floss_tc_assert_sim_clean();
}

/* ------------------------------------------------------------------------ */
/* LE                                                                       */
/* ------------------------------------------------------------------------ */

#if FLOSS_ENABLE_LE

static void inject_encryption(uint16_t h) {
    uint8_t e[4] = {0, (uint8_t)h, (uint8_t)(h >> 8), 1};
    ASSERT_EQ(floss_sim_inject_event(FLOSS_EVT_ENCRYPTION_CHANGE, e, sizeof e), FLOSS_OK);
    floss_sim_pump(3);
}

/** Handle of the attribute with this 128-bit type (big-endian as written) in our own GATT database. */
static uint16_t own_attr128(const uint8_t uuid_be[16]) {
    uint8_t le[16];
    for (int i = 0; i < 16; i++) {
        le[i] = uuid_be[15 - i];
    }
    for (uint16_t h = 1; h <= floss_gatt_last_handle(); h++) {
        const floss_gatt_attr_t *a = floss_gatt_attr(h);
        if (a && a->uuid128 && a->type != FLOSS_GATT_UUID_PRIMARY_SERVICE && memcmp(a->uuid128, le, 16) == 0) {
            return h;
        }
    }
    return 0;
}

static int att_recv_type(uint16_t h, uint8_t op, uint8_t *out, uint16_t cap) {
    for (int i = 0; i < 400; i++) {
        int n = floss_sim_att_recv(h, out, cap);
        if (n > 0 && out[0] == op) {
            return n;
        }
        if (n < 0) {
            floss_sim_pump(1);
        }
    }
    FAIL("no ATT PDU 0x%02x", op);
}

TEST(le_pad_flow) {
    start_link(FLOSS_LINK_ROLE_PAD, FLOSS_LINK_LE, NULL);
    ASSERT_EQ(floss_link_state(), FLOSS_LINK_STATE_PAIRING);
    floss_sim_pump(20);
    ASSERT_TRUE(floss_le_adv_active());
    const floss_sim_cmd_t *ad = floss_sim_cmd_last(FLOSS_HCI_LE_SET_ADV_DATA);
    ASSERT_NOT_NULL(ad);
    ASSERT_TRUE(floss_le_ad_has_uuid128(&ad->params[1], ad->params[0], floss_link_uuid_service));
    uint8_t n8;
    const uint8_t *flags = floss_le_ad_find(&ad->params[1], ad->params[0], FLOSS_AD_FLAGS, &n8);
    ASSERT_NOT_NULL(flags);
    ASSERT_TRUE((flags[0] & FLOSS_AD_FLAG_GENERAL_DISC) != 0);

    uint16_t h = floss_sim_le_connect(&k_peer, 0, 1);
    floss_sim_pump(5);
    uint16_t in_vh = own_attr128(floss_link_uuid_input);
    uint16_t out_vh = own_attr128(floss_link_uuid_output);
    ASSERT_NE(in_vh, 0);
    ASSERT_NE(out_vh, 0);
    uint16_t cccd = floss_gatt_cccd_handle(in_vh);
    ASSERT_NE(cccd, 0);

    /* Subscribing before encryption is refused. */
    uint8_t wr[5] = {FLOSS_ATT_WRITE_REQ, (uint8_t)cccd, (uint8_t)(cccd >> 8), 0x01, 0x00};
    ASSERT_EQ(floss_sim_att_send(h, wr, sizeof wr), FLOSS_OK);
    uint8_t pdu[128];
    ASSERT_EQ(att_recv_type(h, FLOSS_ATT_ERROR_RSP, pdu, sizeof pdu), 5);
    ASSERT_FALSE(floss_link_connected());

    inject_encryption(h);
    /* MTU so a 13-byte report fits. */
    uint8_t mtu[3] = {FLOSS_ATT_MTU_REQ, 100, 0};
    ASSERT_EQ(floss_sim_att_send(h, mtu, sizeof mtu), FLOSS_OK);
    att_recv_type(h, FLOSS_ATT_MTU_RSP, pdu, sizeof pdu);
    ASSERT_EQ(floss_sim_att_send(h, wr, sizeof wr), FLOSS_OK);
    att_recv_type(h, FLOSS_ATT_WRITE_RSP, pdu, sizeof pdu);
    ASSERT_TRUE(floss_sim_pump_until(cond_connected, NULL, 50));
    ASSERT_EQ(g_cb.connected, 1);
    floss_link_peer_t p;
    ASSERT_TRUE(floss_link_peer_get(&p));
    ASSERT_TRUE(floss_addr_eq(&p.addr, &k_peer));

    /* HELLO then INPUTs as notifications on the input characteristic. */
    int n = att_recv_type(h, FLOSS_ATT_NOTIFY, pdu, sizeof pdu);
    ASSERT_EQ(n, 3 + 5);
    ASSERT_EQ(floss_get_le16(&pdu[1]), in_vh);
    ASSERT_EQ(pdu[3], FLOSS_LINK_PDU_HELLO);
    uint8_t r[13];
    memset(r, 0x5A, sizeof r);
    ASSERT_EQ(floss_link_set_input(1, r, sizeof r), FLOSS_OK);
    n = att_recv_type(h, FLOSS_ATT_NOTIFY, pdu, sizeof pdu);
    ASSERT_EQ(n, 3 + 22);
    ASSERT_EQ(pdu[3], FLOSS_LINK_PDU_INPUT);
    ASSERT_MEM_EQ(&pdu[3 + 9], r, 13);

    /* OUTPUT by Write Command. */
    uint8_t wc[3 + 7] = {FLOSS_ATT_WRITE_CMD, (uint8_t)out_vh, (uint8_t)(out_vh >> 8),
                         FLOSS_LINK_PDU_OUTPUT, 1, 0, 9, 2, 0x11, 0x22};
    ASSERT_EQ(floss_sim_att_send(h, wc, sizeof wc), FLOSS_OK);
    floss_sim_pump(5);
    ASSERT_EQ(g_cb.outputs, 1);
    ASSERT_EQ(g_cb.out_id, 9);
    ASSERT_EQ(g_cb.out_len, 2);
    /* Garbage by Write Command: counted, nothing else. */
    uint8_t junk[3 + 3] = {FLOSS_ATT_WRITE_CMD, (uint8_t)out_vh, (uint8_t)(out_vh >> 8), 0x02, 0xFF, 0xFF};
    uint32_t before = floss_link_rx_malformed();
    ASSERT_EQ(floss_sim_att_send(h, junk, sizeof junk), FLOSS_OK);
    floss_sim_pump(5);
    ASSERT_EQ(floss_link_rx_malformed() - before, 1);
    ASSERT_EQ(g_cb.outputs, 1);

    /* Link loss: advertising again, not discoverable (bonded). */
    ASSERT_EQ(floss_sim_remote_disconnect(h, FLOSS_HCI_ERR_CONN_TIMEOUT), FLOSS_OK);
    ASSERT_TRUE(wait_state(FLOSS_LINK_STATE_CONNECTING, 100));
    floss_sim_pump(20);
    ASSERT_EQ(g_cb.disconnected, 1);
    ad = floss_sim_cmd_last(FLOSS_HCI_LE_SET_ADV_DATA);
    flags = floss_le_ad_find(&ad->params[1], ad->params[0], FLOSS_AD_FLAGS, &n8);
    ASSERT_NOT_NULL(flags);
    ASSERT_EQ(flags[0] & FLOSS_AD_FLAG_GENERAL_DISC, 0);
    floss_tc_assert_sim_clean();
}

TEST(le_pad_drops_unsecured_central) {
    start_link(FLOSS_LINK_ROLE_PAD, FLOSS_LINK_LE, NULL);
    uint16_t h = floss_sim_le_connect(&k_stranger, 0, 1);
    floss_sim_pump(5);
    ASSERT_NOT_NULL(floss_sim_link(h));
    floss_sim_pump_us(FLOSS_LINK_LE_SECURE_TIMEOUT_US + 100000u);
    const floss_sim_link_t *l = floss_sim_link(h);
    ASSERT_TRUE(l == NULL || l->state != FLOSS_SIM_LINK_OPEN);
    ASSERT_EQ(g_cb.connected, 0);
    floss_tc_assert_sim_clean();
}

/* The sim pad's GATT server: the link service. */
static floss_sim_gatt_attr_t g_srv[6];
static uint8_t g_svc_le[16], g_in_le[16], g_out_le[16];
static uint8_t g_decl_in[19], g_decl_out[19];

static void build_sim_server(void) {
    for (int i = 0; i < 16; i++) {
        g_svc_le[i] = floss_link_uuid_service[15 - i];
        g_in_le[i] = floss_link_uuid_input[15 - i];
        g_out_le[i] = floss_link_uuid_output[15 - i];
    }
    g_decl_in[0] = FLOSS_GATT_PROP_NOTIFY;
    floss_put_le16(&g_decl_in[1], 3);
    memcpy(&g_decl_in[3], g_in_le, 16);
    g_decl_out[0] = FLOSS_GATT_PROP_WRITE | FLOSS_GATT_PROP_WRITE_NO_RSP;
    floss_put_le16(&g_decl_out[1], 6);
    memcpy(&g_decl_out[3], g_out_le, 16);
    static const uint8_t zero2[2] = {0, 0};
    floss_sim_gatt_attr_t a[6] = {
        {.handle = 1, .type16 = FLOSS_GATT_UUID_PRIMARY_SERVICE, .value = g_svc_le, .len = 16},
        {.handle = 2, .type16 = FLOSS_GATT_UUID_CHARACTERISTIC, .value = g_decl_in, .len = 19},
        {.handle = 3, .type128 = g_in_le, .value = NULL, .len = 0},
        {.handle = 4, .type16 = FLOSS_GATT_UUID_CCCD, .value = zero2, .len = 2},
        {.handle = 5, .type16 = FLOSS_GATT_UUID_CHARACTERISTIC, .value = g_decl_out, .len = 19},
        {.handle = 6, .type128 = g_out_le, .value = NULL, .len = 0},
    };
    memcpy(g_srv, a, sizeof a);
}

static bool cond_le_link(void *arg) {
    (void)arg;
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        if (floss_le_link_handle_at(i) != 0xFFFF) {
            return true;
        }
    }
    return false;
}

TEST(le_dongle_flow) {
    start_link(FLOSS_LINK_ROLE_DONGLE, FLOSS_LINK_LE, NULL);
    /* Advertisers: one without the service, our pad in pairing mode. */
    uint8_t adv_other[] = {2, FLOSS_AD_FLAGS, 0x06, 4, FLOSS_AD_NAME_COMPLETE, 'K', 'B', 'D'};
    ASSERT_NOT_NULL(floss_sim_le_advertiser_add(&k_stranger, 0, adv_other, sizeof adv_other, NULL, 0));
    floss_le_ad_t ad, sr;
    floss_le_ad_init(&ad);
    floss_le_ad_flags(&ad, FLOSS_AD_FLAG_GENERAL_DISC | FLOSS_AD_FLAG_NO_BREDR);
    uint8_t svc_le[16];
    for (int i = 0; i < 16; i++) {
        svc_le[i] = floss_link_uuid_service[15 - i];
    }
    floss_le_ad_add(&ad, FLOSS_AD_UUID128_ALL, svc_le, 16);
    floss_le_ad_init(&sr);
    floss_le_ad_name(&sr, "FLOSS Pad");
    ASSERT_NOT_NULL(floss_sim_le_advertiser_add(&k_peer, 0, ad.data, ad.len, sr.data, sr.len));

    ASSERT_TRUE(floss_sim_pump_until(cond_le_link, NULL, 5000));
    uint16_t h = floss_le_link_handle_at(0);
    if (h == 0xFFFF) {
        h = floss_le_link_handle_at(1);
    }
    ASSERT_TRUE(floss_addr_eq(&floss_le_link(h)->peer, &k_peer));
    ASSERT_EQ(floss_le_link(h)->interval, 6);
    build_sim_server();
    ASSERT_EQ(floss_sim_gatt_serve(h, g_srv, 6, 185), FLOSS_OK);
    inject_encryption(h); /* the sim has no SMP peer: encryption by decree */
    ASSERT_TRUE(floss_sim_pump_until(cond_connected, NULL, 2000));
    uint8_t v[8];
    ASSERT_EQ(floss_sim_gatt_value(4, v, sizeof v), 2);
    ASSERT_EQ(v[0], 0x01); /* subscribed */
    floss_link_peer_t p;
    ASSERT_TRUE(floss_link_peer_get(&p));
    ASSERT_TRUE(floss_addr_eq(&p.addr, &k_peer));

    /* INPUTs by notification. */
    uint8_t pdu[32];
    const uint8_t r[3] = {7, 8, 9};
    build_input(pdu, 1, 10, 1, r, 3);
    ASSERT_EQ(floss_sim_gatt_notify(3, pdu, 12, false), FLOSS_OK);
    build_input(pdu, 3, 20, 1, r, 3);
    ASSERT_EQ(floss_sim_gatt_notify(3, pdu, 12, false), FLOSS_OK);
    floss_sim_pump(10);
    ASSERT_EQ(g_cb.inputs, 2);
    ASSERT_EQ(g_cb.in_info.seq, 3);
    ASSERT_EQ(g_cb.in_info.lost, 1);
    ASSERT_MEM_EQ(g_cb.in_data, r, 3);

    /* OUTPUT by Write Command (HELLO went the same way first). */
    ASSERT_EQ(floss_link_send_output(5, r, 2), FLOSS_OK);
    floss_sim_pump(10);
    int n = floss_sim_gatt_value(6, v, sizeof v);
    ASSERT_EQ(n, 7);
    const uint8_t want[] = {FLOSS_LINK_PDU_OUTPUT, 0, 0, 5, 2, 7, 8};
    ASSERT_MEM_EQ(v, want, 7);
    ASSERT_GE(floss_sim_gatt_stats()->write_cmds, 2);
    ASSERT_EQ(floss_sim_gatt_stats()->pipelined, 0);
    floss_tc_assert_sim_clean();
}

#endif /* FLOSS_ENABLE_LE */

/* ------------------------------------------------------------------------ */
/* Classic: the pad connects (classic_pad_connects), keys or none            */
/* ------------------------------------------------------------------------ */

/** A third remote, for "strongest answer" choices. */
static const floss_addr_t k_third = {{0x03, 0x00, 0x00, 0x5D, 0x05, 0xF1}};

/** A link device the sim answers inquiries for (name, signal strength, link UUID or not). */
static void add_sim_dev(const floss_addr_t *addr, const char *name, int8_t rssi, bool with_uuid) {
    uint8_t eir[64];
    uint8_t n = 0;
    size_t nl = strlen(name);
    eir[n++] = (uint8_t)(nl + 1);
    eir[n++] = FLOSS_EIR_NAME_COMPLETE;
    memcpy(&eir[n], name, nl);
    n = (uint8_t)(n + nl);
    if (with_uuid) {
        eir[n++] = 17;
        eir[n++] = FLOSS_EIR_UUID128_ALL;
        for (int i = 0; i < 16; i++) {
            eir[n++] = floss_link_uuid_service[15 - i];
        }
    }
    ASSERT_NOT_NULL(floss_sim_inquiry_add(addr, 0, rssi, eir, n));
}

static const floss_addr_t *g_want;

static bool cond_chan_to(void *arg) {
    (void)arg;
    const floss_sim_link_t *l = floss_sim_link_by_addr(g_want);
    if (!l || l->state != FLOSS_SIM_LINK_OPEN) {
        return false;
    }
    floss_sim_chan_t *c = floss_sim_chan_find(l->handle, FLOSS_LINK_PSM);
    if (c && c->state == FLOSS_SIM_CHAN_OPEN) {
        g_dch = c;
        g_dh = l->handle;
        return true;
    }
    return false;
}

/** The stack connected to `who` and opened the link PSM; the session is up. */
static void wait_session_to(const floss_addr_t *who) {
    g_want = who;
    /* Up to a page timeout and a search round of fake time. */
    ASSERT_TRUE(floss_sim_pump_until(cond_chan_to, NULL, 300000));
    ASSERT_TRUE(floss_sim_pump_until(cond_connected, NULL, 200));
}

static void tweak_pad_connects(floss_link_config_t *c) {
    c->classic_pad_connects = true;
}

static void tweak_pad_connects_no_keys(floss_link_config_t *c) {
    c->classic_pad_connects = true;
    c->classic_store_keys = false;
}

TEST(classic_pad_connects_to_dongle) {
    start_link(FLOSS_LINK_ROLE_PAD, FLOSS_LINK_CLASSIC, tweak_pad_connects);
    add_sim_dev(&k_stranger, "Some Headset", -40, false); /* not a link device: ignored */
    add_sim_dev(&k_peer, "FLOSS Dongle", -50, true);
    /* No peer yet: the pad searches; it does not wait to be found. */
    ASSERT_EQ(floss_link_state(), FLOSS_LINK_STATE_PAIRING);
    ASSERT_NOT_NULL(floss_sim_expect_cmd(FLOSS_HCI_INQUIRY));
    ASSERT_EQ(floss_sim_scan_enable(), 0x00);
    wait_session_to(&k_peer);
    ASSERT_NULL(floss_sim_link_by_addr(&k_stranger));

    /* The pad runs the link: central, no role switch, QoS 1.25 ms, secured. */
    const floss_sim_link_t *l = floss_sim_link(g_dh);
    ASSERT_EQ(l->role, 0);
    ASSERT_EQ(l->link_policy & FLOSS_LINK_POLICY_ROLE_SWITCH, 0);
    const floss_sim_cmd_t *qos = floss_sim_cmd_last(FLOSS_HCI_QOS_SETUP);
    ASSERT_NOT_NULL(qos);
    ASSERT_EQ(floss_get_le32(&qos->params[12]), 1250);
    ASSERT_TRUE(l->authenticated);
    ASSERT_NE(l->encrypted, 0);
    floss_link_peer_t p;
    ASSERT_TRUE(floss_link_peer_get(&p));
    ASSERT_TRUE(floss_addr_eq(&p.addr, &k_peer));
    ASSERT_NOT_NULL(floss_gap_bond_find(&k_peer));

    /* The data roles are unchanged: the pad says hello as a pad and sends INPUT. */
    uint8_t pdu[64];
    int n = recv_type(g_dch, FLOSS_LINK_PDU_HELLO, pdu, sizeof pdu);
    ASSERT_EQ(n, 5);
    ASSERT_EQ(pdu[2], FLOSS_LINK_ROLE_PAD);
    uint8_t r[13];
    memset(r, 0x5A, sizeof r);
    ASSERT_EQ(floss_link_set_input(1, r, sizeof r), FLOSS_OK);
    n = recv_type(g_dch, FLOSS_LINK_PDU_INPUT, pdu, sizeof pdu);
    ASSERT_EQ(n, 22);
    ASSERT_MEM_EQ(&pdu[9], r, 13);

    /* Lost link: the pad pages its dongle again (no search) with the stored key. */
    unsigned inquiries = floss_sim_inquiry_count();
    ASSERT_EQ(floss_sim_remote_disconnect(g_dh, FLOSS_HCI_ERR_CONN_TIMEOUT), FLOSS_OK);
    ASSERT_TRUE(wait_state(FLOSS_LINK_STATE_CONNECTING, 100));
    wait_session_to(&k_peer);
    ASSERT_EQ(g_cb.connected, 2);
    ASSERT_EQ(floss_sim_inquiry_count(), inquiries);
    floss_tc_assert_sim_clean();
}

static void tweak_dongle_waits(floss_link_config_t *c) {
    c->classic_pad_connects = true;
}

TEST(classic_dongle_waits_for_pad) {
    start_link(FLOSS_LINK_ROLE_DONGLE, FLOSS_LINK_CLASSIC, tweak_dongle_waits);
    /* No peer: findable and connectable, under the dongle's own name; no searching. */
    ASSERT_EQ(floss_link_state(), FLOSS_LINK_STATE_PAIRING);
    ASSERT_EQ(floss_sim_scan_enable(), 0x03);
    ASSERT_STR_EQ(floss_sim_local_name(), "FLOSS Dongle");
    ASSERT_EQ(floss_sim_cmd_count(FLOSS_HCI_INQUIRY), 0);

    floss_sim_chan_t *ch;
    uint16_t h = dongle_connects(&k_peer, &ch); /* here the sim plays the pad */
    ASSERT_EQ(ch->state, FLOSS_SIM_CHAN_OPEN);
    ASSERT_TRUE(floss_sim_pump_until(cond_connected, NULL, 100));
    floss_sim_pump(5);
    ASSERT_EQ(floss_sim_scan_enable(), 0x00); /* a busy dongle stays silent */
    floss_link_peer_t p;
    ASSERT_TRUE(floss_link_peer_get(&p));
    ASSERT_TRUE(floss_addr_eq(&p.addr, &k_peer));

    uint8_t pdu[64];
    ASSERT_EQ(recv_type(ch, FLOSS_LINK_PDU_HELLO, pdu, sizeof pdu), 5);
    ASSERT_EQ(pdu[2], FLOSS_LINK_ROLE_DONGLE);
    const uint8_t r[4] = {1, 2, 3, 4};
    build_input(pdu, 7, 1000, 1, r, sizeof r);
    ASSERT_EQ(floss_sim_l2cap_send(ch, pdu, 9 + sizeof r), FLOSS_OK);
    floss_sim_pump(10);
    ASSERT_EQ(g_cb.inputs, 1);
    ASSERT_MEM_EQ(g_cb.in_data, r, 4);

    /* Link lost: connectable for our pad only, which comes back with its key. */
    ASSERT_EQ(floss_sim_remote_disconnect(h, FLOSS_HCI_ERR_CONN_TIMEOUT), FLOSS_OK);
    ASSERT_TRUE(wait_state(FLOSS_LINK_STATE_CONNECTING, 100));
    floss_sim_pump(5);
    ASSERT_EQ(floss_sim_scan_enable(), 0x02);
    floss_sim_chan_t *sch;
    dongle_connects(&k_stranger, &sch);
    floss_sim_pump(20);
    ASSERT_NE(sch->state, FLOSS_SIM_CHAN_OPEN);
    dongle_connects(&k_peer, &ch);
    ASSERT_TRUE(floss_sim_pump_until(cond_connected, NULL, 100));
    ASSERT_EQ(g_cb.connected, 2);
    floss_tc_assert_sim_clean();
}

static void tweak_dongle_waits_no_keys(floss_link_config_t *c) {
    c->classic_pad_connects = true;
    c->classic_store_keys = false;
}

TEST(classic_no_keys_dongle_takes_any_pad) {
    /* A peer record left over from a keyed setup must not narrow who may connect. */
    const uint8_t old[9] = {1, FLOSS_LINK_CLASSIC, 0, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09};
    floss_tc_sim_init(NULL); /* fresh RAM store for the record below */
    ASSERT_EQ(floss_hal_store_write(FLOSS_LINK_STORE_KEY_PEER, old, sizeof old), FLOSS_OK);
    start_link(FLOSS_LINK_ROLE_DONGLE, FLOSS_LINK_CLASSIC, tweak_dongle_waits_no_keys);
    ASSERT_FALSE(floss_link_peer_get(NULL));
    ASSERT_EQ(floss_link_state(), FLOSS_LINK_STATE_PAIRING);
    ASSERT_EQ(floss_sim_scan_enable(), 0x03);

    floss_sim_chan_t *ch;
    uint16_t h = dongle_connects(&k_peer, &ch);
    ASSERT_TRUE(floss_sim_pump_until(cond_connected, NULL, 100));
    ASSERT_EQ(floss_gap_bond_count(), 0); /* paired, nothing stored */
    ASSERT_FALSE(floss_link_peer_get(NULL));
    uint8_t rec[16];
    ASSERT_EQ(floss_hal_store_read(FLOSS_LINK_STORE_KEY_PEER, rec, sizeof rec), FLOSS_ERR_NOT_FOUND);

    /* Free again: findable, and the next pad is welcome without a pairing window. */
    ASSERT_EQ(floss_sim_remote_disconnect(h, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED), FLOSS_OK);
    ASSERT_TRUE(wait_state(FLOSS_LINK_STATE_PAIRING, 100));
    floss_sim_pump(5);
    ASSERT_EQ(floss_sim_scan_enable(), 0x03);
    dongle_connects(&k_stranger, &ch);
    ASSERT_TRUE(floss_sim_pump_until(cond_connected, NULL, 100));
    ASSERT_EQ(g_cb.connected, 2);
    ASSERT_TRUE(floss_addr_eq(&g_cb.peer, &k_stranger));
    ASSERT_EQ(floss_gap_bond_count(), 0);
    floss_tc_assert_sim_clean();
}

TEST(classic_no_keys_pad_prefers_remembered_then_strongest) {
    start_link(FLOSS_LINK_ROLE_PAD, FLOSS_LINK_CLASSIC, tweak_pad_connects_no_keys);
    /* First time, nobody remembered: the strongest dongle of the round. */
    add_sim_dev(&k_stranger, "FLOSS Dongle", -70, true);
    add_sim_dev(&k_peer, "FLOSS Dongle", -40, true);
    wait_session_to(&k_peer);
    ASSERT_NULL(floss_sim_link_by_addr(&k_stranger));
    ASSERT_EQ(floss_gap_bond_count(), 0);
    floss_link_peer_t p;
    ASSERT_TRUE(floss_link_peer_get(&p)); /* the address is remembered, no key */
    ASSERT_TRUE(floss_addr_eq(&p.addr, &k_peer));
    uint32_t pairings = floss_sim_cmd_count(FLOSS_HCI_IO_CAPABILITY_REQUEST_REPLY);
    ASSERT_GE(pairings, 1);

    /* Reconnect: the remembered dongle is paged first and pairs again. */
    unsigned inquiries = floss_sim_inquiry_count();
    ASSERT_EQ(floss_sim_remote_disconnect(g_dh, FLOSS_HCI_ERR_CONN_TIMEOUT), FLOSS_OK);
    wait_session_to(&k_peer);
    ASSERT_EQ(floss_sim_inquiry_count(), inquiries);
    ASSERT_GT(floss_sim_cmd_count(FLOSS_HCI_IO_CAPABILITY_REQUEST_REPLY), pairings);
    ASSERT_EQ(floss_gap_bond_count(), 0);

    /* The remembered dongle is gone: any free one will do, the strongest. */
    floss_sim_peer_set_present(&k_peer, false);
    floss_sim_inquiry_clear();
    add_sim_dev(&k_stranger, "FLOSS Dongle", -60, true);
    add_sim_dev(&k_third, "FLOSS Dongle", -45, true);
    ASSERT_EQ(floss_sim_remote_disconnect(g_dh, FLOSS_HCI_ERR_CONN_TIMEOUT), FLOSS_OK);
    wait_session_to(&k_third);
    ASSERT_NULL(floss_sim_link_by_addr(&k_stranger));
    ASSERT_TRUE(floss_link_peer_get(&p));
    ASSERT_TRUE(floss_addr_eq(&p.addr, &k_third));
    ASSERT_EQ(floss_gap_bond_count(), 0);
    floss_tc_assert_sim_clean();
}

TEST(classic_no_keys_pad_takes_remembered_at_once) {
    start_link(FLOSS_LINK_ROLE_PAD, FLOSS_LINK_CLASSIC, tweak_pad_connects_no_keys);
    add_sim_dev(&k_peer, "FLOSS Dongle", -60, true);
    wait_session_to(&k_peer);
    /* The remembered dongle is busy elsewhere for a while (no page answer), while a
     * stronger one is around: the search still takes the remembered one when it
     * answers in a round, without waiting for the round to end. */
    floss_sim_peer_set_present(&k_peer, false);
    ASSERT_EQ(floss_sim_remote_disconnect(g_dh, FLOSS_HCI_ERR_CONN_TIMEOUT), FLOSS_OK);
    ASSERT_TRUE(wait_state(FLOSS_LINK_STATE_PAIRING, 400000)); /* page timed out: searching */
    add_sim_dev(&k_third, "FLOSS Dongle", -30, false);       /* not a link device */
    floss_sim_peer_set_present(&k_peer, true);
    wait_session_to(&k_peer);
    ASSERT_EQ(floss_gap_bond_count(), 0);
    floss_tc_assert_sim_clean();
}

static void tweak_pad_2ms(floss_link_config_t *cfg) {
    cfg->report_interval_us = 2000;
}

/** Gap between report ticks i-1 and i is want_us, give or take one pump step. */
#define ASSERT_TICK_GAP(i, want_us)                    do {                                                   ASSERT_GE(tick_gap(i), (want_us) - 100u);          ASSERT_LE(tick_gap(i), (want_us) + 100u);      } while (0)

/** Gap between report ticks i-1 and i. */
static uint32_t tick_gap(unsigned i) {
    return g_tick_us[i] - g_tick_us[i - 1];
}

static bool cond_tick_after(void *arg) {
    return g_cb.ticks > *(unsigned *)arg;
}

/** Pump until the next report tick; returns the index of the tick after it. */
static unsigned next_tick(void) {
    unsigned t = g_cb.ticks;
    ASSERT_TRUE(floss_sim_pump_until(cond_tick_after, &t, 100));
    return g_cb.ticks;
}

static void tweak_pad_connects_fast_page(floss_link_config_t *cfg) {
    tweak_pad_connects_no_keys(cfg);
    cfg->classic_page_timeout_ms = 1280;
}

/** Bring the stack up with the link and return the Write Page Timeout it sent, before the link's
 * own search starts (it soon fills the simulator's command log). */
static uint16_t page_timeout_at_bringup(void (*tweak)(floss_link_config_t *)) {
    memset(&g_gap, 0, sizeof g_gap);
    g_transport = floss_tc_sim_init(NULL);
    ASSERT_EQ(floss_init(&(floss_init_t){.transport = g_transport}), FLOSS_OK);
    ASSERT_EQ(floss_gap_add_listener(gap_rec, NULL), FLOSS_OK);
    floss_link_config_defaults(&g_cfg, FLOSS_LINK_ROLE_PAD, FLOSS_LINK_CLASSIC);
    tweak(&g_cfg);
    ASSERT_EQ(floss_link_init(&g_cfg, &k_cb), FLOSS_OK);
    ASSERT_EQ(floss_start(), FLOSS_OK);
    ASSERT_TRUE(floss_sim_pump_until(cond_ready, NULL, 5000));
    const floss_sim_cmd_t *pt = floss_sim_cmd_last(FLOSS_HCI_WRITE_PAGE_TIMEOUT);
    ASSERT_NOT_NULL(pt);
    return floss_get_le16(&pt->params[0]);
}

TEST(classic_page_timeout) {
    ASSERT_EQ(page_timeout_at_bringup(tweak_pad_connects_no_keys), 0x2000);   /* GAP's 5.12 s */
    ASSERT_EQ(page_timeout_at_bringup(tweak_pad_connects_fast_page), 0x0800); /* 1.28 s */
    ASSERT_EQ(floss_gap_set_page_timeout(0), FLOSS_ERR_INVALID);
}

TEST(classic_pad_shift_phase) {
    start_link(FLOSS_LINK_ROLE_PAD, FLOSS_LINK_CLASSIC, tweak_pad_2ms);
    ASSERT_EQ(floss_link_shift_phase(100), FLOSS_ERR_STATE); /* no session */
    floss_sim_chan_t *ch;
    dongle_connects(&k_peer, &ch);
    ASSERT_TRUE(floss_sim_pump_until(cond_connected, NULL, 100));
    floss_sim_pump_us(10000);

    /* Later: one gap grows by the shift, the period stays. Shifts go in just after a report. */
    unsigned t = next_tick();
    ASSERT_EQ(floss_link_shift_phase(500), FLOSS_OK);
    floss_sim_pump_us(8000);
    ASSERT_GE(g_cb.ticks, t + 3);
    ASSERT_TICK_GAP(t, 2500u);
    ASSERT_TICK_GAP(t + 1, 2000u);
    ASSERT_TICK_GAP(t + 2, 2000u);

    /* The next report's time, ahead of its tick */
    uint32_t at;
    ASSERT_TRUE(floss_link_next_report_at(&at));
    unsigned before = g_cb.ticks;
    while (g_cb.ticks == before) {
        ASSERT_TRUE((int32_t)(floss_hal_time_us() - at) < 200); /* not past it without a tick */
        floss_sim_pump(1);
    }
    ASSERT_TRUE((int32_t)(g_tick_us[before] - at) >= 0);
    ASSERT_TRUE((int32_t)(g_tick_us[before] - at) <= 100);

    /* Earlier. */
    t = next_tick();
    ASSERT_EQ(floss_link_shift_phase(-700), FLOSS_OK);
    floss_sim_pump_us(8000);
    ASSERT_GE(g_cb.ticks, t + 3);
    ASSERT_TICK_GAP(t, 1300u);
    ASSERT_TICK_GAP(t + 1, 2000u);
}

TEST(reinit_forgets_link) {
    start_link(FLOSS_LINK_ROLE_PAD, FLOSS_LINK_CLASSIC, tweak_pad_connects_no_keys);
    add_sim_dev(&k_peer, "FLOSS Dongle", -60, true);
    wait_session_to(&k_peer);
    ASSERT_NOT_NULL(floss_link_poll_hook);

    /* The application restarts the stack for another profile: the old session must not be polled. */
    g_transport = floss_tc_sim_init(NULL);
    ASSERT_EQ(floss_init(&(floss_init_t){.transport = g_transport}), FLOSS_OK);
    ASSERT_NULL(floss_link_poll_hook);
    ASSERT_EQ(floss_start(), FLOSS_OK);
    floss_sim_pump(50); /* with the stale hook, this polled the old session */
}

static const floss_test_case_t tests[] = {
    TEST_CASE(config_defaults),
    TEST_CASE(eir_build_and_parse),
    TEST_CASE(inquiry_result_formats),
    TEST_CASE(classic_pad_flow),
    TEST_CASE(classic_pad_refuses_stranger),
    TEST_CASE(classic_pad_idle_sniff),
    TEST_CASE(classic_pad_malformed_pdus),
    TEST_CASE(classic_dongle_flow),
    TEST_CASE(classic_dongle_jitter_and_drift),
    TEST_CASE(classic_dongle_reconnects),
    TEST_CASE(classic_dongle_ping_rtt_and_malformed),
    TEST_CASE(classic_pad_connects_to_dongle),
    TEST_CASE(classic_dongle_waits_for_pad),
    TEST_CASE(classic_no_keys_dongle_takes_any_pad),
    TEST_CASE(classic_no_keys_pad_prefers_remembered_then_strongest),
    TEST_CASE(classic_no_keys_pad_takes_remembered_at_once),
    TEST_CASE(reinit_forgets_link),
    TEST_CASE(classic_pad_shift_phase),
    TEST_CASE(classic_page_timeout),
#if FLOSS_ENABLE_LE
    TEST_CASE(le_pad_flow),
    TEST_CASE(le_pad_drops_unsecured_central),
    TEST_CASE(le_dongle_flow),
#endif
};

FLOSS_TEST_MAIN("link", tests)
