/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * test_hid_integration.c - The hooks a product firmware uses to fit FLOSS
 * around hosts with their own rules:
 *   - a verbatim HID service record, HID channels without security, and
 *     visibility left entirely to the application;
 *   - a binary legacy PIN supplied per peer;
 *   - a link key obtained out of band, stored as a bond;
 *   - a report built only when it can leave (floss_hid_can_send_now);
 *   - being told when a connect attempt gives up, and why;
 *   - being told when the peer addresses a channel we do not have.
 */
#include "test_common.h"

static floss_hid_config_t g_cfg; /* must outlive the stack */

/* ------------------------------------------------------------------------ */
/* Fixtures                                                                 */
/* ------------------------------------------------------------------------ */

/** A small hand-written HID record: attribute 0x0000 (uint32), service class HID, a marker string. */
static const uint8_t k_record[] = {
    0x35, 0x18,                         /* sequence, 24 bytes */
    0x09, 0x00, 0x00,                   /* attr 0x0000 */
    0x0A, 0x00, 0x01, 0x00, 0x00,       /* uint32 handle (patched by the server) */
    0x09, 0x00, 0x01,                   /* attr 0x0001 service class ID list */
    0x35, 0x03, 0x19, 0x11, 0x24,       /*   { UUID16 HID } */
    0x09, 0x01, 0x00,                   /* attr 0x0100 service name */
    0x25, 0x03, 'R', 'A', 'W',          /*   "RAW" */
};

static struct {
    unsigned failed;
    uint8_t reason;
    uint16_t channel_status;
    unsigned unknown_cid;
    uint16_t unknown_cid_handle;
    uint16_t unknown_cid_value;
} g_log;

static void on_connect_failed(uint8_t reason, uint16_t channel_status) {
    g_log.failed++;
    g_log.reason = reason;
    g_log.channel_status = channel_status;
}

static void on_unknown_cid(uint16_t handle, uint16_t cid) {
    g_log.unknown_cid++;
    g_log.unknown_cid_handle = handle;
    g_log.unknown_cid_value = cid;
}

static const floss_hid_callbacks_t k_callbacks = {
    .on_connect_failed = on_connect_failed,
};

static bool cond_ready(void *arg) {
    (void)arg;
    return floss_hci_ready();
}

/** floss_init + HID with k_callbacks + floss_start, until READY. */
static void start(floss_transport_t *t) {
    memset(&g_log, 0, sizeof g_log);
    floss_init_t init = {.transport = t, .chipset = NULL};
    ASSERT_EQ(floss_init(&init), FLOSS_OK);
    ASSERT_EQ(floss_hid_init(&g_cfg, &k_callbacks), FLOSS_OK);
    ASSERT_EQ(floss_start(), FLOSS_OK);
    ASSERT_TRUE(floss_sim_pump_until(cond_ready, NULL, 5000));
    floss_sim_pump(20);
}

/* ------------------------------------------------------------------------ */
/* Tests                                                                    */
/* ------------------------------------------------------------------------ */

TEST(raw_record_open_channels_app_scan) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 0);
    g_cfg.sdp_record = k_record;
    g_cfg.sdp_record_len = sizeof k_record;
    g_cfg.no_security = true;
    g_cfg.app_controls_scan = true;
    g_cfg.auto_reconnect = false;
    start(t);

    /* No bond and auto_discoverable, yet HID left scan enable alone. */
    ASSERT_EQ(floss_sim_scan_enable(), 0x00);
    ASSERT_EQ(floss_gap_set_scan(true, true), FLOSS_OK);
    floss_sim_pump(20);
    ASSERT_EQ(floss_sim_scan_enable(), 0x03);

    /* The host finds our record, byte for byte apart from the handle. */
    uint16_t h = floss_tc_host_connect();
    static uint8_t sdp[512];
    int n = floss_sim_sdp_ssa(h, FLOSS_UUID_HID, sdp, sizeof sdp);
    ASSERT_GT(n, 0);
    const uint8_t marker[] = {0x25, 0x03, 'R', 'A', 'W'};
    ASSERT_NOT_NULL(floss_test_memfind(sdp, (size_t)n, marker, sizeof marker));
    ASSERT_NULL(floss_test_memfind(sdp, (size_t)n, floss_tc_report_desc, floss_tc_report_desc_len));

    /* HID opens on a link that was never authenticated. */
    floss_sim_chan_t *ctrl, *intr;
    floss_tc_hid_open(h, &ctrl, &intr);
    ASSERT_NULL(floss_sim_cmd_last(FLOSS_HCI_AUTHENTICATION_REQUESTED));
    const floss_sim_link_t *link = floss_sim_link(h);
    ASSERT_NOT_NULL(link);
    ASSERT_FALSE(link->authenticated);

    /* Connected: still the application's call. */
    floss_sim_pump(20);
    ASSERT_EQ(floss_sim_scan_enable(), 0x03);

    /* Built on demand: a report goes out whenever the link can take it. */
    ASSERT_TRUE(floss_hid_can_send_now());
    const uint8_t in[FLOSS_TC_INPUT_LEN] = {1, 2, 3, 4, 5, 6};
    ASSERT_EQ(floss_hid_send_input_now(FLOSS_TC_INPUT_ID, in, sizeof in), FLOSS_OK);
    floss_sim_pump(20);
    uint8_t buf[64];
    int len = floss_sim_l2cap_recv(intr, buf, sizeof buf);
    ASSERT_EQ(len, 2 + FLOSS_TC_INPUT_LEN);
    ASSERT_EQ(buf[0], 0xA1);
    ASSERT_EQ(buf[1], FLOSS_TC_INPUT_ID);
    ASSERT_MEM_EQ(buf + 2, in, sizeof in);
    floss_tc_assert_sim_clean();
}

TEST(can_send_now_false_without_session) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 0);
    start(t);
    ASSERT_FALSE(floss_hid_can_send_now());
    floss_tc_assert_sim_clean();
}

TEST(connect_failed_reports_reason) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 0);
    g_cfg.auto_reconnect = false;
    start(t);
    floss_sim_peer_add(&floss_tc_host, FLOSS_TC_HOST_COD);
    floss_sim_peer_set_present(&floss_tc_host, false);

    ASSERT_EQ(floss_hid_connect(&floss_tc_host), FLOSS_OK);
    /* Every retry is a 5.12 s page timeout plus the back-off. */
    floss_sim_pump_us(40000000);
    ASSERT_EQ(g_log.failed, 1);
    ASSERT_EQ(g_log.reason, FLOSS_HCI_ERR_PAGE_TIMEOUT);
    ASSERT_EQ(g_log.channel_status, 0);
    ASSERT_EQ(floss_hid_state(), FLOSS_HID_STATE_IDLE);
    floss_tc_assert_sim_clean();
}

static uint8_t pin_handler(const floss_addr_t *peer, uint8_t pin[16]) {
    /* A binary PIN: the peer's address, which may contain zero bytes. */
    memcpy(pin, peer->b, 6);
    return 6;
}

static uint8_t pin_refuse(const floss_addr_t *peer, uint8_t pin[16]) {
    (void)peer;
    (void)pin;
    return 0;
}

TEST(pin_handler_binary_pin) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 0);
    start(t);

    floss_gap_security_t sec;
    floss_gap_security_defaults(&sec);
    sec.pin_handler = pin_handler;
    ASSERT_EQ(floss_gap_set_security(&sec), FLOSS_OK);

    const floss_addr_t peer = {{0x00, 0x11, 0x00, 0x22, 0x33, 0x44}};
    ASSERT_EQ(floss_sim_inject_event(FLOSS_EVT_PIN_CODE_REQUEST, peer.b, 6), FLOSS_OK);
    floss_sim_pump(20);
    const floss_sim_cmd_t *rep = floss_sim_cmd_last(FLOSS_HCI_PIN_CODE_REQUEST_REPLY);
    ASSERT_NOT_NULL(rep);
    ASSERT_MEM_EQ(rep->params, peer.b, 6);
    ASSERT_EQ(rep->params[6], 6);
    ASSERT_MEM_EQ(&rep->params[7], peer.b, 6);

    /* Returning 0 refuses. */
    sec.pin_handler = pin_refuse;
    ASSERT_EQ(floss_gap_set_security(&sec), FLOSS_OK);
    ASSERT_EQ(floss_sim_inject_event(FLOSS_EVT_PIN_CODE_REQUEST, peer.b, 6), FLOSS_OK);
    floss_sim_pump(20);
    ASSERT_NOT_NULL(floss_sim_cmd_last(FLOSS_HCI_PIN_CODE_REQUEST_NEG_REPLY));
}

TEST(bond_add_answers_link_key_request) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 0);
    g_cfg.auto_reconnect = false;
    start(t);

    const floss_addr_t peer = {{0x01, 0x02, 0x03, 0x04, 0x05, 0x06}};
    floss_link_key_t key;
    for (int i = 0; i < 16; i++) {
        key.key[i] = (uint8_t)(0xA0 + i);
    }
    key.type = 0x04;
    static const floss_addr_t zero;
    ASSERT_EQ(floss_gap_bond_add(&zero, &key), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_gap_bond_add(&peer, &key), FLOSS_OK);
    ASSERT_EQ(floss_gap_bond_count(), 1);
    ASSERT_TRUE(floss_addr_eq(&floss_gap_bond_last()->addr, &peer));

    /* Persisted like any bond: a reboot still has it. */
    floss_tc_stack_start(t, &g_cfg);
    const floss_bond_t *b = floss_gap_bond_find(&peer);
    ASSERT_NOT_NULL(b);
    ASSERT_MEM_EQ(b->key.key, key.key, 16);
    ASSERT_EQ(b->key.type, 0x04);

    /* And it answers the controller's question. */
    ASSERT_EQ(floss_sim_inject_event(FLOSS_EVT_LINK_KEY_REQUEST, peer.b, 6), FLOSS_OK);
    floss_sim_pump(20);
    const floss_sim_cmd_t *rep = floss_sim_cmd_last(FLOSS_HCI_LINK_KEY_REQUEST_REPLY);
    ASSERT_NOT_NULL(rep);
    ASSERT_MEM_EQ(rep->params, peer.b, 6);
    ASSERT_MEM_EQ(&rep->params[6], key.key, 16);
}

TEST(unknown_cid_hook) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 0);
    start(t);
    floss_l2cap_set_unknown_cid_hook(on_unknown_cid);

    floss_sim_chan_t *ctrl, *intr;
    uint16_t h = floss_tc_hid_session(&ctrl, &intr);

    /* The peer restarted its stack and configures a channel that is gone. */
    const uint8_t cfg_req[] = {0x77, 0x00, 0x00, 0x00}; /* dcid 0x0077, flags 0 */
    floss_sim_l2cap_sig_send(h, 0x04, cfg_req, sizeof cfg_req);
    floss_sim_pump(20);
    ASSERT_EQ(g_log.unknown_cid, 1);
    ASSERT_EQ(g_log.unknown_cid_handle, h);
    ASSERT_EQ(g_log.unknown_cid_value, 0x0077);
    const floss_sim_sig_t *rej = floss_sim_sig_last(0x01); /* Command Reject */
    ASSERT_NOT_NULL(rej);

    /* floss_init clears the hook. */
    floss_tc_stack_start(t, &g_cfg);
    g_log.unknown_cid = 0;
    ASSERT_TRUE(floss_tc_wait_hid_connected(5000)); /* bonded: the stack reconnects by itself */
    floss_sim_l2cap_sig_send(floss_hid_handle(), 0x04, cfg_req, sizeof cfg_req);
    floss_sim_pump(20);
    ASSERT_EQ(g_log.unknown_cid, 0);
}

TEST(reinit_without_hid_forgets_session) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 1000);
    start(t);
    floss_sim_chan_t *ctrl, *intr;
    (void)floss_tc_hid_session(&ctrl, &intr);
    ASSERT_TRUE(floss_hid_connected());

    /* The stack restarts without the HID profile (e.g. to call hosts from another address). */
    floss_tc_stack_start(t, NULL);
    ASSERT_EQ(floss_hid_state(), FLOSS_HID_STATE_IDLE);
    ASSERT_FALSE(floss_hid_can_send_now());
    ASSERT_EQ(floss_hid_handle(), 0xFFFF);
    const uint8_t in[FLOSS_TC_INPUT_LEN] = {0};
    ASSERT_EQ(floss_hid_send_input_now(FLOSS_TC_INPUT_ID, in, sizeof in), FLOSS_ERR_STATE);
    floss_sim_pump(200);
    floss_tc_assert_sim_clean();
}

static const floss_test_case_t tests[] = {
    TEST_CASE(reinit_without_hid_forgets_session),
    TEST_CASE(raw_record_open_channels_app_scan),
    TEST_CASE(can_send_now_false_without_session),
    TEST_CASE(connect_failed_reports_reason),
    TEST_CASE(pin_handler_binary_pin),
    TEST_CASE(bond_add_answers_link_key_request),
    TEST_CASE(unknown_cid_hook),
};

FLOSS_TEST_MAIN("hid_integration", tests)
