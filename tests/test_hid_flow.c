/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * test_hid_flow.c - End-to-end gamepad scenarios: whole stack + virtual
 * controller + scripted PC host.
 *
 * Each test is a story a real user would live through:
 *   - power on: the controller comes up and the pad becomes visible;
 *   - first pairing: the PC connects, pairs ("just works"), queries SDP,
 *     opens HID, input/output reports flow, then the PC disconnects;
 *   - security on demand: the PC opens HID without pairing first and the
 *     stack secures the link itself;
 *   - reboot: with a stored bond the pad pages the PC and restores HID on
 *     its own; and when the PC is away the page times out.
 *
 * HIDP framing used below (header = transaction type << 4 | parameter):
 *   0xA1 DATA/input  0xA2 DATA/output  0x41 GET_REPORT(input)
 *   0x71 SET_PROTOCOL(report)  0x0r HANDSHAKE(result r)
 */
#include "test_common.h"

static floss_hid_config_t g_cfg; /* must outlive the stack */

/** Pop every payload from a channel; true if one equals want (and *last gets the newest). */
static bool drain_find(floss_sim_chan_t *ch, const uint8_t *want, uint16_t want_len, uint8_t *last,
                       int *last_len) {
    uint8_t buf[128];
    bool found = false;
    int len;
    *last_len = -1;
    while ((len = floss_sim_l2cap_recv(ch, buf, sizeof buf)) >= 0) {
        if (len == want_len && memcmp(buf, want, want_len) == 0) {
            found = true;
        }
        memcpy(last, buf, FLOSS_MIN((size_t)len, sizeof buf));
        *last_len = len;
    }
    return found;
}

TEST(bringup_reaches_ready) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 1000);
    floss_tc_stack_start(t, &g_cfg);

    ASSERT_TRUE(floss_hci_ready());
    ASSERT_EQ(floss_tc_gap_count(FLOSS_GAP_EVT_READY), 1);

    /* A controller is always reset first: its state after power-up is unknown. */
    const floss_sim_cmd_t *first = floss_sim_cmd_at(0);
    ASSERT_NOT_NULL(first);
    ASSERT_EQ(first->opcode, FLOSS_HCI_RESET);

    /* Capabilities were read and understood. */
    ASSERT_NOT_NULL(floss_sim_cmd_first(FLOSS_HCI_READ_BUFFER_SIZE));
    ASSERT_NOT_NULL(floss_sim_cmd_first(FLOSS_HCI_READ_BD_ADDR));
    const floss_controller_info_t *ci = floss_hci_controller();
    ASSERT_EQ(ci->acl_mtu, 1021);
    ASSERT_EQ(ci->acl_total, 8);
    ASSERT_TRUE(floss_addr_eq(&ci->addr, floss_sim_addr()));

    /* Secure Simple Pairing on, gamepad class, visible + connectable (no bond yet). */
    const floss_sim_cmd_t *ssp = floss_sim_cmd_last(FLOSS_HCI_WRITE_SIMPLE_PAIRING_MODE);
    ASSERT_NOT_NULL(ssp);
    ASSERT_EQ(ssp->params[0], 0x01);
    const floss_sim_cmd_t *cod = floss_sim_cmd_last(FLOSS_HCI_WRITE_CLASS_OF_DEVICE);
    ASSERT_NOT_NULL(cod);
    ASSERT_EQ(floss_get_le24(cod->params), 0x002508);
    ASSERT_EQ(floss_sim_class_of_device(), 0x002508);
    const floss_sim_cmd_t *scan = floss_sim_cmd_last(FLOSS_HCI_WRITE_SCAN_ENABLE);
    ASSERT_NOT_NULL(scan);
    ASSERT_GT(scan->seq, first->seq);
    ASSERT_EQ(floss_sim_scan_enable(), 0x03);
    ASSERT_STR_EQ(floss_sim_local_name(), "FLOSS Test Pad");

    ASSERT_EQ(floss_sim_stats()->masked_events, 0);
    floss_tc_assert_sim_clean();
}

TEST(first_pairing_session) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 1000); /* paced: one report per millisecond */
    floss_tc_stack_start(t, &g_cfg);

    /* --- the PC connects and pairs ------------------------------------- */
    uint16_t h = floss_tc_host_connect();
    ASSERT_NOT_NULL(floss_tc_gap_last(FLOSS_GAP_EVT_CONNECTED));
    floss_tc_host_pair(h);

    const floss_sim_peer_t *peer = floss_sim_peer_find(&floss_tc_host);
    ASSERT_NOT_NULL(peer);
    ASSERT_TRUE(peer->has_key);
    ASSERT_TRUE(peer->key.type == 0x04 || peer->key.type == 0x07); /* unauthenticated: just works */

    /* The bond is persisted (survives reboot) and matches the key the controller produced. */
    uint8_t raw[FLOSS_TEST_STORE_VAL_MAX];
    ASSERT_GT(floss_hal_store_read(FLOSS_STORE_KEY_BOND_BASE, raw, sizeof raw), 0);
    ASSERT_EQ(floss_gap_bond_count(), 1);
    const floss_bond_t *bond = floss_gap_bond_find(&floss_tc_host);
    ASSERT_NOT_NULL(bond);
    ASSERT_MEM_EQ(bond->key.key, peer->key.key, 16);
    ASSERT_EQ(bond->key.type, peer->key.type);
    ASSERT_NOT_NULL(floss_tc_gap_last(FLOSS_GAP_EVT_PAIRING_COMPLETE));

    /* --- SDP: the PC looks for the HID service ------------------------- */
    static uint8_t sdp[2048];
    int n = floss_sim_sdp_ssa(h, FLOSS_UUID_HID, sdp, sizeof sdp);
    ASSERT_GT(n, 0);
    ASSERT_NOT_NULL(floss_test_memfind(sdp, (size_t)n, floss_tc_report_desc, floss_tc_report_desc_len));
    const uint8_t hid_uuid[] = {0x19, 0x11, 0x24};
    ASSERT_NOT_NULL(floss_test_memfind(sdp, (size_t)n, hid_uuid, sizeof hid_uuid));

    /* --- HID channels: control, then interrupt -------------------------- */
    floss_sim_chan_t *ctrl, *intr;
    floss_tc_hid_open(h, &ctrl, &intr);
    ASSERT_EQ(floss_tc_hid.connected, 1);
    ASSERT_TRUE(floss_addr_eq(&floss_tc_hid.host, &floss_tc_host));
    ASSERT_EQ(floss_hid_state(), FLOSS_HID_STATE_CONNECTED);
    ASSERT_EQ(floss_hid_handle(), h);

    /* --- input: set_input -> DATA(input) on the interrupt channel ------- */
    uint8_t in[FLOSS_TC_INPUT_LEN] = {0x01, 0x80, 0x10, 0x20, 0x30, 0x40};
    uint8_t want[2 + FLOSS_TC_INPUT_LEN] = {0xA1, FLOSS_TC_INPUT_ID};
    uint8_t last[128];
    int last_len;
    memcpy(want + 2, in, sizeof in);
    ASSERT_EQ(floss_hid_set_input(FLOSS_TC_INPUT_ID, in, sizeof in), FLOSS_OK);
    floss_sim_pump_us(3000);
    ASSERT_TRUE(drain_find(intr, want, sizeof want, last, &last_len));
    ASSERT_GE(floss_stats()->hid_reports_sent, 1);

    /* A newer state supersedes the old one within a pacing period or two. */
    in[0] = 0x02;
    want[2] = 0x02;
    ASSERT_EQ(floss_hid_set_input(FLOSS_TC_INPUT_ID, in, sizeof in), FLOSS_OK);
    floss_sim_pump_us(2500);
    ASSERT_TRUE(drain_find(intr, want, sizeof want, last, &last_len));
    ASSERT_EQ(last_len, (int)sizeof want);
    ASSERT_MEM_EQ(last, want, sizeof want); /* the newest report on the air is the newest state */

    /* --- output: DATA(output) from the PC -> on_output_report ----------- */
    const uint8_t rumble[] = {0xA2, FLOSS_TC_OUTPUT_ID, 0x55, 0xAA};
    ASSERT_EQ(floss_sim_l2cap_send(intr, rumble, sizeof rumble), FLOSS_OK);
    floss_sim_pump(20);
    ASSERT_EQ(floss_tc_hid.output_reports, 1);
    ASSERT_EQ(floss_tc_hid.output_id, FLOSS_TC_OUTPUT_ID);
    ASSERT_EQ(floss_tc_hid.output_len, 2);
    ASSERT_MEM_EQ(floss_tc_hid.output, rumble + 2, 2);

    /* --- GET_REPORT(input, id 1) on control -> DATA with the mailbox ---- */
    uint8_t buf[128];
    const uint8_t get_report[] = {0x41, FLOSS_TC_INPUT_ID};
    ASSERT_EQ(floss_sim_l2cap_send(ctrl, get_report, sizeof get_report), FLOSS_OK);
    int len = floss_sim_l2cap_recv_wait(ctrl, buf, sizeof buf);
    ASSERT_EQ(len, (int)sizeof want);
    ASSERT_MEM_EQ(buf, want, sizeof want);

    /* --- SET_PROTOCOL(report) -> HANDSHAKE ------------------------------ */
    const uint8_t set_protocol[] = {0x71};
    ASSERT_EQ(floss_sim_l2cap_send(ctrl, set_protocol, sizeof set_protocol), FLOSS_OK);
    len = floss_sim_l2cap_recv_wait(ctrl, buf, sizeof buf);
    ASSERT_EQ(len, 1);
    ASSERT_EQ(buf[0] & 0xF0, 0x00); /* a HANDSHAKE */
    /* Report protocol is always valid; a non-boot device may only refuse BOOT. */
    ASSERT_EQ(buf[0], FLOSS_HID_HS_SUCCESSFUL);
    ASSERT_EQ(floss_hid_protocol(), FLOSS_HID_PROTOCOL_REPORT);

    /* --- the PC goes away: interrupt, control, then the link ------------ */
    floss_sim_peer_set_present(&floss_tc_host, false); /* no instant auto-reconnect */
    ASSERT_EQ(floss_sim_l2cap_disconnect(intr), FLOSS_OK);
    if (ctrl->state == FLOSS_SIM_CHAN_OPEN) {
        ASSERT_EQ(floss_sim_l2cap_disconnect(ctrl), FLOSS_OK);
    }
    if (floss_sim_link(h)) {
        ASSERT_EQ(floss_sim_remote_disconnect(h, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED), FLOSS_OK);
    }
    floss_sim_pump(50);
    ASSERT_EQ(floss_tc_hid.disconnected, 1);
    ASSERT_FALSE(floss_hid_connected());
    ASSERT_NOT_NULL(floss_tc_gap_last(FLOSS_GAP_EVT_DISCONNECTED));

    floss_tc_assert_sim_clean();
}

TEST(stack_initiated_pairing) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 1000);
    floss_tc_stack_start(t, &g_cfg);

    /* The PC opens HID control straight away; HID requires encryption, so
     * the stack must authenticate (Authentication Requested) and pair as
     * the initiator before it accepts the channel. */
    uint16_t h = floss_tc_host_connect();
    floss_sim_chan_t *ctrl = floss_sim_l2cap_connect(h, FLOSS_PSM_HID_CONTROL);
    ASSERT_NOT_NULL(ctrl);
    ASSERT_EQ(ctrl->state, FLOSS_SIM_CHAN_OPEN);

    ASSERT_NOT_NULL(floss_sim_cmd_last(FLOSS_HCI_AUTHENTICATION_REQUESTED));
    ASSERT_NOT_NULL(floss_sim_cmd_last(FLOSS_HCI_IO_CAPABILITY_REQUEST_REPLY));
    ASSERT_NOT_NULL(floss_sim_cmd_last(FLOSS_HCI_SET_CONNECTION_ENCRYPTION));
    const floss_sim_link_t *link = floss_sim_link(h);
    ASSERT_NOT_NULL(link);
    ASSERT_TRUE(link->authenticated);
    ASSERT_NE(link->encrypted, 0);
    ASSERT_EQ(floss_gap_bond_count(), 1);

    floss_sim_chan_t *intr = floss_sim_l2cap_connect(h, FLOSS_PSM_HID_INTERRUPT);
    ASSERT_NOT_NULL(intr);
    ASSERT_EQ(intr->state, FLOSS_SIM_CHAN_OPEN);
    ASSERT_TRUE(floss_tc_wait_hid_connected(500));
    ASSERT_EQ(floss_tc_hid.connected, 1);
    floss_tc_assert_sim_clean();
}

TEST(reconnect_after_reboot) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 1000);
    floss_tc_stack_start(t, &g_cfg);
    floss_sim_chan_t *ctrl, *intr;
    (void)floss_tc_hid_session(&ctrl, &intr);
    ASSERT_EQ(floss_gap_bond_count(), 1);

    /* Reboot: fresh floss_init on the same "flash" (the RAM store persists).
     * Opening the transport power-cycles the virtual controller; the PC is
     * still around and remembers the bond. */
    floss_sim_log_clear();
    floss_tc_stack_start(t, &g_cfg);
    ASSERT_EQ(floss_gap_bond_count(), 1);

    /* The pad pages its bonded host... */
    const floss_sim_cmd_t *cc = floss_sim_expect_cmd(FLOSS_HCI_CREATE_CONNECTION);
    ASSERT_NOT_NULL(cc);
    ASSERT_MEM_EQ(cc->params, floss_tc_host.b, 6);

    /* ...authenticates with the stored key, then opens control and interrupt. */
    if (!floss_tc_wait_hid_connected(5000)) {
        FAIL("HID did not reconnect (state %d)", (int)floss_hid_state());
    }
    ASSERT_NOT_NULL(floss_sim_cmd_last(FLOSS_HCI_LINK_KEY_REQUEST_REPLY));
    ASSERT_NULL(floss_sim_cmd_last(FLOSS_HCI_LINK_KEY_REQUEST_NEG_REPLY));
    ASSERT_NULL(floss_sim_cmd_last(FLOSS_HCI_IO_CAPABILITY_REQUEST_REPLY)); /* no re-pairing */

    const floss_sim_link_t *link = floss_sim_link_by_addr(&floss_tc_host);
    ASSERT_NOT_NULL(link);
    ASSERT_TRUE(link->stack_initiated);
    ASSERT_TRUE(link->authenticated);
    ASSERT_NE(link->encrypted, 0);

    floss_sim_chan_t *c2 = floss_sim_chan_find(link->handle, FLOSS_PSM_HID_CONTROL);
    floss_sim_chan_t *i2 = floss_sim_chan_find(link->handle, FLOSS_PSM_HID_INTERRUPT);
    ASSERT_NOT_NULL(c2);
    ASSERT_NOT_NULL(i2);
    ASSERT_EQ(c2->state, FLOSS_SIM_CHAN_OPEN);
    ASSERT_EQ(i2->state, FLOSS_SIM_CHAN_OPEN);
    ASSERT_FALSE(c2->sim_initiated);
    ASSERT_FALSE(i2->sim_initiated);
    ASSERT_LT(c2->seq, i2->seq); /* control first, then interrupt */
    ASSERT_EQ(floss_tc_hid.connected, 1);

    /* Reports flow on the restored interrupt channel. */
    const uint8_t in[FLOSS_TC_INPUT_LEN] = {0x10, 0x00, 0x7F, 0x7F, 0x7F, 0x7F};
    uint8_t want[2 + FLOSS_TC_INPUT_LEN] = {0xA1, FLOSS_TC_INPUT_ID};
    uint8_t last[128];
    int last_len;
    memcpy(want + 2, in, sizeof in);
    ASSERT_EQ(floss_hid_set_input(FLOSS_TC_INPUT_ID, in, sizeof in), FLOSS_OK);
    floss_sim_pump_us(3000);
    ASSERT_TRUE(drain_find(i2, want, sizeof want, last, &last_len));
    floss_tc_assert_sim_clean();
}

TEST(reconnect_host_absent_page_timeout) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 1000);
    floss_tc_stack_start(t, &g_cfg);
    floss_sim_chan_t *ctrl, *intr;
    (void)floss_tc_hid_session(&ctrl, &intr);

    /* Reboot while the PC is switched off. */
    floss_sim_peer_set_present(&floss_tc_host, false);
    floss_sim_log_clear();
    floss_tc_stack_start(t, &g_cfg);
    ASSERT_NOT_NULL(floss_sim_expect_cmd(FLOSS_HCI_CREATE_CONNECTION));

    /* Page timeout is controller time (Write Page Timeout, default 5.12 s). */
    floss_sim_pump_us(6000000);
    const floss_gap_evt_t *fail = floss_tc_gap_last(FLOSS_GAP_EVT_CONNECT_FAILED);
    ASSERT_NOT_NULL(fail);
    ASSERT_EQ(fail->status, FLOSS_HCI_ERR_PAGE_TIMEOUT);
    ASSERT_FALSE(floss_hid_connected());
    ASSERT_EQ(floss_tc_hid.connected, 0);
    floss_tc_assert_sim_clean();
}

TEST(app_link_to_host_kept) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 1000);
    floss_tc_stack_start(t, &g_cfg);
    floss_sim_chan_t *ctrl, *intr;
    (void)floss_tc_hid_session(&ctrl, &intr);

    /* HID closes its session and goes idle; the PC stays its host. */
    ASSERT_EQ(floss_hid_disconnect(), FLOSS_OK);
    floss_sim_pump_us(2000000);
    ASSERT_EQ(floss_hid_state(), FLOSS_HID_STATE_IDLE);
    ASSERT_NULL(floss_hci_conn_by_addr(&floss_tc_host));

    /* The application calls the same PC for something else (RFCOMM, say): HID did not
     * page it, so the link stays up and can be secured with the bond. */
    floss_sim_log_clear();
    ASSERT_EQ(floss_gap_connect(&floss_tc_host), FLOSS_OK);
    floss_sim_pump_us(500000);
    const floss_conn_t *c = floss_hci_conn_by_addr(&floss_tc_host);
    ASSERT_NOT_NULL(c);
    ASSERT_EQ(floss_gap_authenticate(c->handle), FLOSS_OK);
    floss_sim_pump_us(2000000);
    ASSERT_NULL(floss_sim_cmd_last(FLOSS_HCI_DISCONNECT));
    c = floss_hci_conn_by_addr(&floss_tc_host);
    ASSERT_NOT_NULL(c);
    ASSERT_TRUE(c->encrypted);
    const floss_gap_evt_t *auth = floss_tc_gap_last(FLOSS_GAP_EVT_AUTHENTICATED);
    ASSERT_NOT_NULL(auth);
    ASSERT_EQ(auth->status, FLOSS_HCI_SUCCESS);
    ASSERT_EQ(floss_hid_state(), FLOSS_HID_STATE_IDLE);
}

/* Restart the stack with the controller reporting `local` as our address. */
static void start_as(const floss_addr_t *local) {
    floss_sim_config_t cfg;
    floss_tc_sim_config(&cfg);
    cfg.addr = *local;
    floss_tc_stack_start(floss_tc_sim_init(&cfg), NULL);
}

/*
 * A device that changes its own address keeps one bond per address pair:
 * a host rejecting the key under one local address (here: deleting the
 * bond for the current pair) must not cost the bond made under another.
 */
TEST(bonds_keyed_by_address_pair) {
    const floss_addr_t local_a = {{0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0x02}};
    const floss_addr_t local_b = {{0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0x02}};
    const floss_addr_t peer = {{0x01, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA}};
    floss_link_key_t key_a = {{0xAA}, 0x07};
    floss_link_key_t key_b = {{0xBB}, 0x07};

    floss_test_store_reset();
    start_as(&local_a);
    ASSERT_EQ(floss_gap_bond_add(&peer, &key_a), FLOSS_OK);
    ASSERT_MEM_EQ(floss_gap_bond_find(&peer)->local.b, local_a.b, 6);

    /* Same host, other local address: the A bond is not ours to use... */
    start_as(&local_b);
    ASSERT_EQ(floss_gap_bond_count(), 1);
    ASSERT_NULL(floss_gap_bond_find(&peer));
    ASSERT_NULL(floss_gap_bond_last());
    /* ...a B bond is separate, and forgetting it leaves A's alone. */
    ASSERT_EQ(floss_gap_bond_add(&peer, &key_b), FLOSS_OK);
    ASSERT_EQ(floss_gap_bond_count(), 2);
    ASSERT_EQ(floss_gap_bond_find(&peer)->key.key[0], 0xBB);
    ASSERT_EQ(floss_gap_bond_delete(&peer), FLOSS_OK);
    ASSERT_EQ(floss_gap_bond_count(), 1);

    /* Back under A: the original key is still there. */
    start_as(&local_a);
    ASSERT_NOT_NULL(floss_gap_bond_find(&peer));
    ASSERT_EQ(floss_gap_bond_find(&peer)->key.key[0], 0xAA);
    ASSERT_NOT_NULL(floss_gap_bond_last());
}

/* Forgetting a host takes every bond with it, loaded or only stored, and nothing else. */
TEST(bond_forget_host) {
    const floss_addr_t local_a = {{0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0x02}};
    const floss_addr_t local_b = {{0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0x02}};
    const floss_addr_t peer = {{0x03, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA}};
    const floss_addr_t other = {{0x04, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA}};
    floss_link_key_t key = {{0xCC}, 0x07};

    floss_test_store_reset();
    start_as(&local_a);
    ASSERT_EQ(floss_gap_bond_add(&peer, &key), FLOSS_OK);
    ASSERT_EQ(floss_gap_bond_add(&other, &key), FLOSS_OK);
    start_as(&local_b);
    ASSERT_EQ(floss_gap_bond_add(&peer, &key), FLOSS_OK);
    ASSERT_EQ(floss_gap_bond_count(), 3);

    /* Both of the host's bonds go, under either local address; the other host's stays. */
    uint8_t rec[64];
    int n = floss_hal_store_read(FLOSS_STORE_KEY_BOND_BASE, rec, sizeof rec); /* the first: peer under A */
    ASSERT_GT(n, 0);
    ASSERT_EQ(floss_gap_bond_forget(&peer), FLOSS_OK);
    ASSERT_EQ(floss_gap_bond_count(), 1);
    ASSERT_EQ(floss_gap_bond_forget(&peer), FLOSS_ERR_NOT_FOUND);

    /* A record the stack never loaded (written behind its back) is found in the store. */
    ASSERT_EQ(floss_hal_store_write(FLOSS_STORE_KEY_BOND_BASE, rec, (uint16_t)n), FLOSS_OK);
    ASSERT_EQ(floss_gap_bond_forget(&peer), FLOSS_OK);
    ASSERT_TRUE(floss_hal_store_read(FLOSS_STORE_KEY_BOND_BASE, rec, sizeof rec) <= 0);

    start_as(&local_a);
    ASSERT_EQ(floss_gap_bond_count(), 1);
    ASSERT_NULL(floss_gap_bond_find(&peer));
    ASSERT_NOT_NULL(floss_gap_bond_find(&other));
}

/* Records written before bonds carried the local address match any address. */
TEST(bond_record_v1_matches_any_local_address) {
    const floss_addr_t peer = {{0x02, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA}};
    const floss_addr_t local_b = {{0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0x02}};
    uint8_t v1[31] = {1};
    memcpy(&v1[1], peer.b, 6);
    v1[7] = 0x5A;              /* key[0] */
    v1[23] = 0x07;             /* key type */
    floss_put_le32(&v1[27], 9); /* seq */

    floss_test_store_reset();
    ASSERT_EQ(floss_hal_store_write(FLOSS_STORE_KEY_BOND_BASE, v1, sizeof v1), FLOSS_OK);
    start_as(&local_b);
    const floss_bond_t *b = floss_gap_bond_find(&peer);
    ASSERT_NOT_NULL(b);
    ASSERT_EQ(b->key.key[0], 0x5A);
    ASSERT_TRUE(floss_addr_is_zero(&b->local));
}

static const floss_test_case_t tests[] = {
    TEST_CASE(bonds_keyed_by_address_pair),
    TEST_CASE(bond_forget_host),
    TEST_CASE(bond_record_v1_matches_any_local_address),
    TEST_CASE(bringup_reaches_ready),
    TEST_CASE(first_pairing_session),
    TEST_CASE(stack_initiated_pairing),
    TEST_CASE(reconnect_after_reboot),
    TEST_CASE(reconnect_host_absent_page_timeout),
    TEST_CASE(app_link_to_host_kept),
};

FLOSS_TEST_MAIN("hid_flow", tests)
