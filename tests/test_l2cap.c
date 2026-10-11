/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * test_l2cap.c - L2CAP signaling edge cases, driven from the sim's remote peer.
 *
 * Hosts probe a device's L2CAP layer in ways the happy path never does:
 * connecting to PSMs it does not offer, echo (ping) requests, information
 * requests before anything else, hostile configuration values, requests for
 * channels that do not exist. Each test sends the raw signaling command and
 * checks the answer against the Core specification (Vol 3, Part A, ch. 4).
 *
 * Signaling payloads (little-endian):
 *   Connection Req   PSM, SCID               Rsp DCID, SCID, result, status
 *   Disconnect Req   DCID, SCID              Command Reject  reason [, data]
 *   Echo Req         data...                 Info Req        type
 */
#include "test_common.h"

static floss_hid_config_t g_cfg;

#define SIG_CMD_REJECT 0x01
#define SIG_DISC_REQ   0x06
#define SIG_DISC_RSP   0x07
#define SIG_ECHO_REQ   0x08
#define SIG_ECHO_RSP   0x09
#define SIG_INFO_REQ   0x0A
#define SIG_INFO_RSP   0x0B

/** Stack without HID (SDP server only), connectable; the PC connects. */
static uint16_t l2_setup(void) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_stack_start(t, NULL);
    return floss_tc_host_connect();
}

TEST(unknown_psm_refused) {
    uint16_t h = l2_setup();
    floss_sim_chan_t *ch = floss_sim_l2cap_connect(h, 0x1001); /* dynamic PSM nobody registered */
    ASSERT_NOT_NULL(ch);
    ASSERT_EQ(ch->state, FLOSS_SIM_CHAN_CLOSED);
    ASSERT_EQ(ch->result, 0x0002); /* "PSM not supported" */
    floss_tc_assert_sim_clean();
}

TEST(echo_request_answered) {
    uint16_t h = l2_setup();
    const uint8_t data[] = {'f', 'l', 'o', 's', 's'};
    uint8_t id = floss_sim_l2cap_sig_send(h, SIG_ECHO_REQ, data, sizeof data);
    ASSERT_NE(id, 0);
    floss_sim_pump(10);
    const floss_sim_sig_t *rsp = floss_sim_sig_find(SIG_ECHO_RSP, id);
    ASSERT_NOT_NULL(rsp); /* same identifier; echoing the data back is optional */
    ASSERT_EQ(rsp->handle, h);
    floss_tc_assert_sim_clean();
}

TEST(info_request_extended_features) {
    uint16_t h = l2_setup();
    const uint8_t req[] = {0x02, 0x00}; /* extended features mask */
    uint8_t id = floss_sim_l2cap_sig_send(h, SIG_INFO_REQ, req, sizeof req);
    floss_sim_pump(10);
    const floss_sim_sig_t *rsp = floss_sim_sig_find(SIG_INFO_RSP, id);
    ASSERT_NOT_NULL(rsp);
    ASSERT_GE(rsp->len, 8);
    ASSERT_EQ(floss_get_le16(rsp->data), 0x0002);     /* type echoed */
    ASSERT_EQ(floss_get_le16(rsp->data + 2), 0x0000); /* success */
    uint32_t mask = floss_get_le32(rsp->data + 4);

    if (mask & 0x80) {
        /* Fixed channels supported: the fixed channel map must then be answerable. */
        const uint8_t req3[] = {0x03, 0x00};
        id = floss_sim_l2cap_sig_send(h, SIG_INFO_REQ, req3, sizeof req3);
        floss_sim_pump(10);
        rsp = floss_sim_sig_find(SIG_INFO_RSP, id);
        ASSERT_NOT_NULL(rsp);
        ASSERT_GE(rsp->len, 12);
        ASSERT_EQ(floss_get_le16(rsp->data), 0x0003);
        ASSERT_EQ(floss_get_le16(rsp->data + 2), 0x0000);
        ASSERT_TRUE(rsp->data[4] & 0x02); /* signaling channel present */
    }
    floss_tc_assert_sim_clean();
}

TEST(info_request_unknown_type) {
    uint16_t h = l2_setup();
    const uint8_t req[] = {0x99, 0x00};
    uint8_t id = floss_sim_l2cap_sig_send(h, SIG_INFO_REQ, req, sizeof req);
    floss_sim_pump(10);
    const floss_sim_sig_t *rsp = floss_sim_sig_find(SIG_INFO_RSP, id);
    ASSERT_NOT_NULL(rsp);
    ASSERT_GE(rsp->len, 4);
    ASSERT_EQ(floss_get_le16(rsp->data), 0x0099);
    ASSERT_EQ(floss_get_le16(rsp->data + 2), 0x0001); /* not supported */
    floss_tc_assert_sim_clean();
}

TEST(config_mtu_below_minimum_rejected) {
    uint16_t h = l2_setup();
    /* The remote proposes MTU 40: below the 48-byte BR/EDR minimum. The
     * stack must answer "unacceptable parameters" (0x0001); the sim then
     * retries with the suggested value, and the channel opens. */
    floss_sim_cfg()->l2cap_mtu = 40;
    floss_sim_chan_t *ch = floss_sim_l2cap_connect(h, FLOSS_PSM_SDP);
    ASSERT_NOT_NULL(ch);
    ASSERT_GE(ch->cfg_rejects, 1);
    ASSERT_EQ(ch->cfg_reject_result, 0x0001);
    if (ch->cfg_suggested_mtu) {
        ASSERT_GE(ch->cfg_suggested_mtu, 48);
    }
    ASSERT_EQ(ch->state, FLOSS_SIM_CHAN_OPEN);
    ASSERT_GE(ch->sim_mtu, 48);
    floss_tc_assert_sim_clean();
}

TEST(disconnect_unknown_cid_rejected) {
    uint16_t h = l2_setup();
    uint8_t req[4];
    floss_put_le16(req, 0x7F00);     /* DCID: no such channel on the stack */
    floss_put_le16(req + 2, 0x0040); /* SCID */
    uint8_t id = floss_sim_l2cap_sig_send(h, SIG_DISC_REQ, req, sizeof req);
    floss_sim_pump(10);
    const floss_sim_sig_t *rej = floss_sim_sig_find(SIG_CMD_REJECT, id);
    ASSERT_NOT_NULL(rej);
    ASSERT_GE(rej->len, 2);
    ASSERT_EQ(floss_get_le16(rej->data), 0x0002); /* invalid CID in request */
    ASSERT_NULL(floss_sim_sig_find(SIG_DISC_RSP, id));
    floss_tc_assert_sim_clean();
}

TEST(unknown_command_rejected) {
    uint16_t h = l2_setup();
    const uint8_t junk[] = {0x00, 0x00};
    uint8_t id = floss_sim_l2cap_sig_send(h, 0x7E, junk, sizeof junk);
    floss_sim_pump(10);
    const floss_sim_sig_t *rej = floss_sim_sig_find(SIG_CMD_REJECT, id);
    ASSERT_NOT_NULL(rej);
    ASSERT_EQ(floss_get_le16(rej->data), 0x0000); /* command not understood */
    floss_tc_assert_sim_clean();
}

TEST(remote_open_and_close_channel) {
    uint16_t h = l2_setup();
    floss_sim_chan_t *ch = floss_sim_l2cap_connect(h, FLOSS_PSM_SDP);
    ASSERT_NOT_NULL(ch);
    ASSERT_EQ(ch->state, FLOSS_SIM_CHAN_OPEN);
    ASSERT_GE(ch->stack_cid, FLOSS_CID_DYNAMIC_FIRST);
    ASSERT_GE(ch->stack_mtu, 48);
    ASSERT_EQ(floss_sim_l2cap_disconnect(ch), FLOSS_OK);
    ASSERT_EQ(ch->state, FLOSS_SIM_CHAN_CLOSED);

    /* Reopening works and gets a channel again. */
    ch = floss_sim_l2cap_connect(h, FLOSS_PSM_SDP);
    ASSERT_NOT_NULL(ch);
    ASSERT_EQ(ch->state, FLOSS_SIM_CHAN_OPEN);
    floss_tc_assert_sim_clean();
}

TEST(sdp_over_tiny_acl_buffers) {
    /* Tiny controller buffers (27 bytes x 2), slow completions and 16-byte
     * fragments towards the stack: exercises TX fragmentation, credit
     * stalls and RX reassembly in one SDP exchange. */
    floss_sim_config_t cfg;
    floss_tc_sim_config(&cfg);
    cfg.acl_mtu = 27;
    cfg.acl_total = 2;
    cfg.acl_rx_frag = 16;
    cfg.nocp_delay_us = 700;
    floss_transport_t *t = floss_tc_sim_init(&cfg);
    floss_tc_hid_config(&g_cfg, 0);
    floss_tc_stack_start(t, &g_cfg);
    ASSERT_EQ(floss_hci_controller()->acl_mtu, 27);
    uint16_t h = floss_tc_host_connect();

    static uint8_t sdp[2048];
    int n = floss_sim_sdp_ssa(h, FLOSS_UUID_HID, sdp, sizeof sdp);
    ASSERT_GT(n, 0);
    ASSERT_NOT_NULL(floss_test_memfind(sdp, (size_t)n, floss_tc_report_desc, floss_tc_report_desc_len));
    ASSERT_GT(floss_sim_stats()->acl_in, 4); /* responses really were split */
    floss_tc_assert_sim_clean();
}

#if FLOSS_ENABLE_LE
TEST(le_att_mtu_exchange) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_stack_start(t, NULL);
    uint16_t h = floss_sim_le_connect(&floss_tc_host, 0 /* public */, 1 /* stack is peripheral */);
    ASSERT_NE(h, FLOSS_SIM_NO_HANDLE);
    floss_sim_pump(10);
    ASSERT_EQ(floss_sim_stats()->masked_events, 0); /* LE meta + connection complete enabled */

    const uint8_t req[] = {0x02, 0xF7, 0x00}; /* ATT Exchange MTU Request, client MTU 247 */
    ASSERT_EQ(floss_sim_att_send(h, req, sizeof req), FLOSS_OK);
    floss_sim_pump(10);
    uint8_t rsp[64];
    int len = floss_sim_att_recv(h, rsp, sizeof rsp);
    ASSERT_EQ(len, 3);
    ASSERT_EQ(rsp[0], 0x03); /* Exchange MTU Response */
    ASSERT_GE(floss_get_le16(rsp + 1), 23);
    floss_tc_assert_sim_clean();
}
#endif

static const floss_test_case_t tests[] = {
    TEST_CASE(unknown_psm_refused),
    TEST_CASE(echo_request_answered),
    TEST_CASE(info_request_extended_features),
    TEST_CASE(info_request_unknown_type),
    TEST_CASE(config_mtu_below_minimum_rejected),
    TEST_CASE(disconnect_unknown_cid_rejected),
    TEST_CASE(unknown_command_rejected),
    TEST_CASE(remote_open_and_close_channel),
    TEST_CASE(sdp_over_tiny_acl_buffers),
#if FLOSS_ENABLE_LE
    TEST_CASE(le_att_mtu_exchange),
#endif
};

FLOSS_TEST_MAIN("l2cap", tests)
