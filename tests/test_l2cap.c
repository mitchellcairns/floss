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

#include "floss_internal.h"
#include "floss/floss_gatt.h"
#include "floss/floss_le.h"
#include "floss/floss_rfcomm.h"
#include "hci/floss_hci_priv.h"
#include "rfcomm/floss_rfcomm_int.h"

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

/* ======================================================================== */
/* Robustness regressions (each found by tests/fuzz/floss_fuzz.c)          */
/* ======================================================================== */

/** Let fake time run in big steps (timer-driven recoveries). */
static void pump_seconds(uint32_t s) {
    floss_sim_config_t *c = floss_sim_cfg();
    uint32_t old = c->pump_step_us;
    c->pump_step_us = 10000;
    floss_sim_pump(s * 100u);
    c->pump_step_us = old;
}

static unsigned used_hci_slots(void) {
    unsigned n = 0;
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        n += floss_hci_conn_at(i)->state != FLOSS_CONN_FREE;
    }
    return n;
}

/* A controller that answers with Num_HCI_Command_Packets = 0 and never sends
 * the NOP that should follow used to wedge the command queue for good. */
TEST(hci_zero_command_credit_recovers) {
    l2_setup();
    floss_sim_pump(20);
    const uint8_t nop_no_credit[] = {0x00, 0x00, 0x00}; /* credits 0, opcode 0x0000 */
    ASSERT_EQ(floss_sim_inject_event(FLOSS_EVT_COMMAND_COMPLETE, nop_no_credit, sizeof nop_no_credit), FLOSS_OK);
    floss_sim_pump(5);
    floss_sim_log_clear();
    ASSERT_EQ(floss_hci_cmd(FLOSS_HCI_READ_BD_ADDR, NULL, 0, NULL, NULL), FLOSS_OK);
    floss_sim_pump(10);
    ASSERT_EQ(floss_sim_cmd_count(FLOSS_HCI_READ_BD_ADDR), 0); /* honours the missing credit... */
    pump_seconds(3);
    ASSERT_EQ(floss_sim_cmd_count(FLOSS_HCI_READ_BD_ADDR), 1); /* ...but not forever */
    ASSERT_EQ(floss_buf_free_count(), FLOSS_BUF_COUNT);
}

/* A Write Scan Enable that found the HCI command queue full (a controller
 * flooding events while granting no command credit) used to be dropped,
 * leaving the device unreachable for good. */
TEST(gap_scan_enable_survives_full_command_queue) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_stack_start(t, NULL);
    ASSERT_EQ(floss_gap_set_scan(false, false), FLOSS_OK);
    floss_sim_pump(20);
    ASSERT_EQ(floss_sim_scan_enable(), 0x00);
    const uint8_t nop_no_credit[] = {0x00, 0x00, 0x00};
    ASSERT_EQ(floss_sim_inject_event(FLOSS_EVT_COMMAND_COMPLETE, nop_no_credit, sizeof nop_no_credit), FLOSS_OK);
    floss_sim_pump(5);
    while (floss_hci_cmd(FLOSS_HCI_READ_BD_ADDR, NULL, 0, NULL, NULL) == FLOSS_OK) {
    }
    ASSERT_EQ(floss_gap_set_scan(true, true), FLOSS_OK);
    pump_seconds(5);
    ASSERT_EQ(floss_sim_scan_enable(), 0x03);
    floss_tc_host_connect();
}

/* Pairing events about devices we have no link with used to take GAP link
 * slots for good, until no host could connect any more. */
TEST(gap_pairing_events_for_strangers_take_no_slot) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_stack_start(t, NULL);
    for (uint8_t i = 0; i < 2 * FLOSS_MAX_CONNECTIONS; i++) {
        uint8_t e[23] = {0x10, 0x20, 0x30, 0x40, 0x50, (uint8_t)(0x60 + i)};
        e[22] = 0x04; /* Link Key Notification: key type */
        ASSERT_EQ(floss_sim_inject_event(FLOSS_EVT_IO_CAPABILITY_REQUEST, e, 6), FLOSS_OK);
        ASSERT_EQ(floss_sim_inject_event(FLOSS_EVT_PIN_CODE_REQUEST, e, 6), FLOSS_OK);
        ASSERT_EQ(floss_sim_inject_event(FLOSS_EVT_LINK_KEY_NOTIFICATION, e, sizeof e), FLOSS_OK);
        floss_sim_pump(10);
    }
    uint16_t h = floss_tc_host_connect(); /* fails if the link table filled up */
    ASSERT_NOT_NULL(floss_hci_conn_by_handle(h));
}

/* Connection Requests whose acceptance the controller never concludes used to
 * keep their connection slots INCOMING forever. */
TEST(hci_unconcluded_incoming_slots_expire) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_stack_start(t, NULL);
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        /* Unknown to the sim: it refuses the Accept, so no Connection Complete follows. */
        const uint8_t req[10] = {0x11, 0x22, 0x33, 0x44, 0x55, (uint8_t)(0x66 + i), 0x0C, 0x01, 0x00, 0x01};
        ASSERT_EQ(floss_sim_inject_event(FLOSS_EVT_CONNECTION_REQUEST, req, sizeof req), FLOSS_OK);
        floss_sim_pump(10);
    }
    ASSERT_EQ(used_hci_slots(), FLOSS_MAX_CONNECTIONS);
    pump_seconds(FLOSS_HCI_CONN_PENDING_TIMEOUT_US / 1000000u + 2u);
    ASSERT_EQ(used_hci_slots(), 0);
    uint16_t h = floss_tc_host_connect();
    ASSERT_NOT_NULL(floss_hci_conn_by_handle(h));
}

#if FLOSS_ENABLE_LE
static void inject_le_conn_complete(uint16_t handle, uint8_t addr_lsb, uint8_t len) {
    /* LE Connection Complete: sub, status, handle, role (peripheral), addr type, addr, interval,
     * latency, supervision timeout, clock accuracy. */
    const uint8_t e[19] = {0x01, 0x00, (uint8_t)handle, (uint8_t)(handle >> 8), 0x01, 0x00, addr_lsb,
                           0x22, 0x33, 0x44, 0x55, 0x66, 24, 0, 0, 0, 0x2C, 0x01, 0x00};
    ASSERT_EQ(floss_sim_inject_event(FLOSS_EVT_LE_META, e, len), FLOSS_OK);
    floss_sim_pump(10);
}

/* A second LE Connection Complete for a live handle used to create a second
 * HCI slot and LE link with that handle; one of them outlived the link. */
TEST(le_duplicate_connection_complete_ignored) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_stack_start(t, NULL);
    uint16_t h = floss_sim_le_connect(&floss_tc_host, 0, 1);
    floss_sim_pump(10);
    inject_le_conn_complete(h, 0x99, 19);
    ASSERT_EQ(floss_le_link_count(), 1);
    ASSERT_EQ(used_hci_slots(), 1);
    ASSERT_EQ(floss_sim_remote_disconnect(h, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED), FLOSS_OK);
    floss_sim_pump(20);
    ASSERT_EQ(floss_le_link_count(), 0);
    ASSERT_EQ(used_hci_slots(), 0);
}

/* A truncated LE Connection Complete used to open an HCI slot the LE module
 * (which needs all 18 bytes) never knew about. */
TEST(le_short_connection_complete_ignored) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_stack_start(t, NULL);
    inject_le_conn_complete(0x0055, 0x77, 13);
    ASSERT_NULL(floss_hci_conn_by_handle(0x0055));
    ASSERT_EQ(floss_le_link_count(), 0);
}

/* With the HCI table full, an LE link used to get an LE slot anyway; HCI
 * could not route its Disconnection Complete, so that slot never freed. */
TEST(le_link_beyond_hci_table_is_dropped) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_stack_start(t, NULL);
    floss_tc_host_connect();
    floss_sim_le_connect(&floss_tc_host, 0, 1);
    floss_sim_pump(10);
    ASSERT_EQ(used_hci_slots(), FLOSS_MAX_CONNECTIONS);
    floss_sim_log_clear();
    inject_le_conn_complete(0x0077, 0x88, 19);
    ASSERT_EQ(floss_le_link_count(), 1);
    const floss_sim_cmd_t *disc = floss_sim_cmd_last(FLOSS_HCI_DISCONNECT);
    ASSERT_NOT_NULL(disc);
    ASSERT_EQ(floss_get_le16(disc->params) & 0x0FFF, 0x0077);
}

/* A long writable value for the ATT tests. */
static uint8_t g_long[64];
static uint16_t g_long_len;
static const floss_gatt_attr_t k_long_svc[] = {
    FLOSS_GATT_PRIMARY_SERVICE16(0xFFF0),
    FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_READ | FLOSS_GATT_PROP_WRITE),
    FLOSS_GATT_VALUE16(0xFFF1, FLOSS_GATT_P_RW, g_long, sizeof g_long, &g_long_len),
};

/** LE link (ATT MTU 23) to a server with k_long_svc; *value_handle = its value. */
static uint16_t att_setup(uint16_t *value_handle) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_stack_start(t, NULL);
    uint16_t base = floss_gatt_register(k_long_svc, 3);
    ASSERT_NE(base, 0);
    *value_handle = (uint16_t)(base + 2);
    g_long_len = 0;
    uint16_t h = floss_sim_le_connect(&floss_tc_host, 0, 1);
    floss_sim_pump(10);
    return h;
}

static int att_transact(uint16_t h, const uint8_t *req, uint16_t len, uint8_t *rsp, uint16_t cap) {
    ASSERT_EQ(floss_sim_att_send(h, req, len), FLOSS_OK);
    floss_sim_pump(10);
    return floss_sim_att_recv(h, rsp, cap);
}

/* Gaps between prepared segments used to be filled with an earlier queue's
 * bytes (possibly another link's) instead of zeros. */
TEST(att_prepare_write_gaps_hold_no_stale_bytes) {
    uint16_t vh;
    uint16_t h = att_setup(&vh);
    uint8_t rsp[64];
    uint8_t req[9] = {FLOSS_ATT_PREPARE_WRITE_REQ, (uint8_t)vh, (uint8_t)(vh >> 8), 0, 0, 'A', 'A', 'A', 'A'};
    const uint8_t exec[2] = {FLOSS_ATT_EXECUTE_WRITE_REQ, 0x01};
    ASSERT_EQ(att_transact(h, req, 9, rsp, sizeof rsp), 9);
    ASSERT_EQ(att_transact(h, exec, 2, rsp, sizeof rsp), 1);

    req[5] = 'B'; /* offset 0: "B" */
    ASSERT_EQ(att_transact(h, req, 6, rsp, sizeof rsp), 6);
    req[3] = 3; /* offset 3: "C" */
    req[5] = 'C';
    ASSERT_EQ(att_transact(h, req, 6, rsp, sizeof rsp), 6);
    ASSERT_EQ(att_transact(h, exec, 2, rsp, sizeof rsp), 1);
    ASSERT_EQ(g_long_len, 4);
    const uint8_t want[4] = {'B', 0, 0, 'C'};
    ASSERT_MEM_EQ(g_long, want, 4);
}

/* The Prepare Write Response echoes the request: one larger than the
 * client's MTU used to be echoed anyway. */
TEST(att_prepare_write_beyond_mtu_rejected) {
    uint16_t vh;
    uint16_t h = att_setup(&vh);
    uint8_t req[30] = {FLOSS_ATT_PREPARE_WRITE_REQ, (uint8_t)vh, (uint8_t)(vh >> 8), 0, 0};
    uint8_t rsp[64];
    ASSERT_EQ(att_transact(h, req, sizeof req, rsp, sizeof rsp), 5);
    ASSERT_EQ(rsp[0], FLOSS_ATT_ERROR_RSP);
    ASSERT_EQ(rsp[4], FLOSS_ATT_ERR_INVALID_PDU);
}

/* A read callback claiming more bytes than it was given room for must not
 * make ATT copy or send past its buffer. */
static int overclaiming_read(uint16_t conn, uint16_t handle, const floss_gatt_attr_t *attr, uint16_t offset,
                             uint8_t *out, uint16_t cap) {
    memset(out, 0x5A, cap);
    return cap + 40;
}

static const floss_gatt_attr_t k_overclaim_svc[] = {
    FLOSS_GATT_PRIMARY_SERVICE16(0xFFE0),
    FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_READ),
    FLOSS_GATT_VALUE16_CB(0xFFE1, FLOSS_GATT_P_READ, overclaiming_read, NULL, 0, NULL),
};

TEST(gatt_read_callback_count_clamped) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_stack_start(t, NULL);
    uint16_t base = floss_gatt_register(k_overclaim_svc, 3);
    ASSERT_NE(base, 0);
    uint16_t h = floss_sim_le_connect(&floss_tc_host, 0, 1);
    floss_sim_pump(10);
    const uint8_t req[3] = {FLOSS_ATT_READ_REQ, (uint8_t)(base + 2), (uint8_t)((base + 2) >> 8)};
    uint8_t rsp[256];
    ASSERT_EQ(att_transact(h, req, sizeof req, rsp, sizeof rsp), FLOSS_ATT_DEFAULT_MTU);
}
/** Start Secure Connections pairing as the central: request, then our public key. */
static void smp_sc_start(uint16_t h) {
    const uint8_t req[7] = {0x01, 0x03 /* NoInputNoOutput */, 0x00, 0x09 /* bonding | SC */, 16, 0, 0};
    ASSERT_EQ(floss_sim_fixed_send(h, FLOSS_SIM_CID_SMP, req, sizeof req), FLOSS_OK);
    floss_sim_pump(20);
    uint8_t pk[65] = {0x0C};
    for (unsigned i = 1; i < sizeof pk; i++) {
        pk[i] = (uint8_t)(i * 7u);
    }
    ASSERT_EQ(floss_sim_fixed_send(h, FLOSS_SIM_CID_SMP, pk, sizeof pk), FLOSS_OK);
    floss_sim_pump(50);
}

/* A controller that accepted LE Generate DHKey but never delivered the DHKey
 * used to block Secure Connections pairing on every link until reboot. */
TEST(smp_lost_dhkey_event_does_not_block_pairing) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_stack_start(t, NULL);
    const uint16_t gen_dhkey = FLOSS_OPCODE(0x08, 0x0026);
    floss_sim_cfg()->drop_le_subevent = 0x09; /* LE Generate DHKey Complete */
    uint16_t h = floss_sim_le_connect(&floss_tc_host, 0, 1);
    floss_sim_pump(10);
    smp_sc_start(h);
    ASSERT_EQ(floss_sim_cmd_count(gen_dhkey), 1);
    ASSERT_EQ(floss_sim_remote_disconnect(h, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED), FLOSS_OK);
    pump_seconds(15);

    floss_sim_cfg()->drop_le_subevent = 0;
    h = floss_sim_le_connect(&floss_tc_host, 0, 1);
    floss_sim_pump(10);
    smp_sc_start(h);
    ASSERT_EQ(floss_sim_cmd_count(gen_dhkey), 2); /* the controller is asked again */
}
#endif /* FLOSS_ENABLE_LE */

/* A peer that stops acknowledging while it keeps sending requests used to
 * pin every pool buffer with queued responses, starving the whole stack. */
TEST(acl_tx_queue_bounded_when_peer_stalls) {
    uint16_t h = l2_setup();
    floss_sim_cfg()->nocp_delay_us = 0x40000000u; /* the air link stops draining */
    for (unsigned i = 0; i < 3u * FLOSS_BUF_COUNT; i++) {
        const uint8_t data[4] = {1, 2, 3, (uint8_t)i};
        floss_sim_l2cap_sig_send(h, SIG_ECHO_REQ, data, sizeof data);
        floss_sim_pump(3);
    }
    ASSERT_GE(floss_buf_free_count(), FLOSS_BUF_COUNT - FLOSS_ACL_TXQ_MAX);
    ASSERT_EQ(floss_sim_remote_disconnect(h, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED), FLOSS_OK);
    floss_sim_pump(20);
    ASSERT_EQ(floss_buf_free_count(), FLOSS_BUF_COUNT);
}

/* An SDP response deferred for lack of a buffer used to be overwritten by the
 * next request's answer, then re-sent from the shared buffer with its old
 * length: the client got the newer answer twice and the older never. */
TEST(sdp_deferred_response_not_overwritten) {
    uint16_t h = l2_setup();
    floss_sim_chan_t *ch = floss_sim_l2cap_connect(h, FLOSS_PSM_SDP);
    ASSERT_NOT_NULL(ch);
    ASSERT_EQ(ch->state, FLOSS_SIM_CHAN_OPEN);
    /* ServiceSearchRequest: pattern {L2CAP}, max 4 records, no continuation. */
    uint8_t req[] = {0x02, 0x00, 0x01, 0x00, 0x08, 0x35, 0x03, 0x19, 0x01, 0x00, 0x00, 0x04, 0x00};

    floss_buf_t *held[FLOSS_BUF_COUNT];
    unsigned n_held = 0;
    floss_buf_t *b;
    while ((b = floss_buf_alloc()) != NULL) {
        held[n_held++] = b; /* pool empty: the first answer has to wait */
    }
    ASSERT_EQ(floss_sim_l2cap_send(ch, req, sizeof req), FLOSS_OK);
    floss_sim_pump(5);
    ASSERT_EQ(floss_sim_l2cap_pending(ch), 0);
    while (n_held) {
        floss_buf_free(held[--n_held]);
    }
    req[2] = 0x02; /* transaction 2 */
    ASSERT_EQ(floss_sim_l2cap_send(ch, req, sizeof req), FLOSS_OK);
    floss_sim_pump(20);

    uint8_t rsp[64];
    ASSERT_EQ(floss_sim_l2cap_pending(ch), 2);
    ASSERT_GT(floss_sim_l2cap_recv(ch, rsp, sizeof rsp), 4);
    ASSERT_EQ(floss_get_be16(rsp + 1), 1); /* the deferred answer, intact and first */
    ASSERT_GT(floss_sim_l2cap_recv(ch, rsp, sizeof rsp), 4);
    ASSERT_EQ(floss_get_be16(rsp + 1), 2);
}

#if FLOSS_ENABLE_RFCOMM
static struct {
    floss_rfcomm_dlc_t *dlc;
    unsigned frames;
    uint16_t max_len;
} g_rfc;

static void rfc_on_open(floss_rfcomm_dlc_t *dlc, uint8_t status) {
    if (status == FLOSS_RFCOMM_STATUS_OK) {
        g_rfc.dlc = dlc;
        floss_rfcomm_set_manual_credits(dlc, true); /* the tests count credits themselves */
    }
}

static void rfc_on_data(floss_rfcomm_dlc_t *dlc, const uint8_t *data, uint16_t len) {
    g_rfc.frames++;
    g_rfc.max_len = FLOSS_MAX(g_rfc.max_len, len);
}

static const floss_rfcomm_ops_t k_rfc_ops = {.on_open = rfc_on_open, .on_data = rfc_on_data};

/** Send one RFCOMM frame as the session initiator (the sim) would; no credit byte. */
static void rfc_send(floss_sim_chan_t *ch, uint8_t dlci, uint8_t type, const uint8_t *info, uint16_t len) {
    uint8_t f[700];
    uint16_t n = 0;
    bool cr = type != RFC_UA && type != RFC_DM; /* initiator: commands carry C/R = 1 */
    f[n++] = (uint8_t)(RFC_EA | (cr ? RFC_CR : 0) | (dlci << 2));
    f[n++] = (uint8_t)(type | (type != RFC_UIH ? RFC_PF : 0));
    if (len <= 127) {
        f[n++] = (uint8_t)((len << 1) | RFC_EA);
    } else {
        f[n++] = (uint8_t)(len << 1);
        f[n++] = (uint8_t)(len >> 7);
    }
    uint8_t fcs = floss_rfcomm_fcs(f, type == RFC_UIH ? 2 : n);
    if (len) {
        memcpy(f + n, info, len);
    }
    n = (uint16_t)(n + len);
    f[n++] = fcs;
    ASSERT_EQ(floss_sim_l2cap_send(ch, f, n), FLOSS_OK);
    floss_sim_pump(10);
}

/* PN for DLCI 2 (server channel 1): credit flow, N1 = 100, 7 initial credits. */
static const uint8_t k_pn_dlci2[10] = {RFC_MCC_PN | RFC_CR | RFC_EA, (8 << 1) | RFC_EA, 2, 0xF0, 0, 0, 100, 0, 0, 7};

/** Server channel 1 open on DLCI 2: credit flow, N1 = 100, 7 credits each way. */
static floss_sim_chan_t *rfc_setup(void) {
    memset(&g_rfc, 0, sizeof g_rfc);
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_stack_start(t, NULL);
    ASSERT_EQ(floss_rfcomm_register_server(1, &k_rfc_ops, NULL), FLOSS_OK);
    uint16_t h = floss_tc_host_connect();
    floss_tc_host_pair(h);
    floss_sim_chan_t *ch = floss_sim_l2cap_connect(h, FLOSS_PSM_RFCOMM);
    ASSERT_NOT_NULL(ch);
    ASSERT_EQ(ch->state, FLOSS_SIM_CHAN_OPEN);
    rfc_send(ch, 0, RFC_SABM, NULL, 0);
    rfc_send(ch, 0, RFC_UIH, k_pn_dlci2, sizeof k_pn_dlci2);
    rfc_send(ch, 2, RFC_SABM, NULL, 0);
    ASSERT_NOT_NULL(g_rfc.dlc);
    ASSERT_EQ(g_rfc.dlc->mtu, 100);
    ASSERT_EQ(g_rfc.dlc->rx_credits, 7);
    uint8_t drop[700];
    while (floss_sim_l2cap_recv(ch, drop, sizeof drop) >= 0) {
    }
    return ch;
}

/* Frames longer than the negotiated N1 used to reach on_data, overflowing
 * applications that sized their buffers for dlc->mtu. */
TEST(rfcomm_frame_beyond_mtu_dropped) {
    floss_sim_chan_t *ch = rfc_setup();
    const uint8_t big[200] = {0};
    rfc_send(ch, 2, RFC_UIH, big, sizeof big);
    ASSERT_EQ(g_rfc.frames, 0);
    rfc_send(ch, 2, RFC_UIH, big, 100);
    ASSERT_EQ(g_rfc.frames, 1);
    ASSERT_EQ(g_rfc.max_len, 100);
}

/* Frames sent without a credit used to be delivered anyway, defeating the
 * back-pressure of applications that grant credits themselves. */
TEST(rfcomm_frame_without_credit_dropped) {
    floss_sim_chan_t *ch = rfc_setup();
    const uint8_t data[10] = {0};
    for (unsigned i = 0; i < 8; i++) {
        rfc_send(ch, 2, RFC_UIH, data, sizeof data);
    }
    ASSERT_EQ(g_rfc.frames, 7);
    ASSERT_EQ(g_rfc.dlc->rx_credits, 0);
}

/* A PN on an open DLC used to reset the receive credit count to the initial
 * window, out of step with the credits the peer actually holds. */
TEST(rfcomm_pn_on_open_dlc_keeps_credits) {
    floss_sim_chan_t *ch = rfc_setup();
    const uint8_t data[10] = {0};
    rfc_send(ch, 2, RFC_UIH, data, sizeof data);
    rfc_send(ch, 2, RFC_UIH, data, sizeof data);
    ASSERT_EQ(g_rfc.dlc->rx_credits, 5);
    rfc_send(ch, 0, RFC_UIH, k_pn_dlci2, sizeof k_pn_dlci2);
    ASSERT_EQ(g_rfc.dlc->rx_credits, 5);
    uint8_t rsp[64];
    ASSERT_EQ(floss_sim_l2cap_recv(ch, rsp, sizeof rsp), 14); /* addr ctrl len | type len | 8 values | FCS */
    ASSERT_EQ(rsp[3] & 0xFC, RFC_MCC_PN);
    ASSERT_EQ(rsp[12], 0); /* no new initial credits */
}

/* Test commands are echoed at the peer's length; while the link does not
 * drain, each used to pin one pool buffer. */
TEST(rfcomm_test_echo_flood_bounded) {
    floss_sim_chan_t *ch = rfc_setup();
    floss_sim_cfg()->nocp_delay_us = 0x40000000u;
    uint8_t mcc[3 + 300] = {RFC_MCC_TEST | RFC_CR | RFC_EA, (uint8_t)(300 << 1), (uint8_t)(300 >> 7)};
    for (unsigned i = 0; i < 2u * FLOSS_BUF_COUNT; i++) {
        rfc_send(ch, 0, RFC_UIH, mcc, sizeof mcc);
    }
    ASSERT_GE(floss_buf_free_count(), FLOSS_BUF_COUNT - 1);
}
#endif /* FLOSS_ENABLE_RFCOMM */

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
    TEST_CASE(hci_zero_command_credit_recovers),
    TEST_CASE(gap_scan_enable_survives_full_command_queue),
    TEST_CASE(gap_pairing_events_for_strangers_take_no_slot),
    TEST_CASE(hci_unconcluded_incoming_slots_expire),
#if FLOSS_ENABLE_LE
    TEST_CASE(le_duplicate_connection_complete_ignored),
    TEST_CASE(le_short_connection_complete_ignored),
    TEST_CASE(le_link_beyond_hci_table_is_dropped),
    TEST_CASE(att_prepare_write_gaps_hold_no_stale_bytes),
    TEST_CASE(att_prepare_write_beyond_mtu_rejected),
    TEST_CASE(gatt_read_callback_count_clamped),
    TEST_CASE(smp_lost_dhkey_event_does_not_block_pairing),
#endif
    TEST_CASE(acl_tx_queue_bounded_when_peer_stalls),
    TEST_CASE(sdp_deferred_response_not_overwritten),
#if FLOSS_ENABLE_RFCOMM
    TEST_CASE(rfcomm_frame_beyond_mtu_dropped),
    TEST_CASE(rfcomm_frame_without_credit_dropped),
    TEST_CASE(rfcomm_pn_on_open_dlc_keeps_credits),
    TEST_CASE(rfcomm_test_echo_flood_bounded),
#endif
};

FLOSS_TEST_MAIN("l2cap", tests)
