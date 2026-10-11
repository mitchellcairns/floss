/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * test_common.c - Shared fixtures for the integration suites (stack + sim).
 *
 * Holds the test gamepad (descriptor + config), recorders for HID callbacks
 * and GAP events, and the scripted opening moves of a host session. Every
 * helper bounds its waiting (pump budget) and fails with a message naming
 * what never happened, so a broken layer shows up as "HID never reached
 * CONNECTED" rather than a hang.
 */
#include "test_common.h"

#include <stdio.h>

const floss_addr_t floss_tc_host = {{0x01, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA}}; /* AA:BB:CC:DD:EE:01 */

const uint8_t floss_tc_report_desc[] = {
    0x05, 0x01,       /* Usage Page (Generic Desktop) */
    0x09, 0x05,       /* Usage (Gamepad) */
    0xA1, 0x01,       /* Collection (Application) */
    0x85, 0x01,       /*   Report ID (1) */
    0x05, 0x09,       /*   Usage Page (Button) */
    0x19, 0x01,       /*   Usage Minimum (1) */
    0x29, 0x10,       /*   Usage Maximum (16) */
    0x15, 0x00,       /*   Logical Minimum (0) */
    0x25, 0x01,       /*   Logical Maximum (1) */
    0x75, 0x01,       /*   Report Size (1) */
    0x95, 0x10,       /*   Report Count (16) */
    0x81, 0x02,       /*   Input (Data, Var, Abs) */
    0x05, 0x01,       /*   Usage Page (Generic Desktop) */
    0x09, 0x30,       /*   Usage (X) */
    0x09, 0x31,       /*   Usage (Y) */
    0x09, 0x32,       /*   Usage (Z) */
    0x09, 0x35,       /*   Usage (Rz) */
    0x15, 0x00,       /*   Logical Minimum (0) */
    0x26, 0xFF, 0x00, /*   Logical Maximum (255) */
    0x75, 0x08,       /*   Report Size (8) */
    0x95, 0x04,       /*   Report Count (4) */
    0x81, 0x02,       /*   Input (Data, Var, Abs) */
    0x85, 0x02,       /*   Report ID (2) */
    0x06, 0x00, 0xFF, /*   Usage Page (Vendor) */
    0x09, 0x01,       /*   Usage (1): rumble */
    0x15, 0x00,       /*   Logical Minimum (0) */
    0x26, 0xFF, 0x00, /*   Logical Maximum (255) */
    0x75, 0x08,       /*   Report Size (8) */
    0x95, 0x02,       /*   Report Count (2) */
    0x91, 0x02,       /*   Output (Data, Var, Abs) */
    0xC0,             /* End Collection */
};
const uint16_t floss_tc_report_desc_len = (uint16_t)sizeof floss_tc_report_desc;

floss_tc_hid_log_t floss_tc_hid;
floss_tc_gap_log_t floss_tc_gap;

/* ------------------------------------------------------------------------ */
/* Recorders                                                                */
/* ------------------------------------------------------------------------ */

static void tc_on_connected(const floss_addr_t *host) {
    floss_tc_hid.connected++;
    floss_tc_hid.host = *host;
}

static void tc_on_disconnected(uint8_t reason) {
    floss_tc_hid.disconnected++;
    floss_tc_hid.disconnect_reason = reason;
}

static void tc_on_output_report(uint8_t report_id, const uint8_t *data, uint16_t len) {
    floss_tc_hid.output_reports++;
    floss_tc_hid.output_id = report_id;
    floss_tc_hid.output_len = len;
    memcpy(floss_tc_hid.output, data, FLOSS_MIN(len, sizeof floss_tc_hid.output));
}

static uint8_t tc_on_set_report(uint8_t type, uint8_t report_id, const uint8_t *data, uint16_t len) {
    floss_tc_hid.set_reports++;
    if (type == FLOSS_HID_REPORT_OUTPUT) {
        tc_on_output_report(report_id, data, len);
    }
    return FLOSS_HID_HS_SUCCESSFUL;
}

static void tc_on_set_protocol(uint8_t protocol) {
    floss_tc_hid.set_protocol++;
    floss_tc_hid.protocol = protocol;
}

static void tc_on_control(uint8_t op) {
    floss_tc_hid.control_ops++;
    floss_tc_hid.control_op = op;
}

static void tc_on_report_tick(void) {
    floss_tc_hid.report_ticks++;
}

const floss_hid_callbacks_t floss_tc_hid_callbacks = {
    .on_connected = tc_on_connected,
    .on_disconnected = tc_on_disconnected,
    .on_output_report = tc_on_output_report,
    .on_get_report = NULL, /* input reports answered from the mailbox */
    .on_set_report = tc_on_set_report,
    .on_set_protocol = tc_on_set_protocol,
    .on_control = tc_on_control,
    .on_report_tick = tc_on_report_tick,
};

static void tc_gap_listener(const floss_gap_evt_t *evt, void *user) {
    (void)user;
    if (floss_tc_gap.count < FLOSS_TC_GAP_LOG) {
        floss_tc_gap.evt[floss_tc_gap.count] = *evt;
    }
    floss_tc_gap.count++;
}

const floss_gap_evt_t *floss_tc_gap_last(floss_gap_evt_type_t type) {
    unsigned n = FLOSS_MIN(floss_tc_gap.count, (unsigned)FLOSS_TC_GAP_LOG);
    for (unsigned i = n; i-- > 0;) {
        if (floss_tc_gap.evt[i].type == type) {
            return &floss_tc_gap.evt[i];
        }
    }
    return NULL;
}

unsigned floss_tc_gap_count(floss_gap_evt_type_t type) {
    unsigned n = FLOSS_MIN(floss_tc_gap.count, (unsigned)FLOSS_TC_GAP_LOG);
    unsigned c = 0;
    for (unsigned i = 0; i < n; i++) {
        c += floss_tc_gap.evt[i].type == type;
    }
    return c;
}

/* ------------------------------------------------------------------------ */
/* Setup                                                                    */
/* ------------------------------------------------------------------------ */

void floss_tc_sim_config(floss_sim_config_t *cfg) {
    floss_sim_config_defaults(cfg);
    cfg->advance_time = floss_test_time_advance;
}

floss_transport_t *floss_tc_sim_init(const floss_sim_config_t *cfg) {
    floss_sim_config_t def;
    if (!cfg) {
        floss_tc_sim_config(&def);
        cfg = &def;
    }
    return floss_sim_init(cfg);
}

void floss_tc_hid_config(floss_hid_config_t *cfg, uint32_t report_interval_us) {
    floss_hid_config_defaults(cfg);
    cfg->name = "FLOSS Test Pad";
    cfg->report_descriptor = floss_tc_report_desc;
    cfg->report_descriptor_len = floss_tc_report_desc_len;
    cfg->subclass = FLOSS_HID_SUBCLASS_GAMEPAD;
    cfg->class_of_device = 0; /* derive: gamepad 0x002508 */
    cfg->vendor_id_source = 2; /* USB-IF namespace */
    cfg->vendor_id = 0x1209;   /* pid.codes (open-source hardware) */
    cfg->product_id = 0x0001;
    cfg->version = 0x0100;
    cfg->boot_device = false;
    cfg->report_interval_us = report_interval_us;
    cfg->auto_discoverable = true;
    cfg->auto_reconnect = true;
    cfg->sniff_max_interval = 0;
}

static bool cond_ready(void *arg) {
    (void)arg;
    return floss_tc_gap_last(FLOSS_GAP_EVT_READY) != NULL;
}

void floss_tc_stack_start(floss_transport_t *t, const floss_hid_config_t *hid) {
    memset(&floss_tc_hid, 0, sizeof floss_tc_hid);
    memset(&floss_tc_gap, 0, sizeof floss_tc_gap);

    floss_init_t init = {.transport = t, .chipset = NULL};
    ASSERT_EQ(floss_init(&init), FLOSS_OK);
    ASSERT_EQ(floss_gap_add_listener(tc_gap_listener, NULL), FLOSS_OK);
    if (hid) {
        ASSERT_EQ(floss_hid_init(hid, &floss_tc_hid_callbacks), FLOSS_OK);
    } else {
        ASSERT_EQ(floss_gap_set_scan(true, true), FLOSS_OK);
    }
    ASSERT_EQ(floss_start(), FLOSS_OK);
    if (!floss_sim_pump_until(cond_ready, NULL, 5000)) {
        uint32_t logged = FLOSS_MIN(floss_sim_cmd_total(), (uint32_t)FLOSS_SIM_CMD_LOG_LEN);
        const floss_sim_cmd_t *last = logged ? floss_sim_cmd_at(logged - 1) : NULL;
        FAIL("stack never reported READY (%u commands sent, last opcode 0x%04x)", (unsigned)floss_sim_cmd_total(),
             last ? (unsigned)last->opcode : 0u);
    }
    /* Let GAP finish applying settings that follow READY (scan enable, EIR...). */
    floss_sim_pump(20);
}

static bool cond_link_open(void *arg) {
    const floss_sim_link_t *l = floss_sim_link((uint16_t)(uintptr_t)arg);
    return l && l->state == FLOSS_SIM_LINK_OPEN;
}

uint16_t floss_tc_host_connect(void) {
    uint16_t h = floss_sim_remote_connect(&floss_tc_host, FLOSS_TC_HOST_COD);
    if (h == FLOSS_SIM_NO_HANDLE) {
        FAIL("host could not page the stack (scan enable 0x%02x)", (unsigned)floss_sim_scan_enable());
    }
    if (!floss_sim_pump_until(cond_link_open, (void *)(uintptr_t)h, 500)) {
        FAIL("stack never accepted the connection request (Accept Connection Request sent %u times)",
             (unsigned)floss_sim_cmd_count(FLOSS_HCI_ACCEPT_CONNECTION_REQUEST));
    }
    floss_sim_pump(10);
    ASSERT_NOT_NULL(floss_hci_conn_by_handle(h));
    return h;
}

static bool cond_link_encrypted(void *arg) {
    const floss_sim_link_t *l = floss_sim_link((uint16_t)(uintptr_t)arg);
    return l && l->authenticated && l->encrypted;
}

void floss_tc_host_pair(uint16_t handle) {
    ASSERT_EQ(floss_sim_remote_pair(handle), FLOSS_OK);
    if (!floss_sim_pump_until(cond_link_encrypted, (void *)(uintptr_t)handle, 1000)) {
        FAIL("pairing did not complete (IO cap replies %u, confirm replies %u, link key neg replies %u)",
             (unsigned)floss_sim_cmd_count(FLOSS_HCI_IO_CAPABILITY_REQUEST_REPLY),
             (unsigned)floss_sim_cmd_count(FLOSS_HCI_USER_CONFIRM_REQUEST_REPLY),
             (unsigned)floss_sim_cmd_count(FLOSS_HCI_LINK_KEY_REQUEST_NEG_REPLY));
    }
    floss_sim_pump(10);
}

static bool cond_hid_connected(void *arg) {
    (void)arg;
    return floss_hid_connected();
}

bool floss_tc_wait_hid_connected(uint32_t max_polls) {
    return floss_sim_pump_until(cond_hid_connected, NULL, max_polls);
}

void floss_tc_hid_open(uint16_t handle, floss_sim_chan_t **ctrl, floss_sim_chan_t **intr) {
    floss_sim_chan_t *c = floss_sim_l2cap_connect(handle, FLOSS_PSM_HID_CONTROL);
    ASSERT_NOT_NULL(c);
    if (c->state != FLOSS_SIM_CHAN_OPEN) {
        FAIL("HID control channel not open (state %d, result 0x%04x)", (int)c->state, (unsigned)c->result);
    }
    floss_sim_chan_t *i = floss_sim_l2cap_connect(handle, FLOSS_PSM_HID_INTERRUPT);
    ASSERT_NOT_NULL(i);
    if (i->state != FLOSS_SIM_CHAN_OPEN) {
        FAIL("HID interrupt channel not open (state %d, result 0x%04x)", (int)i->state, (unsigned)i->result);
    }
    if (!floss_tc_wait_hid_connected(500)) {
        FAIL("HID never reached CONNECTED (state %d)", (int)floss_hid_state());
    }
    *ctrl = c;
    *intr = i;
}

uint16_t floss_tc_hid_session(floss_sim_chan_t **ctrl, floss_sim_chan_t **intr) {
    uint16_t h = floss_tc_host_connect();
    floss_tc_host_pair(h);
    floss_tc_hid_open(h, ctrl, intr);
    return h;
}

void floss_tc_assert_sim_clean(void) {
    const floss_sim_stats_t *st = floss_sim_stats();
    if (floss_sim_violations() != 0) {
        FAIL("sim saw protocol violations: cmd_credit %u, acl_credit %u, acl_size %u, acl_bad_handle %u, "
             "acl_pb %u, l2cap_mtu %u, l2cap_unknown_cid %u, malformed %u",
             (unsigned)st->cmd_credit_violations, (unsigned)st->acl_credit_violations,
             (unsigned)st->acl_size_violations, (unsigned)st->acl_bad_handle, (unsigned)st->acl_pb_violations,
             (unsigned)st->l2cap_mtu_violations, (unsigned)st->l2cap_unknown_cid, (unsigned)st->malformed);
    }
    ASSERT_EQ(st->queue_overflows, 0);
    ASSERT_EQ(floss_test_critical_depth(), 0);
}
