/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * test_hid_boot.c - Keyboard / mouse features: boot protocol (Classic and
 * HID over GATT) and HIDP fragmentation on the control channel.
 *
 * Stories:
 *   - a PC's firmware switches a combo keyboard+mouse to boot protocol: the
 *     boot reports (ID 1 keyboard, ID 2 mouse) flow on interrupt, the
 *     report-protocol ones stop, LEDs and GET_REPORT use the boot format, and
 *     switching back restores report protocol;
 *   - a gamepad (no boot support) refuses boot protocol;
 *   - the host writes a report larger than our control MTU (SET_REPORT +
 *     DATC continuations) and reads one larger than its own (DATA + DATC);
 *   - an LE host subscribes to the boot characteristics and flips Protocol
 *     Mode.
 *
 * HIDP framing used below (header = transaction type << 4 | parameter):
 *   0x70/0x71 SET_PROTOCOL(boot/report)  0x60 GET_PROTOCOL  0xA0 DATA(other)
 *   0xA1 DATA/input  0xA2 DATA/output  0x41 GET_REPORT(input)
 *   0x43 GET_REPORT(feature) -> 0xA3 DATA(feature)  0x52 SET_REPORT(output)
 *   0xB2/0xB3 DATC (continuation of an output / feature transfer)
 *   0x0r HANDSHAKE(result r)
 */
#include "test_common.h"

static floss_hid_config_t g_cfg; /* must outlive the stack */

/** Pop every payload from a channel; true if one equals want. *count = payloads seen. */
static bool drain_find(floss_sim_chan_t *ch, const uint8_t *want, uint16_t want_len, unsigned *count) {
    uint8_t buf[128];
    bool found = false;
    int len;
    unsigned n = 0;
    while ((len = floss_sim_l2cap_recv(ch, buf, sizeof buf)) >= 0) {
        n++;
        if (len == want_len && memcmp(buf, want, want_len) == 0) {
            found = true;
        }
    }
    if (count) {
        *count = n;
    }
    return found;
}

/** Send a control request and return the stack's single answer (length, -1 if none). */
static int ctrl_transact(floss_sim_chan_t *ctrl, const uint8_t *req, uint16_t len, uint8_t *rsp, uint16_t cap) {
    ASSERT_EQ(floss_sim_l2cap_send(ctrl, req, len), FLOSS_OK);
    return floss_sim_l2cap_recv_wait(ctrl, rsp, cap);
}

static void combo_config(void) {
    floss_tc_hid_config(&g_cfg, 0); /* send on change, like a real keyboard */
    g_cfg.subclass = FLOSS_HID_SUBCLASS_COMBO;
    g_cfg.boot_device = true;
}

/* ------------------------------------------------------------------------ */
/* Classic boot protocol                                                    */
/* ------------------------------------------------------------------------ */

TEST(boot_protocol_switch_and_reports) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    combo_config();
    floss_tc_stack_start(t, &g_cfg);
    floss_sim_chan_t *ctrl, *intr;
    (void)floss_tc_hid_session(&ctrl, &intr);
    uint8_t rsp[128];

    /* Every connection starts in report protocol. */
    ASSERT_EQ(floss_hid_protocol(), FLOSS_HID_PROTOCOL_REPORT);
    const uint8_t get_protocol[] = {0x60};
    ASSERT_EQ(ctrl_transact(ctrl, get_protocol, 1, rsp, sizeof rsp), 2);
    ASSERT_EQ(rsp[0], 0xA0);
    ASSERT_EQ(rsp[1], FLOSS_HID_PROTOCOL_REPORT);

    /* Report protocol: the boot slot is kept current but never sent. */
    const uint8_t pad[FLOSS_TC_INPUT_LEN] = {0x01, 0x00, 0x10, 0x20, 0x30, 0x40};
    const uint8_t kb[8] = {FLOSS_HID_MOD_LSHIFT, 0, 0x04, 0, 0, 0, 0, 0}; /* shift + 'a' */
    ASSERT_EQ(floss_hid_set_input(FLOSS_TC_INPUT_ID, pad, sizeof pad), FLOSS_OK);
    ASSERT_EQ(floss_hid_set_boot_input(FLOSS_HID_BOOT_KEYBOARD, kb, sizeof kb), FLOSS_OK);
    floss_sim_pump(20);
    uint8_t want_pad[2 + FLOSS_TC_INPUT_LEN] = {0xA1, FLOSS_TC_INPUT_ID};
    memcpy(want_pad + 2, pad, sizeof pad);
    uint8_t want_kb[2 + 8] = {0xA1, FLOSS_HID_BOOT_KEYBOARD};
    memcpy(want_kb + 2, kb, sizeof kb);
    unsigned seen;
    ASSERT_TRUE(drain_find(intr, want_pad, sizeof want_pad, &seen));
    ASSERT_EQ(seen, 1); /* the boot report did not leave */

    /* Boot keyboard report length is fixed; a boot mouse needs buttons, X, Y. */
    ASSERT_EQ(floss_hid_set_boot_input(FLOSS_HID_BOOT_KEYBOARD, kb, 7), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_hid_set_boot_input(FLOSS_HID_BOOT_MOUSE, kb, 2), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_hid_set_boot_input(3, kb, 8), FLOSS_ERR_INVALID);

    /* --- SET_PROTOCOL(boot) -> HANDSHAKE(successful) ------------------------ */
    const uint8_t set_boot[] = {0x70};
    ASSERT_EQ(ctrl_transact(ctrl, set_boot, 1, rsp, sizeof rsp), 1);
    ASSERT_EQ(rsp[0], FLOSS_HID_HS_SUCCESSFUL);
    ASSERT_EQ(floss_hid_protocol(), FLOSS_HID_PROTOCOL_BOOT);
    ASSERT_EQ(floss_tc_hid.set_protocol, 1);
    ASSERT_EQ(floss_tc_hid.protocol, FLOSS_HID_PROTOCOL_BOOT);
    ASSERT_EQ(ctrl_transact(ctrl, get_protocol, 1, rsp, sizeof rsp), 2);
    ASSERT_EQ(rsp[1], FLOSS_HID_PROTOCOL_BOOT);

    /* The newest boot keyboard state goes out right after the switch:
     * A1 01 <modifiers> <reserved> <6 keys>. */
    floss_sim_pump(20);
    ASSERT_TRUE(drain_find(intr, want_kb, sizeof want_kb, &seen));
    ASSERT_EQ(seen, 1); /* the mouse was never published: nothing for it */

    /* Report-protocol reports are silent now; boot reports flow. */
    const uint8_t pad2[FLOSS_TC_INPUT_LEN] = {0x02, 0x00, 0x11, 0x22, 0x33, 0x44};
    ASSERT_EQ(floss_hid_set_input(FLOSS_TC_INPUT_ID, pad2, sizeof pad2), FLOSS_OK);
    const uint8_t mouse[4] = {FLOSS_HID_MOUSE_LEFT, 0x05, 0xFB, 0x01}; /* left, +5, -5, wheel +1 */
    ASSERT_EQ(floss_hid_set_boot_input(FLOSS_HID_BOOT_MOUSE, mouse, sizeof mouse), FLOSS_OK);
    floss_sim_pump(20);
    const uint8_t want_mouse[] = {0xA1, FLOSS_HID_BOOT_MOUSE, FLOSS_HID_MOUSE_LEFT, 0x05, 0xFB, 0x01};
    ASSERT_TRUE(drain_find(intr, want_mouse, sizeof want_mouse, &seen));
    ASSERT_EQ(seen, 1);

    /* GET_REPORT(input) carries the boot ID and answers in boot format. */
    const uint8_t get_kb[] = {0x41, FLOSS_HID_BOOT_KEYBOARD};
    ASSERT_EQ(ctrl_transact(ctrl, get_kb, sizeof get_kb, rsp, sizeof rsp), (int)sizeof want_kb);
    ASSERT_MEM_EQ(rsp, want_kb, sizeof want_kb);
    const uint8_t get_mouse[] = {0x41, FLOSS_HID_BOOT_MOUSE};
    ASSERT_EQ(ctrl_transact(ctrl, get_mouse, sizeof get_mouse, rsp, sizeof rsp), (int)sizeof want_mouse);
    ASSERT_MEM_EQ(rsp, want_mouse, sizeof want_mouse);
    const uint8_t get_bad[] = {0x41, 0x03};
    ASSERT_EQ(ctrl_transact(ctrl, get_bad, sizeof get_bad, rsp, sizeof rsp), 1);
    ASSERT_EQ(rsp[0], FLOSS_HID_HS_ERR_INVALID_REPORT_ID);

    /* Keyboard LEDs in boot protocol: A2 01 <leds> on interrupt... */
    const uint8_t leds[] = {0xA2, FLOSS_HID_BOOT_KEYBOARD, FLOSS_HID_LED_CAPS_LOCK};
    ASSERT_EQ(floss_sim_l2cap_send(intr, leds, sizeof leds), FLOSS_OK);
    floss_sim_pump(20);
    ASSERT_EQ(floss_tc_hid.output_reports, 1);
    ASSERT_EQ(floss_tc_hid.output_id, FLOSS_HID_BOOT_KEYBOARD);
    ASSERT_EQ(floss_tc_hid.output_len, 1);
    ASSERT_EQ(floss_tc_hid.output[0], FLOSS_HID_LED_CAPS_LOCK);

    /* ...and SET_REPORT(output) on control. */
    const uint8_t set_leds[] = {0x52, FLOSS_HID_BOOT_KEYBOARD, FLOSS_HID_LED_NUM_LOCK};
    ASSERT_EQ(ctrl_transact(ctrl, set_leds, sizeof set_leds, rsp, sizeof rsp), 1);
    ASSERT_EQ(rsp[0], FLOSS_HID_HS_SUCCESSFUL);
    ASSERT_EQ(floss_tc_hid.output_id, FLOSS_HID_BOOT_KEYBOARD);
    ASSERT_EQ(floss_tc_hid.output[0], FLOSS_HID_LED_NUM_LOCK);

    /* --- back to report protocol: the newest gamepad state goes out -------- */
    const uint8_t set_report[] = {0x71};
    ASSERT_EQ(ctrl_transact(ctrl, set_report, 1, rsp, sizeof rsp), 1);
    ASSERT_EQ(rsp[0], FLOSS_HID_HS_SUCCESSFUL);
    ASSERT_EQ(floss_hid_protocol(), FLOSS_HID_PROTOCOL_REPORT);
    floss_sim_pump(20);
    uint8_t want_pad2[2 + FLOSS_TC_INPUT_LEN] = {0xA1, FLOSS_TC_INPUT_ID};
    memcpy(want_pad2 + 2, pad2, sizeof pad2);
    ASSERT_TRUE(drain_find(intr, want_pad2, sizeof want_pad2, &seen));
    ASSERT_EQ(seen, 1);

    /* Boot publishes are silent again. */
    ASSERT_EQ(floss_hid_set_boot_input(FLOSS_HID_BOOT_MOUSE, mouse, 3), FLOSS_OK);
    floss_sim_pump(20);
    ASSERT_EQ(floss_sim_l2cap_pending(intr), 0);
    floss_tc_assert_sim_clean();
}

TEST(boot_protocol_refused_without_boot_support) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 0); /* gamepad: boot_device = false */
    floss_tc_stack_start(t, &g_cfg);
    floss_sim_chan_t *ctrl, *intr;
    (void)floss_tc_hid_session(&ctrl, &intr);
    uint8_t rsp[16];

    const uint8_t set_boot[] = {0x70};
    ASSERT_EQ(ctrl_transact(ctrl, set_boot, 1, rsp, sizeof rsp), 1);
    ASSERT_EQ(rsp[0], FLOSS_HID_HS_ERR_INVALID_PARAMETER);
    ASSERT_EQ(floss_hid_protocol(), FLOSS_HID_PROTOCOL_REPORT);
    ASSERT_EQ(floss_tc_hid.set_protocol, 0);

    const uint8_t kb[8] = {0};
    ASSERT_EQ(floss_hid_set_boot_input(FLOSS_HID_BOOT_KEYBOARD, kb, sizeof kb), FLOSS_ERR_UNSUPPORTED);
    floss_tc_assert_sim_clean();
}

TEST(boot_keyboard_only_device) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 0);
    g_cfg.subclass = FLOSS_HID_SUBCLASS_KEYBOARD;
    g_cfg.boot_device = true;
    floss_tc_stack_start(t, &g_cfg);

    const uint8_t r[4] = {0};
    ASSERT_EQ(floss_hid_set_boot_input(FLOSS_HID_BOOT_MOUSE, r, sizeof r), FLOSS_ERR_UNSUPPORTED);
    const uint8_t kb[8] = {0};
    ASSERT_EQ(floss_hid_set_boot_input(FLOSS_HID_BOOT_KEYBOARD, kb, sizeof kb), FLOSS_OK);
}

/* ------------------------------------------------------------------------ */
/* HIDP fragmentation (DATC)                                                */
/* ------------------------------------------------------------------------ */

TEST(fragmented_set_report_reassembled) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_hid_config(&g_cfg, 0);
    floss_tc_stack_start(t, &g_cfg);
    floss_sim_chan_t *ctrl, *intr;
    uint16_t h = floss_tc_hid_session(&ctrl, &intr);
    uint8_t rsp[16];

    /* The stack always offers a 672-byte MTU, far above any 64-byte report.
     * Shrink what it accepts on control (white box) so a report needs
     * fragments; the sim stays within any MTU >= 32. */
    floss_l2cap_chan_t *sc = floss_l2cap_find(h, FLOSS_PSM_HID_CONTROL);
    ASSERT_NOT_NULL(sc);
    sc->local_mtu = 32;

    uint8_t data[64];
    for (unsigned i = 0; i < sizeof data; i++) {
        data[i] = (uint8_t)(i * 3u + 1u);
    }

    /* 1) ID + 61 bytes = exactly two full packets: a header-only DATC ends it. */
    uint8_t pkt[32];
    pkt[0] = 0x52;
    pkt[1] = FLOSS_TC_OUTPUT_ID;
    memcpy(pkt + 2, data, 30);
    ASSERT_EQ(floss_sim_l2cap_send(ctrl, pkt, 32), FLOSS_OK);
    floss_sim_pump(10);
    ASSERT_EQ(floss_sim_l2cap_pending(ctrl), 0); /* no answer while fragments are owed */
    pkt[0] = 0xB2;
    memcpy(pkt + 1, data + 30, 31);
    ASSERT_EQ(floss_sim_l2cap_send(ctrl, pkt, 32), FLOSS_OK);
    floss_sim_pump(10);
    ASSERT_EQ(floss_sim_l2cap_pending(ctrl), 0);
    ASSERT_EQ(floss_tc_hid.set_reports, 0);
    const uint8_t datc_end[] = {0xB2};
    ASSERT_EQ(ctrl_transact(ctrl, datc_end, 1, rsp, sizeof rsp), 1);
    ASSERT_EQ(rsp[0], FLOSS_HID_HS_SUCCESSFUL);
    ASSERT_EQ(floss_tc_hid.set_reports, 1);
    ASSERT_EQ(floss_tc_hid.output_id, FLOSS_TC_OUTPUT_ID);
    ASSERT_EQ(floss_tc_hid.output_len, 61);
    ASSERT_MEM_EQ(floss_tc_hid.output, data, 61);

    /* 2) A short continuation completes the transfer by itself. */
    pkt[0] = 0x52;
    pkt[1] = FLOSS_TC_OUTPUT_ID;
    memcpy(pkt + 2, data + 1, 30);
    ASSERT_EQ(floss_sim_l2cap_send(ctrl, pkt, 32), FLOSS_OK);
    pkt[0] = 0xB2;
    memcpy(pkt + 1, data + 31, 10);
    ASSERT_EQ(ctrl_transact(ctrl, pkt, 11, rsp, sizeof rsp), 1);
    ASSERT_EQ(rsp[0], FLOSS_HID_HS_SUCCESSFUL);
    ASSERT_EQ(floss_tc_hid.set_reports, 2);
    ASSERT_EQ(floss_tc_hid.output_len, 40);
    ASSERT_MEM_EQ(floss_tc_hid.output, data + 1, 40);

    /* 3) More than report ID + FLOSS_HID_MAX_REPORT: refused once complete. */
    pkt[0] = 0x52;
    pkt[1] = FLOSS_TC_OUTPUT_ID;
    memcpy(pkt + 2, data, 30);
    ASSERT_EQ(floss_sim_l2cap_send(ctrl, pkt, 32), FLOSS_OK);
    pkt[0] = 0xB2;
    memcpy(pkt + 1, data, 31);
    ASSERT_EQ(floss_sim_l2cap_send(ctrl, pkt, 32), FLOSS_OK);
    ASSERT_EQ(floss_sim_l2cap_send(ctrl, pkt, 32), FLOSS_OK);
    floss_sim_pump(10);
    ASSERT_EQ(floss_sim_l2cap_pending(ctrl), 0);
    ASSERT_EQ(ctrl_transact(ctrl, pkt, 5, rsp, sizeof rsp), 1);
    ASSERT_EQ(rsp[0], FLOSS_HID_HS_ERR_INVALID_PARAMETER);
    ASSERT_EQ(floss_tc_hid.set_reports, 2); /* the application never saw it */

    /* 4) An unfragmented report larger than FLOSS_HID_MAX_REPORT is refused too. */
    sc->local_mtu = FLOSS_L2CAP_MTU;
    uint8_t big[2 + FLOSS_HID_MAX_REPORT + 1] = {0x52, FLOSS_TC_OUTPUT_ID};
    ASSERT_EQ(ctrl_transact(ctrl, big, sizeof big, rsp, sizeof rsp), 1);
    ASSERT_EQ(rsp[0], FLOSS_HID_HS_ERR_INVALID_PARAMETER);

    /* The channel still works normally afterwards. */
    const uint8_t small[] = {0x52, FLOSS_TC_OUTPUT_ID, 0x11, 0x22};
    ASSERT_EQ(ctrl_transact(ctrl, small, sizeof small, rsp, sizeof rsp), 1);
    ASSERT_EQ(rsp[0], FLOSS_HID_HS_SUCCESSFUL);
    ASSERT_EQ(floss_tc_hid.output_len, 2);
    floss_tc_assert_sim_clean();
}

TEST(large_get_report_fragmented) {
    floss_sim_config_t scfg;
    floss_tc_sim_config(&scfg);
    scfg.l2cap_mtu = 48; /* the host accepts at most 48 bytes per packet */
    floss_transport_t *t = floss_tc_sim_init(&scfg);
    floss_tc_hid_config(&g_cfg, 0);
    floss_tc_stack_start(t, &g_cfg);
    floss_sim_chan_t *ctrl, *intr;
    uint16_t h = floss_tc_hid_session(&ctrl, &intr);
    ASSERT_EQ(floss_l2cap_find(h, FLOSS_PSM_HID_CONTROL)->remote_mtu, 48);
    uint8_t rsp[128];

    for (unsigned i = 0; i < sizeof floss_tc_hid.feature; i++) {
        floss_tc_hid.feature[i] = (uint8_t)(0x80u + i);
    }
    const uint8_t get_feature[] = {0x43, 0x03};

    /* 64-byte feature: DATA [A3 03 + 46 bytes] (48) then DATC [B3 + 18 bytes]. */
    floss_tc_hid.feature_len = 64;
    ASSERT_EQ(ctrl_transact(ctrl, get_feature, sizeof get_feature, rsp, sizeof rsp), 48);
    ASSERT_EQ(rsp[0], 0xA3); /* DATA(feature) */
    ASSERT_EQ(rsp[1], 0x03);
    ASSERT_MEM_EQ(rsp + 2, floss_tc_hid.feature, 46);
    ASSERT_EQ(floss_sim_l2cap_recv_wait(ctrl, rsp, sizeof rsp), 19);
    ASSERT_EQ(rsp[0], 0xB3);
    ASSERT_MEM_EQ(rsp + 1, floss_tc_hid.feature + 46, 18);
    floss_sim_pump(10);
    ASSERT_EQ(floss_sim_l2cap_pending(ctrl), 0);

    /* Exactly one full packet: a header-only DATC tells the host it is complete. */
    floss_tc_hid.feature_len = 46;
    ASSERT_EQ(ctrl_transact(ctrl, get_feature, sizeof get_feature, rsp, sizeof rsp), 48);
    ASSERT_MEM_EQ(rsp + 2, floss_tc_hid.feature, 46);
    ASSERT_EQ(floss_sim_l2cap_recv_wait(ctrl, rsp, sizeof rsp), 1);
    ASSERT_EQ(rsp[0], 0xB3);
    floss_sim_pump(10);
    ASSERT_EQ(floss_sim_l2cap_pending(ctrl), 0);

    /* BufferSize (param bit 3) still truncates: 20 bytes = ID + 19 report bytes. */
    floss_tc_hid.feature_len = 64;
    const uint8_t get_sized[] = {0x4B, 0x03, 20, 0};
    ASSERT_EQ(ctrl_transact(ctrl, get_sized, sizeof get_sized, rsp, sizeof rsp), 21);
    ASSERT_EQ(rsp[0], 0xA3); /* DATA(feature) */
    ASSERT_MEM_EQ(rsp + 2, floss_tc_hid.feature, 19);
    floss_sim_pump(10);
    ASSERT_EQ(floss_sim_l2cap_pending(ctrl), 0);
    floss_tc_assert_sim_clean(); /* nothing ever exceeded the host's MTU */
}

/* ------------------------------------------------------------------------ */
/* HID over GATT boot protocol                                              */
/* ------------------------------------------------------------------------ */

#if FLOSS_ENABLE_LE
static floss_hogp_config_t g_hogp;
static const floss_hogp_report_t g_hogp_reports[] = {
    {.id = FLOSS_TC_INPUT_ID, .type = FLOSS_HID_REPORT_INPUT, .len = FLOSS_TC_INPUT_LEN},
    {.id = FLOSS_TC_OUTPUT_ID, .type = FLOSS_HID_REPORT_OUTPUT, .len = FLOSS_TC_OUTPUT_LEN},
};

static struct {
    unsigned outputs;
    uint8_t output_id;
    uint8_t output[8];
    uint16_t output_len;
    unsigned set_protocol;
    uint8_t protocol;
} g_le;

static void le_on_output(uint8_t report_id, const uint8_t *data, uint16_t len) {
    g_le.outputs++;
    g_le.output_id = report_id;
    g_le.output_len = len;
    memcpy(g_le.output, data, FLOSS_MIN(len, sizeof g_le.output));
}

static void le_on_set_protocol(uint8_t protocol) {
    g_le.set_protocol++;
    g_le.protocol = protocol;
}

static const floss_hogp_callbacks_t g_le_cb = {
    .on_output_report = le_on_output,
    .on_set_protocol = le_on_set_protocol,
};

static bool cond_hci_ready(void *arg) {
    (void)arg;
    return floss_hci_ready();
}

/** ATT request -> response (opcode checked by the caller). */
static int att_transact(uint16_t h, const uint8_t *req, uint16_t len, uint8_t *rsp, uint16_t cap) {
    uint8_t junk[64];
    while (floss_sim_att_recv(h, junk, sizeof junk) >= 0) {
    }
    ASSERT_EQ(floss_sim_att_send(h, req, len), FLOSS_OK);
    floss_sim_pump(10);
    int n;
    while ((n = floss_sim_att_recv(h, rsp, cap)) >= 0) {
        if (rsp[0] != 0x1B) { /* skip notifications */
            return n;
        }
    }
    return -1;
}

static void att_write(uint16_t h, uint16_t handle, const uint8_t *val, uint16_t len) {
    uint8_t req[32] = {0x12};
    floss_put_le16(req + 1, handle);
    memcpy(req + 3, val, len);
    uint8_t rsp[32];
    ASSERT_EQ(att_transact(h, req, (uint16_t)(3 + len), rsp, sizeof rsp), 1);
    ASSERT_EQ(rsp[0], 0x13); /* Write Response */
}

/** Drain ATT; true if a notification for `handle` with exactly `val` arrived. *count = notifications seen. */
static bool att_find_notify(uint16_t h, uint16_t handle, const uint8_t *val, uint16_t len, unsigned *count) {
    uint8_t buf[64];
    int n;
    bool found = false;
    unsigned c = 0;
    while ((n = floss_sim_att_recv(h, buf, sizeof buf)) >= 0) {
        if (n >= 3 && buf[0] == 0x1B) {
            c++;
            if (floss_get_le16(buf + 1) == handle && n == 3 + len && memcmp(buf + 3, val, len) == 0) {
                found = true;
            }
        }
    }
    if (count) {
        *count = c;
    }
    return found;
}

TEST(hogp_boot_keyboard_mouse) {
    floss_transport_t *t = floss_tc_sim_init(NULL);
    memset(&g_le, 0, sizeof g_le);
    floss_hogp_config_defaults(&g_hogp);
    g_hogp.report_map = floss_tc_report_desc;
    g_hogp.report_map_len = floss_tc_report_desc_len;
    g_hogp.reports = g_hogp_reports;
    g_hogp.report_count = 2;
    g_hogp.boot_keyboard = true;
    g_hogp.boot_mouse = true;

    floss_init_t init = {.transport = t, .chipset = NULL};
    ASSERT_EQ(floss_init(&init), FLOSS_OK);
    ASSERT_EQ(floss_hogp_init(&g_hogp, &g_le_cb), FLOSS_OK);
    ASSERT_EQ(floss_start(), FLOSS_OK);
    ASSERT_TRUE(floss_sim_pump_until(cond_hci_ready, NULL, 5000));
    floss_sim_pump(20);

    /* The static table has the boot characteristics next to the Reports. */
    uint16_t proto = floss_gatt_find(1, 0xFFFF, FLOSS_GATT_CHR_PROTOCOL_MODE);
    uint16_t report_in = floss_gatt_find(1, 0xFFFF, FLOSS_GATT_CHR_REPORT); /* first Report = input ID 1 */
    uint16_t kb_in = floss_gatt_find(1, 0xFFFF, FLOSS_GATT_CHR_BOOT_KB_INPUT);
    uint16_t kb_out = floss_gatt_find(1, 0xFFFF, FLOSS_GATT_CHR_BOOT_KB_OUTPUT);
    uint16_t mouse_in = floss_gatt_find(1, 0xFFFF, FLOSS_GATT_CHR_BOOT_MOUSE_INPUT);
    ASSERT_NE(proto, 0);
    ASSERT_NE(report_in, 0);
    ASSERT_NE(kb_in, 0);
    ASSERT_NE(kb_out, 0);
    ASSERT_NE(mouse_in, 0);
    ASSERT_EQ(floss_gatt_attr(kb_in - 1)->aux, FLOSS_GATT_PROP_READ | FLOSS_GATT_PROP_NOTIFY);
    ASSERT_EQ(floss_gatt_attr(kb_in + 1)->type, FLOSS_GATT_UUID_CCCD);
    ASSERT_EQ(floss_gatt_attr(mouse_in + 1)->type, FLOSS_GATT_UUID_CCCD);
    ASSERT_EQ(floss_gatt_attr(kb_out - 1)->aux,
              FLOSS_GATT_PROP_READ | FLOSS_GATT_PROP_WRITE | FLOSS_GATT_PROP_WRITE_NO_RSP);

    /* An encrypted LE host subscribes to all three inputs. */
    uint16_t h = floss_sim_le_connect(&floss_tc_host, 0, 1);
    ASSERT_NE(h, FLOSS_SIM_NO_HANDLE);
    floss_sim_pump(10);
    const uint8_t enc[] = {0x00, (uint8_t)h, (uint8_t)(h >> 8), 0x01};
    ASSERT_EQ(floss_sim_inject_event(FLOSS_EVT_ENCRYPTION_CHANGE, enc, sizeof enc), FLOSS_OK);
    floss_sim_pump(10);
    ASSERT_GE(floss_le_link(h)->security_level, 2);
    const uint8_t notify_on[] = {0x01, 0x00};
    att_write(h, (uint16_t)(report_in + 1), notify_on, 2);
    att_write(h, (uint16_t)(kb_in + 1), notify_on, 2);
    att_write(h, (uint16_t)(mouse_in + 1), notify_on, 2);
    floss_sim_pump(10);
    ASSERT_TRUE(floss_hogp_connected());
    ASSERT_EQ(floss_hogp_protocol(), FLOSS_HID_PROTOCOL_REPORT);
    (void)att_find_notify(h, 0, NULL, 0, NULL); /* initial state of the Report */

    /* Report protocol: boot publishes stay silent. */
    const uint8_t kb[8] = {0, 0, 0x05, 0, 0, 0, 0, 0}; /* 'b' */
    ASSERT_EQ(floss_hogp_set_boot_input(FLOSS_HID_BOOT_KEYBOARD, kb, sizeof kb), FLOSS_OK);
    const uint8_t pad[FLOSS_TC_INPUT_LEN] = {1, 2, 3, 4, 5, 6};
    ASSERT_EQ(floss_hogp_set_input(FLOSS_TC_INPUT_ID, pad, sizeof pad), FLOSS_OK);
    floss_sim_pump(10);
    unsigned seen;
    ASSERT_TRUE(att_find_notify(h, report_in, pad, sizeof pad, &seen));
    ASSERT_EQ(seen, 1);

    /* Protocol Mode = boot (Write Command): boot inputs take over. */
    uint8_t cmd[4] = {0x52};
    floss_put_le16(cmd + 1, proto);
    cmd[3] = FLOSS_HID_PROTOCOL_BOOT;
    ASSERT_EQ(floss_sim_att_send(h, cmd, sizeof cmd), FLOSS_OK);
    floss_sim_pump(10);
    ASSERT_EQ(floss_hogp_protocol(), FLOSS_HID_PROTOCOL_BOOT);
    ASSERT_EQ(g_le.set_protocol, 1);
    ASSERT_EQ(g_le.protocol, FLOSS_HID_PROTOCOL_BOOT);
    ASSERT_TRUE(att_find_notify(h, kb_in, kb, sizeof kb, &seen)); /* newest state, once */

    ASSERT_EQ(floss_hogp_set_input(FLOSS_TC_INPUT_ID, kb, FLOSS_TC_INPUT_LEN), FLOSS_OK);
    const uint8_t mouse[4] = {FLOSS_HID_MOUSE_RIGHT, 0x10, 0xF0, 0};
    ASSERT_EQ(floss_hogp_set_boot_input(FLOSS_HID_BOOT_MOUSE, mouse, sizeof mouse), FLOSS_OK);
    floss_sim_pump(10);
    ASSERT_TRUE(att_find_notify(h, mouse_in, mouse, sizeof mouse, &seen));
    ASSERT_EQ(seen, 1); /* the Report characteristic stayed silent */
    ASSERT_EQ(floss_hogp_set_boot_input(FLOSS_HID_BOOT_MOUSE, mouse, 2), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_hogp_set_boot_input(FLOSS_HID_BOOT_KEYBOARD, kb, 9), FLOSS_ERR_INVALID);

    /* Boot keyboard output (LEDs) -> on_output_report(ID 1). */
    const uint8_t leds = FLOSS_HID_LED_CAPS_LOCK;
    att_write(h, kb_out, &leds, 1);
    ASSERT_EQ(g_le.outputs, 1);
    ASSERT_EQ(g_le.output_id, FLOSS_HID_BOOT_KEYBOARD);
    ASSERT_EQ(g_le.output_len, 1);
    ASSERT_EQ(g_le.output[0], FLOSS_HID_LED_CAPS_LOCK);

    /* Reading the boot keyboard input returns the boot-format state. */
    uint8_t rd[3] = {0x0A};
    floss_put_le16(rd + 1, kb_in);
    uint8_t rsp[32];
    ASSERT_EQ(att_transact(h, rd, sizeof rd, rsp, sizeof rsp), 1 + 8);
    ASSERT_EQ(rsp[0], 0x0B);
    ASSERT_MEM_EQ(rsp + 1, kb, 8);

    /* A new connection starts in report protocol again. */
    ASSERT_EQ(floss_sim_remote_disconnect(h, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED), FLOSS_OK);
    floss_sim_pump(20);
    ASSERT_EQ(floss_hogp_protocol(), FLOSS_HID_PROTOCOL_REPORT);
    floss_tc_assert_sim_clean();
}
#endif

static const floss_test_case_t tests[] = {
    TEST_CASE(boot_protocol_switch_and_reports),
    TEST_CASE(boot_protocol_refused_without_boot_support),
    TEST_CASE(boot_keyboard_only_device),
    TEST_CASE(fragmented_set_report_reassembled),
    TEST_CASE(large_get_report_fragmented),
#if FLOSS_ENABLE_LE
    TEST_CASE(hogp_boot_keyboard_mouse),
#endif
};

FLOSS_TEST_MAIN("hid_boot", tests)
