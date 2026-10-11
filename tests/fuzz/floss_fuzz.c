/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_fuzz.c - Deterministic, dependency-free fuzzer for every parser a
 * remote peer (or a misbehaving controller) can reach.
 *
 * Overview
 * --------
 * Each target is one test case of the tiny test framework. A target runs
 * ROUNDS; every round boots a fresh stack on the simulator, scripts it into
 * an interesting state (encrypted Classic link with the HID channels, SDP and
 * an RFCOMM session open; an LE link with the GATT database and SMP), then
 * feeds it a burst of hostile packets:
 *
 *   hci      HCI events: every code the stack handles, structured templates
 *            with real/stale/bogus handles and addresses, truncated and
 *            oversized parameter lengths
 *   acl      raw ACL: bad packet boundary flags, lengths, fragment storms,
 *            valid PDUs split at random points
 *   l2cap    Classic signaling (several commands per C-frame, every code,
 *            config options) and LE signaling
 *   sdp      SDP PDUs: random data element trees, continuation states
 *            (captured from real answers, then mutated), huge counts
 *   att      every ATT opcode, bad handles/offsets/lengths, prepare queues
 *   smp      SMP PDUs in and out of order, wrong lengths, controller events
 *   rfcomm   frames with bad FCS/length/EA bits, every MCC command, credits
 *   hidp     HIDP transactions on the control and interrupt channels
 *   h4       the H4 transport's framing, fed garbage byte streams
 *
 * Packets are either generated from a template and then mutated (bit flips,
 * interesting 16-bit values, truncation, extension) or plain random bytes.
 * After each round every link is torn down and the stack must be back to
 * its idle invariants: no connection slot, L2CAP channel, RFCOMM session or
 * DLC in use and every pool buffer returned. Crashes and memory errors are
 * caught by AddressSanitizer / UBSan (the CMake target enables them).
 *
 * Usage:
 *   floss_fuzz [--seed N] [--iters N] [--verbose] [target-substring]
 *
 * --iters is the number of hostile packets per target (default 20000). The
 * run is fully deterministic: the same seed and iteration count replay the
 * same packets, so a failure reproduces with the seed it prints.
 */
#include "test_common.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "floss_internal.h"
#include "hci/floss_hci_priv.h"
#include "l2cap/floss_l2cap_priv.h"
#include "floss/floss_gatt.h"
#include "floss/floss_hogp.h"
#include "floss/floss_le.h"
#include "floss/floss_rfcomm.h"
#include "floss_h4.h"
#include "rfcomm/floss_rfcomm_int.h"

/* ======================================================================== */
/* Options and PRNG                                                         */
/* ======================================================================== */

static uint32_t g_seed = 1;
static uint32_t g_iters = 20000;
static bool g_verbose;

static uint64_t g_rng;

static void rng_seed(const char *target) {
    /* Each target gets its own stream so filtering does not change packets. */
    uint64_t h = 1469598103934665603ull ^ g_seed;
    for (const char *c = target; *c; c++) {
        h = (h ^ (uint8_t)*c) * 1099511628211ull;
    }
    g_rng = h ? h : 1;
}

static uint32_t rnd(void) {
    /* xorshift64* */
    g_rng ^= g_rng >> 12;
    g_rng ^= g_rng << 25;
    g_rng ^= g_rng >> 27;
    return (uint32_t)((g_rng * 2685821657736338717ull) >> 32);
}

static uint32_t below(uint32_t n) {
    return n ? rnd() % n : 0;
}

static bool chance(unsigned pct) {
    return below(100) < pct;
}

static void rnd_bytes(uint8_t *p, uint16_t n) {
    for (uint16_t i = 0; i < n; i++) {
        p[i] = (uint8_t)rnd();
    }
}

static uint16_t interesting16(void) {
    static const uint16_t v[] = {0,      1,      2,      3,      4,      5,      7,      8,      0x0F,
                                 0x10,   0x17,   0x18,   0x30,   0x40,   0x41,   0x7F,   0x80,   0xFF,
                                 0x100,  0x200,  0x2A0,  0x2A1,  0x3FF,  0x400,  0x7FFF, 0x8000, 0xFFFE,
                                 0xFFFF, 0x0FFF, 0x1000, 0x2000, 0xEFFF};
    return chance(80) ? v[below(sizeof v / sizeof v[0])] : (uint16_t)rnd();
}

/* ======================================================================== */
/* Packet buffer + mutation                                                 */
/* ======================================================================== */

#define PKT_MAX 1100

typedef struct {
    uint8_t b[PKT_MAX];
    uint16_t n;
} pkt_t;

static void p_reset(pkt_t *p) {
    p->n = 0;
}

static void p_u8(pkt_t *p, uint8_t v) {
    if (p->n < PKT_MAX) {
        p->b[p->n++] = v;
    }
}

static void p_le16(pkt_t *p, uint16_t v) {
    p_u8(p, (uint8_t)v);
    p_u8(p, (uint8_t)(v >> 8));
}

static void p_be16(pkt_t *p, uint16_t v) {
    p_u8(p, (uint8_t)(v >> 8));
    p_u8(p, (uint8_t)v);
}

static void p_rand(pkt_t *p, uint16_t n) {
    while (n--) {
        p_u8(p, (uint8_t)rnd());
    }
}

static void p_bytes(pkt_t *p, const uint8_t *d, uint16_t n) {
    for (uint16_t i = 0; i < n; i++) {
        p_u8(p, d[i]);
    }
}

/** Apply 1..4 random mutations, keeping the packet within cap bytes. */
static void mutate(pkt_t *p, uint16_t cap) {
    if (cap > PKT_MAX) {
        cap = PKT_MAX;
    }
    unsigned ops = 1 + below(4);
    while (ops--) {
        switch (below(9)) {
        case 0: /* bit flip */
            if (p->n) {
                p->b[below(p->n)] ^= (uint8_t)(1u << below(8));
            }
            break;
        case 1: /* random byte */
            if (p->n) {
                p->b[below(p->n)] = (uint8_t)rnd();
            }
            break;
        case 2: /* interesting byte */
            if (p->n) {
                static const uint8_t v[] = {0x00, 0x01, 0x02, 0x7F, 0x80, 0xFE, 0xFF, 0x40, 0x03};
                p->b[below(p->n)] = v[below(sizeof v)];
            }
            break;
        case 3: /* interesting 16-bit little endian */
            if (p->n >= 2) {
                uint16_t at = (uint16_t)below((uint32_t)p->n - 1);
                uint16_t v = interesting16();
                p->b[at] = (uint8_t)v;
                p->b[at + 1] = (uint8_t)(v >> 8);
            }
            break;
        case 4: /* interesting 16-bit big endian (SDP) */
            if (p->n >= 2) {
                uint16_t at = (uint16_t)below((uint32_t)p->n - 1);
                uint16_t v = interesting16();
                p->b[at] = (uint8_t)(v >> 8);
                p->b[at + 1] = (uint8_t)v;
            }
            break;
        case 5: /* truncate */
            p->n = (uint16_t)below((uint32_t)p->n + 1);
            break;
        case 6: { /* extend */
            uint16_t add = (uint16_t)(1 + below(chance(20) ? 300 : 8));
            while (add-- && p->n < cap) {
                p->b[p->n++] = (uint8_t)rnd();
            }
            break;
        }
        case 7: /* delete one byte */
            if (p->n) {
                uint16_t at = (uint16_t)below(p->n);
                memmove(&p->b[at], &p->b[at + 1], (size_t)(p->n - at - 1));
                p->n--;
            }
            break;
        default: /* duplicate a chunk */
            if (p->n && p->n < cap) {
                uint16_t at = (uint16_t)below(p->n);
                uint32_t want = 1 + below(16);
                uint16_t len = (uint16_t)FLOSS_MIN((uint32_t)(p->n - at), want);
                len = (uint16_t)FLOSS_MIN(len, (uint16_t)(cap - p->n));
                memmove(&p->b[at + len], &p->b[at], (size_t)(p->n - at));
                p->n = (uint16_t)(p->n + len);
            }
            break;
        }
    }
    if (p->n > cap) {
        p->n = cap;
    }
}

/* ======================================================================== */
/* Scenario                                                                 */
/* ======================================================================== */

static const floss_addr_t k_le_peer = {{0x02, 0x11, 0x22, 0x33, 0x44, 0xC5}};
static const floss_addr_t k_rand_addr = {{0x66, 0x55, 0x44, 0x33, 0x22, 0x11}};

#define FZ_SPP_CHANNEL 1
#define FZ_RFC_DLCI (FZ_SPP_CHANNEL << 1)

typedef struct {
    floss_transport_t *t;
    uint16_t classic;   /* 0xFFFF = none */
    uint16_t le;        /* 0xFFFF = none */
    floss_sim_chan_t *ctrl;
    floss_sim_chan_t *intr;
    floss_sim_chan_t *sdp;
    floss_sim_chan_t *rfc;
    /* Real SDP continuation states seen in answers (replayed and mutated). */
    uint8_t cont[17];
    uint8_t cont_len;
    /* Last SDP request (without its continuation) to continue with cont[]. */
    uint8_t sdp_last[700];
    uint16_t sdp_last_len;
    uint8_t sdp_last_pdu;
    unsigned rounds;
    unsigned packets;
} fz_state_t;

static fz_state_t g;

static floss_hid_config_t g_hid_cfg;
static floss_hogp_config_t g_hogp_cfg;

static const floss_hogp_report_t k_hogp_reports[] = {
    {.id = 1, .type = FLOSS_HID_REPORT_INPUT, .len = 6},
    {.id = 2, .type = FLOSS_HID_REPORT_OUTPUT, .len = 2},
    {.id = 3, .type = FLOSS_HID_REPORT_FEATURE, .len = 4},
};

static uint8_t hogp_get_report(uint8_t type, uint8_t report_id, uint8_t *out, uint16_t cap, uint16_t *len) {
    uint16_t n = FLOSS_MIN(cap, (uint16_t)4);
    memset(out, 0x5A, n);
    *len = n;
    return 0;
}

static uint8_t hogp_set_report(uint8_t type, uint8_t report_id, const uint8_t *data, uint16_t len) {
    return 0;
}

static const floss_hogp_callbacks_t k_hogp_cb = {
    .on_get_report = hogp_get_report,
    .on_set_report = hogp_set_report,
};

/* A long, writable stored value: Read Blob, Prepare / Execute Write targets. */
static uint8_t g_long_val[128];
static uint16_t g_long_len;
static const floss_gatt_attr_t k_fuzz_svc[] = {
    FLOSS_GATT_PRIMARY_SERVICE16(0xFFF0),
    FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_READ | FLOSS_GATT_PROP_WRITE),
    FLOSS_GATT_VALUE16(0xFFF1, FLOSS_GATT_P_RW, g_long_val, sizeof g_long_val, &g_long_len),
};

static unsigned g_spp_bytes;

static void spp_on_data(floss_rfcomm_dlc_t *dlc, const uint8_t *data, uint16_t len) {
    /* Touch every byte: ASan flags a payload that points outside the PDU. */
    for (uint16_t i = 0; i < len; i++) {
        g_spp_bytes += data[i];
    }
    /* The API promises at most dlc->mtu bytes per call. */
    if (len > dlc->mtu) {
        FAIL("RFCOMM delivered %u bytes on a DLC with mtu %u", (unsigned)len, (unsigned)dlc->mtu);
    }
}

static const floss_rfcomm_ops_t k_spp_ops = {
    .on_data = spp_on_data,
};

static bool cond_ready(void *arg) {
    (void)arg;
    return floss_hci_ready();
}

/** Fresh stack with HID (Classic), HOGP + GATT services (LE), SDP and SPP. */
static void fz_stack_start(void) {
    floss_test_hal_reset();
    floss_test_log_quiet(!g_verbose);
    floss_test_store_reset();
    memset(&g, 0, offsetof(fz_state_t, rounds));
    g.classic = 0xFFFF;
    g.le = 0xFFFF;

    floss_sim_config_t sc;
    floss_tc_sim_config(&sc);
    sc.seed = rnd() | 1;
    g.t = floss_sim_init(&sc);

    floss_init_t init = {.transport = g.t, .chipset = NULL};
    ASSERT_EQ(floss_init(&init), FLOSS_OK);
    floss_tc_hid_config(&g_hid_cfg, 1000);
    g_hid_cfg.auto_reconnect = false;
    ASSERT_EQ(floss_hid_init(&g_hid_cfg, &floss_tc_hid_callbacks), FLOSS_OK);
    ASSERT_EQ(floss_spp_init(FZ_SPP_CHANNEL, "Fuzz Serial", &k_spp_ops, NULL), FLOSS_OK);

    floss_hogp_config_defaults(&g_hogp_cfg);
    g_hogp_cfg.report_map = floss_tc_report_desc;
    g_hogp_cfg.report_map_len = floss_tc_report_desc_len;
    g_hogp_cfg.reports = k_hogp_reports;
    g_hogp_cfg.report_count = (uint8_t)(sizeof k_hogp_reports / sizeof k_hogp_reports[0]);
    g_hogp_cfg.boot_device = true;
    g_hogp_cfg.manufacturer = "FLOSS";
    g_hogp_cfg.model = "Fuzz";
    g_hogp_cfg.vendor_id_source = 2;
    (void)floss_hogp_init(&g_hogp_cfg, &k_hogp_cb);
    g_long_len = 0;
    ASSERT_NE(floss_gatt_register(k_fuzz_svc, (uint16_t)(sizeof k_fuzz_svc / sizeof k_fuzz_svc[0])), 0);

    ASSERT_EQ(floss_start(), FLOSS_OK);
    if (!floss_sim_pump_until(cond_ready, NULL, 5000)) {
        FAIL("stack never reported READY");
    }
    floss_sim_pump(20);
}

static bool cond_rfc_dlc_open(void *arg) {
    (void)arg;
    for (unsigned i = 0; i < FLOSS_RFCOMM_MAX_DLC; i++) {
        if (floss_rfcomm_ctx.dlc[i].state == FLOSS_RFCOMM_DLC_OPEN) {
            return true;
        }
    }
    return false;
}

/** One RFCOMM frame as the session initiator (the sim side) sends it. */
static void rfc_build(pkt_t *p, uint8_t dlci, uint8_t type, bool pf, const uint8_t *info, uint16_t len, int credits) {
    p_reset(p);
    bool cr = type != RFC_UA && type != RFC_DM; /* initiator: commands C/R = 1 */
    p_u8(p, (uint8_t)(RFC_EA | (cr ? RFC_CR : 0) | (dlci << 2)));
    p_u8(p, (uint8_t)(type | (pf ? RFC_PF : 0)));
    if (len <= 127) {
        p_u8(p, (uint8_t)((len << 1) | RFC_EA));
    } else {
        p_u8(p, (uint8_t)(len << 1));
        p_u8(p, (uint8_t)(len >> 7));
    }
    uint8_t fcs = floss_rfcomm_fcs(p->b, type == RFC_UIH ? 2 : p->n);
    if (credits >= 0) {
        p_u8(p, (uint8_t)credits);
    }
    p_bytes(p, info, len);
    p_u8(p, fcs);
}

static void rfc_mcc(pkt_t *p, uint8_t type, bool cmd, const uint8_t *v, uint16_t vlen) {
    uint8_t info[300];
    uint16_t n = 0;
    info[n++] = (uint8_t)(type | (cmd ? RFC_CR : 0) | RFC_EA);
    if (vlen <= 127) {
        info[n++] = (uint8_t)((vlen << 1) | RFC_EA);
    } else {
        info[n++] = (uint8_t)(vlen << 1);
        info[n++] = (uint8_t)(vlen >> 7);
    }
    vlen = (uint16_t)FLOSS_MIN(vlen, (uint16_t)(sizeof info - n));
    memcpy(&info[n], v, vlen);
    rfc_build(p, 0, RFC_UIH, false, info, (uint16_t)(n + vlen), -1);
}

static void rfc_send(const pkt_t *p) {
    if (g.rfc && g.rfc->state == FLOSS_SIM_CHAN_OPEN) {
        (void)floss_sim_l2cap_send(g.rfc, p->b, p->n);
    }
}

/** Open the RFCOMM session and DLC 2 (server channel 1) like a PC would. */
static void fz_open_rfcomm(void) {
    g.rfc = floss_sim_l2cap_connect(g.classic, FLOSS_PSM_RFCOMM);
    if (!g.rfc || g.rfc->state != FLOSS_SIM_CHAN_OPEN) {
        FAIL("RFCOMM L2CAP channel did not open");
    }
    pkt_t p;
    rfc_build(&p, 0, RFC_SABM, true, NULL, 0, -1);
    rfc_send(&p);
    floss_sim_pump(5);
    const uint8_t pn[8] = {FZ_RFC_DLCI, 0xF0, 7, 0, 0x7F, 0x00, 0, 7};
    rfc_mcc(&p, RFC_MCC_PN, true, pn, sizeof pn);
    rfc_send(&p);
    floss_sim_pump(5);
    rfc_build(&p, FZ_RFC_DLCI, RFC_SABM, true, NULL, 0, -1);
    rfc_send(&p);
    if (!floss_sim_pump_until(cond_rfc_dlc_open, NULL, 200)) {
        FAIL("RFCOMM DLC never opened");
    }
    const uint8_t msc[2] = {(uint8_t)(RFC_EA | RFC_CR | (FZ_RFC_DLCI << 2)), (uint8_t)(0x8D)};
    rfc_mcc(&p, RFC_MCC_MSC, true, msc, sizeof msc);
    rfc_send(&p);
    floss_sim_pump(5);
}

/** Bring the stack to the interesting state for this round. */
static void fz_scenario(bool classic, bool le) {
    fz_stack_start();
    if (classic) {
        g.classic = floss_tc_hid_session(&g.ctrl, &g.intr);
        g.sdp = floss_sim_l2cap_connect(g.classic, FLOSS_PSM_SDP);
        if (!g.sdp || g.sdp->state != FLOSS_SIM_CHAN_OPEN) {
            FAIL("SDP channel did not open");
        }
        fz_open_rfcomm();
    }
    if (le) {
        g.le = floss_sim_le_connect(&k_le_peer, 0, 1);
        ASSERT_NE(g.le, FLOSS_SIM_NO_HANDLE);
        floss_sim_pump(10);
        ASSERT_NOT_NULL(floss_le_link(g.le));
        if (chance(70)) {
            /* As if SMP had encrypted the link: opens the HOGP / encrypted attributes. */
            floss_le_on_security_changed(g.le, (uint8_t)(2 + below(3)));
            floss_sim_pump(5);
        }
    }
}

static bool link_alive(uint16_t h) {
    return h != 0xFFFF && floss_hci_conn_by_handle(h) != NULL && floss_sim_link(h) != NULL;
}

/** Let time pass in big steps (timers: command timeout, RTX, SMP, ...). */
static void fz_pump_time(uint32_t us) {
    floss_sim_config_t *c = floss_sim_cfg();
    uint32_t old = c->pump_step_us;
    c->pump_step_us = 20000;
    floss_sim_pump((us + 19999) / 20000);
    c->pump_step_us = old;
}

/** Between packets: a few polls, sometimes a long pause so timers fire. */
static void fz_settle(void) {
    floss_sim_pump(1 + below(4));
    if (chance(2)) {
        fz_pump_time(below(chance(10) ? 40000000 : 3000000));
    }
}

static bool cond_link_up(void *arg) {
    const floss_sim_link_t *l = floss_sim_link((uint16_t)(uintptr_t)arg);
    return l && l->state == FLOSS_SIM_LINK_OPEN && floss_hci_conn_by_handle((uint16_t)(uintptr_t)arg) != NULL;
}

static void inject_event(const uint8_t *pkt, uint16_t len) {
    g.t->rx(g.t->rx_user, FLOSS_PKT_EVT, pkt, len);
}

static void inject_acl(const uint8_t *pkt, uint16_t len) {
    g.t->rx(g.t->rx_user, FLOSS_PKT_ACL, pkt, len);
}

static void inject_disconnect(uint16_t handle) {
    uint8_t e[6] = {FLOSS_EVT_DISCONNECTION_COMPLETE, 4, 0, 0, 0, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED};
    floss_put_le16(&e[3], handle);
    inject_event(e, sizeof e);
}

/**
 * Tear every link down and check the stack went back to idle. Phantom links
 * (created by injected events, unknown to the sim) get a Disconnection
 * Complete straight from the "controller".
 */
static void fz_teardown(const char *target) {
    /* Links the sim still holds even if injected events made the stack forget them. */
    if (g.classic != 0xFFFF && floss_sim_link(g.classic)) {
        (void)floss_sim_remote_disconnect(g.classic, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED);
    }
    if (g.le != 0xFFFF && floss_sim_link(g.le)) {
        (void)floss_sim_remote_disconnect(g.le, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED);
    }
    for (unsigned pass = 0; pass < 3; pass++) {
        for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
            floss_conn_t *c = floss_hci_conn_at(i);
            if (c->state != FLOSS_CONN_OPEN && c->state != FLOSS_CONN_CLOSING) {
                continue;
            }
            if (floss_sim_link(c->handle) && pass == 0) {
                (void)floss_sim_remote_disconnect(c->handle, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED);
            } else {
                inject_disconnect(c->handle);
            }
        }
        floss_sim_pump(20);
        fz_pump_time(5000000); /* command timeouts, RTX, pairing timers */
    }
    floss_sim_pump(50);
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        const floss_conn_t *c = floss_hci_conn_at(i);
        if (c->state == FLOSS_CONN_OUTGOING || c->state == FLOSS_CONN_INCOMING) {
            /* A page / request the controller never concluded: HCI's guard fails it. */
            fz_pump_time(FLOSS_HCI_CONN_PENDING_TIMEOUT_US + 1000000u);
            break;
        }
    }

    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        const floss_conn_t *c = floss_hci_conn_at(i);
        if (c->state != FLOSS_CONN_FREE) {
            FAIL("%s round %u: connection slot %u stuck in state %d (handle 0x%03x)", target, g.rounds, i,
                 (int)c->state, c->handle);
        }
    }
    for (unsigned i = 0; i < FLOSS_MAX_L2CAP_CHANNELS; i++) {
        const floss_l2cap_slot_t *s = &floss_l2cap_slots[i];
        if (s->in_use) {
            FAIL("%s round %u: L2CAP slot %u (psm 0x%04x state %d) survived link loss", target, g.rounds, i,
                 s->chan.psm, (int)s->chan.state);
        }
    }
    for (unsigned i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        if (floss_rfcomm_ctx.mux[i].state != RFC_MUX_FREE) {
            FAIL("%s round %u: RFCOMM session %u survived link loss", target, g.rounds, i);
        }
    }
    for (unsigned i = 0; i < FLOSS_RFCOMM_MAX_DLC; i++) {
        if (floss_rfcomm_ctx.dlc[i].state != FLOSS_RFCOMM_DLC_FREE) {
            FAIL("%s round %u: RFCOMM DLC %u (state %d) survived link loss", target, g.rounds, i,
                 (int)floss_rfcomm_ctx.dlc[i].state);
        }
    }
    if (floss_le_link_count() != 0) {
        FAIL("%s round %u: %u LE link(s) survived", target, g.rounds, (unsigned)floss_le_link_count());
    }
    if (floss_hid_state() != FLOSS_HID_STATE_IDLE) {
        FAIL("%s round %u: HID state %d after link loss", target, g.rounds, (int)floss_hid_state());
    }
    if (floss_buf_free_count() != FLOSS_BUF_COUNT) {
        FAIL("%s round %u: buffer pool leak: %u of %u free after teardown (HCI queue %u, credits %u, "
             "in flight %u)",
             target, g.rounds, (unsigned)floss_buf_free_count(), (unsigned)FLOSS_BUF_COUNT,
             (unsigned)floss_hci_g.cmdq_count, (unsigned)floss_hci_g.cmd_credits, (unsigned)floss_hci_g.cmd_inflight);
    }
    ASSERT_EQ(floss_test_critical_depth(), 0);

    /* The stack must still be usable: a host can connect again. */
    if (chance(25)) {
        /* Allow for the command-credit guard: injected events may have left
         * the controller "granting" no command credit. */
        uint16_t h = floss_sim_remote_connect(&floss_tc_host, FLOSS_TC_HOST_COD);
        ASSERT_NE(h, FLOSS_SIM_NO_HANDLE);
        floss_sim_config_t *c = floss_sim_cfg();
        uint32_t old = c->pump_step_us;
        c->pump_step_us = 1000;
        bool up = floss_sim_pump_until(cond_link_up, (void *)(uintptr_t)h, 5000);
        c->pump_step_us = old;
        if (!up) {
            FAIL("%s round %u: stack no longer accepts connections (HCI queue %u, credits %u, in flight %u)",
                 target, g.rounds, (unsigned)floss_hci_g.cmdq_count, (unsigned)floss_hci_g.cmd_credits,
                 (unsigned)floss_hci_g.cmd_inflight);
        }
        floss_sim_pump(10);
        ASSERT_EQ(floss_sim_remote_disconnect(h, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED), FLOSS_OK);
        floss_sim_pump(20);
        ASSERT_EQ(floss_buf_free_count(), FLOSS_BUF_COUNT);
    }
}

typedef bool (*fz_step_fn)(void); /* one hostile packet; false = scenario gone, end the round */

/** Run rounds of (scenario, burst of packets, teardown) until g_iters packets were sent. */
static void fz_run(const char *target, bool classic, bool le, fz_step_fn step) {
    rng_seed(target);
    g.rounds = 0;
    g.packets = 0;
    while (g.packets < g_iters) {
        fz_scenario(classic, le);
        unsigned burst = 20 + below(chance(20) ? 400 : 120);
        for (unsigned i = 0; i < burst && g.packets < g_iters; i++) {
            g.packets++;
            if (!step()) {
                break;
            }
            fz_settle();
        }
        fz_teardown(target);
        g.rounds++;
    }
    printf("    %s: %u packets in %u rounds (seed %u)\n", target, g.packets, g.rounds, (unsigned)g_seed);
}

/* ======================================================================== */
/* Field helpers                                                            */
/* ======================================================================== */

static uint16_t pick_classic_handle(void) {
    switch (below(10)) {
    case 0:
        return (uint16_t)rnd();
    case 1:
        return g.le;
    case 2:
        return 0x0FFF;
    case 3:
        return (uint16_t)(g.classic ^ 1);
    default:
        return g.classic;
    }
}

static uint16_t pick_le_handle(void) {
    switch (below(10)) {
    case 0:
        return (uint16_t)rnd();
    case 1:
        return g.classic;
    default:
        return g.le;
    }
}

static void p_addr(pkt_t *p) {
    const floss_addr_t *a;
    switch (below(6)) {
    case 0:
        a = &k_rand_addr;
        break;
    case 1:
        a = &k_le_peer;
        break;
    case 2: {
        uint8_t r[6];
        rnd_bytes(r, 6);
        p_bytes(p, r, 6);
        return;
    }
    default:
        a = &floss_tc_host;
        break;
    }
    p_bytes(p, a->b, 6);
}

/* ======================================================================== */
/* Target: HCI events                                                       */
/* ======================================================================== */

/*
 * Event templates, one character per field:
 *   S status   H Classic handle   L LE handle   A address   B byte   W 16-bit
 *   C class of device  K 16 bytes  F 8 bytes  P 64 bytes  D 32 bytes
 *   N 248-byte name    E EIR       O opcode   R random tail  0-9 literal byte
 */
typedef struct {
    uint8_t code;
    const char *fmt;
} evt_tpl_t;

static const evt_tpl_t k_evts[] = {
    {0x01, "S"},          {0x02, "1ABBBCW"},     {0x03, "SHABB"},     {0x04, "ACB"},
    {0x05, "SHB"},        {0x06, "SH"},          {0x07, "SAN"},       {0x08, "SHB"},
    {0x09, "SH"},         {0x0B, "SHF"},         {0x0C, "SHBWW"},     {0x0E, "BOR"},
    {0x0F, "SBO"},        {0x10, "B"},           {0x12, "SAB"},       {0x13, ""},
    {0x14, "SHBW"},       {0x15, "1AK"},         {0x16, "A"},         {0x17, "A"},
    {0x18, "AKB"},        {0x1A, "B"},           {0x1B, "HB"},        {0x1D, "SHBB"},
    {0x20, "AB"},         {0x22, "1ABBCWB"},     {0x23, "SHBBF"},     {0x2C, "SHABBBWWB"},
    {0x2F, "1ABBCWBE"},   {0x30, "SH"},          {0x31, "A"},         {0x32, "ABBB"},
    {0x33, "AWW"},        {0x34, "A"},           {0x35, "A"},         {0x36, "SA"},
    {0x38, "HW"},         {0x3B, "AWW"},         {0x3C, "AB"},        {0x3D, "AF"},
    {0x57, "H"},          {0xFF, "R"},
};

static const evt_tpl_t k_le_subevts[] = {
    {0x01, "SLBBAWWWB"}, {0x02, "1BBABR"}, {0x03, "SLWWW"}, {0x04, "SLF"},  {0x05, "LFW"},
    {0x06, "LWWWW"},     {0x07, "LWWWW"},  {0x08, "SP"},    {0x09, "SD"},   {0x0A, "SLBBAAAWWWB"},
    {0x0C, "SLBB"},      {0x0D, "R"},      {0x14, "LB"},    {0x12, "R"},
};

static uint16_t pick_opcode(void) {
    static const uint16_t k_ops[] = {
        0x0C03, 0x1001, 0x1002, 0x1003, 0x1009, 0x1005, 0x0C6D, 0x2002, 0x2003, 0x2001, 0x0C01, 0x2063,
        0x0406, 0x0409, 0x040B, 0x040C, 0x0411, 0x0413, 0x0419, 0x041B, 0x0428, 0x042B, 0x042C, 0x042D,
        0x0803, 0x080D, 0x080F, 0x0C13, 0x0C1A, 0x0C24, 0x0C56, 0x0C7A, 0x2005, 0x2006, 0x2008, 0x2009,
        0x200A, 0x200D, 0x2013, 0x2016, 0x2017, 0x2018, 0x201A, 0x201B, 0x2022, 0x2025, 0x2026, 0x0401,
        0x0402, 0x0405, 0x0408, 0x0C05, 0x0C45, 0x0C52, 0x1004,
    };
    if (chance(40) && floss_hci_g.cmd_inflight) {
        return floss_hci_g.inflight_opcode;
    }
    return chance(85) ? k_ops[below(sizeof k_ops / sizeof k_ops[0])] : (uint16_t)rnd();
}

static void p_template(pkt_t *p, const char *fmt) {
    for (const char *f = fmt; *f; f++) {
        switch (*f) {
        case 'S':
            p_u8(p, chance(80) ? 0 : (uint8_t)rnd());
            break;
        case 'H':
            p_le16(p, pick_classic_handle());
            break;
        case 'L':
            p_le16(p, pick_le_handle());
            break;
        case 'A':
            p_addr(p);
            break;
        case 'B':
            p_u8(p, chance(30) ? (uint8_t)below(4) : (uint8_t)rnd());
            break;
        case 'W':
            p_le16(p, interesting16());
            break;
        case 'C':
            p_rand(p, 3);
            break;
        case 'K':
            p_rand(p, 16);
            break;
        case 'F':
            p_rand(p, 8);
            break;
        case 'P':
            p_rand(p, 64);
            break;
        case 'D':
            p_rand(p, 32);
            break;
        case 'N':
            for (unsigned i = 0; i < 248; i++) {
                p_u8(p, chance(90) ? (uint8_t)('a' + below(26)) : (uint8_t)rnd());
            }
            break;
        case 'E': {
            /* AD structures with random (sometimes lying) lengths. */
            uint16_t start = p->n;
            while (p->n < start + 240) {
                uint8_t l = chance(70) ? (uint8_t)below(12) : (uint8_t)rnd();
                p_u8(p, l);
                p_u8(p, (uint8_t)(chance(50) ? below(10) : rnd()));
                for (uint8_t k = 1; k < l && p->n < start + 240; k++) {
                    p_u8(p, (uint8_t)rnd());
                }
            }
            break;
        }
        case 'O':
            p_le16(p, pick_opcode());
            break;
        case 'R':
            p_rand(p, (uint16_t)below(chance(20) ? 250 : 20));
            break;
        default:
            if (*f >= '0' && *f <= '9') {
                p_u8(p, (uint8_t)(*f - '0'));
            }
            break;
        }
    }
}

static bool step_hci(void) {
    pkt_t params;
    p_reset(&params);
    uint8_t code;
    if (chance(5)) {
        code = (uint8_t)rnd();
        p_rand(&params, (uint16_t)below(256));
    } else {
        const evt_tpl_t *t = &k_evts[below(sizeof k_evts / sizeof k_evts[0])];
        code = t->code;
        if (code == FLOSS_EVT_NUMBER_OF_COMPLETED_PACKETS) {
            uint8_t n = chance(80) ? (uint8_t)(1 + below(3)) : (uint8_t)rnd();
            p_u8(&params, n);
            for (unsigned i = 0; i < n && params.n < 250; i++) {
                p_le16(&params, chance(50) ? pick_classic_handle() : pick_le_handle());
                p_le16(&params, chance(70) ? (uint16_t)below(3) : interesting16());
            }
        } else if (code == FLOSS_EVT_LE_META) {
            const evt_tpl_t *s = &k_le_subevts[below(sizeof k_le_subevts / sizeof k_le_subevts[0])];
            p_u8(&params, s->code);
            p_template(&params, s->fmt);
        } else if (code == 0x01 && chance(50)) {
            code = FLOSS_EVT_LE_META;
            p_u8(&params, 0x05); /* LTK request for the real LE link */
            p_le16(&params, g.le);
            p_rand(&params, 10);
        } else {
            p_template(&params, t->fmt);
        }
        if (chance(30)) {
            mutate(&params, 255);
        }
    }
    if (params.n > 255) {
        params.n = 255;
    }
    pkt_t e;
    p_reset(&e);
    p_u8(&e, code);
    p_u8(&e, (uint8_t)params.n);
    p_bytes(&e, params.b, params.n);
    if (chance(5)) {
        e.b[1] = (uint8_t)rnd(); /* parameter length lies (truncated or padded) */
    }
    if (chance(3)) {
        p_rand(&e, (uint16_t)below(10)); /* trailing transport padding */
    }
    if (chance(3)) {
        e.n = (uint16_t)below(e.n + 1u); /* transport delivered a short packet */
    }
    inject_event(e.b, e.n);
    return true; /* the HCI layer has no scenario to lose: keep going */
}

TEST(fuzz_hci_events) {
    fz_run("hci", true, true, step_hci);
}

/* ======================================================================== */
/* L2CAP PDU builders (shared)                                              */
/* ======================================================================== */

static uint16_t pick_cid(void) {
    static const uint16_t fixed[] = {0x0000, 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x003F, 0x0040};
    if (chance(60)) {
        /* A live channel's local CID (the stack side). */
        unsigned start = below(FLOSS_MAX_L2CAP_CHANNELS);
        for (unsigned k = 0; k < FLOSS_MAX_L2CAP_CHANNELS; k++) {
            const floss_l2cap_slot_t *s = &floss_l2cap_slots[(start + k) % FLOSS_MAX_L2CAP_CHANNELS];
            if (s->in_use) {
                return chance(80) ? s->chan.local_cid : s->chan.remote_cid;
            }
        }
    }
    return chance(50) ? fixed[below(sizeof fixed / sizeof fixed[0])] : interesting16();
}

/** Random configuration options (valid and broken). */
static void p_cfg_opts(pkt_t *p) {
    unsigned n = below(5);
    for (unsigned i = 0; i < n; i++) {
        uint8_t type = (uint8_t)(chance(80) ? 1 + below(7) : rnd());
        if (chance(20)) {
            type |= 0x80; /* hint */
        }
        uint8_t len;
        switch (type & 0x7F) {
        case 1: case 2: len = 2; break;
        case 3: len = 14; break;
        case 4: len = 9; break;
        case 5: len = 1; break;
        case 6: len = 16; break;
        case 7: len = 2; break;
        default: len = (uint8_t)below(8); break;
        }
        if (chance(15)) {
            len = (uint8_t)(chance(50) ? below(20) : rnd());
        }
        p_u8(p, type);
        p_u8(p, len);
        for (uint8_t k = 0; k < len && p->n < 600; k++) {
            p_u8(p, (uint8_t)(k < 2 && chance(50) ? interesting16() : rnd()));
        }
    }
}

/** One Classic signaling command (header + data). */
static void p_sig_cmd(pkt_t *p) {
    static const uint8_t codes[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
                                    0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18};
    uint8_t code = chance(90) ? codes[below(sizeof codes)] : (uint8_t)rnd();
    uint8_t id = chance(90) ? (uint8_t)(1 + below(255)) : 0;
    if (chance(25)) {
        /* Echo the identifier of a request the stack has outstanding. */
        for (unsigned k = 0; k < FLOSS_MAX_L2CAP_CHANNELS; k++) {
            if (floss_l2cap_slots[k].in_use && floss_l2cap_slots[k].chan.sig_id) {
                id = floss_l2cap_slots[k].chan.sig_id;
                break;
            }
        }
    }
    pkt_t d;
    p_reset(&d);
    static const uint16_t psms[] = {0x0001, 0x0003, 0x0011, 0x0013, 0x0005, 0x1001, 0x0000, 0xFFFF};
    switch (code) {
    case 0x01: p_le16(&d, interesting16()); p_le16(&d, pick_cid()); p_le16(&d, pick_cid()); break;
    case 0x02: p_le16(&d, psms[below(sizeof psms / sizeof psms[0])]); p_le16(&d, (uint16_t)(0x40 + below(8))); break;
    case 0x03: p_le16(&d, pick_cid()); p_le16(&d, pick_cid()); p_le16(&d, (uint16_t)below(5)); p_le16(&d, (uint16_t)below(3)); break;
    case 0x04: p_le16(&d, pick_cid()); p_le16(&d, (uint16_t)below(2)); p_cfg_opts(&d); break;
    case 0x05: p_le16(&d, pick_cid()); p_le16(&d, (uint16_t)below(2)); p_le16(&d, (uint16_t)below(5)); p_cfg_opts(&d); break;
    case 0x06: case 0x07: p_le16(&d, pick_cid()); p_le16(&d, pick_cid()); break;
    case 0x08: case 0x09: p_rand(&d, (uint16_t)below(chance(10) ? 300 : 20)); break;
    case 0x0A: p_le16(&d, (uint16_t)(1 + below(4))); break;
    case 0x0B: p_le16(&d, (uint16_t)(1 + below(4))); p_le16(&d, (uint16_t)below(2)); p_rand(&d, (uint16_t)below(10)); break;
    default: p_rand(&d, (uint16_t)below(16)); break;
    }
    if (chance(15)) {
        mutate(&d, 600);
    }
    uint16_t len = d.n;
    if (chance(5)) {
        len = interesting16(); /* the length field lies */
    }
    p_u8(p, code);
    p_u8(p, id);
    p_le16(p, len);
    p_bytes(p, d.b, d.n);
}

/** L2CAP basic frame around payload (header length normally right). */
static void p_l2cap(pkt_t *p, uint16_t cid, const uint8_t *payload, uint16_t n) {
    p_reset(p);
    p_le16(p, chance(95) ? n : interesting16());
    p_le16(p, cid);
    p_bytes(p, payload, n);
}

/* ======================================================================== */
/* Target: ACL / reassembly                                                 */
/* ======================================================================== */

/** A plausible L2CAP PDU for the classic or LE link. */
static void p_some_pdu(pkt_t *out, bool le) {
    pkt_t pl;
    p_reset(&pl);
    uint16_t cid;
    if (le) {
        static const uint16_t cids[] = {0x0004, 0x0005, 0x0006};
        cid = chance(90) ? cids[below(3)] : pick_cid();
        p_rand(&pl, (uint16_t)below(chance(20) ? 700 : 40));
    } else if (chance(40)) {
        cid = 0x0001;
        unsigned n = 1 + below(3);
        while (n--) {
            p_sig_cmd(&pl);
        }
    } else {
        cid = pick_cid();
        p_rand(&pl, (uint16_t)below(chance(20) ? 900 : 60));
    }
    p_l2cap(out, cid, pl.b, pl.n);
}

static bool step_acl(void) {
    bool le = chance(30);
    uint16_t handle = le ? pick_le_handle() : pick_classic_handle();
    pkt_t pdu;
    p_some_pdu(&pdu, le);

    if (chance(15)) {
        /* Completely random ACL packet. */
        pkt_t a;
        p_reset(&a);
        p_le16(&a, (uint16_t)(chance(50) ? handle | ((uint16_t)below(16) << 12) : rnd()));
        uint16_t n = (uint16_t)below(chance(10) ? 1050 : 64);
        p_le16(&a, chance(80) ? n : interesting16());
        p_rand(&a, n);
        if (chance(10)) {
            a.n = (uint16_t)below(a.n + 1u);
        }
        inject_acl(a.b, a.n);
        return true;
    }

    /* Split the PDU into fragments with mostly-right boundary flags. */
    uint16_t off = 0;
    bool first = true;
    do {
        uint32_t want = 1 + below(chance(50) ? 64 : 1021); /* FLOSS_MIN evaluates twice */
        uint16_t frag = (uint16_t)FLOSS_MIN((uint32_t)(pdu.n - off), want);
        uint8_t pb = first ? (uint8_t)(chance(50) ? 0x2 : 0x0) : 0x1;
        if (chance(8)) {
            pb = (uint8_t)below(4);
        }
        pkt_t a;
        p_reset(&a);
        p_le16(&a, (uint16_t)(handle | ((uint16_t)pb << 12) | (chance(5) ? 0x4000 : 0)));
        p_le16(&a, chance(97) ? frag : interesting16());
        p_bytes(&a, &pdu.b[off], frag);
        if (chance(3)) {
            p_rand(&a, (uint16_t)below(5)); /* padding */
        }
        inject_acl(a.b, a.n);
        off = (uint16_t)(off + frag);
        first = false;
        if (chance(10)) {
            floss_sim_pump(1); /* interleave with normal traffic */
        }
        if (chance(5)) {
            break; /* abandon the PDU halfway */
        }
    } while (off < pdu.n);
    return true;
}

TEST(fuzz_acl) {
    fz_run("acl", true, true, step_acl);
}

/* ======================================================================== */
/* Target: L2CAP signaling (Classic + LE)                                   */
/* ======================================================================== */

static void p_le_sig_cmd(pkt_t *p) {
    static const uint8_t codes[] = {0x01, 0x06, 0x07, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x0A, 0x08};
    uint8_t code = chance(90) ? codes[below(sizeof codes)] : (uint8_t)rnd();
    pkt_t d;
    p_reset(&d);
    switch (code) {
    case 0x12: for (int i = 0; i < 4; i++) { p_le16(&d, interesting16()); } break;
    case 0x13: p_le16(&d, (uint16_t)below(3)); break;
    case 0x14: for (int i = 0; i < 5; i++) { p_le16(&d, interesting16()); } break;
    case 0x16: p_le16(&d, pick_cid()); p_le16(&d, interesting16()); break;
    default: p_rand(&d, (uint16_t)below(20)); break;
    }
    if (chance(15)) {
        mutate(&d, 200);
    }
    p_u8(p, code);
    p_u8(p, (uint8_t)(chance(95) ? 1 + below(255) : 0));
    p_le16(p, chance(95) ? d.n : interesting16());
    p_bytes(p, d.b, d.n);
}

static bool step_l2cap(void) {
    pkt_t frame;
    p_reset(&frame);
    bool le = chance(25);
    unsigned n = chance(70) ? 1 : 2 + below(6);
    for (unsigned i = 0; i < n && frame.n < 640; i++) {
        if (le) {
            p_le_sig_cmd(&frame);
        } else {
            p_sig_cmd(&frame);
        }
    }
    if (chance(10)) {
        mutate(&frame, 660);
    }
    pkt_t pdu;
    p_l2cap(&pdu, le ? 0x0005 : 0x0001, frame.b, frame.n);
    uint16_t h = le ? g.le : g.classic;
    if (!link_alive(h)) {
        return false;
    }
    (void)floss_sim_acl_send_pdu(h, pdu.b, pdu.n);
    return true;
}

TEST(fuzz_l2cap_signaling) {
    fz_run("l2cap", true, true, step_l2cap);
}

/* ======================================================================== */
/* Target: SDP                                                              */
/* ======================================================================== */

/** A random data element (bounded nesting), mostly well formed. */
static void p_de(pkt_t *p, unsigned depth) {
    unsigned kind = below(depth < 4 ? 8 : 6);
    switch (kind) {
    case 0: /* UUID16 */
        p_u8(p, 0x19);
        p_be16(p, chance(70) ? (uint16_t)(chance(50) ? 0x1124 : 0x0100 + below(0x30)) : interesting16());
        break;
    case 1: /* UUID32 */
        p_u8(p, 0x1A);
        p_be16(p, 0);
        p_be16(p, interesting16());
        break;
    case 2: { /* UUID128 */
        static const uint8_t base[12] = {0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
        p_u8(p, 0x1C);
        p_be16(p, 0);
        p_be16(p, chance(50) ? 0x1101 : interesting16());
        if (chance(80)) {
            p_bytes(p, base, 12);
        } else {
            p_rand(p, 12);
        }
        break;
    }
    case 3: /* uint16 attribute id */
        p_u8(p, 0x09);
        p_be16(p, interesting16());
        break;
    case 4: /* uint32 range */
        p_u8(p, 0x0A);
        p_be16(p, interesting16());
        p_be16(p, interesting16());
        break;
    case 5: { /* random header byte + random body */
        uint8_t hdr = (uint8_t)rnd();
        p_u8(p, hdr);
        if ((hdr & 7) == 5) {
            p_u8(p, (uint8_t)below(chance(80) ? 16 : 256));
        } else if ((hdr & 7) == 6) {
            p_be16(p, interesting16());
        } else if ((hdr & 7) == 7) {
            p_be16(p, 0);
            p_be16(p, interesting16());
        }
        p_rand(p, (uint16_t)below(8));
        break;
    }
    default: { /* sequence / alternative with a (usually right) length */
        uint8_t hdr = chance(85) ? 0x35 : (chance(50) ? 0x3D : (uint8_t)(0x36 + below(2)));
        p_u8(p, hdr);
        uint16_t len_at = p->n;
        if (hdr == 0x35 || hdr == 0x3D) {
            p_u8(p, 0);
        } else if (hdr == 0x36) {
            p_be16(p, 0);
        } else {
            p_be16(p, 0);
            p_be16(p, 0);
        }
        uint16_t body = p->n;
        unsigned items = below(chance(10) ? 20 : 4);
        for (unsigned i = 0; i < items && p->n < 500; i++) {
            p_de(p, depth + 1);
        }
        uint16_t blen = (uint16_t)(p->n - body);
        if (chance(10)) {
            blen = interesting16();
        }
        if (hdr == 0x35 || hdr == 0x3D) {
            p->b[len_at] = (uint8_t)blen;
        } else if (hdr == 0x36) {
            p->b[len_at] = (uint8_t)(blen >> 8);
            p->b[len_at + 1] = (uint8_t)blen;
        } else {
            p->b[len_at + 2] = (uint8_t)(blen >> 8);
            p->b[len_at + 3] = (uint8_t)blen;
        }
        break;
    }
    }
}

/** A sequence DE wrapping n elements: UUIDs (pattern) or attribute IDs / ranges. */
static void p_de_seq(pkt_t *p, unsigned n, bool uuids) {
    p_u8(p, 0x35);
    uint16_t at = p->n;
    p_u8(p, 0);
    for (unsigned i = 0; i < n && p->n < 400; i++) {
        if (uuids && chance(92)) {
            static const uint16_t k[] = {0x1124, 0x0100, 0x1101, 0x0003, 0x0011, 0x1000, 0x1200, 0x0001};
            if (chance(80)) {
                p_u8(p, 0x19);
                p_be16(p, chance(85) ? k[below(sizeof k / sizeof k[0])] : interesting16());
            } else if (chance(50)) {
                p_u8(p, 0x1A);
                p_be16(p, 0);
                p_be16(p, k[below(sizeof k / sizeof k[0])]);
            } else {
                static const uint8_t base[12] = {0x00, 0x00, 0x10, 0x00, 0x80, 0x00,
                                                 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
                p_u8(p, 0x1C);
                p_be16(p, 0);
                p_be16(p, k[below(sizeof k / sizeof k[0])]);
                p_bytes(p, base, 12);
            }
        } else if (!uuids && chance(92)) {
            if (chance(60)) {
                uint16_t a = chance(70) ? 0x0000 : interesting16();
                uint16_t b = chance(70) ? 0xFFFF : interesting16();
                p_u8(p, 0x0A);
                p_be16(p, chance(90) ? FLOSS_MIN(a, b) : a);
                p_be16(p, chance(90) ? FLOSS_MAX(a, b) : b);
            } else {
                static const uint16_t ids[] = {0x0000, 0x0001, 0x0004, 0x0005, 0x0006, 0x0009, 0x0100,
                                               0x0200, 0x0206, 0x020D, 0x0311, 0xFFFF};
                p_u8(p, 0x09);
                p_be16(p, chance(80) ? ids[below(sizeof ids / sizeof ids[0])] : interesting16());
            }
        } else {
            p_de(p, 1);
        }
    }
    p->b[at] = (uint8_t)(p->n - at - 1);
}

/** Remember the continuation state of an SDP response for later replay. */
static void sdp_capture(const uint8_t *r, int n) {
    if (n < 6) {
        return;
    }
    uint16_t plen = (uint16_t)((r[3] << 8) | r[4]);
    if ((uint32_t)plen + 5 != (uint32_t)n || r[0] == 0x01) {
        return;
    }
    /* Find the continuation field from the response layout. */
    uint32_t at;
    if (r[0] == 0x03 && n >= 9) {
        at = 9u + 4u * (uint32_t)((r[7] << 8) | r[8]); /* total(2) current(2) handles */
    } else if ((r[0] == 0x05 || r[0] == 0x07) && n >= 7) {
        at = 7u + (uint32_t)((r[5] << 8) | r[6]); /* byte count(2) attribute lists */
    } else {
        return;
    }
    if (at < (uint32_t)n && at + 1u + r[at] == (uint32_t)n && r[at] <= 16) {
        g.cont_len = r[at];
        memcpy(g.cont, &r[at + 1], r[at]);
    }
}

static bool step_sdp(void) {
    if (!g.sdp || g.sdp->state != FLOSS_SIM_CHAN_OPEN) {
        return false;
    }
    pkt_t params;
    p_reset(&params);
    uint8_t pdu;
    bool resume = g.cont_len && g.sdp_last_len && chance(60);
    if (resume) {
        /* Continue the previous request, the way a client pages through a long answer. */
        pdu = g.sdp_last_pdu;
        p_bytes(&params, g.sdp_last, g.sdp_last_len);
        p_u8(&params, g.cont_len);
        p_bytes(&params, g.cont, g.cont_len);
        if (chance(25)) {
            /* Forged offset (the check word still matches) or a broken token. */
            uint16_t at = (uint16_t)(params.n - g.cont_len + below(g.cont_len));
            if (chance(50)) {
                params.b[at] = (uint8_t)rnd();
            } else {
                uint16_t v = interesting16();
                params.b[params.n - 6] = 0;
                params.b[params.n - 5] = 0;
                params.b[params.n - 4] = (uint8_t)(v >> 8);
                params.b[params.n - 3] = (uint8_t)v;
            }
        }
    } else {
        static const uint8_t pdus[] = {0x02, 0x04, 0x06, 0x01, 0x03, 0x05, 0x07, 0x00, 0x08};
        pdu = chance(90) ? pdus[below(3)] : (chance(50) ? pdus[below(sizeof pdus)] : (uint8_t)rnd());
        switch (pdu) {
        case 0x02: /* ServiceSearchRequest */
            p_de_seq(&params, chance(60) ? 1 : 1 + below(chance(10) ? 16 : 3), true);
            p_be16(&params, chance(70) ? (uint16_t)(1 + below(8)) : interesting16());
            break;
        case 0x04: /* ServiceAttributeRequest: handle (32-bit) */
            p_u8(&params, 0x00);
            p_u8(&params, (uint8_t)(chance(90) ? 0x01 : rnd()));
            p_be16(&params, chance(70) ? (uint16_t)below(4) : interesting16());
            p_be16(&params, chance(60) ? (uint16_t)(7 + below(200)) : interesting16());
            p_de_seq(&params, 1 + below(3), false);
            break;
        case 0x06: /* ServiceSearchAttributeRequest */
            p_de_seq(&params, chance(60) ? 1 : 1 + below(chance(10) ? 16 : 3), true);
            p_be16(&params, chance(60) ? (uint16_t)(7 + below(200)) : interesting16());
            p_de_seq(&params, 1 + below(3), false);
            break;
        default:
            p_rand(&params, (uint16_t)below(40));
            break;
        }
        if (params.n <= sizeof g.sdp_last) {
            memcpy(g.sdp_last, params.b, params.n);
            g.sdp_last_len = params.n;
            g.sdp_last_pdu = pdu;
            g.cont_len = 0;
        }
        if (chance(85)) {
            p_u8(&params, 0); /* no continuation */
        } else {
            uint8_t l = (uint8_t)(chance(70) ? below(17) : rnd());
            p_u8(&params, l);
            p_rand(&params, (uint16_t)FLOSS_MIN(l, (uint8_t)40));
        }
        if (chance(12)) {
            mutate(&params, 660);
        }
    }
    pkt_t req;
    p_reset(&req);
    p_u8(&req, pdu);
    p_be16(&req, (uint16_t)rnd());
    p_be16(&req, chance(97) ? params.n : interesting16());
    p_bytes(&req, params.b, params.n);
    if (req.n > g.sdp->stack_mtu) {
        req.n = g.sdp->stack_mtu;
    }
    (void)floss_sim_l2cap_send(g.sdp, req.b, req.n);
    for (unsigned k = 0; k < 20 && floss_sim_l2cap_pending(g.sdp) == 0; k++) {
        floss_sim_pump(1);
    }
    uint8_t rsp[1100];
    int n;
    while ((n = floss_sim_l2cap_recv(g.sdp, rsp, sizeof rsp)) >= 0) {
        sdp_capture(rsp, n);
    }
    return true;
}

TEST(fuzz_sdp) {
    fz_run("sdp", true, false, step_sdp);
}

/* ======================================================================== */
/* Target: ATT                                                              */
/* ======================================================================== */

static uint16_t pick_attr_handle(void) {
    uint16_t last = floss_gatt_last_handle();
    switch (below(8)) {
    case 0:
        return interesting16();
    case 1:
        return 0;
    case 2:
        return (uint16_t)(last + below(3));
    default:
        return (uint16_t)(1 + below(last ? last : 1));
    }
}

/** A random Client Characteristic Configuration descriptor handle (0 if none). */
static uint16_t pick_cccd(void) {
    uint16_t last = floss_gatt_last_handle();
    uint16_t found[32];
    unsigned n = 0;
    for (uint32_t h = 1; h <= last && n < 32; h++) {
        if (floss_gatt_type16(floss_gatt_attr((uint16_t)h)) == FLOSS_GATT_UUID_CCCD) {
            found[n++] = (uint16_t)h;
        }
    }
    return n ? found[below(n)] : 0;
}

static void p_att_uuid(pkt_t *p) {
    static const uint16_t uu[] = {0x2800, 0x2801, 0x2802, 0x2803, 0x2902, 0x2A4D, 0x2A4B, 0x2A19, 0x2A00, 0x1812};
    if (chance(80)) {
        p_le16(p, uu[below(sizeof uu / sizeof uu[0])]);
    } else {
        static const uint8_t base[12] = {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00};
        if (chance(60)) {
            p_bytes(p, base, 12);
        } else {
            p_rand(p, 12);
        }
        p_le16(p, uu[below(sizeof uu / sizeof uu[0])]);
        p_le16(p, 0);
    }
}

static bool step_att(void) {
    if (!link_alive(g.le)) {
        return false;
    }
    static const uint8_t ops[] = {0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12, 0x52,
                                  0x16, 0x18, 0x1E, 0xD2, 0x20, 0x01, 0x03, 0x1B, 0x1D, 0x13};
    pkt_t p;
    p_reset(&p);
    uint8_t op = chance(92) ? ops[below(sizeof ops)] : (uint8_t)rnd();
    p_u8(&p, op);
    uint16_t start = pick_attr_handle();
    uint16_t end = chance(60) ? 0xFFFF : pick_attr_handle();
    switch (op) {
    case 0x02:
        p_le16(&p, interesting16());
        break;
    case 0x04:
        p_le16(&p, start);
        p_le16(&p, end);
        break;
    case 0x06:
        p_le16(&p, start);
        p_le16(&p, end);
        p_le16(&p, chance(70) ? 0x2800 : interesting16());
        if (chance(70)) {
            p_le16(&p, chance(50) ? 0x1812 : 0x180F);
        } else {
            p_rand(&p, (uint16_t)below(40));
        }
        break;
    case 0x08:
    case 0x10:
        p_le16(&p, start);
        p_le16(&p, end);
        p_att_uuid(&p);
        break;
    case 0x0A:
        p_le16(&p, start);
        break;
    case 0x0C:
        p_le16(&p, start);
        p_le16(&p, interesting16());
        break;
    case 0x0E: {
        unsigned n = 1 + below(chance(10) ? 60 : 4);
        for (unsigned i = 0; i < n; i++) {
            p_le16(&p, pick_attr_handle());
        }
        break;
    }
    case 0x12:
    case 0x52:
    case 0xD2:
        if (chance(30) && pick_cccd()) {
            /* Enable notifications / indications (starts HOGP reporting). */
            p_le16(&p, pick_cccd());
            p_u8(&p, (uint8_t)(chance(80) ? 1 + below(3) : rnd()));
            p_u8(&p, 0);
            break;
        }
        p_le16(&p, start);
        p_rand(&p, (uint16_t)below(chance(10) ? 200 : 8));
        break;
    case 0x16: {
        p_le16(&p, start);
        p_le16(&p, chance(60) ? (uint16_t)(below(10) * 18) : interesting16());
        p_rand(&p, (uint16_t)below(chance(10) ? 200 : 19));
        break;
    }
    case 0x18:
        p_u8(&p, (uint8_t)(chance(80) ? below(2) : rnd()));
        break;
    default:
        p_rand(&p, (uint16_t)below(30));
        break;
    }
    if (chance(15)) {
        mutate(&p, 300);
    }
    if (p.n == 0) {
        p_u8(&p, op);
    }
    (void)floss_sim_att_send(g.le, p.b, p.n);
    if (chance(20)) {
        /* A burst of prepare writes against one handle, then execute. */
        uint16_t h = pick_attr_handle();
        unsigned n = 1 + below(6);
        for (unsigned i = 0; i < n; i++) {
            pkt_t w;
            p_reset(&w);
            p_u8(&w, 0x16);
            p_le16(&w, h);
            p_le16(&w, chance(70) ? (uint16_t)(i * 18) : interesting16());
            p_rand(&w, (uint16_t)below(19));
            (void)floss_sim_att_send(g.le, w.b, w.n);
            floss_sim_pump(1);
        }
        const uint8_t ex[2] = {0x18, (uint8_t)below(2)};
        (void)floss_sim_att_send(g.le, ex, 2);
    }
    uint8_t rsp[600];
    while (floss_sim_att_recv(g.le, rsp, sizeof rsp) >= 0) {
    }
    return true;
}

TEST(fuzz_att) {
    fz_run("att", false, true, step_att);
}

/* ======================================================================== */
/* Target: SMP                                                              */
/* ======================================================================== */

static const uint8_t k_smp_len[] = {
    /* code: expected PDU length including the code byte */
    0, 7, 7, 17, 17, 2, 17, 11, 17, 8, 17, 2, 65, 17, 2,
};

static bool step_smp(void) {
    if (!link_alive(g.le)) {
        return false;
    }
    pkt_t p;
    p_reset(&p);
    if (chance(8)) {
        /* Misbehaving controller: LE crypto events out of the blue. */
        pkt_t e;
        p_reset(&e);
        p_u8(&e, FLOSS_EVT_LE_META);
        p_u8(&e, 0);
        switch (below(4)) {
        case 0: p_u8(&e, 0x05); p_le16(&e, g.le); p_rand(&e, 10); break;   /* LTK request */
        case 1: p_u8(&e, 0x08); p_u8(&e, (uint8_t)(chance(80) ? 0 : rnd())); p_rand(&e, 64); break;
        case 2: p_u8(&e, 0x09); p_u8(&e, (uint8_t)(chance(80) ? 0 : rnd())); p_rand(&e, 32); break;
        default:
            p_reset(&e);
            p_u8(&e, FLOSS_EVT_ENCRYPTION_CHANGE);
            p_u8(&e, 4);
            p_u8(&e, (uint8_t)(chance(70) ? 0 : rnd()));
            p_le16(&e, g.le);
            p_u8(&e, (uint8_t)below(3));
            break;
        }
        if (e.b[0] == FLOSS_EVT_LE_META) {
            e.b[1] = (uint8_t)(e.n - 2);
        }
        if (chance(20)) {
            e.n = (uint16_t)(2 + below(e.n - 1u));
            e.b[1] = (uint8_t)(e.n - 2);
        }
        inject_event(e.b, e.n);
        return true;
    }
    uint8_t code = chance(90) ? (uint8_t)(1 + below(14)) : (uint8_t)rnd();
    p_u8(&p, code);
    if (code == 0x01 || code == 0x02) {
        p_u8(&p, (uint8_t)(chance(80) ? below(5) : rnd()));      /* IO capability */
        p_u8(&p, (uint8_t)(chance(80) ? 0 : rnd()));             /* OOB */
        p_u8(&p, (uint8_t)(chance(80) ? (0x01 | (below(2) << 3) | (below(2) << 2)) : rnd())); /* AuthReq */
        p_u8(&p, (uint8_t)(chance(80) ? 7 + below(10) : rnd()));  /* max key size */
        p_u8(&p, (uint8_t)(chance(80) ? below(8) : rnd()));      /* initiator keys */
        p_u8(&p, (uint8_t)(chance(80) ? below(8) : rnd()));      /* responder keys */
    } else if (code < sizeof k_smp_len && k_smp_len[code]) {
        p_rand(&p, (uint16_t)(k_smp_len[code] - 1));
        if (code == 0x09 && chance(50)) {
            p.b[1] = (uint8_t)below(2);
            memcpy(&p.b[2], floss_tc_host.b, 6);
        }
    } else {
        p_rand(&p, (uint16_t)below(70));
    }
    if (chance(12)) {
        mutate(&p, 80);
    }
    if (p.n == 0) {
        p_u8(&p, code);
    }
    (void)floss_sim_fixed_send(g.le, FLOSS_SIM_CID_SMP, p.b, p.n);
    uint8_t rsp[100];
    while (floss_sim_fixed_recv(g.le, FLOSS_SIM_CID_SMP, rsp, sizeof rsp) >= 0) {
    }
    return true;
}

TEST(fuzz_smp) {
    fz_run("smp", false, true, step_smp);
}

/* ======================================================================== */
/* Target: RFCOMM                                                           */
/* ======================================================================== */

static uint8_t pick_dlci(void) {
    switch (below(6)) {
    case 0:
        return 0;
    case 1:
        return (uint8_t)below(64);
    case 2:
        return (uint8_t)(FZ_RFC_DLCI ^ 1);
    default:
        return FZ_RFC_DLCI;
    }
}

static bool step_rfcomm(void) {
    if (!g.rfc || g.rfc->state != FLOSS_SIM_CHAN_OPEN) {
        return false;
    }
    pkt_t p;
    uint8_t v[300];
    unsigned kind = below(10);
    if (kind < 4) {
        /* MCC command / response on DLCI 0. */
        static const uint8_t types[] = {RFC_MCC_PN, RFC_MCC_MSC, RFC_MCC_RPN, RFC_MCC_RLS, RFC_MCC_TEST,
                                        RFC_MCC_FCON, RFC_MCC_FCOFF, RFC_MCC_NSC, RFC_MCC_PSC, RFC_MCC_CLD};
        uint8_t type = chance(90) ? types[below(sizeof types)] : (uint8_t)(rnd() & 0xFC);
        uint16_t vlen;
        switch (type) {
        case RFC_MCC_PN:
            vlen = 8;
            v[0] = pick_dlci();
            v[1] = (uint8_t)(chance(70) ? 0xF0 : rnd());
            v[2] = (uint8_t)rnd();
            v[3] = 0;
            floss_put_le16(&v[4], interesting16());
            v[6] = 0;
            v[7] = (uint8_t)rnd();
            break;
        case RFC_MCC_MSC:
            vlen = (uint16_t)(chance(80) ? 2 + below(2) : below(200));
            rnd_bytes(v, vlen);
            v[0] = (uint8_t)(RFC_EA | RFC_CR | (pick_dlci() << 2));
            break;
        case RFC_MCC_RPN:
            vlen = (uint16_t)(chance(50) ? 1 : (chance(80) ? 8 : below(20)));
            rnd_bytes(v, vlen);
            if (vlen) {
                v[0] = (uint8_t)(RFC_EA | RFC_CR | (pick_dlci() << 2));
            }
            break;
        case RFC_MCC_TEST:
            vlen = (uint16_t)below(chance(20) ? 300 : 20);
            rnd_bytes(v, vlen);
            break;
        default:
            vlen = (uint16_t)below(chance(10) ? 200 : 4);
            rnd_bytes(v, vlen);
            break;
        }
        rfc_mcc(&p, type, chance(85), v, vlen);
        if (chance(10)) {
            /* Several messages in one frame / broken EA bits. */
            mutate(&p, 600);
        }
    } else if (kind < 7) {
        /* UIH data on the DLC, with or without a credit byte. */
        uint16_t len = (uint16_t)below(chance(15) ? 660 : 40);
        rnd_bytes(v, (uint16_t)FLOSS_MIN(len, (uint16_t)sizeof v));
        bool pf = chance(30);
        uint8_t info[700];
        for (uint16_t i = 0; i < len; i++) {
            info[i] = v[i % sizeof v];
        }
        rfc_build(&p, pick_dlci(), RFC_UIH, pf, info, len, pf ? (int)(chance(80) ? below(8) : 255) : -1);
    } else {
        static const uint8_t types[] = {RFC_SABM, RFC_UA, RFC_DM, RFC_DISC, RFC_UIH};
        uint8_t type = chance(90) ? types[below(sizeof types)] : (uint8_t)rnd();
        uint8_t dlci = pick_dlci();
        if (type == RFC_DISC && dlci == 0 && chance(70)) {
            dlci = FZ_RFC_DLCI; /* closing the whole session ends the round too early */
        }
        rfc_build(&p, dlci, type, chance(70), NULL, 0, -1);
    }
    if (chance(10) && p.n > 0) {
        p.b[p.n - 1] ^= (uint8_t)(1 + below(255)); /* bad FCS */
    }
    if (chance(8)) {
        mutate(&p, 670);
    }
    rfc_send(&p);
    uint8_t rsp[1100];
    while (floss_sim_l2cap_recv(g.rfc, rsp, sizeof rsp) >= 0) {
    }
    return true;
}

TEST(fuzz_rfcomm) {
    fz_run("rfcomm", true, false, step_rfcomm);
}

/* ======================================================================== */
/* Target: HIDP                                                             */
/* ======================================================================== */

static bool step_hidp(void) {
    bool ctrl = chance(60);
    floss_sim_chan_t *ch = ctrl ? g.ctrl : g.intr;
    if (!ch || ch->state != FLOSS_SIM_CHAN_OPEN) {
        return false;
    }
    pkt_t p;
    p_reset(&p);
    static const uint8_t hdrs[] = {0x00, 0x01, 0x03, 0x04, 0x05, 0x11, 0x13, 0x14, 0x15, 0x41, 0x42, 0x43,
                                   0x49, 0x4A, 0x4B, 0x51, 0x52, 0x53, 0x60, 0x70, 0x71, 0xA1, 0xA2, 0xA3,
                                   0x80, 0x90, 0xB0, 0xC0};
    uint8_t hdr = chance(90) ? hdrs[below(sizeof hdrs)] : (uint8_t)rnd();
    if (hdr == 0x15 && chance(90)) {
        hdr = 0x13; /* virtual cable unplug ends the round; keep it rare */
    }
    p_u8(&p, hdr);
    switch (hdr & 0xF0) {
    case 0x40: /* GET_REPORT: [id] [max size le16] */
        if (chance(80)) {
            p_u8(&p, (uint8_t)below(5));
        }
        if (hdr & 0x08) {
            p_le16(&p, interesting16());
        }
        break;
    case 0x50: /* SET_REPORT */
    case 0xA0: /* DATA */
        p_u8(&p, (uint8_t)below(5));
        p_rand(&p, (uint16_t)below(chance(10) ? 670 : 10));
        break;
    default:
        p_rand(&p, (uint16_t)below(chance(10) ? 100 : 3));
        break;
    }
    if (chance(10)) {
        mutate(&p, 672);
    }
    if (p.n > ch->stack_mtu) {
        p.n = ch->stack_mtu;
    }
    (void)floss_sim_l2cap_send(ch, p.b, p.n);
    uint8_t rsp[1100];
    while (floss_sim_l2cap_recv(g.ctrl, rsp, sizeof rsp) >= 0) {
    }
    while (floss_sim_l2cap_recv(g.intr, rsp, sizeof rsp) >= 0) {
    }
    return true;
}

TEST(fuzz_hidp) {
    fz_run("hidp", true, false, step_hidp);
}

/* ======================================================================== */
/* Target: H4 framing                                                       */
/* ======================================================================== */

/*
 * The H4 driver alone, fed a byte stream of valid, truncated, oversized and
 * garbage frames in random read sizes. Every delivered packet must be
 * self-consistent (its header length matches what was delivered).
 */
static struct {
    uint8_t data[4096];
    uint16_t len;
    uint16_t pos;
} g_h4s;
static unsigned g_h4_delivered;

static int h4s_read(void *ctx, uint8_t *buf, uint16_t max) {
    (void)ctx;
    uint16_t left = (uint16_t)(g_h4s.len - g_h4s.pos);
    uint32_t want = 1 + below(chance(20) ? 300 : 8);
    uint16_t n = (uint16_t)FLOSS_MIN((uint32_t)FLOSS_MIN(left, max), want);
    memcpy(buf, &g_h4s.data[g_h4s.pos], n);
    g_h4s.pos = (uint16_t)(g_h4s.pos + n);
    return n;
}

static int h4s_write(void *ctx, const uint8_t *data, uint16_t len) {
    (void)ctx;
    (void)data;
    return len;
}

static void h4s_on_rx(void *user, uint8_t type, const uint8_t *pkt, uint16_t len) {
    (void)user;
    unsigned sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += pkt[i]; /* ASan: the whole packet must be readable */
    }
    (void)sum;
    uint32_t expect;
    switch (type) {
    case FLOSS_PKT_EVT: expect = len >= 2 ? 2u + pkt[1] : 0xFFFFFFFFu; break;
    case FLOSS_PKT_ACL: expect = len >= 4 ? 4u + floss_get_le16(pkt + 2) : 0xFFFFFFFFu; break;
    case FLOSS_PKT_SCO: expect = len >= 3 ? 3u + pkt[2] : 0xFFFFFFFFu; break;
    case FLOSS_PKT_ISO: expect = len >= 4 ? 4u + (floss_get_le16(pkt + 2) & 0x3FFF) : 0xFFFFFFFFu; break;
    default: FAIL("h4 delivered packet type 0x%02x", type);
    }
    if (expect != len) {
        FAIL("h4 delivered type 0x%02x with %u bytes, header says %lu", type, len, (unsigned long)expect);
    }
    g_h4_delivered++;
}

TEST(fuzz_h4) {
    rng_seed("h4");
    floss_test_hal_reset();
    floss_test_log_quiet(!g_verbose);
    static floss_h4_t h4;
    floss_stream_t stream = {.ctx = NULL, .write = h4s_write, .read = h4s_read};
    floss_transport_t *t = floss_h4_init(&h4, &stream);
    t->rx = h4s_on_rx;
    t->rx_user = NULL;
    ASSERT_EQ(t->ops->open(t), FLOSS_OK);
    for (uint32_t i = 0; i < g_iters; i++) {
        pkt_t p;
        p_reset(&p);
        unsigned frames = 1 + below(6);
        for (unsigned f = 0; f < frames; f++) {
            static const uint8_t types[] = {FLOSS_PKT_EVT, FLOSS_PKT_ACL, FLOSS_PKT_SCO, FLOSS_PKT_ISO};
            uint8_t type = chance(90) ? types[below(4)] : (uint8_t)rnd();
            p_u8(&p, type);
            uint16_t plen = (uint16_t)below(chance(10) ? 1100 : 64);
            switch (type) {
            case FLOSS_PKT_EVT: p_u8(&p, (uint8_t)rnd()); p_u8(&p, (uint8_t)plen); plen = (uint8_t)plen; break;
            case FLOSS_PKT_ACL: p_le16(&p, (uint16_t)rnd()); p_le16(&p, chance(95) ? plen : interesting16()); break;
            case FLOSS_PKT_SCO: p_le16(&p, (uint16_t)rnd()); p_u8(&p, (uint8_t)plen); plen = (uint8_t)plen; break;
            case FLOSS_PKT_ISO: p_le16(&p, (uint16_t)rnd()); p_le16(&p, chance(95) ? plen : interesting16()); break;
            default: break;
            }
            p_rand(&p, plen);
        }
        if (chance(20)) {
            mutate(&p, PKT_MAX);
        }
        memcpy(g_h4s.data, p.b, p.n);
        g_h4s.len = p.n;
        g_h4s.pos = 0;
        for (unsigned k = 0; k < 2000 && g_h4s.pos < g_h4s.len; k++) {
            t->ops->poll(t);
        }
        if (g_h4s.pos < g_h4s.len) {
            FAIL("h4 stopped reading its stream (%u of %u bytes)", g_h4s.pos, g_h4s.len);
        }
    }
    printf("    h4: %u streams, %u packets delivered (seed %u)\n", (unsigned)g_iters, g_h4_delivered,
           (unsigned)g_seed);
}

/* ======================================================================== */
/* Runner                                                                   */
/* ======================================================================== */

static const floss_test_case_t tests[] = {
    TEST_CASE(fuzz_hci_events), TEST_CASE(fuzz_acl),    TEST_CASE(fuzz_l2cap_signaling), TEST_CASE(fuzz_sdp),
    TEST_CASE(fuzz_att),        TEST_CASE(fuzz_smp),    TEST_CASE(fuzz_rfcomm),          TEST_CASE(fuzz_hidp),
    TEST_CASE(fuzz_h4),
};

int main(int argc, char **argv) {
    char *args[2] = {argv[0], NULL};
    int nargs = 1;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
            g_seed = (uint32_t)strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "--iters") == 0 && i + 1 < argc) {
            g_iters = (uint32_t)strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "--verbose") == 0) {
            g_verbose = true;
        } else if (nargs < 2) {
            args[nargs++] = argv[i];
        }
    }
    printf("floss_fuzz: seed %u, %u packets per target\n", (unsigned)g_seed, (unsigned)g_iters);
    return floss_test_main("fuzz", tests, sizeof tests / sizeof tests[0], nargs, args);
}
