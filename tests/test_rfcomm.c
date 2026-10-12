/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * test_rfcomm.c - RFCOMM client role (the stack initiates the session and
 * the DLC) against a responder scripted in the test, plus newest-only send
 * gating (floss_hci_acl_can_send_fresh()).
 *
 * The stack opens L2CAP PSM 3 to the sim (which accepts every PSM); the test
 * then plays the responding host frame by frame, the way a PC answers a
 * device that connects to one of its serial-style services:
 *
 *   stack                              host (test)
 *   SABM DLCI 0           ------>
 *                         <------      UA DLCI 0
 *   UIH PN (CFC offered)  ------>
 *                         <------      UIH PN response (CFC accepted, N1, credits)
 *   SABM DLCI n           ------>
 *                         <------      UA DLCI n
 *   UIH MSC               <------>     UIH MSC (both directions, both answered)
 *   UIH data + credits    <------>     UIH data + credits
 *   DISC DLCI n / DLCI 0  ------>
 *                         <------      UA, then the stack closes L2CAP
 *
 * Responder frame rules (TS 07.10 as used by RFCOMM): commands from the
 * responder carry C/R = 0 in the address octet, its responses C/R = 1;
 * inside multiplexer control messages C/R = 1 marks a command.
 */
#include "test_common.h"

#include "floss_internal.h"
#include "floss/floss_rfcomm.h"
#include "rfcomm/floss_rfcomm_int.h"

#if FLOSS_ENABLE_RFCOMM

#define RC_CHANNEL 7
#define RC_DLCI    (RC_CHANNEL << 1) /* the initiator addresses the responder's channel with D = 0 */

/** What the DLC callbacks saw. */
static struct {
    unsigned opens;
    uint8_t open_status;
    unsigned closes;
    unsigned can_send;
    unsigned frames;
    uint16_t bytes;
    uint8_t last[64];
    uint16_t last_len;
} g_rc;

static void rc_on_open(floss_rfcomm_dlc_t *dlc, uint8_t status) {
    g_rc.opens++;
    g_rc.open_status = status;
}

static void rc_on_data(floss_rfcomm_dlc_t *dlc, const uint8_t *data, uint16_t len) {
    g_rc.frames++;
    g_rc.bytes = (uint16_t)(g_rc.bytes + len);
    g_rc.last_len = FLOSS_MIN(len, (uint16_t)sizeof g_rc.last);
    memcpy(g_rc.last, data, g_rc.last_len);
}

static void rc_on_close(floss_rfcomm_dlc_t *dlc) {
    g_rc.closes++;
}

static void rc_on_can_send(floss_rfcomm_dlc_t *dlc) {
    g_rc.can_send++;
}

static const floss_rfcomm_ops_t k_rc_ops = {
    .on_open = rc_on_open,
    .on_data = rc_on_data,
    .on_close = rc_on_close,
    .on_can_send = rc_on_can_send,
};

/* ------------------------------------------------------------------------ */
/* Responder side                                                           */
/* ------------------------------------------------------------------------ */

/** One frame from the stack, decoded and FCS-checked. */
typedef struct {
    uint8_t dlci;
    uint8_t type;      /**< control octet without P/F */
    bool pf;
    bool cr;           /**< address C/R bit */
    int credits;       /**< credit byte, -1 if none */
    const uint8_t *info;
    uint16_t len;
    uint8_t raw[FLOSS_SIM_ACL_MAX];
} rc_frame_t;

static void rc_recv(floss_sim_chan_t *ch, rc_frame_t *f) {
    int n = floss_sim_l2cap_recv_wait(ch, f->raw, sizeof f->raw);
    if (n < 4) {
        FAIL("no RFCOMM frame from the stack (got %d)", n);
    }
    const uint8_t *p = f->raw;
    ASSERT_TRUE(p[0] & RFC_EA);
    f->dlci = (uint8_t)(p[0] >> 2);
    f->cr = (p[0] & RFC_CR) != 0;
    f->type = (uint8_t)(p[1] & ~RFC_PF);
    f->pf = (p[1] & RFC_PF) != 0;
    uint16_t hl;
    if (p[2] & RFC_EA) {
        f->len = (uint16_t)(p[2] >> 1);
        hl = 3;
    } else {
        f->len = (uint16_t)((p[2] >> 1) | (p[3] << 7));
        hl = 4;
    }
    uint8_t fcs = floss_rfcomm_fcs(p, f->type == RFC_UIH ? 2 : hl);
    f->credits = -1;
    if (f->type == RFC_UIH && f->pf && f->dlci != 0) {
        f->credits = p[hl++];
    }
    ASSERT_EQ((int)hl + f->len + 1, n);
    ASSERT_EQ(p[hl + f->len], fcs);
    f->info = p + hl;
}

/**
 * Next frame that is not a bare credit grant on the DLC; credits granted on
 * the way (bare or piggybacked) are added to *credits.
 */
static void rc_next(floss_sim_chan_t *ch, rc_frame_t *f, unsigned *credits) {
    for (unsigned i = 0; i < 16; i++) {
        rc_recv(ch, f);
        if (f->credits > 0 && credits != NULL) {
            *credits += (unsigned)f->credits;
        }
        if (!(f->type == RFC_UIH && f->dlci != 0 && f->len == 0)) {
            return;
        }
    }
    FAIL("only credit frames from the stack");
}

/** Send one frame as the responder. credits >= 0 adds a credit byte (UIH with P/F). */
static void rc_send(floss_sim_chan_t *ch, uint8_t dlci, uint8_t type, int credits, const uint8_t *info,
                    uint16_t len) {
    uint8_t f[300];
    uint16_t n = 0;
    bool command = type != RFC_UA && type != RFC_DM;
    bool pf = type != RFC_UIH || credits >= 0;
    f[n++] = (uint8_t)(RFC_EA | (command ? 0 : RFC_CR) | (dlci << 2));
    f[n++] = (uint8_t)(type | (pf ? RFC_PF : 0));
    if (len <= 127) {
        f[n++] = (uint8_t)((len << 1) | RFC_EA);
    } else {
        f[n++] = (uint8_t)(len << 1);
        f[n++] = (uint8_t)(len >> 7);
    }
    uint8_t fcs = floss_rfcomm_fcs(f, type == RFC_UIH ? 2 : n);
    if (credits >= 0) {
        f[n++] = (uint8_t)credits;
    }
    if (len) {
        memcpy(f + n, info, len);
    }
    n = (uint16_t)(n + len);
    f[n++] = fcs;
    ASSERT_EQ(floss_sim_l2cap_send(ch, f, n), FLOSS_OK);
    floss_sim_pump(10);
}

/** Multiplexer control message on DLCI 0 (short form, value <= 127 bytes). */
static void rc_send_mcc(floss_sim_chan_t *ch, uint8_t type, bool command, const uint8_t *v, uint8_t vlen) {
    uint8_t info[2 + 127];
    info[0] = (uint8_t)(type | (command ? RFC_CR : 0) | RFC_EA);
    info[1] = (uint8_t)((vlen << 1) | RFC_EA);
    memcpy(info + 2, v, vlen);
    rc_send(ch, 0, RFC_UIH, -1, info, (uint16_t)(2 + vlen));
}

static bool cond_rfcomm_chan_open(void *arg) {
    const floss_sim_chan_t *c = floss_sim_chan_find((uint16_t)(uintptr_t)arg, FLOSS_PSM_RFCOMM);
    return c != NULL && c->state == FLOSS_SIM_CHAN_OPEN;
}

/** Paired link, floss_rfcomm_connect() issued, L2CAP PSM 3 open on the sim. */
static floss_sim_chan_t *rc_start(uint16_t *handle, floss_rfcomm_dlc_t **dlc) {
    memset(&g_rc, 0, sizeof g_rc);
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_stack_start(t, NULL);
    uint16_t h = floss_tc_host_connect();
    floss_tc_host_pair(h);
    ASSERT_EQ(floss_rfcomm_connect(h, RC_CHANNEL, &k_rc_ops, NULL, dlc), FLOSS_OK);
    ASSERT_NOT_NULL(*dlc);
    if (!floss_sim_pump_until(cond_rfcomm_chan_open, (void *)(uintptr_t)h, 500)) {
        FAIL("the stack never opened L2CAP PSM 3");
    }
    floss_sim_chan_t *ch = floss_sim_chan_find(h, FLOSS_PSM_RFCOMM);
    ASSERT_FALSE(ch->sim_initiated);
    *handle = h;
    return ch;
}

/** Session start: SABM 0 / UA 0. */
static void rc_accept_session(floss_sim_chan_t *ch) {
    rc_frame_t f;
    rc_next(ch, &f, NULL);
    ASSERT_EQ(f.dlci, 0);
    ASSERT_EQ(f.type, RFC_SABM);
    ASSERT_TRUE(f.pf);
    ASSERT_TRUE(f.cr); /* command from the initiator */
    rc_send(ch, 0, RFC_UA, -1, NULL, 0);
}

/** PN exchange for RC_DLCI: answer with CFC, N1 = n1 and `credits` initial credits. */
static void rc_answer_pn(floss_sim_chan_t *ch, uint16_t n1, uint8_t credits, uint16_t *offered_n1) {
    rc_frame_t f;
    rc_next(ch, &f, NULL);
    ASSERT_EQ(f.dlci, 0);
    ASSERT_EQ(f.type, RFC_UIH);
    ASSERT_EQ(f.len, 10);
    ASSERT_EQ(f.info[0], RFC_MCC_PN | RFC_CR | RFC_EA); /* PN command */
    ASSERT_EQ(f.info[1], (8 << 1) | RFC_EA);
    const uint8_t *v = f.info + 2;
    ASSERT_EQ(v[0], RC_DLCI);
    ASSERT_EQ(v[1], RFC_PN_CL_CFC_REQ); /* UIH frames, credit based flow control offered */
    ASSERT_EQ(v[7], FLOSS_MIN(FLOSS_RFCOMM_RX_CREDITS, 7));
    *offered_n1 = floss_get_le16(v + 4);

    uint8_t rsp[8] = {RC_DLCI, RFC_PN_CL_CFC_RSP, v[2], 0, 0, 0, 0, credits};
    floss_put_le16(rsp + 4, n1);
    rc_send_mcc(ch, RFC_MCC_PN, false, rsp, sizeof rsp);
}

/* ------------------------------------------------------------------------ */
/* Tests                                                                    */
/* ------------------------------------------------------------------------ */

/* The whole client life cycle, with credits both ways. */
TEST(rfcomm_client_full_session) {
    uint16_t h;
    floss_rfcomm_dlc_t *dlc;
    floss_sim_chan_t *ch = rc_start(&h, &dlc);
    rc_frame_t f;
    unsigned granted = 0; /* credits the stack handed to the host */

    rc_accept_session(ch);
    uint16_t offered;
    rc_answer_pn(ch, 100, 5, &offered);
    ASSERT_GE(offered, 100); /* the stack offers what its L2CAP channel carries */
    granted += FLOSS_MIN(FLOSS_RFCOMM_RX_CREDITS, 7);

    /* DLC start. */
    rc_next(ch, &f, &granted);
    ASSERT_EQ(f.dlci, RC_DLCI);
    ASSERT_EQ(f.type, RFC_SABM);
    ASSERT_TRUE(f.cr);
    ASSERT_EQ(g_rc.opens, 0);
    rc_send(ch, RC_DLCI, RFC_UA, -1, NULL, 0);
    ASSERT_EQ(g_rc.opens, 1);
    ASSERT_EQ(g_rc.open_status, FLOSS_RFCOMM_STATUS_OK);
    ASSERT_EQ(dlc->state, FLOSS_RFCOMM_DLC_OPEN);
    ASSERT_TRUE(dlc->cfc);
    ASSERT_EQ(dlc->mtu, 100);
    ASSERT_EQ(dlc->tx_credits, 5);
    ASSERT_EQ(dlc->channel, RC_CHANNEL);

    /* MSC: the stack's command, our response; our command, its response. */
    rc_next(ch, &f, &granted);
    ASSERT_EQ(f.dlci, 0);
    ASSERT_EQ(f.len, 4);
    ASSERT_EQ(f.info[0], RFC_MCC_MSC | RFC_CR | RFC_EA);
    ASSERT_EQ(f.info[2] >> 2, RC_DLCI);
    ASSERT_EQ(f.info[3] & ~RFC_EA, FLOSS_RFCOMM_V24_DEFAULT);
    rc_send_mcc(ch, RFC_MCC_MSC, false, f.info + 2, 2);
    const uint8_t msc[2] = {(uint8_t)(RFC_EA | RFC_CR | (RC_DLCI << 2)), (uint8_t)(FLOSS_RFCOMM_V24_DEFAULT | RFC_EA)};
    rc_send_mcc(ch, RFC_MCC_MSC, true, msc, sizeof msc);
    rc_next(ch, &f, &granted);
    ASSERT_EQ(f.dlci, 0);
    ASSERT_EQ(f.info[0], RFC_MCC_MSC | RFC_EA); /* response */
    ASSERT_EQ(dlc->remote_v24, FLOSS_RFCOMM_V24_DEFAULT | RFC_EA);

    /* Stack -> host: one frame per credit, then BUSY until the host grants more. */
    const uint8_t hello[5] = {'h', 'e', 'l', 'l', 'o'};
    for (unsigned i = 0; i < 5; i++) {
        ASSERT_TRUE(floss_rfcomm_can_send_now(dlc));
        ASSERT_EQ(floss_rfcomm_send(dlc, hello, sizeof hello), (int)sizeof hello);
        floss_sim_pump(5);
        rc_next(ch, &f, &granted);
        ASSERT_EQ(f.dlci, RC_DLCI);
        ASSERT_EQ(f.type, RFC_UIH);
        ASSERT_TRUE(f.cr); /* data from the initiator */
        ASSERT_EQ(f.len, sizeof hello);
        ASSERT_MEM_EQ(f.info, hello, sizeof hello);
    }
    ASSERT_EQ(dlc->tx_credits, 0);
    ASSERT_FALSE(floss_rfcomm_can_send_now(dlc));
    ASSERT_EQ(floss_rfcomm_send(dlc, hello, sizeof hello), FLOSS_ERR_BUSY);
    rc_send(ch, RC_DLCI, RFC_UIH, 3, NULL, 0); /* bare credit grant */
    ASSERT_EQ(dlc->tx_credits, 3);
    ASSERT_GE(g_rc.can_send, 1);
    ASSERT_EQ(floss_rfcomm_send(dlc, hello, 3), 3);
    rc_next(ch, &f, &granted);
    ASSERT_EQ(f.len, 3);

    /* Host -> stack: the stack's initial credits, refilled automatically. */
    const uint8_t world[5] = {'w', 'o', 'r', 'l', 'd'};
    unsigned host_credits = granted;
    for (unsigned i = 0; i < 12; i++) {
        if (host_credits == 0) {
            rc_recv(ch, &f); /* wait for a refill */
            ASSERT_EQ(f.dlci, RC_DLCI);
            ASSERT_GT(f.credits, 0);
            host_credits += (unsigned)f.credits;
        }
        rc_send(ch, RC_DLCI, RFC_UIH, -1, world, sizeof world);
        host_credits--;
        while (floss_sim_l2cap_pending(ch) > 0) {
            rc_recv(ch, &f);
            ASSERT_EQ(f.dlci, RC_DLCI);
            ASSERT_EQ(f.len, 0); /* nothing but credits */
            ASSERT_GT(f.credits, 0);
            host_credits += (unsigned)f.credits;
        }
    }
    ASSERT_EQ(g_rc.frames, 12);
    ASSERT_EQ(g_rc.bytes, 12 * sizeof world);
    ASSERT_MEM_EQ(g_rc.last, world, sizeof world);

    /* Close: DISC on the DLC, then the stack shuts the session and L2CAP down. */
    ASSERT_EQ(floss_rfcomm_disconnect(dlc), FLOSS_OK);
    rc_next(ch, &f, NULL);
    ASSERT_EQ(f.dlci, RC_DLCI);
    ASSERT_EQ(f.type, RFC_DISC);
    rc_send(ch, RC_DLCI, RFC_UA, -1, NULL, 0);
    ASSERT_EQ(g_rc.closes, 1);
    rc_next(ch, &f, NULL);
    ASSERT_EQ(f.dlci, 0);
    ASSERT_EQ(f.type, RFC_DISC);
    rc_send(ch, 0, RFC_UA, -1, NULL, 0);
    floss_sim_pump(20);
    ASSERT_EQ(ch->state, FLOSS_SIM_CHAN_CLOSED);
    ASSERT_NULL(floss_l2cap_find(h, FLOSS_PSM_RFCOMM));
    ASSERT_EQ(g_rc.opens, 1);
    ASSERT_EQ(g_rc.closes, 1);
    floss_tc_assert_sim_clean();
}

/* The host has no service on that channel: DM after PN -> REFUSED, session closed. */
TEST(rfcomm_client_refused_by_dm) {
    uint16_t h;
    floss_rfcomm_dlc_t *dlc;
    floss_sim_chan_t *ch = rc_start(&h, &dlc);
    rc_frame_t f;
    rc_accept_session(ch);
    rc_next(ch, &f, NULL);
    ASSERT_EQ(f.info[0], RFC_MCC_PN | RFC_CR | RFC_EA);
    rc_send(ch, RC_DLCI, RFC_DM, -1, NULL, 0);
    ASSERT_EQ(g_rc.opens, 1);
    ASSERT_EQ(g_rc.open_status, FLOSS_RFCOMM_STATUS_REFUSED);
    ASSERT_EQ(g_rc.closes, 0);
    rc_next(ch, &f, NULL);
    ASSERT_EQ(f.dlci, 0);
    ASSERT_EQ(f.type, RFC_DISC);
    rc_send(ch, 0, RFC_UA, -1, NULL, 0);
    floss_sim_pump(20);
    ASSERT_EQ(ch->state, FLOSS_SIM_CHAN_CLOSED);
    floss_tc_assert_sim_clean();
}

/* The host refuses the session itself (DM on DLCI 0). */
TEST(rfcomm_client_session_refused) {
    uint16_t h;
    floss_rfcomm_dlc_t *dlc;
    floss_sim_chan_t *ch = rc_start(&h, &dlc);
    rc_frame_t f;
    rc_next(ch, &f, NULL);
    ASSERT_EQ(f.type, RFC_SABM);
    rc_send(ch, 0, RFC_DM, -1, NULL, 0);
    floss_sim_pump(20);
    ASSERT_EQ(g_rc.opens, 1);
    ASSERT_EQ(g_rc.open_status, FLOSS_RFCOMM_STATUS_NO_MUX);
    ASSERT_EQ(ch->state, FLOSS_SIM_CHAN_CLOSED);
    floss_tc_assert_sim_clean();
}

/* The link drops with the DLC open: on_close once. */
TEST(rfcomm_client_link_loss) {
    uint16_t h;
    floss_rfcomm_dlc_t *dlc;
    floss_sim_chan_t *ch = rc_start(&h, &dlc);
    rc_frame_t f;
    uint16_t offered;
    rc_accept_session(ch);
    rc_answer_pn(ch, 100, 5, &offered);
    rc_next(ch, &f, NULL);
    ASSERT_EQ(f.type, RFC_SABM);
    rc_send(ch, RC_DLCI, RFC_UA, -1, NULL, 0);
    ASSERT_EQ(g_rc.opens, 1);
    ASSERT_EQ(floss_sim_remote_disconnect(h, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED), FLOSS_OK);
    floss_sim_pump(50);
    ASSERT_EQ(g_rc.closes, 1);
    ASSERT_EQ(g_rc.opens, 1);
    floss_tc_assert_sim_clean();
}

/* ------------------------------------------------------------------------ */
/* Newest-only send gating                                                  */
/* ------------------------------------------------------------------------ */

static bool cond_link_idle(void *arg) {
    const floss_conn_t *c = floss_hci_conn_by_handle((uint16_t)(uintptr_t)arg);
    return c != NULL && c->inflight == 0 && c->txq.count == 0;
}

/* floss_hci_acl_can_send_fresh(): open link, nothing queued, in-flight cap. */
TEST(acl_can_send_fresh_gates_on_inflight) {
    uint16_t h;
    floss_rfcomm_dlc_t *dlc;
    floss_sim_chan_t *ch = rc_start(&h, &dlc);
    rc_frame_t f;
    uint16_t offered;
    rc_accept_session(ch);
    rc_answer_pn(ch, 100, 7, &offered);
    rc_next(ch, &f, NULL);
    rc_send(ch, RC_DLCI, RFC_UA, -1, NULL, 0);
    ASSERT_EQ(g_rc.open_status, FLOSS_RFCOMM_STATUS_OK);
    ASSERT_TRUE(floss_sim_pump_until(cond_link_idle, (void *)(uintptr_t)h, 200));

    ASSERT_FALSE(floss_hci_acl_can_send_fresh(0x0EEE, 1)); /* no such link */
    ASSERT_TRUE(floss_hci_acl_can_send_fresh(h, 1));
    ASSERT_TRUE(floss_hci_acl_can_send_fresh(h, 0));

    /* Completions now take 20 ms of air time: sent frames stay in flight. */
    floss_sim_cfg()->nocp_delay_us = 20000;
    const uint8_t sample[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    ASSERT_EQ(floss_rfcomm_send(dlc, sample, sizeof sample), (int)sizeof sample);
    const floss_conn_t *c = floss_hci_conn_by_handle(h);
    ASSERT_EQ(c->inflight, 1);
    ASSERT_FALSE(floss_hci_acl_can_send_fresh(h, 1)); /* one in flight = the cap */
    ASSERT_TRUE(floss_hci_acl_can_send_fresh(h, 2));
    ASSERT_TRUE(floss_hci_acl_can_send_fresh(h, 0)); /* no cap: same as can_send_now */
    ASSERT_EQ(floss_hci_acl_can_send_fresh(h, 0), floss_hci_acl_can_send_now(h));

    ASSERT_EQ(floss_rfcomm_send(dlc, sample, sizeof sample), (int)sizeof sample);
    ASSERT_EQ(c->inflight, 2);
    ASSERT_FALSE(floss_hci_acl_can_send_fresh(h, 2));
    ASSERT_TRUE(floss_hci_acl_can_send_fresh(h, 3));

    /* Once the controller reports them sent, a fresh sample may go again. */
    ASSERT_TRUE(floss_sim_pump_until(cond_link_idle, (void *)(uintptr_t)h, 2000));
    ASSERT_TRUE(floss_hci_acl_can_send_fresh(h, 1));

    /* Controller buffers exhausted: data would queue, so never "fresh". */
    floss_sim_cfg()->nocp_delay_us = 0x40000000u;
    unsigned sent = 0;
    while (floss_hci_acl_can_send_now(h) && sent < 64) {
        ASSERT_EQ(floss_rfcomm_send(dlc, sample, 1), 1);
        sent++;
        if (dlc->tx_credits == 0) {
            rc_send(ch, RC_DLCI, RFC_UIH, 7, NULL, 0);
        }
    }
    ASSERT_FALSE(floss_hci_acl_can_send_now(h));
    ASSERT_FALSE(floss_hci_acl_can_send_fresh(h, 0));
    ASSERT_FALSE(floss_hci_acl_can_send_fresh(h, 255));
}

static const floss_test_case_t tests[] = {
    TEST_CASE(rfcomm_client_full_session),
    TEST_CASE(rfcomm_client_refused_by_dm),
    TEST_CASE(rfcomm_client_session_refused),
    TEST_CASE(rfcomm_client_link_loss),
    TEST_CASE(acl_can_send_fresh_gates_on_inflight),
};

#else /* !FLOSS_ENABLE_RFCOMM */

TEST(rfcomm_disabled) {
}

static const floss_test_case_t tests[] = {
    TEST_CASE(rfcomm_disabled),
};

#endif /* FLOSS_ENABLE_RFCOMM */

FLOSS_TEST_MAIN("rfcomm", tests)
