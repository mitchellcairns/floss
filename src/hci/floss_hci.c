/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_hci.c - HCI core: transport glue, command queue, event dispatch and
 * the connection table.
 *
 * Overview
 * --------
 * Commands. The controller tells us how many commands it can accept right
 * now (Num_HCI_Command_Packets, carried by every Command Complete / Command
 * Status event; 1 after power-up). Callers queue commands with
 * floss_hci_cmd(); the parameters are copied into a pool buffer that holds
 * the fully encoded packet, so it can be (re)sent at any time. We keep at
 * most ONE command outstanding: matching completions to callbacks is then
 * trivial, ordering is strict FIFO, and HCI command traffic is never on a
 * latency-critical path (ACL data is). A floss_timer guards the outstanding
 * command; if the controller never answers we report status 0xFF and assume
 * the credit came back so the queue does not wedge.
 *
 * Until bring-up finishes (READY) only bring-up commands are sent; anything
 * other layers queue early waits in FIFO order and goes out right after.
 *
 * Events. Command Complete/Status and Number Of Completed Packets are
 * consumed here (the latter by the ACL code). Link lifecycle events update
 * the connection table BEFORE they are forwarded, so GAP/L2CAP/LE can look
 * the link up while handling them. Everything else is forwarded verbatim to
 * GAP (or, for LE Meta subevents, to the LE module).
 *
 * Connection table. One floss_conn_t per baseband link. Slots are created
 * when we see an outgoing Create Connection command (OUTGOING), an incoming
 * Connection Request (INCOMING) or an LE connection completing, and freed on
 * Disconnection Complete (after upper layers were told) or on failure.
 */
#include "floss_hci_priv.h"

floss_hci_state_t floss_hci_g;

#define G floss_hci_g

/* ======================================================================== */
/* Connection table                                                         */
/* ======================================================================== */

/** Handles are only meaningful once the link is up. */
static bool conn_has_handle(const floss_conn_t *c) {
    return c->state == FLOSS_CONN_OPEN || c->state == FLOSS_CONN_CLOSING;
}

floss_conn_t *floss_hci_conn_by_handle(uint16_t handle) {
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        floss_conn_t *c = &G.conns[i];
        if (conn_has_handle(c) && c->handle == handle) {
            return c;
        }
    }
    return NULL;
}

/** Find a non-free slot by address and transport kind (a peer may have both a Classic and an LE link). */
static floss_conn_t *conn_find(const floss_addr_t *addr, bool le) {
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        floss_conn_t *c = &G.conns[i];
        if (c->state != FLOSS_CONN_FREE && c->le == le && floss_addr_eq(&c->addr, addr)) {
            return c;
        }
    }
    return NULL;
}

floss_conn_t *floss_hci_conn_by_addr(const floss_addr_t *addr) {
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        floss_conn_t *c = &G.conns[i];
        if (c->state != FLOSS_CONN_FREE && floss_addr_eq(&c->addr, addr)) {
            return c;
        }
    }
    return NULL;
}

floss_conn_t *floss_hci_conn_at(uint8_t i) {
    return i < FLOSS_MAX_CONNECTIONS ? &G.conns[i] : NULL;
}

static floss_conn_t *conn_alloc(void) {
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        floss_conn_t *c = &G.conns[i];
        if (c->state == FLOSS_CONN_FREE) {
            memset(c, 0, sizeof(*c));
            floss_bufq_init(&c->txq);
            return c;
        }
    }
    return NULL;
}

/** Free a slot. The caller has already returned its credits (floss_hci_acl_link_down). */
static void conn_release(floss_conn_t *c) {
    floss_bufq_drain(&c->txq);
    floss_buf_free(c->rx_buf);
    memset(c, 0, sizeof(*c));
    c->state = FLOSS_CONN_FREE;
}

static void on_conn_complete(const uint8_t *p, uint8_t len);

/** Re-arm the pending-slot guard for the oldest OUTGOING/INCOMING slot (stop if none). */
static void conn_pending_arm(void) {
    uint32_t now = floss_hal_time_us();
    uint32_t wait = 0;
    bool any = false;
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        const floss_conn_t *c = &G.conns[i];
        if (c->state != FLOSS_CONN_OUTGOING && c->state != FLOSS_CONN_INCOMING) {
            continue;
        }
        uint32_t age = now - c->pending_since_us;
        uint32_t left = age >= FLOSS_HCI_CONN_PENDING_TIMEOUT_US ? 0 : FLOSS_HCI_CONN_PENDING_TIMEOUT_US - age;
        if (!any || left < wait) {
            wait = left;
        }
        any = true;
    }
    if (any) {
        floss_timer_start(&G.pending_timer, wait);
    } else {
        floss_timer_stop(&G.pending_timer);
    }
}

/** A slot now waits for Connection Complete. */
static void conn_pending_start(floss_conn_t *c, floss_conn_state_t state) {
    c->state = state;
    c->pending_since_us = floss_hal_time_us();
    conn_pending_arm();
}

/**
 * The controller never reported how a page / accepted request ended (it
 * rejected the command, or is broken). Fail the slot exactly as a Connection
 * Complete with Connection Timeout would, so GAP sees the usual outcome.
 */
static void conn_pending_timeout(floss_timer_t *timer, void *arg) {
    FLOSS_UNUSED(timer);
    FLOSS_UNUSED(arg);
    uint32_t now = floss_hal_time_us();
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        floss_conn_t *c = &G.conns[i];
        if ((c->state == FLOSS_CONN_OUTGOING || c->state == FLOSS_CONN_INCOMING) &&
            now - c->pending_since_us >= FLOSS_HCI_CONN_PENDING_TIMEOUT_US) {
            /* Status(1) Handle(2) BD_ADDR(6) Link_Type(1) Encryption_Enabled(1) */
            uint8_t e[11] = {FLOSS_HCI_ERR_CONN_TIMEOUT, 0, 0};
            memcpy(&e[3], c->addr.b, 6);
            e[9] = 0x01; /* ACL */
            e[10] = 0;
            FLOSS_LOGW(HCI, "no Connection Complete for %s slot %u, failing it",
                       c->state == FLOSS_CONN_OUTGOING ? "outgoing" : "incoming", i);
            on_conn_complete(e, sizeof(e));
        }
    }
    conn_pending_arm();
}

/** First pending outgoing Classic slot (only one page can be in progress at a time). */
static floss_conn_t *conn_first_outgoing(void) {
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        if (G.conns[i].state == FLOSS_CONN_OUTGOING) {
            return &G.conns[i];
        }
    }
    return NULL;
}

/* ======================================================================== */
/* Transport glue                                                           */
/* ======================================================================== */

/**
 * The single exit point towards the controller. Snoops only packets the
 * driver actually accepted, so a BUSY retry is not captured twice.
 */
floss_err_t floss_hci_transport_send(uint8_t type, uint8_t *pkt, uint16_t len) {
    floss_transport_t *t = G.transport;

    FLOSS_DBG_PIN(FLOSS_DBG_PIN_TRANS_TX, 1);
#if FLOSS_ENABLE_STATS
    uint32_t t0 = floss_hal_time_us();
#endif
    floss_err_t err = t->ops->send(t, type, pkt, len);
    FLOSS_STAT_HIST(transport_send_time, floss_hal_time_us() - t0);
    FLOSS_DBG_PIN(FLOSS_DBG_PIN_TRANS_TX, 0);

    if (err == FLOSS_OK) {
        floss_snoop_packet(type, false, pkt, len);
        FLOSS_STAT_ADD(tx_bytes, len);
    } else if (err == FLOSS_ERR_BUSY) {
        FLOSS_STAT_INC(transport_busy);
    } else {
        FLOSS_STAT_INC(transport_errors);
        FLOSS_LOGE(HCI, "transport %s send failed: %s", t->ops->name, floss_err_str(err));
    }
    return err;
}

static void on_event(const uint8_t *pkt, uint16_t len);

/** Transport RX callback: every packet from the controller enters here. */
static void hci_rx(void *user, uint8_t type, const uint8_t *pkt, uint16_t len) {
    FLOSS_UNUSED(user);
    FLOSS_DBG_PIN(FLOSS_DBG_PIN_TRANS_RX, 1);
    floss_snoop_packet(type, true, pkt, len);
    FLOSS_STAT_ADD(rx_bytes, len);

    switch (type) {
    case FLOSS_PKT_EVT:
        on_event(pkt, len);
        break;
    case FLOSS_PKT_ACL:
        floss_hci_acl_on_rx(pkt, len);
        break;
    default:
        FLOSS_LOGD(HCI, "ignoring packet type 0x%02x (%u bytes)", type, len);
        break;
    }
    FLOSS_DBG_PIN(FLOSS_DBG_PIN_TRANS_RX, 0);
}

/* ======================================================================== */
/* Command queue                                                            */
/* ======================================================================== */

static void cmd_timeout(floss_timer_t *timer, void *arg);

/**
 * GAP pages a device by sending Create Connection itself. We watch for it so
 * the link has a slot (with its address and "we initiated" bit) before the
 * Connection Complete event arrives.
 */
static void note_outgoing_command(uint16_t opcode, const uint8_t *params, uint8_t len) {
    if (opcode != FLOSS_HCI_CREATE_CONNECTION || len < 6) {
        return;
    }
    floss_addr_t addr;
    memcpy(addr.b, params, 6);
    if (conn_find(&addr, false)) {
        return; /* already paging / crossing incoming request / connected */
    }
    floss_conn_t *c = conn_alloc();
    if (!c) {
        FLOSS_LOGW(HCI, "no free connection slot for outgoing page");
        return;
    }
    c->addr = addr;
    c->initiator = true;
    conn_pending_start(c, FLOSS_CONN_OUTGOING);
}

static floss_err_t cmd_enqueue(uint16_t opcode, const void *params, uint8_t len, floss_hci_cmd_cb cb, void *ctx,
                               bool internal) {
    if (len > 0 && params == NULL) {
        return FLOSS_ERR_INVALID;
    }
    if (G.cmdq_count >= FLOSS_HCI_CMD_QUEUE_LEN) {
        FLOSS_LOGW(HCI, "command queue full, dropping 0x%04x", opcode);
        return FLOSS_ERR_NOMEM;
    }
    floss_buf_t *b = floss_buf_alloc();
    if (!b) {
        FLOSS_STAT_INC(buf_alloc_failures);
        FLOSS_LOGW(HCI, "no buffer for command 0x%04x", opcode);
        return FLOSS_ERR_NOMEM;
    }
    b->type = FLOSS_PKT_CMD;
    b->opcode = opcode;
    b->flags = internal ? FLOSS_HCI_BUF_F_INTERNAL : 0;
    b->t_us = floss_hal_time_us();

    uint8_t *p = floss_buf_put(b, (uint16_t)(FLOSS_HCI_CMD_HDR_LEN + len));
    FLOSS_ASSERT(p != NULL);
    floss_put_le16(p, opcode);
    p[2] = len;
    if (len) {
        memcpy(p + FLOSS_HCI_CMD_HDR_LEN, params, len);
    }

    /*
     * Before READY, bring-up commands go to the FRONT so anything other
     * layers queued early stays behind the whole sequence. Bring-up only ever
     * has one command queued, so this keeps its own order too.
     */
    uint8_t idx;
    if (internal && !G.ready) {
        G.cmdq_head = (uint8_t)((G.cmdq_head + FLOSS_HCI_CMD_QUEUE_LEN - 1) % FLOSS_HCI_CMD_QUEUE_LEN);
        idx = G.cmdq_head;
    } else {
        idx = (uint8_t)((G.cmdq_head + G.cmdq_count) % FLOSS_HCI_CMD_QUEUE_LEN);
    }
    G.cmdq[idx].buf = b;
    G.cmdq[idx].cb = cb;
    G.cmdq[idx].ctx = ctx;
    G.cmdq_count++;

    note_outgoing_command(opcode, (const uint8_t *)params, len);
    floss_hci_cmd_pump();
    return FLOSS_OK;
}

floss_err_t floss_hci_cmd(uint16_t opcode, const void *params, uint8_t len, floss_hci_cmd_cb cb, void *ctx) {
    return cmd_enqueue(opcode, params, len, cb, ctx, false);
}

floss_err_t floss_hci_cmd_internal(uint16_t opcode, const void *params, uint8_t len, floss_hci_cmd_cb cb, void *ctx) {
    return cmd_enqueue(opcode, params, len, cb, ctx, true);
}

/**
 * Commands are waiting but the controller granted no credit and none is
 * outstanding (e.g. a Command Complete/Status with Num_HCI_Command_Packets = 0
 * for a command we never sent). A conforming controller follows up with a
 * NOP that returns a credit; guard against one that never does, or the queue
 * (and every buffer in it) would be stuck forever.
 */
static void cmd_watch_credits(void) {
    if (G.started && !G.cmd_inflight && G.cmd_credits == 0 && G.cmdq_count > 0 &&
        !floss_timer_active(&G.cmd_timer)) {
        floss_timer_start(&G.cmd_timer, FLOSS_HCI_CMD_TIMEOUT_US);
    }
}

void floss_hci_cmd_pump(void) {
    cmd_watch_credits();
    while (G.started && !G.cmd_inflight && G.cmd_credits > 0 && G.cmdq_count > 0) {
        floss_hci_cmd_slot_t slot = G.cmdq[G.cmdq_head];
        floss_buf_t *b = slot.buf;
        if (!G.ready && !(b->flags & FLOSS_HCI_BUF_F_INTERNAL)) {
            return; /* held until bring-up is done */
        }

        floss_err_t err = floss_hci_transport_send(FLOSS_PKT_CMD, b->data, b->len);
        if (err == FLOSS_ERR_BUSY) {
            return; /* stays at the head; floss_hci_poll() retries */
        }

        G.cmdq[G.cmdq_head].buf = NULL;
        G.cmdq_head = (uint8_t)((G.cmdq_head + 1) % FLOSS_HCI_CMD_QUEUE_LEN);
        G.cmdq_count--;
        uint16_t opcode = b->opcode;
        floss_buf_free(b); /* transport contract: synchronous ownership */

        if (err != FLOSS_OK) {
            /* The command never reached the controller: fail it like a timeout. */
            if (slot.cb) {
                slot.cb(slot.ctx, opcode, FLOSS_HCI_STATUS_TIMEOUT, NULL, 0);
            }
            continue;
        }

        G.cmd_credits--;
        G.cmd_inflight = true;
        G.inflight_opcode = opcode;
        G.inflight_cb = slot.cb;
        G.inflight_ctx = slot.ctx;
        G.inflight_t_us = floss_hal_time_us();
        floss_timer_start(&G.cmd_timer, FLOSS_HCI_CMD_TIMEOUT_US);
        FLOSS_STAT_INC(tx_cmd);
        FLOSS_TRACE(FLOSS_TR_TX_CMD, opcode);
        FLOSS_LOGD(HCI, "> cmd 0x%04x %s", opcode, floss_hci_opcode_str(opcode));
    }
}

/**
 * The outstanding command is done (Complete, Status or timeout). Clears the
 * in-flight state BEFORE calling back, because callbacks routinely queue the
 * next command (bring-up is a chain of them).
 */
static void cmd_finish(uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len) {
    if (!G.cmd_inflight || G.inflight_opcode != opcode) {
        FLOSS_LOGW(HCI, "unexpected completion for 0x%04x (outstanding: %s0x%04x)", opcode,
                   G.cmd_inflight ? "" : "none, last ", G.inflight_opcode);
        floss_hci_cmd_pump();
        return;
    }
    G.cmd_inflight = false;
    floss_timer_stop(&G.cmd_timer);
    FLOSS_STAT_HIST(cmd_latency, (uint32_t)(floss_hal_time_us() - G.inflight_t_us));
    FLOSS_TRACE(FLOSS_TR_CMD_DONE, opcode);

    if (status != FLOSS_HCI_SUCCESS) {
        if (status != FLOSS_HCI_STATUS_TIMEOUT) {
            FLOSS_STAT_INC(cmd_failures);
            FLOSS_LOGW(HCI, "cmd 0x%04x %s failed: %s (0x%02x)", opcode, floss_hci_opcode_str(opcode),
                       floss_hci_status_str(status), status);
        }
        /* A page that never started will not produce a Connection Complete. */
        if (opcode == FLOSS_HCI_CREATE_CONNECTION) {
            floss_conn_t *c = conn_first_outgoing();
            if (c) {
                conn_release(c);
            }
        }
    }

    floss_hci_cmd_cb cb = G.inflight_cb;
    void *ctx = G.inflight_ctx;
    G.inflight_cb = NULL;
    G.inflight_ctx = NULL;
    if (cb) {
        cb(ctx, opcode, status, ret, ret_len);
    }
    floss_hci_cmd_pump();
}

static void cmd_timeout(floss_timer_t *timer, void *arg) {
    FLOSS_UNUSED(timer);
    FLOSS_UNUSED(arg);
    if (!G.cmd_inflight) {
        if (G.cmd_credits == 0 && G.cmdq_count > 0) {
            FLOSS_STAT_INC(cmd_timeouts);
            FLOSS_LOGE(HCI, "controller granted no command credit for %u ms, assuming one",
                       (unsigned)(FLOSS_HCI_CMD_TIMEOUT_US / 1000u));
            G.cmd_credits = 1;
            floss_hci_cmd_pump();
        }
        return;
    }
    FLOSS_STAT_INC(cmd_timeouts);
    FLOSS_LOGE(HCI, "cmd 0x%04x %s timed out", G.inflight_opcode, floss_hci_opcode_str(G.inflight_opcode));
    /* Assume the controller can take another command, or the queue would wedge forever. */
    G.cmd_credits = 1;
    cmd_finish(G.inflight_opcode, FLOSS_HCI_STATUS_TIMEOUT, NULL, 0);
}

/* Command Complete: Num_HCI_Command_Packets(1) Opcode(2) Return_Parameters(...) */
static void on_cmd_complete(const uint8_t *p, uint8_t len) {
    if (len < 3) {
        FLOSS_STAT_INC(rx_dropped);
        return;
    }
    G.cmd_credits = p[0];
    uint16_t opcode = floss_get_le16(p + 1);
    if (opcode == 0) {
        /* NOP: the controller is just announcing how many commands it accepts. */
        floss_hci_cmd_pump();
        return;
    }
    const uint8_t *ret = p + 3;
    uint16_t ret_len = (uint16_t)(len - 3);
    uint8_t status = ret_len ? ret[0] : FLOSS_HCI_SUCCESS;
    cmd_finish(opcode, status, ret_len ? ret + 1 : NULL, ret_len ? (uint16_t)(ret_len - 1) : 0);
}

/* Command Status: Status(1) Num_HCI_Command_Packets(1) Opcode(2) */
static void on_cmd_status(const uint8_t *p, uint8_t len) {
    if (len < 4) {
        FLOSS_STAT_INC(rx_dropped);
        return;
    }
    G.cmd_credits = p[1];
    uint16_t opcode = floss_get_le16(p + 2);
    if (opcode == 0) {
        floss_hci_cmd_pump();
        return;
    }
    cmd_finish(opcode, p[0], NULL, 0);
}

/* ======================================================================== */
/* Link events                                                              */
/* ======================================================================== */

/* Connection Request: BD_ADDR(6) Class_Of_Device(3) Link_Type(1) */
static void on_conn_request(const uint8_t *p, uint8_t len) {
    if (len >= 10 && p[9] == 0x01 /* ACL */) {
        floss_addr_t addr;
        memcpy(addr.b, p, 6);
        if (!conn_find(&addr, false)) {
            floss_conn_t *c = conn_alloc();
            if (c) {
                c->addr = addr;
                conn_pending_start(c, FLOSS_CONN_INCOMING);
            } else {
                FLOSS_LOGW(HCI, "connection request but no free slot");
            }
        }
    }
    floss_gap_on_hci_event(FLOSS_EVT_CONNECTION_REQUEST, p, len);
}

/** Common bookkeeping once a link (Classic or LE) is up. */
static void conn_opened(floss_conn_t *c, uint16_t handle) {
    c->state = FLOSS_CONN_OPEN;
    c->handle = handle;
    c->mode = FLOSS_LINK_MODE_ACTIVE;
    FLOSS_STAT_INC(connections);
    FLOSS_STAT_SET(current_mode, FLOSS_LINK_MODE_ACTIVE);
    FLOSS_STAT_SET(sniff_interval, 0);
    FLOSS_TRACE(FLOSS_TR_CONN, handle);

    char s[18];
    floss_addr_to_str(&c->addr, s);
    FLOSS_LOGI(HCI, "%s link 0x%03x up: %s (%s, %s)", c->le ? "LE" : "ACL", handle, s,
               c->initiator ? "outgoing" : "incoming", c->role == 0 ? "central" : "peripheral");
}

/* Connection Complete: Status(1) Handle(2) BD_ADDR(6) Link_Type(1) Encryption_Enabled(1) */
static void on_conn_complete(const uint8_t *p, uint8_t len) {
    if (len < 11 || p[9] != 0x01 /* not ACL (SCO/eSCO is not ours) */) {
        floss_gap_on_hci_event(FLOSS_EVT_CONNECTION_COMPLETE, p, len);
        return;
    }
    uint8_t status = p[0];
    uint16_t handle = floss_get_le16(p + 1) & 0x0FFF;
    floss_addr_t addr;
    memcpy(addr.b, p + 3, 6);

    floss_conn_t *c = conn_find(&addr, false);
    if (c && conn_has_handle(c)) {
        c = NULL; /* stale open link with same peer should not exist; never clobber it */
    }

    if (status == FLOSS_HCI_SUCCESS && floss_hci_conn_by_handle(handle)) {
        /* A broken controller reusing a live handle: two slots with one handle
         * would make every lookup ambiguous. Ignore the event. */
        FLOSS_LOGE(HCI, "Connection Complete for handle 0x%03x already in use, ignored", handle);
        FLOSS_STAT_INC(rx_dropped);
        if (c) {
            conn_release(c);
        }
        return;
    }

    if (status == FLOSS_HCI_SUCCESS) {
        if (!c) {
            c = conn_alloc();
        }
        if (c) {
            c->initiator = (c->state == FLOSS_CONN_OUTGOING);
            c->addr = addr;
            c->le = false;
            c->role = c->initiator ? 0 : 1; /* paging device starts as central; Role Change updates */
            c->encrypted = p[10] != 0;
            c->authenticated = false;
            conn_opened(c, handle);
        } else {
            FLOSS_LOGE(HCI, "link 0x%03x up but connection table full, disconnecting", handle);
            uint8_t dp[3];
            floss_put_le16(dp, handle);
            dp[2] = FLOSS_HCI_ERR_REJECTED_LIMITED_RESOURCES;
            floss_hci_cmd(FLOSS_HCI_DISCONNECT, dp, sizeof(dp), NULL, NULL);
        }
    } else {
        if (c) {
            conn_release(c);
        }
        FLOSS_LOGI(HCI, "connection failed: %s (0x%02x)", floss_hci_status_str(status), status);
    }
    floss_gap_on_hci_event(FLOSS_EVT_CONNECTION_COMPLETE, p, len);
}

/* Disconnection Complete: Status(1) Handle(2) Reason(1) */
static void on_disconn_complete(const uint8_t *p, uint8_t len) {
    floss_conn_t *c = NULL;
    if (len >= 4 && p[0] == FLOSS_HCI_SUCCESS) {
        c = floss_hci_conn_by_handle(floss_get_le16(p + 1) & 0x0FFF);
    }
    if (!c) {
        floss_gap_on_hci_event(FLOSS_EVT_DISCONNECTION_COMPLETE, p, len);
        return;
    }
    uint16_t handle = c->handle;
    uint8_t reason = p[3];

    /* Upper layers first, while the slot (address, role...) is still readable. */
    if (c->le) {
#if FLOSS_ENABLE_LE
        floss_le_on_link_down(handle, reason);
#endif
    } else {
        floss_gap_on_hci_event(FLOSS_EVT_DISCONNECTION_COMPLETE, p, len);
    }

    uint16_t returned = floss_hci_acl_link_down(c);
    conn_release(c);

    FLOSS_STAT_INC(disconnections);
    FLOSS_STAT_SET(last_disconnect_reason, reason);
    FLOSS_TRACE(FLOSS_TR_DISCONN, ((uint32_t)handle << 16) | reason);
    FLOSS_LOGI(HCI, "link 0x%03x down: %s (0x%02x)", handle, floss_hci_status_str(reason), reason);

    if (returned) {
        /* The controller freed that link's buffers: other links may proceed. */
        floss_hci_acl_flush();
        floss_hci_acl_notify_credits();
    }
}

/* Mode Change: Status(1) Handle(2) Current_Mode(1) Interval(2) */
static void on_mode_change(const uint8_t *p, uint8_t len) {
    if (len >= 6 && p[0] == FLOSS_HCI_SUCCESS) {
        floss_conn_t *c = floss_hci_conn_by_handle(floss_get_le16(p + 1) & 0x0FFF);
        if (c) {
            c->mode = p[3];
            c->sniff_interval = (c->mode == FLOSS_LINK_MODE_ACTIVE) ? 0 : floss_get_le16(p + 4);
            FLOSS_STAT_INC(mode_changes);
            FLOSS_STAT_SET(current_mode, c->mode);
            FLOSS_STAT_SET(sniff_interval, c->sniff_interval);
            FLOSS_TRACE(FLOSS_TR_MODE, ((uint32_t)c->mode << 16) | c->sniff_interval);
            FLOSS_LOGI(HCI, "link 0x%03x mode %u interval %u slots", c->handle, c->mode, c->sniff_interval);
        }
    }
    floss_gap_on_hci_event(FLOSS_EVT_MODE_CHANGE, p, len);
}

/*
 * Encryption Change: Status(1) Handle(2) Encryption_Enabled(1)
 * Encryption Key Refresh Complete: Status(1) Handle(2)
 * Both end up as "the link's encryption state is now X".
 */
static void on_encryption(uint8_t code, const uint8_t *p, uint8_t len) {
    floss_conn_t *c = NULL;
    if (len >= 3) {
        c = floss_hci_conn_by_handle(floss_get_le16(p + 1) & 0x0FFF);
    }
    if (c && p[0] == FLOSS_HCI_SUCCESS) {
        if (code == FLOSS_EVT_ENCRYPTION_CHANGE) {
            c->encrypted = (len >= 4) && p[3] != 0;
        } else {
            c->encrypted = true; /* a refreshed key implies encryption is on */
        }
    }
    if (c && c->le) {
#if FLOSS_ENABLE_LE
        floss_le_on_encryption(c->handle, p[0], c->encrypted);
#endif
        return;
    }
    floss_gap_on_hci_event(code, p, len);
}

/* Authentication Complete: Status(1) Handle(2) */
static void on_auth_complete(const uint8_t *p, uint8_t len) {
    if (len >= 3 && p[0] == FLOSS_HCI_SUCCESS) {
        floss_conn_t *c = floss_hci_conn_by_handle(floss_get_le16(p + 1) & 0x0FFF);
        if (c) {
            c->authenticated = true;
        }
    }
    floss_gap_on_hci_event(FLOSS_EVT_AUTHENTICATION_COMPLETE, p, len);
}

/* Role Change: Status(1) BD_ADDR(6) New_Role(1) */
static void on_role_change(const uint8_t *p, uint8_t len) {
    if (len >= 8 && p[0] == FLOSS_HCI_SUCCESS) {
        floss_addr_t addr;
        memcpy(addr.b, p + 1, 6);
        floss_conn_t *c = conn_find(&addr, false);
        if (c) {
            c->role = p[7];
        }
    }
    floss_gap_on_hci_event(FLOSS_EVT_ROLE_CHANGE, p, len);
}

/*
 * LE Meta. (Enhanced) Connection Complete share the leading layout:
 *   Status(1) Handle(2) Role(1) Peer_Address_Type(1) Peer_Address(6) ...
 * HCI fills the connection slot, then every subevent goes to the LE module.
 */
static void on_le_meta(const uint8_t *p, uint8_t len) {
    if (len < 1) {
        FLOSS_STAT_INC(rx_dropped);
        return;
    }
    uint8_t sub = p[0];
#if FLOSS_ENABLE_LE
    bool conn_evt = sub == FLOSS_LE_SUBEVT_CONN_COMPLETE || sub == FLOSS_LE_SUBEVT_ENHANCED_CONN_COMPLETE;
    if (conn_evt && len < (sub == FLOSS_LE_SUBEVT_CONN_COMPLETE ? 1 + 18 : 1 + 30)) {
        /* The LE module ignores short ones too: never open a slot it does not know. */
        FLOSS_STAT_INC(rx_dropped);
        return;
    }
    if (conn_evt && p[1] == FLOSS_HCI_SUCCESS && floss_hci_conn_by_handle(floss_get_le16(p + 2) & 0x0FFF)) {
        FLOSS_LOGE(HCI, "LE Connection Complete for handle 0x%03x already in use, ignored",
                   floss_get_le16(p + 2) & 0x0FFF);
        FLOSS_STAT_INC(rx_dropped);
        return;
    }
    if (conn_evt && p[1] == FLOSS_HCI_SUCCESS) {
        floss_conn_t *c = conn_alloc();
        if (c) {
            c->le = true;
            c->role = p[4];
            c->initiator = (c->role == 0); /* LE: the initiator always becomes central */
            /* Enhanced reports resolved identities as 0x02/0x03: keep public/random. */
            c->addr_type = p[5] & 0x01;
            memcpy(c->addr.b, p + 6, 6);
            conn_opened(c, floss_get_le16(p + 2) & 0x0FFF);
        } else {
            /* Without a slot the link could never be torn down cleanly: drop it,
             * and let the LE module see a failed connection (it restarts
             * advertising) instead of a link HCI does not know. */
            uint16_t handle = floss_get_le16(p + 2) & 0x0FFF;
            FLOSS_LOGE(HCI, "LE link 0x%03x up but connection table full, disconnecting", handle);
            uint8_t dp[3];
            floss_put_le16(dp, handle);
            dp[2] = FLOSS_HCI_ERR_REJECTED_LIMITED_RESOURCES;
            floss_hci_cmd(FLOSS_HCI_DISCONNECT, dp, sizeof(dp), NULL, NULL);
            uint8_t failed[30];
            uint8_t n = (uint8_t)FLOSS_MIN((uint16_t)(len - 1), (uint16_t)sizeof(failed));
            memcpy(failed, p + 1, n);
            failed[0] = FLOSS_HCI_ERR_REJECTED_LIMITED_RESOURCES; /* Status */
            floss_le_on_hci_event(sub, failed, n);
            return;
        }
    }
    floss_le_on_hci_event(sub, p + 1, (uint8_t)(len - 1));
#else
    FLOSS_LOGD(HCI, "LE subevent 0x%02x ignored (LE disabled)", sub);
#endif
}

/* ======================================================================== */
/* Event dispatch                                                           */
/* ======================================================================== */

static void on_event(const uint8_t *pkt, uint16_t len) {
    if (len < FLOSS_HCI_EVT_HDR_LEN || (uint16_t)(FLOSS_HCI_EVT_HDR_LEN + pkt[1]) > len) {
        FLOSS_STAT_INC(rx_dropped);
        FLOSS_LOGW(HCI, "malformed event (%u bytes)", len);
        return;
    }
    uint8_t code = pkt[0];
    uint8_t plen = pkt[1]; /* trailing transport padding, if any, is ignored */
    const uint8_t *p = pkt + FLOSS_HCI_EVT_HDR_LEN;

    FLOSS_STAT_INC(rx_evt);
    FLOSS_TRACE(FLOSS_TR_RX_EVT, code);
    if (code != FLOSS_EVT_NUMBER_OF_COMPLETED_PACKETS) { /* too chatty to log */
        FLOSS_LOGD(HCI, "< evt 0x%02x %s (%u)", code, floss_hci_event_str(code), plen);
    }

    switch (code) {
    case FLOSS_EVT_COMMAND_COMPLETE:
        on_cmd_complete(p, plen);
        break;
    case FLOSS_EVT_COMMAND_STATUS:
        on_cmd_status(p, plen);
        break;
    case FLOSS_EVT_NUMBER_OF_COMPLETED_PACKETS:
        floss_hci_acl_on_completed(p, plen);
        break;
    case FLOSS_EVT_CONNECTION_REQUEST:
        on_conn_request(p, plen);
        break;
    case FLOSS_EVT_CONNECTION_COMPLETE:
        on_conn_complete(p, plen);
        break;
    case FLOSS_EVT_DISCONNECTION_COMPLETE:
        on_disconn_complete(p, plen);
        break;
    case FLOSS_EVT_MODE_CHANGE:
        on_mode_change(p, plen);
        break;
    case FLOSS_EVT_ENCRYPTION_CHANGE:
    case FLOSS_EVT_ENCRYPTION_KEY_REFRESH_COMPLETE:
        on_encryption(code, p, plen);
        break;
    case FLOSS_EVT_AUTHENTICATION_COMPLETE:
        on_auth_complete(p, plen);
        break;
    case FLOSS_EVT_ROLE_CHANGE:
        on_role_change(p, plen);
        break;
    case FLOSS_EVT_LE_META:
        on_le_meta(p, plen);
        break;
    case FLOSS_EVT_HARDWARE_ERROR:
        FLOSS_LOGE(HCI, "controller hardware error 0x%02x", plen ? p[0] : 0);
        floss_gap_on_hci_event(code, p, plen);
        break;
    case FLOSS_EVT_DATA_BUFFER_OVERFLOW:
        FLOSS_LOGE(HCI, "controller reports data buffer overflow (credit accounting bug?)");
        floss_gap_on_hci_event(code, p, plen);
        break;
    default:
        floss_gap_on_hci_event(code, p, plen);
        break;
    }
}

/* ======================================================================== */
/* Lifecycle                                                                */
/* ======================================================================== */

uint8_t floss_hci_completion_batch(void) {
    uint8_t b = G.chipset ? G.chipset->completion_batch : 0;
    return b ? b : 1;
}

void floss_hci_init(floss_transport_t *transport, const floss_chipset_t *chipset) {
    memset(&G, 0, sizeof(G));
    G.transport = transport;
    G.chipset = chipset;
    G.cmd_credits = 1; /* the controller accepts one command after power-up */
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        floss_bufq_init(&G.conns[i].txq);
    }
    floss_timer_init(&G.cmd_timer, cmd_timeout, NULL);
    floss_timer_init(&G.pending_timer, conn_pending_timeout, NULL);
}

floss_err_t floss_hci_start(void) {
    floss_transport_t *t = G.transport;
    if (!t || !t->ops || !t->ops->send || !t->ops->poll || !t->ops->open) {
        return FLOSS_ERR_INVALID;
    }
    if (t->ops->headroom > FLOSS_TRANSPORT_MAX_HEADROOM) {
        FLOSS_LOGE(HCI, "transport %s needs %u bytes headroom, FLOSS_TRANSPORT_MAX_HEADROOM is %u", t->ops->name,
                   t->ops->headroom, (unsigned)FLOSS_TRANSPORT_MAX_HEADROOM);
        return FLOSS_ERR_INVALID;
    }
    if (G.started) {
        return FLOSS_ERR_STATE;
    }

    t->rx = hci_rx;
    t->rx_user = NULL;
    floss_err_t err = t->ops->open(t);
    if (err != FLOSS_OK) {
        FLOSS_LOGE(HCI, "transport %s open failed: %s", t->ops->name, floss_err_str(err));
        return err;
    }
    G.started = true;
    FLOSS_LOGI(HCI, "transport %s open, chipset %s", t->ops->name, G.chipset && G.chipset->name ? G.chipset->name : "generic");
    floss_hci_bringup_start();
    return FLOSS_OK;
}

void floss_hci_poll(void) {
    if (!G.started) {
        return;
    }
#if FLOSS_ENABLE_STATS
    uint32_t t0 = floss_hal_time_us();
    int delivered = G.transport->ops->poll(G.transport);
    if (delivered > 0) {
        /* Only polls that moved data: idle polls would drown the signal. */
        FLOSS_STAT_HIST(transport_poll_time, floss_hal_time_us() - t0);
    }
#else
    G.transport->ops->poll(G.transport);
#endif
    /* Retry anything the transport refused (BUSY) earlier. */
    floss_hci_cmd_pump();
    floss_hci_acl_flush();
}

bool floss_hci_ready(void) {
    return G.ready;
}

const floss_controller_info_t *floss_hci_controller(void) {
    return &G.info;
}
