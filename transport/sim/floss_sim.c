/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_sim.c - Virtual controller + scripted remote peer behind a floss_transport_t.
 *
 * Overview
 * --------
 *
 *   stack --send()--> sim_rx_cmd() / sim_rx_acl()        (synchronous)
 *                          | answers are QUEUED, never delivered re-entrantly
 *                          v
 *                    delivery queue (FIFO of complete HCI packets)
 *                          |
 *   stack <--poll()--  sim_tick() (time based work: page timeout, completions)
 *                      then deliver queue head..tail
 *
 * Queuing every answer instead of calling back into the stack from inside
 * send() mirrors real hardware: the host only sees a response on a later
 * transport poll, so the stack's ordering and re-entrancy assumptions are
 * exercised exactly as on a UART or the CYW43 shared bus.
 *
 * The remote side is not a full stack. It implements just enough of what a
 * PC/console host does (pairing decisions, L2CAP signaling, an SDP client)
 * to drive the device-side stack through every flow, and it records what it
 * saw so tests can assert on it.
 *
 * File map:
 *   1. state, PRNG, delivery queue, event emission (+ event masks)
 *   2. command log
 *   3. peers and links
 *   4. pairing (what LMP and the remote host would do)
 *   5. HCI command handlers, grouped by OGF
 *   6. ACL from the stack: credits, reassembly, completions
 *   7. remote L2CAP peer: signaling, channels, inboxes
 *   8. ACL to the stack
 *   9. public helpers: links, channels, SDP client, LE
 *  10. transport ops and pump helpers
 *  11. LE central support: remote advertisers (scanning), scripted GATT server
 */
#include "floss_sim.h"

#include <string.h>

/* ------------------------------------------------------------------------ */
/* Protocol constants not named by the public headers                       */
/* ------------------------------------------------------------------------ */

#define SIM_OP_READ_LOCAL_EXT_FEATURES     FLOSS_OPCODE(0x04, 0x0004)
#define SIM_OP_REMOTE_NAME_CANCEL          FLOSS_OPCODE(0x01, 0x001A)
#define SIM_OP_READ_REMOTE_EXT_FEATURES    FLOSS_OPCODE(0x01, 0x001C)
#define SIM_OP_READ_LINK_POLICY            FLOSS_OPCODE(0x02, 0x000C)
#define SIM_OP_FLUSH                       FLOSS_OPCODE(0x03, 0x0008)
#define SIM_OP_VENDOR_SET_ADDR             0xFC01 /* CYW43xxx-style "write BD_ADDR" */
#define SIM_OP_LE_READ_ADV_TX_POWER        FLOSS_OPCODE(0x08, 0x0007)
#define SIM_OP_LE_CREATE_CONNECTION        FLOSS_OPCODE(0x08, 0x000D)
#define SIM_OP_LE_CREATE_CONNECTION_CANCEL FLOSS_OPCODE(0x08, 0x000E)
#define SIM_OP_LE_READ_ACCEPT_LIST_SIZE    FLOSS_OPCODE(0x08, 0x000F)
#define SIM_OP_LE_CONNECTION_UPDATE        FLOSS_OPCODE(0x08, 0x0013)
#define SIM_OP_LE_READ_REMOTE_FEATURES     FLOSS_OPCODE(0x08, 0x0016)
#define SIM_OP_LE_ENCRYPT                  FLOSS_OPCODE(0x08, 0x0017)
#define SIM_OP_LE_RAND                     FLOSS_OPCODE(0x08, 0x0018)
#define SIM_OP_LE_START_ENCRYPTION         FLOSS_OPCODE(0x08, 0x0019)
#define SIM_OP_LE_LTK_REPLY                FLOSS_OPCODE(0x08, 0x001A)
#define SIM_OP_LE_LTK_NEG_REPLY            FLOSS_OPCODE(0x08, 0x001B)
#define SIM_OP_LE_READ_SUPPORTED_STATES    FLOSS_OPCODE(0x08, 0x001C)
#define SIM_OP_LE_SET_DATA_LENGTH          FLOSS_OPCODE(0x08, 0x0022)
#define SIM_OP_LE_READ_DEFAULT_DATA_LENGTH FLOSS_OPCODE(0x08, 0x0023)
#define SIM_OP_LE_READ_P256_KEY            FLOSS_OPCODE(0x08, 0x0025)
#define SIM_OP_LE_GENERATE_DHKEY           FLOSS_OPCODE(0x08, 0x0026)
#define SIM_OP_LE_READ_RESOLVING_LIST_SIZE FLOSS_OPCODE(0x08, 0x002A)
#define SIM_OP_LE_READ_MAX_DATA_LENGTH     FLOSS_OPCODE(0x08, 0x002F)
#define SIM_OP_LE_SET_SCAN_PARAMS          FLOSS_OPCODE(0x08, 0x000B)
#define SIM_OP_LE_SET_SCAN_ENABLE          FLOSS_OPCODE(0x08, 0x000C)
#define SIM_OP_LE_SET_PHY                  FLOSS_OPCODE(0x08, 0x0032)

#define SIM_LE_SUB_ADV_REPORT       0x02
#define SIM_LE_SUB_PHY_UPDATE       0x0C
#define SIM_LE_SUB_CONN_COMPLETE    0x01
#define SIM_LE_SUB_CONN_UPDATE      0x03
#define SIM_LE_SUB_REMOTE_FEATURES  0x04
#define SIM_LE_SUB_P256_KEY         0x08
#define SIM_LE_SUB_DHKEY            0x09

#define SIM_ERR_CONN_LIMIT          0x09
#define SIM_ERR_ACL_EXISTS          0x0B
#define SIM_ERR_COMMAND_DISALLOWED  0x0C
#define SIM_ERR_INVALID_PARAMS      0x12

/* Default event masks after Reset (Core spec: Set Event Mask / LE Set Event Mask). */
#define SIM_EVT_MASK_DEFAULT        0x00001FFFFFFFFFFFull
#define SIM_LE_EVT_MASK_DEFAULT     0x000000000000001Full

/* L2CAP signaling command codes */
#define SIG_CMD_REJECT  0x01
#define SIG_CONN_REQ    0x02
#define SIG_CONN_RSP    0x03
#define SIG_CONF_REQ    0x04
#define SIG_CONF_RSP    0x05
#define SIG_DISC_REQ    0x06
#define SIG_DISC_RSP    0x07
#define SIG_ECHO_REQ    0x08
#define SIG_ECHO_RSP    0x09
#define SIG_INFO_REQ    0x0A
#define SIG_INFO_RSP    0x0B

#define L2CAP_MIN_MTU      48  /* BR/EDR minimum MTU every implementation must accept */
#define L2CAP_DEFAULT_MTU  672
#define L2CAP_OPT_MTU      0x01
#define L2CAP_OPT_FLUSH_TO 0x02
#define L2CAP_OPT_QOS      0x03

/** Packets delivered per transport poll at most (a real UART delivers what arrived). */
#define SIM_DELIVER_PER_POLL 64
/** ACL packets per link whose completion has not been reported yet. */
#define SIM_INFLIGHT_MAX 64
/** Handles that died but whose Disconnection Complete the stack has not seen. */
#define SIM_ZOMBIES 8

/* ------------------------------------------------------------------------ */
/* 1. State                                                                 */
/* ------------------------------------------------------------------------ */

/** One packet waiting to be delivered to the stack. */
typedef struct {
    uint8_t type;          /* FLOSS_PKT_EVT / FLOSS_PKT_ACL */
    uint8_t cmd_credit;    /* Command Complete/Status: gives the host its command credit back */
    uint16_t len;
    uint16_t credits[2];   /* ACL buffers this packet returns: [0] BR/EDR pool, [1] LE pool */
    uint8_t data[4 + FLOSS_SIM_ACL_MAX];
} sim_pkt_t;

enum {
    PAIR_NONE = 0,
    PAIR_WAIT_LINK_KEY,  /* Link Key Request sent, waiting for the host's (negative) reply */
    PAIR_WAIT_IO_CAP,    /* IO Capability Request sent */
    PAIR_WAIT_CONFIRM,   /* User Confirmation Request sent */
    PAIR_WAIT_PIN,       /* legacy: PIN Code Request sent */
};

typedef struct {
    floss_sim_link_t pub;      /* MUST stay first: public pointers are cast back */
    uint32_t page_due;         /* PAGING: when Page Timeout fires */
    uint8_t pair_step;
    bool pair_local;           /* the stack asked for authentication */
    uint8_t stack_io_cap;
    uint8_t next_sig_id;
    /* stack -> sim ACL reassembly */
    bool rx_active;
    uint16_t rx_len;
    uint8_t rx[4 + FLOSS_SIM_ACL_MAX];
    /* ACL packets accepted but not yet reported complete (FIFO of due times) */
    uint32_t done_due[SIM_INFLIGHT_MAX];
    uint8_t done_head;
    uint8_t done_count;
} sim_link_t;

typedef struct {
    floss_sim_chan_t pub;      /* MUST stay first */
    uint8_t sig_id;            /* identifier of our outstanding request */
    bool fixed;
    bool cfg_in_done;          /* we accepted the stack's configuration */
    bool cfg_out_done;         /* the stack accepted ours */
    uint8_t in_head;
    uint8_t in_count;
    uint16_t in_len[FLOSS_SIM_INBOX_LEN];
    uint8_t in_data[FLOSS_SIM_INBOX_LEN][FLOSS_SIM_ACL_MAX];
} sim_chan_t;

/** A remote advertiser (section 11). */
typedef struct {
    floss_sim_advertiser_t pub; /* MUST stay first */
    uint32_t due;               /* next advertising event */
    bool seen;                  /* reported in this scan session (duplicate filtering) */
} sim_adv_t;

/** One attribute of the scripted GATT server (section 11). */
typedef struct {
    uint16_t handle;
    uint16_t type16;
    bool is128;
    uint8_t type128[16];
    uint8_t error;
    uint16_t len;
    uint8_t value[FLOSS_SIM_GATT_VALUE_MAX];
} sim_gatt_attr_t;

typedef struct {
    bool active;
    bool silent;
    bool req_pending;           /* a request arrived while silent */
    uint16_t link;              /* LE link it serves */
    uint16_t server_mtu;
    uint8_t count;
    sim_gatt_attr_t attrs[FLOSS_SIM_GATT_MAX_ATTRS];
    floss_sim_gatt_stats_t stats;
} sim_gatt_t;

static struct {
    floss_transport_t transport;
    floss_sim_config_t cfg;
    floss_sim_stats_t stats;
    bool open;
    bool in_poll;
    uint32_t rng;

    /* Controller state the host writes. */
    floss_addr_t addr;
    uint8_t scan;
    uint32_t cod;
    char name[249];
    bool ssp_mode;
    bool sc_host;
    bool le_host;
    uint16_t page_timeout;
    uint16_t default_policy;
    uint64_t evt_mask;
    uint64_t evt_mask2;
    uint64_t le_evt_mask;
    uint16_t next_handle;
    uint16_t next_sim_cid;
    bool le_connecting;

    /* Flow control as the controller sees it. */
    uint8_t cmd_outstanding;   /* commands received whose Complete/Status is not delivered */
    uint16_t pool_out[2];      /* ACL buffers in use: [0] BR/EDR, [1] LE */
    uint16_t zombies[SIM_ZOMBIES];

    /* Delivery queue. */
    sim_pkt_t q[FLOSS_SIM_QUEUE_LEN];
    uint16_t q_head;
    uint16_t q_count;

    /* Logs. */
    floss_sim_cmd_t cmd_log[FLOSS_SIM_CMD_LOG_LEN];
    uint32_t cmd_total;
    floss_sim_sig_t sig_log[FLOSS_SIM_SIG_LOG_LEN];
    uint32_t sig_total;

    floss_sim_peer_t peers[FLOSS_SIM_MAX_PEERS];
    sim_link_t links[FLOSS_SIM_MAX_LINKS];
    sim_chan_t chans[FLOSS_SIM_MAX_CHANS];
    uint32_t chan_seq;

    /* LE central support (section 11). */
    floss_sim_le_scan_t le_scan;
    sim_adv_t advs[FLOSS_SIM_MAX_ADVERTISERS];
    sim_gatt_t gatt;
} sim;

/* LMP features page 0: 3/5-slot, encryption, slot offset, timing accuracy,
 * role switch, sniff; power control; EDR 2/3 Mb/s, interlaced scans; AFH,
 * LE supported (byte 4 bit 6), 3-slot EDR; 5-slot EDR, sniff subrating;
 * EIR, simultaneous LE + BR/EDR (byte 6 bit 1), SSP (byte 6 bit 3),
 * non-flushable PB flag; LSTO changed event, extended features. */
static const uint8_t sim_features[8] = {0xBF, 0x02, 0x04, 0x3E, 0xD8, 0x1B, 0x4B, 0x83};
/* LE features: encryption, conn param request, ext reject, peripheral feature exchange, ping, data length ext. */
static const uint8_t sim_le_features[8] = {0x3F, 0, 0, 0, 0, 0, 0, 0};

/* ------------------------------------------------------------------------ */
/* Small helpers                                                            */
/* ------------------------------------------------------------------------ */

static uint64_t get_le64(const uint8_t *p) {
    return (uint64_t)floss_get_le32(p) | ((uint64_t)floss_get_le32(p + 4) << 32);
}

/** xorshift32: deterministic per seed, independent of the HAL's entropy. */
static uint32_t sim_rand(void) {
    uint32_t x = sim.rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    sim.rng = x;
    return x;
}

static void sim_rand_bytes(uint8_t *p, uint16_t n) {
    for (uint16_t i = 0; i < n; i++) {
        p[i] = (uint8_t)(sim_rand() >> 24);
    }
}

static const char *addr_str(const floss_addr_t *a) {
    static char buf[2][18];
    static uint8_t which;
    which ^= 1;
    floss_addr_to_str(a, buf[which]);
    return buf[which];
}

/** Parameter builder for events (bounded; an overflow is a sim bug). */
typedef struct {
    uint8_t b[255];
    uint8_t n;
} sim_w_t;

static void w_u8(sim_w_t *w, uint8_t v) {
    if (w->n < sizeof w->b) {
        w->b[w->n++] = v;
    }
}
static void w_u16(sim_w_t *w, uint16_t v) {
    w_u8(w, (uint8_t)v);
    w_u8(w, (uint8_t)(v >> 8));
}
static void w_u32(sim_w_t *w, uint32_t v) {
    w_u16(w, (uint16_t)v);
    w_u16(w, (uint16_t)(v >> 16));
}
static void w_bytes(sim_w_t *w, const uint8_t *p, uint16_t n) {
    for (uint16_t i = 0; i < n; i++) {
        w_u8(w, p[i]);
    }
}
static void w_addr(sim_w_t *w, const floss_addr_t *a) {
    w_bytes(w, a->b, 6);
}

/* ------------------------------------------------------------------------ */
/* Delivery queue and event emission                                        */
/* ------------------------------------------------------------------------ */

static sim_pkt_t *q_push(uint8_t type, uint16_t len) {
    if (sim.q_count >= FLOSS_SIM_QUEUE_LEN || len > sizeof sim.q[0].data) {
        sim.stats.queue_overflows++;
        FLOSS_LOGE(TRANSPORT, "sim: delivery queue full, packet dropped");
        return NULL;
    }
    sim_pkt_t *p = &sim.q[(sim.q_head + sim.q_count) % FLOSS_SIM_QUEUE_LEN];
    sim.q_count++;
    p->type = type;
    p->len = len;
    p->cmd_credit = 0;
    p->credits[0] = 0;
    p->credits[1] = 0;
    return p;
}

/**
 * A real controller silently drops events the host did not enable. Codes
 * 0x01..0x3F map to Set Event Mask bit (code - 1), 0x40+ to page 2 bit
 * (code - 0x40), and LE subevents additionally need LE Set Event Mask bit
 * (subevent - 1). Command Complete, Command Status and Number Of Completed
 * Packets cannot be masked: their mask bits (13, 14, 18) are reserved.
 */
static bool evt_enabled(uint8_t code, const uint8_t *params, uint8_t len) {
    if (!sim.cfg.enforce_event_mask || code == 0 || code == FLOSS_EVT_COMMAND_COMPLETE ||
        code == FLOSS_EVT_COMMAND_STATUS || code == FLOSS_EVT_NUMBER_OF_COMPLETED_PACKETS) {
        return true;
    }
    bool on;
    if (code < 0x40) {
        on = (sim.evt_mask >> (code - 1)) & 1u;
    } else {
        on = (sim.evt_mask2 >> (code - 0x40)) & 1u;
    }
    if (on && code == FLOSS_EVT_LE_META && len > 0 && params[0] >= 1 && params[0] <= 64) {
        on = (sim.le_evt_mask >> (params[0] - 1)) & 1u;
    }
    return on;
}

static sim_pkt_t *emit_event(uint8_t code, const uint8_t *params, uint8_t len) {
    if (!evt_enabled(code, params, len)) {
        sim.stats.masked_events++;
        if (code == FLOSS_EVT_LE_META) {
            FLOSS_LOGW(TRANSPORT, "sim: LE subevent 0x%02x dropped: masked (LE Set Event Mask / Set Event Mask bit 61)",
                       (unsigned)params[0]);
        } else {
            FLOSS_LOGW(TRANSPORT, "sim: event 0x%02x dropped: masked by the host's Set Event Mask", (unsigned)code);
        }
        return NULL;
    }
    sim_pkt_t *p = q_push(FLOSS_PKT_EVT, (uint16_t)(len + 2));
    if (p) {
        p->data[0] = code;
        p->data[1] = len;
        if (len) {
            memcpy(p->data + 2, params, len);
        }
    }
    return p;
}

static void emit_w(uint8_t code, const sim_w_t *w) {
    (void)emit_event(code, w->b, w->n);
}

/** Command Complete: Num_HCI_Command_Packets = 1, opcode, return parameters. */
static void emit_cc(uint16_t opcode, const sim_w_t *ret) {
    sim_w_t w = {.n = 0};
    w_u8(&w, 1);
    w_u16(&w, opcode);
    w_bytes(&w, ret->b, ret->n);
    sim_pkt_t *p = emit_event(FLOSS_EVT_COMMAND_COMPLETE, w.b, w.n);
    if (p) {
        p->cmd_credit = 1;
    }
}

static void emit_cc_status(uint16_t opcode, uint8_t status) {
    sim_w_t r = {.n = 0};
    w_u8(&r, status);
    emit_cc(opcode, &r);
}

static void emit_cc_addr(uint16_t opcode, uint8_t status, const uint8_t *addr6) {
    sim_w_t r = {.n = 0};
    w_u8(&r, status);
    w_bytes(&r, addr6, 6);
    emit_cc(opcode, &r);
}

static void emit_cc_handle(uint16_t opcode, uint8_t status, uint16_t handle) {
    sim_w_t r = {.n = 0};
    w_u8(&r, status);
    w_u16(&r, handle);
    emit_cc(opcode, &r);
}

/** Command Status: the command was accepted (status 0) and completes later with its own event. */
static void emit_cs(uint16_t opcode, uint8_t status) {
    sim_w_t w = {.n = 0};
    w_u8(&w, status);
    w_u8(&w, 1);
    w_u16(&w, opcode);
    sim_pkt_t *p = emit_event(FLOSS_EVT_COMMAND_STATUS, w.b, w.n);
    if (p) {
        p->cmd_credit = 1;
    }
}

static void evt_addr_only(uint8_t code, const floss_addr_t *addr) {
    sim_w_t w = {.n = 0};
    w_addr(&w, addr);
    emit_w(code, &w);
}

static void evt_conn_complete(uint8_t status, uint16_t handle, const floss_addr_t *addr) {
    sim_w_t w = {.n = 0};
    w_u8(&w, status);
    w_u16(&w, handle);
    w_addr(&w, addr);
    w_u8(&w, 0x01); /* link type ACL */
    w_u8(&w, 0x00); /* encryption off */
    emit_w(FLOSS_EVT_CONNECTION_COMPLETE, &w);
}

static void evt_auth_complete(uint8_t status, uint16_t handle) {
    sim_w_t w = {.n = 0};
    w_u8(&w, status);
    w_u16(&w, handle);
    emit_w(FLOSS_EVT_AUTHENTICATION_COMPLETE, &w);
}

static void evt_enc_change(uint8_t status, uint16_t handle, uint8_t enabled) {
    sim_w_t w = {.n = 0};
    w_u8(&w, status);
    w_u16(&w, handle);
    w_u8(&w, enabled);
    emit_w(FLOSS_EVT_ENCRYPTION_CHANGE, &w);
}

static void evt_mode_change(uint16_t handle, uint8_t mode, uint16_t interval) {
    sim_w_t w = {.n = 0};
    w_u8(&w, 0);
    w_u16(&w, handle);
    w_u8(&w, mode);
    w_u16(&w, interval);
    emit_w(FLOSS_EVT_MODE_CHANGE, &w);
}

static void evt_role_change(const floss_addr_t *addr, uint8_t role) {
    sim_w_t w = {.n = 0};
    w_u8(&w, 0);
    w_addr(&w, addr);
    w_u8(&w, role);
    emit_w(FLOSS_EVT_ROLE_CHANGE, &w);
}

/* ------------------------------------------------------------------------ */
/* 2. Command log                                                           */
/* ------------------------------------------------------------------------ */

static void log_cmd(uint16_t opcode, const uint8_t *p, uint8_t len) {
    floss_sim_cmd_t *c = &sim.cmd_log[sim.cmd_total % FLOSS_SIM_CMD_LOG_LEN];
    c->seq = sim.cmd_total++;
    c->t_us = floss_hal_time_us();
    c->opcode = opcode;
    c->len = len;
    memcpy(c->params, p, FLOSS_MIN(len, FLOSS_SIM_CMD_PARAMS_MAX));
}

static uint32_t cmd_ring_count(void) {
    return sim.cmd_total < FLOSS_SIM_CMD_LOG_LEN ? sim.cmd_total : FLOSS_SIM_CMD_LOG_LEN;
}

void floss_sim_log_clear(void) {
    sim.cmd_total = 0;
    sim.sig_total = 0;
}

uint32_t floss_sim_cmd_total(void) {
    return sim.cmd_total;
}

const floss_sim_cmd_t *floss_sim_cmd_at(uint32_t i) {
    uint32_t n = cmd_ring_count();
    if (i >= n) {
        return NULL;
    }
    return &sim.cmd_log[(sim.cmd_total - n + i) % FLOSS_SIM_CMD_LOG_LEN];
}

uint32_t floss_sim_cmd_count(uint16_t opcode) {
    uint32_t count = 0;
    for (uint32_t i = 0; i < cmd_ring_count(); i++) {
        count += floss_sim_cmd_at(i)->opcode == opcode;
    }
    return count;
}

const floss_sim_cmd_t *floss_sim_cmd_first(uint16_t opcode) {
    for (uint32_t i = 0; i < cmd_ring_count(); i++) {
        if (floss_sim_cmd_at(i)->opcode == opcode) {
            return floss_sim_cmd_at(i);
        }
    }
    return NULL;
}

const floss_sim_cmd_t *floss_sim_cmd_last(uint16_t opcode) {
    for (uint32_t i = cmd_ring_count(); i-- > 0;) {
        if (floss_sim_cmd_at(i)->opcode == opcode) {
            return floss_sim_cmd_at(i);
        }
    }
    return NULL;
}

static bool cond_cmd_seen(void *arg) {
    return floss_sim_cmd_last((uint16_t)(uintptr_t)arg) != NULL;
}

const floss_sim_cmd_t *floss_sim_expect_cmd(uint16_t opcode) {
    floss_sim_pump_until(cond_cmd_seen, (void *)(uintptr_t)opcode, 0);
    return floss_sim_cmd_last(opcode);
}

static void sig_log(uint16_t handle, uint8_t code, uint8_t id, const uint8_t *data, uint16_t len) {
    floss_sim_sig_t *e = &sim.sig_log[sim.sig_total % FLOSS_SIM_SIG_LOG_LEN];
    e->seq = sim.sig_total++;
    e->handle = handle;
    e->code = code;
    e->id = id;
    e->len = len;
    memcpy(e->data, data, FLOSS_MIN(len, FLOSS_SIM_SIG_DATA_MAX));
}

const floss_sim_sig_t *floss_sim_sig_find(uint8_t code, uint8_t id) {
    uint32_t n = sim.sig_total < FLOSS_SIM_SIG_LOG_LEN ? sim.sig_total : FLOSS_SIM_SIG_LOG_LEN;
    for (uint32_t i = 0; i < n; i++) {
        const floss_sim_sig_t *e = &sim.sig_log[(sim.sig_total - 1 - i) % FLOSS_SIM_SIG_LOG_LEN];
        if (e->code == code && (id == 0 || e->id == id)) {
            return e;
        }
    }
    return NULL;
}

const floss_sim_sig_t *floss_sim_sig_last(uint8_t code) {
    return floss_sim_sig_find(code, 0);
}

void floss_sim_sig_clear(void) {
    sim.sig_total = 0;
}

/* ------------------------------------------------------------------------ */
/* 3. Peers and links                                                       */
/* ------------------------------------------------------------------------ */

floss_sim_peer_t *floss_sim_peer_find(const floss_addr_t *addr) {
    for (unsigned i = 0; i < FLOSS_SIM_MAX_PEERS; i++) {
        if (sim.peers[i].in_use && floss_addr_eq(&sim.peers[i].addr, addr)) {
            return &sim.peers[i];
        }
    }
    return NULL;
}

floss_sim_peer_t *floss_sim_peer_add(const floss_addr_t *addr, uint32_t cod) {
    floss_sim_peer_t *p = floss_sim_peer_find(addr);
    if (p) {
        if (cod) {
            p->cod = cod;
        }
        return p;
    }
    for (unsigned i = 0; i < FLOSS_SIM_MAX_PEERS; i++) {
        p = &sim.peers[i];
        if (!p->in_use) {
            memset(p, 0, sizeof *p);
            p->in_use = true;
            p->addr = *addr;
            p->cod = cod;
            p->present = true;
            p->ssp = true;
            p->sc = true;
            p->io_cap = FLOSS_IO_CAP_NO_INPUT_NO_OUTPUT;
            p->auto_encrypt = true;
            memcpy(p->pin, "0000", 5);
            memcpy(p->name, "FLOSS sim host", 15);
            return p;
        }
    }
    FLOSS_LOGE(TRANSPORT, "sim: peer table full");
    return NULL;
}

void floss_sim_peer_set_present(const floss_addr_t *addr, bool present) {
    floss_sim_peer_t *p = floss_sim_peer_add(addr, 0);
    if (p) {
        p->present = present;
    }
}

static floss_sim_peer_t *link_peer(sim_link_t *l) {
    floss_sim_peer_t *p = floss_sim_peer_find(&l->pub.addr);
    return p ? p : floss_sim_peer_add(&l->pub.addr, 0);
}

static sim_link_t *link_alloc(void) {
    for (unsigned i = 0; i < FLOSS_SIM_MAX_LINKS; i++) {
        sim_link_t *l = &sim.links[i];
        if (l->pub.state == FLOSS_SIM_LINK_FREE) {
            memset(l, 0, sizeof *l);
            l->next_sig_id = 1;
            l->pub.link_policy = sim.default_policy;
            return l;
        }
    }
    FLOSS_LOGE(TRANSPORT, "sim: no free link (FLOSS_SIM_MAX_LINKS)");
    return NULL;
}

static void link_assign_handle(sim_link_t *l) {
    l->pub.handle = sim.next_handle++;
    if (sim.next_handle > 0x0EFF) {
        sim.next_handle = 0x0040;
    }
}

/** Any non-free link with this handle (including one still waiting for Accept). */
static sim_link_t *link_find(uint16_t handle) {
    for (unsigned i = 0; i < FLOSS_SIM_MAX_LINKS; i++) {
        sim_link_t *l = &sim.links[i];
        if (l->pub.state != FLOSS_SIM_LINK_FREE && l->pub.handle == handle && l->pub.handle != 0) {
            return l;
        }
    }
    return NULL;
}

/** A link the stack knows about (Connection Complete sent). */
static sim_link_t *link_open(uint16_t handle) {
    sim_link_t *l = link_find(handle);
    return (l && l->pub.state == FLOSS_SIM_LINK_OPEN) ? l : NULL;
}

static sim_link_t *link_by_addr(const floss_addr_t *addr) {
    for (unsigned i = 0; i < FLOSS_SIM_MAX_LINKS; i++) {
        sim_link_t *l = &sim.links[i];
        if (l->pub.state != FLOSS_SIM_LINK_FREE && !l->pub.le && floss_addr_eq(&l->pub.addr, addr)) {
            return l;
        }
    }
    return NULL;
}

static sim_link_t *link_by_addr6(const uint8_t *p) {
    floss_addr_t a;
    memcpy(a.b, p, 6);
    sim_link_t *l = link_by_addr(&a);
    return (l && l->pub.state == FLOSS_SIM_LINK_OPEN) ? l : NULL;
}

/** Which ACL buffer pool a link draws from: LE has its own only if LE Read Buffer Size said so. */
static uint8_t link_pool(const sim_link_t *l) {
    return (l->pub.le && sim.cfg.le_acl_total > 0) ? 1 : 0;
}

static void zombie_add(uint16_t handle) {
    for (unsigned i = 0; i < SIM_ZOMBIES; i++) {
        if (sim.zombies[i] == 0) {
            sim.zombies[i] = handle;
            return;
        }
    }
}

static bool zombie_check(uint16_t handle, bool remove) {
    for (unsigned i = 0; i < SIM_ZOMBIES; i++) {
        if (handle != 0 && sim.zombies[i] == handle) {
            if (remove) {
                sim.zombies[i] = 0;
            }
            return true;
        }
    }
    return false;
}

static void link_free(sim_link_t *l) {
    for (unsigned i = 0; i < FLOSS_SIM_MAX_CHANS; i++) {
        sim_chan_t *c = &sim.chans[i];
        if (c->pub.state != FLOSS_SIM_CHAN_FREE && c->pub.state != FLOSS_SIM_CHAN_CLOSED &&
            c->pub.handle == l->pub.handle) {
            c->pub.state = FLOSS_SIM_CHAN_CLOSED;
        }
    }
    if (sim.gatt.active && sim.gatt.link == l->pub.handle) {
        sim.gatt.active = false; /* stats stay readable */
    }
    memset(l, 0, sizeof *l);
}

/**
 * Link gone (either side). The Disconnection Complete also hands back every
 * ACL buffer the link still held: the host reclaims them on this event
 * instead of waiting for Number Of Completed Packets.
 */
static void link_disconnected(sim_link_t *l, uint8_t reason) {
    sim_w_t w = {.n = 0};
    w_u8(&w, 0);
    w_u16(&w, l->pub.handle);
    w_u8(&w, reason);
    sim_pkt_t *p = emit_event(FLOSS_EVT_DISCONNECTION_COMPLETE, w.b, w.n);
    if (p) {
        p->credits[link_pool(l)] = l->done_count;
    }
    FLOSS_LOGI(TRANSPORT, "sim: link 0x%03x to %s closed, reason 0x%02x", (unsigned)l->pub.handle,
               addr_str(&l->pub.addr), (unsigned)reason);
    zombie_add(l->pub.handle);
    link_free(l);
}

/** Baseband link established: the stack learns the handle now. */
static void link_set_open(sim_link_t *l) {
    if (l->pub.handle == 0) {
        link_assign_handle(l);
    }
    l->pub.state = FLOSS_SIM_LINK_OPEN;
    evt_conn_complete(0, l->pub.handle, &l->pub.addr);
    FLOSS_LOGI(TRANSPORT, "sim: link 0x%03x to %s open (stack is %s)", (unsigned)l->pub.handle,
               addr_str(&l->pub.addr), l->pub.role ? "peripheral" : "central");
}

static void controller_reset(void);
static sim_chan_t *chan_alloc(uint16_t handle, uint16_t psm);
/* Section 11 (LE central support). */
static bool sim_le_central_cmd(uint16_t op, const uint8_t *p, uint8_t n);
static void sim_le_scan_tick(uint32_t now);
static void sim_gatt_rx(sim_link_t *l, const uint8_t *p, uint16_t n);
static void sim_gatt_on_remote_tx(uint16_t handle, const uint8_t *p, uint16_t n);

/* ------------------------------------------------------------------------ */
/* 4. Pairing                                                               */
/* ------------------------------------------------------------------------ */
/*
 * The controller runs the cryptography (LMP); the host only answers
 * questions. Which questions, and in which order, depends on who started:
 *
 *  stack starts (Authentication Requested)      remote starts (floss_sim_remote_pair)
 *  ---------------------------------------      -------------------------------------
 *  -> Link Key Request                          remote has a key:  -> Link Key Request
 *  <- reply: key ok => Authentication Complete  remote has no key: (straight to pairing)
 *  <- negative reply => pairing:                pairing as responder:
 *  -> IO Capability Request                     -> IO Capability Response (remote's caps)
 *  <- IO Capability Request Reply               -> IO Capability Request
 *  -> IO Capability Response (remote's caps)    <- IO Capability Request Reply
 *  -> User Confirmation Request                 -> User Confirmation Request
 *  <- User Confirmation Request Reply           <- User Confirmation Request Reply
 *  -> Simple Pairing Complete                   -> Simple Pairing Complete
 *  -> Link Key Notification                     -> Link Key Notification
 *  -> Authentication Complete                   -> Encryption Change (remote enables it)
 *
 * Without SSP on both sides the legacy path asks for a PIN instead.
 */

static uint8_t enc_mode_for_key(uint8_t key_type) {
    /* P-256 keys (Secure Connections) encrypt with AES-CCM, reported as 0x02 on BR/EDR. */
    return (key_type == 0x07 || key_type == 0x08) ? 0x02 : 0x01;
}

static void pair_success(sim_link_t *l, uint8_t key_type) {
    floss_sim_peer_t *peer = link_peer(l);
    l->pair_step = PAIR_NONE;
    l->pub.authenticated = true;
    l->pub.key_type = key_type;
    FLOSS_LOGI(TRANSPORT, "sim: link 0x%03x authenticated (key type 0x%02x)", (unsigned)l->pub.handle,
               (unsigned)key_type);
    if (l->pair_local) {
        evt_auth_complete(0, l->pub.handle);
    } else if (peer && peer->auto_encrypt) {
        l->pub.encrypted = enc_mode_for_key(key_type);
        evt_enc_change(0, l->pub.handle, l->pub.encrypted);
    }
}

/** May free the link (remote-initiated failure ends in a disconnect). */
static void pair_fail(sim_link_t *l, uint8_t status) {
    bool ssp = l->pair_step == PAIR_WAIT_IO_CAP || l->pair_step == PAIR_WAIT_CONFIRM;
    l->pair_step = PAIR_NONE;
    FLOSS_LOGI(TRANSPORT, "sim: pairing on 0x%03x failed, status 0x%02x", (unsigned)l->pub.handle, (unsigned)status);
    if (ssp) {
        sim_w_t w = {.n = 0};
        w_u8(&w, status);
        w_addr(&w, &l->pub.addr);
        emit_w(FLOSS_EVT_SIMPLE_PAIRING_COMPLETE, &w);
    }
    if (l->pair_local) {
        evt_auth_complete(status, l->pub.handle);
    } else {
        /* The remote host could not secure the link it wanted secured: it hangs up. */
        link_disconnected(l, FLOSS_HCI_ERR_AUTH_FAILURE);
    }
}

/** A new link key exists on both sides: tell our host, then finish. */
static void pair_new_key(sim_link_t *l, uint8_t key_type) {
    floss_sim_peer_t *peer = link_peer(l);
    peer->has_key = true;
    peer->key.type = key_type;
    sim_rand_bytes(peer->key.key, 16);

    sim_w_t w = {.n = 0};
    w_addr(&w, &l->pub.addr);
    w_bytes(&w, peer->key.key, 16);
    w_u8(&w, key_type);
    emit_w(FLOSS_EVT_LINK_KEY_NOTIFICATION, &w);
    pair_success(l, key_type);
}

static void evt_io_cap_response(sim_link_t *l, const floss_sim_peer_t *peer) {
    sim_w_t w = {.n = 0};
    w_addr(&w, &l->pub.addr);
    w_u8(&w, peer->io_cap);
    w_u8(&w, 0x00); /* no OOB data */
    w_u8(&w, 0x04); /* MITM not required, general bonding */
    emit_w(FLOSS_EVT_IO_CAPABILITY_RESPONSE, &w);
}

/** Start creating a new link key (no usable stored key). */
static void pair_new(sim_link_t *l) {
    floss_sim_peer_t *peer = link_peer(l);
    if (!sim.ssp_mode || !peer->ssp) {
        l->pair_step = PAIR_WAIT_PIN;
        evt_addr_only(FLOSS_EVT_PIN_CODE_REQUEST, &l->pub.addr);
        return;
    }
    l->pair_step = PAIR_WAIT_IO_CAP;
    if (!l->pair_local) {
        /* Responder: the initiator's IO capabilities arrive before we are asked for ours. */
        evt_io_cap_response(l, peer);
    }
    evt_addr_only(FLOSS_EVT_IO_CAPABILITY_REQUEST, &l->pub.addr);
}

static void pair_begin(sim_link_t *l, bool local) {
    floss_sim_peer_t *peer = link_peer(l);
    l->pair_local = local;
    FLOSS_LOGI(TRANSPORT, "sim: %s authenticates link 0x%03x", local ? "stack" : "remote", (unsigned)l->pub.handle);
    if (local || peer->has_key) {
        /* Authentication with a stored key: the controller asks its host for the key. */
        l->pair_step = PAIR_WAIT_LINK_KEY;
        evt_addr_only(FLOSS_EVT_LINK_KEY_REQUEST, &l->pub.addr);
    } else {
        pair_new(l);
    }
}

static void pair_on_link_key(sim_link_t *l, const uint8_t *key) {
    if (!l || l->pair_step != PAIR_WAIT_LINK_KEY) {
        FLOSS_LOGW(TRANSPORT, "sim: unexpected link key reply");
        return;
    }
    floss_sim_peer_t *peer = link_peer(l);
    if (key) {
        if (peer->has_key && memcmp(peer->key.key, key, 16) == 0) {
            pair_success(l, peer->key.type);
        } else {
            /* Remote lost the bond (key missing) or has a different one (failure). */
            pair_fail(l, peer->has_key ? FLOSS_HCI_ERR_AUTH_FAILURE : FLOSS_HCI_ERR_PIN_OR_KEY_MISSING);
        }
        return;
    }
    /* Our host has no key: pair afresh. A remote that offered its stored
     * key gives up on it and pairs again (what desktop hosts do after the
     * device was reset). */
    peer->has_key = false;
    pair_new(l);
}

static void pair_on_io_cap(sim_link_t *l, uint8_t io_cap) {
    if (!l || l->pair_step != PAIR_WAIT_IO_CAP) {
        FLOSS_LOGW(TRANSPORT, "sim: unexpected IO capability reply");
        return;
    }
    l->stack_io_cap = io_cap;
    if (l->pair_local) {
        evt_io_cap_response(l, link_peer(l));
    }
    /* Every association model the sim supports ends in a yes/no question:
     * just works (either side NoInputNoOutput) or numeric comparison. */
    l->pair_step = PAIR_WAIT_CONFIRM;
    sim_w_t w = {.n = 0};
    w_addr(&w, &l->pub.addr);
    w_u32(&w, sim_rand() % 1000000u);
    emit_w(FLOSS_EVT_USER_CONFIRMATION_REQUEST, &w);
}

static void pair_on_confirm(sim_link_t *l, bool accept) {
    if (!l || l->pair_step != PAIR_WAIT_CONFIRM) {
        FLOSS_LOGW(TRANSPORT, "sim: unexpected user confirmation reply");
        return;
    }
    if (!accept) {
        pair_fail(l, FLOSS_HCI_ERR_AUTH_FAILURE);
        return;
    }
    floss_sim_peer_t *peer = link_peer(l);
    bool sc = sim.sc_host && peer->sc;
    bool mitm = peer->io_cap == FLOSS_IO_CAP_DISPLAY_YES_NO && l->stack_io_cap == FLOSS_IO_CAP_DISPLAY_YES_NO;
    /* 0x04/0x05 unauthenticated/authenticated P-192, 0x07/0x08 the P-256 equivalents. */
    uint8_t key_type = sc ? (mitm ? 0x08 : 0x07) : (mitm ? 0x05 : 0x04);

    sim_w_t w = {.n = 0};
    w_u8(&w, 0);
    w_addr(&w, &l->pub.addr);
    emit_w(FLOSS_EVT_SIMPLE_PAIRING_COMPLETE, &w);
    pair_new_key(l, key_type);
}

static void pair_on_pin(sim_link_t *l, const uint8_t *pin, uint8_t pin_len) {
    if (!l || l->pair_step != PAIR_WAIT_PIN) {
        FLOSS_LOGW(TRANSPORT, "sim: unexpected PIN reply");
        return;
    }
    floss_sim_peer_t *peer = link_peer(l);
    if (!pin) {
        pair_fail(l, FLOSS_HCI_ERR_PIN_OR_KEY_MISSING);
    } else if (pin_len == strlen(peer->pin) && memcmp(pin, peer->pin, pin_len) == 0) {
        pair_new_key(l, 0x00); /* legacy combination key */
    } else {
        pair_fail(l, FLOSS_HCI_ERR_AUTH_FAILURE);
    }
}

/* ------------------------------------------------------------------------ */
/* 5. HCI command handlers                                                  */
/* ------------------------------------------------------------------------ */

/* Inside a handler: reply "Invalid HCI Command Parameters" if params are short. */
#define NEED(nbytes, async)                                  \
    do {                                                     \
        if (n < (nbytes)) {                                  \
            if (async) {                                     \
                emit_cs(op, SIM_ERR_INVALID_PARAMS);         \
            } else {                                         \
                emit_cc_status(op, SIM_ERR_INVALID_PARAMS);  \
            }                                                \
            return true;                                     \
        }                                                    \
    } while (0)

static void build_supported_commands(uint8_t out[64]) {
    /* Octets 0..31: everything up to the 4.1 command set. 32: Write Secure
     * Connections Host Support (bit 3). 33..35: LE 4.2 (data length, P-256,
     * resolving list, max data length). Nothing newer, so a host never
     * picks extended advertising or LE Read Buffer Size v2. */
    memset(out, 0, 64);
    memset(out, 0xFF, 32);
    out[32] = 0x08;
    out[33] = 0xF0;
    out[34] = 0xFF;
    out[35] = 0x0F;
}

static void cmd_create_connection(uint16_t op, const uint8_t *p) {
    floss_addr_t addr;
    memcpy(addr.b, p, 6);
    if (link_by_addr(&addr)) {
        emit_cs(op, SIM_ERR_ACL_EXISTS);
        return;
    }
    sim_link_t *l = link_alloc();
    if (!l) {
        emit_cs(op, SIM_ERR_CONN_LIMIT);
        return;
    }
    emit_cs(op, 0);
    l->pub.addr = addr;
    l->pub.stack_initiated = true;
    l->pub.role = 0; /* the pager starts as central */
    floss_sim_peer_t *peer = floss_sim_peer_find(&addr);
    if (peer && peer->present) {
        link_set_open(l); /* the remote answered the page scan at once */
    } else {
        l->pub.state = FLOSS_SIM_LINK_PAGING;
        l->page_due = floss_hal_time_us() + (uint32_t)sim.page_timeout * 625u;
        FLOSS_LOGI(TRANSPORT, "sim: paging %s: nobody answers, Page Timeout in %u ms", addr_str(&addr),
                   (unsigned)((uint32_t)sim.page_timeout * 625u / 1000u));
    }
}

/* OGF 0x01: link control */
static bool cmd_link_control(uint16_t op, const uint8_t *p, uint8_t n) {
    sim_link_t *l;
    sim_w_t w = {.n = 0};

    switch (op) {
    case FLOSS_HCI_INQUIRY:
        NEED(5, true);
        emit_cs(op, 0);
        w_u8(&w, 0);
        emit_w(FLOSS_EVT_INQUIRY_COMPLETE, &w);
        return true;

    case FLOSS_HCI_CREATE_CONNECTION:
        NEED(13, true);
        cmd_create_connection(op, p);
        return true;

    case FLOSS_HCI_CREATE_CONNECTION_CANCEL: {
        NEED(6, false);
        floss_addr_t a;
        memcpy(a.b, p, 6);
        l = link_by_addr(&a);
        if (!l || l->pub.state != FLOSS_SIM_LINK_PAGING) {
            emit_cc_addr(op, l ? SIM_ERR_ACL_EXISTS : FLOSS_HCI_ERR_UNKNOWN_CONN_ID, p);
            return true;
        }
        emit_cc_addr(op, 0, p);
        evt_conn_complete(FLOSS_HCI_ERR_UNKNOWN_CONN_ID, 0, &a);
        link_free(l);
        return true;
    }

    case FLOSS_HCI_ACCEPT_CONNECTION_REQUEST: {
        NEED(7, true);
        floss_addr_t a;
        memcpy(a.b, p, 6);
        l = link_by_addr(&a);
        if (!l || l->pub.state != FLOSS_SIM_LINK_REQUESTED) {
            emit_cs(op, FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
            return true;
        }
        emit_cs(op, 0);
        if (p[6] == 0x00) {
            /* Host asked to become central: role switch during setup. */
            l->pub.role = 0;
            evt_role_change(&l->pub.addr, 0);
        } else {
            l->pub.role = 1;
        }
        link_set_open(l);
        return true;
    }

    case FLOSS_HCI_REJECT_CONNECTION_REQUEST: {
        NEED(7, true);
        floss_addr_t a;
        memcpy(a.b, p, 6);
        l = link_by_addr(&a);
        if (!l || l->pub.state != FLOSS_SIM_LINK_REQUESTED) {
            emit_cs(op, FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
            return true;
        }
        emit_cs(op, 0);
        evt_conn_complete(p[6], l->pub.handle, &a);
        link_free(l);
        return true;
    }

    case FLOSS_HCI_DISCONNECT:
        NEED(3, true);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        if (!l) {
            emit_cs(op, FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
            return true;
        }
        emit_cs(op, 0);
        /* On the side that asked, the reason reads "terminated by local host". */
        link_disconnected(l, FLOSS_HCI_ERR_LOCAL_HOST_TERMINATED);
        return true;

    case FLOSS_HCI_AUTHENTICATION_REQUESTED:
        NEED(2, true);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        if (!l || l->pub.le) {
            emit_cs(op, FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
            return true;
        }
        if (l->pair_step != PAIR_NONE) {
            emit_cs(op, SIM_ERR_COMMAND_DISALLOWED);
            return true;
        }
        emit_cs(op, 0);
        pair_begin(l, true);
        return true;

    case FLOSS_HCI_SET_CONNECTION_ENCRYPTION:
        NEED(3, true);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        if (!l) {
            emit_cs(op, FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
            return true;
        }
        if (!l->pub.authenticated) {
            /* No link key yet: nothing to derive an encryption key from. */
            emit_cs(op, SIM_ERR_COMMAND_DISALLOWED);
            return true;
        }
        emit_cs(op, 0);
        l->pub.encrypted = p[2] ? enc_mode_for_key(l->pub.key_type) : 0;
        evt_enc_change(0, l->pub.handle, l->pub.encrypted);
        return true;

    case FLOSS_HCI_LINK_KEY_REQUEST_REPLY:
        NEED(22, false);
        emit_cc_addr(op, 0, p);
        pair_on_link_key(link_by_addr6(p), p + 6);
        return true;
    case FLOSS_HCI_LINK_KEY_REQUEST_NEG_REPLY:
        NEED(6, false);
        emit_cc_addr(op, 0, p);
        pair_on_link_key(link_by_addr6(p), NULL);
        return true;
    case FLOSS_HCI_PIN_CODE_REQUEST_REPLY:
        NEED(23, false);
        emit_cc_addr(op, 0, p);
        pair_on_pin(link_by_addr6(p), p + 7, FLOSS_MIN(p[6], 16));
        return true;
    case FLOSS_HCI_PIN_CODE_REQUEST_NEG_REPLY:
        NEED(6, false);
        emit_cc_addr(op, 0, p);
        pair_on_pin(link_by_addr6(p), NULL, 0);
        return true;
    case FLOSS_HCI_IO_CAPABILITY_REQUEST_REPLY:
        NEED(9, false);
        emit_cc_addr(op, 0, p);
        pair_on_io_cap(link_by_addr6(p), p[6]);
        return true;
    case FLOSS_HCI_IO_CAPABILITY_REQUEST_NEG_REPLY:
        NEED(7, false);
        emit_cc_addr(op, 0, p);
        l = link_by_addr6(p);
        if (l && l->pair_step == PAIR_WAIT_IO_CAP) {
            pair_fail(l, p[6]);
        }
        return true;
    case FLOSS_HCI_USER_CONFIRM_REQUEST_REPLY:
    case FLOSS_HCI_USER_PASSKEY_REQUEST_REPLY:
        NEED(6, false);
        emit_cc_addr(op, 0, p);
        pair_on_confirm(link_by_addr6(p), true);
        return true;
    case FLOSS_HCI_USER_CONFIRM_REQUEST_NEG_REPLY:
    case FLOSS_HCI_USER_PASSKEY_REQUEST_NEG_REPLY:
        NEED(6, false);
        emit_cc_addr(op, 0, p);
        pair_on_confirm(link_by_addr6(p), false);
        return true;

    case FLOSS_HCI_REMOTE_NAME_REQUEST: {
        NEED(10, true);
        floss_addr_t a;
        memcpy(a.b, p, 6);
        floss_sim_peer_t *peer = floss_sim_peer_find(&a);
        uint8_t name[248] = {0};
        emit_cs(op, 0);
        w_u8(&w, (peer && peer->present) ? 0 : FLOSS_HCI_ERR_PAGE_TIMEOUT);
        w_addr(&w, &a);
        if (peer) {
            memcpy(name, peer->name, strlen(peer->name));
        }
        w_bytes(&w, name, sizeof name);
        emit_w(FLOSS_EVT_REMOTE_NAME_REQUEST_COMPLETE, &w);
        return true;
    }
    case SIM_OP_REMOTE_NAME_CANCEL:
        NEED(6, false);
        emit_cc_addr(op, FLOSS_HCI_ERR_UNKNOWN_CONN_ID, p); /* names complete instantly */
        return true;

    case FLOSS_HCI_READ_REMOTE_FEATURES:
    case SIM_OP_READ_REMOTE_EXT_FEATURES:
    case FLOSS_HCI_READ_REMOTE_VERSION: {
        NEED(2, true);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        if (!l) {
            emit_cs(op, FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
            return true;
        }
        emit_cs(op, 0);
        w_u8(&w, 0);
        w_u16(&w, l->pub.handle);
        if (op == FLOSS_HCI_READ_REMOTE_FEATURES) {
            w_bytes(&w, sim_features, 8);
            emit_w(FLOSS_EVT_READ_REMOTE_FEATURES_COMPLETE, &w);
        } else if (op == SIM_OP_READ_REMOTE_EXT_FEATURES) {
            uint8_t page = n > 2 ? p[2] : 0;
            uint8_t feat[8] = {0};
            if (page == 0) {
                memcpy(feat, sim_features, 8);
            } else if (page == 1) {
                feat[0] = 0x0B; /* SSP host, LE host, SC host */
            }
            w_u8(&w, page);
            w_u8(&w, 2);
            w_bytes(&w, feat, 8);
            emit_w(FLOSS_EVT_READ_REMOTE_EXT_FEATURES_COMPLETE, &w);
        } else {
            w_u8(&w, 0x0B);    /* LMP 5.2 */
            w_u16(&w, 0xFFFF); /* manufacturer: reserved for testing */
            w_u16(&w, 0x0100);
            emit_w(FLOSS_EVT_READ_REMOTE_VERSION_COMPLETE, &w);
        }
        return true;
    }

    default:
        return false;
    }
}

/* OGF 0x02: link policy */
static bool cmd_link_policy(uint16_t op, const uint8_t *p, uint8_t n) {
    sim_link_t *l;
    sim_w_t w = {.n = 0};

    switch (op) {
    case FLOSS_HCI_SNIFF_MODE:
        NEED(10, true);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        if (!l) {
            emit_cs(op, FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
            return true;
        }
        if (!(l->pub.link_policy & FLOSS_LINK_POLICY_SNIFF) || l->pub.mode == FLOSS_LINK_MODE_SNIFF) {
            FLOSS_LOGW(TRANSPORT, "sim: Sniff Mode refused on 0x%03x (link policy 0x%04x does not allow sniff, "
                                  "or already sniffing)", (unsigned)l->pub.handle, (unsigned)l->pub.link_policy);
            emit_cs(op, SIM_ERR_COMMAND_DISALLOWED);
            return true;
        }
        emit_cs(op, 0);
        l->pub.mode = FLOSS_LINK_MODE_SNIFF;
        l->pub.interval = floss_get_le16(p + 2); /* the remote grants the maximum interval */
        evt_mode_change(l->pub.handle, l->pub.mode, l->pub.interval);
        return true;

    case FLOSS_HCI_EXIT_SNIFF_MODE:
        NEED(2, true);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        if (!l || l->pub.mode != FLOSS_LINK_MODE_SNIFF) {
            emit_cs(op, l ? SIM_ERR_COMMAND_DISALLOWED : FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
            return true;
        }
        emit_cs(op, 0);
        l->pub.mode = FLOSS_LINK_MODE_ACTIVE;
        l->pub.interval = 0;
        evt_mode_change(l->pub.handle, l->pub.mode, 0);
        return true;

    case FLOSS_HCI_QOS_SETUP:
        NEED(20, true);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        if (!l) {
            emit_cs(op, FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
            return true;
        }
        emit_cs(op, 0);
        w_u8(&w, 0);
        w_u16(&w, l->pub.handle);
        w_bytes(&w, p + 2, 18); /* granted exactly what was asked */
        emit_w(FLOSS_EVT_QOS_SETUP_COMPLETE, &w);
        return true;

    case FLOSS_HCI_ROLE_DISCOVERY:
        NEED(2, false);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        w_u8(&w, l ? 0 : FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
        w_u16(&w, floss_get_le16(p));
        w_u8(&w, l ? l->pub.role : 0);
        emit_cc(op, &w);
        return true;

    case FLOSS_HCI_SWITCH_ROLE:
        NEED(7, true);
        l = link_by_addr6(p);
        if (!l) {
            emit_cs(op, FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
            return true;
        }
        emit_cs(op, 0);
        l->pub.role = p[6];
        evt_role_change(&l->pub.addr, l->pub.role);
        return true;

    case FLOSS_HCI_WRITE_LINK_POLICY_SETTINGS:
        NEED(4, false);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        if (l) {
            l->pub.link_policy = floss_get_le16(p + 2);
        }
        emit_cc_handle(op, l ? 0 : FLOSS_HCI_ERR_UNKNOWN_CONN_ID, floss_get_le16(p));
        return true;

    case SIM_OP_READ_LINK_POLICY:
        NEED(2, false);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        w_u8(&w, l ? 0 : FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
        w_u16(&w, floss_get_le16(p));
        w_u16(&w, l ? l->pub.link_policy : 0);
        emit_cc(op, &w);
        return true;

    case FLOSS_HCI_WRITE_DEFAULT_LINK_POLICY:
        NEED(2, false);
        sim.default_policy = floss_get_le16(p);
        emit_cc_status(op, 0);
        return true;

    case FLOSS_HCI_SNIFF_SUBRATING:
        NEED(8, false);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        emit_cc_handle(op, l ? 0 : FLOSS_HCI_ERR_UNKNOWN_CONN_ID, floss_get_le16(p));
        return true;

    default:
        return false;
    }
}

/* OGF 0x03: controller & baseband */
static bool cmd_baseband(uint16_t op, const uint8_t *p, uint8_t n) {
    sim_link_t *l;
    sim_w_t w = {.n = 0};

    switch (op) {
    case FLOSS_HCI_RESET:
        controller_reset();
        sim.cmd_outstanding = 1; /* the Reset itself is still waiting for its Complete */
        FLOSS_LOGI(TRANSPORT, "sim: HCI Reset");
        emit_cc_status(op, 0);
        return true;

    case FLOSS_HCI_SET_EVENT_MASK:
        NEED(8, false);
        sim.evt_mask = get_le64(p);
        break;
    case FLOSS_HCI_SET_EVENT_MASK_PAGE_2:
        NEED(8, false);
        sim.evt_mask2 = get_le64(p);
        break;

    case FLOSS_HCI_WRITE_LOCAL_NAME: {
        uint8_t len = FLOSS_MIN(n, 248);
        memcpy(sim.name, p, len);
        sim.name[len] = '\0';
        break;
    }
    case FLOSS_HCI_READ_LOCAL_NAME: {
        uint8_t name[248] = {0};
        memcpy(name, sim.name, strlen(sim.name));
        w_u8(&w, 0);
        w_bytes(&w, name, sizeof name);
        emit_cc(op, &w);
        return true;
    }

    case FLOSS_HCI_WRITE_SCAN_ENABLE:
        NEED(1, false);
        sim.scan = p[0];
        FLOSS_LOGI(TRANSPORT, "sim: scan enable 0x%02x (%s%s)", (unsigned)sim.scan,
                   (sim.scan & 1) ? "discoverable " : "", (sim.scan & 2) ? "connectable" : "");
        break;
    case FLOSS_HCI_WRITE_CLASS_OF_DEVICE:
        NEED(3, false);
        sim.cod = floss_get_le24(p);
        break;
    case FLOSS_HCI_WRITE_PAGE_TIMEOUT:
        NEED(2, false);
        sim.page_timeout = floss_get_le16(p);
        break;
    case FLOSS_HCI_WRITE_SIMPLE_PAIRING_MODE:
        NEED(1, false);
        sim.ssp_mode = p[0] != 0;
        break;
    case FLOSS_HCI_WRITE_SECURE_CONNECTIONS_HOST_SUPPORT:
        NEED(1, false);
        sim.sc_host = p[0] != 0;
        break;
    case FLOSS_HCI_WRITE_LE_HOST_SUPPORT:
        NEED(1, false);
        sim.le_host = p[0] != 0;
        break;

    case FLOSS_HCI_WRITE_AUTOMATIC_FLUSH_TIMEOUT:
        NEED(4, false);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        if (l) {
            l->pub.flush_timeout = floss_get_le16(p + 2);
        }
        emit_cc_handle(op, l ? 0 : FLOSS_HCI_ERR_UNKNOWN_CONN_ID, floss_get_le16(p));
        return true;
    case FLOSS_HCI_WRITE_LINK_SUPERVISION_TIMEOUT:
    case SIM_OP_FLUSH:
        NEED(2, false);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        emit_cc_handle(op, l ? 0 : FLOSS_HCI_ERR_UNKNOWN_CONN_ID, floss_get_le16(p));
        return true;

    default:
        return false;
    }
    emit_cc_status(op, 0);
    return true;
}

/* OGF 0x04 informational, 0x05 status */
static bool cmd_info(uint16_t op, const uint8_t *p, uint8_t n) {
    sim_w_t w = {.n = 0};
    sim_link_t *l;

    switch (op) {
    case FLOSS_HCI_READ_LOCAL_VERSION:
        w_u8(&w, 0);
        w_u8(&w, 0x0B);    /* HCI 5.2 */
        w_u16(&w, 0x0100); /* HCI revision */
        w_u8(&w, 0x0B);    /* LMP 5.2 */
        w_u16(&w, 0xFFFF); /* manufacturer: reserved for testing */
        w_u16(&w, 0x0100); /* LMP subversion */
        break;
    case FLOSS_HCI_READ_LOCAL_COMMANDS: {
        uint8_t cmds[64];
        build_supported_commands(cmds);
        w_u8(&w, 0);
        w_bytes(&w, cmds, 64);
        break;
    }
    case FLOSS_HCI_READ_LOCAL_FEATURES:
        w_u8(&w, 0);
        w_bytes(&w, sim_features, 8);
        break;
    case SIM_OP_READ_LOCAL_EXT_FEATURES: {
        NEED(1, false);
        uint8_t feat[8] = {0};
        if (p[0] == 0) {
            memcpy(feat, sim_features, 8);
        } else if (p[0] == 1) {
            /* Host features page: mirrors what the host enabled. */
            feat[0] = (uint8_t)((sim.ssp_mode ? 0x01 : 0) | (sim.le_host ? 0x02 : 0) | (sim.sc_host ? 0x08 : 0));
        } else if (p[0] == 2) {
            feat[1] = 0x03; /* Secure Connections (controller), ping */
        } else {
            emit_cc_status(op, SIM_ERR_INVALID_PARAMS);
            return true;
        }
        w_u8(&w, 0);
        w_u8(&w, p[0]);
        w_u8(&w, 2);
        w_bytes(&w, feat, 8);
        break;
    }
    case FLOSS_HCI_READ_BUFFER_SIZE:
        w_u8(&w, 0);
        w_u16(&w, sim.cfg.acl_mtu);
        w_u8(&w, 64); /* SCO packet length */
        w_u16(&w, sim.cfg.acl_total);
        w_u16(&w, 0); /* no SCO buffers */
        break;
    case FLOSS_HCI_READ_BD_ADDR:
        w_u8(&w, 0);
        w_addr(&w, &sim.addr);
        break;
    case FLOSS_HCI_READ_RSSI:
        NEED(2, false);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        w_u8(&w, l ? 0 : FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
        w_u16(&w, floss_get_le16(p));
        w_u8(&w, (uint8_t)(int8_t)-42);
        break;
    default:
        return false;
    }
    emit_cc(op, &w);
    return true;
}

/** LE link established: fixed channels exist from the first moment. */
static void le_link_complete(sim_link_t *l, uint8_t addr_type, uint16_t interval, uint16_t latency,
                             uint16_t timeout) {
    static const uint16_t fixed[] = {FLOSS_SIM_CID_ATT, FLOSS_SIM_CID_LE_SIG, FLOSS_SIM_CID_SMP};
    link_assign_handle(l);
    l->pub.state = FLOSS_SIM_LINK_OPEN;
    for (unsigned i = 0; i < FLOSS_ARRAY_SIZE(fixed); i++) {
        sim_chan_t *c = chan_alloc(l->pub.handle, 0);
        if (c) {
            c->fixed = true;
            c->pub.sim_cid = fixed[i];
            c->pub.stack_cid = fixed[i];
            c->pub.stack_mtu = FLOSS_SIM_ACL_MAX;
            c->pub.sim_mtu = FLOSS_SIM_ACL_MAX;
            c->pub.state = FLOSS_SIM_CHAN_OPEN;
        }
    }
    sim_w_t w = {.n = 0};
    w_u8(&w, SIM_LE_SUB_CONN_COMPLETE);
    w_u8(&w, 0);
    w_u16(&w, l->pub.handle);
    w_u8(&w, l->pub.role);
    w_u8(&w, addr_type);
    w_addr(&w, &l->pub.addr);
    w_u16(&w, interval);
    w_u16(&w, latency);
    w_u16(&w, timeout);
    w_u8(&w, 0); /* central clock accuracy */
    emit_w(FLOSS_EVT_LE_META, &w);
    FLOSS_LOGI(TRANSPORT, "sim: LE link 0x%03x to %s open (stack is %s)", (unsigned)l->pub.handle,
               addr_str(&l->pub.addr), l->pub.role ? "peripheral" : "central");
}

/* OGF 0x08: LE controller */
static bool cmd_le(uint16_t op, const uint8_t *p, uint8_t n) {
    sim_w_t w = {.n = 0};
    sim_link_t *l;

    switch (op) {
    case FLOSS_HCI_LE_SET_EVENT_MASK:
        NEED(8, false);
        sim.le_evt_mask = get_le64(p);
        emit_cc_status(op, 0);
        return true;
    case FLOSS_HCI_LE_READ_BUFFER_SIZE:
        w_u8(&w, 0);
        w_u16(&w, sim.cfg.le_acl_total ? sim.cfg.le_acl_mtu : 0);
        w_u8(&w, sim.cfg.le_acl_total);
        break;
    case FLOSS_HCI_LE_READ_LOCAL_FEATURES:
        w_u8(&w, 0);
        w_bytes(&w, sim_le_features, 8);
        break;
    case SIM_OP_LE_READ_SUPPORTED_STATES: {
        static const uint8_t states[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0, 0};
        w_u8(&w, 0);
        w_bytes(&w, states, 8);
        break;
    }
    case SIM_OP_LE_RAND: {
        uint8_t r[8];
        sim_rand_bytes(r, 8);
        w_u8(&w, 0);
        w_bytes(&w, r, 8);
        break;
    }
    case SIM_OP_LE_ENCRYPT: {
        /* NOT AES: a deterministic stand-in so hosts get well-formed answers. */
        NEED(32, false);
        w_u8(&w, 0);
        for (unsigned i = 0; i < 16; i++) {
            w_u8(&w, (uint8_t)(p[i] ^ p[16 + i] ^ (uint8_t)(0x5A + i)));
        }
        break;
    }
    case SIM_OP_LE_READ_ADV_TX_POWER:
        w_u8(&w, 0);
        w_u8(&w, 0);
        break;
    case SIM_OP_LE_READ_ACCEPT_LIST_SIZE:
    case SIM_OP_LE_READ_RESOLVING_LIST_SIZE:
        w_u8(&w, 0);
        w_u8(&w, 8);
        break;
    case SIM_OP_LE_READ_DEFAULT_DATA_LENGTH:
        w_u8(&w, 0);
        w_u16(&w, 27);
        w_u16(&w, 328);
        break;
    case SIM_OP_LE_READ_MAX_DATA_LENGTH:
        w_u8(&w, 0);
        w_u16(&w, 251);
        w_u16(&w, 2120);
        w_u16(&w, 251);
        w_u16(&w, 2120);
        break;
    case SIM_OP_LE_SET_DATA_LENGTH:
    case SIM_OP_LE_LTK_REPLY:
    case SIM_OP_LE_LTK_NEG_REPLY:
        NEED(2, false);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        emit_cc_handle(op, l ? 0 : FLOSS_HCI_ERR_UNKNOWN_CONN_ID, floss_get_le16(p));
        return true;

    case SIM_OP_LE_CREATE_CONNECTION: {
        NEED(25, true);
        if (sim.le_scan.enabled || sim.le_connecting) {
            /* Initiating while scanning (or initiating twice) is refused by real controllers. */
            emit_cs(op, SIM_ERR_COMMAND_DISALLOWED);
            return true;
        }
        floss_addr_t a;
        memcpy(a.b, p + 6, 6);
        floss_sim_peer_t *peer = floss_sim_peer_find(&a);
        emit_cs(op, 0);
        if (peer && peer->present && (l = link_alloc()) != NULL) {
            l->pub.le = true;
            l->pub.addr = a;
            l->pub.role = 0;
            l->pub.stack_initiated = true;
            le_link_complete(l, p[5], floss_get_le16(p + 15), floss_get_le16(p + 17), floss_get_le16(p + 19));
        } else {
            sim.le_connecting = true; /* keeps "scanning" until cancelled */
        }
        return true;
    }
    case SIM_OP_LE_CREATE_CONNECTION_CANCEL:
        if (!sim.le_connecting) {
            emit_cc_status(op, SIM_ERR_COMMAND_DISALLOWED);
            return true;
        }
        sim.le_connecting = false;
        emit_cc_status(op, 0);
        w_u8(&w, SIM_LE_SUB_CONN_COMPLETE);
        w_u8(&w, FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
        while (w.n < 19) {
            w_u8(&w, 0);
        }
        emit_w(FLOSS_EVT_LE_META, &w);
        return true;

    case SIM_OP_LE_CONNECTION_UPDATE:
        NEED(14, true);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        if (!l || !l->pub.le) {
            emit_cs(op, FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
            return true;
        }
        emit_cs(op, 0);
        w_u8(&w, SIM_LE_SUB_CONN_UPDATE);
        w_u8(&w, 0);
        w_u16(&w, l->pub.handle);
        w_u16(&w, floss_get_le16(p + 4)); /* interval max */
        w_u16(&w, floss_get_le16(p + 6)); /* latency */
        w_u16(&w, floss_get_le16(p + 8)); /* supervision timeout */
        emit_w(FLOSS_EVT_LE_META, &w);
        return true;

    case SIM_OP_LE_READ_REMOTE_FEATURES:
        NEED(2, true);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        if (!l || !l->pub.le) {
            emit_cs(op, FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
            return true;
        }
        emit_cs(op, 0);
        w_u8(&w, SIM_LE_SUB_REMOTE_FEATURES);
        w_u8(&w, 0);
        w_u16(&w, l->pub.handle);
        w_bytes(&w, sim_le_features, 8);
        emit_w(FLOSS_EVT_LE_META, &w);
        return true;

    case SIM_OP_LE_START_ENCRYPTION:
        NEED(28, true);
        l = link_open(floss_get_le16(p) & 0x0FFF);
        if (!l || !l->pub.le) {
            emit_cs(op, FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
            return true;
        }
        emit_cs(op, 0);
        l->pub.encrypted = 1;
        evt_enc_change(0, l->pub.handle, 1);
        return true;

    case SIM_OP_LE_READ_P256_KEY:
    case SIM_OP_LE_GENERATE_DHKEY: {
        /* Well-formed but NOT real elliptic-curve results. */
        uint8_t key[64];
        uint8_t len = op == SIM_OP_LE_READ_P256_KEY ? 64 : 32;
        sim_rand_bytes(key, len);
        emit_cs(op, 0);
        w_u8(&w, op == SIM_OP_LE_READ_P256_KEY ? SIM_LE_SUB_P256_KEY : SIM_LE_SUB_DHKEY);
        w_u8(&w, 0);
        w_bytes(&w, key, len);
        emit_w(FLOSS_EVT_LE_META, &w);
        return true;
    }

    case SIM_OP_LE_SET_SCAN_PARAMS:
    case SIM_OP_LE_SET_SCAN_ENABLE:
    case SIM_OP_LE_SET_PHY:
        return sim_le_central_cmd(op, p, n);

    default:
        return false;
    }
    emit_cc(op, &w);
    return true;
}

/** Every command gets exactly one Command Complete or Command Status. */
static void sim_rx_cmd(const uint8_t *pkt, uint16_t len) {
    if (len < 3 || pkt[2] != len - 3) {
        sim.stats.malformed++;
        FLOSS_LOGW(TRANSPORT, "sim: malformed command packet (len %u)", (unsigned)len);
        return;
    }
    uint16_t op = floss_get_le16(pkt);
    uint8_t n = pkt[2];
    const uint8_t *p = pkt + 3;

    sim.stats.cmds++;
    log_cmd(op, p, n);
    FLOSS_LOGD(TRANSPORT, "sim: <- cmd 0x%04x (%u bytes)", (unsigned)op, (unsigned)n);
    if (sim.cmd_outstanding >= 1) {
        sim.stats.cmd_credit_violations++;
        FLOSS_LOGW(TRANSPORT, "sim: command 0x%04x sent without a command credit (Num_HCI_Command_Packets = 1)",
                   (unsigned)op);
    }
    sim.cmd_outstanding++;

    bool handled;
    switch (op >> 10) {
    case 0x01:
        handled = cmd_link_control(op, p, n);
        break;
    case 0x02:
        handled = cmd_link_policy(op, p, n);
        break;
    case 0x03:
        handled = cmd_baseband(op, p, n);
        break;
    case 0x04:
    case 0x05:
        handled = cmd_info(op, p, n);
        break;
    case 0x08:
        handled = cmd_le(op, p, n);
        break;
    default:
        handled = false;
        break;
    }
    if (handled) {
        return;
    }
    if (op == SIM_OP_VENDOR_SET_ADDR && n >= 6) {
        memcpy(sim.addr.b, p, 6);
        FLOSS_LOGI(TRANSPORT, "sim: address set to %s", addr_str(&sim.addr));
    }
    /* Anything else: accepted, nothing to report beyond the status. */
    emit_cc_status(op, 0);
}

/* ------------------------------------------------------------------------ */
/* 6. ACL from the stack                                                    */
/* ------------------------------------------------------------------------ */

static void l2cap_rx(sim_link_t *l, const uint8_t *pdu, uint16_t len);

/**
 * Rebuild L2CAP PDUs from ACL fragments. Packet boundary flag: 0b00 first
 * fragment (non-flushable), 0b10 first fragment (automatically flushable;
 * BR/EDR only), 0b01 continuation. A new start while a PDU is unfinished
 * means the stack interleaved or lost fragments.
 */
static void acl_reassemble(sim_link_t *l, uint8_t pb, const uint8_t *d, uint16_t n) {
    if (pb == 0x01) {
        if (!l->rx_active) {
            sim.stats.acl_pb_violations++;
            FLOSS_LOGW(TRANSPORT, "sim: continuation fragment without a start on 0x%03x", (unsigned)l->pub.handle);
            return;
        }
    } else {
        if (pb == 0x03 || (pb == 0x02 && l->pub.le)) {
            sim.stats.acl_pb_violations++;
            FLOSS_LOGW(TRANSPORT, "sim: packet boundary flag %u not allowed host->controller here", (unsigned)pb);
        }
        if (pb == 0x02) {
            sim.stats.acl_in_flushable++;
        }
        if (l->rx_active) {
            sim.stats.acl_pb_violations++;
            FLOSS_LOGW(TRANSPORT, "sim: new PDU started before the previous one finished on 0x%03x",
                       (unsigned)l->pub.handle);
        }
        l->rx_active = true;
        l->rx_len = 0;
    }
    if ((uint32_t)l->rx_len + n > sizeof l->rx) {
        sim.stats.malformed++;
        l->rx_active = false;
        return;
    }
    memcpy(l->rx + l->rx_len, d, n);
    l->rx_len = (uint16_t)(l->rx_len + n);
    if (l->rx_len < FLOSS_L2CAP_HDR_LEN) {
        return;
    }
    uint32_t need = FLOSS_L2CAP_HDR_LEN + (uint32_t)floss_get_le16(l->rx);
    if (l->rx_len < need) {
        return;
    }
    l->rx_active = false;
    if (l->rx_len > need) {
        sim.stats.malformed++;
        FLOSS_LOGW(TRANSPORT, "sim: ACL data longer than the L2CAP length on 0x%03x", (unsigned)l->pub.handle);
    }
    l2cap_rx(l, l->rx, (uint16_t)need);
}

static void sim_rx_acl(const uint8_t *pkt, uint16_t len) {
    if (len < 4 || floss_get_le16(pkt + 2) != len - 4) {
        sim.stats.malformed++;
        FLOSS_LOGW(TRANSPORT, "sim: malformed ACL packet (len %u)", (unsigned)len);
        return;
    }
    uint16_t handle = floss_get_le16(pkt) & 0x0FFF;
    uint8_t pb = (uint8_t)((floss_get_le16(pkt) >> 12) & 0x03);
    uint16_t dlen = (uint16_t)(len - 4);

    sim_link_t *l = link_open(handle);
    if (!l) {
        if (zombie_check(handle, false)) {
            /* Raced with a disconnect the stack has not heard about yet: legal, dropped. */
            FLOSS_LOGD(TRANSPORT, "sim: ACL for closing link 0x%03x dropped", (unsigned)handle);
        } else {
            sim.stats.acl_bad_handle++;
            FLOSS_LOGW(TRANSPORT, "sim: ACL for unknown handle 0x%03x", (unsigned)handle);
        }
        return;
    }

    sim.stats.acl_in++;
    uint8_t pool = link_pool(l);
    uint16_t total = pool ? sim.cfg.le_acl_total : sim.cfg.acl_total;
    uint16_t mtu = pool ? sim.cfg.le_acl_mtu : sim.cfg.acl_mtu;
    if (sim.pool_out[pool] >= total) {
        sim.stats.acl_credit_violations++;
        FLOSS_LOGW(TRANSPORT, "sim: ACL on 0x%03x with all %u controller buffers in use", (unsigned)handle,
                   (unsigned)total);
    }
    if (dlen > mtu) {
        sim.stats.acl_size_violations++;
        FLOSS_LOGW(TRANSPORT, "sim: ACL payload %u > controller ACL MTU %u", (unsigned)dlen, (unsigned)mtu);
    }
    sim.pool_out[pool]++;

    /* The packet occupies a controller buffer until it has been "sent over the air". */
    if (l->done_count < SIM_INFLIGHT_MAX) {
        l->done_due[(l->done_head + l->done_count) % SIM_INFLIGHT_MAX] = floss_hal_time_us() + sim.cfg.nocp_delay_us;
        l->done_count++;
    }
    acl_reassemble(l, pb, pkt + 4, dlen);
}

/**
 * Time-driven controller work, run at the start of every transport poll:
 * page timeouts and Number Of Completed Packets. All completions that are
 * due are coalesced into a single event (one entry per handle).
 */
static void sim_tick(void) {
    uint32_t now = floss_hal_time_us();

    for (unsigned i = 0; i < FLOSS_SIM_MAX_LINKS; i++) {
        sim_link_t *l = &sim.links[i];
        if (l->pub.state == FLOSS_SIM_LINK_PAGING && floss_time_diff(now, l->page_due) >= 0) {
            FLOSS_LOGI(TRANSPORT, "sim: page to %s timed out", addr_str(&l->pub.addr));
            evt_conn_complete(FLOSS_HCI_ERR_PAGE_TIMEOUT, 0, &l->pub.addr);
            link_free(l);
        }
    }

    sim_w_t w = {.n = 0};
    uint16_t credits[2] = {0, 0};
    uint8_t handles = 0;
    w_u8(&w, 0); /* Num_Handles, patched below */
    for (unsigned i = 0; i < FLOSS_SIM_MAX_LINKS; i++) {
        sim_link_t *l = &sim.links[i];
        uint16_t done = 0;
        while (l->pub.state == FLOSS_SIM_LINK_OPEN && l->done_count &&
               floss_time_diff(now, l->done_due[l->done_head]) >= 0) {
            l->done_head = (uint8_t)((l->done_head + 1) % SIM_INFLIGHT_MAX);
            l->done_count--;
            done++;
        }
        if (done) {
            w_u16(&w, l->pub.handle);
            w_u16(&w, done);
            credits[link_pool(l)] = (uint16_t)(credits[link_pool(l)] + done);
            handles++;
        }
    }
    if (handles) {
        w.b[0] = handles;
        sim_pkt_t *p = emit_event(FLOSS_EVT_NUMBER_OF_COMPLETED_PACKETS, w.b, w.n);
        if (p) {
            p->credits[0] = credits[0];
            p->credits[1] = credits[1];
        }
        sim.stats.nocp_events++;
    }

    sim_le_scan_tick(now);
}

/* ------------------------------------------------------------------------ */
/* 7. Remote L2CAP peer                                                     */
/* ------------------------------------------------------------------------ */

static void acl_to_stack(sim_link_t *l, const uint8_t *pdu, uint16_t len);

static sim_chan_t *chan_alloc(uint16_t handle, uint16_t psm) {
    sim_chan_t *best = NULL;
    for (unsigned i = 0; i < FLOSS_SIM_MAX_CHANS; i++) {
        sim_chan_t *c = &sim.chans[i];
        if (c->pub.state == FLOSS_SIM_CHAN_FREE) {
            best = c;
            break;
        }
        if (c->pub.state == FLOSS_SIM_CHAN_CLOSED && (!best || c->pub.seq < best->pub.seq)) {
            best = c; /* recycle the oldest closed channel */
        }
    }
    if (!best) {
        FLOSS_LOGE(TRANSPORT, "sim: no free channel (FLOSS_SIM_MAX_CHANS)");
        return NULL;
    }
    memset(best, 0, sizeof *best);
    best->pub.handle = handle;
    best->pub.psm = psm;
    best->pub.sim_cid = sim.next_sim_cid++;
    if (sim.next_sim_cid < FLOSS_CID_DYNAMIC_FIRST) {
        sim.next_sim_cid = FLOSS_CID_DYNAMIC_FIRST;
    }
    best->pub.stack_mtu = L2CAP_DEFAULT_MTU; /* until the stack's configuration says otherwise */
    best->pub.sim_mtu = sim.cfg.l2cap_mtu;
    best->pub.stack_flush_timeout = 0xFFFF;
    best->pub.seq = ++sim.chan_seq;
    return best;
}

static sim_chan_t *chan_by_sim_cid(uint16_t handle, uint16_t cid) {
    for (unsigned i = 0; i < FLOSS_SIM_MAX_CHANS; i++) {
        sim_chan_t *c = &sim.chans[i];
        if (c->pub.state != FLOSS_SIM_CHAN_FREE && c->pub.state != FLOSS_SIM_CHAN_CLOSED &&
            c->pub.handle == handle && c->pub.sim_cid == cid) {
            return c;
        }
    }
    return NULL;
}

static void inbox_push(sim_chan_t *c, const uint8_t *d, uint16_t len) {
    c->pub.rx_count++;
    if (c->in_count == FLOSS_SIM_INBOX_LEN) {
        c->in_head = (uint8_t)((c->in_head + 1) % FLOSS_SIM_INBOX_LEN);
        c->in_count--;
        c->pub.rx_dropped++;
        sim.stats.inbox_overflows++;
    }
    uint8_t idx = (uint8_t)((c->in_head + c->in_count) % FLOSS_SIM_INBOX_LEN);
    uint16_t keep = FLOSS_MIN(len, FLOSS_SIM_ACL_MAX);
    memcpy(c->in_data[idx], d, keep);
    c->in_len[idx] = keep;
    c->in_count++;
}

static uint8_t sig_next_id(sim_link_t *l) {
    uint8_t id = l->next_sig_id++;
    if (l->next_sig_id == 0) {
        l->next_sig_id = 1; /* identifier 0 is never valid */
    }
    return id;
}

static void sig_send(sim_link_t *l, uint8_t code, uint8_t id, const uint8_t *data, uint16_t len) {
    uint8_t pdu[8 + FLOSS_SIM_ACL_MAX];
    len = FLOSS_MIN(len, FLOSS_SIM_ACL_MAX);
    floss_put_le16(pdu, (uint16_t)(4 + len));
    floss_put_le16(pdu + 2, FLOSS_CID_SIGNALING);
    pdu[4] = code;
    pdu[5] = id;
    floss_put_le16(pdu + 6, len);
    if (len) {
        memcpy(pdu + 8, data, len);
    }
    acl_to_stack(l, pdu, (uint16_t)(8 + len));
}

static void sig_reject(sim_link_t *l, uint8_t id, uint16_t reason, uint16_t cid_a, uint16_t cid_b) {
    uint8_t d[6];
    floss_put_le16(d, reason);
    floss_put_le16(d + 2, cid_a);
    floss_put_le16(d + 4, cid_b);
    sig_send(l, SIG_CMD_REJECT, id, d, reason == 0x0002 ? 6 : 2);
}

/** Our Configuration Request: just the MTU we can receive. */
static void sig_send_conf_req(sim_link_t *l, sim_chan_t *c, uint16_t mtu) {
    uint8_t d[8];
    floss_put_le16(d, c->pub.stack_cid); /* destination = the receiver's (stack's) CID */
    floss_put_le16(d + 2, 0);            /* flags: no continuation */
    d[4] = L2CAP_OPT_MTU;
    d[5] = 2;
    floss_put_le16(d + 6, mtu);
    c->pub.sim_mtu = mtu;
    c->sig_id = sig_next_id(l);
    sig_send(l, SIG_CONF_REQ, c->sig_id, d, sizeof d);
}

static void chan_send_disc_req(sim_link_t *l, sim_chan_t *c) {
    uint8_t d[4];
    floss_put_le16(d, c->pub.stack_cid); /* DCID: endpoint on the receiver */
    floss_put_le16(d + 2, c->pub.sim_cid);
    c->sig_id = sig_next_id(l);
    c->pub.state = FLOSS_SIM_CHAN_WAIT_DISCONNECT;
    sig_send(l, SIG_DISC_REQ, c->sig_id, d, sizeof d);
}

static void chan_check_open(sim_chan_t *c) {
    if (c->pub.state == FLOSS_SIM_CHAN_CONFIG && c->cfg_in_done && c->cfg_out_done) {
        c->pub.state = FLOSS_SIM_CHAN_OPEN;
        FLOSS_LOGI(TRANSPORT, "sim: L2CAP PSM 0x%04x open (sim cid 0x%04x, stack cid 0x%04x, stack MTU %u)",
                   (unsigned)c->pub.psm, (unsigned)c->pub.sim_cid, (unsigned)c->pub.stack_cid,
                   (unsigned)c->pub.stack_mtu);
    }
}

/** The stack opens a channel to the remote (e.g. HID reconnect). The remote supports every PSM. */
static void sig_on_conn_req(sim_link_t *l, uint8_t id, uint16_t psm, uint16_t scid) {
    uint8_t rsp[8];
    sim_chan_t *c = chan_alloc(l->pub.handle, psm);
    floss_put_le16(rsp + 2, scid);
    floss_put_le16(rsp + 6, 0);
    if (!c) {
        floss_put_le16(rsp, 0);
        floss_put_le16(rsp + 4, 0x0004); /* no resources */
        sig_send(l, SIG_CONN_RSP, id, rsp, sizeof rsp);
        return;
    }
    c->pub.stack_cid = scid;
    c->pub.state = FLOSS_SIM_CHAN_CONFIG;
    floss_put_le16(rsp, c->pub.sim_cid);
    floss_put_le16(rsp + 4, 0x0000); /* success */
    sig_send(l, SIG_CONN_RSP, id, rsp, sizeof rsp);
    FLOSS_LOGI(TRANSPORT, "sim: stack connects PSM 0x%04x (stack cid 0x%04x)", (unsigned)psm, (unsigned)scid);
    sig_send_conf_req(l, c, sim.cfg.l2cap_mtu);
}

static void sig_on_conn_rsp(sim_link_t *l, uint16_t dcid, uint16_t scid, uint16_t result) {
    sim_chan_t *c = chan_by_sim_cid(l->pub.handle, scid);
    if (!c || c->pub.state != FLOSS_SIM_CHAN_WAIT_CONNECT) {
        FLOSS_LOGW(TRANSPORT, "sim: Connection Response for unknown cid 0x%04x", (unsigned)scid);
        return;
    }
    c->pub.result = result;
    if (result == 0x0001) {
        return; /* pending (security / authorization): the final response follows */
    }
    if (result != 0x0000) {
        c->pub.state = FLOSS_SIM_CHAN_CLOSED;
        FLOSS_LOGI(TRANSPORT, "sim: stack refused PSM 0x%04x, result 0x%04x", (unsigned)c->pub.psm, (unsigned)result);
        return;
    }
    c->pub.stack_cid = dcid;
    c->pub.state = FLOSS_SIM_CHAN_CONFIG;
    sig_send_conf_req(l, c, sim.cfg.l2cap_mtu);
}

/**
 * The stack's Configuration Request. The sim accepts any MTU >= 48 (the
 * BR/EDR minimum), records the flush timeout, ignores QoS, refuses unknown
 * non-hint options (result 0x0003 lists them back).
 */
static void sig_on_conf_req(sim_link_t *l, uint8_t id, uint16_t dcid, uint16_t flags, const uint8_t *o,
                            uint16_t n) {
    sim_chan_t *c = chan_by_sim_cid(l->pub.handle, dcid);
    if (!c || (c->pub.state != FLOSS_SIM_CHAN_CONFIG && c->pub.state != FLOSS_SIM_CHAN_OPEN)) {
        sig_reject(l, id, 0x0002, dcid, 0);
        return;
    }
    uint8_t rsp[6 + 32];
    uint16_t rn = 6;
    uint16_t result = 0;
    while (n >= 2) {
        uint8_t type = o[0];
        uint8_t olen = o[1];
        if ((uint16_t)(2 + olen) > n) {
            sim.stats.malformed++;
            break;
        }
        switch (type & 0x7F) {
        case L2CAP_OPT_MTU:
            if (olen >= 2) {
                uint16_t mtu = floss_get_le16(o + 2);
                if (mtu < L2CAP_MIN_MTU) {
                    result = 0x0001; /* unacceptable: suggest the minimum */
                    rsp[rn++] = L2CAP_OPT_MTU;
                    rsp[rn++] = 2;
                    floss_put_le16(rsp + rn, L2CAP_MIN_MTU);
                    rn = (uint16_t)(rn + 2);
                } else {
                    c->pub.stack_mtu = mtu;
                }
            }
            break;
        case L2CAP_OPT_FLUSH_TO:
            if (olen >= 2) {
                c->pub.stack_flush_timeout = floss_get_le16(o + 2);
            }
            break;
        case L2CAP_OPT_QOS:
            break;
        default:
            if (!(type & 0x80) && rn + 2 + olen <= sizeof rsp) {
                result = 0x0003; /* unknown option, not a hint */
                memcpy(rsp + rn, o, (size_t)2 + olen);
                rn = (uint16_t)(rn + 2 + olen);
            }
            break;
        }
        o += 2 + olen;
        n = (uint16_t)(n - 2 - olen);
    }
    floss_put_le16(rsp, c->pub.stack_cid); /* SCID: endpoint on the receiver of this response */
    floss_put_le16(rsp + 2, flags & 0x0001);
    floss_put_le16(rsp + 4, result);
    sig_send(l, SIG_CONF_RSP, id, rsp, rn);
    if (result == 0 && !(flags & 0x0001)) {
        c->cfg_in_done = true;
        chan_check_open(c);
    }
}

/** The stack's answer to our configuration. A refusal is retried with its suggestion, like a real peer. */
static void sig_on_conf_rsp(sim_link_t *l, uint16_t scid, uint16_t flags, uint16_t result, const uint8_t *o,
                            uint16_t n) {
    sim_chan_t *c = chan_by_sim_cid(l->pub.handle, scid);
    if (!c || c->pub.state != FLOSS_SIM_CHAN_CONFIG) {
        FLOSS_LOGW(TRANSPORT, "sim: Configuration Response for unknown cid 0x%04x", (unsigned)scid);
        return;
    }
    if (result == 0x0000) {
        if (!(flags & 0x0001)) {
            c->cfg_out_done = true;
            chan_check_open(c);
        }
        return;
    }
    if (result == 0x0004) {
        return; /* pending */
    }
    c->pub.cfg_reject_result = result;
    c->pub.cfg_rejects++;
    FLOSS_LOGI(TRANSPORT, "sim: stack refused our configuration, result 0x%04x", (unsigned)result);
    if (result == 0x0001 && c->pub.cfg_rejects < 3) {
        uint16_t mtu = 0;
        while (n >= 2 && (uint16_t)(2 + o[1]) <= n) {
            if ((o[0] & 0x7F) == L2CAP_OPT_MTU && o[1] >= 2) {
                mtu = floss_get_le16(o + 2);
            }
            n = (uint16_t)(n - 2 - o[1]);
            o += 2 + o[1];
        }
        c->pub.cfg_suggested_mtu = mtu;
        sig_send_conf_req(l, c, mtu ? mtu : L2CAP_MIN_MTU);
        return;
    }
    chan_send_disc_req(l, c);
}

static void sig_on_disc_req(sim_link_t *l, uint8_t id, uint16_t dcid, uint16_t scid) {
    sim_chan_t *c = chan_by_sim_cid(l->pub.handle, dcid);
    if (!c || c->fixed || c->pub.stack_cid != scid) {
        /* Unknown endpoint: Command Reject "invalid CID in request". */
        sig_reject(l, id, 0x0002, dcid, scid);
        return;
    }
    uint8_t rsp[4];
    floss_put_le16(rsp, dcid); /* echo the request's CIDs */
    floss_put_le16(rsp + 2, scid);
    sig_send(l, SIG_DISC_RSP, id, rsp, sizeof rsp);
    c->pub.state = FLOSS_SIM_CHAN_CLOSED;
    FLOSS_LOGI(TRANSPORT, "sim: stack closed PSM 0x%04x", (unsigned)c->pub.psm);
}

static void sig_on_info_req(sim_link_t *l, uint8_t id, uint16_t type) {
    uint8_t rsp[12];
    uint16_t rn = 4;
    floss_put_le16(rsp, type);
    floss_put_le16(rsp + 2, 0x0000);
    switch (type) {
    case 0x0001: /* connectionless MTU */
        floss_put_le16(rsp + 4, L2CAP_DEFAULT_MTU);
        rn = 6;
        break;
    case 0x0002: /* extended features: fixed channels supported */
        floss_put_le32(rsp + 4, 0x00000080);
        rn = 8;
        break;
    case 0x0003: /* fixed channels: signaling */
        memset(rsp + 4, 0, 8);
        rsp[4] = 0x02;
        rn = 12;
        break;
    default:
        floss_put_le16(rsp + 2, 0x0001); /* not supported */
        break;
    }
    sig_send(l, SIG_INFO_RSP, id, rsp, rn);
}

static void sig_on_reject(sim_link_t *l, uint8_t id) {
    for (unsigned i = 0; i < FLOSS_SIM_MAX_CHANS; i++) {
        sim_chan_t *c = &sim.chans[i];
        if (c->pub.handle == l->pub.handle && c->sig_id == id &&
            (c->pub.state == FLOSS_SIM_CHAN_WAIT_CONNECT || c->pub.state == FLOSS_SIM_CHAN_WAIT_DISCONNECT ||
             c->pub.state == FLOSS_SIM_CHAN_CONFIG)) {
            if (c->pub.state == FLOSS_SIM_CHAN_WAIT_CONNECT) {
                c->pub.result = 0xFFFF;
            }
            c->pub.state = FLOSS_SIM_CHAN_CLOSED;
        }
    }
}

static void sig_handle(sim_link_t *l, uint8_t code, uint8_t id, const uint8_t *d, uint16_t n) {
    static const uint8_t min_len[12] = {0, 2, 4, 8, 4, 6, 4, 4, 0, 0, 2, 4};
    if (code < sizeof min_len && n < min_len[code]) {
        sim.stats.malformed++;
        FLOSS_LOGW(TRANSPORT, "sim: short signaling command 0x%02x", (unsigned)code);
        return;
    }
    switch (code) {
    case SIG_CMD_REJECT:
        sig_on_reject(l, id);
        break;
    case SIG_CONN_REQ:
        sig_on_conn_req(l, id, floss_get_le16(d), floss_get_le16(d + 2));
        break;
    case SIG_CONN_RSP:
        sig_on_conn_rsp(l, floss_get_le16(d), floss_get_le16(d + 2), floss_get_le16(d + 4));
        break;
    case SIG_CONF_REQ:
        sig_on_conf_req(l, id, floss_get_le16(d), floss_get_le16(d + 2), d + 4, (uint16_t)(n - 4));
        break;
    case SIG_CONF_RSP:
        sig_on_conf_rsp(l, floss_get_le16(d), floss_get_le16(d + 2), floss_get_le16(d + 4), d + 6,
                        (uint16_t)(n - 6));
        break;
    case SIG_DISC_REQ:
        sig_on_disc_req(l, id, floss_get_le16(d), floss_get_le16(d + 2));
        break;
    case SIG_DISC_RSP: {
        /* CIDs echo our request: DCID = stack's, SCID = ours. */
        sim_chan_t *c = chan_by_sim_cid(l->pub.handle, floss_get_le16(d + 2));
        if (c && c->pub.state == FLOSS_SIM_CHAN_WAIT_DISCONNECT) {
            c->pub.state = FLOSS_SIM_CHAN_CLOSED;
        }
        break;
    }
    case SIG_ECHO_REQ:
        sig_send(l, SIG_ECHO_RSP, id, d, n);
        break;
    case SIG_INFO_REQ:
        sig_on_info_req(l, id, floss_get_le16(d));
        break;
    case SIG_ECHO_RSP:
    case SIG_INFO_RSP:
        break; /* recorded in the signaling log for the test */
    default:
        sig_reject(l, id, 0x0000, 0, 0); /* command not understood */
        break;
    }
}

static void sig_rx(sim_link_t *l, const uint8_t *p, uint16_t len) {
    while (len >= 4) {
        uint8_t code = p[0];
        uint8_t id = p[1];
        uint16_t clen = floss_get_le16(p + 2);
        if ((uint32_t)clen + 4 > len) {
            sim.stats.malformed++;
            FLOSS_LOGW(TRANSPORT, "sim: signaling command 0x%02x overruns its PDU", (unsigned)code);
            return;
        }
        sig_log(l->pub.handle, code, id, p + 4, clen);
        sig_handle(l, code, id, p + 4, clen);
        p += 4 + clen;
        len = (uint16_t)(len - 4 - clen);
    }
    if (len) {
        sim.stats.malformed++;
    }
}

/** A complete L2CAP PDU from the stack arrives at the remote. */
static void l2cap_rx(sim_link_t *l, const uint8_t *pdu, uint16_t len) {
    uint16_t plen = floss_get_le16(pdu);
    uint16_t cid = floss_get_le16(pdu + 2);
    FLOSS_LOGD(TRANSPORT, "sim: L2CAP rx cid 0x%04x len %u", (unsigned)cid, (unsigned)plen);
    (void)len;
    if (!l->pub.le && cid == FLOSS_CID_SIGNALING) {
        sig_rx(l, pdu + 4, plen);
        return;
    }
    sim_chan_t *c = chan_by_sim_cid(l->pub.handle, cid);
    if (!c || c->pub.state != FLOSS_SIM_CHAN_OPEN) {
        sim.stats.l2cap_unknown_cid++;
        FLOSS_LOGW(TRANSPORT, "sim: data for cid 0x%04x which is not open on the remote", (unsigned)cid);
        return;
    }
    if (plen > c->pub.sim_mtu) {
        sim.stats.l2cap_mtu_violations++;
        FLOSS_LOGW(TRANSPORT, "sim: SDU of %u bytes exceeds the MTU %u the remote configured", (unsigned)plen,
                   (unsigned)c->pub.sim_mtu);
    }
    inbox_push(c, pdu + 4, plen);
    if (l->pub.le && cid == FLOSS_SIM_CID_ATT) {
        sim_gatt_rx(l, pdu + 4, plen); /* scripted GATT server, if one serves this link */
    }
}

/* ------------------------------------------------------------------------ */
/* 8. ACL to the stack                                                      */
/* ------------------------------------------------------------------------ */

/** Queue one PDU, fragmented to acl_rx_frag: first 0b10 (start), then 0b01 (continuation). */
static void acl_to_stack(sim_link_t *l, const uint8_t *pdu, uint16_t len) {
    uint16_t frag = sim.cfg.acl_rx_frag;
    if (frag == 0 || frag > FLOSS_SIM_ACL_MAX) {
        frag = FLOSS_SIM_ACL_MAX;
    }
    uint16_t off = 0;
    do {
        uint16_t n = FLOSS_MIN(frag, (uint16_t)(len - off));
        sim_pkt_t *p = q_push(FLOSS_PKT_ACL, (uint16_t)(4 + n));
        if (!p) {
            return;
        }
        uint16_t pb = off == 0 ? 0x2 : 0x1;
        floss_put_le16(p->data, (uint16_t)(l->pub.handle | (pb << 12)));
        floss_put_le16(p->data + 2, n);
        memcpy(p->data + 4, pdu + off, n);
        off = (uint16_t)(off + n);
    } while (off < len);
}

/* ------------------------------------------------------------------------ */
/* 9. Public helpers                                                        */
/* ------------------------------------------------------------------------ */

const floss_sim_link_t *floss_sim_link(uint16_t handle) {
    sim_link_t *l = link_find(handle);
    return l ? &l->pub : NULL;
}

const floss_sim_link_t *floss_sim_link_by_addr(const floss_addr_t *addr) {
    sim_link_t *l = link_by_addr(addr);
    return l ? &l->pub : NULL;
}

uint16_t floss_sim_remote_connect(const floss_addr_t *addr, uint32_t cod) {
    if (!sim.open) {
        return FLOSS_SIM_NO_HANDLE;
    }
    if (!(sim.scan & 0x02)) {
        FLOSS_LOGW(TRANSPORT, "sim: %s pages us but page scan is off (Write Scan Enable)", addr_str(addr));
        return FLOSS_SIM_NO_HANDLE;
    }
    floss_sim_peer_t *peer = floss_sim_peer_add(addr, cod);
    if (!peer || link_by_addr(addr)) {
        return FLOSS_SIM_NO_HANDLE;
    }
    peer->present = true;
    sim_link_t *l = link_alloc();
    if (!l) {
        return FLOSS_SIM_NO_HANDLE;
    }
    l->pub.state = FLOSS_SIM_LINK_REQUESTED;
    l->pub.addr = *addr;
    l->pub.role = 1; /* the paged side is peripheral unless it asks to switch */
    link_assign_handle(l);

    sim_w_t w = {.n = 0};
    w_addr(&w, addr);
    w_u8(&w, (uint8_t)cod);
    w_u8(&w, (uint8_t)(cod >> 8));
    w_u8(&w, (uint8_t)(cod >> 16));
    w_u8(&w, 0x01); /* ACL */
    emit_w(FLOSS_EVT_CONNECTION_REQUEST, &w);
    FLOSS_LOGI(TRANSPORT, "sim: %s pages the stack (handle will be 0x%03x)", addr_str(addr), (unsigned)l->pub.handle);
    return l->pub.handle;
}

floss_err_t floss_sim_remote_disconnect(uint16_t handle, uint8_t reason) {
    sim_link_t *l = link_open(handle);
    if (!l) {
        return FLOSS_ERR_NOT_FOUND;
    }
    link_disconnected(l, reason);
    return FLOSS_OK;
}

floss_err_t floss_sim_remote_pair(uint16_t handle) {
    sim_link_t *l = link_open(handle);
    if (!l || l->pub.le) {
        return FLOSS_ERR_NOT_FOUND;
    }
    if (l->pair_step != PAIR_NONE) {
        return FLOSS_ERR_BUSY;
    }
    pair_begin(l, false);
    return FLOSS_OK;
}

uint16_t floss_sim_le_connect(const floss_addr_t *addr, uint8_t addr_type, uint8_t role) {
    sim_link_t *l = sim.open ? link_alloc() : NULL;
    if (!l) {
        return FLOSS_SIM_NO_HANDLE;
    }
    floss_sim_peer_add(addr, 0);
    l->pub.le = true;
    l->pub.addr = *addr;
    l->pub.role = role;
    l->pub.stack_initiated = role == 0;
    le_link_complete(l, addr_type, 0x0018 /* 30 ms */, 0, 0x0048 /* 720 ms */);
    return l->pub.handle;
}

static bool cond_chan_settled(void *arg) {
    floss_sim_chan_state_t st = ((sim_chan_t *)arg)->pub.state;
    return st == FLOSS_SIM_CHAN_OPEN || st == FLOSS_SIM_CHAN_CLOSED || st == FLOSS_SIM_CHAN_FREE;
}

static bool cond_chan_closed(void *arg) {
    floss_sim_chan_state_t st = ((sim_chan_t *)arg)->pub.state;
    return st == FLOSS_SIM_CHAN_CLOSED || st == FLOSS_SIM_CHAN_FREE;
}

static bool cond_chan_has_data(void *arg) {
    sim_chan_t *c = arg;
    return c->in_count > 0 || c->pub.state == FLOSS_SIM_CHAN_FREE;
}

floss_sim_chan_t *floss_sim_l2cap_connect(uint16_t handle, uint16_t psm) {
    sim_link_t *l = link_open(handle);
    if (!l || l->pub.le) {
        return NULL;
    }
    sim_chan_t *c = chan_alloc(handle, psm);
    if (!c) {
        return NULL;
    }
    c->pub.sim_initiated = true;
    c->pub.state = FLOSS_SIM_CHAN_WAIT_CONNECT;
    uint8_t d[4];
    floss_put_le16(d, psm);
    floss_put_le16(d + 2, c->pub.sim_cid);
    c->sig_id = sig_next_id(l);
    sig_send(l, SIG_CONN_REQ, c->sig_id, d, sizeof d);
    FLOSS_LOGI(TRANSPORT, "sim: remote connects PSM 0x%04x", (unsigned)psm);
    floss_sim_pump_until(cond_chan_settled, c, 0);
    return &c->pub;
}

floss_err_t floss_sim_l2cap_send(floss_sim_chan_t *ch, const uint8_t *data, uint16_t len) {
    sim_chan_t *c = (sim_chan_t *)ch;
    if (!c || c->pub.state != FLOSS_SIM_CHAN_OPEN) {
        return FLOSS_ERR_STATE;
    }
    sim_link_t *l = link_open(c->pub.handle);
    if (!l) {
        return FLOSS_ERR_STATE;
    }
    if (len > c->pub.stack_mtu || len > FLOSS_SIM_ACL_MAX) {
        FLOSS_LOGW(TRANSPORT, "sim: refusing to send %u bytes, stack MTU is %u", (unsigned)len,
                   (unsigned)c->pub.stack_mtu);
        return FLOSS_ERR_INVALID;
    }
    uint8_t pdu[4 + FLOSS_SIM_ACL_MAX];
    floss_put_le16(pdu, len);
    floss_put_le16(pdu + 2, c->pub.stack_cid);
    if (len) {
        memcpy(pdu + 4, data, len);
    }
    acl_to_stack(l, pdu, (uint16_t)(4 + len));
    return FLOSS_OK;
}

int floss_sim_l2cap_recv(floss_sim_chan_t *ch, uint8_t *out, uint16_t cap) {
    sim_chan_t *c = (sim_chan_t *)ch;
    if (!c || c->in_count == 0) {
        return -1;
    }
    uint16_t len = c->in_len[c->in_head];
    memcpy(out, c->in_data[c->in_head], FLOSS_MIN(len, cap));
    c->in_head = (uint8_t)((c->in_head + 1) % FLOSS_SIM_INBOX_LEN);
    c->in_count--;
    return len;
}

uint8_t floss_sim_l2cap_pending(const floss_sim_chan_t *ch) {
    return ch ? ((const sim_chan_t *)ch)->in_count : 0;
}

int floss_sim_l2cap_recv_wait(floss_sim_chan_t *ch, uint8_t *out, uint16_t cap) {
    if (!ch) {
        return -1;
    }
    floss_sim_pump_until(cond_chan_has_data, ch, 0);
    return floss_sim_l2cap_recv(ch, out, cap);
}

floss_err_t floss_sim_l2cap_disconnect(floss_sim_chan_t *ch) {
    sim_chan_t *c = (sim_chan_t *)ch;
    if (!c || c->fixed || (c->pub.state != FLOSS_SIM_CHAN_OPEN && c->pub.state != FLOSS_SIM_CHAN_CONFIG)) {
        return FLOSS_ERR_STATE;
    }
    sim_link_t *l = link_open(c->pub.handle);
    if (!l) {
        return FLOSS_ERR_STATE;
    }
    chan_send_disc_req(l, c);
    return floss_sim_pump_until(cond_chan_closed, c, 0) ? FLOSS_OK : FLOSS_ERR_TIMEOUT;
}

floss_sim_chan_t *floss_sim_chan_find(uint16_t handle, uint16_t psm) {
    sim_chan_t *best = NULL;
    for (unsigned i = 0; i < FLOSS_SIM_MAX_CHANS; i++) {
        sim_chan_t *c = &sim.chans[i];
        if (c->pub.state != FLOSS_SIM_CHAN_FREE && !c->fixed && c->pub.handle == handle && c->pub.psm == psm &&
            (!best || c->pub.seq > best->pub.seq)) {
            best = c;
        }
    }
    return best ? &best->pub : NULL;
}

uint8_t floss_sim_l2cap_sig_send(uint16_t handle, uint8_t code, const uint8_t *data, uint16_t len) {
    sim_link_t *l = link_open(handle);
    if (!l) {
        return 0;
    }
    uint8_t id = sig_next_id(l);
    sig_send(l, code, id, data, len);
    return id;
}

floss_err_t floss_sim_acl_send_pdu(uint16_t handle, const uint8_t *pdu, uint16_t len) {
    sim_link_t *l = link_open(handle);
    if (!l) {
        return FLOSS_ERR_NOT_FOUND;
    }
    if (len == 0 || len > FLOSS_SIM_ACL_MAX + 4) {
        return FLOSS_ERR_INVALID;
    }
    acl_to_stack(l, pdu, len);
    return FLOSS_OK;
}

static sim_chan_t *fixed_chan(uint16_t handle, uint16_t cid) {
    for (unsigned i = 0; i < FLOSS_SIM_MAX_CHANS; i++) {
        sim_chan_t *c = &sim.chans[i];
        if (c->fixed && c->pub.state == FLOSS_SIM_CHAN_OPEN && c->pub.handle == handle && c->pub.sim_cid == cid) {
            return c;
        }
    }
    return NULL;
}

floss_err_t floss_sim_fixed_send(uint16_t handle, uint16_t cid, const uint8_t *data, uint16_t len) {
    sim_chan_t *c = fixed_chan(handle, cid);
    if (c && cid == FLOSS_SIM_CID_ATT) {
        sim_gatt_on_remote_tx(handle, data, len); /* a hand-written ATT response settles the request */
    }
    return c ? floss_sim_l2cap_send(&c->pub, data, len) : FLOSS_ERR_NOT_FOUND;
}

int floss_sim_fixed_recv(uint16_t handle, uint16_t cid, uint8_t *out, uint16_t cap) {
    sim_chan_t *c = fixed_chan(handle, cid);
    return c ? floss_sim_l2cap_recv(&c->pub, out, cap) : -1;
}

/*
 * SDP ServiceSearchAttribute, the request every host sends to find the HID
 * record. PDU: id(1) transaction(2, BE) param_len(2, BE) | pattern DES |
 * MaximumAttributeByteCount(2) | attribute ID list DES | continuation.
 * The server returns at most that many AttributeLists bytes per response and
 * a continuation state to send back for the next chunk.
 */
int floss_sim_sdp_ssa(uint16_t handle, uint16_t uuid16, uint8_t *out, uint16_t cap) {
    floss_sim_chan_t *ch = floss_sim_l2cap_connect(handle, FLOSS_PSM_SDP);
    if (!ch || ch->state != FLOSS_SIM_CHAN_OPEN) {
        return FLOSS_ERR_STATE;
    }
    uint8_t cont[17];
    uint8_t cont_len = 0;
    uint16_t tid = 0x0100;
    uint32_t total = 0;
    int result = FLOSS_ERR_PROTOCOL;

    for (unsigned round = 0; round < 64; round++, tid++) {
        uint8_t req[48];
        uint16_t n = 0;
        req[n++] = 0x06;                       /* ServiceSearchAttributeRequest */
        floss_put_be16(req + n, tid);
        n += 4;                                /* + parameter length, patched below */
        req[n++] = 0x35;                       /* DES, 1-byte length */
        req[n++] = 3;
        req[n++] = 0x19;                       /* UUID16 */
        floss_put_be16(req + n, uuid16);
        n += 2;
        floss_put_be16(req + n, sim.cfg.sdp_max_attr_bytes);
        n += 2;
        req[n++] = 0x35;
        req[n++] = 5;
        req[n++] = 0x0A;                       /* uint32 range 0x0000-0xFFFF */
        floss_put_be32(req + n, 0x0000FFFF);
        n += 4;
        req[n++] = cont_len;
        memcpy(req + n, cont, cont_len);
        n = (uint16_t)(n + cont_len);
        floss_put_be16(req + 3, (uint16_t)(n - 5));
        if (floss_sim_l2cap_send(ch, req, n) != FLOSS_OK) {
            break;
        }

        uint8_t rsp[FLOSS_SIM_ACL_MAX];
        int rl = floss_sim_l2cap_recv_wait(ch, rsp, sizeof rsp);
        if (rl < 0) {
            result = FLOSS_ERR_TIMEOUT;
            break;
        }
        if (rl < 5 || floss_get_be16(rsp + 1) != tid || rsp[0] != 0x07) {
            FLOSS_LOGW(TRANSPORT, "sim: SDP error/unexpected response 0x%02x", (unsigned)rsp[0]);
            break;
        }
        uint16_t plen = floss_get_be16(rsp + 3);
        if ((int)plen + 5 > rl || plen < 3) {
            break;
        }
        uint16_t count = floss_get_be16(rsp + 5);
        if ((uint32_t)count + 3 > plen) {
            break;
        }
        uint8_t cl = rsp[7 + count];
        if (cl > 16 || (uint32_t)count + 3 + cl > plen) {
            break;
        }
        if (total + count > cap) {
            result = FLOSS_ERR_NOMEM;
            break;
        }
        memcpy(out + total, rsp + 7, count);
        total += count;
        cont_len = cl;
        memcpy(cont, rsp + 8 + count, cl);
        if (cl == 0) {
            result = (int)total;
            break;
        }
    }
    floss_sim_l2cap_disconnect(ch);
    return result;
}

floss_err_t floss_sim_inject_event(uint8_t code, const uint8_t *params, uint8_t len) {
    return emit_event(code, params, len) ? FLOSS_OK : FLOSS_ERR_NOMEM;
}

/* ------------------------------------------------------------------------ */
/* Controller state accessors                                               */
/* ------------------------------------------------------------------------ */

const floss_addr_t *floss_sim_addr(void) {
    return &sim.addr;
}
uint8_t floss_sim_scan_enable(void) {
    return sim.scan;
}
uint32_t floss_sim_class_of_device(void) {
    return sim.cod;
}
const char *floss_sim_local_name(void) {
    return sim.name;
}
bool floss_sim_ssp_enabled(void) {
    return sim.ssp_mode;
}
bool floss_sim_sc_enabled(void) {
    return sim.sc_host;
}
const floss_sim_stats_t *floss_sim_stats(void) {
    return &sim.stats;
}

uint32_t floss_sim_violations(void) {
    const floss_sim_stats_t *st = &sim.stats;
    return st->cmd_credit_violations + st->acl_credit_violations + st->acl_size_violations + st->acl_bad_handle +
           st->acl_pb_violations + st->l2cap_mtu_violations + st->l2cap_unknown_cid + st->malformed;
}

/* ------------------------------------------------------------------------ */
/* 10. Transport ops, reset, pump                                           */
/* ------------------------------------------------------------------------ */

/**
 * Everything HCI Reset clears: links vanish silently (the host forgets them
 * too), undelivered events are dropped, settings return to defaults.
 * Peers (and their link keys) live on the far side of the air and survive.
 */
static void controller_reset(void) {
    memset(sim.links, 0, sizeof sim.links);
    memset(sim.chans, 0, sizeof sim.chans);
    memset(sim.zombies, 0, sizeof sim.zombies);
    /* While delivering, the head packet is still being read by the stack: keep it. */
    sim.q_count = sim.in_poll ? 1 : 0;
    sim.cmd_outstanding = 0;
    sim.pool_out[0] = 0;
    sim.pool_out[1] = 0;
    sim.scan = 0;
    sim.cod = 0;
    sim.name[0] = '\0';
    sim.ssp_mode = false;
    sim.sc_host = false;
    sim.le_host = false;
    sim.page_timeout = 0x2000; /* 5.12 s */
    sim.default_policy = 0;
    sim.evt_mask = SIM_EVT_MASK_DEFAULT;
    sim.evt_mask2 = 0;
    sim.le_evt_mask = SIM_LE_EVT_MASK_DEFAULT;
    sim.next_handle = 0x0040;
    sim.next_sim_cid = FLOSS_CID_DYNAMIC_FIRST;
    sim.le_connecting = false;
    memset(&sim.le_scan, 0, sizeof sim.le_scan);
    sim.gatt.active = false; /* its link vanished with the others */
}

static floss_err_t sim_open(floss_transport_t *t) {
    (void)t;
    /* Power-on: a fresh controller, but the remote world (peers, keys) persists. */
    controller_reset();
    sim.addr = sim.cfg.addr;
    sim.open = true;
    FLOSS_LOGI(TRANSPORT, "sim: controller powered on, address %s", addr_str(&sim.addr));
    return FLOSS_OK;
}

static void sim_close(floss_transport_t *t) {
    (void)t;
    sim.open = false;
}

static floss_err_t sim_send(floss_transport_t *t, uint8_t type, uint8_t *pkt, uint16_t len) {
    (void)t;
    if (!sim.open) {
        return FLOSS_ERR_STATE;
    }
    switch (type) {
    case FLOSS_PKT_CMD:
        sim_rx_cmd(pkt, len);
        break;
    case FLOSS_PKT_ACL:
        sim_rx_acl(pkt, len);
        break;
    default:
        sim.stats.malformed++;
        FLOSS_LOGW(TRANSPORT, "sim: unsupported packet type %u", (unsigned)type);
        break;
    }
    return FLOSS_OK;
}

static int sim_poll(floss_transport_t *t) {
    if (!sim.open || sim.in_poll) {
        return 0;
    }
    sim.in_poll = true;
    sim_tick();
    int delivered = 0;
    while (sim.q_count && delivered < SIM_DELIVER_PER_POLL) {
        sim_pkt_t *p = &sim.q[sim.q_head];
        /* Credits first: the stack may send from inside its rx callback using them. */
        if (p->cmd_credit && sim.cmd_outstanding) {
            sim.cmd_outstanding--;
        }
        for (unsigned i = 0; i < 2; i++) {
            sim.pool_out[i] = (uint16_t)(sim.pool_out[i] - FLOSS_MIN(sim.pool_out[i], p->credits[i]));
        }
        if (p->type == FLOSS_PKT_EVT) {
            sim.stats.events++;
            if (p->data[0] == FLOSS_EVT_DISCONNECTION_COMPLETE) {
                (void)zombie_check(floss_get_le16(p->data + 3), true);
            }
        } else {
            sim.stats.acl_out++;
        }
        floss_transport_deliver(t, p->type, p->data, p->len);
        sim.q_head = (uint16_t)((sim.q_head + 1) % FLOSS_SIM_QUEUE_LEN);
        sim.q_count--;
        delivered++;
    }
    sim.in_poll = false;
    return delivered;
}

static const floss_transport_ops_t sim_ops = {
    .name = "sim",
    .headroom = 1,
    .open = sim_open,
    .close = sim_close,
    .send = sim_send,
    .poll = sim_poll,
};

void floss_sim_config_defaults(floss_sim_config_t *cfg) {
    static const floss_addr_t def_addr = {{0x01, 0x00, 0x50, 0x05, 0xF1, 0x02}}; /* 02:F1:05:50:00:01 */
    memset(cfg, 0, sizeof *cfg);
    cfg->addr = def_addr;
    cfg->acl_mtu = 1021;
    cfg->acl_total = 8;
    cfg->le_acl_mtu = 251;
    cfg->le_acl_total = 8;
    cfg->acl_rx_frag = 1021;
    cfg->nocp_delay_us = 0;
    cfg->enforce_event_mask = true;
    cfg->seed = 0x5EED1234u;
    cfg->l2cap_mtu = L2CAP_DEFAULT_MTU;
    cfg->sdp_max_attr_bytes = 100; /* small and odd on purpose: forces continuation */
    cfg->pump_step_us = 100;
    cfg->wait_polls = 2000;
    cfg->advance_time = NULL;
}

floss_transport_t *floss_sim_init(const floss_sim_config_t *cfg) {
    memset(&sim, 0, sizeof sim);
    if (cfg) {
        sim.cfg = *cfg;
    } else {
        floss_sim_config_defaults(&sim.cfg);
    }
    if (sim.cfg.acl_rx_frag == 0 || sim.cfg.acl_rx_frag > FLOSS_SIM_ACL_MAX) {
        sim.cfg.acl_rx_frag = FLOSS_SIM_ACL_MAX;
    }
    if (sim.cfg.l2cap_mtu == 0 || sim.cfg.l2cap_mtu > FLOSS_SIM_ACL_MAX) {
        sim.cfg.l2cap_mtu = L2CAP_DEFAULT_MTU;
    }
    sim.rng = sim.cfg.seed ? sim.cfg.seed : 1u;
    controller_reset();
    sim.addr = sim.cfg.addr;
    sim.transport.ops = &sim_ops;
    sim.transport.ctx = &sim;
    return &sim.transport;
}

floss_sim_config_t *floss_sim_cfg(void) {
    return &sim.cfg;
}

int floss_sim_run(void) {
    return sim_poll(&sim.transport);
}

void floss_sim_pump(uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
        (void)floss_poll();
        if (sim.cfg.advance_time && sim.cfg.pump_step_us) {
            sim.cfg.advance_time(sim.cfg.pump_step_us);
        }
    }
}

void floss_sim_pump_us(uint32_t us) {
    uint32_t step = sim.cfg.pump_step_us ? sim.cfg.pump_step_us : 1;
    floss_sim_pump((us + step - 1) / step);
}

bool floss_sim_pump_until(bool (*cond)(void *arg), void *arg, uint32_t max_polls) {
    if (max_polls == 0) {
        max_polls = sim.cfg.wait_polls ? sim.cfg.wait_polls : 2000;
    }
    for (uint32_t i = 0; i < max_polls; i++) {
        if (cond(arg)) {
            return true;
        }
        floss_sim_pump(1);
    }
    return cond(arg);
}

/* ------------------------------------------------------------------------ */
/* 11. LE central support: advertisers, scanning, scripted GATT server      */
/* ------------------------------------------------------------------------ */

/* ATT opcodes the scripted server speaks */
#define SIM_ATT_ERROR_RSP      0x01
#define SIM_ATT_MTU_REQ        0x02
#define SIM_ATT_MTU_RSP        0x03
#define SIM_ATT_FIND_INFO_REQ  0x04
#define SIM_ATT_FIND_INFO_RSP  0x05
#define SIM_ATT_READ_TYPE_REQ  0x08
#define SIM_ATT_READ_TYPE_RSP  0x09
#define SIM_ATT_READ_REQ       0x0A
#define SIM_ATT_READ_RSP       0x0B
#define SIM_ATT_WRITE_REQ      0x12
#define SIM_ATT_WRITE_RSP      0x13
#define SIM_ATT_NOTIFY         0x1B
#define SIM_ATT_INDICATE       0x1D
#define SIM_ATT_CONFIRM        0x1E
#define SIM_ATT_WRITE_CMD      0x52
#define SIM_ATT_CMD_FLAG       0x40
#define SIM_ATT_DEFAULT_MTU    23

#define SIM_ATT_ERR_INVALID_HANDLE  0x01
#define SIM_ATT_ERR_INVALID_PDU     0x04
#define SIM_ATT_ERR_NOT_SUPPORTED   0x06
#define SIM_ATT_ERR_NOT_FOUND       0x0A
#define SIM_ATT_ERR_INVALID_LEN     0x0D

/** PHY picked for one direction of LE Set PHY: 2M when allowed, else 1M; Coded only when it is the only choice. */
static uint8_t sim_pick_phy(bool no_pref, uint8_t phys) {
    if (no_pref || phys == 0) {
        return 1;
    }
    if (phys & 0x02) {
        return 2;
    }
    return (phys & 0x01) ? 1 : 3;
}

/** LE Set Scan Parameters / Set Scan Enable / Set PHY. */
static bool sim_le_central_cmd(uint16_t op, const uint8_t *p, uint8_t n) {
    sim_w_t w = {.n = 0};
    switch (op) {
    case SIM_OP_LE_SET_SCAN_PARAMS: {
        NEED(7, false);
        uint16_t interval = floss_get_le16(p + 1);
        uint16_t window = floss_get_le16(p + 3);
        if (sim.le_scan.enabled) {
            emit_cc_status(op, SIM_ERR_COMMAND_DISALLOWED); /* parameters are fixed while scanning */
            return true;
        }
        if (p[0] > 1 || interval < 4 || interval > 0x4000 || window < 4 || window > interval || p[5] > 3 ||
            p[6] > 3) {
            emit_cc_status(op, SIM_ERR_INVALID_PARAMS);
            return true;
        }
        sim.le_scan.active = p[0] == 1;
        sim.le_scan.interval = interval;
        sim.le_scan.window = window;
        sim.le_scan.own_addr_type = p[5];
        sim.le_scan.filter_policy = p[6];
        emit_cc_status(op, 0);
        return true;
    }
    case SIM_OP_LE_SET_SCAN_ENABLE: {
        NEED(2, false);
        if (p[0] > 1 || p[1] > 1) {
            emit_cc_status(op, SIM_ERR_INVALID_PARAMS);
            return true;
        }
        if (p[0] && sim.le_connecting) {
            emit_cc_status(op, SIM_ERR_COMMAND_DISALLOWED);
            return true;
        }
        if (p[0] && !sim.le_scan.enabled) {
            uint32_t now = floss_hal_time_us();
            for (unsigned i = 0; i < FLOSS_SIM_MAX_ADVERTISERS; i++) {
                sim.advs[i].due = now; /* first advertising event at the next tick */
                sim.advs[i].seen = false;
            }
        }
        sim.le_scan.enabled = p[0] == 1;
        sim.le_scan.filter_duplicates = p[0] == 1 && p[1] == 1;
        FLOSS_LOGI(TRANSPORT, "sim: LE scanning %s", sim.le_scan.enabled ? "on" : "off");
        emit_cc_status(op, 0);
        return true;
    }
    case SIM_OP_LE_SET_PHY: {
        NEED(7, true);
        sim_link_t *l = link_open(floss_get_le16(p) & 0x0FFF);
        if (!l || !l->pub.le) {
            emit_cs(op, FLOSS_HCI_ERR_UNKNOWN_CONN_ID);
            return true;
        }
        emit_cs(op, 0);
        w_u8(&w, SIM_LE_SUB_PHY_UPDATE);
        w_u8(&w, 0);
        w_u16(&w, l->pub.handle);
        w_u8(&w, sim_pick_phy((p[2] & 0x01) != 0, p[3]));
        w_u8(&w, sim_pick_phy((p[2] & 0x02) != 0, p[4]));
        emit_w(FLOSS_EVT_LE_META, &w);
        return true;
    }
    default:
        return false;
    }
}

/** One LE Advertising Report carrying a single report. */
static void sim_adv_report(sim_adv_t *a, uint8_t evt_type, const uint8_t *data, uint8_t len) {
    sim_w_t w = {.n = 0};
    w_u8(&w, SIM_LE_SUB_ADV_REPORT);
    w_u8(&w, 1); /* Num_Reports */
    w_u8(&w, evt_type);
    w_u8(&w, a->pub.addr_type);
    w_addr(&w, &a->pub.addr);
    w_u8(&w, len);
    w_bytes(&w, data, len);
    w_u8(&w, (uint8_t)a->pub.rssi);
    emit_w(FLOSS_EVT_LE_META, &w);
    a->pub.reports++;
}

/** Legacy connectable advertising ends when a link is created: is the advertiser connected to us? */
static bool sim_adv_connected(const sim_adv_t *a) {
    for (unsigned i = 0; i < FLOSS_SIM_MAX_LINKS; i++) {
        const sim_link_t *l = &sim.links[i];
        if (l->pub.state == FLOSS_SIM_LINK_OPEN && l->pub.le && floss_addr_eq(&l->pub.addr, &a->pub.addr)) {
            return true;
        }
    }
    return false;
}

/** While the host scans: every advertiser that is due sends its report (and a scan response when active). */
static void sim_le_scan_tick(uint32_t now) {
    if (!sim.le_scan.enabled) {
        return;
    }
    for (unsigned i = 0; i < FLOSS_SIM_MAX_ADVERTISERS; i++) {
        sim_adv_t *a = &sim.advs[i];
        if (!a->pub.in_use || floss_time_diff(now, a->due) < 0 || sim_adv_connected(a)) {
            continue;
        }
        a->due = now + (a->pub.interval_us ? a->pub.interval_us : 30000u);
        if (sim.le_scan.filter_duplicates && a->seen) {
            continue;
        }
        a->seen = true;
        sim_adv_report(a, a->pub.evt_type, a->pub.adv, a->pub.adv_len);
        bool scannable = a->pub.evt_type == 0x00 || a->pub.evt_type == 0x02;
        if (sim.le_scan.active && scannable && a->pub.rsp_len) {
            sim_adv_report(a, 0x04 /* SCAN_RSP */, a->pub.rsp, a->pub.rsp_len);
        }
    }
}

floss_sim_advertiser_t *floss_sim_le_advertiser_add(const floss_addr_t *addr, uint8_t addr_type, const uint8_t *adv,
                                                    uint8_t adv_len, const uint8_t *rsp, uint8_t rsp_len) {
    sim_adv_t *a = NULL;
    for (unsigned i = 0; i < FLOSS_SIM_MAX_ADVERTISERS && !a; i++) {
        if (sim.advs[i].pub.in_use && floss_addr_eq(&sim.advs[i].pub.addr, addr)) {
            a = &sim.advs[i];
        }
    }
    for (unsigned i = 0; i < FLOSS_SIM_MAX_ADVERTISERS && !a; i++) {
        if (!sim.advs[i].pub.in_use) {
            a = &sim.advs[i];
        }
    }
    if (!a) {
        FLOSS_LOGE(TRANSPORT, "sim: no free advertiser (FLOSS_SIM_MAX_ADVERTISERS)");
        return NULL;
    }
    memset(a, 0, sizeof *a);
    a->pub.in_use = true;
    a->pub.addr = *addr;
    a->pub.addr_type = addr_type;
    a->pub.evt_type = 0x00;
    a->pub.rssi = -50;
    a->pub.interval_us = 30000u;
    a->pub.adv_len = (uint8_t)FLOSS_MIN(adv_len, sizeof a->pub.adv);
    if (adv && a->pub.adv_len) {
        memcpy(a->pub.adv, adv, a->pub.adv_len);
    }
    a->pub.rsp_len = (uint8_t)FLOSS_MIN(rsp_len, sizeof a->pub.rsp);
    if (rsp && a->pub.rsp_len) {
        memcpy(a->pub.rsp, rsp, a->pub.rsp_len);
    }
    a->due = floss_hal_time_us();
    floss_sim_peer_add(addr, 0); /* present: LE Create Connection to it succeeds */
    return &a->pub;
}

void floss_sim_le_advertiser_remove(const floss_addr_t *addr) {
    for (unsigned i = 0; i < FLOSS_SIM_MAX_ADVERTISERS; i++) {
        if (sim.advs[i].pub.in_use && floss_addr_eq(&sim.advs[i].pub.addr, addr)) {
            sim.advs[i].pub.in_use = false;
        }
    }
}

const floss_sim_le_scan_t *floss_sim_le_scan(void) {
    return &sim.le_scan;
}

bool floss_sim_le_initiating(void) {
    return sim.le_connecting;
}

/* ---- scripted GATT server -------------------------------------------- */

static uint16_t sim_gatt_mtu(void) {
    return sim.gatt.stats.mtu ? sim.gatt.stats.mtu : SIM_ATT_DEFAULT_MTU;
}

static void sim_gatt_send(const uint8_t *pdu, uint16_t len) {
    (void)floss_sim_fixed_send(sim.gatt.link, FLOSS_SIM_CID_ATT, pdu, len);
}

static void sim_gatt_error(uint8_t req, uint16_t handle, uint8_t err) {
    uint8_t e[5] = {SIM_ATT_ERROR_RSP, req, (uint8_t)handle, (uint8_t)(handle >> 8), err};
    sim.gatt.stats.errors_sent++;
    sim_gatt_send(e, sizeof e);
}

static sim_gatt_attr_t *sim_gatt_attr(uint16_t handle) {
    for (unsigned i = 0; i < sim.gatt.count; i++) {
        if (sim.gatt.attrs[i].handle == handle) {
            return &sim.gatt.attrs[i];
        }
    }
    return NULL;
}

/* Find Information: (handle, type) pairs of one format, as many as fit. */
static void sim_gatt_find_info(const uint8_t *p, uint16_t n) {
    if (n != 5) {
        sim_gatt_error(p[0], 0, SIM_ATT_ERR_INVALID_PDU);
        return;
    }
    uint16_t start = floss_get_le16(p + 1), end = floss_get_le16(p + 3);
    if (start == 0 || start > end) {
        sim_gatt_error(p[0], start, SIM_ATT_ERR_INVALID_HANDLE);
        return;
    }
    uint8_t rsp[FLOSS_SIM_ACL_MAX];
    uint16_t mtu = sim_gatt_mtu(), pos = 2;
    uint8_t fmt = 0;
    for (unsigned i = 0; i < sim.gatt.count; i++) {
        const sim_gatt_attr_t *a = &sim.gatt.attrs[i];
        if (a->handle < start || a->handle > end) {
            continue;
        }
        uint8_t f = a->is128 ? 2 : 1;
        uint16_t entry = f == 1 ? 4 : 18;
        if ((fmt && f != fmt) || pos + entry > mtu) {
            break;
        }
        fmt = f;
        floss_put_le16(rsp + pos, a->handle);
        if (f == 1) {
            floss_put_le16(rsp + pos + 2, a->type16);
        } else {
            memcpy(rsp + pos + 2, a->type128, 16);
        }
        pos = (uint16_t)(pos + entry);
    }
    if (!fmt) {
        sim_gatt_error(p[0], start, SIM_ATT_ERR_NOT_FOUND);
        return;
    }
    rsp[0] = SIM_ATT_FIND_INFO_RSP;
    rsp[1] = fmt;
    sim_gatt_send(rsp, pos);
}

/* Read By Type: (handle, value) pairs of equal length, as many as fit. */
static void sim_gatt_read_by_type(const uint8_t *p, uint16_t n) {
    if (n != 7 && n != 21) {
        sim_gatt_error(p[0], 0, SIM_ATT_ERR_INVALID_PDU);
        return;
    }
    uint16_t start = floss_get_le16(p + 1), end = floss_get_le16(p + 3);
    if (start == 0 || start > end) {
        sim_gatt_error(p[0], start, SIM_ATT_ERR_INVALID_HANDLE);
        return;
    }
    uint8_t rsp[FLOSS_SIM_ACL_MAX];
    uint16_t mtu = sim_gatt_mtu(), pos = 2, entry = 0;
    for (unsigned i = 0; i < sim.gatt.count; i++) {
        const sim_gatt_attr_t *a = &sim.gatt.attrs[i];
        bool match = n == 7 ? (!a->is128 && a->type16 == floss_get_le16(p + 5))
                            : (a->is128 && memcmp(a->type128, p + 5, 16) == 0);
        if (a->handle < start || a->handle > end || !match) {
            continue;
        }
        if (a->error) {
            if (entry == 0) {
                sim_gatt_error(p[0], a->handle, a->error);
                return;
            }
            break;
        }
        uint16_t vlen = FLOSS_MIN(a->len, (uint16_t)(mtu - 4));
        if (entry == 0) {
            entry = (uint16_t)(2 + vlen);
        } else if (entry != 2 + vlen) {
            break;
        }
        if (pos + entry > mtu) {
            break;
        }
        floss_put_le16(rsp + pos, a->handle);
        memcpy(rsp + pos + 2, a->value, vlen);
        pos = (uint16_t)(pos + entry);
    }
    if (entry == 0) {
        sim_gatt_error(p[0], start, SIM_ATT_ERR_NOT_FOUND);
        return;
    }
    rsp[0] = SIM_ATT_READ_TYPE_RSP;
    rsp[1] = (uint8_t)entry;
    sim_gatt_send(rsp, pos);
}

/** Write Request (cmd = false) or Write Command. Returns the ATT error, 0 = applied. */
static uint8_t sim_gatt_write(const uint8_t *p, uint16_t n, bool cmd) {
    if (n < 3) {
        return SIM_ATT_ERR_INVALID_PDU;
    }
    uint16_t h = floss_get_le16(p + 1);
    sim_gatt_attr_t *a = sim_gatt_attr(h);
    if (!a) {
        return SIM_ATT_ERR_INVALID_HANDLE;
    }
    if (a->error) {
        return a->error;
    }
    if (n - 3 > FLOSS_SIM_GATT_VALUE_MAX) {
        return SIM_ATT_ERR_INVALID_LEN;
    }
    a->len = (uint16_t)(n - 3);
    memcpy(a->value, p + 3, a->len);
    sim.gatt.stats.last_write_handle = h;
    if (cmd) {
        sim.gatt.stats.write_cmds++;
    } else {
        sim.gatt.stats.writes++;
    }
    return 0;
}

/** An ATT PDU from the stack on the served link. */
static void sim_gatt_rx(sim_link_t *l, const uint8_t *p, uint16_t n) {
    if (!sim.gatt.active || l->pub.handle != sim.gatt.link || n == 0) {
        return;
    }
    uint8_t op = p[0];
    if (op == SIM_ATT_CONFIRM) {
        sim.gatt.stats.confirmations++;
        return;
    }
    bool cmd = (op & SIM_ATT_CMD_FLAG) != 0;
    bool req = !cmd && (op & 1) == 0;
    if (!req && !cmd) {
        return; /* responses / notifications from the stack's own server: inbox only */
    }
    if (req) {
        sim.gatt.stats.requests++;
        if (sim.gatt.req_pending) {
            sim.gatt.stats.pipelined++;
        }
        if (sim.gatt.silent) {
            sim.gatt.req_pending = true;
            return;
        }
    }
    uint8_t rsp[FLOSS_SIM_ACL_MAX];
    switch (op) {
    case SIM_ATT_MTU_REQ: {
        if (n != 3) {
            sim_gatt_error(op, 0, SIM_ATT_ERR_INVALID_PDU);
            break;
        }
        uint16_t mtu = FLOSS_MIN(floss_get_le16(p + 1), sim.gatt.server_mtu);
        rsp[0] = SIM_ATT_MTU_RSP;
        floss_put_le16(rsp + 1, sim.gatt.server_mtu);
        sim_gatt_send(rsp, 3); /* still at the old MTU */
        sim.gatt.stats.mtu = FLOSS_MAX(mtu, (uint16_t)SIM_ATT_DEFAULT_MTU);
        break;
    }
    case SIM_ATT_FIND_INFO_REQ:
        sim_gatt_find_info(p, n);
        break;
    case SIM_ATT_READ_TYPE_REQ:
        sim_gatt_read_by_type(p, n);
        break;
    case SIM_ATT_READ_REQ: {
        sim_gatt_attr_t *a = n == 3 ? sim_gatt_attr(floss_get_le16(p + 1)) : NULL;
        if (n != 3) {
            sim_gatt_error(op, 0, SIM_ATT_ERR_INVALID_PDU);
        } else if (!a) {
            sim_gatt_error(op, floss_get_le16(p + 1), SIM_ATT_ERR_INVALID_HANDLE);
        } else if (a->error) {
            sim_gatt_error(op, a->handle, a->error);
        } else {
            uint16_t len = FLOSS_MIN(a->len, (uint16_t)(sim_gatt_mtu() - 1));
            rsp[0] = SIM_ATT_READ_RSP;
            memcpy(rsp + 1, a->value, len);
            sim_gatt_send(rsp, (uint16_t)(1 + len));
        }
        break;
    }
    case SIM_ATT_WRITE_REQ: {
        uint8_t err = sim_gatt_write(p, n, false);
        if (err) {
            sim_gatt_error(op, n >= 3 ? floss_get_le16(p + 1) : 0, err);
        } else {
            rsp[0] = SIM_ATT_WRITE_RSP;
            sim_gatt_send(rsp, 1);
        }
        break;
    }
    case SIM_ATT_WRITE_CMD:
        (void)sim_gatt_write(p, n, true);
        break;
    default:
        if (req) {
            sim_gatt_error(op, 0, SIM_ATT_ERR_NOT_SUPPORTED);
        }
        break;
    }
}

floss_err_t floss_sim_gatt_serve(uint16_t handle, const floss_sim_gatt_attr_t *attrs, uint8_t count,
                                 uint16_t server_mtu) {
    sim_link_t *l = link_open(handle);
    if (!l || !l->pub.le) {
        return FLOSS_ERR_NOT_FOUND;
    }
    if ((count && !attrs) || count > FLOSS_SIM_GATT_MAX_ATTRS) {
        return FLOSS_ERR_INVALID;
    }
    for (unsigned i = 1; i < count; i++) {
        if (attrs[i].handle <= attrs[i - 1].handle) {
            return FLOSS_ERR_INVALID; /* the table must be sorted by handle */
        }
    }
    memset(&sim.gatt, 0, sizeof sim.gatt);
    for (unsigned i = 0; i < count; i++) {
        sim_gatt_attr_t *a = &sim.gatt.attrs[i];
        a->handle = attrs[i].handle;
        a->type16 = attrs[i].type16;
        a->is128 = attrs[i].type128 != NULL;
        if (a->is128) {
            memcpy(a->type128, attrs[i].type128, 16);
        }
        a->error = attrs[i].error;
        a->len = FLOSS_MIN(attrs[i].len, (uint16_t)FLOSS_SIM_GATT_VALUE_MAX);
        if (attrs[i].value && a->len) {
            memcpy(a->value, attrs[i].value, a->len);
        }
    }
    sim.gatt.count = count;
    sim.gatt.link = handle;
    sim.gatt.server_mtu = server_mtu < SIM_ATT_DEFAULT_MTU ? SIM_ATT_DEFAULT_MTU : server_mtu;
    sim.gatt.active = true;
    return FLOSS_OK;
}

/** Any ATT response the remote sends on the served link (scripted or by the test) answers the pending request. */
static void sim_gatt_on_remote_tx(uint16_t handle, const uint8_t *p, uint16_t n) {
    if (sim.gatt.link == handle && n > 0 && (p[0] & 1) && p[0] != SIM_ATT_NOTIFY && p[0] != SIM_ATT_INDICATE) {
        sim.gatt.req_pending = false;
    }
}

void floss_sim_gatt_silence(bool silent) {
    sim.gatt.silent = silent;
    if (!silent) {
        sim.gatt.req_pending = false;
    }
}

int floss_sim_gatt_value(uint16_t attr_handle, uint8_t *out, uint16_t cap) {
    const sim_gatt_attr_t *a = sim_gatt_attr(attr_handle);
    if (!a) {
        return -1;
    }
    memcpy(out, a->value, FLOSS_MIN(a->len, cap));
    return a->len;
}

floss_err_t floss_sim_gatt_notify(uint16_t attr_handle, const uint8_t *data, uint16_t len, bool indicate) {
    if (!sim.gatt.active) {
        return FLOSS_ERR_STATE;
    }
    if (len + 3u > sim_gatt_mtu() || (len && !data)) {
        return FLOSS_ERR_INVALID;
    }
    uint8_t pdu[FLOSS_SIM_ACL_MAX];
    pdu[0] = indicate ? SIM_ATT_INDICATE : SIM_ATT_NOTIFY;
    floss_put_le16(pdu + 1, attr_handle);
    if (len) {
        memcpy(pdu + 3, data, len);
    }
    return floss_sim_fixed_send(sim.gatt.link, FLOSS_SIM_CID_ATT, pdu, (uint16_t)(3 + len));
}

const floss_sim_gatt_stats_t *floss_sim_gatt_stats(void) {
    return &sim.gatt.stats;
}
