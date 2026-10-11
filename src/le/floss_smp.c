/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_smp.c - LE Security Manager: pairing state machine, LTK requests, bonds.
 *
 * Overview
 * --------
 * One smp_conn_t per LE link. Its `state` says which SMP PDU we wait for
 * next; the nested smp_pairing_t holds everything that only lives during a
 * pairing (nonces, keys, DHKey) and is wiped with one memset when pairing
 * ends, so no key material lingers.
 *
 * Pairing (Core Spec Vol 3 Part H, section 2.3):
 *
 *   Phase 1  Pairing Request/Response -> smp_negotiate() picks legacy or
 *            Secure Connections (SC), the association model, key size and
 *            which keys each side distributes.
 *   Phase 2  legacy: Confirm/Random with c1, STK = s1(TK, Srand, Mrand)
 *            SC:     Public Key exchange, Confirm/Random with f4 (once, or 20
 *                    rounds for Passkey Entry), user check with g2, then
 *                    DHKey Check with f5/f6 -> LTK
 *            The central then encrypts the link (we answer the controller's
 *            LTK Request when we are the peripheral).
 *   Phase 3  Over the encrypted link the responder distributes its keys,
 *            then the initiator; the bond is stored and PAIRING_COMPLETE
 *            reported.
 *
 * Several inputs arrive asynchronously in any order: the user's answer
 * (passkey / numeric comparison), the controller's P-256 public key and
 * DHKey, and the peer's PDUs. Instead of a state for every combination, the
 * "try" functions (smp_try_send_confirm, sc_send_public_key,
 * sc_try_dhkey_check) check all of their preconditions and are simply called
 * again whenever one of those inputs arrives.
 *
 * ECDH runs in the controller. The local P-256 key pair is generated once
 * (pre-warmed on the first link) and cached; asking again would replace the
 * private key the controller uses for DHKey generation. Only one DHKey
 * generation can be outstanding in the controller, so requests are queued
 * per link (dhkey_queued) and served one at a time (s_dhkey_owner).
 *
 * Byte order: every key, nonce and address is held in wire order (LSB
 * first) exactly as it appears in SMP PDUs and HCI packets; the toolbox in
 * floss_smp_crypto.c converts to the specification's MSB-first form.
 *
 * Bonds are cached in RAM (s_bonds) and written through to the HAL store as
 * an explicit, versioned 56-byte record (smp_bond_serialize). Our IRK is
 * generated once and kept at FLOSS_SMP_STORE_KEY_IRK.
 */
#include "floss/floss_config.h"

#if FLOSS_ENABLE_LE

#include "floss_internal.h"
#include "floss/floss_smp.h"
#include "floss_smp_crypto.h"

/* ------------------------------------------------------------------------ */
/* Protocol constants                                                       */
/* ------------------------------------------------------------------------ */

#define SMP_CID          0x0006
#define SMP_TIMEOUT_US   30000000u /* Vol 3 Part H 3.4: 30 s per SMP transaction */
/* The controller's P-256 key / DHKey work takes well under a second. Past
 * this, its completion event is presumed lost (a broken controller would
 * otherwise block LE Secure Connections on every link until reboot). */
#define SMP_CTRL_TIMEOUT_US 10000000u
#define SMP_MAX_KEY_SIZE 16
#define SMP_PASSKEY_MAX  999999u
#define SMP_PASSKEY_BITS 20        /* SC Passkey Entry: one round per bit */

/* SMP command codes */
#define SMP_PAIRING_REQUEST        0x01
#define SMP_PAIRING_RESPONSE       0x02
#define SMP_PAIRING_CONFIRM        0x03
#define SMP_PAIRING_RANDOM         0x04
#define SMP_PAIRING_FAILED         0x05
#define SMP_ENCRYPTION_INFORMATION 0x06
#define SMP_CENTRAL_IDENTIFICATION 0x07
#define SMP_IDENTITY_INFORMATION   0x08
#define SMP_IDENTITY_ADDRESS_INFO  0x09
#define SMP_SIGNING_INFORMATION    0x0A
#define SMP_SECURITY_REQUEST       0x0B
#define SMP_PAIRING_PUBLIC_KEY     0x0C
#define SMP_PAIRING_DHKEY_CHECK    0x0D
#define SMP_KEYPRESS_NOTIFICATION  0x0E
#define SMP_CODE_MAX               SMP_KEYPRESS_NOTIFICATION

/** Exact PDU length (code byte included) per command code. */
static const uint8_t s_pdu_len[SMP_CODE_MAX + 1] = {
    0,  /* 0x00 reserved */
    7,  /* Pairing Request */
    7,  /* Pairing Response */
    17, /* Pairing Confirm */
    17, /* Pairing Random */
    2,  /* Pairing Failed */
    17, /* Encryption Information */
    11, /* Central Identification */
    17, /* Identity Information */
    8,  /* Identity Address Information */
    17, /* Signing Information */
    2,  /* Security Request */
    65, /* Pairing Public Key */
    17, /* Pairing DHKey Check */
    2,  /* Keypress Notification */
};

/* AuthReq bits */
#define SMP_AUTH_BONDING      0x01
#define SMP_AUTH_BONDING_MASK 0x03
#define SMP_AUTH_MITM         0x04
#define SMP_AUTH_SC           0x08

/* HCI (LE controller commands and LE meta subevents used by SMP) */
#define SMP_OP_LE_ENABLE_ENCRYPTION FLOSS_OPCODE(0x08, 0x0019)
#define SMP_OP_LE_LTK_REPLY         FLOSS_OPCODE(0x08, 0x001A)
#define SMP_OP_LE_LTK_NEG_REPLY     FLOSS_OPCODE(0x08, 0x001B)
#define SMP_OP_LE_READ_LOCAL_P256   FLOSS_OPCODE(0x08, 0x0025)
#define SMP_OP_LE_GENERATE_DHKEY    FLOSS_OPCODE(0x08, 0x0026)
#define SMP_SUBEVT_LTK_REQUEST      0x05
#define SMP_SUBEVT_P256_COMPLETE    0x08
#define SMP_SUBEVT_DHKEY_COMPLETE   0x09

#define SMP_ROLE_CENTRAL    0
#define SMP_ROLE_PERIPHERAL 1

#define SMP_HANDLE_NONE   0xFFFF
#define SMP_HANDLE_ORPHAN 0xFFFE /* DHKey in flight for a link that went away */

/* Key distribution PDUs still expected from the peer. */
#define SMP_RX_ENC_INFO   0x01
#define SMP_RX_CENTRAL_ID 0x02
#define SMP_RX_ID_INFO    0x04
#define SMP_RX_ID_ADDR    0x08

/* Bond record (persistent storage format, version 1). */
#define SMP_BOND_VERSION 1
#define SMP_BOND_REC_LEN 56

/* ------------------------------------------------------------------------ */
/* Types                                                                    */
/* ------------------------------------------------------------------------ */

typedef enum {
    SMP_IDLE = 0,
    SMP_PHASE1,           /* responder: Pairing Request being answered */
    SMP_WAIT_PAIRING_RSP, /* initiator: Pairing Request sent */
    SMP_WAIT_PUBLIC_KEY,  /* SC: public key exchange */
    SMP_WAIT_CONFIRM,     /* waiting for the peer's Pairing Confirm */
    SMP_WAIT_RANDOM,      /* waiting for the peer's Pairing Random */
    SMP_WAIT_DHKEY_CHECK, /* SC: user check / DHKey / DHKey Check exchange */
    SMP_WAIT_ENCRYPTION,  /* link being encrypted with the STK / new LTK */
    SMP_KEY_DIST,         /* phase 3: key distribution */
    SMP_TIMED_OUT,        /* SMP timeout: no more SMP on this link (3.4) */
} smp_state_t;

/** Association models (Vol 3 Part H 2.3.5.1). */
typedef enum {
    SMP_JUST_WORKS = 0,
    SMP_NUMERIC_COMPARISON,  /* SC only */
    SMP_PASSKEY_INIT_INPUTS, /* responder displays, initiator inputs */
    SMP_PASSKEY_RESP_INPUTS, /* initiator displays, responder inputs */
    SMP_PASSKEY_BOTH_INPUT,  /* both users type the same passkey */
} smp_method_t;

/** What we are waiting for from the application. */
typedef enum {
    SMP_USER_NONE = 0,
    SMP_USER_PASSKEY, /* floss_smp_passkey_reply() */
    SMP_USER_CONFIRM, /* floss_smp_confirm() */
} smp_user_t;

/** State that only exists during one pairing (wiped when it ends). */
typedef struct {
    uint8_t preq[7];          /* Pairing Request PDU as on the air */
    uint8_t pres[7];          /* Pairing Response PDU as on the air */
    uint8_t method;           /* smp_method_t */
    uint8_t user;             /* smp_user_t */
    uint8_t key_size;
    uint8_t local_dist;       /* FLOSS_SMP_DIST_* we send */
    uint8_t rx_keys;          /* SMP_RX_* still expected */
    uint8_t round;            /* SC passkey entry round 0..19 */
    bool sc;                  /* LE Secure Connections */
    bool bonding;             /* both sides asked to bond */
    bool authenticated;       /* association model gives MITM protection */
    bool tk_ready;            /* TK / passkey known */
    bool local_confirm_sent;
    bool peer_confirm_rx;
    bool peer_pk_rx;
    bool pk_tx_pending;       /* our public key must go out once the controller has it */
    bool dhkey_queued;        /* waiting for the controller's DHKey slot */
    bool dhkey_inflight;      /* our LE Generate DHKey is in the controller */
    bool dhkey_ready;
    bool local_check_sent;    /* initiator: DHKey Check Ea sent */
    bool peer_check_rx;       /* responder: DHKey Check Ea received */
    bool got_identity;        /* peer sent Identity Address Information */
    uint32_t passkey;
    uint8_t tk[16];           /* legacy temporary key */
    uint8_t local_rand[16];   /* Mrand/Srand or Na/Nb (current round) */
    uint8_t peer_rand[16];
    uint8_t peer_confirm[16];
    uint8_t peer_check[16];   /* responder: received Ea; initiator: expected Eb */
    uint8_t peer_pk[64];      /* X || Y, wire order */
    uint8_t dhkey[32];
    uint8_t ltk[16];          /* STK (legacy) or LTK (SC) used to encrypt now */
    floss_smp_bond_t bond;    /* keys collected for the bond */
} smp_pairing_t;

typedef struct {
    bool used;
    uint8_t state;           /* smp_state_t */
    uint16_t handle;
    uint8_t role;            /* SMP_ROLE_* */
    uint8_t level;           /* current security level 1..4 */
    uint8_t peer_type;       /* over-the-air peer address */
    uint8_t local_type;      /* over-the-air local address */
    floss_addr_t peer_addr;
    floss_addr_t local_addr;
    int8_t bond;             /* s_bonds index for this peer, -1 none */
    int8_t enc_bond;         /* bond whose LTK the pending encryption uses, -1 none */
    floss_timer_t timer;     /* SMP transaction timeout */
    smp_pairing_t p;
} smp_conn_t;

/* ------------------------------------------------------------------------ */
/* Module state                                                             */
/* ------------------------------------------------------------------------ */

static floss_smp_config_t s_cfg;
static smp_conn_t s_conns[FLOSS_MAX_CONNECTIONS];

static struct {
    floss_smp_evt_fn fn;
    void *user;
} s_listeners[FLOSS_SMP_MAX_LISTENERS];

static floss_smp_bond_t s_bonds[FLOSS_MAX_BONDS];
static bool s_bond_valid[FLOSS_MAX_BONDS];
static uint32_t s_bond_seq; /* highest seq in use */

static uint8_t s_irk[16];
static bool s_local_addr_set;
static uint8_t s_local_type;
static floss_addr_t s_local_addr;

enum { PK_NONE = 0, PK_PENDING, PK_READY, PK_FAILED };
static uint8_t s_pk_state;
static uint8_t s_local_pk[64];    /* X || Y, wire order */
static uint16_t s_dhkey_owner;    /* handle whose DHKey the controller computes */
static floss_timer_t s_ctrl_timer; /* guards the P-256 key and DHKey requests */

/* Association model tables, [initiator IO capability][responder IO capability]
 * (Vol 3 Part H Table 2.8). The two differ only where both sides can show a
 * value and confirm it: Numeric Comparison exists only with SC. */
#define JW  SMP_JUST_WORKS
#define NC  SMP_NUMERIC_COMPARISON
#define PII SMP_PASSKEY_INIT_INPUTS
#define PRI SMP_PASSKEY_RESP_INPUTS
#define PBI SMP_PASSKEY_BOTH_INPUT
static const uint8_t s_method_legacy[5][5] = {
    /* responder:      DispOnly DispYesNo KbdOnly NoIO KbdDisp */
    /* DisplayOnly  */ {JW,      JW,       PRI,    JW,  PRI},
    /* DisplayYesNo */ {JW,      JW,       PRI,    JW,  PRI},
    /* KeyboardOnly */ {PII,     PII,      PBI,    JW,  PII},
    /* NoInputNoOut */ {JW,      JW,       JW,     JW,  JW},
    /* KbdDisplay   */ {PII,     PII,      PRI,    JW,  PRI},
};
static const uint8_t s_method_sc[5][5] = {
    /* responder:      DispOnly DispYesNo KbdOnly NoIO KbdDisp */
    /* DisplayOnly  */ {JW,      JW,       PRI,    JW,  PRI},
    /* DisplayYesNo */ {JW,      NC,       PRI,    JW,  NC},
    /* KeyboardOnly */ {PII,     PII,      PBI,    JW,  PII},
    /* NoInputNoOut */ {JW,      JW,       JW,     JW,  JW},
    /* KbdDisplay   */ {PII,     NC,       PRI,    JW,  NC},
};
#undef JW
#undef NC
#undef PII
#undef PRI
#undef PBI

static const char *const s_method_name[] = {"just works", "numeric comparison", "passkey (initiator inputs)",
                                            "passkey (responder inputs)", "passkey (both input)"};

/* Forward declarations of the asynchronous "try" steps. */
static bool smp_sc_supported(void);
static void smp_try_send_confirm(smp_conn_t *c);
static void sc_send_public_key(smp_conn_t *c);
static void sc_try_dhkey_check(smp_conn_t *c);

/* ------------------------------------------------------------------------ */
/* Small helpers                                                            */
/* ------------------------------------------------------------------------ */

static smp_conn_t *smp_conn(uint16_t handle) {
    for (unsigned i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        if (s_conns[i].used && s_conns[i].handle == handle) {
            return &s_conns[i];
        }
    }
    return NULL;
}

static inline bool smp_is_initiator(const smp_conn_t *c) {
    return c->role == SMP_ROLE_CENTRAL;
}

static inline bool smp_pairing_active(const smp_conn_t *c) {
    return c->state != SMP_IDLE && c->state != SMP_TIMED_OUT;
}

static inline bool smp_method_is_passkey(uint8_t method) {
    return method >= SMP_PASSKEY_INIT_INPUTS;
}

/** Constant-time comparison of confirm / check values. */
static bool smp_equal16(const uint8_t *a, const uint8_t *b) {
    uint8_t diff = 0;
    for (unsigned i = 0; i < 16; i++) {
        diff |= (uint8_t)(a[i] ^ b[i]);
    }
    return diff == 0;
}

static bool smp_is_zero(const uint8_t *p, unsigned n) {
    uint8_t acc = 0;
    for (unsigned i = 0; i < n; i++) {
        acc |= p[i];
    }
    return acc == 0;
}

/**
 * A key negotiated shorter than 16 octets is the 128-bit value with its most
 * significant octets zeroed (Vol 3 Part H 2.3.4). Wire order: zero the tail.
 */
static void smp_mask_key(uint8_t key[16], uint8_t key_size) {
    if (key_size < 16) {
        memset(&key[key_size], 0, 16u - key_size);
    }
}

static uint8_t smp_level(bool authenticated, bool sc, uint8_t key_size) {
    if (!authenticated) {
        return 2;
    }
    return (sc && key_size == 16) ? 4 : 3;
}

static uint8_t smp_bond_level(const floss_smp_bond_t *b) {
    return smp_level((b->flags & FLOSS_SMP_BOND_AUTHENTICATED) != 0, (b->flags & FLOSS_SMP_BOND_SC) != 0,
                     b->key_size);
}

static void smp_set_level(smp_conn_t *c, uint8_t level) {
    if (c->level != level) {
        c->level = level;
        floss_le_on_security_changed(c->handle, level);
    }
}

/** Initiator (A) and responder (B) over-the-air addresses of this link. */
static void smp_addrs(const smp_conn_t *c, uint8_t *at, const floss_addr_t **a, uint8_t *bt,
                      const floss_addr_t **b) {
    if (smp_is_initiator(c)) {
        *at = c->local_type;
        *a = &c->local_addr;
        *bt = c->peer_type;
        *b = &c->peer_addr;
    } else {
        *at = c->peer_type;
        *a = &c->peer_addr;
        *bt = c->local_type;
        *b = &c->local_addr;
    }
}

/* ------------------------------------------------------------------------ */
/* Events                                                                   */
/* ------------------------------------------------------------------------ */

/** Event pre-filled with the link and the best known peer address (identity if bonded). */
static void smp_evt_init(floss_smp_evt_t *evt, floss_smp_evt_type_t type, const smp_conn_t *c) {
    memset(evt, 0, sizeof *evt);
    evt->type = type;
    if (c) {
        evt->handle = c->handle;
        evt->level = c->level;
        if (c->bond >= 0) {
            evt->addr_type = s_bonds[c->bond].addr_type;
            evt->addr = s_bonds[c->bond].addr;
        } else {
            evt->addr_type = c->peer_type;
            evt->addr = c->peer_addr;
        }
    }
}

static void smp_emit(const floss_smp_evt_t *evt) {
    for (unsigned i = 0; i < FLOSS_SMP_MAX_LISTENERS; i++) {
        if (s_listeners[i].fn) {
            s_listeners[i].fn(evt, s_listeners[i].user);
        }
    }
}

static void smp_emit_simple(floss_smp_evt_type_t type, const smp_conn_t *c) {
    floss_smp_evt_t evt;
    smp_evt_init(&evt, type, c);
    smp_emit(&evt);
}

/* ------------------------------------------------------------------------ */
/* Sending                                                                  */
/* ------------------------------------------------------------------------ */

/**
 * Send an SMP PDU. Every command sent during pairing restarts the 30 s SMP
 * timer (Vol 3 Part H 3.4). A failed send is only logged: the timer then
 * cleans up, which keeps every caller free of error unwinding.
 */
static void smp_send(smp_conn_t *c, const uint8_t *pdu, uint16_t len) {
    if (smp_pairing_active(c)) {
        floss_timer_start(&c->timer, SMP_TIMEOUT_US);
    }
    floss_err_t err = floss_le_send_fixed(c->handle, SMP_CID, pdu, len);
    if (err != FLOSS_OK) {
        FLOSS_LOGW(SMP, "SMP %u: send 0x%02x failed: %s", c->handle, pdu[0], floss_err_str(err));
    }
}

static void smp_send_value(smp_conn_t *c, uint8_t code, const uint8_t value[16]) {
    uint8_t pdu[17];
    pdu[0] = code;
    memcpy(&pdu[1], value, 16);
    smp_send(c, pdu, sizeof pdu);
}

static void smp_send_failed_raw(uint16_t handle, uint8_t reason) {
    uint8_t pdu[2] = {SMP_PAIRING_FAILED, reason};
    floss_le_send_fixed(handle, SMP_CID, pdu, sizeof pdu);
}

/** Our AuthReq octet for Pairing Request/Response and Security Request. */
static uint8_t smp_local_auth_req(void) {
    uint8_t auth = 0;
    if (s_cfg.bondable) {
        auth |= SMP_AUTH_BONDING;
    }
    if (s_cfg.mitm) {
        auth |= SMP_AUTH_MITM;
    }
    if (smp_sc_supported()) {
        auth |= SMP_AUTH_SC;
    }
    return auth;
}

/* ------------------------------------------------------------------------ */
/* Pairing lifecycle                                                        */
/* ------------------------------------------------------------------------ */

static void smp_pairing_begin(smp_conn_t *c, smp_state_t state) {
    memset(&c->p, 0, sizeof c->p);
    c->state = (uint8_t)state;
    floss_timer_start(&c->timer, SMP_TIMEOUT_US);
    smp_emit_simple(FLOSS_SMP_EVT_PAIRING_STARTED, c);
}

/** Leave pairing: stop the timer and wipe nonces/keys. */
static void smp_pairing_end(smp_conn_t *c, smp_state_t next) {
    floss_timer_stop(&c->timer);
    c->state = (uint8_t)next;
    memset(&c->p, 0, sizeof c->p);
}

static void smp_report_failed(smp_conn_t *c, uint8_t reason) {
    floss_smp_evt_t evt;
    smp_evt_init(&evt, FLOSS_SMP_EVT_PAIRING_FAILED, c);
    evt.reason = reason;
    smp_emit(&evt);
}

/**
 * Abort the pairing locally: tell the peer why (Pairing Failed) and report
 * it. Callers must return right after calling this.
 */
static void smp_fail(smp_conn_t *c, uint8_t reason) {
    if (!smp_pairing_active(c)) {
        return;
    }
    FLOSS_LOGW(SMP, "SMP %u: pairing failed, reason 0x%02x", c->handle, reason);
    smp_send_failed_raw(c->handle, reason);
    smp_pairing_end(c, SMP_IDLE);
    smp_report_failed(c, reason);
}

/** Reject an invalid PDU: abort pairing if one runs, else just answer Pairing Failed. */
static void smp_reject(smp_conn_t *c, uint8_t reason) {
    if (smp_pairing_active(c)) {
        smp_fail(c, reason);
    } else {
        smp_send_failed_raw(c->handle, reason);
    }
}

/**
 * A well-formed PDU that does not fit the current state. During pairing that
 * is fatal; outside pairing it is a straggler from an aborted pairing and is
 * dropped, so a late PDU cannot fail a pairing the peer just restarted.
 */
static void smp_unexpected(smp_conn_t *c, uint8_t code) {
    if (smp_pairing_active(c)) {
        FLOSS_LOGW(SMP, "SMP %u: unexpected 0x%02x in state %u", c->handle, code, c->state);
        smp_fail(c, FLOSS_SMP_REASON_UNSPECIFIED);
    } else {
        FLOSS_LOGD(SMP, "SMP %u: ignoring 0x%02x outside pairing", c->handle, code);
    }
}

static void smp_timeout(floss_timer_t *t, void *arg) {
    smp_conn_t *c = (smp_conn_t *)arg;
    if (!smp_pairing_active(c)) {
        return;
    }
    /* No Pairing Failed: after a timeout no SMP command may be sent on this
     * link; a new pairing needs a new link (Vol 3 Part H 3.4). */
    FLOSS_LOGW(SMP, "SMP %u: timeout", c->handle);
    smp_pairing_end(c, SMP_TIMED_OUT);
    smp_report_failed(c, FLOSS_SMP_REASON_TIMEOUT);
}

/* ------------------------------------------------------------------------ */
/* Bond database                                                            */
/* ------------------------------------------------------------------------ */

static void smp_bond_serialize(const floss_smp_bond_t *b, uint8_t rec[SMP_BOND_REC_LEN]) {
    rec[0] = SMP_BOND_VERSION;
    rec[1] = b->flags;
    rec[2] = b->key_size;
    rec[3] = b->addr_type;
    memcpy(&rec[4], b->addr.b, 6);
    memcpy(&rec[10], b->ltk, 16);
    floss_put_le16(&rec[26], b->ediv);
    memcpy(&rec[28], b->rand, 8);
    memcpy(&rec[36], b->peer_irk, 16);
    floss_put_le32(&rec[52], b->seq);
}

static bool smp_bond_parse(const uint8_t *rec, int len, floss_smp_bond_t *b) {
    if (len != SMP_BOND_REC_LEN || rec[0] != SMP_BOND_VERSION || rec[2] < 7 || rec[2] > 16) {
        return false;
    }
    b->flags = rec[1];
    b->key_size = rec[2];
    b->addr_type = rec[3];
    memcpy(b->addr.b, &rec[4], 6);
    memcpy(b->ltk, &rec[10], 16);
    b->ediv = floss_get_le16(&rec[26]);
    memcpy(b->rand, &rec[28], 8);
    memcpy(b->peer_irk, &rec[36], 16);
    b->seq = floss_get_le32(&rec[52]);
    return true;
}

static void smp_bond_write(unsigned idx) {
    uint8_t rec[SMP_BOND_REC_LEN];
    smp_bond_serialize(&s_bonds[idx], rec);
    floss_err_t err = floss_hal_store_write((uint16_t)(FLOSS_SMP_STORE_KEY_BOND_BASE + idx), rec, sizeof rec);
    if (err != FLOSS_OK) {
        FLOSS_LOGW(SMP, "SMP: bond %u store failed: %s", idx, floss_err_str(err));
    }
    memset(rec, 0, sizeof rec);
}

static void smp_bonds_load(void) {
    uint8_t rec[SMP_BOND_REC_LEN + 1]; /* +1 detects over-long records */
    s_bond_seq = 0;
    for (unsigned i = 0; i < FLOSS_MAX_BONDS; i++) {
        int n = floss_hal_store_read((uint16_t)(FLOSS_SMP_STORE_KEY_BOND_BASE + i), rec, sizeof rec);
        s_bond_valid[i] = smp_bond_parse(rec, n, &s_bonds[i]);
        if (s_bond_valid[i] && s_bonds[i].seq > s_bond_seq) {
            s_bond_seq = s_bonds[i].seq;
        }
    }
    memset(rec, 0, sizeof rec);
}

static int smp_bond_find_identity(uint8_t type, const floss_addr_t *addr) {
    for (unsigned i = 0; i < FLOSS_MAX_BONDS; i++) {
        if (s_bond_valid[i] && s_bonds[i].addr_type == (type & 1u) && floss_addr_eq(&s_bonds[i].addr, addr)) {
            return (int)i;
        }
    }
    return -1;
}

/** Identity match, else resolvable private address resolution with each bonded IRK. */
static int smp_bond_find(uint8_t type, const floss_addr_t *addr) {
    int idx = smp_bond_find_identity(type, addr);
    if (idx >= 0 || (type & 1u) == 0 || (addr->b[5] & 0xC0) != 0x40) {
        return idx;
    }
    for (unsigned i = 0; i < FLOSS_MAX_BONDS; i++) {
        if (s_bond_valid[i] && (s_bonds[i].flags & FLOSS_SMP_BOND_HAS_IRK) &&
            floss_smp_rpa_matches(s_bonds[i].peer_irk, addr)) {
            return (int)i;
        }
    }
    return -1;
}

/** Forget a bond slot: storage, RAM cache, links that referenced it. */
static void smp_bond_remove(unsigned idx) {
    floss_smp_evt_t evt;
    smp_evt_init(&evt, FLOSS_SMP_EVT_BOND_DELETED, NULL);
    evt.addr_type = s_bonds[idx].addr_type;
    evt.addr = s_bonds[idx].addr;

    floss_hal_store_delete((uint16_t)(FLOSS_SMP_STORE_KEY_BOND_BASE + idx));
    s_bond_valid[idx] = false;
    memset(&s_bonds[idx], 0, sizeof s_bonds[idx]);
    for (unsigned i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        if (s_conns[i].used && s_conns[i].bond == (int8_t)idx) {
            s_conns[i].bond = -1;
        }
        if (s_conns[i].used && s_conns[i].enc_bond == (int8_t)idx) {
            s_conns[i].enc_bond = -1;
        }
    }
    smp_emit(&evt);
}

/** Store a new bond: replaces the peer's old bond, else a free slot, else the least recently used. */
static int smp_bond_save(const floss_smp_bond_t *nb) {
    int idx = smp_bond_find_identity(nb->addr_type, &nb->addr);
    if (idx < 0) {
        for (unsigned i = 0; i < FLOSS_MAX_BONDS; i++) {
            if (!s_bond_valid[i]) {
                idx = (int)i;
                break;
            }
        }
    }
    if (idx < 0) {
        idx = 0;
        for (unsigned i = 1; i < FLOSS_MAX_BONDS; i++) {
            if (s_bonds[i].seq < s_bonds[idx].seq) {
                idx = (int)i;
            }
        }
        FLOSS_LOGI(SMP, "SMP: bond table full, evicting slot %d", idx);
        smp_bond_remove((unsigned)idx);
    }
    s_bonds[idx] = *nb;
    s_bonds[idx].seq = ++s_bond_seq;
    s_bond_valid[idx] = true;
    smp_bond_write((unsigned)idx);
    return idx;
}

/** Mark a bond most recently used (one flash write, skipped if it already is). */
static void smp_bond_touch(unsigned idx) {
    if (s_bonds[idx].seq != s_bond_seq) {
        s_bonds[idx].seq = ++s_bond_seq;
        smp_bond_write(idx);
    }
}

static void smp_irk_load(void) {
    if (floss_hal_store_read(FLOSS_SMP_STORE_KEY_IRK, s_irk, sizeof s_irk) == (int)sizeof s_irk) {
        return;
    }
    /* First boot: our IRK is generated once and kept forever, so bonded
     * peers can keep resolving our private addresses. */
    floss_hal_random(s_irk, sizeof s_irk);
    if (floss_hal_store_write(FLOSS_SMP_STORE_KEY_IRK, s_irk, sizeof s_irk) != FLOSS_OK) {
        FLOSS_LOGW(SMP, "SMP: could not persist IRK");
    }
}

/* ------------------------------------------------------------------------ */
/* Controller: P-256 key, DHKey, LTK, encryption                            */
/* ------------------------------------------------------------------------ */

static bool smp_sc_supported(void) {
    if (!s_cfg.secure_connections || s_pk_state == PK_FAILED) {
        return false;
    }
    /* If HCI read the supported-commands bitmap, require LE Read Local P-256
     * Public Key (octet 34 bit 1) and LE Generate DHKey (octet 34 bit 2). */
    const floss_controller_info_t *ci = floss_hci_controller();
    if (!smp_is_zero(ci->commands, sizeof ci->commands)) {
        return floss_hci_cmd_supported(34, 1) && floss_hci_cmd_supported(34, 2);
    }
    return true;
}

static void smp_pk_failed(void) {
    FLOSS_LOGW(SMP, "SMP: controller cannot provide a P-256 key, LE Secure Connections disabled");
    s_pk_state = PK_FAILED;
    for (unsigned i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        if (s_conns[i].used && s_conns[i].p.pk_tx_pending) {
            smp_fail(&s_conns[i], FLOSS_SMP_REASON_UNSPECIFIED);
        }
    }
}

static void smp_pk_cmd_cb(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len) {
    if (status != FLOSS_HCI_SUCCESS) {
        smp_pk_failed();
    }
}

/** Ask the controller for its P-256 key pair, once. */
static void smp_pk_request(void) {
    if (s_pk_state != PK_NONE) {
        return;
    }
    /* State first: the HCI layer may report a failure from inside the call. */
    s_pk_state = PK_PENDING;
    if (floss_hci_cmd(SMP_OP_LE_READ_LOCAL_P256, NULL, 0, smp_pk_cmd_cb, NULL) != FLOSS_OK &&
        s_pk_state == PK_PENDING) {
        s_pk_state = PK_NONE; /* queue full: retried on the next need */
    }
    if (s_pk_state == PK_PENDING) {
        floss_timer_start(&s_ctrl_timer, SMP_CTRL_TIMEOUT_US);
    }
}

static void smp_on_p256_complete(const uint8_t *params, uint8_t len) {
    if (len < 65 || params[0] != FLOSS_HCI_SUCCESS) {
        smp_pk_failed();
        return;
    }
    memcpy(s_local_pk, &params[1], 64);
    s_pk_state = PK_READY;
    for (unsigned i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        if (s_conns[i].used && s_conns[i].p.pk_tx_pending) {
            sc_send_public_key(&s_conns[i]);
        }
    }
}

static void smp_dhkey_done(uint8_t status, const uint8_t *dhkey);

static void smp_dhkey_cmd_cb(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len) {
    if (status != FLOSS_HCI_SUCCESS) {
        smp_dhkey_done(status, NULL); /* no completion event will follow */
    }
}

/** Start the next queued DHKey generation if the controller is free. */
static void smp_dhkey_kick(void) {
    if (s_dhkey_owner != SMP_HANDLE_NONE) {
        return;
    }
    for (unsigned i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        smp_conn_t *c = &s_conns[i];
        if (!c->used || !c->p.dhkey_queued) {
            continue;
        }
        c->p.dhkey_queued = false;
        /* Owner first: the HCI layer may report a failure from inside the call. */
        c->p.dhkey_inflight = true;
        s_dhkey_owner = c->handle;
        if (floss_hci_cmd(SMP_OP_LE_GENERATE_DHKEY, c->p.peer_pk, 64, smp_dhkey_cmd_cb, NULL) != FLOSS_OK &&
            s_dhkey_owner == c->handle) {
            c->p.dhkey_inflight = false;
            s_dhkey_owner = SMP_HANDLE_NONE;
            smp_fail(c, FLOSS_SMP_REASON_UNSPECIFIED);
            continue;
        }
        if (s_dhkey_owner == c->handle) {
            floss_timer_start(&s_ctrl_timer, SMP_CTRL_TIMEOUT_US);
        }
        return;
    }
}

static void smp_dhkey_done(uint8_t status, const uint8_t *dhkey) {
    uint16_t owner = s_dhkey_owner;
    s_dhkey_owner = SMP_HANDLE_NONE;
    smp_conn_t *c = (owner < SMP_HANDLE_ORPHAN) ? smp_conn(owner) : NULL;
    if (c && c->p.dhkey_inflight) {
        c->p.dhkey_inflight = false;
        if (status != FLOSS_HCI_SUCCESS || !dhkey) {
            /* The controller rejects public keys that are not on the curve. */
            smp_fail(c, FLOSS_SMP_REASON_DHKEY_CHECK_FAILED);
        } else {
            memcpy(c->p.dhkey, dhkey, 32);
            c->p.dhkey_ready = true;
            sc_try_dhkey_check(c);
        }
    }
    smp_dhkey_kick();
}

/** The controller never answered a P-256 key or DHKey request: give up on it. */
static void smp_ctrl_timeout(floss_timer_t *t, void *arg) {
    FLOSS_UNUSED(t);
    FLOSS_UNUSED(arg);
    if (s_pk_state == PK_PENDING) {
        smp_pk_failed();
    }
    if (s_dhkey_owner != SMP_HANDLE_NONE) {
        FLOSS_LOGW(SMP, "SMP: no DHKey from the controller, failing that pairing");
        smp_dhkey_done(0xFF /* no answer */, NULL);
    }
}

static void smp_ltk_reply(uint16_t handle, const uint8_t ltk[16]) {
    uint8_t cmd[18];
    floss_put_le16(cmd, handle);
    memcpy(&cmd[2], ltk, 16);
    if (floss_hci_cmd(SMP_OP_LE_LTK_REPLY, cmd, sizeof cmd, NULL, NULL) != FLOSS_OK) {
        FLOSS_LOGE(SMP, "SMP %u: cannot queue LTK reply", handle);
    }
    memset(cmd, 0, sizeof cmd);
}

static void smp_ltk_neg_reply(uint16_t handle) {
    uint8_t cmd[2];
    floss_put_le16(cmd, handle);
    if (floss_hci_cmd(SMP_OP_LE_LTK_NEG_REPLY, cmd, sizeof cmd, NULL, NULL) != FLOSS_OK) {
        FLOSS_LOGE(SMP, "SMP %u: cannot queue LTK negative reply", handle);
    }
}

static void smp_enc_cmd_cb(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len) {
    if (status == FLOSS_HCI_SUCCESS) {
        return;
    }
    smp_conn_t *c = smp_conn((uint16_t)(uintptr_t)ctx);
    if (!c) {
        return;
    }
    FLOSS_LOGW(SMP, "SMP %u: LE Enable Encryption rejected (0x%02x)", c->handle, status);
    c->enc_bond = -1;
    if (c->state == SMP_WAIT_ENCRYPTION) {
        smp_fail(c, FLOSS_SMP_REASON_UNSPECIFIED);
    }
}

/** Central: encrypt the link (HCI LE Enable Encryption). */
static void smp_start_encryption(smp_conn_t *c, const uint8_t rand[8], uint16_t ediv, const uint8_t ltk[16]) {
    uint8_t cmd[28];
    floss_put_le16(&cmd[0], c->handle);
    memcpy(&cmd[2], rand, 8);
    floss_put_le16(&cmd[10], ediv);
    memcpy(&cmd[12], ltk, 16);
    if (floss_hci_cmd(SMP_OP_LE_ENABLE_ENCRYPTION, cmd, sizeof cmd, smp_enc_cmd_cb,
                      (void *)(uintptr_t)c->handle) != FLOSS_OK) {
        smp_enc_cmd_cb((void *)(uintptr_t)c->handle, SMP_OP_LE_ENABLE_ENCRYPTION, 0xFF, NULL, 0);
    }
    memset(cmd, 0, sizeof cmd);
}

/**
 * Peripheral: the central started encryption and the controller asks for
 * the key identified by (EDIV, Rand). During pairing that is the STK / new
 * LTK (EDIV = Rand = 0). Otherwise it is a bond: legacy keys are found by the
 * EDIV/Rand we handed out, SC keys (EDIV = Rand = 0) by the peer's identity.
 */
static void smp_on_ltk_request(const uint8_t *params, uint8_t len) {
    if (len < 12) {
        return;
    }
    uint16_t handle = floss_get_le16(params) & 0x0FFF;
    const uint8_t *rand = &params[2];
    uint16_t ediv = floss_get_le16(&params[10]);
    bool zero = ediv == 0 && smp_is_zero(rand, 8);
    smp_conn_t *c = smp_conn(handle);

    if (c && c->state == SMP_WAIT_ENCRYPTION && !smp_is_initiator(c) && zero) {
        c->enc_bond = -1;
        smp_ltk_reply(handle, c->p.ltk);
        return;
    }

    int idx = -1;
    if (c && zero) {
        if (c->bond >= 0 && (s_bonds[c->bond].flags & FLOSS_SMP_BOND_SC)) {
            idx = c->bond;
        }
    } else if (c) {
        for (unsigned i = 0; i < FLOSS_MAX_BONDS; i++) {
            const floss_smp_bond_t *b = &s_bonds[i];
            if (s_bond_valid[i] && !(b->flags & (FLOSS_SMP_BOND_SC | FLOSS_SMP_BOND_PEER_LTK)) && b->ediv == ediv &&
                memcmp(b->rand, rand, 8) == 0) {
                idx = (int)i;
                break;
            }
        }
    }
    if (idx < 0) {
        FLOSS_LOGI(SMP, "SMP %u: no key for LTK request (ediv 0x%04x)", handle, ediv);
        smp_ltk_neg_reply(handle);
        return;
    }
    c->enc_bond = (int8_t)idx;
    smp_ltk_reply(handle, s_bonds[idx].ltk);
}

/* ------------------------------------------------------------------------ */
/* Phase 1: feature exchange                                                */
/* ------------------------------------------------------------------------ */

/**
 * Both Pairing Request and Response are known: derive the pairing
 * parameters. Returns 0 or the Pairing Failed reason.
 */
static uint8_t smp_negotiate(smp_conn_t *c) {
    smp_pairing_t *p = &c->p;
    const uint8_t *rq = p->preq, *rs = p->pres;
    bool init = smp_is_initiator(c);

    if (rq[1] > FLOSS_SMP_IO_KEYBOARD_DISPLAY || rs[1] > FLOSS_SMP_IO_KEYBOARD_DISPLAY || rq[4] > 16 ||
        rs[4] > 16) {
        return FLOSS_SMP_REASON_INVALID_PARAMETERS;
    }
    /* The responder may only narrow the key distribution. */
    if (init && ((rs[5] & ~rq[5]) || (rs[6] & ~rq[6]))) {
        return FLOSS_SMP_REASON_INVALID_PARAMETERS;
    }
    p->key_size = FLOSS_MIN(rq[4], rs[4]);
    if (p->key_size < FLOSS_MAX(7, s_cfg.min_key_size)) {
        return FLOSS_SMP_REASON_ENC_KEY_SIZE;
    }
    p->sc = (rq[3] & SMP_AUTH_SC) && (rs[3] & SMP_AUTH_SC);
    if (s_cfg.secure_connections_only && (!p->sc || p->key_size != 16)) {
        return p->sc ? FLOSS_SMP_REASON_ENC_KEY_SIZE : FLOSS_SMP_REASON_AUTH_REQUIREMENTS;
    }
    p->bonding = (rq[3] & SMP_AUTH_BONDING_MASK) == SMP_AUTH_BONDING &&
                 (rs[3] & SMP_AUTH_BONDING_MASK) == SMP_AUTH_BONDING;

    /* IO capabilities only matter if at least one side asks for MITM. */
    if (((rq[3] | rs[3]) & SMP_AUTH_MITM) == 0) {
        p->method = SMP_JUST_WORKS;
    } else {
        p->method = (p->sc ? s_method_sc : s_method_legacy)[rq[1]][rs[1]];
    }
    p->authenticated = p->method != SMP_JUST_WORKS;
    if (s_cfg.mitm && !p->authenticated) {
        return FLOSS_SMP_REASON_AUTH_REQUIREMENTS;
    }

    /* Agreed distribution is in the Pairing Response. SC derives the LTK, so
     * EncKey is ignored there; CSRK and cross-transport keys are unsupported. */
    uint8_t mask = FLOSS_SMP_DIST_ENC | FLOSS_SMP_DIST_ID;
    if (p->sc) {
        mask = FLOSS_SMP_DIST_ID;
    }
    uint8_t init_dist = rs[5] & mask, resp_dist = rs[6] & mask;
    uint8_t peer_dist = init ? resp_dist : init_dist;
    p->local_dist = init ? init_dist : resp_dist;
    p->rx_keys = (uint8_t)(((peer_dist & FLOSS_SMP_DIST_ENC) ? (SMP_RX_ENC_INFO | SMP_RX_CENTRAL_ID) : 0) |
                           ((peer_dist & FLOSS_SMP_DIST_ID) ? (SMP_RX_ID_INFO | SMP_RX_ID_ADDR) : 0));

    FLOSS_LOGI(SMP, "SMP %u: %s pairing, %s, key size %u%s", c->handle, p->sc ? "SC" : "legacy",
               s_method_name[p->method], p->key_size, p->bonding ? ", bonding" : "");
    return 0;
}

static void smp_set_passkey(smp_pairing_t *p, uint32_t passkey) {
    p->passkey = passkey;
    memset(p->tk, 0, sizeof p->tk);
    floss_put_le32(p->tk, passkey); /* legacy TK = passkey as a 128-bit number */
    p->tk_ready = true;
}

/** Phase 1 done: involve the user if needed, then start phase 2. */
static void smp_features_exchanged(smp_conn_t *c) {
    smp_pairing_t *p = &c->p;
    bool init = smp_is_initiator(c);

    if (!smp_method_is_passkey(p->method)) {
        p->tk_ready = true; /* Just Works / Numeric Comparison: TK = 0 */
    } else if ((p->method == SMP_PASSKEY_INIT_INPUTS && !init) || (p->method == SMP_PASSKEY_RESP_INPUTS && init)) {
        /* We display, the other side types it in. */
        uint8_t r[4];
        floss_hal_random(r, sizeof r);
        smp_set_passkey(p, floss_get_le32(r) % (SMP_PASSKEY_MAX + 1));
        floss_smp_evt_t evt;
        smp_evt_init(&evt, FLOSS_SMP_EVT_PASSKEY_DISPLAY, c);
        evt.passkey = p->passkey;
        smp_emit(&evt);
    } else {
        p->user = SMP_USER_PASSKEY;
        smp_emit_simple(FLOSS_SMP_EVT_PASSKEY_INPUT, c);
    }
    if (!smp_pairing_active(c)) {
        return; /* the application rejected the passkey from inside the callback */
    }

    if (!p->sc) {
        c->state = SMP_WAIT_CONFIRM;
        smp_try_send_confirm(c); /* initiator goes first with Mconfirm */
    } else {
        c->state = SMP_WAIT_PUBLIC_KEY;
        if (init) {
            sc_send_public_key(c); /* initiator goes first with its key */
        }
    }
}

/** Responder: Pairing Request received. */
static void smp_rx_pairing_request(smp_conn_t *c, const uint8_t *d) {
    if (c->role != SMP_ROLE_PERIPHERAL) {
        smp_reject(c, FLOSS_SMP_REASON_CMD_NOT_SUPPORTED);
        return;
    }
    if (c->state != SMP_IDLE) {
        smp_unexpected(c, d[0]);
        return;
    }
    if (!s_cfg.pairable && c->bond < 0) {
        smp_send_failed_raw(c->handle, FLOSS_SMP_REASON_PAIRING_NOT_SUPPORTED);
        return;
    }
    smp_pairing_begin(c, SMP_PHASE1);
    smp_pairing_t *p = &c->p;
    memcpy(p->preq, d, 7);

    uint8_t *rs = p->pres;
    bool bond = s_cfg.bondable && (d[3] & SMP_AUTH_BONDING_MASK) == SMP_AUTH_BONDING;
    rs[0] = SMP_PAIRING_RESPONSE;
    rs[1] = s_cfg.io_capability;
    rs[2] = 0x00; /* no OOB data */
    rs[3] = smp_local_auth_req();
    rs[4] = SMP_MAX_KEY_SIZE;
    /* Without bonding nothing would be kept, so nothing is exchanged. */
    rs[5] = bond ? (uint8_t)(d[5] & s_cfg.peer_keys & FLOSS_SMP_DIST_ID) : 0;
    rs[6] = bond ? (uint8_t)(d[6] & s_cfg.local_keys & (FLOSS_SMP_DIST_ENC | FLOSS_SMP_DIST_ID)) : 0;

    uint8_t reason = smp_negotiate(c);
    if (reason) {
        smp_fail(c, reason);
        return;
    }
    smp_send(c, rs, 7);
    smp_features_exchanged(c);
}

/** Initiator: Pairing Response received. */
static void smp_rx_pairing_response(smp_conn_t *c, const uint8_t *d) {
    if (!smp_is_initiator(c) || c->state != SMP_WAIT_PAIRING_RSP) {
        smp_unexpected(c, d[0]);
        return;
    }
    memcpy(c->p.pres, d, 7);
    uint8_t reason = smp_negotiate(c);
    if (reason) {
        smp_fail(c, reason);
        return;
    }
    smp_features_exchanged(c);
}

/** Initiator: send a Pairing Request. */
static floss_err_t smp_central_pair(smp_conn_t *c) {
    smp_pairing_begin(c, SMP_WAIT_PAIRING_RSP);
    uint8_t *rq = c->p.preq;
    rq[0] = SMP_PAIRING_REQUEST;
    rq[1] = s_cfg.io_capability;
    rq[2] = 0x00; /* no OOB data */
    rq[3] = smp_local_auth_req();
    rq[4] = SMP_MAX_KEY_SIZE;
    rq[5] = s_cfg.bondable ? (uint8_t)(s_cfg.local_keys & FLOSS_SMP_DIST_ID) : 0;
    rq[6] = s_cfg.bondable ? (uint8_t)(s_cfg.peer_keys & (FLOSS_SMP_DIST_ENC | FLOSS_SMP_DIST_ID)) : 0;
    smp_send(c, rq, 7);
    return FLOSS_OK;
}

/**
 * Central: secure the link, by encrypting with a stored bond when it is good
 * enough, otherwise by pairing.
 */
static floss_err_t smp_central_secure(smp_conn_t *c, bool peer_wants_mitm) {
    bool mitm = peer_wants_mitm || s_cfg.mitm;
    if (c->level >= (mitm ? 3 : 2)) {
        return FLOSS_OK; /* already there */
    }
    const floss_smp_bond_t *b = (c->bond >= 0) ? &s_bonds[c->bond] : NULL;
    if (b && (b->flags & (FLOSS_SMP_BOND_SC | FLOSS_SMP_BOND_PEER_LTK)) &&
        (!mitm || (b->flags & FLOSS_SMP_BOND_AUTHENTICATED))) {
        c->enc_bond = c->bond;
        smp_start_encryption(c, b->rand, b->ediv, b->ltk);
        return FLOSS_OK;
    }
    if (!s_cfg.pairable && !b) {
        return FLOSS_ERR_STATE;
    }
    return smp_central_pair(c);
}

/** Central: the peripheral asks for security. */
static void smp_rx_security_request(smp_conn_t *c, const uint8_t *d) {
    if (c->role != SMP_ROLE_CENTRAL) {
        smp_reject(c, FLOSS_SMP_REASON_CMD_NOT_SUPPORTED);
        return;
    }
    if (c->state != SMP_IDLE) {
        return; /* pairing already running */
    }
    if (smp_central_secure(c, (d[1] & SMP_AUTH_MITM) != 0) != FLOSS_OK) {
        smp_send_failed_raw(c->handle, FLOSS_SMP_REASON_PAIRING_NOT_SUPPORTED);
    }
}

/* ------------------------------------------------------------------------ */
/* Phase 2: confirm / random (legacy and SC)                                */
/* ------------------------------------------------------------------------ */

/**
 * Confirm value over our own nonce (local = true, what we send) or the peer's
 * (local = false, to check what the peer committed to).
 *  legacy: c1(TK, rand, preq, pres, iat, ia, rat, ra) - same for both sides
 *  SC:     f4(PKx_sender, PKx_receiver, N_sender, Z), Z = 0 or 0x80|passkey bit
 */
static void smp_calc_confirm(const smp_conn_t *c, bool local, uint8_t out[16]) {
    const smp_pairing_t *p = &c->p;
    const uint8_t *rand = local ? p->local_rand : p->peer_rand;
    if (!p->sc) {
        uint8_t at, bt;
        const floss_addr_t *a, *b;
        smp_addrs(c, &at, &a, &bt, &b);
        floss_smp_c1(p->tk, rand, p->preq, p->pres, at, a->b, bt, b->b, out);
    } else {
        uint8_t z = 0;
        if (smp_method_is_passkey(p->method)) {
            z = (uint8_t)(0x80 | ((p->passkey >> p->round) & 1u));
        }
        floss_smp_f4(local ? s_local_pk : p->peer_pk, local ? p->peer_pk : s_local_pk, rand, z, out);
    }
}

/**
 * Send our Pairing Confirm when possible: legacy, and every SC Passkey Entry
 * round. The initiator commits first; the responder only after the
 * initiator's confirm arrived. (SC Just Works / Numeric Comparison send a
 * single responder confirm from sc_pk_exchanged instead.)
 */
static void smp_try_send_confirm(smp_conn_t *c) {
    smp_pairing_t *p = &c->p;
    bool init = smp_is_initiator(c);
    if (c->state != SMP_WAIT_CONFIRM || !p->tk_ready || p->local_confirm_sent) {
        return;
    }
    if ((p->sc && !smp_method_is_passkey(p->method)) || (!init && !p->peer_confirm_rx)) {
        return;
    }
    uint8_t cfm[16];
    floss_hal_random(p->local_rand, 16);
    smp_calc_confirm(c, true, cfm);
    p->local_confirm_sent = true;
    if (!init) {
        c->state = SMP_WAIT_RANDOM;
    }
    smp_send_value(c, SMP_PAIRING_CONFIRM, cfm);
}

static void smp_rx_confirm(smp_conn_t *c, const uint8_t *d) {
    smp_pairing_t *p = &c->p;
    if (c->state != SMP_WAIT_CONFIRM || p->peer_confirm_rx) {
        smp_unexpected(c, d[0]);
        return;
    }
    memcpy(p->peer_confirm, &d[1], 16);
    p->peer_confirm_rx = true;
    if (!smp_is_initiator(c)) {
        smp_try_send_confirm(c); /* may still wait for the passkey */
        return;
    }
    /* Initiator: the responder committed; reveal our nonce. In SC Just Works /
     * Numeric Comparison we never sent a confirm, so the nonce is made now. */
    if (p->sc && !smp_method_is_passkey(p->method)) {
        floss_hal_random(p->local_rand, 16);
    } else if (!p->local_confirm_sent) {
        smp_fail(c, FLOSS_SMP_REASON_UNSPECIFIED);
        return;
    }
    c->state = SMP_WAIT_RANDOM;
    smp_send_value(c, SMP_PAIRING_RANDOM, p->local_rand);
}

/** Legacy: peer's random arrived; check its confirm and derive the STK. */
static void legacy_rx_random(smp_conn_t *c) {
    smp_pairing_t *p = &c->p;
    uint8_t chk[16];
    smp_calc_confirm(c, false, chk);
    if (!smp_equal16(chk, p->peer_confirm)) {
        smp_fail(c, FLOSS_SMP_REASON_CONFIRM_VALUE_FAILED);
        return;
    }
    /* STK = s1(TK, Srand, Mrand) */
    if (smp_is_initiator(c)) {
        floss_smp_s1(p->tk, p->peer_rand, p->local_rand, p->ltk);
    } else {
        floss_smp_s1(p->tk, p->local_rand, p->peer_rand, p->ltk);
    }
    smp_mask_key(p->ltk, p->key_size);
    c->state = SMP_WAIT_ENCRYPTION;
    if (smp_is_initiator(c)) {
        static const uint8_t zero_rand[8] = {0};
        smp_start_encryption(c, zero_rand, 0, p->ltk);
    } else {
        smp_send_value(c, SMP_PAIRING_RANDOM, p->local_rand);
    }
}

/** SC: nonces exchanged (after the last passkey round): user check, then DHKey Check. */
static void sc_nonces_done(smp_conn_t *c) {
    smp_pairing_t *p = &c->p;
    c->state = SMP_WAIT_DHKEY_CHECK;
    if (p->method == SMP_NUMERIC_COMPARISON) {
        bool init = smp_is_initiator(c);
        const uint8_t *pka = init ? s_local_pk : p->peer_pk;
        const uint8_t *pkb = init ? p->peer_pk : s_local_pk;
        const uint8_t *na = init ? p->local_rand : p->peer_rand;
        const uint8_t *nb = init ? p->peer_rand : p->local_rand;
        floss_smp_evt_t evt;
        smp_evt_init(&evt, FLOSS_SMP_EVT_NUMERIC_COMPARISON, c);
        evt.passkey = floss_smp_g2(pka, pkb, na, nb) % (SMP_PASSKEY_MAX + 1);
        p->user = SMP_USER_CONFIRM;
        smp_emit(&evt);
        if (!smp_pairing_active(c)) {
            return;
        }
    }
    sc_try_dhkey_check(c);
}

/** SC: peer's random arrived. */
static void sc_rx_random(smp_conn_t *c) {
    smp_pairing_t *p = &c->p;
    bool init = smp_is_initiator(c);
    bool passkey = smp_method_is_passkey(p->method);

    /* The initiator always checks the responder's commitment; with Passkey
     * Entry the responder checks the initiator's too. */
    if (init || passkey) {
        uint8_t chk[16];
        smp_calc_confirm(c, false, chk);
        if (!smp_equal16(chk, p->peer_confirm)) {
            smp_fail(c, FLOSS_SMP_REASON_CONFIRM_VALUE_FAILED);
            return;
        }
    }
    if (!init) {
        smp_send_value(c, SMP_PAIRING_RANDOM, p->local_rand);
    }
    if (passkey && ++p->round < SMP_PASSKEY_BITS) {
        p->local_confirm_sent = false;
        p->peer_confirm_rx = false;
        c->state = SMP_WAIT_CONFIRM;
        smp_try_send_confirm(c);
        return;
    }
    sc_nonces_done(c);
}

static void smp_rx_random(smp_conn_t *c, const uint8_t *d) {
    if (c->state != SMP_WAIT_RANDOM) {
        smp_unexpected(c, d[0]);
        return;
    }
    memcpy(c->p.peer_rand, &d[1], 16);
    if (c->p.sc) {
        sc_rx_random(c);
    } else {
        legacy_rx_random(c);
    }
}

/* ------------------------------------------------------------------------ */
/* Phase 2: SC public keys and DHKey check                                  */
/* ------------------------------------------------------------------------ */

/** Both public keys known: start the DHKey and the confirm/nonce exchange. */
static void sc_pk_exchanged(smp_conn_t *c) {
    smp_pairing_t *p = &c->p;
    if (smp_method_is_passkey(p->method)) {
        c->state = SMP_WAIT_CONFIRM;
        smp_try_send_confirm(c); /* initiator starts round 0 */
    } else if (smp_is_initiator(c)) {
        c->state = SMP_WAIT_CONFIRM; /* wait for Cb */
    } else {
        /* Responder commits to Nb: Cb = f4(PKbx, PKax, Nb, 0). */
        uint8_t cfm[16];
        floss_hal_random(p->local_rand, 16);
        smp_calc_confirm(c, true, cfm);
        c->state = SMP_WAIT_RANDOM;
        smp_send_value(c, SMP_PAIRING_CONFIRM, cfm);
    }
    p->dhkey_queued = true;
    smp_dhkey_kick();
}

/**
 * A peer that echoes our own public key would make the DHKey something an
 * attacker can predict; matching X also catches the negated point.
 */
static bool sc_reflected(const smp_conn_t *c) {
    return memcmp(c->p.peer_pk, s_local_pk, 32) == 0;
}

/** Send our public key once the controller has produced it. */
static void sc_send_public_key(smp_conn_t *c) {
    smp_pairing_t *p = &c->p;
    if (s_pk_state != PK_READY) {
        p->pk_tx_pending = true; /* resumed from smp_on_p256_complete */
        smp_pk_request();
        return;
    }
    p->pk_tx_pending = false;
    bool init = smp_is_initiator(c);
    if (!init && sc_reflected(c)) {
        smp_fail(c, FLOSS_SMP_REASON_DHKEY_CHECK_FAILED);
        return;
    }
    uint8_t pdu[65];
    pdu[0] = SMP_PAIRING_PUBLIC_KEY;
    memcpy(&pdu[1], s_local_pk, 64);
    smp_send(c, pdu, sizeof pdu);
    if (!init) {
        sc_pk_exchanged(c);
    }
}

static void smp_rx_public_key(smp_conn_t *c, const uint8_t *d) {
    smp_pairing_t *p = &c->p;
    bool init = smp_is_initiator(c);
    if (!p->sc || c->state != SMP_WAIT_PUBLIC_KEY || p->peer_pk_rx || (init && p->pk_tx_pending)) {
        smp_unexpected(c, d[0]);
        return;
    }
    memcpy(p->peer_pk, &d[1], 64);
    p->peer_pk_rx = true;
    if (!init) {
        sc_send_public_key(c); /* responder answers with its key */
        return;
    }
    if (sc_reflected(c)) {
        smp_fail(c, FLOSS_SMP_REASON_DHKEY_CHECK_FAILED);
        return;
    }
    sc_pk_exchanged(c);
}

/**
 * DHKey Check (Vol 3 Part H 2.3.5.6.5) once the DHKey is known and the user
 * agreed:
 *   (MacKey, LTK) = f5(DHKey, Na, Nb, A, B)
 *   Ea = f6(MacKey, Na, Nb, rb, IOcapA, A, B)
 *   Eb = f6(MacKey, Nb, Na, ra, IOcapB, B, A)
 * ra = rb = passkey for Passkey Entry, 0 otherwise. The initiator sends Ea
 * first; the responder answers with Eb only after Ea checked out.
 */
static void sc_try_dhkey_check(smp_conn_t *c) {
    smp_pairing_t *p = &c->p;
    bool init = smp_is_initiator(c);
    if (c->state != SMP_WAIT_DHKEY_CHECK || p->user != SMP_USER_NONE || !p->dhkey_ready) {
        return;
    }
    if (init ? p->local_check_sent : !p->peer_check_rx) {
        return;
    }

    uint8_t at, bt, a[7], b[7], r[16], mackey[16], ea[16], eb[16];
    const floss_addr_t *pa, *pb;
    smp_addrs(c, &at, &pa, &bt, &pb);
    floss_smp_addr7(a, at, pa);
    floss_smp_addr7(b, bt, pb);
    const uint8_t *na = init ? p->local_rand : p->peer_rand;
    const uint8_t *nb = init ? p->peer_rand : p->local_rand;

    memset(r, 0, sizeof r);
    if (smp_method_is_passkey(p->method)) {
        floss_put_le32(r, p->passkey);
    }
    floss_smp_f5(p->dhkey, na, nb, a, b, mackey, p->ltk);
    floss_smp_f6(mackey, na, nb, r, &p->preq[1], a, b, ea);
    floss_smp_f6(mackey, nb, na, r, &p->pres[1], b, a, eb);
    memset(mackey, 0, sizeof mackey);
    smp_mask_key(p->ltk, p->key_size);

    if (init) {
        memcpy(p->peer_check, eb, 16); /* what the responder must answer */
        p->local_check_sent = true;
        smp_send_value(c, SMP_PAIRING_DHKEY_CHECK, ea);
    } else {
        if (!smp_equal16(p->peer_check, ea)) {
            smp_fail(c, FLOSS_SMP_REASON_DHKEY_CHECK_FAILED);
            return;
        }
        c->state = SMP_WAIT_ENCRYPTION;
        smp_send_value(c, SMP_PAIRING_DHKEY_CHECK, eb);
    }
}

static void smp_rx_dhkey_check(smp_conn_t *c, const uint8_t *d) {
    smp_pairing_t *p = &c->p;
    if (!p->sc || c->state != SMP_WAIT_DHKEY_CHECK) {
        smp_unexpected(c, d[0]);
        return;
    }
    if (smp_is_initiator(c)) {
        if (!p->local_check_sent) {
            smp_fail(c, FLOSS_SMP_REASON_UNSPECIFIED);
            return;
        }
        if (!smp_equal16(&d[1], p->peer_check)) {
            smp_fail(c, FLOSS_SMP_REASON_DHKEY_CHECK_FAILED);
            return;
        }
        static const uint8_t zero_rand[8] = {0};
        c->state = SMP_WAIT_ENCRYPTION;
        smp_start_encryption(c, zero_rand, 0, p->ltk);
        return;
    }
    if (p->peer_check_rx) {
        smp_fail(c, FLOSS_SMP_REASON_UNSPECIFIED);
        return;
    }
    memcpy(p->peer_check, &d[1], 16);
    p->peer_check_rx = true;
    sc_try_dhkey_check(c);
}

/* ------------------------------------------------------------------------ */
/* Phase 3: key distribution                                                */
/* ------------------------------------------------------------------------ */

static void smp_send_local_keys(smp_conn_t *c) {
    smp_pairing_t *p = &c->p;
    uint8_t pdu[17];

    if (p->local_dist & FLOSS_SMP_DIST_ENC) {
        /* Legacy only (masked for SC). A fresh random LTK identified by
         * EDIV/Rand; (0, 0) is reserved for SC keys, so never use it. */
        floss_smp_bond_t *b = &p->bond;
        floss_hal_random(b->ltk, 16);
        smp_mask_key(b->ltk, p->key_size);
        do {
            uint8_t e[2];
            floss_hal_random(e, 2);
            b->ediv = floss_get_le16(e);
            floss_hal_random(b->rand, 8);
        } while (b->ediv == 0 && smp_is_zero(b->rand, 8));

        smp_send_value(c, SMP_ENCRYPTION_INFORMATION, b->ltk);
        pdu[0] = SMP_CENTRAL_IDENTIFICATION;
        floss_put_le16(&pdu[1], b->ediv);
        memcpy(&pdu[3], b->rand, 8);
        smp_send(c, pdu, 11);
    }
    if (p->local_dist & FLOSS_SMP_DIST_ID) {
        smp_send_value(c, SMP_IDENTITY_INFORMATION, s_irk);
        pdu[0] = SMP_IDENTITY_ADDRESS_INFO;
        pdu[1] = c->local_type;
        memcpy(&pdu[2], c->local_addr.b, 6);
        smp_send(c, pdu, 8);
    }
}

/** All keys exchanged: store the bond and report. */
static void smp_pairing_complete(smp_conn_t *c) {
    smp_pairing_t *p = &c->p;
    floss_smp_bond_t *b = &p->bond;
    bool have_ltk;

    if (p->sc) {
        memcpy(b->ltk, p->ltk, 16); /* EDIV = Rand = 0 */
        have_ltk = true;
    } else if (smp_is_initiator(c)) {
        /* Our usable LTK is the one the peripheral gave us. */
        have_ltk = (p->pres[6] & FLOSS_SMP_DIST_ENC) != 0;
        b->flags |= FLOSS_SMP_BOND_PEER_LTK;
    } else {
        have_ltk = (p->local_dist & FLOSS_SMP_DIST_ENC) != 0;
    }
    if (!p->got_identity) {
        b->addr_type = c->peer_type & 1u;
        b->addr = c->peer_addr;
    }
    b->key_size = p->key_size;
    if (p->authenticated) {
        b->flags |= FLOSS_SMP_BOND_AUTHENTICATED;
    }
    if (p->sc) {
        b->flags |= FLOSS_SMP_BOND_SC;
    }

    bool bonded = false;
    if (p->bonding && have_ltk) {
        int idx = smp_bond_save(b);
        c->bond = (int8_t)idx;
        bonded = true;
    }
    FLOSS_LOGI(SMP, "SMP %u: pairing complete, level %u%s", c->handle, c->level, bonded ? ", bonded" : "");

    floss_smp_evt_t evt;
    smp_pairing_end(c, SMP_IDLE);
    smp_evt_init(&evt, FLOSS_SMP_EVT_PAIRING_COMPLETE, c);
    evt.bonded = bonded;
    smp_emit(&evt);
}

/** Initiator sends its keys once the responder's have all arrived. */
static void smp_key_dist_check(smp_conn_t *c) {
    if (c->p.rx_keys != 0) {
        return;
    }
    if (smp_is_initiator(c)) {
        smp_send_local_keys(c);
    }
    smp_pairing_complete(c);
}

static void smp_rx_key(smp_conn_t *c, const uint8_t *d) {
    smp_pairing_t *p = &c->p;
    uint8_t bit;
    switch (d[0]) {
    case SMP_ENCRYPTION_INFORMATION: bit = SMP_RX_ENC_INFO; break;
    case SMP_CENTRAL_IDENTIFICATION: bit = SMP_RX_CENTRAL_ID; break;
    case SMP_IDENTITY_INFORMATION: bit = SMP_RX_ID_INFO; break;
    case SMP_IDENTITY_ADDRESS_INFO: bit = SMP_RX_ID_ADDR; break;
    default: bit = 0; break; /* Signing Information: never agreed */
    }
    if (c->state != SMP_KEY_DIST || !(p->rx_keys & bit)) {
        smp_unexpected(c, d[0]);
        return;
    }
    switch (d[0]) {
    case SMP_ENCRYPTION_INFORMATION:
        memcpy(p->bond.ltk, &d[1], 16);
        break;
    case SMP_CENTRAL_IDENTIFICATION:
        p->bond.ediv = floss_get_le16(&d[1]);
        memcpy(p->bond.rand, &d[3], 8);
        break;
    case SMP_IDENTITY_INFORMATION:
        /* An all-zero IRK means the peer does not use private addresses. */
        memcpy(p->bond.peer_irk, &d[1], 16);
        if (!smp_is_zero(p->bond.peer_irk, 16)) {
            p->bond.flags |= FLOSS_SMP_BOND_HAS_IRK;
        }
        break;
    default: /* SMP_IDENTITY_ADDRESS_INFO */
        p->bond.addr_type = d[1] & 1u;
        memcpy(p->bond.addr.b, &d[2], 6);
        p->got_identity = true;
        break;
    }
    p->rx_keys &= (uint8_t)~bit;
    smp_key_dist_check(c);
}

/* ------------------------------------------------------------------------ */
/* LE <-> SMP contract (floss_internal.h)                                   */
/* ------------------------------------------------------------------------ */

void floss_smp_init(void) {
    memset(s_conns, 0, sizeof s_conns);
    memset(s_listeners, 0, sizeof s_listeners);
    floss_smp_config_defaults(&s_cfg);
    s_local_addr_set = false;
    s_pk_state = PK_NONE;
    s_dhkey_owner = SMP_HANDLE_NONE;
    floss_timer_init(&s_ctrl_timer, smp_ctrl_timeout, NULL);
    smp_irk_load();
    smp_bonds_load();
}

void floss_smp_on_link_up(uint16_t handle, uint8_t role, uint8_t peer_addr_type, const floss_addr_t *peer_addr) {
    smp_conn_t *c = smp_conn(handle);
    if (!c) {
        for (unsigned i = 0; i < FLOSS_MAX_CONNECTIONS && !c; i++) {
            if (!s_conns[i].used) {
                c = &s_conns[i];
            }
        }
    }
    if (!c) {
        FLOSS_LOGE(SMP, "SMP: no context for handle %u", handle);
        return;
    }
    floss_timer_stop(&c->timer);
    memset(c, 0, sizeof *c);
    c->used = true;
    c->handle = handle;
    c->role = role;
    c->level = 1;
    c->peer_type = peer_addr_type & 1u;
    c->peer_addr = *peer_addr;
    if (s_local_addr_set) {
        c->local_type = s_local_type;
        c->local_addr = s_local_addr;
    } else {
        c->local_type = 0;
        c->local_addr = floss_hci_controller()->addr;
    }
    c->bond = (int8_t)smp_bond_find(c->peer_type, &c->peer_addr);
    c->enc_bond = -1;
    floss_timer_init(&c->timer, smp_timeout, c);
    if (smp_sc_supported()) {
        smp_pk_request(); /* pre-warm: key generation takes a while on some controllers */
    }
}

void floss_smp_on_link_down(uint16_t handle) {
    smp_conn_t *c = smp_conn(handle);
    if (!c) {
        return;
    }
    bool was_pairing = smp_pairing_active(c);
    floss_timer_stop(&c->timer);
    if (s_dhkey_owner == handle) {
        s_dhkey_owner = SMP_HANDLE_ORPHAN; /* the completion event still has to drain */
    }
    c->state = SMP_IDLE;
    memset(&c->p, 0, sizeof c->p);
    if (was_pairing) {
        smp_report_failed(c, FLOSS_SMP_REASON_UNSPECIFIED);
    }
    c->used = false;
}

void floss_smp_on_pdu(uint16_t handle, const uint8_t *data, uint16_t len) {
    smp_conn_t *c = smp_conn(handle);
    if (!c || len == 0 || c->state == SMP_TIMED_OUT) {
        return;
    }
    uint8_t code = data[0];
    if (code == 0 || code > SMP_CODE_MAX) {
        smp_reject(c, FLOSS_SMP_REASON_CMD_NOT_SUPPORTED);
        return;
    }
    if (len != s_pdu_len[code]) {
        if (code != SMP_PAIRING_FAILED) {
            smp_reject(c, FLOSS_SMP_REASON_INVALID_PARAMETERS);
        }
        return;
    }
    if (smp_pairing_active(c)) {
        floss_timer_start(&c->timer, SMP_TIMEOUT_US); /* any SMP traffic restarts the timer */
    }

    switch (code) {
    case SMP_PAIRING_REQUEST: smp_rx_pairing_request(c, data); break;
    case SMP_PAIRING_RESPONSE: smp_rx_pairing_response(c, data); break;
    case SMP_PAIRING_CONFIRM: smp_rx_confirm(c, data); break;
    case SMP_PAIRING_RANDOM: smp_rx_random(c, data); break;
    case SMP_PAIRING_PUBLIC_KEY: smp_rx_public_key(c, data); break;
    case SMP_PAIRING_DHKEY_CHECK: smp_rx_dhkey_check(c, data); break;
    case SMP_SECURITY_REQUEST: smp_rx_security_request(c, data); break;
    case SMP_KEYPRESS_NOTIFICATION: break; /* informational only */
    case SMP_PAIRING_FAILED:
        if (smp_pairing_active(c)) {
            FLOSS_LOGW(SMP, "SMP %u: peer aborted pairing, reason 0x%02x", handle, data[1]);
            smp_pairing_end(c, SMP_IDLE);
            smp_report_failed(c, data[1]);
        }
        break;
    default: /* key distribution PDUs 0x06..0x0A */
        smp_rx_key(c, data);
        break;
    }
}

void floss_smp_on_le_event(uint8_t subevent, const uint8_t *params, uint8_t len) {
    switch (subevent) {
    case SMP_SUBEVT_LTK_REQUEST:
        smp_on_ltk_request(params, len);
        break;
    case SMP_SUBEVT_P256_COMPLETE:
        smp_on_p256_complete(params, len);
        break;
    case SMP_SUBEVT_DHKEY_COMPLETE:
        if (len >= 33) {
            smp_dhkey_done(params[0], &params[1]);
        } else {
            smp_dhkey_done(len ? params[0] : 0xFF, NULL);
        }
        break;
    default:
        break;
    }
}

void floss_smp_on_encryption(uint16_t handle, uint8_t status, bool encrypted) {
    smp_conn_t *c = smp_conn(handle);
    if (!c) {
        return;
    }
    if (c->state == SMP_WAIT_ENCRYPTION) {
        if (status != FLOSS_HCI_SUCCESS || !encrypted) {
            if (!encrypted) {
                smp_set_level(c, 1);
            }
            smp_fail(c, FLOSS_SMP_REASON_UNSPECIFIED);
            return;
        }
        /* Encrypted with the STK / new LTK: move on to key distribution;
         * the responder distributes first. */
        smp_set_level(c, smp_level(c->p.authenticated, c->p.sc, c->p.key_size));
        c->state = SMP_KEY_DIST;
        floss_timer_start(&c->timer, SMP_TIMEOUT_US);
        if (!smp_is_initiator(c)) {
            smp_send_local_keys(c);
        }
        smp_key_dist_check(c);
        return;
    }

    int8_t idx = c->enc_bond;
    c->enc_bond = -1;
    if (status != FLOSS_HCI_SUCCESS) {
        /* e.g. 0x06 PIN or Key Missing: the peer lost its bond with us. */
        FLOSS_LOGW(SMP, "SMP %u: encryption failed (0x%02x)", handle, status);
        return;
    }
    if (!encrypted) {
        smp_set_level(c, 1);
        smp_emit_simple(FLOSS_SMP_EVT_SECURITY_CHANGED, c);
        return;
    }
    if (idx >= 0 && s_bond_valid[idx]) {
        c->bond = idx;
        smp_set_level(c, smp_bond_level(&s_bonds[idx]));
        smp_bond_touch((unsigned)idx);
        FLOSS_LOGI(SMP, "SMP %u: encrypted with bond, level %u", handle, c->level);
        smp_emit_simple(FLOSS_SMP_EVT_SECURITY_CHANGED, c);
    }
    /* else: key refresh with the same key, nothing changes */
}

/* ------------------------------------------------------------------------ */
/* Public API                                                               */
/* ------------------------------------------------------------------------ */

void floss_smp_config_defaults(floss_smp_config_t *cfg) {
    memset(cfg, 0, sizeof *cfg);
    cfg->io_capability = FLOSS_SMP_IO_NO_INPUT_NO_OUTPUT;
    cfg->pairable = true;
    cfg->bondable = true;
    cfg->mitm = false;
    cfg->secure_connections = true;
    cfg->secure_connections_only = false;
    cfg->min_key_size = 7;
    cfg->local_keys = FLOSS_SMP_DIST_ENC | FLOSS_SMP_DIST_ID;
    cfg->peer_keys = FLOSS_SMP_DIST_ENC | FLOSS_SMP_DIST_ID;
}

floss_err_t floss_smp_set_config(const floss_smp_config_t *cfg) {
    if (!cfg || cfg->io_capability > FLOSS_SMP_IO_KEYBOARD_DISPLAY || cfg->min_key_size > 16) {
        return FLOSS_ERR_INVALID;
    }
    s_cfg = *cfg;
    if (s_cfg.min_key_size < 7) {
        s_cfg.min_key_size = 7;
    }
    if (s_cfg.secure_connections_only) {
        s_cfg.secure_connections = true;
    }
    return FLOSS_OK;
}

void floss_smp_set_local_addr(uint8_t type, const floss_addr_t *addr) {
    s_local_addr_set = true;
    s_local_type = type & 1u;
    s_local_addr = *addr;
}

const uint8_t *floss_smp_local_irk(void) {
    return s_irk;
}

floss_err_t floss_smp_add_listener(floss_smp_evt_fn fn, void *user) {
    for (unsigned i = 0; i < FLOSS_SMP_MAX_LISTENERS; i++) {
        if (!s_listeners[i].fn) {
            s_listeners[i].fn = fn;
            s_listeners[i].user = user;
            return FLOSS_OK;
        }
    }
    return FLOSS_ERR_NOMEM;
}

floss_err_t floss_smp_confirm(uint16_t handle, bool accept) {
    smp_conn_t *c = smp_conn(handle);
    if (!c) {
        return FLOSS_ERR_NOT_FOUND;
    }
    if (c->p.user != SMP_USER_CONFIRM) {
        return FLOSS_ERR_STATE;
    }
    c->p.user = SMP_USER_NONE;
    if (!accept) {
        smp_fail(c, FLOSS_SMP_REASON_NUMERIC_COMPARISON_FAILED);
        return FLOSS_OK;
    }
    sc_try_dhkey_check(c);
    return FLOSS_OK;
}

floss_err_t floss_smp_passkey_reply(uint16_t handle, uint32_t passkey) {
    smp_conn_t *c = smp_conn(handle);
    if (!c) {
        return FLOSS_ERR_NOT_FOUND;
    }
    if (c->p.user != SMP_USER_PASSKEY) {
        return FLOSS_ERR_STATE;
    }
    c->p.user = SMP_USER_NONE;
    if (passkey > SMP_PASSKEY_MAX) {
        smp_fail(c, FLOSS_SMP_REASON_PASSKEY_ENTRY_FAILED);
        return FLOSS_OK;
    }
    smp_set_passkey(&c->p, passkey);
    smp_try_send_confirm(c);
    return FLOSS_OK;
}

floss_err_t floss_smp_request_security(uint16_t handle) {
    smp_conn_t *c = smp_conn(handle);
    if (!c) {
        return FLOSS_ERR_NOT_FOUND;
    }
    if (c->state != SMP_IDLE) {
        return smp_pairing_active(c) ? FLOSS_OK : FLOSS_ERR_STATE;
    }
    if (c->role == SMP_ROLE_PERIPHERAL) {
        uint8_t pdu[2] = {SMP_SECURITY_REQUEST, smp_local_auth_req()};
        smp_send(c, pdu, sizeof pdu);
        return FLOSS_OK;
    }
    return smp_central_secure(c, false);
}

uint8_t floss_smp_security_level(uint16_t handle) {
    const smp_conn_t *c = smp_conn(handle);
    return c ? c->level : 0;
}

uint8_t floss_smp_bond_count(void) {
    uint8_t n = 0;
    for (unsigned i = 0; i < FLOSS_MAX_BONDS; i++) {
        n = (uint8_t)(n + (s_bond_valid[i] ? 1 : 0));
    }
    return n;
}

const floss_smp_bond_t *floss_smp_bond_at(uint8_t idx) {
    for (unsigned i = 0; i < FLOSS_MAX_BONDS; i++) {
        if (s_bond_valid[i] && idx-- == 0) {
            return &s_bonds[i];
        }
    }
    return NULL;
}

const floss_smp_bond_t *floss_smp_resolve(uint8_t addr_type, const floss_addr_t *addr) {
    int idx = smp_bond_find(addr_type, addr);
    return idx >= 0 ? &s_bonds[idx] : NULL;
}

floss_err_t floss_smp_bond_delete(uint8_t addr_type, const floss_addr_t *addr) {
    int idx = smp_bond_find(addr_type, addr);
    if (idx < 0) {
        return FLOSS_ERR_NOT_FOUND;
    }
    smp_bond_remove((unsigned)idx);
    return FLOSS_OK;
}

floss_err_t floss_smp_bond_delete_all(void) {
    for (unsigned i = 0; i < FLOSS_MAX_BONDS; i++) {
        if (s_bond_valid[i]) {
            smp_bond_remove(i);
        }
    }
    return FLOSS_OK;
}

#else
typedef int floss_smp_unused_t; /* ISO C forbids an empty translation unit */
#endif /* FLOSS_ENABLE_LE */
