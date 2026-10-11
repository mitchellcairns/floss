/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_gatt_client.c - Minimal GATT client: Exchange MTU, find one
 * characteristic (and its CCCD) by UUID, read, write, subscribe, receive
 * notifications and indications.
 *
 * Overview
 * --------
 * ATT is strictly request/response: a client has at most one request
 * outstanding per bearer, so each link has exactly one procedure context
 * (gc_link_t, indexed like every other per-link LE table by
 * floss_le_link_index). A second procedure while one runs gets
 * FLOSS_ERR_BUSY. Commands (Write Command) and Handle Value Confirmations
 * are not requests and may go out at any time.
 *
 * Every request arms the 30 s ATT transaction timer. If it fires the
 * procedure completes with 0xFF and, as the Core Specification requires (no
 * further ATT traffic on a bearer whose transaction timed out), the link is
 * disconnected - the same rule floss_att.c applies to unconfirmed
 * indications. Link loss also completes the procedure with 0xFF.
 *
 * Find characteristic (one procedure, several transactions):
 *
 *   Read By Type 0x2803 [start..0xFFFF]   repeated from last handle + 1 until
 *        |                                the UUID matches or the server says
 *        |                                Attribute Not Found (-> 0x0A)
 *        v  match with NOTIFY or INDICATE in the properties
 *   Find Information [value+1 .. next declaration - 1 (or 0xFFFF)]
 *                                         repeated until a 0x2902 descriptor
 *                                         (the CCCD), a declaration (end of
 *                                         the characteristic) or Not Found
 *
 * UUIDs are compared in their 128-bit little-endian form; 16-bit UUIDs are
 * expanded with the base UUID, so a 16-bit search also matches a server that
 * spells the same UUID in 128 bits (and vice versa).
 *
 * Responses are bounds-checked; a response that is malformed (wrong length,
 * handles going backwards or out of the requested range) ends the procedure
 * with FLOSS_ATT_ERR_INVALID_PDU instead of waiting for the timeout.
 * An Error Response for a request we did not send is ignored.
 *
 * Follow-up requests inside a procedure are tiny and kept in gc_link_t.req,
 * so a momentary buffer shortage is retried from a short timer instead of
 * failing the procedure; so is the confirmation of an indication.
 */
#include "floss_gatt_client.h"
#include "floss/floss_le.h"
#include "floss/floss_gatt.h"
#include "floss/floss_le_central.h"

#include <stddef.h>

#if FLOSS_ENABLE_LE

/** att_err reported for timeouts and link loss (from the public contract). */
#define GC_ERR_LOCAL 0xFF
/** Retry period after a buffer shortage. */
#define GC_RETRY_US 2000u
/** Largest request built inside a procedure: Read By Type with a 16-bit type is 7, Find Information 5. */
#define GC_REQ_MAX 8

enum {
    GC_IDLE = 0,
    GC_MTU,        /* Exchange MTU */
    GC_FIND_DECL,  /* Read By Type over characteristic declarations */
    GC_FIND_CCCD,  /* Find Information after the value handle */
    GC_WRITE,      /* Write Request (also subscribe) */
    GC_READ,       /* Read Request */
};

typedef struct {
    uint16_t conn;          /* link this context belongs to (0xFFFF = none) */
    uint8_t proc;           /* GC_* */
    uint8_t req_op;         /* opcode of the outstanding request */
    bool closing;           /* link going down: no new procedures */
    bool dead;              /* a transaction timed out: the bearer is unusable */
    bool mtu_done;          /* Exchange MTU is sent once per connection */
    bool req_retry;         /* req[] still has to go out */
    bool confirm_pending;   /* indication received, confirmation not sent yet */
    uint8_t req_len;
    uint8_t req[GC_REQ_MAX];

    floss_gattc_done_fn done_fn;
    floss_gattc_char_fn char_fn;
    floss_gattc_read_fn read_fn;
    void *user;

    /* characteristic discovery */
    uint8_t uuid[16];       /* target, 128-bit little-endian */
    uint16_t next;          /* first handle of the next request */
    uint16_t end;           /* last handle of the CCCD search */
    uint8_t props;
    uint16_t value_handle;

    /* timers stay last: gc_reset() clears everything before them */
    floss_timer_t timer;    /* ATT transaction timeout */
    floss_timer_t retry;    /* resend req[] / confirmation */
} gc_link_t;

static gc_link_t s_gc[FLOSS_MAX_CONNECTIONS];
static floss_gattc_notify_fn s_notify_fn;
static void *s_notify_user;

/* Base UUID 00000000-0000-1000-8000-00805F9B34FB, little-endian. */
static const uint8_t k_base_uuid[16] = {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
                                        0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/* ------------------------------------------------------------------------ */
/* Helpers                                                                  */
/* ------------------------------------------------------------------------ */

static void gc_reset(gc_link_t *L) {
    floss_timer_stop(&L->timer);
    floss_timer_stop(&L->retry);
    memset(L, 0, offsetof(gc_link_t, timer));
    L->conn = 0xFFFF;
}

/** Context of an open LE link (claimed for it if the slot held an older link), NULL if none. */
static gc_link_t *gc_get(uint16_t conn) {
    int idx = floss_le_link_index(conn);
    if (idx < 0) {
        return NULL;
    }
    gc_link_t *L = &s_gc[idx];
    if (L->conn != conn) {
        gc_reset(L);
        L->conn = conn;
    }
    return L;
}

/** Negotiated ATT MTU of the link, as the client must respect it. */
static uint16_t gc_mtu(uint16_t conn) {
    const floss_le_link_t *l = floss_le_link(conn);
    uint16_t mtu = l ? l->att_mtu : FLOSS_ATT_DEFAULT_MTU;
    return FLOSS_MIN(mtu, (uint16_t)FLOSS_ATT_MTU);
}

/** Send [op][handle][data] on the ATT channel, written straight into a pool buffer. */
static floss_err_t gc_send_pdu(uint16_t conn, uint8_t op, uint16_t handle, const uint8_t *data, uint16_t len) {
    floss_buf_t *b = floss_buf_alloc();
    if (b == NULL) {
        FLOSS_STAT_INC(buf_alloc_failures);
        return FLOSS_ERR_BUSY;
    }
    uint8_t *p = floss_buf_put(b, (uint16_t)(3 + len));
    if (p == NULL) {
        floss_buf_free(b);
        return FLOSS_ERR_INVALID;
    }
    p[0] = op;
    floss_put_le16(&p[1], handle);
    if (len) {
        memcpy(&p[3], data, len);
    }
    uint8_t *hdr = floss_buf_push(b, FLOSS_L2CAP_HDR_LEN);
    floss_put_le16(&hdr[0], (uint16_t)(3 + len));
    floss_put_le16(&hdr[2], FLOSS_CID_ATT);
    b->handle = conn;
    b->type = FLOSS_PKT_ACL;
    b->flags = 0;
    return floss_hci_acl_send(b);
}

static floss_err_t gc_send_raw(uint16_t conn, const uint8_t *pdu, uint16_t len) {
    return floss_le_send_fixed(conn, FLOSS_CID_ATT, pdu, len);
}

/** Context for a new procedure, or NULL with *err set. */
static gc_link_t *gc_begin(uint16_t conn, floss_err_t *err) {
    gc_link_t *L = gc_get(conn);
    if (L == NULL) {
        *err = FLOSS_ERR_NOT_FOUND;
        return NULL;
    }
    if (L->closing || L->dead) {
        *err = FLOSS_ERR_STATE;
        return NULL;
    }
    if (L->proc != GC_IDLE) {
        *err = FLOSS_ERR_BUSY;
        return NULL;
    }
    return L;
}

/** The first request of a procedure: on failure nothing changed and the caller gets the error. */
static floss_err_t gc_start(gc_link_t *L, uint8_t proc, uint8_t op, uint16_t handle, const uint8_t *data,
                            uint16_t len) {
    floss_err_t err = gc_send_pdu(L->conn, op, handle, data, len);
    if (err != FLOSS_OK) {
        return err;
    }
    L->proc = proc;
    L->req_op = op;
    floss_timer_start(&L->timer, FLOSS_ATT_TIMEOUT_US);
    return FLOSS_OK;
}

/** A further request inside the running procedure (a new ATT transaction). */
static void gc_next_request(gc_link_t *L, const uint8_t *pdu, uint8_t len) {
    memcpy(L->req, pdu, len);
    L->req_len = len;
    L->req_op = pdu[0];
    floss_timer_start(&L->timer, FLOSS_ATT_TIMEOUT_US);
    if (gc_send_raw(L->conn, L->req, len) == FLOSS_OK) {
        L->req_retry = false;
    } else {
        L->req_retry = true;
        floss_timer_start(&L->retry, GC_RETRY_US);
    }
}

/** Leave the procedure state BEFORE calling back: callbacks start the next procedure. */
static void gc_idle(gc_link_t *L) {
    L->proc = GC_IDLE;
    L->req_retry = false;
    floss_timer_stop(&L->timer);
    if (!L->confirm_pending) {
        floss_timer_stop(&L->retry);
    }
}

static void gc_finish_done(gc_link_t *L, uint8_t err) {
    floss_gattc_done_fn fn = L->done_fn;
    void *user = L->user;
    gc_idle(L);
    if (fn) {
        fn(L->conn, err, user);
    }
}

static void gc_finish_char(gc_link_t *L, uint8_t err, uint16_t cccd) {
    floss_gattc_char_fn fn = L->char_fn;
    void *user = L->user;
    uint8_t props = L->props;
    uint16_t vh = L->value_handle;
    gc_idle(L);
    fn(L->conn, err, props, vh, cccd, user);
}

static void gc_finish_read(gc_link_t *L, uint8_t err, const uint8_t *data, uint16_t len) {
    floss_gattc_read_fn fn = L->read_fn;
    void *user = L->user;
    gc_idle(L);
    fn(L->conn, err, data, len, user);
}

/** End whatever runs with an error code. */
static void gc_fail(gc_link_t *L, uint8_t err) {
    switch (L->proc) {
    case GC_FIND_DECL:
    case GC_FIND_CCCD:
        gc_finish_char(L, err, 0);
        break;
    case GC_READ:
        gc_finish_read(L, err, NULL, 0);
        break;
    case GC_MTU:
    case GC_WRITE:
        gc_finish_done(L, err);
        break;
    default:
        break;
    }
}

static void gc_timeout(floss_timer_t *t, void *arg) {
    FLOSS_UNUSED(t);
    gc_link_t *L = (gc_link_t *)arg;
    if (L->proc == GC_IDLE) {
        return;
    }
    FLOSS_LOGW(LE, "gattc: request 0x%02x not answered in 30 s, closing link", (unsigned)L->req_op);
    uint16_t conn = L->conn;
    L->dead = true;
    gc_fail(L, GC_ERR_LOCAL);
    floss_le_disconnect(conn, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED);
}

static void gc_retry_fired(floss_timer_t *t, void *arg) {
    FLOSS_UNUSED(t);
    gc_link_t *L = (gc_link_t *)arg;
    if (L->confirm_pending) {
        const uint8_t c = FLOSS_ATT_CONFIRM;
        if (gc_send_raw(L->conn, &c, 1) == FLOSS_OK) {
            L->confirm_pending = false;
        }
    }
    if (L->req_retry && L->proc != GC_IDLE) {
        if (gc_send_raw(L->conn, L->req, L->req_len) == FLOSS_OK) {
            L->req_retry = false;
        }
    }
    if (L->confirm_pending || (L->req_retry && L->proc != GC_IDLE)) {
        floss_timer_start(&L->retry, GC_RETRY_US);
    }
}

/** 2- or 16-byte UUID from the wire equals the target? */
static bool gc_uuid_match(const gc_link_t *L, const uint8_t *u, uint16_t ulen) {
    uint8_t full[16];
    if (ulen == 2) {
        memcpy(full, k_base_uuid, 16);
        full[12] = u[0];
        full[13] = u[1];
        u = full;
    } else if (ulen != 16) {
        return false;
    }
    return memcmp(u, L->uuid, 16) == 0;
}

/** 16-bit value of a 128-bit UUID built on the base UUID, 0 for any other UUID. */
static uint16_t gc_uuid16_of(const uint8_t *u128) {
    if (memcmp(u128, k_base_uuid, 12) != 0 || u128[14] != 0 || u128[15] != 0) {
        return 0;
    }
    return floss_get_le16(&u128[12]);
}

/* ------------------------------------------------------------------------ */
/* Characteristic discovery                                                 */
/* ------------------------------------------------------------------------ */

static void gc_send_read_by_type(gc_link_t *L) {
    uint8_t r[7];
    r[0] = FLOSS_ATT_READ_BY_TYPE_REQ;
    floss_put_le16(&r[1], L->next);
    floss_put_le16(&r[3], 0xFFFF);
    floss_put_le16(&r[5], FLOSS_GATT_UUID_CHARACTERISTIC);
    gc_next_request(L, r, sizeof(r));
}

static void gc_send_find_info(gc_link_t *L) {
    uint8_t r[5];
    r[0] = FLOSS_ATT_FIND_INFO_REQ;
    floss_put_le16(&r[1], L->next);
    floss_put_le16(&r[3], L->end);
    gc_next_request(L, r, sizeof(r));
}

/** Characteristic found: look for its CCCD if it can notify or indicate. */
static void gc_found(gc_link_t *L) {
    if (!(L->props & (FLOSS_GATT_PROP_NOTIFY | FLOSS_GATT_PROP_INDICATE)) || L->value_handle == 0xFFFF ||
        (uint16_t)(L->value_handle + 1) > L->end) {
        gc_finish_char(L, 0, 0);
        return;
    }
    L->proc = GC_FIND_CCCD;
    L->next = (uint16_t)(L->value_handle + 1);
    gc_send_find_info(L);
}

/* Read By Type Response: [0x09][pair length][handle, properties, value handle, UUID]... */
static void gc_on_read_by_type(gc_link_t *L, const uint8_t *p, uint16_t len) {
    uint8_t plen = len >= 2 ? p[1] : 0;
    if ((plen != 7 && plen != 21) || len == 2 || (uint16_t)(len - 2) % plen != 0) {
        gc_finish_char(L, FLOSS_ATT_ERR_INVALID_PDU, 0);
        return;
    }
    bool found = false;
    uint16_t last = 0;
    for (uint16_t off = 2; off < len; off = (uint16_t)(off + plen)) {
        uint16_t h = floss_get_le16(&p[off]);
        if (h < L->next || (off > 2 && h <= last)) {
            gc_finish_char(L, FLOSS_ATT_ERR_INVALID_PDU, 0); /* handles must move forward */
            return;
        }
        if (found) {
            /* The next declaration bounds the found characteristic's descriptors. */
            L->end = (uint16_t)(h - 1);
            gc_found(L);
            return;
        }
        uint16_t vh = floss_get_le16(&p[off + 3]);
        if (gc_uuid_match(L, &p[off + 5], (uint16_t)(plen - 5))) {
            if (vh <= h) {
                gc_finish_char(L, FLOSS_ATT_ERR_INVALID_PDU, 0);
                return;
            }
            found = true;
            L->props = p[off + 2];
            L->value_handle = vh;
        }
        last = h;
    }
    if (found) {
        L->end = 0xFFFF; /* next declaration unknown: Find Information stops at it */
        gc_found(L);
    } else if (last == 0xFFFF) {
        gc_finish_char(L, FLOSS_ATT_ERR_ATTR_NOT_FOUND, 0);
    } else {
        L->next = (uint16_t)(last + 1);
        gc_send_read_by_type(L);
    }
}

/* Find Information Response: [0x05][format 1: handle + UUID16 | 2: handle + UUID128]... */
static void gc_on_find_info(gc_link_t *L, const uint8_t *p, uint16_t len) {
    uint8_t fmt = len >= 2 ? p[1] : 0;
    uint16_t entry = fmt == 1 ? 4 : (fmt == 2 ? 18 : 0);
    if (entry == 0 || len == 2 || (uint16_t)(len - 2) % entry != 0) {
        gc_finish_char(L, FLOSS_ATT_ERR_INVALID_PDU, 0);
        return;
    }
    uint16_t last = 0;
    for (uint16_t off = 2; off < len; off = (uint16_t)(off + entry)) {
        uint16_t h = floss_get_le16(&p[off]);
        if (h < L->next || h > L->end || (off > 2 && h <= last)) {
            gc_finish_char(L, FLOSS_ATT_ERR_INVALID_PDU, 0);
            return;
        }
        uint16_t t = fmt == 1 ? floss_get_le16(&p[off + 2]) : gc_uuid16_of(&p[off + 2]);
        if (t == FLOSS_GATT_UUID_CCCD) {
            gc_finish_char(L, 0, h);
            return;
        }
        if (t == FLOSS_GATT_UUID_CHARACTERISTIC || t == FLOSS_GATT_UUID_PRIMARY_SERVICE ||
            t == FLOSS_GATT_UUID_SECONDARY_SERVICE || t == FLOSS_GATT_UUID_INCLUDE) {
            gc_finish_char(L, 0, 0); /* next characteristic/service: this one has no CCCD */
            return;
        }
        last = h;
    }
    if (last >= L->end) {
        gc_finish_char(L, 0, 0);
    } else {
        L->next = (uint16_t)(last + 1);
        gc_send_find_info(L);
    }
}

static floss_err_t gc_find(uint16_t conn, const uint8_t uuid_le[16], floss_gattc_char_fn fn, void *user) {
    floss_err_t err;
    if (fn == NULL) {
        return FLOSS_ERR_INVALID;
    }
    gc_link_t *L = gc_begin(conn, &err);
    if (L == NULL) {
        return err;
    }
    uint8_t t[4];
    floss_put_le16(&t[0], 0xFFFF);
    floss_put_le16(&t[2], FLOSS_GATT_UUID_CHARACTERISTIC);
    err = gc_start(L, GC_FIND_DECL, FLOSS_ATT_READ_BY_TYPE_REQ, 0x0001, t, sizeof(t));
    if (err != FLOSS_OK) {
        return err;
    }
    memcpy(L->uuid, uuid_le, 16);
    L->char_fn = fn;
    L->user = user;
    L->next = 0x0001;
    L->end = 0xFFFF;
    L->props = 0;
    L->value_handle = 0;
    return FLOSS_OK;
}

/* ------------------------------------------------------------------------ */
/* Public API                                                               */
/* ------------------------------------------------------------------------ */

floss_err_t floss_gattc_exchange_mtu(uint16_t conn, floss_gattc_done_fn fn, void *user) {
    floss_err_t err;
    gc_link_t *L = gc_begin(conn, &err);
    if (L == NULL) {
        return err;
    }
    if (L->mtu_done) {
        return FLOSS_ERR_STATE; /* the client may send it once per connection */
    }
    /* [0x02][client rx MTU]: gc_send_pdu's "handle" slot carries the MTU. */
    err = gc_start(L, GC_MTU, FLOSS_ATT_MTU_REQ, (uint16_t)FLOSS_ATT_MTU, NULL, 0);
    if (err != FLOSS_OK) {
        return err;
    }
    L->mtu_done = true;
    L->done_fn = fn;
    L->user = user;
    return FLOSS_OK;
}

floss_err_t floss_gattc_find_char128(uint16_t conn, const uint8_t uuid_be[16], floss_gattc_char_fn fn, void *user) {
    if (uuid_be == NULL) {
        return FLOSS_ERR_INVALID;
    }
    uint8_t le[16];
    for (uint8_t i = 0; i < 16; i++) {
        le[i] = uuid_be[15 - i];
    }
    return gc_find(conn, le, fn, user);
}

floss_err_t floss_gattc_find_char16(uint16_t conn, uint16_t uuid16, floss_gattc_char_fn fn, void *user) {
    uint8_t le[16];
    memcpy(le, k_base_uuid, 16);
    floss_put_le16(&le[12], uuid16);
    return gc_find(conn, le, fn, user);
}

floss_err_t floss_gattc_write(uint16_t conn, uint16_t handle, const uint8_t *data, uint16_t len, bool with_response,
                              floss_gattc_done_fn fn, void *user) {
    if (handle == 0 || (len && data == NULL)) {
        return FLOSS_ERR_INVALID;
    }
    gc_link_t *L = gc_get(conn);
    if (L == NULL) {
        return FLOSS_ERR_NOT_FOUND;
    }
    if (L->closing || L->dead) {
        return FLOSS_ERR_STATE;
    }
    if (len > gc_mtu(conn) - 3) {
        return FLOSS_ERR_INVALID; /* one PDU only: no long writes */
    }
    if (!with_response) {
        /* A command is not a transaction: allowed while a request is outstanding. */
        floss_err_t err = gc_send_pdu(conn, FLOSS_ATT_WRITE_CMD, handle, data, len);
        if (err == FLOSS_OK && fn) {
            fn(conn, 0, user); /* handed to HCI; there is no acknowledgement to wait for */
        }
        return err;
    }
    if (L->proc != GC_IDLE) {
        return FLOSS_ERR_BUSY;
    }
    floss_err_t err = gc_start(L, GC_WRITE, FLOSS_ATT_WRITE_REQ, handle, data, len);
    if (err == FLOSS_OK) {
        L->done_fn = fn;
        L->user = user;
    }
    return err;
}

floss_err_t floss_gattc_read(uint16_t conn, uint16_t handle, floss_gattc_read_fn fn, void *user) {
    floss_err_t err;
    if (fn == NULL || handle == 0) {
        return FLOSS_ERR_INVALID;
    }
    gc_link_t *L = gc_begin(conn, &err);
    if (L == NULL) {
        return err;
    }
    err = gc_start(L, GC_READ, FLOSS_ATT_READ_REQ, handle, NULL, 0);
    if (err == FLOSS_OK) {
        L->read_fn = fn;
        L->user = user;
    }
    return err;
}

floss_err_t floss_gattc_subscribe(uint16_t conn, uint16_t cccd_handle, uint16_t value, floss_gattc_done_fn fn,
                                  void *user) {
    uint8_t v[2];
    floss_put_le16(v, value);
    return floss_gattc_write(conn, cccd_handle, v, sizeof(v), true, fn, user);
}

void floss_gattc_set_notify_handler(floss_gattc_notify_fn fn, void *user) {
    s_notify_fn = fn;
    s_notify_user = user;
}

bool floss_gattc_busy(uint16_t conn) {
    int idx = floss_le_link_index(conn);
    return idx >= 0 && s_gc[idx].conn == conn && s_gc[idx].proc != GC_IDLE;
}

/* ------------------------------------------------------------------------ */
/* Hooks                                                                    */
/* ------------------------------------------------------------------------ */

void floss_gattc_init(void) {
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        gc_link_t *L = &s_gc[i];
        memset(L, 0, sizeof(*L));
        L->conn = 0xFFFF;
        floss_timer_init(&L->timer, gc_timeout, L);
        floss_timer_init(&L->retry, gc_retry_fired, L);
    }
    s_notify_fn = NULL;
    s_notify_user = NULL;
}

/** PDUs a server sends to a client. */
static bool gc_is_client_pdu(uint8_t op) {
    switch (op) {
    case FLOSS_ATT_ERROR_RSP:
    case FLOSS_ATT_MTU_RSP:
    case FLOSS_ATT_FIND_INFO_RSP:
    case FLOSS_ATT_FIND_BY_TYPE_RSP:
    case FLOSS_ATT_READ_BY_TYPE_RSP:
    case FLOSS_ATT_READ_RSP:
    case FLOSS_ATT_READ_BLOB_RSP:
    case FLOSS_ATT_READ_MULTI_RSP:
    case FLOSS_ATT_READ_BY_GROUP_RSP:
    case FLOSS_ATT_WRITE_RSP:
    case FLOSS_ATT_PREPARE_WRITE_RSP:
    case FLOSS_ATT_EXECUTE_WRITE_RSP:
    case FLOSS_ATT_NOTIFY:
    case FLOSS_ATT_INDICATE:
        return true;
    default:
        return false;
    }
}

static void gc_on_value(gc_link_t *L, const uint8_t *p, uint16_t len, bool indication) {
    if (len < 3) {
        FLOSS_STAT_INC(rx_dropped);
        return; /* no handle: nothing to deliver, nothing to confirm */
    }
    if (s_notify_fn) {
        s_notify_fn(L->conn, floss_get_le16(&p[1]), &p[3], (uint16_t)(len - 3), indication, s_notify_user);
    }
    if (indication && !L->closing) {
        const uint8_t c = FLOSS_ATT_CONFIRM;
        if (gc_send_raw(L->conn, &c, 1) != FLOSS_OK) {
            L->confirm_pending = true;
            floss_timer_start(&L->retry, GC_RETRY_US);
        }
    }
}

static void gc_on_error(gc_link_t *L, const uint8_t *p, uint16_t len) {
    if (len != 5 || p[1] != L->req_op) {
        FLOSS_LOGD(LE, "gattc: stray error response (req 0x%02x)", len >= 2 ? (unsigned)p[1] : 0u);
        return;
    }
    uint8_t err = p[4] ? p[4] : FLOSS_ATT_ERR_UNLIKELY; /* 0 would read as success */
    FLOSS_LOGD(LE, "gattc: req 0x%02x handle 0x%04x error 0x%02x", (unsigned)p[1], (unsigned)floss_get_le16(&p[2]),
               (unsigned)err);
    if (L->proc == GC_FIND_CCCD && err == FLOSS_ATT_ERR_ATTR_NOT_FOUND) {
        gc_finish_char(L, 0, 0); /* characteristic found, it just has no CCCD */
        return;
    }
    gc_fail(L, err);
}

bool floss_gattc_on_pdu(uint16_t conn, const uint8_t *pdu, uint16_t len) {
    if (len == 0 || !gc_is_client_pdu(pdu[0])) {
        return false;
    }
    uint8_t op = pdu[0];
    gc_link_t *L = gc_get(conn);
    if (L == NULL) {
        return true;
    }
    if (op == FLOSS_ATT_NOTIFY || op == FLOSS_ATT_INDICATE) {
        gc_on_value(L, pdu, len, op == FLOSS_ATT_INDICATE);
        return true;
    }
    if (L->proc == GC_IDLE || L->closing) {
        FLOSS_LOGD(LE, "gattc: unexpected response 0x%02x dropped", (unsigned)op);
        return true;
    }
    if (op == FLOSS_ATT_ERROR_RSP) {
        gc_on_error(L, pdu, len);
        return true;
    }
    if (op != (uint8_t)(L->req_op + 1)) {
        FLOSS_LOGD(LE, "gattc: response 0x%02x does not answer 0x%02x, dropped", (unsigned)op, (unsigned)L->req_op);
        return true;
    }
    switch (L->proc) {
    case GC_MTU:
        if (len != 3) {
            gc_finish_done(L, FLOSS_ATT_ERR_INVALID_PDU);
        } else {
            uint16_t mtu = FLOSS_MIN(floss_get_le16(&pdu[1]), (uint16_t)FLOSS_ATT_MTU);
            floss_le_set_att_mtu(conn, FLOSS_MAX(mtu, (uint16_t)FLOSS_ATT_DEFAULT_MTU));
            gc_finish_done(L, 0);
        }
        break;
    case GC_FIND_DECL:
        gc_on_read_by_type(L, pdu, len);
        break;
    case GC_FIND_CCCD:
        gc_on_find_info(L, pdu, len);
        break;
    case GC_WRITE:
        gc_finish_done(L, len == 1 ? 0 : FLOSS_ATT_ERR_INVALID_PDU);
        break;
    case GC_READ:
        gc_finish_read(L, 0, &pdu[1], (uint16_t)(len - 1));
        break;
    default:
        break;
    }
    return true;
}

void floss_gattc_on_link_down(uint16_t conn) {
    int idx = floss_le_link_index(conn);
    if (idx < 0 || s_gc[idx].conn != conn) {
        return;
    }
    gc_link_t *L = &s_gc[idx];
    L->closing = true;
    floss_timer_stop(&L->timer);
    floss_timer_stop(&L->retry);
    if (L->proc != GC_IDLE) {
        gc_fail(L, GC_ERR_LOCAL);
    }
    gc_reset(L);
}

#endif /* FLOSS_ENABLE_LE */
