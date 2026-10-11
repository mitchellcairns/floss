/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_att.c - ATT server: request handling, notifications, indications.
 *
 * Overview
 * --------
 * ATT is a strict request/response protocol: a client has at most one
 * request outstanding, so each link needs exactly one response buffer
 * (att_link_t.rsp). A request is parsed straight from the received PDU and
 * the response is built in place, then handed to floss_le_send_fixed(). If
 * the buffer pool is momentarily empty the response stays in rsp[] and is
 * retried from floss_att_poll() - dropping it would stall the client for the
 * 30 s ATT timeout.
 *
 * Server-initiated PDUs take the zero-copy path instead: notifications are
 * written directly into a pool buffer (floss_gatt_notify_alloc), which is
 * the HOGP hot path.
 *
 * Security: every access is checked against the attribute permissions and
 * the link's security level (set by SMP through floss_le_on_security_changed).
 * A failure answers Insufficient Encryption/Authentication, which makes the
 * client pair or encrypt and retry. As a peripheral we additionally ask SMP
 * to send a Security Request, so hosts that do not react to the error code
 * still start pairing.
 *
 * Long writes: one shared Prepare Write queue holds up to
 * FLOSS_ATT_PREPARE_MAX bytes of ONE attribute for ONE link at a time; a
 * second attribute or link gets Prepare Queue Full. HID never needs more.
 */
#include "floss_internal.h"
#include "floss/floss_le.h"
#include "floss/floss_gatt.h"

#if FLOSS_ENABLE_LE

#include "floss/floss_smp.h"

/* Responses are built in rsp[]; a pool buffer must also fit them + L2CAP. */
_Static_assert(FLOSS_ATT_MTU >= FLOSS_ATT_DEFAULT_MTU, "FLOSS_ATT_MTU below the ATT minimum");
_Static_assert(FLOSS_ATT_MTU + FLOSS_L2CAP_HDR_LEN <= FLOSS_BUF_SIZE, "FLOSS_BUF_SIZE too small for FLOSS_ATT_MTU");

typedef struct {
    bool used;
    uint16_t conn;
    uint16_t rsp_pending;      /* > 0: rsp[] holds a response that still has to go out */
    bool sec_requested;        /* Security Request already sent on this link */
    bool ind_pending;          /* indication sent, waiting for confirmation */
    uint16_t ind_handle;
    floss_timer_t ind_timer;
    uint8_t rsp[FLOSS_ATT_MTU];
} att_link_t;

static att_link_t s_att[FLOSS_MAX_CONNECTIONS];

/** Shared Prepare Write queue (see overview). */
static struct {
    bool active;
    uint16_t conn;
    uint16_t handle;
    uint16_t start;  /* lowest queued offset */
    uint16_t end;    /* highest queued offset + length */
    uint8_t data[FLOSS_ATT_PREPARE_MAX];
} s_prep;

/* ------------------------------------------------------------------------ */
/* Helpers                                                                  */
/* ------------------------------------------------------------------------ */

static att_link_t *att_link(uint16_t conn) {
    int idx = floss_le_link_index(conn);
    if (idx < 0 || !s_att[idx].used) {
        return NULL;
    }
    return &s_att[idx];
}

static uint16_t att_mtu(uint16_t conn) {
    const floss_le_link_t *l = floss_le_link(conn);
    uint16_t mtu = l ? l->att_mtu : FLOSS_ATT_DEFAULT_MTU;
    return FLOSS_MIN(mtu, (uint16_t)FLOSS_ATT_MTU);
}

static void att_send_rsp(att_link_t *L, uint16_t len) {
    if (floss_le_send_fixed(L->conn, FLOSS_CID_ATT, L->rsp, len) == FLOSS_OK) {
        L->rsp_pending = 0;
    } else {
        L->rsp_pending = len; /* pool empty: floss_att_poll() retries */
    }
}

static void att_request_security(att_link_t *L) {
    const floss_le_link_t *l = floss_le_link(L->conn);
    if (l && l->role == 1 && !L->sec_requested) {
        L->sec_requested = true;
        FLOSS_LOGI(LE, "att: client hit a secured attribute, requesting security");
        (void)floss_smp_request_security(L->conn);
    }
}

static void att_error(att_link_t *L, uint8_t req, uint16_t handle, uint8_t err) {
    L->rsp[0] = FLOSS_ATT_ERROR_RSP;
    L->rsp[1] = req;
    floss_put_le16(&L->rsp[2], handle);
    L->rsp[4] = err;
    FLOSS_LOGD(LE, "att: error req 0x%02x handle 0x%04x err 0x%02x", (unsigned)req, (unsigned)handle,
               (unsigned)err);
    att_send_rsp(L, 5);
    if (err == FLOSS_ATT_ERR_INSUFFICIENT_ENC || err == FLOSS_ATT_ERR_INSUFFICIENT_AUTHEN) {
        att_request_security(L);
    }
}

/**
 * Permission check. Encryption-required attributes need level >= 2,
 * authentication-required ones level >= 3 (MITM-protected keys).
 */
static uint8_t att_check(uint16_t conn, const floss_gatt_attr_t *a, bool write) {
    uint16_t p = a->perms;
    if (!(p & (write ? FLOSS_GATT_P_WRITE : FLOSS_GATT_P_READ))) {
        return write ? FLOSS_ATT_ERR_WRITE_NOT_PERMITTED : FLOSS_ATT_ERR_READ_NOT_PERMITTED;
    }
    const floss_le_link_t *l = floss_le_link(conn);
    uint8_t level = l ? l->security_level : 1;
    bool need_authen = (p & (write ? FLOSS_GATT_P_WRITE_AUTHEN : FLOSS_GATT_P_READ_AUTHEN)) != 0;
    bool need_enc = (p & (write ? FLOSS_GATT_P_WRITE_ENC : FLOSS_GATT_P_READ_ENC)) != 0;
    if (need_authen && level < 3) {
        return FLOSS_ATT_ERR_INSUFFICIENT_AUTHEN;
    }
    if (need_enc && level < 2) {
        return FLOSS_ATT_ERR_INSUFFICIENT_ENC;
    }
    return 0;
}

/** A UUID from a request: 16-bit when it is (or reduces to) a 16-bit UUID. */
typedef struct {
    bool is16;
    uint16_t u16;
    uint8_t u128[16];
} att_uuid_t;

/* Base UUID 00000000-0000-1000-8000-00805F9B34FB, little-endian. */
static const uint8_t k_base_uuid[16] = {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
                                        0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static bool att_parse_uuid(const uint8_t *p, uint16_t len, att_uuid_t *u) {
    if (len == 2) {
        u->is16 = true;
        u->u16 = floss_get_le16(p);
        return true;
    }
    if (len == 16) {
        memcpy(u->u128, p, 16);
        /* A 128-bit form of a 16-bit UUID must match 16-bit attributes too. */
        u->is16 = memcmp(p, k_base_uuid, 12) == 0 && p[14] == 0 && p[15] == 0;
        u->u16 = floss_get_le16(&p[12]);
        return true;
    }
    return false;
}

static bool att_type_match(const floss_gatt_attr_t *a, const att_uuid_t *u) {
    if (a->uuid128 == NULL) {
        return u->is16 && a->type == u->u16;
    }
    return !u->is16 && memcmp(a->uuid128, u->u128, 16) == 0;
}

/** Last handle of the service whose declaration is at `h`. */
static uint16_t att_service_end(uint16_t h) {
    uint16_t last = floss_gatt_last_handle();
    for (uint32_t i = (uint32_t)h + 1; i <= last; i++) {
        uint16_t t = floss_gatt_type16(floss_gatt_attr((uint16_t)i));
        if (t == FLOSS_GATT_UUID_PRIMARY_SERVICE || t == FLOSS_GATT_UUID_SECONDARY_SERVICE) {
            return (uint16_t)(i - 1);
        }
    }
    return last;
}

/** Validate a [start, end] range; on failure answer Invalid Handle and return false. */
static bool att_range_ok(att_link_t *L, uint8_t op, uint16_t start, uint16_t end) {
    if (start == 0 || start > end) {
        att_error(L, op, start, FLOSS_ATT_ERR_INVALID_HANDLE);
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------------ */
/* Request handlers                                                         */
/* ------------------------------------------------------------------------ */

static void att_exchange_mtu(att_link_t *L, const uint8_t *p, uint16_t len) {
    if (len != 3) {
        att_error(L, p[0], 0, FLOSS_ATT_ERR_INVALID_PDU);
        return;
    }
    uint16_t client_mtu = floss_get_le16(&p[1]);
    if (client_mtu < FLOSS_ATT_DEFAULT_MTU) {
        client_mtu = FLOSS_ATT_DEFAULT_MTU;
    }
    /* The response itself still uses the old MTU (it is 3 bytes anyway). */
    L->rsp[0] = FLOSS_ATT_MTU_RSP;
    floss_put_le16(&L->rsp[1], FLOSS_ATT_MTU);
    att_send_rsp(L, 3);
    floss_le_set_att_mtu(L->conn, FLOSS_MIN(client_mtu, (uint16_t)FLOSS_ATT_MTU));
}

/* Find Information: (handle, type) pairs, all 16-bit or all 128-bit. */
static void att_find_info(att_link_t *L, const uint8_t *p, uint16_t len) {
    if (len != 5) {
        att_error(L, p[0], 0, FLOSS_ATT_ERR_INVALID_PDU);
        return;
    }
    uint16_t start = floss_get_le16(&p[1]);
    uint16_t end = floss_get_le16(&p[3]);
    if (!att_range_ok(L, p[0], start, end)) {
        return;
    }
    uint16_t mtu = att_mtu(L->conn);
    uint16_t pos = 2;
    uint8_t fmt = 0; /* 1 = 16-bit UUIDs, 2 = 128-bit */
    uint16_t last = FLOSS_MIN(end, floss_gatt_last_handle());
    for (uint32_t h = start; h <= last; h++) {
        const floss_gatt_attr_t *a = floss_gatt_attr((uint16_t)h);
        uint8_t f = a->uuid128 ? 2 : 1;
        uint16_t entry = (f == 1) ? 4 : 18;
        if ((fmt && f != fmt) || pos + entry > mtu) {
            break;
        }
        fmt = f;
        floss_put_le16(&L->rsp[pos], (uint16_t)h);
        if (f == 1) {
            floss_put_le16(&L->rsp[pos + 2], a->type);
        } else {
            memcpy(&L->rsp[pos + 2], a->uuid128, 16);
        }
        pos = (uint16_t)(pos + entry);
    }
    if (!fmt) {
        att_error(L, p[0], start, FLOSS_ATT_ERR_ATTR_NOT_FOUND);
        return;
    }
    L->rsp[0] = FLOSS_ATT_FIND_INFO_RSP;
    L->rsp[1] = fmt;
    att_send_rsp(L, pos);
}

/* Find By Type Value: used by clients to locate a primary service by UUID. */
static void att_find_by_type(att_link_t *L, const uint8_t *p, uint16_t len) {
    if (len < 7) {
        att_error(L, p[0], 0, FLOSS_ATT_ERR_INVALID_PDU);
        return;
    }
    uint16_t start = floss_get_le16(&p[1]);
    uint16_t end = floss_get_le16(&p[3]);
    uint16_t type = floss_get_le16(&p[5]);
    const uint8_t *val = &p[7];
    uint16_t vlen = (uint16_t)(len - 7);
    if (!att_range_ok(L, p[0], start, end)) {
        return;
    }
    uint8_t tmp[FLOSS_ATT_MTU];
    uint16_t mtu = att_mtu(L->conn);
    uint16_t pos = 1;
    uint16_t last = FLOSS_MIN(end, floss_gatt_last_handle());
    for (uint32_t h = start; h <= last && pos + 4 <= mtu; h++) {
        const floss_gatt_attr_t *a = floss_gatt_attr((uint16_t)h);
        if (a->uuid128 || a->type != type || !(a->perms & FLOSS_GATT_P_READ)) {
            continue;
        }
        /* Read one byte more than the pattern so a longer value cannot match. */
        int n = floss_gatt_read_value(L->conn, (uint16_t)h, 0, tmp, (uint16_t)FLOSS_MIN(vlen + 1u, sizeof(tmp)));
        if (n != (int)vlen || memcmp(tmp, val, vlen) != 0) {
            continue;
        }
        uint16_t group_end = (uint16_t)h;
        if (type == FLOSS_GATT_UUID_PRIMARY_SERVICE || type == FLOSS_GATT_UUID_SECONDARY_SERVICE) {
            group_end = att_service_end((uint16_t)h);
        }
        floss_put_le16(&L->rsp[pos], (uint16_t)h);
        floss_put_le16(&L->rsp[pos + 2], group_end);
        pos = (uint16_t)(pos + 4);
        h = group_end; /* continue after the group */
    }
    if (pos == 1) {
        att_error(L, p[0], start, FLOSS_ATT_ERR_ATTR_NOT_FOUND);
        return;
    }
    L->rsp[0] = FLOSS_ATT_FIND_BY_TYPE_RSP;
    att_send_rsp(L, pos);
}

/*
 * Read By Type: (handle, value) pairs of equal length. Clients use it for
 * characteristic discovery (type 0x2803) and "read by UUID".
 */
static void att_read_by_type(att_link_t *L, const uint8_t *p, uint16_t len) {
    att_uuid_t u;
    if ((len != 7 && len != 21) || !att_parse_uuid(&p[5], (uint16_t)(len - 5), &u)) {
        att_error(L, p[0], 0, FLOSS_ATT_ERR_INVALID_PDU);
        return;
    }
    uint16_t start = floss_get_le16(&p[1]);
    uint16_t end = floss_get_le16(&p[3]);
    if (!att_range_ok(L, p[0], start, end)) {
        return;
    }
    uint8_t tmp[FLOSS_ATT_MTU];
    uint16_t mtu = att_mtu(L->conn);
    uint16_t max_val = FLOSS_MIN((uint16_t)(mtu - 4), (uint16_t)253); /* length octet covers handle + value */
    uint16_t pos = 2;
    uint16_t entry = 0;
    uint16_t last = FLOSS_MIN(end, floss_gatt_last_handle());
    for (uint32_t h = start; h <= last; h++) {
        const floss_gatt_attr_t *a = floss_gatt_attr((uint16_t)h);
        if (!att_type_match(a, &u)) {
            continue;
        }
        uint8_t err = att_check(L->conn, a, false);
        int n = err ? 0 : floss_gatt_read_value(L->conn, (uint16_t)h, 0, tmp, (uint16_t)(max_val + 1));
        if (n < 0) {
            err = (uint8_t)(-n);
        }
        if (err) {
            if (entry == 0) {
                att_error(L, p[0], (uint16_t)h, err);
                return;
            }
            break; /* report what we have; the client continues from here */
        }
        /* Values longer than fit are truncated; the client reads the rest
         * with Read Blob when a pair comes back at the maximum length. */
        uint16_t vlen = FLOSS_MIN((uint16_t)n, max_val);
        if (entry == 0) {
            entry = (uint16_t)(2 + vlen);
        } else if ((uint16_t)(2 + vlen) != entry) {
            break; /* all pairs in one response must have the same length */
        }
        if (pos + entry > mtu) {
            break;
        }
        floss_put_le16(&L->rsp[pos], (uint16_t)h);
        memcpy(&L->rsp[pos + 2], tmp, vlen);
        pos = (uint16_t)(pos + entry);
    }
    if (entry == 0) {
        att_error(L, p[0], start, FLOSS_ATT_ERR_ATTR_NOT_FOUND);
        return;
    }
    L->rsp[0] = FLOSS_ATT_READ_BY_TYPE_RSP;
    L->rsp[1] = (uint8_t)entry;
    att_send_rsp(L, pos);
}

static void att_read(att_link_t *L, const uint8_t *p, uint16_t len, bool blob) {
    if (len != (blob ? 5 : 3)) {
        att_error(L, p[0], 0, FLOSS_ATT_ERR_INVALID_PDU);
        return;
    }
    uint16_t h = floss_get_le16(&p[1]);
    uint16_t offset = blob ? floss_get_le16(&p[3]) : 0;
    const floss_gatt_attr_t *a = floss_gatt_attr(h);
    if (a == NULL) {
        att_error(L, p[0], h, FLOSS_ATT_ERR_INVALID_HANDLE);
        return;
    }
    uint8_t err = att_check(L->conn, a, false);
    if (err) {
        att_error(L, p[0], h, err);
        return;
    }
    uint16_t mtu = att_mtu(L->conn);
    /* A value that can never exceed one Read Response is "not long": a
     * client asking for a later part of it is confused. Values that might
     * fill the response exactly are served (possibly with 0 bytes) so the
     * client can detect the end itself. */
    if (blob && offset > 0 && floss_gatt_value_max_len(h) < mtu - 1) {
        att_error(L, p[0], h, FLOSS_ATT_ERR_ATTR_NOT_LONG);
        return;
    }
    int n = floss_gatt_read_value(L->conn, h, offset, &L->rsp[1], (uint16_t)(mtu - 1));
    if (n < 0) {
        att_error(L, p[0], h, (uint8_t)(-n));
        return;
    }
    L->rsp[0] = blob ? FLOSS_ATT_READ_BLOB_RSP : FLOSS_ATT_READ_RSP;
    att_send_rsp(L, (uint16_t)(1 + n));
}

static void att_read_multiple(att_link_t *L, const uint8_t *p, uint16_t len) {
    if (len < 5 || ((len - 1) & 1)) {
        att_error(L, p[0], 0, FLOSS_ATT_ERR_INVALID_PDU);
        return;
    }
    uint16_t mtu = att_mtu(L->conn);
    uint16_t pos = 1;
    for (uint16_t i = 1; i + 1 < len; i = (uint16_t)(i + 2)) {
        uint16_t h = floss_get_le16(&p[i]);
        const floss_gatt_attr_t *a = floss_gatt_attr(h);
        if (a == NULL) {
            att_error(L, p[0], h, FLOSS_ATT_ERR_INVALID_HANDLE);
            return;
        }
        uint8_t err = att_check(L->conn, a, false);
        if (err) {
            att_error(L, p[0], h, err);
            return;
        }
        if (pos < mtu) {
            int n = floss_gatt_read_value(L->conn, h, 0, &L->rsp[pos], (uint16_t)(mtu - pos));
            if (n < 0) {
                att_error(L, p[0], h, (uint8_t)(-n));
                return;
            }
            pos = (uint16_t)(pos + n);
        }
    }
    L->rsp[0] = FLOSS_ATT_READ_MULTI_RSP;
    att_send_rsp(L, pos);
}

/* Read By Group Type: primary service discovery. */
static void att_read_by_group(att_link_t *L, const uint8_t *p, uint16_t len) {
    att_uuid_t u;
    if ((len != 7 && len != 21) || !att_parse_uuid(&p[5], (uint16_t)(len - 5), &u)) {
        att_error(L, p[0], 0, FLOSS_ATT_ERR_INVALID_PDU);
        return;
    }
    uint16_t start = floss_get_le16(&p[1]);
    uint16_t end = floss_get_le16(&p[3]);
    if (!att_range_ok(L, p[0], start, end)) {
        return;
    }
    if (!u.is16 || (u.u16 != FLOSS_GATT_UUID_PRIMARY_SERVICE && u.u16 != FLOSS_GATT_UUID_SECONDARY_SERVICE)) {
        att_error(L, p[0], start, FLOSS_ATT_ERR_UNSUPPORTED_GROUP);
        return;
    }
    uint8_t tmp[17];
    uint16_t mtu = att_mtu(L->conn);
    uint16_t pos = 2;
    uint16_t entry = 0;
    uint16_t last = FLOSS_MIN(end, floss_gatt_last_handle());
    for (uint32_t h = start; h <= last; h++) {
        const floss_gatt_attr_t *a = floss_gatt_attr((uint16_t)h);
        if (a->uuid128 || a->type != u.u16) {
            continue;
        }
        int n = floss_gatt_read_value(L->conn, (uint16_t)h, 0, tmp, sizeof(tmp));
        if (n != 2 && n != 16) {
            break; /* malformed declaration; stop rather than send garbage */
        }
        if (entry == 0) {
            entry = (uint16_t)(4 + n);
        } else if (entry != 4 + n) {
            break;
        }
        if (pos + entry > mtu) {
            break;
        }
        uint16_t group_end = att_service_end((uint16_t)h);
        floss_put_le16(&L->rsp[pos], (uint16_t)h);
        floss_put_le16(&L->rsp[pos + 2], group_end);
        memcpy(&L->rsp[pos + 4], tmp, (size_t)n);
        pos = (uint16_t)(pos + entry);
        h = group_end;
    }
    if (entry == 0) {
        att_error(L, p[0], start, FLOSS_ATT_ERR_ATTR_NOT_FOUND);
        return;
    }
    L->rsp[0] = FLOSS_ATT_READ_BY_GROUP_RSP;
    L->rsp[1] = (uint8_t)entry;
    att_send_rsp(L, pos);
}

/* Write Request (with response) and Write Command (without). */
static void att_write(att_link_t *L, const uint8_t *p, uint16_t len, bool cmd) {
    if (len < 3) {
        if (!cmd) {
            att_error(L, p[0], 0, FLOSS_ATT_ERR_INVALID_PDU);
        }
        return;
    }
    uint16_t h = floss_get_le16(&p[1]);
    const floss_gatt_attr_t *a = floss_gatt_attr(h);
    uint8_t err = a ? att_check(L->conn, a, true) : FLOSS_ATT_ERR_INVALID_HANDLE;
    if (!err) {
        err = floss_gatt_write_value(L->conn, h, 0, &p[3], (uint16_t)(len - 3), cmd ? FLOSS_GATT_WRITE_F_CMD : 0);
    }
    if (cmd) {
        if (err) {
            FLOSS_LOGD(LE, "att: write cmd 0x%04x dropped (0x%02x)", (unsigned)h, (unsigned)err);
            if (err == FLOSS_ATT_ERR_INSUFFICIENT_ENC || err == FLOSS_ATT_ERR_INSUFFICIENT_AUTHEN) {
                att_request_security(L);
            }
        }
        return; /* commands never get a response, not even an error */
    }
    if (err) {
        att_error(L, p[0], h, err);
        return;
    }
    L->rsp[0] = FLOSS_ATT_WRITE_RSP;
    att_send_rsp(L, 1);
}

static void att_prep_clear(void) {
    s_prep.active = false;
    s_prep.start = s_prep.end = 0;
    /* Gaps between queued segments are written too: never with stale bytes
     * from an earlier queue (possibly another link's). */
    memset(s_prep.data, 0, sizeof(s_prep.data));
}

static void att_prepare_write(att_link_t *L, const uint8_t *p, uint16_t len) {
    if (len < 5) {
        att_error(L, p[0], 0, FLOSS_ATT_ERR_INVALID_PDU);
        return;
    }
    if (len > att_mtu(L->conn)) {
        /* The response echoes the request: it could not fit the client's MTU. */
        att_error(L, p[0], 0, FLOSS_ATT_ERR_INVALID_PDU);
        return;
    }
    uint16_t h = floss_get_le16(&p[1]);
    uint16_t offset = floss_get_le16(&p[3]);
    uint16_t vlen = (uint16_t)(len - 5);
    const floss_gatt_attr_t *a = floss_gatt_attr(h);
    if (a == NULL) {
        att_error(L, p[0], h, FLOSS_ATT_ERR_INVALID_HANDLE);
        return;
    }
    /* Permissions are checked now; offset/length validity at execute time. */
    uint8_t err = att_check(L->conn, a, true);
    if (err) {
        att_error(L, p[0], h, err);
        return;
    }
    if ((s_prep.active && (s_prep.conn != L->conn || s_prep.handle != h)) ||
        (uint32_t)offset + vlen > FLOSS_ATT_PREPARE_MAX) {
        att_error(L, p[0], h, FLOSS_ATT_ERR_PREPARE_QUEUE_FULL);
        return;
    }
    if (!s_prep.active) {
        s_prep.active = true;
        s_prep.conn = L->conn;
        s_prep.handle = h;
        s_prep.start = offset;
        s_prep.end = (uint16_t)(offset + vlen);
    } else {
        s_prep.start = FLOSS_MIN(s_prep.start, offset);
        s_prep.end = FLOSS_MAX(s_prep.end, (uint16_t)(offset + vlen));
    }
    memcpy(&s_prep.data[offset], &p[5], vlen);
    /* The response echoes the request so the client can verify it. */
    memcpy(L->rsp, p, len);
    L->rsp[0] = FLOSS_ATT_PREPARE_WRITE_RSP;
    att_send_rsp(L, len);
}

static void att_execute_write(att_link_t *L, const uint8_t *p, uint16_t len) {
    if (len != 2 || p[1] > 1) {
        att_error(L, p[0], 0, FLOSS_ATT_ERR_INVALID_PDU);
        return;
    }
    bool mine = s_prep.active && s_prep.conn == L->conn;
    if (mine && p[1] == 1) {
        uint16_t h = s_prep.handle;
        uint8_t err = floss_gatt_write_value(L->conn, h, s_prep.start, &s_prep.data[s_prep.start],
                                             (uint16_t)(s_prep.end - s_prep.start), FLOSS_GATT_WRITE_F_PREPARED);
        att_prep_clear();
        if (err) {
            att_error(L, p[0], h, err);
            return;
        }
    } else if (mine) {
        att_prep_clear(); /* cancel */
    }
    L->rsp[0] = FLOSS_ATT_EXECUTE_WRITE_RSP;
    att_send_rsp(L, 1);
}

static void att_confirm(att_link_t *L) {
    if (!L->ind_pending) {
        return;
    }
    L->ind_pending = false;
    floss_timer_stop(&L->ind_timer);
    floss_le_evt_t evt = {.type = FLOSS_LE_EVT_INDICATION_DONE, .handle = L->conn, .attr_handle = L->ind_handle};
    floss_le_emit(&evt);
}

/** PDUs a server may receive but never answers (responses to requests we never send, etc.). */
static bool att_is_unsolicited(uint8_t op) {
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

void floss_att_on_pdu(uint16_t conn, const uint8_t *pdu, uint16_t len) {
    att_link_t *L = att_link(conn);
    if (L == NULL || len == 0) {
        return;
    }
    if (len > FLOSS_ATT_MTU) {
        /* Larger than the MTU we offered: protocol violation. Commands are
         * dropped; requests get Invalid PDU so the client is not left waiting. */
        if (!(pdu[0] & FLOSS_ATT_OP_COMMAND_FLAG) && !att_is_unsolicited(pdu[0])) {
            att_error(L, pdu[0], 0, FLOSS_ATT_ERR_INVALID_PDU);
        }
        return;
    }
    uint8_t op = pdu[0];
    FLOSS_LOGT(LE, "att: rx op 0x%02x len %u", (unsigned)op, (unsigned)len);
    switch (op) {
    case FLOSS_ATT_MTU_REQ:
        att_exchange_mtu(L, pdu, len);
        break;
    case FLOSS_ATT_FIND_INFO_REQ:
        att_find_info(L, pdu, len);
        break;
    case FLOSS_ATT_FIND_BY_TYPE_REQ:
        att_find_by_type(L, pdu, len);
        break;
    case FLOSS_ATT_READ_BY_TYPE_REQ:
        att_read_by_type(L, pdu, len);
        break;
    case FLOSS_ATT_READ_REQ:
        att_read(L, pdu, len, false);
        break;
    case FLOSS_ATT_READ_BLOB_REQ:
        att_read(L, pdu, len, true);
        break;
    case FLOSS_ATT_READ_MULTI_REQ:
        att_read_multiple(L, pdu, len);
        break;
    case FLOSS_ATT_READ_BY_GROUP_REQ:
        att_read_by_group(L, pdu, len);
        break;
    case FLOSS_ATT_WRITE_REQ:
        att_write(L, pdu, len, false);
        break;
    case FLOSS_ATT_WRITE_CMD:
        att_write(L, pdu, len, true);
        break;
    case FLOSS_ATT_PREPARE_WRITE_REQ:
        att_prepare_write(L, pdu, len);
        break;
    case FLOSS_ATT_EXECUTE_WRITE_REQ:
        att_execute_write(L, pdu, len);
        break;
    case FLOSS_ATT_CONFIRM:
        att_confirm(L);
        break;
    default:
        if (op & FLOSS_ATT_OP_COMMAND_FLAG) {
            /* Signed Write Command and unknown commands: silently ignored. */
            FLOSS_LOGD(LE, "att: ignoring command 0x%02x", (unsigned)op);
        } else if (!att_is_unsolicited(op)) {
            att_error(L, op, 0, FLOSS_ATT_ERR_REQUEST_NOT_SUPPORTED);
        }
        break;
    }
}

/* ------------------------------------------------------------------------ */
/* Notifications and indications                                            */
/* ------------------------------------------------------------------------ */

/** Allocate a buffer holding [opcode][handle] for a server-initiated PDU. */
static floss_buf_t *att_alloc_pdu(uint16_t conn, uint8_t opcode, uint16_t value_handle, uint8_t **value,
                                  uint16_t *cap) {
    const floss_le_link_t *l = floss_le_link(conn);
    if (l == NULL) {
        return NULL;
    }
    floss_buf_t *b = floss_buf_alloc();
    if (b == NULL) {
        FLOSS_STAT_INC(buf_alloc_failures);
        return NULL;
    }
    b->handle = conn;
    b->type = FLOSS_PKT_ACL;
    b->flags = 0;
    uint8_t *p = floss_buf_put(b, 3);
    p[0] = opcode;
    floss_put_le16(&p[1], value_handle);
    *value = b->data + 3;
    *cap = FLOSS_MIN((uint16_t)(att_mtu(conn) - 3), floss_buf_tailroom(b));
    return b;
}

floss_buf_t *floss_gatt_notify_alloc(uint16_t conn, uint16_t value_handle, uint8_t **value, uint16_t *cap) {
    return att_alloc_pdu(conn, FLOSS_ATT_NOTIFY, value_handle, value, cap);
}

floss_err_t floss_gatt_notify_send(floss_buf_t *buf, uint16_t len, uint8_t flags) {
    buf->len = (uint16_t)(3 + len);
    uint8_t *hdr = floss_buf_push(buf, FLOSS_L2CAP_HDR_LEN);
    floss_put_le16(hdr, (uint16_t)(buf->len - FLOSS_L2CAP_HDR_LEN));
    floss_put_le16(&hdr[2], FLOSS_CID_ATT);
    buf->flags = flags;
    return floss_hci_acl_send(buf);
}

floss_err_t floss_gatt_notify(uint16_t conn, uint16_t value_handle, const uint8_t *data, uint16_t len) {
    if (!(floss_gatt_cccd_get(conn, value_handle) & FLOSS_GATT_CCCD_NOTIFY)) {
        return FLOSS_ERR_STATE;
    }
    uint8_t *value;
    uint16_t cap;
    floss_buf_t *b = floss_gatt_notify_alloc(conn, value_handle, &value, &cap);
    if (b == NULL) {
        return floss_le_link(conn) ? FLOSS_ERR_BUSY : FLOSS_ERR_STATE;
    }
    len = FLOSS_MIN(len, cap);
    memcpy(value, data, len);
    return floss_gatt_notify_send(b, len, 0);
}

uint8_t floss_gatt_notify_all(uint16_t value_handle, const uint8_t *data, uint16_t len) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        uint16_t conn = floss_le_link_handle_at(i);
        if (conn != 0xFFFF && floss_gatt_notify(conn, value_handle, data, len) == FLOSS_OK) {
            n++;
        }
    }
    return n;
}

static void att_ind_timeout(floss_timer_t *t, void *arg) {
    att_link_t *L = (att_link_t *)arg;
    if (!L->used || !L->ind_pending) {
        return;
    }
    FLOSS_LOGW(LE, "att: indication 0x%04x not confirmed in 30 s, closing link", (unsigned)L->ind_handle);
    L->ind_pending = false;
    floss_le_evt_t evt = {.type = FLOSS_LE_EVT_INDICATION_DONE,
                          .handle = L->conn,
                          .attr_handle = L->ind_handle,
                          .status = FLOSS_HCI_ERR_CONN_TIMEOUT};
    floss_le_emit(&evt);
    /* An ATT transaction timeout ends the bearer: no further ATT traffic
     * is allowed on this link, so drop it. */
    floss_le_disconnect(L->conn, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED);
}

floss_err_t floss_gatt_indicate(uint16_t conn, uint16_t value_handle, const uint8_t *data, uint16_t len) {
    att_link_t *L = att_link(conn);
    if (L == NULL || !(floss_gatt_cccd_get(conn, value_handle) & FLOSS_GATT_CCCD_INDICATE)) {
        return FLOSS_ERR_STATE;
    }
    if (L->ind_pending) {
        return FLOSS_ERR_BUSY;
    }
    uint8_t *value;
    uint16_t cap;
    floss_buf_t *b = att_alloc_pdu(conn, FLOSS_ATT_INDICATE, value_handle, &value, &cap);
    if (b == NULL) {
        return FLOSS_ERR_BUSY;
    }
    len = FLOSS_MIN(len, cap);
    memcpy(value, data, len);
    floss_err_t err = floss_gatt_notify_send(b, len, 0);
    if (err == FLOSS_OK) {
        L->ind_pending = true;
        L->ind_handle = value_handle;
        floss_timer_start(&L->ind_timer, FLOSS_ATT_TIMEOUT_US);
    }
    return err;
}

bool floss_gatt_indication_pending(uint16_t conn) {
    att_link_t *L = att_link(conn);
    return L && L->ind_pending;
}

/* ------------------------------------------------------------------------ */
/* Lifecycle                                                                */
/* ------------------------------------------------------------------------ */

void floss_att_init(void) {
    memset(s_att, 0, sizeof(s_att));
    att_prep_clear();
}

void floss_att_on_link_up(uint16_t conn) {
    int idx = floss_le_link_index(conn);
    if (idx < 0) {
        return;
    }
    att_link_t *L = &s_att[idx];
    memset(L, 0, sizeof(*L));
    L->used = true;
    L->conn = conn;
    floss_timer_init(&L->ind_timer, att_ind_timeout, L);
}

void floss_att_on_link_down(uint16_t conn) {
    att_link_t *L = att_link(conn);
    if (L == NULL) {
        return;
    }
    floss_timer_stop(&L->ind_timer);
    if (L->ind_pending) {
        floss_le_evt_t evt = {.type = FLOSS_LE_EVT_INDICATION_DONE,
                              .handle = conn,
                              .attr_handle = L->ind_handle,
                              .status = FLOSS_HCI_ERR_UNKNOWN_CONN_ID};
        L->ind_pending = false;
        floss_le_emit(&evt);
    }
    if (s_prep.active && s_prep.conn == conn) {
        att_prep_clear();
    }
    L->used = false;
}

void floss_att_poll(void) {
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        att_link_t *L = &s_att[i];
        if (L->used && L->rsp_pending) {
            att_send_rsp(L, L->rsp_pending);
        }
    }
}

#endif /* FLOSS_ENABLE_LE */
