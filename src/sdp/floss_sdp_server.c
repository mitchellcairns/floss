/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_sdp_server.c - SDP server: record table and request/response PDUs.
 *
 * Overview
 * --------
 * SDP runs on L2CAP PSM 0x0001 with no security requirement (a host must be
 * able to discover services before pairing). Every request is answered with
 * exactly one response PDU; there is no server-side session state.
 *
 *   PDU    = id(1) | transaction id(2, BE) | parameter length(2, BE) | params
 *
 *   0x02 ServiceSearch          pattern -> list of record handles
 *   0x04 ServiceAttribute       handle + attribute IDs -> one attribute list
 *   0x06 ServiceSearchAttribute pattern + attribute IDs -> list of lists
 *   0x01 ErrorResponse          for anything malformed
 *
 * Large answers are split with "continuation state": the server returns a
 * chunk plus an opaque token (<= 16 bytes); the client repeats the identical
 * request with that token to get the next chunk.
 *
 * The "virtual response" model
 * ----------------------------
 * Rather than caching a partially sent response per client, the server
 * regenerates the complete response deterministically on every request and
 * copies out only the requested byte window [offset, offset + chunk). The
 * generator (emit_*) streams record bytes through a window filter, so the
 * full response never exists in memory. Records are a few hundred bytes, so
 * regenerating is cheaper than any bookkeeping and keeps the server
 * stateless across clients and links.
 *
 * Our continuation token is 6 bytes:
 *   offset (uint32 BE: byte offset into the virtual response, or record
 *           index for ServiceSearch) | check (uint16 BE)
 * `check` hashes the request (PDU ID, pattern, attribute list, handle) and a
 * database generation counter, so a token replayed against a different
 * request, or after a record was (un)registered, is rejected with
 * "invalid continuation state" instead of returning spliced garbage.
 *
 * Records are validated once at registration (well-formed elements, sorted
 * unique attribute IDs, bounded nesting), so the request path can walk them
 * without re-checking every length.
 */
#include "floss_sdp_priv.h"

/* ------------------------------------------------------------------------ */
/* Protocol constants                                                       */
/* ------------------------------------------------------------------------ */

#define SDP_PDU_ERROR_RSP           0x01
#define SDP_PDU_SEARCH_REQ          0x02
#define SDP_PDU_SEARCH_RSP          0x03
#define SDP_PDU_ATTR_REQ            0x04
#define SDP_PDU_ATTR_RSP            0x05
#define SDP_PDU_SEARCH_ATTR_REQ     0x06
#define SDP_PDU_SEARCH_ATTR_RSP     0x07

#define SDP_ERR_INVALID_HANDLE      0x0002
#define SDP_ERR_INVALID_SYNTAX      0x0003
#define SDP_ERR_INVALID_PDU_SIZE    0x0004
#define SDP_ERR_INVALID_CONT        0x0005
#define SDP_ERR_INSUFFICIENT_RES    0x0006

#define SDP_HDR_LEN        5   /* id + tid + param length */
#define SDP_CONT_MAX       16  /* protocol limit on continuation state bytes */
#define SDP_CONT_LEN       6   /* our token: offset32 + check16 */
#define SDP_CONT_FIELD     (1 + SDP_CONT_LEN) /* length byte + token, reserved in every response */
#define SDP_MAX_PATTERN    12  /* protocol limit on UUIDs in a search pattern */
#define SDP_MIN_ATTR_BYTES 7   /* smallest legal MaximumAttributeByteCount */
#define SDP_MIN_MTU        48  /* Classic L2CAP minimum MTU (used if unknown) */
#define SDP_FIRST_HANDLE   0x00010000u /* 0x00000000 is reserved for the SDP server itself */

/** Response scratch buffer: one PDU, never more than the peer's MTU. */
#ifndef FLOSS_SDP_TX_MAX
#define FLOSS_SDP_TX_MAX FLOSS_L2CAP_MTU
#endif

_Static_assert(FLOSS_SDP_MAX_RECORDS <= 32, "record match masks are 32 bits");
_Static_assert(FLOSS_SDP_TX_MAX >= SDP_MIN_MTU, "SDP TX buffer below minimum MTU");

/* ------------------------------------------------------------------------ */
/* State                                                                    */
/* ------------------------------------------------------------------------ */

static struct {
    floss_sdp_record_t *rec[FLOSS_SDP_MAX_RECORDS]; /**< registration order */
    uint8_t count;
    uint32_t next_handle;
    uint16_t generation;          /**< bumped on (un)register; invalidates tokens */
    uint8_t tx[FLOSS_SDP_TX_MAX]; /**< response being built / awaiting a credit */
    floss_l2cap_chan_t *pending;  /**< channel whose send returned BUSY */
    uint16_t pending_len;
} sdp;

/** A parsed request. Raw pattern / ID-list payloads point into the PDU. */
typedef struct {
    uint8_t pdu_id;
    uint16_t tid;
    uint8_t uuids[SDP_MAX_PATTERN][16]; /**< pattern, promoted to 128-bit */
    uint8_t n_uuids;
    const uint8_t *ids;   /**< AttributeIDList payload (validated) */
    uint16_t ids_len;
    uint32_t handle;      /**< ServiceAttribute only */
    uint16_t max;         /**< MaximumServiceRecordCount / MaximumAttributeByteCount */
    bool has_cont;
    uint32_t offset;      /**< from continuation token, else 0 */
    uint32_t hash;        /**< running FNV-1a over the request identity */
} sdp_req_t;

/** Read cursor over the request parameters. */
typedef struct {
    const uint8_t *p;
    uint16_t left;
} cursor_t;

/* ------------------------------------------------------------------------ */
/* Small helpers                                                            */
/* ------------------------------------------------------------------------ */

static uint32_t fnv1a(uint32_t h, const uint8_t *p, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        h = (h ^ p[i]) * 16777619u;
    }
    return h;
}

static uint16_t hash_fold(uint32_t h)
{
    return (uint16_t)((h >> 16) ^ h);
}

/** Header size of the most compact sequence header for `content` bytes. */
static uint8_t seq_hdr_len(uint32_t content)
{
    return content < 0x100u ? 2 : (content < 0x10000u ? 3 : 5);
}

static uint8_t seq_hdr_write(uint8_t *out, uint32_t content)
{
    if (content < 0x100u) {
        out[0] = FLOSS_DE_HDR(FLOSS_DE_TYPE_SEQ, FLOSS_DE_SIZE_LEN8);
        out[1] = (uint8_t)content;
        return 2;
    }
    if (content < 0x10000u) {
        out[0] = FLOSS_DE_HDR(FLOSS_DE_TYPE_SEQ, FLOSS_DE_SIZE_LEN16);
        floss_put_be16(out + 1, (uint16_t)content);
        return 3;
    }
    out[0] = FLOSS_DE_HDR(FLOSS_DE_TYPE_SEQ, FLOSS_DE_SIZE_LEN32);
    floss_put_be32(out + 1, content);
    return 5;
}

/* ------------------------------------------------------------------------ */
/* Record walking                                                           */
/* ------------------------------------------------------------------------ */

/** Iterator over the (attribute ID, value) pairs of a validated record. */
typedef struct {
    const uint8_t *p;
    const uint8_t *end;
} attr_iter_t;

static void attr_iter_init(attr_iter_t *it, const floss_sdp_record_t *rec)
{
    uint8_t t;
    uint32_t n;
    uint16_t h = floss_de_header(rec->data, rec->len, &t, &n);
    it->p = rec->data + h;
    it->end = it->p + n;
}

/**
 * Next pair: *id = attribute ID, *pair and *pair_len = the ID element followed
 * by the value element. Both are contiguous in the record, which is what
 * lets responses copy record bytes verbatim.
 */
static bool attr_next(attr_iter_t *it, uint16_t *id, const uint8_t **pair, uint16_t *pair_len)
{
    if (it->p + 3 > it->end) {
        return false;
    }
    uint8_t t;
    uint32_t n;
    uint16_t h = floss_de_header(it->p + 3, (uint16_t)(it->end - it->p - 3), &t, &n);
    if (h == 0) {
        return false; /* unreachable for validated records */
    }
    *id = floss_get_be16(it->p + 1);
    *pair = it->p;
    *pair_len = (uint16_t)(3u + h + n);
    it->p += *pair_len;
    return true;
}

/**
 * Validate the payload of a sequence/alternative: every element header is
 * legal, every payload fits its container, nesting stays bounded.
 */
static bool validate_elements(const uint8_t *p, uint32_t n, uint8_t depth)
{
    if (depth > FLOSS_SDP_MAX_NEST) {
        return false;
    }
    while (n > 0) {
        uint8_t t;
        uint32_t len;
        uint16_t h = floss_de_header(p, (uint16_t)n, &t, &len);
        if (h == 0) {
            return false;
        }
        if ((t == FLOSS_DE_TYPE_SEQ || t == FLOSS_DE_TYPE_ALT) &&
            !validate_elements(p + h, len, (uint8_t)(depth + 1))) {
            return false;
        }
        p += h + len;
        n -= h + len;
    }
    return true;
}

/**
 * Check a record before accepting it: one sequence spanning exactly
 * rec->len, starting with attribute 0x0000 as a uint32 (so the handle can be
 * patched in place), followed by pairs whose IDs are uint16 and strictly
 * ascending - the order the protocol requires in responses.
 */
static bool validate_record(const floss_sdp_record_t *rec)
{
    uint8_t t;
    uint32_t n;
    uint16_t h = floss_de_header(rec->data, rec->len, &t, &n);
    if (h == 0 || t != FLOSS_DE_TYPE_SEQ || h + n != rec->len || n < 8) {
        return false;
    }
    const uint8_t *p = rec->data + h;
    if (p[0] != FLOSS_DE_HDR_UINT16 || p[1] != 0 || p[2] != 0 || p[3] != FLOSS_DE_HDR_UINT32) {
        return false;
    }
    const uint8_t *end = p + n;
    int32_t prev = -1;
    while (p < end) {
        uint16_t left = (uint16_t)(end - p);
        if (left < 3 || p[0] != FLOSS_DE_HDR_UINT16) {
            return false;
        }
        uint16_t id = floss_get_be16(p + 1);
        if ((int32_t)id <= prev) {
            return false;
        }
        prev = id;
        uint32_t len;
        uint16_t vh = floss_de_header(p + 3, (uint16_t)(left - 3), &t, &len);
        if (vh == 0) {
            return false;
        }
        if ((t == FLOSS_DE_TYPE_SEQ || t == FLOSS_DE_TYPE_ALT) && !validate_elements(p + 3 + vh, len, 1)) {
            return false;
        }
        p += 3u + vh + len;
    }
    return true;
}

/**
 * Pattern match: every pattern UUID must appear somewhere in the record.
 * Sequences are contiguous in memory, so a flat left-to-right scan that
 * steps INTO sequence headers (rather than over them) visits every nested
 * element without recursion. Text payloads are skipped whole, so binary
 * blobs such as report descriptors can never produce false UUIDs.
 */
static bool record_matches(const floss_sdp_record_t *rec, const sdp_req_t *rq)
{
    const uint32_t want = (1u << rq->n_uuids) - 1u;
    uint32_t got = 0;
    const uint8_t *p = rec->data;
    const uint8_t *end = rec->data + rec->len;

    while (p < end && got != want) {
        uint8_t t;
        uint32_t len;
        uint16_t h = floss_de_header(p, (uint16_t)(end - p), &t, &len);
        if (h == 0) {
            break;
        }
        if (t == FLOSS_DE_TYPE_SEQ || t == FLOSS_DE_TYPE_ALT) {
            p += h; /* descend */
            continue;
        }
        if (t == FLOSS_DE_TYPE_UUID) {
            uint8_t u[16];
            floss_sdp_uuid_promote(p + h, len, u);
            for (uint8_t i = 0; i < rq->n_uuids; i++) {
                if (memcmp(u, rq->uuids[i], 16) == 0) {
                    got |= 1u << i;
                }
            }
        }
        p += h + len;
    }
    return got == want;
}

/** Is `id` covered by the (validated) AttributeIDList payload? */
static bool id_wanted(const sdp_req_t *rq, uint16_t id)
{
    const uint8_t *p = rq->ids;
    const uint8_t *end = rq->ids + rq->ids_len;
    while (p < end) {
        if (p[0] == FLOSS_DE_HDR_UINT16) {
            if (floss_get_be16(p + 1) == id) {
                return true;
            }
            p += 3;
        } else { /* uint32 range: high 16 bits = first, low 16 = last */
            if (id >= floss_get_be16(p + 1) && id <= floss_get_be16(p + 3)) {
                return true;
            }
            p += 5;
        }
    }
    return false;
}

/** Bytes of the filtered attribute list payload for one record. */
static uint32_t filtered_size(const floss_sdp_record_t *rec, const sdp_req_t *rq)
{
    attr_iter_t it;
    uint16_t id, plen;
    const uint8_t *pair;
    uint32_t total = 0;
    attr_iter_init(&it, rec);
    while (attr_next(&it, &id, &pair, &plen)) {
        if (id_wanted(rq, id)) {
            total += plen;
        }
    }
    return total;
}

/* ------------------------------------------------------------------------ */
/* Windowed emitter (the virtual response)                                  */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint8_t *dst;   /**< receives bytes [start, end) of the virtual stream */
    uint32_t pos;   /**< virtual position of the next emitted byte */
    uint32_t start;
    uint32_t end;
} emitter_t;

static void emit(emitter_t *e, const uint8_t *src, uint32_t n)
{
    uint32_t a = e->pos;
    uint32_t b = e->pos + n;
    if (b > e->start && a < e->end) {
        uint32_t lo = FLOSS_MAX(a, e->start);
        uint32_t hi = FLOSS_MIN(b, e->end);
        memcpy(e->dst + (lo - e->start), src + (lo - a), hi - lo);
    }
    e->pos = b;
}

static void emit_seq_hdr(emitter_t *e, uint32_t content)
{
    uint8_t hdr[5];
    emit(e, hdr, seq_hdr_write(hdr, content));
}

/** Emit one record's filtered attribute list (header + matching pairs). */
static void emit_attr_list(emitter_t *e, const floss_sdp_record_t *rec, const sdp_req_t *rq, uint32_t content)
{
    attr_iter_t it;
    uint16_t id, plen;
    const uint8_t *pair;
    emit_seq_hdr(e, content);
    attr_iter_init(&it, rec);
    while (attr_next(&it, &id, &pair, &plen) && e->pos < e->end) {
        if (id_wanted(rq, id)) {
            emit(e, pair, plen);
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Request parsing                                                          */
/* ------------------------------------------------------------------------ */

static uint16_t take_u16(cursor_t *c, uint16_t *v)
{
    if (c->left < 2) {
        return SDP_ERR_INVALID_PDU_SIZE;
    }
    *v = floss_get_be16(c->p);
    c->p += 2;
    c->left -= 2;
    return 0;
}

static uint16_t take_u32(cursor_t *c, uint32_t *v)
{
    if (c->left < 4) {
        return SDP_ERR_INVALID_PDU_SIZE;
    }
    *v = floss_get_be32(c->p);
    c->p += 4;
    c->left -= 4;
    return 0;
}

/** ServiceSearchPattern: a sequence of 1..12 UUIDs of any width. */
static uint16_t parse_pattern(cursor_t *c, sdp_req_t *rq)
{
    uint8_t t;
    uint32_t n;
    uint16_t h = floss_de_header(c->p, c->left, &t, &n);
    if (h == 0 || t != FLOSS_DE_TYPE_SEQ) {
        return SDP_ERR_INVALID_SYNTAX;
    }
    const uint8_t *p = c->p + h;
    const uint8_t *end = p + n;
    rq->n_uuids = 0;
    while (p < end) {
        uint32_t len;
        uint16_t eh = floss_de_header(p, (uint16_t)(end - p), &t, &len);
        if (eh == 0 || t != FLOSS_DE_TYPE_UUID || rq->n_uuids >= SDP_MAX_PATTERN) {
            return SDP_ERR_INVALID_SYNTAX;
        }
        floss_sdp_uuid_promote(p + eh, len, rq->uuids[rq->n_uuids++]);
        p += eh + len;
    }
    if (rq->n_uuids == 0) {
        return SDP_ERR_INVALID_SYNTAX;
    }
    rq->hash = fnv1a(rq->hash, c->p, (uint16_t)(h + n));
    c->p += h + n;
    c->left = (uint16_t)(c->left - h - n);
    return 0;
}

/** AttributeIDList: a sequence of uint16 IDs and/or uint32 ID ranges. */
static uint16_t parse_id_list(cursor_t *c, sdp_req_t *rq)
{
    uint8_t t;
    uint32_t n;
    uint16_t h = floss_de_header(c->p, c->left, &t, &n);
    if (h == 0 || t != FLOSS_DE_TYPE_SEQ) {
        return SDP_ERR_INVALID_SYNTAX;
    }
    const uint8_t *p = c->p + h;
    const uint8_t *end = p + n;
    while (p < end) {
        /* Exact header bytes only: id_wanted() relies on these two shapes. */
        if (p[0] == FLOSS_DE_HDR_UINT16 && end - p >= 3) {
            p += 3;
        } else if (p[0] == FLOSS_DE_HDR_UINT32 && end - p >= 5) {
            if (floss_get_be16(p + 1) > floss_get_be16(p + 3)) {
                return SDP_ERR_INVALID_SYNTAX; /* inverted range */
            }
            p += 5;
        } else {
            return SDP_ERR_INVALID_SYNTAX;
        }
    }
    rq->ids = c->p + h;
    rq->ids_len = (uint16_t)n;
    rq->hash = fnv1a(rq->hash, c->p, (uint16_t)(h + n));
    c->p += h + n;
    c->left = (uint16_t)(c->left - h - n);
    return 0;
}

/**
 * ContinuationState: must be the last parameter. Empty = first request.
 * Otherwise it has to be one of our own tokens for this exact request and
 * the current record database.
 */
static uint16_t parse_cont(cursor_t *c, sdp_req_t *rq)
{
    if (c->left < 1) {
        return SDP_ERR_INVALID_PDU_SIZE;
    }
    uint8_t n = c->p[0];
    if (n > SDP_CONT_MAX) {
        return SDP_ERR_INVALID_CONT;
    }
    if (c->left != 1u + n) {
        return SDP_ERR_INVALID_PDU_SIZE;
    }
    rq->has_cont = (n != 0);
    rq->offset = 0;
    if (n == 0) {
        return 0;
    }
    if (n != SDP_CONT_LEN || floss_get_be16(c->p + 5) != hash_fold(rq->hash)) {
        return SDP_ERR_INVALID_CONT;
    }
    rq->offset = floss_get_be32(c->p + 1);
    return 0;
}

/** Append our continuation token for `next` (or an empty one) at out. */
static uint16_t put_cont(uint8_t *out, const sdp_req_t *rq, bool more, uint32_t next)
{
    if (!more) {
        out[0] = 0;
        return 1;
    }
    out[0] = SDP_CONT_LEN;
    floss_put_be32(out + 1, next);
    floss_put_be16(out + 5, hash_fold(rq->hash));
    return SDP_CONT_FIELD;
}

/* ------------------------------------------------------------------------ */
/* Request handlers                                                         */
/* Each writes its parameters at sdp.tx + SDP_HDR_LEN and returns their      */
/* length, or 0 with *err set.                                              */
/* ------------------------------------------------------------------------ */

static uint16_t do_search(const sdp_req_t *rq, uint16_t mtu, uint16_t *err)
{
    if (rq->max == 0) {
        *err = SDP_ERR_INVALID_SYNTAX;
        return 0;
    }
    uint32_t matched[FLOSS_SDP_MAX_RECORDS];
    uint16_t total = 0;
    for (uint8_t i = 0; i < sdp.count && total < rq->max; i++) {
        if (record_matches(sdp.rec[i], rq)) {
            matched[total++] = sdp.rec[i]->handle;
        }
    }
    if (rq->has_cont && rq->offset >= total) {
        *err = SDP_ERR_INVALID_CONT;
        return 0;
    }
    /* total(2) + current(2) + handles + continuation field */
    int32_t room = (int32_t)mtu - SDP_HDR_LEN - 4 - SDP_CONT_FIELD;
    if (room < 4) {
        *err = SDP_ERR_INSUFFICIENT_RES;
        return 0;
    }
    uint16_t off = (uint16_t)rq->offset;
    uint16_t cur = (uint16_t)FLOSS_MIN((uint32_t)(total - off), (uint32_t)room / 4u);

    uint8_t *o = sdp.tx + SDP_HDR_LEN;
    floss_put_be16(o, total);
    floss_put_be16(o + 2, cur);
    for (uint16_t i = 0; i < cur; i++) {
        floss_put_be32(o + 4 + 4 * i, matched[off + i]);
    }
    uint16_t n = (uint16_t)(4 + 4 * cur);
    n = (uint16_t)(n + put_cont(o + n, rq, off + cur < total, (uint32_t)off + cur));
    FLOSS_LOGD(SDP, "search: %u match(es), sent %u from %u", total, cur, off);
    return n;
}

/**
 * Shared tail of ServiceAttribute / ServiceSearchAttribute: given the size
 * of the virtual response, emit the requested window and the token.
 * `gen` re-runs the generator into the emitter.
 */
typedef void (*gen_fn)(emitter_t *e, const sdp_req_t *rq, const void *ctx);

static uint16_t send_window(const sdp_req_t *rq, uint16_t mtu, uint32_t total, gen_fn gen, const void *ctx,
                            uint16_t *err)
{
    if (rq->max < SDP_MIN_ATTR_BYTES) {
        *err = SDP_ERR_INVALID_SYNTAX;
        return 0;
    }
    if (rq->has_cont && rq->offset >= total) {
        *err = SDP_ERR_INVALID_CONT;
        return 0;
    }
    /* byte count(2) + data + continuation field */
    int32_t room = (int32_t)mtu - SDP_HDR_LEN - 2 - SDP_CONT_FIELD;
    if (room < 1) {
        *err = SDP_ERR_INSUFFICIENT_RES;
        return 0;
    }
    uint32_t chunk = FLOSS_MIN(total - rq->offset, (uint32_t)FLOSS_MIN((int32_t)rq->max, room));

    uint8_t *o = sdp.tx + SDP_HDR_LEN;
    emitter_t e = {.dst = o + 2, .pos = 0, .start = rq->offset, .end = rq->offset + chunk};
    gen(&e, rq, ctx);
    floss_put_be16(o, (uint16_t)chunk);
    uint16_t n = (uint16_t)(2 + chunk);
    bool more = rq->offset + chunk < total;
    n = (uint16_t)(n + put_cont(o + n, rq, more, rq->offset + chunk));
    FLOSS_LOGD(SDP, "attr: %lu/%lu bytes from %lu%s", (unsigned long)chunk, (unsigned long)total,
               (unsigned long)rq->offset, more ? " (more)" : "");
    return n;
}

/* ---- ServiceAttribute ---- */

typedef struct {
    const floss_sdp_record_t *rec;
    uint32_t content;
} sa_ctx_t;

static void gen_sa(emitter_t *e, const sdp_req_t *rq, const void *ctx)
{
    const sa_ctx_t *c = ctx;
    emit_attr_list(e, c->rec, rq, c->content);
}

static uint16_t do_attr(const sdp_req_t *rq, uint16_t mtu, uint16_t *err)
{
    sa_ctx_t c = {NULL, 0};
    for (uint8_t i = 0; i < sdp.count; i++) {
        if (sdp.rec[i]->handle == rq->handle) {
            c.rec = sdp.rec[i];
            break;
        }
    }
    if (c.rec == NULL) {
        *err = SDP_ERR_INVALID_HANDLE;
        return 0;
    }
    c.content = filtered_size(c.rec, rq);
    uint32_t total = seq_hdr_len(c.content) + c.content;
    return send_window(rq, mtu, total, gen_sa, &c, err);
}

/* ---- ServiceSearchAttribute ---- */

typedef struct {
    uint32_t mask;                          /**< matching records with a non-empty list */
    uint32_t size[FLOSS_SDP_MAX_RECORDS];   /**< filtered payload size per record */
    uint32_t outer;                         /**< payload of the outer sequence */
} ssa_ctx_t;

static void gen_ssa(emitter_t *e, const sdp_req_t *rq, const void *ctx)
{
    const ssa_ctx_t *c = ctx;
    emit_seq_hdr(e, c->outer);
    for (uint8_t i = 0; i < sdp.count && e->pos < e->end; i++) {
        if (c->mask & (1u << i)) {
            emit_attr_list(e, sdp.rec[i], rq, c->size[i]);
        }
    }
}

/**
 * The response is one sequence holding one attribute-list sequence per
 * matching record. A matching record with none of the requested attributes
 * contributes nothing (an empty list carries no information).
 */
static uint16_t do_search_attr(const sdp_req_t *rq, uint16_t mtu, uint16_t *err)
{
    ssa_ctx_t c;
    c.mask = 0;
    c.outer = 0;
    for (uint8_t i = 0; i < sdp.count; i++) {
        c.size[i] = 0;
        if (!record_matches(sdp.rec[i], rq)) {
            continue;
        }
        c.size[i] = filtered_size(sdp.rec[i], rq);
        if (c.size[i] > 0) {
            c.mask |= 1u << i;
            c.outer += seq_hdr_len(c.size[i]) + c.size[i];
        }
    }
    uint32_t total = seq_hdr_len(c.outer) + c.outer;
    return send_window(rq, mtu, total, gen_ssa, &c, err);
}

/* ------------------------------------------------------------------------ */
/* PDU dispatch                                                             */
/* ------------------------------------------------------------------------ */

/** Parse the parameters of a request PDU into rq. Returns 0 or an SDP error. */
static uint16_t parse_request(sdp_req_t *rq, const uint8_t *params, uint16_t len)
{
    cursor_t c = {params, len};
    uint16_t err;

    rq->hash = fnv1a(2166136261u, &rq->pdu_id, 1);
    uint8_t gen[2];
    floss_put_be16(gen, sdp.generation);
    rq->hash = fnv1a(rq->hash, gen, 2);

    switch (rq->pdu_id) {
    case SDP_PDU_SEARCH_REQ:
        if ((err = parse_pattern(&c, rq)) != 0 || (err = take_u16(&c, &rq->max)) != 0) {
            return err;
        }
        break;
    case SDP_PDU_ATTR_REQ:
        if ((err = take_u32(&c, &rq->handle)) != 0 || (err = take_u16(&c, &rq->max)) != 0 ||
            (err = parse_id_list(&c, rq)) != 0) {
            return err;
        }
        rq->hash = fnv1a(rq->hash, params, 4); /* the handle */
        break;
    case SDP_PDU_SEARCH_ATTR_REQ:
        if ((err = parse_pattern(&c, rq)) != 0 || (err = take_u16(&c, &rq->max)) != 0 ||
            (err = parse_id_list(&c, rq)) != 0) {
            return err;
        }
        break;
    default:
        return SDP_ERR_INVALID_SYNTAX; /* responses or unknown PDUs sent to a server */
    }
    return parse_cont(&c, rq);
}

static void sdp_send(floss_l2cap_chan_t *chan, uint16_t len)
{
    floss_err_t rc = floss_l2cap_send(chan, sdp.tx, len);
    if (rc == FLOSS_ERR_BUSY) {
        /* Keep the PDU in sdp.tx and retry from on_can_send. */
        if (sdp.pending != NULL && sdp.pending != chan) {
            FLOSS_LOGW(SDP, "dropping deferred response on cid 0x%04x", sdp.pending->local_cid);
        }
        sdp.pending = chan;
        sdp.pending_len = len;
    } else if (rc != FLOSS_OK) {
        FLOSS_LOGW(SDP, "send failed: %s", floss_err_str(rc));
    }
}

/**
 * sdp.tx is about to be rebuilt: a response still waiting in it (BUSY) must
 * go out first or be dropped, never be sent later with another response's
 * bytes (possibly to another client).
 */
static void sdp_settle_pending(void)
{
    floss_l2cap_chan_t *p = sdp.pending;
    if (p == NULL) {
        return;
    }
    sdp.pending = NULL;
    sdp_send(p, sdp.pending_len);
    if (sdp.pending != NULL) {
        FLOSS_LOGW(SDP, "dropping deferred response on cid 0x%04x", sdp.pending->local_cid);
        sdp.pending = NULL;
    }
}

/**
 * One request in, one response out. Everything the response needs is
 * derived from the request bytes and the record table, so this function has
 * no per-client state.
 */
static void sdp_on_data(floss_l2cap_chan_t *chan, const uint8_t *data, uint16_t len)
{
    sdp_req_t rq;
    uint16_t err = 0;
    uint16_t plen = 0;

    sdp_settle_pending();
    rq.pdu_id = (len >= 1) ? data[0] : 0;
    rq.tid = (len >= 3) ? floss_get_be16(data + 1) : 0;
    FLOSS_HEXDUMP(SDP, "req", data, len);

    uint16_t mtu = chan->remote_mtu ? chan->remote_mtu : SDP_MIN_MTU;
    mtu = FLOSS_MIN(mtu, (uint16_t)FLOSS_SDP_TX_MAX);

    if (len < SDP_HDR_LEN || floss_get_be16(data + 3) != len - SDP_HDR_LEN) {
        err = SDP_ERR_INVALID_PDU_SIZE;
    } else {
        err = parse_request(&rq, data + SDP_HDR_LEN, (uint16_t)(len - SDP_HDR_LEN));
    }
    if (err == 0) {
        switch (rq.pdu_id) {
        case SDP_PDU_SEARCH_REQ:
            plen = do_search(&rq, mtu, &err);
            break;
        case SDP_PDU_ATTR_REQ:
            plen = do_attr(&rq, mtu, &err);
            break;
        default:
            plen = do_search_attr(&rq, mtu, &err);
            break;
        }
    }

    uint8_t rsp_id;
    if (err != 0) {
        FLOSS_LOGW(SDP, "request 0x%02x tid %u rejected: error 0x%04x", rq.pdu_id, rq.tid, err);
        rsp_id = SDP_PDU_ERROR_RSP;
        floss_put_be16(sdp.tx + SDP_HDR_LEN, err);
        plen = 2;
    } else {
        rsp_id = (uint8_t)(rq.pdu_id + 1); /* every response ID is request ID + 1 */
    }
    sdp.tx[0] = rsp_id;
    floss_put_be16(sdp.tx + 1, rq.tid);
    floss_put_be16(sdp.tx + 3, plen);
    sdp_send(chan, (uint16_t)(SDP_HDR_LEN + plen));
}

static void sdp_on_can_send(floss_l2cap_chan_t *chan)
{
    if (sdp.pending == chan) {
        sdp.pending = NULL;
        sdp_send(chan, sdp.pending_len);
    }
}

static void sdp_on_close(floss_l2cap_chan_t *chan)
{
    if (sdp.pending == chan) {
        sdp.pending = NULL;
    }
}

static const floss_l2cap_ops_t sdp_ops = {
    .on_open = NULL,
    .on_close = sdp_on_close,
    .on_data = sdp_on_data,
    .on_can_send = sdp_on_can_send,
};

/* ------------------------------------------------------------------------ */
/* Registration                                                             */
/* ------------------------------------------------------------------------ */

/** Advertise every 16-bit UUID of the ServiceClassIDList in the EIR. */
static void eir_add_classes(const floss_sdp_record_t *rec)
{
    attr_iter_t it;
    uint16_t id, plen;
    const uint8_t *pair;
    attr_iter_init(&it, rec);
    while (attr_next(&it, &id, &pair, &plen)) {
        if (id < FLOSS_ATTR_SERVICE_CLASS_ID_LIST) {
            continue;
        }
        if (id > FLOSS_ATTR_SERVICE_CLASS_ID_LIST) {
            return; /* sorted: not present */
        }
        const uint8_t *p = pair + 3;
        const uint8_t *end = pair + plen;
        while (p < end) {
            uint8_t t;
            uint32_t len;
            uint16_t h = floss_de_header(p, (uint16_t)(end - p), &t, &len);
            if (h == 0) {
                return;
            }
            if (t == FLOSS_DE_TYPE_SEQ || t == FLOSS_DE_TYPE_ALT) {
                p += h;
                continue;
            }
            if (t == FLOSS_DE_TYPE_UUID && len == 2) {
                floss_gap_eir_add_uuid16(floss_get_be16(p + h));
            }
            p += h + len;
        }
        return;
    }
}

floss_err_t floss_sdp_register(floss_sdp_record_t *rec)
{
    if (rec == NULL || rec->data == NULL || !validate_record(rec)) {
        FLOSS_LOGE(SDP, "register: malformed record");
        return FLOSS_ERR_INVALID;
    }
    for (uint8_t i = 0; i < sdp.count; i++) {
        if (sdp.rec[i] == rec) {
            return FLOSS_ERR_STATE;
        }
    }
    if (sdp.count >= FLOSS_SDP_MAX_RECORDS) {
        FLOSS_LOGE(SDP, "register: table full (FLOSS_SDP_MAX_RECORDS=%u)", (unsigned)FLOSS_SDP_MAX_RECORDS);
        return FLOSS_ERR_NOMEM;
    }
    if (sdp.next_handle < SDP_FIRST_HANDLE) {
        sdp.next_handle = SDP_FIRST_HANDLE; /* registered before floss_sdp_init() */
    }

    /* Patch the uint32 value of attribute 0x0000 (validated position). */
    uint8_t t;
    uint32_t n;
    uint16_t h = floss_de_header(rec->data, rec->len, &t, &n);
    rec->handle = sdp.next_handle++;
    floss_put_be32(rec->data + h + 4, rec->handle);

    sdp.rec[sdp.count++] = rec;
    sdp.generation++;
    eir_add_classes(rec);
    FLOSS_LOGI(SDP, "record 0x%08lx registered (%u bytes)", (unsigned long)rec->handle, rec->len);
    return FLOSS_OK;
}

floss_err_t floss_sdp_unregister(floss_sdp_record_t *rec)
{
    for (uint8_t i = 0; i < sdp.count; i++) {
        if (sdp.rec[i] == rec) {
            /* Shift down to keep registration order (search result order). */
            for (uint8_t j = i; j + 1 < sdp.count; j++) {
                sdp.rec[j] = sdp.rec[j + 1];
            }
            sdp.count--;
            sdp.rec[sdp.count] = NULL;
            sdp.generation++;
            FLOSS_LOGI(SDP, "record 0x%08lx unregistered", (unsigned long)rec->handle);
            return FLOSS_OK;
        }
    }
    return FLOSS_ERR_NOT_FOUND;
}

void floss_sdp_init(void)
{
    memset(&sdp, 0, sizeof(sdp));
    sdp.next_handle = SDP_FIRST_HANDLE;
    floss_err_t rc = floss_l2cap_register_psm(FLOSS_PSM_SDP, &sdp_ops, NULL, FLOSS_SEC_NONE);
    if (rc != FLOSS_OK) {
        FLOSS_LOGE(SDP, "cannot register PSM 0x0001: %s", floss_err_str(rc));
    }
#if FLOSS_ENABLE_SDP_CLIENT
    floss_sdp_client_init();
#endif
}
