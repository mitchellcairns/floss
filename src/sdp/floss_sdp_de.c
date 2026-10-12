/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_sdp_de.c - SDP data element writer and header parser.
 *
 * Overview
 * --------
 * The writer streams elements into a caller-supplied buffer. Fixed-size
 * values (bool, integers, UUIDs) always use their natural single-byte
 * header. Sequences are the only element whose size is unknown when it is
 * opened, so seq_begin() writes a 3-byte header with a 2-byte length
 * placeholder (0x36 LL LL) and remembers its offset; seq_end() patches the
 * length in place. Using the 16-bit length form for every sequence costs at
 * most one byte per sequence compared with the minimal form, and in return
 * nesting never needs to move bytes around.
 *
 * Errors are sticky: the first overflow or bracket mismatch sets w->error
 * and every later call becomes a no-op, so builders can write a whole
 * record and check the flag once at the end.
 *
 * floss_de_header() is the single place that decodes element headers, for
 * the server's request parser, record validation and the client's result
 * walker.
 */
#include "floss_sdp_priv.h"

/* ------------------------------------------------------------------------ */
/* Writer                                                                   */
/* ------------------------------------------------------------------------ */

void floss_de_init(floss_de_writer_t *w, uint8_t *buf, uint16_t cap)
{
    w->buf = buf;
    w->cap = cap;
    w->len = 0;
    w->depth = 0;
    w->error = (buf == NULL);
}

/** Reserve n bytes at the tail; NULL (and sticky error) if they do not fit. */
static uint8_t *de_reserve(floss_de_writer_t *w, uint16_t n)
{
    if (w->error) {
        return NULL;
    }
    if ((uint32_t)w->len + n > w->cap) {
        w->error = true;
        return NULL;
    }
    uint8_t *p = w->buf + w->len;
    w->len = (uint16_t)(w->len + n);
    return p;
}

void floss_de_seq_begin(floss_de_writer_t *w)
{
    if (w->error) {
        return;
    }
    if (w->depth >= FLOSS_DE_MAX_DEPTH) {
        w->error = true;
        return;
    }
    uint16_t at = w->len;
    uint8_t *p = de_reserve(w, 3);
    if (p == NULL) {
        return;
    }
    p[0] = FLOSS_DE_HDR_SEQ16;
    p[1] = 0;
    p[2] = 0;
    w->open[w->depth++] = at;
}

void floss_de_seq_end(floss_de_writer_t *w)
{
    if (w->error) {
        return;
    }
    if (w->depth == 0) {
        w->error = true; /* unbalanced: more ends than begins */
        return;
    }
    uint16_t at = w->open[--w->depth];
    /* Payload = everything written since the 3-byte header. */
    floss_put_be16(&w->buf[at + 1], (uint16_t)(w->len - at - 3));
}

void floss_de_nil(floss_de_writer_t *w)
{
    uint8_t *p = de_reserve(w, 1);
    if (p != NULL) {
        p[0] = FLOSS_DE_HDR(FLOSS_DE_TYPE_NIL, FLOSS_DE_SIZE_1);
    }
}

void floss_de_bool(floss_de_writer_t *w, bool v)
{
    uint8_t *p = de_reserve(w, 2);
    if (p != NULL) {
        p[0] = FLOSS_DE_HDR(FLOSS_DE_TYPE_BOOL, FLOSS_DE_SIZE_1); /* 0x28 */
        p[1] = v ? 1 : 0;
    }
}

void floss_de_uint8(floss_de_writer_t *w, uint8_t v)
{
    uint8_t *p = de_reserve(w, 2);
    if (p != NULL) {
        p[0] = FLOSS_DE_HDR(FLOSS_DE_TYPE_UINT, FLOSS_DE_SIZE_1); /* 0x08 */
        p[1] = v;
    }
}

void floss_de_uint16(floss_de_writer_t *w, uint16_t v)
{
    uint8_t *p = de_reserve(w, 3);
    if (p != NULL) {
        p[0] = FLOSS_DE_HDR_UINT16;
        floss_put_be16(p + 1, v);
    }
}

void floss_de_uint32(floss_de_writer_t *w, uint32_t v)
{
    uint8_t *p = de_reserve(w, 5);
    if (p != NULL) {
        p[0] = FLOSS_DE_HDR_UINT32;
        floss_put_be32(p + 1, v);
    }
}

void floss_de_int8(floss_de_writer_t *w, int8_t v)
{
    uint8_t *p = de_reserve(w, 2);
    if (p != NULL) {
        p[0] = FLOSS_DE_HDR(FLOSS_DE_TYPE_INT, FLOSS_DE_SIZE_1); /* 0x10 */
        p[1] = (uint8_t)v;
    }
}

void floss_de_uuid16(floss_de_writer_t *w, uint16_t uuid)
{
    uint8_t *p = de_reserve(w, 3);
    if (p != NULL) {
        p[0] = FLOSS_DE_HDR(FLOSS_DE_TYPE_UUID, FLOSS_DE_SIZE_2); /* 0x19 */
        floss_put_be16(p + 1, uuid);
    }
}

void floss_de_uuid32(floss_de_writer_t *w, uint32_t uuid)
{
    uint8_t *p = de_reserve(w, 5);
    if (p != NULL) {
        p[0] = FLOSS_DE_HDR(FLOSS_DE_TYPE_UUID, FLOSS_DE_SIZE_4); /* 0x1A */
        floss_put_be32(p + 1, uuid);
    }
}

void floss_de_uuid128(floss_de_writer_t *w, const uint8_t uuid[16])
{
    uint8_t *p = de_reserve(w, 17);
    if (p != NULL) {
        p[0] = FLOSS_DE_HDR(FLOSS_DE_TYPE_UUID, FLOSS_DE_SIZE_16); /* 0x1C */
        memcpy(p + 1, uuid, 16);
    }
}

/**
 * Text string. SDP text has no terminator and no implied encoding beyond the
 * language base attribute, so it doubles as the container for binary blobs
 * (the HID report descriptor). Short strings use the 1-byte length form.
 */
void floss_de_text(floss_de_writer_t *w, const uint8_t *data, uint16_t len)
{
    uint8_t *p;
    if (len < 256) {
        p = de_reserve(w, (uint16_t)(2u + len));
        if (p == NULL) {
            return;
        }
        p[0] = FLOSS_DE_HDR(FLOSS_DE_TYPE_TEXT, FLOSS_DE_SIZE_LEN8); /* 0x25 */
        p[1] = (uint8_t)len;
        p += 2;
    } else {
        if ((uint32_t)len + 3u > 0xFFFFu) {
            w->error = true;
            return;
        }
        p = de_reserve(w, (uint16_t)(3u + len));
        if (p == NULL) {
            return;
        }
        p[0] = FLOSS_DE_HDR(FLOSS_DE_TYPE_TEXT, FLOSS_DE_SIZE_LEN16); /* 0x26 */
        floss_put_be16(p + 1, len);
        p += 3;
    }
    if (len > 0) {
        memcpy(p, data, len);
    }
}

void floss_de_str(floss_de_writer_t *w, const char *s)
{
    size_t n = (s != NULL) ? strlen(s) : 0;
    if (n > 0xFFFFu) {
        w->error = true;
        return;
    }
    floss_de_text(w, (const uint8_t *)s, (uint16_t)n);
}

void floss_de_raw(floss_de_writer_t *w, const uint8_t *data, uint16_t len)
{
    uint8_t *p = de_reserve(w, len);
    if (p != NULL && len > 0) {
        memcpy(p, data, len);
    }
}

/* ------------------------------------------------------------------------ */
/* Header parser                                                            */
/* ------------------------------------------------------------------------ */

/**
 * Decode the element header at p.
 *
 * *type receives the 5-bit type descriptor (FLOSS_DE_TYPE_*, i.e. the
 * header byte shifted right by 3) and *len the payload length. Returns the
 * header size (1, 2, 3 or 5), or 0 when the element is malformed:
 *   - the header or the payload does not fit in `avail` bytes,
 *   - the type is reserved (9..31),
 *   - the size index is not allowed for the type (e.g. a 3-byte UUID, a
 *     sequence without an explicit length, a nil with a payload).
 * Because the payload is bounds-checked too, callers may walk the payload
 * immediately without further length checks.
 */
uint16_t floss_de_header(const uint8_t *p, uint16_t avail, uint8_t *type, uint32_t *len)
{
    static const uint8_t fixed_len[5] = {1, 2, 4, 8, 16};

    if (p == NULL || avail < 1) {
        return 0;
    }
    uint8_t t = (uint8_t)(p[0] >> 3);
    uint8_t s = (uint8_t)(p[0] & 0x07);
    uint16_t hdr;
    uint32_t n;

    switch (s) {
    case FLOSS_DE_SIZE_LEN8:
        if (avail < 2) {
            return 0;
        }
        hdr = 2;
        n = p[1];
        break;
    case FLOSS_DE_SIZE_LEN16:
        if (avail < 3) {
            return 0;
        }
        hdr = 3;
        n = floss_get_be16(p + 1);
        break;
    case FLOSS_DE_SIZE_LEN32:
        if (avail < 5) {
            return 0;
        }
        hdr = 5;
        n = floss_get_be32(p + 1);
        break;
    default:
        hdr = 1;
        n = fixed_len[s];
        break;
    }

    /* Which size indexes each type may legally use. */
    bool ok;
    switch (t) {
    case FLOSS_DE_TYPE_NIL:
        ok = (s == FLOSS_DE_SIZE_1);
        n = 0; /* nil carries no payload despite size index 0 */
        break;
    case FLOSS_DE_TYPE_UINT:
    case FLOSS_DE_TYPE_INT:
        ok = (s <= FLOSS_DE_SIZE_16);
        break;
    case FLOSS_DE_TYPE_UUID:
        ok = (s == FLOSS_DE_SIZE_2 || s == FLOSS_DE_SIZE_4 || s == FLOSS_DE_SIZE_16);
        break;
    case FLOSS_DE_TYPE_BOOL:
        ok = (s == FLOSS_DE_SIZE_1);
        break;
    case FLOSS_DE_TYPE_TEXT:
    case FLOSS_DE_TYPE_SEQ:
    case FLOSS_DE_TYPE_ALT:
    case FLOSS_DE_TYPE_URL:
        ok = (s >= FLOSS_DE_SIZE_LEN8);
        break;
    default:
        ok = false; /* reserved type descriptor */
        break;
    }
    if (!ok || n > (uint32_t)(avail - hdr)) {
        return 0;
    }
    *type = t;
    *len = n;
    return hdr;
}

/* ------------------------------------------------------------------------ */
/* UUIDs                                                                    */
/* ------------------------------------------------------------------------ */

/**
 * Expand a 16/32/128-bit UUID payload to 128 bits. Short UUIDs are aliases
 * into the base UUID 00000000-0000-1000-8000-00805F9B34FB: the 16/32-bit
 * value replaces the first 32 bits. Comparing everything at full width makes
 * a 16-bit pattern match a 128-bit record entry and vice versa.
 */
void floss_sdp_uuid_promote(const uint8_t *p, uint32_t len, uint8_t out[16])
{
    static const uint8_t base[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
                                     0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
    if (len == 16) {
        memcpy(out, p, 16);
        return;
    }
    memcpy(out, base, 16);
    if (len == 2) {
        out[2] = p[0];
        out[3] = p[1];
    } else { /* 4 */
        memcpy(out, p, 4);
    }
}
