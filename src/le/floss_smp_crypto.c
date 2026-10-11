/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_smp_crypto.c - LE Security Manager toolbox (e, c1, s1, ah, f4, f5, f6, g2).
 *
 * Design
 * ------
 * Each function is a direct transcription of its definition in Core
 * Specification Vol 3 Part H section 2.2, plus one twist: the API is in wire
 * order (LSB-first, see floss_smp_crypto.h) while the definitions, AES and
 * AES-CMAC are MSB-first.
 *
 *  - The AES based legacy functions (e, c1, s1, ah) are cheapest done in wire
 *    order: their concatenations are fixed-width, so "a || b" (a more
 *    significant) in wire order is simply "b's bytes, then a's bytes". Only
 *    e() itself reverses its operands around the block cipher.
 *
 *  - The CMAC based functions (f4, f5, f6, g2) have byte-string messages, so
 *    their messages are assembled MSB-first by copying each field reversed
 *    (put_rev), and the 128-bit results are reversed back on the way out.
 *
 * Nothing here keeps state; every function may be called at any time.
 */
#include "floss_smp_crypto.h"

#if FLOSS_ENABLE_LE

#include "floss/floss_hal.h"
#include "floss_aes.h"

/** dst[0..n-1] = src reversed. dst and src must not overlap. */
static void put_rev(uint8_t *dst, const uint8_t *src, size_t n) {
    for (size_t i = 0; i < n; i++) {
        dst[i] = src[n - 1 - i];
    }
}

static void xor16(uint8_t *dst, const uint8_t *a, const uint8_t *b) {
    for (unsigned i = 0; i < 16; i++) {
        dst[i] = (uint8_t)(a[i] ^ b[i]);
    }
}

/** AES-CMAC with a wire-order key, on an already MSB-first message; wire-order result. */
static void cmac_wire_key(const uint8_t key_le[16], const uint8_t *msg, size_t len, uint8_t out_le[16]) {
    uint8_t key[16], mac[16];
    put_rev(key, key_le, 16);
    floss_aes_cmac(key, msg, len, mac);
    put_rev(out_le, mac, 16);
}

/* ------------------------------------------------------------------------ */
/* Legacy pairing and privacy (AES based)                                   */
/* ------------------------------------------------------------------------ */

void floss_smp_e(const uint8_t key[16], const uint8_t plaintext[16], uint8_t out[16]) {
    /* e's operands are numbers whose most significant octet is AES byte 0
     * (this is also how HCI_LE_Encrypt is specified). */
    uint8_t k[16], p[16], c[16];
    put_rev(k, key, 16);
    put_rev(p, plaintext, 16);
    floss_aes128(k, p, c);
    put_rev(out, c, 16);
}

void floss_smp_c1(const uint8_t k[16], const uint8_t r[16], const uint8_t preq[7], const uint8_t pres[7],
                  uint8_t iat, const uint8_t ia[6], uint8_t rat, const uint8_t ra[6], uint8_t out[16]) {
    uint8_t p[16], t[16];

    /* p1 = pres || preq || rat' || iat'  (iat' least significant) */
    p[0] = iat & 1u;
    p[1] = rat & 1u;
    memcpy(&p[2], preq, 7);
    memcpy(&p[9], pres, 7);
    xor16(t, r, p);
    floss_smp_e(k, t, t);

    /* p2 = padding(32 zero bits) || ia || ra  (ra least significant) */
    memcpy(&p[0], ra, 6);
    memcpy(&p[6], ia, 6);
    memset(&p[12], 0, 4);
    xor16(t, t, p);
    floss_smp_e(k, t, out);
}

void floss_smp_s1(const uint8_t k[16], const uint8_t r1[16], const uint8_t r2[16], uint8_t out[16]) {
    /* r' = r1' || r2', the least significant 64 bits of each (r2' least significant). */
    uint8_t r[16];
    memcpy(&r[0], r2, 8);
    memcpy(&r[8], r1, 8);
    floss_smp_e(k, r, out);
}

uint32_t floss_smp_ah(const uint8_t irk[16], const uint8_t prand[3]) {
    /* r' = 104 zero bits || r; ah = e(k, r') mod 2^24 */
    uint8_t r[16], h[16];
    memset(r, 0, sizeof r);
    memcpy(r, prand, 3);
    floss_smp_e(irk, r, h);
    return floss_get_le24(h);
}

bool floss_smp_rpa_matches(const uint8_t irk[16], const floss_addr_t *rpa) {
    /* RPA layout (wire order): hash = bytes 0..2, prand = bytes 3..5 with the
     * two most significant bits of byte 5 equal to 0b01. */
    if ((rpa->b[5] & 0xC0) != 0x40) {
        return false;
    }
    return floss_smp_ah(irk, &rpa->b[3]) == floss_get_le24(&rpa->b[0]);
}

void floss_smp_rpa_generate(const uint8_t irk[16], floss_addr_t *out) {
    uint8_t *p = &out->b[3];
    do {
        floss_hal_random(p, 3);
        p[2] = (uint8_t)((p[2] & 0x3F) | 0x40);
        /* The random part of prand shall not be all 0 or all 1. */
    } while ((p[0] == 0x00 && p[1] == 0x00 && p[2] == 0x40) ||
             (p[0] == 0xFF && p[1] == 0xFF && p[2] == 0x7F));
    floss_put_le24(&out->b[0], floss_smp_ah(irk, p));
}

/* ------------------------------------------------------------------------ */
/* LE Secure Connections (AES-CMAC based)                                   */
/* ------------------------------------------------------------------------ */

void floss_smp_f4(const uint8_t u[32], const uint8_t v[32], const uint8_t x[16], uint8_t z, uint8_t out[16]) {
    uint8_t m[65]; /* U || V || Z */
    put_rev(&m[0], u, 32);
    put_rev(&m[32], v, 32);
    m[64] = z;
    cmac_wire_key(x, m, sizeof m, out);
}

void floss_smp_f5(const uint8_t w[32], const uint8_t n1[16], const uint8_t n2[16], const uint8_t a1[7],
                  const uint8_t a2[7], uint8_t mackey[16], uint8_t ltk[16]) {
    /* SALT = 0x6C888391AAF5A53860370BDB5A6083BE (MSB-first, used directly as the CMAC key). */
    static const uint8_t salt[16] = {0x6C, 0x88, 0x83, 0x91, 0xAA, 0xF5, 0xA5, 0x38,
                                     0x60, 0x37, 0x0B, 0xDB, 0x5A, 0x60, 0x83, 0xBE};
    uint8_t t[16], mac[16];
    uint8_t m[53]; /* Counter || keyID || N1 || N2 || A1 || A2 || Length */

    /* T = AES-CMAC_SALT(W) */
    put_rev(m, w, 32);
    floss_aes_cmac(salt, m, 32, t);

    m[0] = 0;                 /* Counter */
    m[1] = 0x62;              /* keyID "btle" = 0x62746C65 */
    m[2] = 0x74;
    m[3] = 0x6C;
    m[4] = 0x65;
    put_rev(&m[5], n1, 16);
    put_rev(&m[21], n2, 16);
    put_rev(&m[37], a1, 7);
    put_rev(&m[44], a2, 7);
    m[51] = 0x01;             /* Length = 256 bits */
    m[52] = 0x00;

    floss_aes_cmac(t, m, sizeof m, mac);
    put_rev(mackey, mac, 16);
    m[0] = 1;
    floss_aes_cmac(t, m, sizeof m, mac);
    put_rev(ltk, mac, 16);

    memset(t, 0, sizeof t);
}

void floss_smp_f6(const uint8_t w[16], const uint8_t n1[16], const uint8_t n2[16], const uint8_t r[16],
                  const uint8_t iocap[3], const uint8_t a1[7], const uint8_t a2[7], uint8_t out[16]) {
    uint8_t m[65]; /* N1 || N2 || R || IOcap || A1 || A2 */
    put_rev(&m[0], n1, 16);
    put_rev(&m[16], n2, 16);
    put_rev(&m[32], r, 16);
    put_rev(&m[48], iocap, 3);
    put_rev(&m[51], a1, 7);
    put_rev(&m[58], a2, 7);
    cmac_wire_key(w, m, sizeof m, out);
}

uint32_t floss_smp_g2(const uint8_t u[32], const uint8_t v[32], const uint8_t x[16], const uint8_t y[16]) {
    uint8_t m[80]; /* U || V || Y */
    uint8_t key[16], mac[16];
    put_rev(&m[0], u, 32);
    put_rev(&m[32], v, 32);
    put_rev(&m[64], y, 16);
    put_rev(key, x, 16);
    floss_aes_cmac(key, m, sizeof m, mac);
    return floss_get_be32(&mac[12]); /* mod 2^32: the least significant 32 bits */
}

#else
typedef int floss_smp_crypto_unused_t; /* ISO C forbids an empty translation unit */
#endif /* FLOSS_ENABLE_LE */
