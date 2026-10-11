/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_aes.c - Compact AES-128 encryption (FIPS-197) and AES-CMAC (RFC 4493).
 *
 * Design
 * ------
 * Written directly from FIPS-197 and RFC 4493. The goal is small flash/RAM on
 * a Cortex-M0+ rather than raw speed: pairing runs a few dozen block
 * encryptions in total, so a 256-byte S-box plus on-the-fly MixColumns
 * (xtime) is plenty fast and avoids 4 KiB of T-tables.
 *
 * State layout follows FIPS-197 section 3.4: input byte i lands in state row
 * (i % 4), column (i / 4). Keeping the 16 bytes in that column-major order
 * means the input copies straight in, and each MixColumns column is four
 * consecutive bytes.
 *
 * Side channels: the S-box lookup is data dependent. On the cache-less MCUs
 * FLOSS targets that does not leak timing; desktop builds are for testing.
 *
 * Validated against FIPS-197 Appendix C.1, the four RFC 4493 examples and
 * (through the SMP toolbox) the sample data of the Core Specification,
 * Vol 3 Part H, Appendix D.
 */
#include "floss_aes.h"

#if FLOSS_ENABLE_LE

/* FIPS-197 Figure 7: multiplicative inverse in GF(2^8) followed by the affine
 * transform. Generated from the definition, not copied. */
static const uint8_t s_sbox[256] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
};

/** Multiply by x (i.e. {02}) in GF(2^8) modulo x^8 + x^4 + x^3 + x + 1. */
static inline uint8_t aes_xtime(uint8_t x) {
    return (uint8_t)((x << 1) ^ ((x >> 7) * 0x1b));
}

void floss_aes128_init(floss_aes128_t *ctx, const uint8_t key[16]) {
    uint8_t *rk = ctx->rk;
    uint8_t rcon = 0x01;

    memcpy(rk, key, 16);
    /* FIPS-197 5.2 KeyExpansion with Nk = 4, working one 4-byte word at a time. */
    for (unsigned i = 16; i < 176; i += 4) {
        uint8_t t0 = rk[i - 4], t1 = rk[i - 3], t2 = rk[i - 2], t3 = rk[i - 1];
        if ((i & 15) == 0) {
            /* SubWord(RotWord(w)) xor Rcon */
            uint8_t tmp = t0;
            t0 = (uint8_t)(s_sbox[t1] ^ rcon);
            t1 = s_sbox[t2];
            t2 = s_sbox[t3];
            t3 = s_sbox[tmp];
            rcon = aes_xtime(rcon);
        }
        rk[i + 0] = (uint8_t)(rk[i - 16] ^ t0);
        rk[i + 1] = (uint8_t)(rk[i - 15] ^ t1);
        rk[i + 2] = (uint8_t)(rk[i - 14] ^ t2);
        rk[i + 3] = (uint8_t)(rk[i - 13] ^ t3);
    }
}

/**
 * SubBytes and ShiftRows fused. Row r rotates left by r columns, so the
 * byte at (row r, column c) comes from (row r, column (c + r) mod 4).
 */
static void aes_sub_shift(uint8_t s[16]) {
    uint8_t t;
    /* row 0: no shift */
    s[0] = s_sbox[s[0]];
    s[4] = s_sbox[s[4]];
    s[8] = s_sbox[s[8]];
    s[12] = s_sbox[s[12]];
    /* row 1: rotate by 1 */
    t = s[1];
    s[1] = s_sbox[s[5]];
    s[5] = s_sbox[s[9]];
    s[9] = s_sbox[s[13]];
    s[13] = s_sbox[t];
    /* row 2: rotate by 2 (two swaps) */
    t = s[2];
    s[2] = s_sbox[s[10]];
    s[10] = s_sbox[t];
    t = s[6];
    s[6] = s_sbox[s[14]];
    s[14] = s_sbox[t];
    /* row 3: rotate by 3 (= right by 1) */
    t = s[15];
    s[15] = s_sbox[s[11]];
    s[11] = s_sbox[s[7]];
    s[7] = s_sbox[s[3]];
    s[3] = s_sbox[t];
}

/**
 * MixColumns (FIPS-197 5.1.3). With t = a0^a1^a2^a3, the matrix row
 * {02}a0 ^ {03}a1 ^ a2 ^ a3 equals a0 ^ t ^ xtime(a0 ^ a1), and likewise
 * for the other rows, which needs only one xtime per output byte.
 */
static void aes_mix_columns(uint8_t s[16]) {
    for (unsigned c = 0; c < 16; c += 4) {
        uint8_t a0 = s[c], a1 = s[c + 1], a2 = s[c + 2], a3 = s[c + 3];
        uint8_t t = (uint8_t)(a0 ^ a1 ^ a2 ^ a3);
        s[c + 0] = (uint8_t)(a0 ^ t ^ aes_xtime((uint8_t)(a0 ^ a1)));
        s[c + 1] = (uint8_t)(a1 ^ t ^ aes_xtime((uint8_t)(a1 ^ a2)));
        s[c + 2] = (uint8_t)(a2 ^ t ^ aes_xtime((uint8_t)(a2 ^ a3)));
        s[c + 3] = (uint8_t)(a3 ^ t ^ aes_xtime((uint8_t)(a3 ^ a0)));
    }
}

static inline void aes_add_round_key(uint8_t s[16], const uint8_t *rk) {
    for (unsigned i = 0; i < 16; i++) {
        s[i] ^= rk[i];
    }
}

void floss_aes128_encrypt(const floss_aes128_t *ctx, const uint8_t in[16], uint8_t out[16]) {
    uint8_t s[16];

    memcpy(s, in, 16);
    aes_add_round_key(s, ctx->rk);
    for (unsigned round = 1; round < 10; round++) {
        aes_sub_shift(s);
        aes_mix_columns(s);
        aes_add_round_key(s, ctx->rk + 16 * round);
    }
    /* Final round omits MixColumns. */
    aes_sub_shift(s);
    aes_add_round_key(s, ctx->rk + 160);
    memcpy(out, s, 16);
}

void floss_aes128(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]) {
    floss_aes128_t ctx;
    floss_aes128_init(&ctx, key);
    floss_aes128_encrypt(&ctx, in, out);
    memset(&ctx, 0, sizeof ctx); /* do not leave the key schedule on the stack */
}

/* ------------------------------------------------------------------------ */
/* AES-CMAC (RFC 4493)                                                      */
/* ------------------------------------------------------------------------ */

/** Left shift a 128-bit big-endian value by one bit, xoring in Rb (0x87) if the MSB fell off. */
static void cmac_double(const uint8_t in[16], uint8_t out[16]) {
    uint8_t carry = (uint8_t)(in[0] >> 7);
    for (unsigned i = 0; i < 15; i++) {
        out[i] = (uint8_t)((in[i] << 1) | (in[i + 1] >> 7));
    }
    out[15] = (uint8_t)((in[15] << 1) ^ (carry * 0x87));
}

void floss_aes_cmac(const uint8_t key[16], const uint8_t *msg, size_t len, uint8_t mac[16]) {
    floss_aes128_t ctx;
    uint8_t k[16];   /* L, then subkey K1 or K2 */
    uint8_t x[16];   /* CBC chaining value */

    floss_aes128_init(&ctx, key);

    /* RFC 4493 2.3 Generate_Subkey: L = AES(K, 0); K1 = dbl(L); K2 = dbl(K1). */
    memset(x, 0, 16);
    floss_aes128_encrypt(&ctx, x, k);
    cmac_double(k, k);

    /* All blocks but the last are plain CBC-MAC. A message whose length is a
     * non-zero multiple of 16 keeps its final full block for the K1 step. */
    size_t full = (len == 0) ? 0 : (len - 1) / 16; /* blocks before the last */
    for (size_t b = 0; b < full; b++) {
        for (unsigned i = 0; i < 16; i++) {
            x[i] ^= msg[16 * b + i];
        }
        floss_aes128_encrypt(&ctx, x, x);
    }

    size_t rem = len - 16 * full; /* 0..16 bytes in the last block */
    const uint8_t *last = msg ? msg + 16 * full : NULL;
    if (rem == 16) {
        /* Complete block: xor with K1. */
        for (unsigned i = 0; i < 16; i++) {
            x[i] ^= (uint8_t)(last[i] ^ k[i]);
        }
    } else {
        /* Incomplete (or empty) block: pad with 10*, xor with K2. */
        cmac_double(k, k);
        for (unsigned i = 0; i < 16; i++) {
            uint8_t m = (i < rem) ? last[i] : (i == rem ? 0x80 : 0x00);
            x[i] ^= (uint8_t)(m ^ k[i]);
        }
    }
    floss_aes128_encrypt(&ctx, x, mac);

    memset(&ctx, 0, sizeof ctx);
    memset(k, 0, sizeof k);
}

#else
typedef int floss_aes_unused_t; /* ISO C forbids an empty translation unit */
#endif /* FLOSS_ENABLE_LE */
