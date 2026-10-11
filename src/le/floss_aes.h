/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_aes.h - AES-128 block encryption (FIPS-197) and AES-CMAC (RFC 4493).
 *
 * Internal to the LE Security Manager; not part of the public API.
 *
 * BYTE ORDER: everything here uses the natural "FIPS order" of the standards:
 * byte 0 of a key/block/MAC is the first byte written in FIPS-197 and
 * RFC 4493 hex strings, i.e. the MOST significant octet when the value is read
 * as a 128-bit number. The SMP toolbox (floss_smp_crypto.h) works in wire
 * order (least significant octet first) and does the conversion, so nothing
 * outside floss_smp_crypto.c needs to think about this.
 *
 * Only encryption is implemented: every security function the LE Security
 * Manager uses (e, c1, s1, ah, and CMAC for f4/f5/f6/g2) is built on the
 * forward cipher, so the inverse cipher would be dead code.
 */
#ifndef FLOSS_AES_H
#define FLOSS_AES_H

#include "floss/floss_types.h"

#if FLOSS_ENABLE_LE

#ifdef __cplusplus
extern "C" {
#endif

/** Expanded AES-128 key: 11 round keys of 16 bytes (FIPS-197 section 5.2). */
typedef struct {
    uint8_t rk[176];
} floss_aes128_t;

/** Expand `key` (16 bytes, FIPS order) into the round-key schedule. */
void floss_aes128_init(floss_aes128_t *ctx, const uint8_t key[16]);

/** Encrypt one 16-byte block (FIPS order). `in` and `out` may alias. */
void floss_aes128_encrypt(const floss_aes128_t *ctx, const uint8_t in[16], uint8_t out[16]);

/** One-shot convenience: out = AES-128(key, in). */
void floss_aes128(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

/**
 * AES-CMAC (RFC 4493 / NIST SP 800-38B) with a 128-bit key over `len` bytes.
 * `mac` receives the full 16-byte tag (FIPS order). `msg` may be NULL when
 * len is 0.
 */
void floss_aes_cmac(const uint8_t key[16], const uint8_t *msg, size_t len, uint8_t mac[16]);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_ENABLE_LE */

#endif /* FLOSS_AES_H */
