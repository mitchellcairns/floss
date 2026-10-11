/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_smp_crypto.h - LE Security Manager cryptographic toolbox
 * (Core Specification Vol 3 Part H, section 2.2): e, c1, s1, ah, f4, f5, f6, g2.
 *
 * Internal to the LE Security Manager; not part of the public API.
 *
 * ===========================================================================
 *  BYTE ORDER - read this before calling anything here
 *
 *  The specification defines these functions on big numbers written most
 *  significant octet first (MSB-first), and builds messages by concatenation
 *  "a || b" with a in the more significant position. SMP PDUs and HCI
 *  commands/events, however, carry every multi-octet value least significant
 *  octet first (LSB-first, "wire order").
 *
 *  EVERY parameter and result of this API is in WIRE ORDER (LSB-first):
 *  exactly the bytes found in, or sent in, an SMP PDU or HCI packet. The
 *  conversion to the specification's MSB-first form happens once, inside
 *  floss_smp_crypto.c. Protocol code therefore never reverses bytes.
 *
 *  Examples:
 *   - a Pairing Random PDU's 16 value bytes are passed as-is as a nonce;
 *   - the X coordinate of a public key (first 32 bytes of the Pairing Public
 *     Key PDU or of the HCI P-256 event) is passed as-is as U/V;
 *   - the HCI DHKey (32 bytes) is passed as-is as W of f5;
 *   - an LE address with its type ("A1/A2", 56 bits) is 7 bytes:
 *     the 6 address bytes in HCI order followed by the type octet
 *     (0x00 public, 0x01 random). Use floss_smp_addr7().
 *   - the specification's sample data (Appendix D), printed MSB-first,
 *     must be byte-reversed before being fed to these functions.
 * ===========================================================================
 */
#ifndef FLOSS_SMP_CRYPTO_H
#define FLOSS_SMP_CRYPTO_H

#include "floss/floss_types.h"

#if FLOSS_ENABLE_LE

#ifdef __cplusplus
extern "C" {
#endif

/** Build the 56-bit "address with type" operand used by f5/f6 (wire order). */
static inline void floss_smp_addr7(uint8_t out[7], uint8_t type, const floss_addr_t *addr) {
    memcpy(out, addr->b, 6);
    out[6] = type;
}

/** Security function e: AES-128 with key/plaintext/result as 128-bit numbers. */
void floss_smp_e(const uint8_t key[16], const uint8_t plaintext[16], uint8_t out[16]);

/**
 * Legacy confirm value generation c1.
 *  k     TK
 *  r     Mrand or Srand
 *  preq  the 7-byte Pairing Request PDU exactly as sent on the air (code first)
 *  pres  the 7-byte Pairing Response PDU exactly as sent on the air
 *  iat/ia, rat/ra  initiator / responder address type (0 public, 1 random) and address
 */
void floss_smp_c1(const uint8_t k[16], const uint8_t r[16], const uint8_t preq[7], const uint8_t pres[7],
                  uint8_t iat, const uint8_t ia[6], uint8_t rat, const uint8_t ra[6], uint8_t out[16]);

/** Legacy key generation s1: STK = s1(TK, Srand, Mrand). */
void floss_smp_s1(const uint8_t k[16], const uint8_t r1[16], const uint8_t r2[16], uint8_t out[16]);

/** Random address hash ah: 24-bit hash of the 24-bit prand (3 bytes, wire order). Result in bits 0..23. */
uint32_t floss_smp_ah(const uint8_t irk[16], const uint8_t prand[3]);

/** True if `rpa` (a resolvable private address, wire order) was generated from `irk`. */
bool floss_smp_rpa_matches(const uint8_t irk[16], const floss_addr_t *rpa);

/** Make a fresh resolvable private address from `irk` (uses floss_hal_random). */
void floss_smp_rpa_generate(const uint8_t irk[16], floss_addr_t *out);

/** LE Secure Connections confirm value f4(U, V, X, Z) = AES-CMAC_X(U || V || Z). */
void floss_smp_f4(const uint8_t u[32], const uint8_t v[32], const uint8_t x[16], uint8_t z, uint8_t out[16]);

/**
 * LE Secure Connections key generation f5(W, N1, N2, A1, A2).
 *  w       DHKey (32 bytes)
 *  n1, n2  Na, Nb
 *  a1, a2  initiator / responder address with type (floss_smp_addr7)
 * Produces MacKey and LTK.
 */
void floss_smp_f5(const uint8_t w[32], const uint8_t n1[16], const uint8_t n2[16], const uint8_t a1[7],
                  const uint8_t a2[7], uint8_t mackey[16], uint8_t ltk[16]);

/**
 * LE Secure Connections check value f6(W, N1, N2, R, IOcap, A1, A2).
 *  iocap   3 bytes in wire order = bytes 1..3 of the sender's Pairing
 *          Request/Response PDU (IO capability, OOB flag, AuthReq), i.e. the
 *          specification's "AuthReq || OOB || IOcap" read LSB-first.
 */
void floss_smp_f6(const uint8_t w[16], const uint8_t n1[16], const uint8_t n2[16], const uint8_t r[16],
                  const uint8_t iocap[3], const uint8_t a1[7], const uint8_t a2[7], uint8_t out[16]);

/** Numeric comparison value g2(U, V, X, Y) (32 bits; display it modulo 1 000 000). */
uint32_t floss_smp_g2(const uint8_t u[32], const uint8_t v[32], const uint8_t x[16], const uint8_t y[16]);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_ENABLE_LE */

#endif /* FLOSS_SMP_CRYPTO_H */
