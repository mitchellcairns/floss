/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_sdp_priv.h - Definitions shared by the SDP source files only.
 *
 * Overview
 * --------
 * SDP serialises everything as "data elements": a one-byte header holding a
 * 5-bit type descriptor and a 3-bit size index, optionally followed by an
 * explicit 1/2/4-byte length, followed by the payload. All multi-byte values
 * in SDP are big-endian (unlike HCI/L2CAP, which are little-endian).
 *
 *   header byte = (type << 3) | size_index
 *   size_index 0..4 -> payload is 1, 2, 4, 8, 16 bytes (no length field)
 *   size_index 5..7 -> payload length follows in 1, 2, 4 bytes
 *
 * The type values below are what floss_de_header() reports in *type.
 */
#ifndef FLOSS_SDP_PRIV_H
#define FLOSS_SDP_PRIV_H

#include "floss_internal.h"

/* Data element type descriptors (header byte >> 3). */
#define FLOSS_DE_TYPE_NIL  0
#define FLOSS_DE_TYPE_UINT 1
#define FLOSS_DE_TYPE_INT  2
#define FLOSS_DE_TYPE_UUID 3
#define FLOSS_DE_TYPE_TEXT 4
#define FLOSS_DE_TYPE_BOOL 5
#define FLOSS_DE_TYPE_SEQ  6
#define FLOSS_DE_TYPE_ALT  7
#define FLOSS_DE_TYPE_URL  8

/* Size indexes (header byte & 7). */
#define FLOSS_DE_SIZE_1     0
#define FLOSS_DE_SIZE_2     1
#define FLOSS_DE_SIZE_4     2
#define FLOSS_DE_SIZE_8     3
#define FLOSS_DE_SIZE_16    4
#define FLOSS_DE_SIZE_LEN8  5
#define FLOSS_DE_SIZE_LEN16 6
#define FLOSS_DE_SIZE_LEN32 7

/** Build a header byte from type and size index. */
#define FLOSS_DE_HDR(type, size) ((uint8_t)(((type) << 3) | (size)))

/** Header bytes the writer and the record builders rely on. */
#define FLOSS_DE_HDR_UINT16 FLOSS_DE_HDR(FLOSS_DE_TYPE_UINT, FLOSS_DE_SIZE_2) /* 0x09 */
#define FLOSS_DE_HDR_UINT32 FLOSS_DE_HDR(FLOSS_DE_TYPE_UINT, FLOSS_DE_SIZE_4) /* 0x0A */
#define FLOSS_DE_HDR_SEQ16  FLOSS_DE_HDR(FLOSS_DE_TYPE_SEQ, FLOSS_DE_SIZE_LEN16) /* 0x36 */

/** Nesting limit enforced when a record is validated at registration. */
#define FLOSS_SDP_MAX_NEST 8

#endif /* FLOSS_SDP_PRIV_H */
