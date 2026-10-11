/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_types.h - Basic types, error codes and byte-order helpers shared by
 * every layer. Pure C99, no platform dependencies.
 */
#ifndef FLOSS_TYPES_H
#define FLOSS_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "floss_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLOSS_UNUSED(x) ((void)(x))
#define FLOSS_ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define FLOSS_MIN(a, b) ((a) < (b) ? (a) : (b))
#define FLOSS_MAX(a, b) ((a) > (b) ? (a) : (b))

/* ------------------------------------------------------------------------ */
/* Error codes                                                              */
/* ------------------------------------------------------------------------ */

/** Return type of every fallible FLOSS call. 0 = success, negative = error. */
typedef int floss_err_t;

enum {
    FLOSS_OK              = 0,
    FLOSS_ERR_NOMEM       = -1,  /**< buffer pool / table exhausted */
    FLOSS_ERR_BUSY        = -2,  /**< try again later (no credits, queue full) */
    FLOSS_ERR_INVALID     = -3,  /**< bad argument */
    FLOSS_ERR_STATE       = -4,  /**< not allowed in current state */
    FLOSS_ERR_NOT_FOUND   = -5,  /**< no such connection / channel / bond */
    FLOSS_ERR_TIMEOUT     = -6,
    FLOSS_ERR_IO          = -7,  /**< transport failure */
    FLOSS_ERR_UNSUPPORTED = -8,
    FLOSS_ERR_PROTOCOL    = -9,  /**< peer or controller sent something malformed */
};

/** Human readable name of an error code (never NULL). */
const char *floss_err_str(floss_err_t err);

/* ------------------------------------------------------------------------ */
/* Device address                                                           */
/* ------------------------------------------------------------------------ */

/**
 * 48-bit device address, stored in wire order (least significant byte first),
 * exactly as it appears in HCI packets. Printing shows it MSB-first.
 */
typedef struct {
    uint8_t b[6];
} floss_addr_t;

/** Format as "AA:BB:CC:DD:EE:FF" (MSB first). out must hold 18 bytes. */
void floss_addr_to_str(const floss_addr_t *addr, char out[18]);
/** Parse "AA:BB:CC:DD:EE:FF" ('-' separators also accepted). Returns false on malformed input. */
bool floss_addr_from_str(const char *str, floss_addr_t *out);

static inline bool floss_addr_eq(const floss_addr_t *a, const floss_addr_t *b) {
    return memcmp(a->b, b->b, 6) == 0;
}
static inline bool floss_addr_is_zero(const floss_addr_t *a) {
    static const uint8_t z[6] = {0};
    return memcmp(a->b, z, 6) == 0;
}

/** 128-bit link key plus its HCI key type. */
typedef struct {
    uint8_t key[16];
    uint8_t type;
} floss_link_key_t;

/* ------------------------------------------------------------------------ */
/* Time                                                                     */
/* ------------------------------------------------------------------------ */

/**
 * Microsecond timestamps are uint32_t and wrap every ~71.6 minutes.
 * Always compare with floss_time_diff(), never with < or >.
 */
static inline int32_t floss_time_diff(uint32_t a, uint32_t b) {
    return (int32_t)(a - b); /* > 0 when a is after b */
}

/* ------------------------------------------------------------------------ */
/* Little/big-endian helpers (wire formats are explicit, never cast structs) */
/* ------------------------------------------------------------------------ */

static inline uint16_t floss_get_le16(const uint8_t *p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}
static inline uint32_t floss_get_le24(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}
static inline uint32_t floss_get_le32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline uint16_t floss_get_be16(const uint8_t *p) {
    return (uint16_t)((p[0] << 8) | p[1]);
}
static inline uint32_t floss_get_be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}
static inline void floss_put_le16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}
static inline void floss_put_le24(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
}
static inline void floss_put_le32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}
static inline void floss_put_be16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}
static inline void floss_put_be32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_TYPES_H */
