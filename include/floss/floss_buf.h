/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_buf.h - Fixed-size packet buffer pool and FIFO queues.
 *
 * Zero-copy TX: a buffer is allocated with FLOSS_BUF_HEADROOM free bytes in
 * front of `data`. Each layer writes its payload, then lower layers
 * floss_buf_push() their headers in front:
 *
 *     mem: [ headroom .............. | payload ]
 *     HID:                       push 2 -> [HIDP hdr][id][payload]
 *     L2CAP:               push 4 -> [L2CAP hdr][HIDP]...
 *     HCI:          push 4 -> [ACL hdr][L2CAP]...
 *     transport: writes its own bytes into pkt[-headroom..-1] (no push)
 *
 * No heap. O(1) alloc/free. Not ISR safe: stack context only.
 */
#ifndef FLOSS_BUF_H
#define FLOSS_BUF_H

#include "floss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Buffer flags. */
#define FLOSS_BUF_F_FLUSHABLE 0x01 /**< ACL: may be flushed by the controller (automatic flush timeout) */
#define FLOSS_BUF_F_PRIORITY  0x02 /**< ACL: jump the per-link TX queue (latency critical, e.g. HID input) */

typedef struct floss_buf {
    struct floss_buf *next; /**< queue link (owned by whoever holds the buffer) */
    uint8_t *data;          /**< start of valid bytes */
    uint16_t len;           /**< number of valid bytes at data */
    uint16_t handle;        /**< ACL connection handle (TX) */
    uint8_t type;           /**< floss_pkt_type_t */
    uint8_t flags;          /**< FLOSS_BUF_F_* */
    uint16_t opcode;        /**< HCI command opcode (command queue) */
    uint32_t t_us;          /**< timestamp (enqueue time) for latency statistics */
    uint32_t mem[(FLOSS_BUF_HEADROOM + FLOSS_BUF_SIZE + 3) / 4]; /**< word aligned storage */
} floss_buf_t;

/** Initialise the pool. Called by floss_init(). */
void floss_buf_pool_init(void);

/** Take a buffer; data = mem + FLOSS_BUF_HEADROOM, len = 0. NULL if exhausted. */
floss_buf_t *floss_buf_alloc(void);

/** Return a buffer to the pool. NULL is ignored. */
void floss_buf_free(floss_buf_t *buf);

/** Number of free buffers right now (and lowest ever, for sizing). */
uint16_t floss_buf_free_count(void);
uint16_t floss_buf_low_watermark(void);

/** Capacity remaining after data+len. */
static inline uint16_t floss_buf_tailroom(const floss_buf_t *b) {
    const uint8_t *end = (const uint8_t *)b->mem + sizeof(b->mem);
    return (uint16_t)(end - (b->data + b->len));
}

/** Prepend n bytes (header) and return pointer to them. Asserts headroom. */
uint8_t *floss_buf_push(floss_buf_t *b, uint16_t n);

/** Remove n bytes from the front and return the old data pointer. */
uint8_t *floss_buf_pull(floss_buf_t *b, uint16_t n);

/** Append n bytes at the tail and return pointer to them (NULL if no room). */
uint8_t *floss_buf_put(floss_buf_t *b, uint16_t n);

/* ------------------------------------------------------------------------ */
/* Singly linked FIFO                                                       */
/* ------------------------------------------------------------------------ */

typedef struct {
    floss_buf_t *head;
    floss_buf_t *tail;
    uint16_t count;
} floss_bufq_t;

static inline void floss_bufq_init(floss_bufq_t *q) {
    q->head = q->tail = NULL;
    q->count = 0;
}
void floss_bufq_push_tail(floss_bufq_t *q, floss_buf_t *b);
void floss_bufq_push_head(floss_bufq_t *q, floss_buf_t *b);
floss_buf_t *floss_bufq_pop(floss_bufq_t *q);
/** Free every buffer in the queue. */
void floss_bufq_drain(floss_bufq_t *q);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_BUF_H */
