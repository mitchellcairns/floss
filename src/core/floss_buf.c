/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_buf.c - Static packet buffer pool with O(1) alloc/free, plus FIFOs.
 *
 * Overview
 * --------
 * Every packet the stack builds or reassembles lives in one of
 * FLOSS_BUF_COUNT fixed-size floss_buf_t. Free buffers are chained through
 * their own `next` field into a LIFO free list, so alloc and free are a
 * couple of pointer moves - cheap enough for the HID report hot path.
 * (LIFO also keeps recently used buffers warm in cache.)
 *
 * Each allocation starts with `data` FLOSS_BUF_HEADROOM bytes into `mem`, so
 * lower layers can floss_buf_push() their headers in front of the payload
 * without copying (see the diagram in floss_buf.h).
 *
 * Sizing aids: the pool remembers the lowest free count ever seen
 * (floss_buf_low_watermark) and counts allocation failures in the stats; if
 * the low watermark reaches 0 in normal operation, raise FLOSS_BUF_COUNT.
 *
 * Misuse detection: a per-buffer "is free" flag catches double frees and
 * frees of foreign pointers with FLOSS_ASSERT, which is far easier to debug
 * than the free-list corruption they would otherwise cause.
 */
#include "floss/floss_buf.h"
#include "floss/floss_debug.h"

static floss_buf_t s_pool[FLOSS_BUF_COUNT];
static floss_buf_t *s_free_list;
static uint16_t s_free_count;
static uint16_t s_low_watermark;
static bool s_is_free[FLOSS_BUF_COUNT];

void floss_buf_pool_init(void)
{
    s_free_list = NULL;
    /* Push in reverse so the first alloc returns s_pool[0] (easier to read in a debugger). */
    for (int i = FLOSS_BUF_COUNT - 1; i >= 0; i--) {
        s_pool[i].next = s_free_list;
        s_free_list = &s_pool[i];
        s_is_free[i] = true;
    }
    s_free_count = FLOSS_BUF_COUNT;
    s_low_watermark = FLOSS_BUF_COUNT;
}

floss_buf_t *floss_buf_alloc(void)
{
    floss_buf_t *b = s_free_list;

    if (b == NULL) {
        FLOSS_STAT_INC(buf_alloc_failures);
        return NULL;
    }
    s_free_list = b->next;
    s_is_free[b - s_pool] = false;
    s_free_count--;
    if (s_free_count < s_low_watermark) {
        s_low_watermark = s_free_count;
    }

    b->next = NULL;
    b->data = (uint8_t *)b->mem + FLOSS_BUF_HEADROOM;
    b->len = 0;
    b->handle = 0;
    b->type = 0;
    b->flags = 0;
    b->opcode = 0;
    b->t_us = 0;
    return b;
}

void floss_buf_free(floss_buf_t *buf)
{
    if (buf == NULL) {
        return;
    }
    FLOSS_ASSERT(buf >= s_pool && buf < s_pool + FLOSS_BUF_COUNT);
    FLOSS_ASSERT(!s_is_free[buf - s_pool]); /* double free */

    s_is_free[buf - s_pool] = true;
    buf->next = s_free_list;
    s_free_list = buf;
    s_free_count++;
}

uint16_t floss_buf_free_count(void)
{
    return s_free_count;
}

uint16_t floss_buf_low_watermark(void)
{
    return s_low_watermark;
}

uint8_t *floss_buf_push(floss_buf_t *b, uint16_t n)
{
    /* Running out of headroom means a layer pushed a header nobody sized
     * FLOSS_BUF_HEADROOM for: a programming error, not a runtime condition. */
    FLOSS_ASSERT((size_t)(b->data - (uint8_t *)b->mem) >= n);
    b->data -= n;
    b->len = (uint16_t)(b->len + n);
    return b->data;
}

uint8_t *floss_buf_pull(floss_buf_t *b, uint16_t n)
{
    uint8_t *old = b->data;

    FLOSS_ASSERT(n <= b->len);
    b->data += n;
    b->len = (uint16_t)(b->len - n);
    return old;
}

uint8_t *floss_buf_put(floss_buf_t *b, uint16_t n)
{
    uint8_t *tail;

    /* Unlike push/pull this can legitimately fail on peer-controlled sizes
     * (e.g. reassembly), so report it instead of asserting. */
    if (floss_buf_tailroom(b) < n) {
        return NULL;
    }
    tail = b->data + b->len;
    b->len = (uint16_t)(b->len + n);
    return tail;
}

/* ------------------------------------------------------------------------ */
/* FIFO                                                                     */
/* ------------------------------------------------------------------------ */

void floss_bufq_push_tail(floss_bufq_t *q, floss_buf_t *b)
{
    FLOSS_ASSERT(b != NULL);
    b->next = NULL;
    if (q->tail != NULL) {
        q->tail->next = b;
    } else {
        q->head = b;
    }
    q->tail = b;
    q->count++;
}

void floss_bufq_push_head(floss_bufq_t *q, floss_buf_t *b)
{
    FLOSS_ASSERT(b != NULL);
    b->next = q->head;
    q->head = b;
    if (q->tail == NULL) {
        q->tail = b;
    }
    q->count++;
}

floss_buf_t *floss_bufq_pop(floss_bufq_t *q)
{
    floss_buf_t *b = q->head;

    if (b == NULL) {
        return NULL;
    }
    q->head = b->next;
    if (q->head == NULL) {
        q->tail = NULL;
    }
    q->count--;
    b->next = NULL;
    return b;
}

void floss_bufq_drain(floss_bufq_t *q)
{
    floss_buf_t *b;

    while ((b = floss_bufq_pop(q)) != NULL) {
        floss_buf_free(b);
    }
}
