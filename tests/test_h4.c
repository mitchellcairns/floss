/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * test_h4.c - H4 transport over a fake in-memory byte stream.
 *
 * A UART or USB-CDC link hands the driver bytes in arbitrary chunks, may
 * accept only part of a write, and occasionally delivers garbage (noise at
 * power-up, a lost byte). The fake stream reproduces all three
 * deterministically: reads return 1..7 bytes chosen by a seeded LCG, writes
 * can be capped per call, and tests inject junk between packets.
 */
#include "floss/floss.h"
#include "floss_h4.h"
#include "floss_test.h"

/* ------------------------------------------------------------------------ */
/* Fake stream                                                              */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint8_t rx[4096];
    size_t rx_len;
    size_t rx_pos;
    uint32_t lcg;
    uint8_t tx[4096];
    size_t tx_len;
    uint16_t tx_chunk; /* max bytes accepted per write call (0 = all) */
} fake_stream_t;

static fake_stream_t g_fs;
static floss_stream_t g_stream;
static floss_h4_t g_h4;

static int fs_write(void *ctx, const uint8_t *data, uint16_t len) {
    fake_stream_t *fs = ctx;
    uint16_t n = len;
    if (fs->tx_chunk && n > fs->tx_chunk) {
        n = fs->tx_chunk;
    }
    if (fs->tx_len + n > sizeof fs->tx) {
        n = (uint16_t)(sizeof fs->tx - fs->tx_len);
    }
    memcpy(fs->tx + fs->tx_len, data, n);
    fs->tx_len += n;
    return n;
}

static int fs_read(void *ctx, uint8_t *buf, uint16_t max) {
    fake_stream_t *fs = ctx;
    if (fs->rx_pos >= fs->rx_len) {
        return 0;
    }
    fs->lcg = fs->lcg * 1103515245u + 12345u;
    size_t n = 1 + (fs->lcg >> 16) % 7; /* 1..7 bytes per read */
    if (n > max) {
        n = max;
    }
    if (n > fs->rx_len - fs->rx_pos) {
        n = fs->rx_len - fs->rx_pos;
    }
    memcpy(buf, fs->rx + fs->rx_pos, n);
    fs->rx_pos += n;
    return (int)n;
}

static void fs_feed(const uint8_t *data, size_t len) {
    ASSERT_LE(g_fs.rx_len + len, sizeof g_fs.rx);
    memcpy(g_fs.rx + g_fs.rx_len, data, len);
    g_fs.rx_len += len;
}

/* ------------------------------------------------------------------------ */
/* Packet collector                                                         */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint8_t type;
    uint16_t len;
    uint8_t data[4 + FLOSS_BUF_SIZE];
} rx_pkt_t;

static rx_pkt_t g_rx[16];
static unsigned g_rx_n;

static void on_rx(void *user, uint8_t type, const uint8_t *pkt, uint16_t len) {
    (void)user;
    if (g_rx_n < FLOSS_ARRAY_SIZE(g_rx)) {
        g_rx[g_rx_n].type = type;
        g_rx[g_rx_n].len = len;
        memcpy(g_rx[g_rx_n].data, pkt, FLOSS_MIN(len, sizeof g_rx[0].data));
    }
    g_rx_n++;
}

static floss_transport_t *h4_setup(uint32_t seed) {
    memset(&g_fs, 0, sizeof g_fs);
    memset(&g_h4, 0, sizeof g_h4);
    g_fs.lcg = seed;
    g_rx_n = 0;
    g_stream.ctx = &g_fs;
    g_stream.write = fs_write;
    g_stream.read = fs_read;
    floss_transport_t *t = floss_h4_init(&g_h4, &g_stream);
    ASSERT_NOT_NULL(t);
    ASSERT_NOT_NULL(t->ops);
    t->rx = on_rx;
    t->rx_user = NULL;
    if (t->ops->open) {
        ASSERT_EQ(t->ops->open(t), FLOSS_OK);
    }
    return t;
}

/** Poll until the stream is drained (plus a few idle polls). */
static void h4_drain(floss_transport_t *t) {
    unsigned idle = 0;
    for (unsigned i = 0; i < 100000 && idle < 8; i++) {
        size_t before = g_fs.rx_pos;
        t->ops->poll(t);
        idle = (g_fs.rx_pos == before && g_fs.rx_pos >= g_fs.rx_len) ? idle + 1 : 0;
    }
}

/* Packets used by the RX tests (H4 type byte included). */
static const uint8_t pkt_cc[] = {0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00};
static const uint8_t pkt_acl_small[] = {0x02, 0x40, 0x20, 0x08, 0x00, 0x04, 0x00, 0x40, 0x00, 0xA1, 0x01, 0x02, 0x03};
static const uint8_t pkt_nocp[] = {0x04, 0x13, 0x05, 0x01, 0x40, 0x00, 0x01, 0x00};

static size_t build_big_event(uint8_t *out) {
    out[0] = 0x04;
    out[1] = 0xFF; /* vendor event */
    out[2] = 255;  /* maximum parameter length */
    for (unsigned i = 0; i < 255; i++) {
        out[3 + i] = (uint8_t)(i * 7);
    }
    return 3 + 255;
}

static size_t build_big_acl(uint8_t *out, uint16_t payload) {
    out[0] = 0x02;
    floss_put_le16(out + 1, 0x2041); /* handle 0x041, PB = first flushable */
    floss_put_le16(out + 3, payload);
    for (unsigned i = 0; i < payload; i++) {
        out[5 + i] = (uint8_t)(i ^ 0x5A);
    }
    return 5 + (size_t)payload;
}

static void assert_pkt(unsigned idx, const uint8_t *h4, size_t h4_len) {
    ASSERT_LT(idx, g_rx_n);
    ASSERT_EQ(g_rx[idx].type, h4[0]);
    ASSERT_EQ(g_rx[idx].len, h4_len - 1);
    ASSERT_MEM_EQ(g_rx[idx].data, h4 + 1, h4_len - 1);
}

/* ------------------------------------------------------------------------ */
/* Tests                                                                    */
/* ------------------------------------------------------------------------ */

TEST(h4_tx_framing) {
    floss_transport_t *t = h4_setup(1);
    /* The type byte goes into the driver's headroom in front of the packet. */
    ASSERT_GE(t->ops->headroom, 1);
    ASSERT_LE(t->ops->headroom, FLOSS_TRANSPORT_MAX_HEADROOM);

    static uint8_t frame[FLOSS_TRANSPORT_MAX_HEADROOM + 300];
    uint8_t *pkt = frame + FLOSS_TRANSPORT_MAX_HEADROOM;

    const uint8_t reset[] = {0x03, 0x0C, 0x00};
    memcpy(pkt, reset, sizeof reset);
    ASSERT_EQ(t->ops->send(t, FLOSS_PKT_CMD, pkt, sizeof reset), FLOSS_OK);
    const uint8_t want_cmd[] = {0x01, 0x03, 0x0C, 0x00};
    ASSERT_EQ(g_fs.tx_len, sizeof want_cmd);
    ASSERT_MEM_EQ(g_fs.tx, want_cmd, sizeof want_cmd);

    const uint8_t acl[] = {0x40, 0x20, 0x05, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
    memcpy(pkt, acl, sizeof acl);
    ASSERT_EQ(t->ops->send(t, FLOSS_PKT_ACL, pkt, sizeof acl), FLOSS_OK);
    ASSERT_EQ(g_fs.tx_len, sizeof want_cmd + 1 + sizeof acl);
    ASSERT_EQ(g_fs.tx[sizeof want_cmd], 0x02);
    ASSERT_MEM_EQ(g_fs.tx + sizeof want_cmd + 1, acl, sizeof acl);
}

TEST(h4_tx_partial_writes) {
    floss_transport_t *t = h4_setup(2);
    static uint8_t frame[FLOSS_TRANSPORT_MAX_HEADROOM + 300];
    uint8_t *pkt = frame + FLOSS_TRANSPORT_MAX_HEADROOM;
    static uint8_t expect[1 + 204];

    /* The stream takes at most 3 bytes per call: the frame must still go out whole. */
    g_fs.tx_chunk = 3;
    floss_put_le16(pkt, 0x0040);
    floss_put_le16(pkt + 2, 200);
    for (unsigned i = 0; i < 200; i++) {
        pkt[4 + i] = (uint8_t)i;
    }
    expect[0] = 0x02;
    memcpy(expect + 1, pkt, 204);
    ASSERT_EQ(t->ops->send(t, FLOSS_PKT_ACL, pkt, 204), FLOSS_OK);
    ASSERT_EQ(g_fs.tx_len, sizeof expect);
    ASSERT_MEM_EQ(g_fs.tx, expect, sizeof expect);
}

TEST(h4_rx_random_chunks) {
    static uint8_t big_evt[300];
    static uint8_t big_acl[8 + FLOSS_BUF_SIZE];
    size_t big_evt_len = build_big_event(big_evt);
    size_t big_acl_len = build_big_acl(big_acl, FLOSS_BUF_SIZE - 4);

    /* Several seeds: chunk boundaries fall at different places each time. */
    for (uint32_t seed = 1; seed <= 6; seed++) {
        floss_transport_t *t = h4_setup(seed * 2654435761u);
        fs_feed(pkt_cc, sizeof pkt_cc);
        fs_feed(pkt_acl_small, sizeof pkt_acl_small);
        fs_feed(big_evt, big_evt_len);
        fs_feed(big_acl, big_acl_len);
        fs_feed(pkt_nocp, sizeof pkt_nocp);
        h4_drain(t);

        ASSERT_EQ(g_rx_n, 5);
        assert_pkt(0, pkt_cc, sizeof pkt_cc);
        assert_pkt(1, pkt_acl_small, sizeof pkt_acl_small);
        assert_pkt(2, big_evt, big_evt_len);
        assert_pkt(3, big_acl, big_acl_len);
        assert_pkt(4, pkt_nocp, sizeof pkt_nocp);
    }
}

TEST(h4_rx_resync_after_garbage) {
    floss_transport_t *t = h4_setup(7);
    const uint8_t junk[] = {0x00, 0xFF, 0x7E};
    fs_feed(junk, sizeof junk);
    fs_feed(pkt_cc, sizeof pkt_cc);
    fs_feed(junk, 1);
    fs_feed(pkt_nocp, sizeof pkt_nocp);
    h4_drain(t);

    ASSERT_EQ(g_rx_n, 2);
    assert_pkt(0, pkt_cc, sizeof pkt_cc);
    assert_pkt(1, pkt_nocp, sizeof pkt_nocp);
    ASSERT_GE(g_h4.resyncs, 1);
}

TEST(h4_rx_oversized_dropped) {
    floss_transport_t *t = h4_setup(9);
    /* ACL header announcing 1000 bytes: more than any pool buffer can hold. */
    static uint8_t huge[5 + 1000];
    huge[0] = 0x02;
    floss_put_le16(huge + 1, 0x2040);
    floss_put_le16(huge + 3, 1000);
    memset(huge + 5, 0x00, 1000);
    fs_feed(huge, sizeof huge);
    fs_feed(pkt_cc, sizeof pkt_cc);
    h4_drain(t);

    ASSERT_EQ(g_rx_n, 1); /* the oversized packet never reaches the stack */
    assert_pkt(0, pkt_cc, sizeof pkt_cc);
}

static const floss_test_case_t tests[] = {
    TEST_CASE(h4_tx_framing),
    TEST_CASE(h4_tx_partial_writes),
    TEST_CASE(h4_rx_random_chunks),
    TEST_CASE(h4_rx_resync_after_garbage),
    TEST_CASE(h4_rx_oversized_dropped),
};

FLOSS_TEST_MAIN("h4", tests)
