/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * test_sdp_client.c - SDP client: queries against the sim's scripted SDP
 * server, and the result walker on hand-made (and broken) data.
 *
 * The scenario behind the client: a device pages a host and opens RFCOMM
 * to an application-defined service whose server channel the host assigns
 * at run time and publishes in SDP. The device looks the channel up first:
 *
 *   floss_sdp_query_rfcomm_channel(handle, service_uuid128, cb, user)
 *     -> L2CAP PSM 0x0001 -> ServiceSearchAttribute(uuid, 0x0004..0x0004)
 *     -> continuation rounds -> ProtocolDescriptorList -> RFCOMM channel
 *
 * The sim serves canned AttributeLists bytes (floss_sim_sdp_serve()), so
 * each test decides exactly what the "host" publishes and how it is split.
 */
#include "test_common.h"

#include <stdlib.h>

#include "floss_internal.h"

/* Application service the device looks for (any 128-bit UUID, big-endian). */
static const uint8_t k_svc_uuid[16] = {0x6E, 0x40, 0x00, 0x01, 0xB5, 0xA3, 0xF3, 0x93,
                                       0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E};

/* RFCOMM protocol UUID 0x0003 in its 128-bit form (base UUID). */
static const uint8_t k_rfcomm_uuid128[16] = {0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x10, 0x00,
                                             0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};

/** What the callbacks saw. */
static struct {
    unsigned calls;
    uint16_t handle;
    int status;
    uint8_t channel;
    uint16_t len;
    uint8_t data[FLOSS_SDP_CLIENT_BUF];
    bool chain;           /**< start another rfcomm query from the callback */
    floss_err_t chain_rc;
    bool chain_busy; /* floss_sdp_query_busy() right after the chained start */
} g_res;

static void on_attr(uint16_t handle, int status, const uint8_t *lists, uint16_t len, void *user) {
    g_res.calls++;
    g_res.handle = handle;
    g_res.status = status;
    g_res.len = len;
    if (lists != NULL && len <= sizeof g_res.data) {
        memcpy(g_res.data, lists, len);
    }
    if (user != NULL) {
        (*(unsigned *)user)++;
    }
}

static void on_rfcomm(uint16_t handle, int status, uint8_t channel, void *user) {
    g_res.calls++;
    g_res.handle = handle;
    g_res.status = status;
    g_res.channel = channel;
    if (user != NULL) {
        (*(unsigned *)user)++;
    }
    if (g_res.chain) {
        g_res.chain = false;
        g_res.chain_rc = floss_sdp_query_rfcomm_channel(handle, k_svc_uuid, on_rfcomm, NULL);
        g_res.chain_busy = floss_sdp_query_busy();
    }
}

static bool cond_calls(void *arg) {
    return g_res.calls >= (unsigned)(uintptr_t)arg;
}

static void wait_calls(unsigned n) {
    if (!floss_sim_pump_until(cond_calls, (void *)(uintptr_t)n, 2000)) {
        FAIL("SDP client callback did not fire (calls %u, wanted %u)", g_res.calls, n);
    }
}

/** Stack without HID, connectable; the host connects (no pairing: SDP needs none). */
static uint16_t sdpc_setup(void) {
    memset(&g_res, 0, sizeof g_res);
    floss_transport_t *t = floss_tc_sim_init(NULL);
    floss_tc_stack_start(t, NULL);
    return floss_tc_host_connect();
}

/** The stack's SDP channel is gone on both sides. */
static void assert_channel_closed(uint16_t h) {
    floss_sim_pump(20);
    ASSERT_NULL(floss_l2cap_find(h, FLOSS_PSM_SDP));
    floss_sim_chan_t *ch = floss_sim_chan_find(h, FLOSS_PSM_SDP);
    ASSERT_NOT_NULL(ch);
    ASSERT_EQ(ch->state, FLOSS_SIM_CHAN_CLOSED);
}

/* ------------------------------------------------------------------------ */
/* Canned records                                                           */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint8_t rfcomm_form;  /**< 0 = no RFCOMM descriptor, else 2 / 4 / 16 byte UUID */
    uint8_t channel;
    uint16_t name_len;    /**< > 0 adds a ServiceName of that many bytes (padding) */
} rec_spec_t;

static void write_record(floss_de_writer_t *w, uint32_t handle, const rec_spec_t *r) {
    static const uint8_t name[256] = {'S', 'e', 'r', 'v', 'i', 'c', 'e'};
    floss_de_seq_begin(w);
    floss_de_uint16(w, FLOSS_ATTR_SERVICE_RECORD_HANDLE);
    floss_de_uint32(w, handle);
    floss_de_uint16(w, FLOSS_ATTR_SERVICE_CLASS_ID_LIST);
    floss_de_seq_begin(w);
    floss_de_uuid128(w, k_svc_uuid);
    floss_de_seq_end(w);
    floss_de_uint16(w, FLOSS_ATTR_PROTOCOL_DESCRIPTOR_LIST);
    floss_de_seq_begin(w);
    floss_de_seq_begin(w);
    floss_de_uuid16(w, FLOSS_UUID_L2CAP);
    floss_de_seq_end(w);
    if (r->rfcomm_form) {
        floss_de_seq_begin(w);
        if (r->rfcomm_form == 2) {
            floss_de_uuid16(w, FLOSS_UUID_RFCOMM);
        } else if (r->rfcomm_form == 4) {
            floss_de_uuid32(w, FLOSS_UUID_RFCOMM);
        } else {
            floss_de_uuid128(w, k_rfcomm_uuid128);
        }
        floss_de_uint8(w, r->channel);
        floss_de_seq_end(w);
    }
    floss_de_seq_end(w);
    if (r->name_len) {
        floss_de_uint16(w, FLOSS_ATTR_SERVICE_NAME);
        floss_de_text(w, name, r->name_len);
    }
    floss_de_seq_end(w);
}

/** AttributeLists with n records. Returns the length. */
static uint16_t build_lists(uint8_t *buf, uint16_t cap, const rec_spec_t *recs, unsigned n) {
    floss_de_writer_t w;
    floss_de_init(&w, buf, cap);
    floss_de_seq_begin(&w);
    for (unsigned i = 0; i < n; i++) {
        write_record(&w, 0x00010000u + i, &recs[i]);
    }
    floss_de_seq_end(&w);
    ASSERT_FALSE(w.error);
    return w.len;
}

/* ------------------------------------------------------------------------ */
/* Queries                                                                  */
/* ------------------------------------------------------------------------ */

/* The common case, with the RFCOMM descriptor in every UUID width a host may use. */
TEST(sdpc_rfcomm_channel_found_every_uuid_form) {
    uint16_t h = sdpc_setup();
    static const uint8_t forms[3] = {2, 4, 16};
    for (unsigned i = 0; i < 3; i++) {
        uint8_t lists[128];
        rec_spec_t rec = {.rfcomm_form = forms[i], .channel = (uint8_t)(5 + i)};
        uint16_t n = build_lists(lists, sizeof lists, &rec, 1);
        ASSERT_EQ(floss_sim_sdp_serve(lists, n, 0), FLOSS_OK);

        unsigned calls = 0;
        ASSERT_EQ(floss_sdp_query_rfcomm_channel(h, k_svc_uuid, on_rfcomm, &calls), FLOSS_OK);
        ASSERT_TRUE(floss_sdp_query_busy());
        wait_calls(i + 1);
        ASSERT_EQ(calls, 1);
        ASSERT_EQ(g_res.handle, h);
        ASSERT_EQ(g_res.status, 0);
        ASSERT_EQ(g_res.channel, 5 + i);
        ASSERT_FALSE(floss_sdp_query_busy());

        /* The request: the service UUID in 128-bit form, only attribute 0x0004,
         * and a byte limit that lets a response fill (not exceed) our MTU. */
        const floss_sim_sdp_server_stats_t *st = floss_sim_sdp_server_stats();
        ASSERT_EQ(st->requests, 1);
        ASSERT_EQ(st->responses, 1);
        ASSERT_EQ(st->bad_requests, 0);
        ASSERT_EQ(st->last_uuid_len, 16);
        ASSERT_MEM_EQ(st->last_uuid, k_svc_uuid, 16);
        ASSERT_EQ(st->last_range, 0x00040004u);
        ASSERT_EQ(st->last_max_bytes, FLOSS_L2CAP_MTU - 24);
        ASSERT_EQ(st->last_cont_len, 0);
        assert_channel_closed(h);
    }
    floss_tc_assert_sim_clean();
}

/* Several records match; only the second publishes RFCOMM. */
TEST(sdpc_rfcomm_channel_from_second_record) {
    uint16_t h = sdpc_setup();
    uint8_t lists[256];
    rec_spec_t recs[2] = {{.rfcomm_form = 0}, {.rfcomm_form = 16, .channel = 30}};
    uint16_t n = build_lists(lists, sizeof lists, recs, 2);
    ASSERT_EQ(floss_sim_sdp_serve(lists, n, 0), FLOSS_OK);
    ASSERT_EQ(floss_sdp_query_rfcomm_channel(h, k_svc_uuid, on_rfcomm, NULL), FLOSS_OK);
    wait_calls(1);
    ASSERT_EQ(g_res.status, 0);
    ASSERT_EQ(g_res.channel, 30);
    floss_tc_assert_sim_clean();
}

/* A long answer split into many small responses is reassembled byte-exact. */
TEST(sdpc_continuation_reassembles_many_responses) {
    uint16_t h = sdpc_setup();
    uint8_t lists[400];
    rec_spec_t recs[2] = {{.rfcomm_form = 2, .channel = 3, .name_len = 120},
                          {.rfcomm_form = 16, .channel = 4, .name_len = 90}};
    uint16_t n = build_lists(lists, sizeof lists, recs, 2);
    ASSERT_GT(n, 250);
    ASSERT_EQ(floss_sim_sdp_serve(lists, n, 40), FLOSS_OK);

    const uint8_t uuid16[2] = {0x11, 0x01};
    ASSERT_EQ(floss_sdp_query_ssa(h, uuid16, sizeof uuid16, 0x0000, 0xFFFF, on_attr, NULL), FLOSS_OK);
    wait_calls(1);
    ASSERT_EQ(g_res.status, 0);
    ASSERT_EQ(g_res.len, n);
    ASSERT_MEM_EQ(g_res.data, lists, n);

    const floss_sim_sdp_server_stats_t *st = floss_sim_sdp_server_stats();
    ASSERT_EQ(st->responses, (n + 39) / 40);
    ASSERT_GE(st->responses, 3);
    ASSERT_EQ(st->bad_requests, 0);
    ASSERT_EQ(st->last_uuid_len, 2);
    ASSERT_MEM_EQ(st->last_uuid, uuid16, 2);
    ASSERT_EQ(st->last_range, 0x0000FFFFu);
    ASSERT_EQ(st->last_cont_len, 2); /* the last request carried the server's token */

    /* The walker finds the first record's channel in the reassembled result. */
    uint8_t ch = 0;
    ASSERT_EQ(floss_sdp_rfcomm_channel(g_res.data, g_res.len, &ch), FLOSS_OK);
    ASSERT_EQ(ch, 3);
    assert_channel_closed(h);
    floss_tc_assert_sim_clean();
}

/* The host has no such service: an empty AttributeLists sequence. */
TEST(sdpc_no_matching_record) {
    uint16_t h = sdpc_setup();
    const uint8_t empty[2] = {0x35, 0x00};
    ASSERT_EQ(floss_sim_sdp_serve(empty, sizeof empty, 0), FLOSS_OK);

    ASSERT_EQ(floss_sdp_query_ssa(h, k_svc_uuid, 16, 0x0004, 0x0004, on_attr, NULL), FLOSS_OK);
    wait_calls(1);
    ASSERT_EQ(g_res.status, 0);
    ASSERT_EQ(g_res.len, 2);

    ASSERT_EQ(floss_sdp_query_rfcomm_channel(h, k_svc_uuid, on_rfcomm, NULL), FLOSS_OK);
    wait_calls(2);
    ASSERT_EQ(g_res.status, FLOSS_ERR_NOT_FOUND);
    ASSERT_EQ(g_res.channel, 0);

    /* A record without an RFCOMM descriptor is "not found" too. */
    uint8_t lists[128];
    rec_spec_t rec = {.rfcomm_form = 0};
    uint16_t n = build_lists(lists, sizeof lists, &rec, 1);
    ASSERT_EQ(floss_sim_sdp_serve(lists, n, 0), FLOSS_OK);
    ASSERT_EQ(floss_sdp_query_rfcomm_channel(h, k_svc_uuid, on_rfcomm, NULL), FLOSS_OK);
    wait_calls(3);
    ASSERT_EQ(g_res.status, FLOSS_ERR_NOT_FOUND);
    floss_tc_assert_sim_clean();
}

/* An Error Response ends the query with its ErrorCode. */
TEST(sdpc_error_response) {
    uint16_t h = sdpc_setup();
    const uint8_t empty[2] = {0x35, 0x00};
    ASSERT_EQ(floss_sim_sdp_serve(empty, sizeof empty, 0), FLOSS_OK);
    floss_sim_sdp_serve_error(FLOSS_SDP_ERR_INVALID_SYNTAX);
    ASSERT_EQ(floss_sdp_query_rfcomm_channel(h, k_svc_uuid, on_rfcomm, NULL), FLOSS_OK);
    wait_calls(1);
    ASSERT_EQ(g_res.status, FLOSS_SDP_ERR_INVALID_SYNTAX);
    ASSERT_EQ(g_res.channel, 0);
    ASSERT_EQ(floss_sim_sdp_server_stats()->errors_sent, 1);
    assert_channel_closed(h);
    floss_tc_assert_sim_clean();
}

/* Broken data elements end in an error, never in a crash or a bogus channel. */
TEST(sdpc_malformed_results) {
    uint16_t h = sdpc_setup();

    /* Record whose ProtocolDescriptorList claims 9 bytes it does not have. */
    const uint8_t bad_inner[] = {0x35, 0x07, 0x35, 0x05, 0x09, 0x00, 0x04, 0x35, 0x09};
    ASSERT_EQ(floss_sim_sdp_serve(bad_inner, sizeof bad_inner, 0), FLOSS_OK);
    ASSERT_EQ(floss_sdp_query_rfcomm_channel(h, k_svc_uuid, on_rfcomm, NULL), FLOSS_OK);
    wait_calls(1);
    ASSERT_EQ(g_res.status, FLOSS_ERR_PROTOCOL);
    ASSERT_EQ(g_res.channel, 0);

    /* Top level is not one well-formed sequence. */
    const uint8_t bad_top[] = {0x35, 0x10, 0x01};
    ASSERT_EQ(floss_sim_sdp_serve(bad_top, sizeof bad_top, 0), FLOSS_OK);
    ASSERT_EQ(floss_sdp_query_ssa(h, k_svc_uuid, 16, 0, 0xFFFF, on_attr, NULL), FLOSS_OK);
    wait_calls(2);
    ASSERT_EQ(g_res.status, FLOSS_ERR_PROTOCOL);
    ASSERT_EQ(g_res.len, 0);

    /* RFCOMM channel 0 (out of range) is a protocol error, not channel 0. */
    uint8_t lists[128];
    rec_spec_t rec = {.rfcomm_form = 2, .channel = 0};
    uint16_t n = build_lists(lists, sizeof lists, &rec, 1);
    ASSERT_EQ(floss_sim_sdp_serve(lists, n, 0), FLOSS_OK);
    ASSERT_EQ(floss_sdp_query_rfcomm_channel(h, k_svc_uuid, on_rfcomm, NULL), FLOSS_OK);
    wait_calls(3);
    ASSERT_EQ(g_res.status, FLOSS_ERR_PROTOCOL);
    ASSERT_EQ(g_res.channel, 0);
    floss_tc_assert_sim_clean();
}

/* An answer larger than FLOSS_SDP_CLIENT_BUF fails instead of being truncated. */
TEST(sdpc_result_overflow) {
    uint16_t h = sdpc_setup();
    static uint8_t big[FLOSS_SDP_CLIENT_BUF + 64];
    uint16_t body = (uint16_t)(sizeof big - 3);
    big[0] = 0x36; /* sequence, 16-bit length */
    floss_put_be16(big + 1, body);
    big[3] = 0x26; /* text, 16-bit length */
    floss_put_be16(big + 4, (uint16_t)(body - 3));
    ASSERT_EQ(floss_sim_sdp_serve(big, sizeof big, 0), FLOSS_OK);
    ASSERT_EQ(floss_sdp_query_ssa(h, k_svc_uuid, 16, 0, 0xFFFF, on_attr, NULL), FLOSS_OK);
    wait_calls(1);
    ASSERT_EQ(g_res.status, FLOSS_ERR_NOMEM);
    assert_channel_closed(h);
    floss_tc_assert_sim_clean();
}

/* A host that never answers: one TIMEOUT callback, channel closed. */
TEST(sdpc_timeout) {
    uint16_t h = sdpc_setup();
    const uint8_t empty[2] = {0x35, 0x00};
    ASSERT_EQ(floss_sim_sdp_serve(empty, sizeof empty, 0), FLOSS_OK);
    floss_sim_sdp_serve_silence(true);
    unsigned calls = 0;
    ASSERT_EQ(floss_sdp_query_rfcomm_channel(h, k_svc_uuid, on_rfcomm, &calls), FLOSS_OK);
    floss_sim_pump(50);
    ASSERT_EQ(floss_sim_sdp_server_stats()->requests, 1);
    ASSERT_EQ(calls, 0);

    floss_sim_cfg()->pump_step_us = 10000;
    floss_sim_pump_us(FLOSS_SDP_CLIENT_TIMEOUT_MS * 1000u - 200000u);
    ASSERT_EQ(calls, 0); /* not early */
    floss_sim_pump_us(400000u);
    ASSERT_EQ(calls, 1);
    ASSERT_EQ(g_res.status, FLOSS_ERR_TIMEOUT);
    ASSERT_FALSE(floss_sdp_query_busy());
    assert_channel_closed(h);
    floss_sim_pump_us(FLOSS_SDP_CLIENT_TIMEOUT_MS * 1000u);
    ASSERT_EQ(calls, 1);
    floss_tc_assert_sim_clean();
}

/* The link drops mid-query: exactly one IO callback, and nothing later. */
TEST(sdpc_link_loss_mid_query) {
    uint16_t h = sdpc_setup();
    const uint8_t empty[2] = {0x35, 0x00};
    ASSERT_EQ(floss_sim_sdp_serve(empty, sizeof empty, 0), FLOSS_OK);
    floss_sim_sdp_serve_silence(true);
    unsigned calls = 0;
    ASSERT_EQ(floss_sdp_query_ssa(h, k_svc_uuid, 16, 4, 4, on_attr, &calls), FLOSS_OK);
    floss_sim_pump(50);
    ASSERT_EQ(floss_sim_sdp_server_stats()->requests, 1);

    ASSERT_EQ(floss_sim_remote_disconnect(h, FLOSS_HCI_ERR_REMOTE_USER_TERMINATED), FLOSS_OK);
    floss_sim_pump(50);
    ASSERT_EQ(calls, 1);
    ASSERT_EQ(g_res.status, FLOSS_ERR_IO);
    ASSERT_EQ(g_res.len, 0);
    ASSERT_FALSE(floss_sdp_query_busy());

    floss_sim_cfg()->pump_step_us = 10000;
    floss_sim_pump_us(FLOSS_SDP_CLIENT_TIMEOUT_MS * 1000u + 1000000u);
    ASSERT_EQ(calls, 1); /* the timer died with the query */
    floss_tc_assert_sim_clean();
}

/* One query at a time; the callback may start the next one. */
TEST(sdpc_busy_and_chaining) {
    uint16_t h = sdpc_setup();
    uint8_t lists[128];
    rec_spec_t rec = {.rfcomm_form = 2, .channel = 9};
    uint16_t n = build_lists(lists, sizeof lists, &rec, 1);
    ASSERT_EQ(floss_sim_sdp_serve(lists, n, 0), FLOSS_OK);

    const uint8_t bad_len_uuid[3] = {0};
    ASSERT_EQ(floss_sdp_query_ssa(h, bad_len_uuid, 3, 0, 0xFFFF, on_attr, NULL), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_sdp_query_ssa(h, k_svc_uuid, 16, 5, 4, on_attr, NULL), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_sdp_query_ssa(h, k_svc_uuid, 16, 0, 0xFFFF, NULL, NULL), FLOSS_ERR_INVALID);
    ASSERT_EQ(floss_sdp_query_ssa(0x0EEE, k_svc_uuid, 16, 0, 0xFFFF, on_attr, NULL), FLOSS_ERR_NOT_FOUND);
    ASSERT_FALSE(floss_sdp_query_busy());

    g_res.chain = true;
    ASSERT_EQ(floss_sdp_query_rfcomm_channel(h, k_svc_uuid, on_rfcomm, NULL), FLOSS_OK);
    ASSERT_EQ(floss_sdp_query_rfcomm_channel(h, k_svc_uuid, on_rfcomm, NULL), FLOSS_ERR_BUSY);
    ASSERT_EQ(floss_sdp_query_ssa(h, k_svc_uuid, 16, 0, 0xFFFF, on_attr, NULL), FLOSS_ERR_BUSY);
    wait_calls(1);
    ASSERT_EQ(g_res.status, 0);
    ASSERT_EQ(g_res.channel, 9);
    ASSERT_EQ(g_res.chain_rc, FLOSS_OK); /* started from inside the callback */
    /* The chained query owned the client from inside the callback (with the
     * simulator's zero latency it may already have finished by now). */
    ASSERT_TRUE(g_res.chain_busy);
    wait_calls(2); /* the chained query's own answer (g_res holds the latest call) */
    ASSERT_EQ(g_res.status, 0);
    ASSERT_EQ(g_res.channel, 9);
    ASSERT_EQ(floss_sim_sdp_server_stats()->requests, 2);
    floss_sim_pump(20);
    ASSERT_NULL(floss_l2cap_find(h, FLOSS_PSM_SDP));
    floss_tc_assert_sim_clean();
}

/* ------------------------------------------------------------------------ */
/* Walker                                                                   */
/* ------------------------------------------------------------------------ */

TEST(sdpc_walker_iterates_records) {
    uint8_t lists[256];
    rec_spec_t recs[2] = {{.rfcomm_form = 0, .name_len = 4}, {.rfcomm_form = 4, .channel = 12}};
    uint16_t n = build_lists(lists, sizeof lists, recs, 2);

    floss_sdp_iter_t it;
    const uint8_t *rec;
    uint16_t rec_len;
    ASSERT_EQ(floss_sdp_iter_init(&it, lists, n), FLOSS_OK);
    ASSERT_FALSE(it.single);
    unsigned count = 0;
    const uint8_t *second = NULL;
    uint16_t second_len = 0;
    while (floss_sdp_iter_next(&it, &rec, &rec_len) == FLOSS_OK) {
        count++;
        second = rec;
        second_len = rec_len;
    }
    ASSERT_EQ(count, 2);

    /* First record that has the attribute wins. */
    const uint8_t *v;
    uint16_t vlen;
    ASSERT_EQ(floss_sdp_find_attr(lists, n, FLOSS_ATTR_SERVICE_RECORD_HANDLE, &v, &vlen), FLOSS_OK);
    ASSERT_EQ(vlen, 5);
    ASSERT_EQ(v[0], 0x0A);
    ASSERT_EQ(floss_get_be32(v + 1), 0x00010000u);
    ASSERT_EQ(floss_sdp_find_attr(lists, n, FLOSS_ATTR_SERVICE_NAME, &v, &vlen), FLOSS_OK);
    ASSERT_EQ(vlen, 6);
    ASSERT_EQ(floss_sdp_find_attr(lists, n, 0x0999, &v, &vlen), FLOSS_ERR_NOT_FOUND);

    /* A single record (attribute list) works the same way. */
    ASSERT_EQ(floss_sdp_iter_init(&it, second, second_len), FLOSS_OK);
    ASSERT_TRUE(it.single);
    ASSERT_EQ(floss_sdp_iter_next(&it, &rec, &rec_len), FLOSS_OK);
    ASSERT_EQ(rec, second);
    ASSERT_EQ(rec_len, second_len);
    ASSERT_EQ(floss_sdp_iter_next(&it, &rec, &rec_len), FLOSS_ERR_NOT_FOUND);
    ASSERT_EQ(floss_sdp_find_attr(second, second_len, FLOSS_ATTR_SERVICE_RECORD_HANDLE, &v, &vlen), FLOSS_OK);
    ASSERT_EQ(floss_get_be32(v + 1), 0x00010001u);
    uint8_t ch = 0;
    ASSERT_EQ(floss_sdp_rfcomm_channel(second, second_len, &ch), FLOSS_OK);
    ASSERT_EQ(ch, 12);
    ASSERT_EQ(floss_sdp_rfcomm_channel(lists, n, &ch), FLOSS_OK);
    ASSERT_EQ(ch, 12);
}

/* An alternative of protocol descriptor lists is searched too. */
TEST(sdpc_walker_alternative_descriptor_lists) {
    uint8_t buf[128];
    floss_de_writer_t w;
    floss_de_init(&w, buf, sizeof buf);
    floss_de_seq_begin(&w);
    floss_de_uint16(&w, FLOSS_ATTR_PROTOCOL_DESCRIPTOR_LIST);
    /* DEA { DES{ DES{L2CAP} }, DES{ DES{L2CAP} DES{RFCOMM 7} } } written raw (the writer has no alternative). */
    const uint8_t alt[] = {0x3D, 0x15,                               /* alternative, 21 bytes */
                           0x35, 0x05, 0x35, 0x03, 0x19, 0x01, 0x00, /* L2CAP only */
                           0x35, 0x0C, 0x35, 0x03, 0x19, 0x01, 0x00, /* L2CAP */
                           0x35, 0x05, 0x19, 0x00, 0x03, 0x08, 0x07}; /* RFCOMM, channel 7 */
    floss_de_raw(&w, alt, sizeof alt);
    floss_de_seq_end(&w);
    ASSERT_FALSE(w.error);
    uint8_t ch = 0;
    ASSERT_EQ(floss_sdp_rfcomm_channel(buf, w.len, &ch), FLOSS_OK);
    ASSERT_EQ(ch, 7);
}

/* Every truncation and many corruptions of a valid result: bounded, no false hit. */
TEST(sdpc_walker_survives_truncation_and_corruption) {
    uint8_t lists[256];
    rec_spec_t recs[2] = {{.rfcomm_form = 16, .channel = 21, .name_len = 10}, {.rfcomm_form = 2, .channel = 22}};
    uint16_t n = build_lists(lists, sizeof lists, recs, 2);

    uint8_t ch;
    const uint8_t *v;
    uint16_t vlen;
    for (uint16_t len = 0; len < n; len++) {
        /* The copy ends exactly at len, so any over-read would leave the buffer.
         * The outer sequence is patched to claim exactly what is left, so the
         * truncation hits the records inside. */
        uint8_t *cut = (uint8_t *)malloc(len ? len : 1);
        ASSERT_NOT_NULL(cut);
        memcpy(cut, lists, len);
        if (len >= 3) {
            floss_put_be16(cut + 1, (uint16_t)(len - 3));
        }
        floss_err_t err = floss_sdp_rfcomm_channel(cut, len, &ch);
        if (err == FLOSS_OK) {
            ASSERT_EQ(ch, 21); /* only once the first record is complete */
        } else {
            ASSERT_EQ(ch, 0);
        }
        if (floss_sdp_find_attr(cut, len, FLOSS_ATTR_SERVICE_NAME, &v, &vlen) == FLOSS_OK) {
            ASSERT_TRUE(v >= cut && v + vlen <= cut + len);
        }
        free(cut);
    }

    uint32_t seed = 0x1234567u;
    for (unsigned round = 0; round < 4000; round++) {
        uint8_t mut[256];
        memcpy(mut, lists, n);
        for (unsigned k = 0; k < 3; k++) {
            seed = seed * 1103515245u + 12345u;
            mut[(seed >> 8) % n] = (uint8_t)(seed >> 24);
        }
        floss_err_t err = floss_sdp_rfcomm_channel(mut, n, &ch);
        if (err == FLOSS_OK) {
            ASSERT_TRUE(ch >= 1 && ch <= 30);
        } else {
            ASSERT_EQ(ch, 0);
        }
        if (floss_sdp_find_attr(mut, n, FLOSS_ATTR_PROTOCOL_DESCRIPTOR_LIST, &v, &vlen) == FLOSS_OK) {
            ASSERT_TRUE(v >= mut && v + vlen <= mut + n);
        }
    }

    ASSERT_EQ(floss_sdp_rfcomm_channel(NULL, 0, &ch), FLOSS_ERR_PROTOCOL);
    ASSERT_EQ(floss_sdp_rfcomm_channel(lists, n, NULL), FLOSS_ERR_INVALID);
}

static const floss_test_case_t tests[] = {
    TEST_CASE(sdpc_rfcomm_channel_found_every_uuid_form),
    TEST_CASE(sdpc_rfcomm_channel_from_second_record),
    TEST_CASE(sdpc_continuation_reassembles_many_responses),
    TEST_CASE(sdpc_no_matching_record),
    TEST_CASE(sdpc_error_response),
    TEST_CASE(sdpc_malformed_results),
    TEST_CASE(sdpc_result_overflow),
    TEST_CASE(sdpc_timeout),
    TEST_CASE(sdpc_link_loss_mid_query),
    TEST_CASE(sdpc_busy_and_chaining),
    TEST_CASE(sdpc_walker_iterates_records),
    TEST_CASE(sdpc_walker_alternative_descriptor_lists),
    TEST_CASE(sdpc_walker_survives_truncation_and_corruption),
};

FLOSS_TEST_MAIN("sdp_client", tests)
