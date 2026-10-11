/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * test_sdp.c - SDP data element writer/parser and the ready-made records.
 *
 * Data elements are big-endian TLVs: a header byte (5-bit type, 3-bit size
 * index) followed by an optional length and the value. The writer always
 * uses 2-byte lengths for sequences (0x36) so nesting needs no memmove;
 * these tests pin the exact bytes, because a host's SDP parser is the
 * least forgiving reader a device will ever meet.
 *
 * The record tests walk the serialised HID / Device ID records with
 * floss_de_header() the way a host would, and check every attribute the
 * HID profile marks mandatory is present, in strictly ascending order.
 */
#include "floss/floss.h"
#include "floss_test.h"

/* floss_de_header() reports the type as the 5-bit descriptor; normalise in
 * case it is returned still shifted into the header-byte position. */
static uint8_t de_type(uint8_t t) {
    return (t >= 8 && (t & 0x07) == 0) ? (uint8_t)(t >> 3) : t; /* (URL, type 8, is never tested) */
}

#define DE_NIL  0
#define DE_UINT 1
#define DE_INT  2
#define DE_UUID 3
#define DE_TEXT 4
#define DE_BOOL 5
#define DE_SEQ  6
#define DE_ALT  7

/* ------------------------------------------------------------------------ */
/* Writer                                                                   */
/* ------------------------------------------------------------------------ */

TEST(de_scalar_encodings) {
    uint8_t b[64];
    floss_de_writer_t w;
    floss_de_init(&w, b, sizeof b);
    floss_de_nil(&w);
    floss_de_bool(&w, true);
    floss_de_bool(&w, false);
    floss_de_uint8(&w, 0x12);
    floss_de_uint16(&w, 0x1234);
    floss_de_uint32(&w, 0x12345678);
    floss_de_int8(&w, -2);
    floss_de_uuid16(&w, 0x1124);
    floss_de_uuid32(&w, 0x00011124);
    const uint8_t want[] = {
        0x00,                         /* nil */
        0x28, 0x01,                   /* bool true */
        0x28, 0x00,                   /* bool false */
        0x08, 0x12,                   /* uint8 */
        0x09, 0x12, 0x34,             /* uint16, big-endian */
        0x0A, 0x12, 0x34, 0x56, 0x78, /* uint32 */
        0x10, 0xFE,                   /* int8 -2 */
        0x19, 0x11, 0x24,             /* uuid16 */
        0x1A, 0x00, 0x01, 0x11, 0x24, /* uuid32 */
    };
    ASSERT_FALSE(w.error);
    ASSERT_EQ(w.len, sizeof want);
    ASSERT_MEM_EQ(b, want, sizeof want);
}

TEST(de_uuid128_and_text) {
    static uint8_t b[512];
    static const uint8_t uuid[16] = {0x00, 0x00, 0x11, 0x24, 0x00, 0x00, 0x10, 0x00,
                                     0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
    floss_de_writer_t w;
    floss_de_init(&w, b, sizeof b);
    floss_de_uuid128(&w, uuid);
    ASSERT_EQ(w.len, 17);
    ASSERT_EQ(b[0], 0x1C);
    ASSERT_MEM_EQ(b + 1, uuid, 16);

    floss_de_init(&w, b, sizeof b);
    floss_de_str(&w, "Pad");
    const uint8_t want_str[] = {0x25, 0x03, 'P', 'a', 'd'};
    ASSERT_EQ(w.len, sizeof want_str);
    ASSERT_MEM_EQ(b, want_str, sizeof want_str);

    /* Text longer than 255 bytes needs the 2-byte length form (0x26). */
    static uint8_t blob[300];
    for (unsigned i = 0; i < sizeof blob; i++) {
        blob[i] = (uint8_t)i;
    }
    floss_de_init(&w, b, sizeof b);
    floss_de_text(&w, blob, sizeof blob);
    ASSERT_FALSE(w.error);
    ASSERT_EQ(w.len, 3 + sizeof blob);
    ASSERT_EQ(b[0], 0x26);
    ASSERT_EQ(floss_get_be16(b + 1), sizeof blob);
    ASSERT_MEM_EQ(b + 3, blob, sizeof blob);

    /* Raw bytes are copied verbatim. */
    const uint8_t raw[] = {0x09, 0xAB, 0xCD};
    floss_de_init(&w, b, sizeof b);
    floss_de_raw(&w, raw, sizeof raw);
    ASSERT_EQ(w.len, sizeof raw);
    ASSERT_MEM_EQ(b, raw, sizeof raw);
}

TEST(de_nested_sequences) {
    uint8_t b[64];
    floss_de_writer_t w;
    floss_de_init(&w, b, sizeof b);
    floss_de_seq_begin(&w);
    floss_de_uint16(&w, 0x0001);
    floss_de_seq_begin(&w);
    floss_de_uuid16(&w, 0x1124);
    floss_de_seq_end(&w);
    floss_de_seq_begin(&w);
    floss_de_seq_end(&w);
    floss_de_seq_end(&w);
    const uint8_t want[] = {
        0x36, 0x00, 0x0C,             /* outer sequence, 12 bytes */
        0x09, 0x00, 0x01,
        0x36, 0x00, 0x03, 0x19, 0x11, 0x24,
        0x36, 0x00, 0x00,             /* empty sequence */
    };
    ASSERT_FALSE(w.error);
    ASSERT_EQ(w.depth, 0);
    ASSERT_EQ(w.len, sizeof want);
    ASSERT_MEM_EQ(b, want, sizeof want);
}

TEST(de_overflow_and_balance_errors) {
    uint8_t b[64];
    floss_de_writer_t w;

    floss_de_init(&w, b, 4);
    floss_de_uint32(&w, 1); /* needs 5 bytes */
    ASSERT_TRUE(w.error);
    ASSERT_LE(w.len, 4);

    floss_de_init(&w, b, 5);
    floss_de_seq_begin(&w);
    floss_de_uint16(&w, 1); /* 3 + 3 > 5 */
    ASSERT_TRUE(w.error);
    ASSERT_LE(w.len, 5);

    floss_de_init(&w, b, sizeof b);
    floss_de_seq_end(&w); /* close without open */
    ASSERT_TRUE(w.error);

    floss_de_init(&w, b, sizeof b);
    for (unsigned i = 0; i < FLOSS_DE_MAX_DEPTH + 1; i++) {
        floss_de_seq_begin(&w);
    }
    ASSERT_TRUE(w.error);
}

TEST(de_header_parse) {
    uint8_t type;
    uint32_t len;
    const uint8_t u16[] = {0x09, 0x12, 0x34};
    ASSERT_EQ(floss_de_header(u16, sizeof u16, &type, &len), 1);
    ASSERT_EQ(de_type(type), DE_UINT);
    ASSERT_EQ(len, 2);

    const uint8_t nil[] = {0x00};
    ASSERT_EQ(floss_de_header(nil, 1, &type, &len), 1);
    ASSERT_EQ(de_type(type), DE_NIL);
    ASSERT_EQ(len, 0);

    const uint8_t text[] = {0x25, 0x03, 'a', 'b', 'c'};
    ASSERT_EQ(floss_de_header(text, sizeof text, &type, &len), 2);
    ASSERT_EQ(de_type(type), DE_TEXT);
    ASSERT_EQ(len, 3);

    const uint8_t seq8[] = {0x35, 0x05, 0, 0, 0, 0, 0}; /* header + payload: the parser checks the payload fits */
    ASSERT_EQ(floss_de_header(seq8, sizeof seq8, &type, &len), 2);
    ASSERT_EQ(de_type(type), DE_SEQ);
    ASSERT_EQ(len, 5);

    /* The parser also checks that the payload fits in avail, so the longer
     * elements are parsed from a zero-padded buffer. */
    static uint8_t pad[0x200];
    memset(pad, 0, sizeof pad);
    pad[0] = 0x36; pad[1] = 0x01; pad[2] = 0x02;
    ASSERT_EQ(floss_de_header(pad, sizeof pad, &type, &len), 3);
    ASSERT_EQ(len, 0x0102);
    const uint8_t *seq16 = pad;

    static uint8_t pad32[0x200];
    memset(pad32, 0, sizeof pad32);
    pad32[0] = 0x37; pad32[3] = 0x01;
    ASSERT_EQ(floss_de_header(pad32, sizeof pad32, &type, &len), 5);
    ASSERT_EQ(len, 0x100);

    const uint8_t uuid128[17] = {0x1C};
    ASSERT_EQ(floss_de_header(uuid128, sizeof uuid128, &type, &len), 1);
    ASSERT_EQ(de_type(type), DE_UUID);
    ASSERT_EQ(len, 16);

    const uint8_t b8[] = {0x28, 0x01};
    ASSERT_EQ(floss_de_header(b8, 2, &type, &len), 1);
    ASSERT_EQ(de_type(type), DE_BOOL);

    /* Truncated length field -> error. */
    ASSERT_EQ(floss_de_header(seq16, 2, &type, &len), 0);
    ASSERT_EQ(floss_de_header(seq16, 0, &type, &len), 0);
}

/* ------------------------------------------------------------------------ */
/* Record walking                                                           */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint16_t id;
    const uint8_t *val; /* value element, header included */
    uint16_t val_len;
} attr_t;

/** Parse a record (one sequence of id/value pairs) like a host would. */
static unsigned walk_record(const uint8_t *rec, uint16_t len, attr_t *out, unsigned max) {
    uint8_t type;
    uint32_t l;
    uint16_t h = floss_de_header(rec, len, &type, &l);
    ASSERT_GT(h, 0);
    ASSERT_EQ(de_type(type), DE_SEQ);
    ASSERT_EQ(h + l, len); /* the outer sequence spans the whole record */

    const uint8_t *p = rec + h;
    const uint8_t *end = p + l;
    unsigned n = 0;
    while (p < end) {
        uint32_t il;
        uint16_t ih = floss_de_header(p, (uint16_t)(end - p), &type, &il);
        ASSERT_GT(ih, 0);
        ASSERT_EQ(de_type(type), DE_UINT);
        ASSERT_EQ(il, 2); /* attribute IDs are uint16 */
        uint16_t id = floss_get_be16(p + ih);
        p += ih + 2;

        uint32_t vl;
        uint16_t vh = floss_de_header(p, (uint16_t)(end - p), &type, &vl);
        ASSERT_GT(vh, 0);
        ASSERT_LE((size_t)vh + vl, (size_t)(end - p));
        ASSERT_LT(n, max);
        out[n].id = id;
        out[n].val = p;
        out[n].val_len = (uint16_t)(vh + vl);
        n++;
        p += vh + vl;
    }
    for (unsigned i = 1; i < n; i++) {
        if (out[i].id <= out[i - 1].id) {
            FAIL("attribute 0x%04x follows 0x%04x: IDs must be strictly ascending", out[i].id, out[i - 1].id);
        }
    }
    return n;
}

static const attr_t *find_attr(const attr_t *a, unsigned n, uint16_t id) {
    for (unsigned i = 0; i < n; i++) {
        if (a[i].id == id) {
            return &a[i];
        }
    }
    return NULL;
}

static bool attr_contains(const attr_t *a, const uint8_t *bytes, size_t len) {
    return a && floss_test_memfind(a->val, a->val_len, bytes, len) != NULL;
}

static const uint8_t g_desc[] = {0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0x85, 0x01, 0x05, 0x09,
                                 0x19, 0x01, 0x29, 0x10, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01,
                                 0x95, 0x10, 0x81, 0x02, 0xC0};

static void hid_params(floss_sdp_hid_params_t *p) {
    memset(p, 0, sizeof *p);
    p->service_name = "FLOSS Pad";
    p->service_description = "Gamepad";
    p->provider_name = "FLOSS";
    p->subclass = 0x08;
    p->country_code = 0;
    p->virtual_cable = true;
    p->reconnect_initiate = true;
    p->battery_power = true;
    p->remote_wake = false;
    p->normally_connectable = false;
    p->boot_device = false;
    p->supervision_timeout = 0x0C80;
    p->ssr_host_max_latency = 0;
    p->ssr_host_min_timeout = 0;
    p->report_descriptor = g_desc;
    p->report_descriptor_len = sizeof g_desc;
}

TEST(hid_record_mandatory_attributes) {
    static uint8_t rec[512];
    static attr_t attrs[48];
    floss_sdp_hid_params_t p;
    hid_params(&p);
    uint16_t len = floss_sdp_build_hid(rec, sizeof rec, &p);
    ASSERT_GT(len, 0);
    unsigned n = walk_record(rec, len, attrs, FLOSS_ARRAY_SIZE(attrs));

    static const uint16_t mandatory[] = {
        FLOSS_ATTR_SERVICE_RECORD_HANDLE,
        FLOSS_ATTR_SERVICE_CLASS_ID_LIST,
        FLOSS_ATTR_PROTOCOL_DESCRIPTOR_LIST,
        FLOSS_ATTR_LANGUAGE_BASE_LIST,
        FLOSS_ATTR_PROFILE_DESCRIPTOR_LIST,
        FLOSS_ATTR_ADDITIONAL_PROTOCOL_DESCRIPTOR_LISTS,
        FLOSS_ATTR_SERVICE_NAME,
        FLOSS_ATTR_HID_PARSER_VERSION,
        FLOSS_ATTR_HID_DEVICE_SUBCLASS,
        FLOSS_ATTR_HID_COUNTRY_CODE,
        FLOSS_ATTR_HID_VIRTUAL_CABLE,
        FLOSS_ATTR_HID_RECONNECT_INITIATE,
        FLOSS_ATTR_HID_DESCRIPTOR_LIST,
        FLOSS_ATTR_HID_LANGID_BASE_LIST,
        FLOSS_ATTR_HID_BOOT_DEVICE,
    };
    for (unsigned i = 0; i < FLOSS_ARRAY_SIZE(mandatory); i++) {
        if (!find_attr(attrs, n, mandatory[i])) {
            FAIL("HID record lacks mandatory attribute 0x%04x", mandatory[i]);
        }
    }

    /* Record handle first, as a uint32 (the server patches it in). */
    ASSERT_EQ(attrs[0].id, FLOSS_ATTR_SERVICE_RECORD_HANDLE);
    ASSERT_EQ(attrs[0].val[0], 0x0A);

    /* Service class: HID. */
    const uint8_t hid_uuid[] = {0x19, 0x11, 0x24};
    ASSERT_TRUE(attr_contains(find_attr(attrs, n, FLOSS_ATTR_SERVICE_CLASS_ID_LIST), hid_uuid, 3));

    /* Protocol stack: L2CAP PSM 0x0011 (control) + HIDP; interrupt 0x0013 in the additional list. */
    const uint8_t l2cap_ctrl[] = {0x19, 0x01, 0x00, 0x09, 0x00, 0x11};
    const uint8_t l2cap_intr[] = {0x19, 0x01, 0x00, 0x09, 0x00, 0x13};
    const uint8_t hidp[] = {0x19, 0x00, 0x11};
    const attr_t *pdl = find_attr(attrs, n, FLOSS_ATTR_PROTOCOL_DESCRIPTOR_LIST);
    ASSERT_TRUE(attr_contains(pdl, l2cap_ctrl, sizeof l2cap_ctrl));
    ASSERT_TRUE(attr_contains(pdl, hidp, sizeof hidp));
    const attr_t *apdl = find_attr(attrs, n, FLOSS_ATTR_ADDITIONAL_PROTOCOL_DESCRIPTOR_LISTS);
    ASSERT_TRUE(attr_contains(apdl, l2cap_intr, sizeof l2cap_intr));
    ASSERT_TRUE(attr_contains(apdl, hidp, sizeof hidp));

    /* Profile descriptor: HID. */
    ASSERT_TRUE(attr_contains(find_attr(attrs, n, FLOSS_ATTR_PROFILE_DESCRIPTOR_LIST), hid_uuid, 3));

    /* Scalar values. */
    const uint8_t subclass[] = {0x08, 0x08};
    ASSERT_MEM_EQ(find_attr(attrs, n, FLOSS_ATTR_HID_DEVICE_SUBCLASS)->val, subclass, 2);
    const uint8_t country[] = {0x08, 0x00};
    ASSERT_MEM_EQ(find_attr(attrs, n, FLOSS_ATTR_HID_COUNTRY_CODE)->val, country, 2);
    const uint8_t yes[] = {0x28, 0x01};
    const uint8_t no[] = {0x28, 0x00};
    ASSERT_MEM_EQ(find_attr(attrs, n, FLOSS_ATTR_HID_VIRTUAL_CABLE)->val, yes, 2);
    ASSERT_MEM_EQ(find_attr(attrs, n, FLOSS_ATTR_HID_RECONNECT_INITIATE)->val, yes, 2);
    ASSERT_MEM_EQ(find_attr(attrs, n, FLOSS_ATTR_HID_BOOT_DEVICE)->val, no, 2);

    /* Descriptor list: sequence { sequence { uint8 0x22 (report), text descriptor } }. */
    const attr_t *dl = find_attr(attrs, n, FLOSS_ATTR_HID_DESCRIPTOR_LIST);
    const uint8_t report_type[] = {0x08, 0x22};
    ASSERT_TRUE(attr_contains(dl, report_type, 2));
    const uint8_t *d = floss_test_memfind(dl->val, dl->val_len, g_desc, sizeof g_desc);
    ASSERT_NOT_NULL(d);
    ASSERT_EQ(d[-2], 0x25); /* text, 1-byte length */
    ASSERT_EQ(d[-1], sizeof g_desc);

    /* Optional attributes follow their parameters. */
    ASSERT_NOT_NULL(find_attr(attrs, n, FLOSS_ATTR_HID_SUPERVISION_TIMEOUT));
    ASSERT_NOT_NULL(find_attr(attrs, n, FLOSS_ATTR_HID_BATTERY_POWER));
    ASSERT_NULL(find_attr(attrs, n, FLOSS_ATTR_HID_SSR_HOST_MAX_LATENCY));
    ASSERT_NULL(find_attr(attrs, n, FLOSS_ATTR_HID_SSR_HOST_MIN_TIMEOUT));

    /* Service name as text. */
    const uint8_t name[] = {0x25, 9, 'F', 'L', 'O', 'S', 'S', ' ', 'P', 'a', 'd'};
    ASSERT_MEM_EQ(find_attr(attrs, n, FLOSS_ATTR_SERVICE_NAME)->val, name, sizeof name);
}

TEST(hid_record_overflow) {
    uint8_t rec[40];
    floss_sdp_hid_params_t p;
    hid_params(&p);
    ASSERT_EQ(floss_sdp_build_hid(rec, sizeof rec, &p), 0);
}

TEST(did_record) {
    static uint8_t rec[256];
    static attr_t attrs[24];
    floss_sdp_did_params_t p = {
        .vendor_id_source = 2,
        .vendor_id = 0x1209,
        .product_id = 0xF105,
        .version = 0x0123,
        .primary = true,
    };
    uint16_t len = floss_sdp_build_did(rec, sizeof rec, &p);
    ASSERT_GT(len, 0);
    unsigned n = walk_record(rec, len, attrs, FLOSS_ARRAY_SIZE(attrs));

    ASSERT_EQ(attrs[0].id, FLOSS_ATTR_SERVICE_RECORD_HANDLE);
    const uint8_t pnp[] = {0x19, 0x12, 0x00};
    ASSERT_TRUE(attr_contains(find_attr(attrs, n, FLOSS_ATTR_SERVICE_CLASS_ID_LIST), pnp, 3));

    const uint8_t spec[] = {0x09, 0x01, 0x03}; /* Device ID specification 1.3 */
    const uint8_t vid[] = {0x09, 0x12, 0x09};
    const uint8_t pid[] = {0x09, 0xF1, 0x05};
    const uint8_t ver[] = {0x09, 0x01, 0x23};
    const uint8_t primary[] = {0x28, 0x01};
    const uint8_t src[] = {0x09, 0x00, 0x02};
    const attr_t *a;
    ASSERT_NOT_NULL(a = find_attr(attrs, n, FLOSS_ATTR_DID_SPECIFICATION_ID));
    ASSERT_MEM_EQ(a->val, spec, 3);
    ASSERT_NOT_NULL(a = find_attr(attrs, n, FLOSS_ATTR_DID_VENDOR_ID));
    ASSERT_MEM_EQ(a->val, vid, 3);
    ASSERT_NOT_NULL(a = find_attr(attrs, n, FLOSS_ATTR_DID_PRODUCT_ID));
    ASSERT_MEM_EQ(a->val, pid, 3);
    ASSERT_NOT_NULL(a = find_attr(attrs, n, FLOSS_ATTR_DID_VERSION));
    ASSERT_MEM_EQ(a->val, ver, 3);
    ASSERT_NOT_NULL(a = find_attr(attrs, n, FLOSS_ATTR_DID_PRIMARY_RECORD));
    ASSERT_MEM_EQ(a->val, primary, 2);
    ASSERT_NOT_NULL(a = find_attr(attrs, n, FLOSS_ATTR_DID_VENDOR_ID_SOURCE));
    ASSERT_MEM_EQ(a->val, src, 3);
}

static const floss_test_case_t tests[] = {
    TEST_CASE(de_scalar_encodings),
    TEST_CASE(de_uuid128_and_text),
    TEST_CASE(de_nested_sequences),
    TEST_CASE(de_overflow_and_balance_errors),
    TEST_CASE(de_header_parse),
    TEST_CASE(hid_record_mandatory_attributes),
    TEST_CASE(hid_record_overflow),
    TEST_CASE(did_record),
};

FLOSS_TEST_MAIN("sdp", tests)
