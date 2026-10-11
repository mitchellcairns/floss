/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_sdp_records.c - Ready-made service records (HID, Device ID).
 *
 * Overview
 * --------
 * A service record is one data element sequence of (attribute ID, value)
 * pairs in ascending ID order. The builders below write those pairs with the
 * floss_de_* writer, in the order the server will return them, so the bytes
 * can be served without any sorting or re-encoding.
 *
 * Every record starts with attribute 0x0000 (ServiceRecordHandle) encoded as
 * a uint32 placeholder; floss_sdp_register() overwrites it with the handle it
 * assigns. Records are built once at start-up into application-owned storage
 * (a HID record with a ~150 byte report descriptor is ~330 bytes).
 *
 * Attribute choices follow what desktop and console HID hosts expect from a
 * Classic gamepad; see the per-attribute comments for the reasons.
 */
#include "floss_sdp_priv.h"

/** HID profile / parser versions advertised (HID 1.1 profile, HID 1.11 parser). */
#define HID_PROFILE_VERSION 0x0101
#define HID_PARSER_VERSION  0x0111
#define HID_DEVICE_RELEASE  0x0100 /* attribute 0x020B */
/** Device ID specification version 1.3. */
#define DID_SPEC_VERSION    0x0103

/** Language base: ISO 639 "en", MIBenum 106 (UTF-8), attribute base 0x0100. */
#define LANG_EN        0x656E
#define LANG_UTF8_MIB  0x006A
#define LANG_BASE      0x0100

/** HID descriptor type "report" inside HIDDescriptorList. */
#define HID_DESC_TYPE_REPORT 0x22
/** USB LANGID English (United States) for HIDLANGIDBaseList. */
#define HID_LANGID_EN_US     0x0409

void floss_sdp_record_begin(floss_de_writer_t *w)
{
    floss_de_seq_begin(w);
    floss_de_uint16(w, FLOSS_ATTR_SERVICE_RECORD_HANDLE);
    floss_de_uint32(w, 0); /* patched by floss_sdp_register() */
}

void floss_sdp_record_end(floss_de_writer_t *w)
{
    floss_de_seq_end(w);
}

/** Final check shared by the builders: no overflow and all brackets closed. */
static uint16_t record_finish(floss_de_writer_t *w)
{
    floss_sdp_record_end(w);
    if (w->error || w->depth != 0) {
        return 0;
    }
    return w->len;
}

/** Write attribute ID + a sequence containing one 16-bit UUID. */
static void attr_uuid_list(floss_de_writer_t *w, uint16_t attr, uint16_t uuid)
{
    floss_de_uint16(w, attr);
    floss_de_seq_begin(w);
    floss_de_uuid16(w, uuid);
    floss_de_seq_end(w);
}

/** Write attribute ID + a single-profile ProfileDescriptorList. */
static void attr_profile(floss_de_writer_t *w, uint16_t uuid, uint16_t version)
{
    floss_de_uint16(w, FLOSS_ATTR_PROFILE_DESCRIPTOR_LIST);
    floss_de_seq_begin(w);
    floss_de_seq_begin(w);
    floss_de_uuid16(w, uuid);
    floss_de_uint16(w, version);
    floss_de_seq_end(w);
    floss_de_seq_end(w);
}

static void attr_bool(floss_de_writer_t *w, uint16_t attr, bool v)
{
    floss_de_uint16(w, attr);
    floss_de_bool(w, v);
}

static void attr_u16(floss_de_writer_t *w, uint16_t attr, uint16_t v)
{
    floss_de_uint16(w, attr);
    floss_de_uint16(w, v);
}

/**
 * HID service record.
 *
 * The protocol descriptor lists are what hosts use to find the two L2CAP
 * channels: 0x0004 names the control channel (PSM 0x11) and 0x000D the
 * interrupt channel (PSM 0x13), each with HIDP on top. The report
 * descriptor travels as a text element inside HIDDescriptorList, which is
 * how a host learns the report layout without opening a channel first.
 */
uint16_t floss_sdp_build_hid(uint8_t *buf, uint16_t cap, const floss_sdp_hid_params_t *p)
{
    if (p == NULL) {
        return 0;
    }
    floss_de_writer_t w;
    floss_de_init(&w, buf, cap);
    floss_sdp_record_begin(&w);

    attr_uuid_list(&w, FLOSS_ATTR_SERVICE_CLASS_ID_LIST, FLOSS_UUID_HID);

    /* 0x0004: L2CAP(PSM 0x0011) / HIDP */
    floss_de_uint16(&w, FLOSS_ATTR_PROTOCOL_DESCRIPTOR_LIST);
    floss_de_seq_begin(&w);
    floss_de_seq_begin(&w);
    floss_de_uuid16(&w, FLOSS_UUID_L2CAP);
    floss_de_uint16(&w, FLOSS_PSM_HID_CONTROL);
    floss_de_seq_end(&w);
    floss_de_seq_begin(&w);
    floss_de_uuid16(&w, FLOSS_UUID_HIDP);
    floss_de_seq_end(&w);
    floss_de_seq_end(&w);

    attr_uuid_list(&w, FLOSS_ATTR_BROWSE_GROUP_LIST, FLOSS_UUID_PUBLIC_BROWSE_ROOT);

    /* 0x0006: one language (English, UTF-8) whose strings start at 0x0100. */
    floss_de_uint16(&w, FLOSS_ATTR_LANGUAGE_BASE_LIST);
    floss_de_seq_begin(&w);
    floss_de_uint16(&w, LANG_EN);
    floss_de_uint16(&w, LANG_UTF8_MIB);
    floss_de_uint16(&w, LANG_BASE);
    floss_de_seq_end(&w);

    attr_profile(&w, FLOSS_UUID_HID, HID_PROFILE_VERSION);

    /* 0x000D: one additional stack, L2CAP(PSM 0x0013) / HIDP. */
    floss_de_uint16(&w, FLOSS_ATTR_ADDITIONAL_PROTOCOL_DESCRIPTOR_LISTS);
    floss_de_seq_begin(&w);
    floss_de_seq_begin(&w);
    floss_de_seq_begin(&w);
    floss_de_uuid16(&w, FLOSS_UUID_L2CAP);
    floss_de_uint16(&w, FLOSS_PSM_HID_INTERRUPT);
    floss_de_seq_end(&w);
    floss_de_seq_begin(&w);
    floss_de_uuid16(&w, FLOSS_UUID_HIDP);
    floss_de_seq_end(&w);
    floss_de_seq_end(&w);
    floss_de_seq_end(&w);

    /* Human readable strings (language base offset 0). */
    if (p->service_name != NULL) {
        floss_de_uint16(&w, FLOSS_ATTR_SERVICE_NAME);
        floss_de_str(&w, p->service_name);
    }
    if (p->service_description != NULL) {
        floss_de_uint16(&w, FLOSS_ATTR_SERVICE_DESCRIPTION);
        floss_de_str(&w, p->service_description);
    }
    if (p->provider_name != NULL) {
        floss_de_uint16(&w, FLOSS_ATTR_PROVIDER_NAME);
        floss_de_str(&w, p->provider_name);
    }

    /* HID-specific attributes, 0x0201.. in ascending order. */
    attr_u16(&w, FLOSS_ATTR_HID_PARSER_VERSION, HID_PARSER_VERSION);
    floss_de_uint16(&w, FLOSS_ATTR_HID_DEVICE_SUBCLASS);
    floss_de_uint8(&w, p->subclass);
    floss_de_uint16(&w, FLOSS_ATTR_HID_COUNTRY_CODE);
    floss_de_uint8(&w, p->country_code);
    attr_bool(&w, FLOSS_ATTR_HID_VIRTUAL_CABLE, p->virtual_cable);
    attr_bool(&w, FLOSS_ATTR_HID_RECONNECT_INITIATE, p->reconnect_initiate);

    /* 0x0206: { { 0x22 (report), <descriptor bytes> } } */
    floss_de_uint16(&w, FLOSS_ATTR_HID_DESCRIPTOR_LIST);
    floss_de_seq_begin(&w);
    floss_de_seq_begin(&w);
    floss_de_uint8(&w, HID_DESC_TYPE_REPORT);
    floss_de_text(&w, p->report_descriptor, p->report_descriptor_len);
    floss_de_seq_end(&w);
    floss_de_seq_end(&w);

    /* 0x0207: { { LANGID en-US, language base 0x0100 } } */
    floss_de_uint16(&w, FLOSS_ATTR_HID_LANGID_BASE_LIST);
    floss_de_seq_begin(&w);
    floss_de_seq_begin(&w);
    floss_de_uint16(&w, HID_LANGID_EN_US);
    floss_de_uint16(&w, LANG_BASE);
    floss_de_seq_end(&w);
    floss_de_seq_end(&w);

    attr_bool(&w, FLOSS_ATTR_HID_BATTERY_POWER, p->battery_power);
    attr_bool(&w, FLOSS_ATTR_HID_REMOTE_WAKE, p->remote_wake);
    attr_u16(&w, FLOSS_ATTR_HID_PROFILE_VERSION, HID_DEVICE_RELEASE);
    if (p->supervision_timeout != 0) {
        attr_u16(&w, FLOSS_ATTR_HID_SUPERVISION_TIMEOUT, p->supervision_timeout);
    }
    attr_bool(&w, FLOSS_ATTR_HID_NORMALLY_CONNECTABLE, p->normally_connectable);
    attr_bool(&w, FLOSS_ATTR_HID_BOOT_DEVICE, p->boot_device);
    /* Sniff subrating hints: hosts treat absence as "no preference". */
    if (p->ssr_host_max_latency != 0) {
        attr_u16(&w, FLOSS_ATTR_HID_SSR_HOST_MAX_LATENCY, p->ssr_host_max_latency);
    }
    if (p->ssr_host_min_timeout != 0) {
        attr_u16(&w, FLOSS_ATTR_HID_SSR_HOST_MIN_TIMEOUT, p->ssr_host_min_timeout);
    }

    uint16_t len = record_finish(&w);
    if (len == 0) {
        FLOSS_LOGE(SDP, "HID record does not fit in %u bytes", (unsigned)cap);
    }
    return len;
}

/**
 * Device ID (PnP Information) record. Hosts use the vendor/product/version
 * triple to pick drivers and button mappings, so a gamepad that wants to be
 * recognised as a specific product publishes this next to its HID record.
 * The protocol descriptor points at SDP itself (PSM 0x0001): Device ID has
 * no channel of its own.
 */
uint16_t floss_sdp_build_did(uint8_t *buf, uint16_t cap, const floss_sdp_did_params_t *p)
{
    if (p == NULL) {
        return 0;
    }
    floss_de_writer_t w;
    floss_de_init(&w, buf, cap);
    floss_sdp_record_begin(&w);

    attr_uuid_list(&w, FLOSS_ATTR_SERVICE_CLASS_ID_LIST, FLOSS_UUID_PNP_INFORMATION);

    floss_de_uint16(&w, FLOSS_ATTR_PROTOCOL_DESCRIPTOR_LIST);
    floss_de_seq_begin(&w);
    floss_de_seq_begin(&w);
    floss_de_uuid16(&w, FLOSS_UUID_L2CAP);
    floss_de_uint16(&w, FLOSS_PSM_SDP);
    floss_de_seq_end(&w);
    floss_de_seq_begin(&w);
    floss_de_uuid16(&w, FLOSS_UUID_SDP);
    floss_de_seq_end(&w);
    floss_de_seq_end(&w);

    attr_uuid_list(&w, FLOSS_ATTR_BROWSE_GROUP_LIST, FLOSS_UUID_PUBLIC_BROWSE_ROOT);
    attr_profile(&w, FLOSS_UUID_PNP_INFORMATION, DID_SPEC_VERSION);

    attr_u16(&w, FLOSS_ATTR_DID_SPECIFICATION_ID, DID_SPEC_VERSION);
    attr_u16(&w, FLOSS_ATTR_DID_VENDOR_ID, p->vendor_id);
    attr_u16(&w, FLOSS_ATTR_DID_PRODUCT_ID, p->product_id);
    attr_u16(&w, FLOSS_ATTR_DID_VERSION, p->version);
    attr_bool(&w, FLOSS_ATTR_DID_PRIMARY_RECORD, p->primary);
    attr_u16(&w, FLOSS_ATTR_DID_VENDOR_ID_SOURCE, p->vendor_id_source);

    uint16_t len = record_finish(&w);
    if (len == 0) {
        FLOSS_LOGE(SDP, "Device ID record does not fit in %u bytes", (unsigned)cap);
    }
    return len;
}
