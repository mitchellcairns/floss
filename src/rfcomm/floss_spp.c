/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_spp.c - Serial port service: an RFCOMM server channel plus the SDP
 * record that lets peers find it.
 *
 * The service itself is just RFCOMM; what makes it "the serial port" is the
 * record. Peers search SDP for service class 0x1101, read the protocol
 * descriptor list to learn the RFCOMM server channel, and connect to it:
 *
 *   0x0000 ServiceRecordHandle      uint32 (patched by the SDP server)
 *   0x0001 ServiceClassIDList       { UUID 0x1101 }
 *   0x0004 ProtocolDescriptorList   { { UUID L2CAP }, { UUID RFCOMM, uint8 ch } }
 *   0x0005 BrowseGroupList          { UUID PublicBrowseRoot }
 *   0x0006 LanguageBaseAttributeIDList { "en", UTF-8, 0x0100 }
 *   0x0009 ProfileDescriptorList    { { UUID 0x1101, version 1.2 } }
 *   0x0100 ServiceName              text
 *
 * No PSM appears under L2CAP: RFCOMM's PSM (0x0003) is fixed. Records are
 * built once into static storage (they must outlive registration).
 */
#include "floss_rfcomm_int.h"

#if FLOSS_ENABLE_RFCOMM

/** Record storage per instance: ~80 bytes of attributes plus the name. */
#ifndef FLOSS_SPP_RECORD_MAX
#define FLOSS_SPP_RECORD_MAX 160
#endif

#define SPP_PROFILE_VERSION 0x0102
#define SPP_LANG_EN         0x656E /* ISO 639 "en" */
#define SPP_ENCODING_UTF8   0x006A /* IANA MIBenum 106 */
#define SPP_LANG_BASE       0x0100

typedef struct {
    uint8_t channel; /* 0 = free */
    floss_sdp_record_t rec;
    uint8_t buf[FLOSS_SPP_RECORD_MAX];
} spp_instance_t;

static spp_instance_t s_spp[FLOSS_SPP_MAX_INSTANCES];

void floss_spp_reset(void) {
    memset(s_spp, 0, sizeof s_spp);
}

/** Serialise the record; returns its length or 0 if it did not fit. */
static uint16_t build_record(uint8_t *buf, uint16_t cap, uint8_t channel, const char *name) {
    floss_de_writer_t w;
    floss_de_init(&w, buf, cap);
    floss_sdp_record_begin(&w);

    floss_de_uint16(&w, FLOSS_ATTR_SERVICE_CLASS_ID_LIST);
    floss_de_seq_begin(&w);
    floss_de_uuid16(&w, FLOSS_UUID_SERIAL_PORT);
    floss_de_seq_end(&w);

    floss_de_uint16(&w, FLOSS_ATTR_PROTOCOL_DESCRIPTOR_LIST);
    floss_de_seq_begin(&w);
    floss_de_seq_begin(&w);
    floss_de_uuid16(&w, FLOSS_UUID_L2CAP);
    floss_de_seq_end(&w);
    floss_de_seq_begin(&w);
    floss_de_uuid16(&w, FLOSS_UUID_RFCOMM);
    floss_de_uint8(&w, channel);
    floss_de_seq_end(&w);
    floss_de_seq_end(&w);

    floss_de_uint16(&w, FLOSS_ATTR_BROWSE_GROUP_LIST);
    floss_de_seq_begin(&w);
    floss_de_uuid16(&w, FLOSS_UUID_PUBLIC_BROWSE_ROOT);
    floss_de_seq_end(&w);

    floss_de_uint16(&w, FLOSS_ATTR_LANGUAGE_BASE_LIST);
    floss_de_seq_begin(&w);
    floss_de_uint16(&w, SPP_LANG_EN);
    floss_de_uint16(&w, SPP_ENCODING_UTF8);
    floss_de_uint16(&w, SPP_LANG_BASE);
    floss_de_seq_end(&w);

    floss_de_uint16(&w, FLOSS_ATTR_PROFILE_DESCRIPTOR_LIST);
    floss_de_seq_begin(&w);
    floss_de_seq_begin(&w);
    floss_de_uuid16(&w, FLOSS_UUID_SERIAL_PORT);
    floss_de_uint16(&w, SPP_PROFILE_VERSION);
    floss_de_seq_end(&w);
    floss_de_seq_end(&w);

    floss_de_uint16(&w, FLOSS_ATTR_SERVICE_NAME);
    floss_de_str(&w, name);

    floss_sdp_record_end(&w);
    return w.error ? 0 : w.len;
}

floss_err_t floss_spp_init(uint8_t channel, const char *service_name, const floss_rfcomm_ops_t *ops, void *user) {
    if (service_name == NULL) {
        return FLOSS_ERR_INVALID;
    }
    spp_instance_t *inst = NULL;
    for (uint8_t i = 0; i < FLOSS_SPP_MAX_INSTANCES; i++) {
        if (s_spp[i].channel == 0) {
            inst = &s_spp[i];
            break;
        }
    }
    if (inst == NULL) {
        return FLOSS_ERR_NOMEM;
    }

    floss_err_t err = floss_rfcomm_register_server(channel, ops, user);
    if (err != FLOSS_OK) {
        return err;
    }
    uint16_t len = build_record(inst->buf, sizeof inst->buf, channel, service_name);
    if (len == 0) {
        RFC_LOGE("serial port record too large (name too long?)");
        floss_rfcomm_unregister_server(channel);
        return FLOSS_ERR_NOMEM;
    }
    inst->rec.handle = 0;
    inst->rec.data = inst->buf;
    inst->rec.len = len;
    err = floss_sdp_register(&inst->rec);
    if (err != FLOSS_OK) {
        floss_rfcomm_unregister_server(channel);
        return err;
    }
    inst->channel = channel;
    RFC_LOGI("serial port \"%s\" on channel %u", service_name, channel);
    return FLOSS_OK;
}

#endif /* FLOSS_ENABLE_RFCOMM */
