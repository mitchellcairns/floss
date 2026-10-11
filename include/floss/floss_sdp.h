/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_sdp.h - Service Discovery server.
 *
 * A record is a pre-serialised byte array: one Data Element Sequence of
 * (attribute ID uint16, value) pairs sorted by ascending attribute ID.
 * Build it once at start-up with the floss_de_* writer (or the ready-made
 * HID / Device ID builders below), register it, done. The server answers
 * ServiceSearch, ServiceAttribute and ServiceSearchAttribute requests with
 * continuation support, straight from that array (no per-request building).
 */
#ifndef FLOSS_SDP_H
#define FLOSS_SDP_H

#include "floss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Well-known UUIDs and attribute IDs                                       */
/* ------------------------------------------------------------------------ */

#define FLOSS_UUID_SDP                    0x0001
#define FLOSS_UUID_L2CAP                  0x0100
#define FLOSS_UUID_HIDP                   0x0011
#define FLOSS_UUID_PUBLIC_BROWSE_ROOT     0x1002
#define FLOSS_UUID_SERVICE_DISCOVERY_SERVER 0x1000
#define FLOSS_UUID_HID                    0x1124
#define FLOSS_UUID_PNP_INFORMATION        0x1200

#define FLOSS_ATTR_SERVICE_RECORD_HANDLE  0x0000
#define FLOSS_ATTR_SERVICE_CLASS_ID_LIST  0x0001
#define FLOSS_ATTR_SERVICE_RECORD_STATE   0x0002
#define FLOSS_ATTR_SERVICE_ID             0x0003
#define FLOSS_ATTR_PROTOCOL_DESCRIPTOR_LIST 0x0004
#define FLOSS_ATTR_BROWSE_GROUP_LIST      0x0005
#define FLOSS_ATTR_LANGUAGE_BASE_LIST     0x0006
#define FLOSS_ATTR_SERVICE_INFO_TIME_TO_LIVE 0x0007
#define FLOSS_ATTR_SERVICE_AVAILABILITY   0x0008
#define FLOSS_ATTR_PROFILE_DESCRIPTOR_LIST 0x0009
#define FLOSS_ATTR_ADDITIONAL_PROTOCOL_DESCRIPTOR_LISTS 0x000D
#define FLOSS_ATTR_SERVICE_NAME           0x0100 /* + language base (0x0100) offset 0 */
#define FLOSS_ATTR_SERVICE_DESCRIPTION    0x0101
#define FLOSS_ATTR_PROVIDER_NAME          0x0102

/* HID attributes */
#define FLOSS_ATTR_HID_PARSER_VERSION     0x0201
#define FLOSS_ATTR_HID_DEVICE_SUBCLASS    0x0202
#define FLOSS_ATTR_HID_COUNTRY_CODE       0x0203
#define FLOSS_ATTR_HID_VIRTUAL_CABLE      0x0204
#define FLOSS_ATTR_HID_RECONNECT_INITIATE 0x0205
#define FLOSS_ATTR_HID_DESCRIPTOR_LIST    0x0206
#define FLOSS_ATTR_HID_LANGID_BASE_LIST   0x0207
#define FLOSS_ATTR_HID_SDP_DISABLE        0x0208
#define FLOSS_ATTR_HID_BATTERY_POWER      0x0209
#define FLOSS_ATTR_HID_REMOTE_WAKE        0x020A
#define FLOSS_ATTR_HID_PROFILE_VERSION    0x020B
#define FLOSS_ATTR_HID_SUPERVISION_TIMEOUT 0x020C
#define FLOSS_ATTR_HID_NORMALLY_CONNECTABLE 0x020D
#define FLOSS_ATTR_HID_BOOT_DEVICE        0x020E
#define FLOSS_ATTR_HID_SSR_HOST_MAX_LATENCY 0x020F
#define FLOSS_ATTR_HID_SSR_HOST_MIN_TIMEOUT 0x0210

/* Device ID attributes */
#define FLOSS_ATTR_DID_SPECIFICATION_ID   0x0200
#define FLOSS_ATTR_DID_VENDOR_ID          0x0201
#define FLOSS_ATTR_DID_PRODUCT_ID         0x0202
#define FLOSS_ATTR_DID_VERSION            0x0203
#define FLOSS_ATTR_DID_PRIMARY_RECORD     0x0204
#define FLOSS_ATTR_DID_VENDOR_ID_SOURCE   0x0205

/* ------------------------------------------------------------------------ */
/* Data Element writer                                                      */
/* ------------------------------------------------------------------------ */

/**
 * Streaming writer for SDP data elements. Sequences are opened/closed like
 * brackets; lengths are back-patched on close (sequences use 2-byte length
 * headers internally so nesting never needs memmove).
 *
 *   floss_de_writer_t w; floss_de_init(&w, buf, sizeof buf);
 *   floss_de_seq_begin(&w);
 *     floss_de_uint16(&w, FLOSS_ATTR_SERVICE_CLASS_ID_LIST);
 *     floss_de_seq_begin(&w); floss_de_uuid16(&w, FLOSS_UUID_HID); floss_de_seq_end(&w);
 *   floss_de_seq_end(&w);
 *   if (w.error) ...  // overflow or unbalanced
 */
#define FLOSS_DE_MAX_DEPTH 6
typedef struct {
    uint8_t *buf;
    uint16_t cap;
    uint16_t len;
    uint8_t depth;
    bool error;
    uint16_t open[FLOSS_DE_MAX_DEPTH]; /**< offsets of open sequence headers */
} floss_de_writer_t;

void floss_de_init(floss_de_writer_t *w, uint8_t *buf, uint16_t cap);
void floss_de_seq_begin(floss_de_writer_t *w);
void floss_de_seq_end(floss_de_writer_t *w);
void floss_de_nil(floss_de_writer_t *w);
void floss_de_bool(floss_de_writer_t *w, bool v);
void floss_de_uint8(floss_de_writer_t *w, uint8_t v);
void floss_de_uint16(floss_de_writer_t *w, uint16_t v);
void floss_de_uint32(floss_de_writer_t *w, uint32_t v);
void floss_de_int8(floss_de_writer_t *w, int8_t v);
void floss_de_uuid16(floss_de_writer_t *w, uint16_t uuid);
void floss_de_uuid32(floss_de_writer_t *w, uint32_t uuid);
void floss_de_uuid128(floss_de_writer_t *w, const uint8_t uuid[16]); /* big-endian bytes */
/** Text string (also used for binary blobs such as the HID report descriptor). */
void floss_de_text(floss_de_writer_t *w, const uint8_t *data, uint16_t len);
void floss_de_str(floss_de_writer_t *w, const char *s);
/** Copy already-encoded element bytes verbatim. */
void floss_de_raw(floss_de_writer_t *w, const uint8_t *data, uint16_t len);

/* Data element parsing helpers (used by the server; public for tests/tools) */

/** Decode a DE header at p (avail bytes). Sets type (high 5 bits) and returns header size, data length in *len. 0 on error. */
uint16_t floss_de_header(const uint8_t *p, uint16_t avail, uint8_t *type, uint32_t *len);

/* ------------------------------------------------------------------------ */
/* Records                                                                  */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint32_t handle;        /**< assigned on registration (0x00010000+) */
    uint8_t *data;          /**< serialised attribute-list sequence (writable: handle is patched in) */
    uint16_t len;
} floss_sdp_record_t;

/**
 * Register a record. The record must start with attribute 0x0000 encoded as a
 * uint32 (the builders below do this; use floss_sdp_record_begin() when
 * building your own) - the server writes the assigned handle into it.
 * The record storage must stay valid while registered.
 */
floss_err_t floss_sdp_register(floss_sdp_record_t *rec);
floss_err_t floss_sdp_unregister(floss_sdp_record_t *rec);

/** Begin a custom record: opens the outer sequence and writes attr 0x0000 placeholder. */
void floss_sdp_record_begin(floss_de_writer_t *w);
/** Close the outer sequence. */
void floss_sdp_record_end(floss_de_writer_t *w);

/* ------------------------------------------------------------------------ */
/* Ready-made records                                                       */
/* ------------------------------------------------------------------------ */

typedef struct {
    const char *service_name;        /**< e.g. "Gamepad" */
    const char *service_description; /**< optional */
    const char *provider_name;       /**< optional */
    uint8_t subclass;                /**< 0x08 gamepad, 0x04 joystick, 0x40 keyboard, 0x80 mouse */
    uint8_t country_code;            /**< 0 = not localised */
    bool virtual_cable;
    bool reconnect_initiate;
    bool battery_power;
    bool remote_wake;
    bool normally_connectable;
    bool boot_device;
    uint16_t supervision_timeout;    /**< slots, 0 = omit */
    uint16_t ssr_host_max_latency;   /**< 0 = omit */
    uint16_t ssr_host_min_timeout;   /**< 0 = omit */
    const uint8_t *report_descriptor;
    uint16_t report_descriptor_len;
} floss_sdp_hid_params_t;

/** Build a complete HID service record into buf. Returns length or 0 on overflow. */
uint16_t floss_sdp_build_hid(uint8_t *buf, uint16_t cap, const floss_sdp_hid_params_t *p);

typedef struct {
    uint16_t vendor_id_source; /**< 1 = assigned by the radio consortium, 2 = USB-IF */
    uint16_t vendor_id;
    uint16_t product_id;
    uint16_t version;          /**< 0xJJMN for version JJ.M.N */
    bool primary;
} floss_sdp_did_params_t;

/** Build a Device ID (PnP Information) record. Returns length or 0 on overflow. */
uint16_t floss_sdp_build_did(uint8_t *buf, uint16_t cap, const floss_sdp_did_params_t *p);

/* ------------------------------------------------------------------------ */
/* Lifecycle (called by floss.c)                                            */
/* ------------------------------------------------------------------------ */

void floss_sdp_init(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_SDP_H */
