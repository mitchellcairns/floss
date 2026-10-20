/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_gatt.h - GATT server: static attribute database + ATT protocol.
 *
 * THE DATABASE
 * ------------
 * The application (or a profile such as HOGP) declares its services as plain
 * arrays of floss_gatt_attr_t, usually `static const` so they live in flash,
 * and registers each array once at start-up. Arrays are concatenated into one
 * handle space in registration order: handle = base + index, contiguous, so a
 * handle lookup is an index operation. The GAP (0x1800) and GATT (0x1801)
 * services are registered by the stack first and always start at handle 1.
 *
 *     static uint8_t level = 100;
 *     static floss_gatt_cccd_t level_cccd;
 *     static const floss_gatt_attr_t battery[] = {
 *         FLOSS_GATT_PRIMARY_SERVICE16(FLOSS_GATT_SVC_BATTERY),
 *         FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_READ | FLOSS_GATT_PROP_NOTIFY),
 *         FLOSS_GATT_VALUE16(FLOSS_GATT_CHR_BATTERY_LEVEL, FLOSS_GATT_P_READ, &level, 1, NULL),
 *         FLOSS_GATT_CCCD(level_cccd),
 *     };
 *     uint16_t base = floss_gatt_register(battery, FLOSS_ARRAY_SIZE(battery));
 *     // value handle = base + 2
 *
 * Two values are generated rather than stored, which is what keeps tables
 * const and position independent:
 *  - a service declaration's value is its `aux` UUID (or the 16-byte UUID in
 *    `value` for 128-bit services);
 *  - a characteristic declaration's value [properties][value handle][UUID] is
 *    built from `aux` (properties) and the attribute right after it, which by
 *    convention is the characteristic value.
 * CCCDs keep one 16-bit value per link (floss_gatt_cccd_t), cleared when the
 * link drops.
 *
 * Values are either plain storage (`value`, `max_len`, optional `len`) or
 * served by callbacks (`read` / `write`). With both, `write` is called first
 * (it may reject with an ATT error) and the bytes are stored afterwards.
 *
 * NOTIFICATIONS
 * -------------
 * floss_gatt_notify() is zero-copy: it allocates one pool buffer, writes the
 * ATT header and value at buf->data, pushes the L2CAP header in the headroom
 * and hands it to HCI. floss_gatt_notify_alloc()/send() expose the same path
 * for producers that want to write the value in place (HOGP does).
 */
#ifndef FLOSS_GATT_H
#define FLOSS_GATT_H

#include "floss_types.h"
#include "floss_buf.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Sizing                                                                   */
/* ------------------------------------------------------------------------ */

/** Max attribute tables registered at once (GAP + GATT + app/profile services). */
#ifndef FLOSS_GATT_MAX_TABLES
#define FLOSS_GATT_MAX_TABLES 8
#endif

/** Prepare Write queue capacity (one attribute at a time, shared by all links). */
#ifndef FLOSS_ATT_PREPARE_MAX
#define FLOSS_ATT_PREPARE_MAX 512
#endif

/** Default ATT MTU before Exchange MTU. */
#define FLOSS_ATT_DEFAULT_MTU 23
/** ATT transaction timeout (spec: 30 s). */
#define FLOSS_ATT_TIMEOUT_US 30000000u
/** Longest attribute value the protocol allows. */
#define FLOSS_ATT_MAX_VALUE 512

/* ------------------------------------------------------------------------ */
/* ATT opcodes and error codes                                              */
/* ------------------------------------------------------------------------ */

#define FLOSS_ATT_ERROR_RSP             0x01
#define FLOSS_ATT_MTU_REQ               0x02
#define FLOSS_ATT_MTU_RSP               0x03
#define FLOSS_ATT_FIND_INFO_REQ         0x04
#define FLOSS_ATT_FIND_INFO_RSP         0x05
#define FLOSS_ATT_FIND_BY_TYPE_REQ      0x06
#define FLOSS_ATT_FIND_BY_TYPE_RSP      0x07
#define FLOSS_ATT_READ_BY_TYPE_REQ      0x08
#define FLOSS_ATT_READ_BY_TYPE_RSP      0x09
#define FLOSS_ATT_READ_REQ              0x0A
#define FLOSS_ATT_READ_RSP              0x0B
#define FLOSS_ATT_READ_BLOB_REQ         0x0C
#define FLOSS_ATT_READ_BLOB_RSP         0x0D
#define FLOSS_ATT_READ_MULTI_REQ        0x0E
#define FLOSS_ATT_READ_MULTI_RSP        0x0F
#define FLOSS_ATT_READ_BY_GROUP_REQ     0x10
#define FLOSS_ATT_READ_BY_GROUP_RSP     0x11
#define FLOSS_ATT_WRITE_REQ             0x12
#define FLOSS_ATT_WRITE_RSP             0x13
#define FLOSS_ATT_PREPARE_WRITE_REQ     0x16
#define FLOSS_ATT_PREPARE_WRITE_RSP     0x17
#define FLOSS_ATT_EXECUTE_WRITE_REQ     0x18
#define FLOSS_ATT_EXECUTE_WRITE_RSP     0x19
#define FLOSS_ATT_NOTIFY                0x1B
#define FLOSS_ATT_INDICATE              0x1D
#define FLOSS_ATT_CONFIRM               0x1E
#define FLOSS_ATT_WRITE_CMD             0x52
#define FLOSS_ATT_SIGNED_WRITE_CMD      0xD2
/** Bit 6 of the opcode: command (no response, never an error response). */
#define FLOSS_ATT_OP_COMMAND_FLAG       0x40

#define FLOSS_ATT_ERR_INVALID_HANDLE        0x01
#define FLOSS_ATT_ERR_READ_NOT_PERMITTED    0x02
#define FLOSS_ATT_ERR_WRITE_NOT_PERMITTED   0x03
#define FLOSS_ATT_ERR_INVALID_PDU           0x04
#define FLOSS_ATT_ERR_INSUFFICIENT_AUTHEN   0x05
#define FLOSS_ATT_ERR_REQUEST_NOT_SUPPORTED 0x06
#define FLOSS_ATT_ERR_INVALID_OFFSET        0x07
#define FLOSS_ATT_ERR_INSUFFICIENT_AUTHOR   0x08
#define FLOSS_ATT_ERR_PREPARE_QUEUE_FULL    0x09
#define FLOSS_ATT_ERR_ATTR_NOT_FOUND        0x0A
#define FLOSS_ATT_ERR_ATTR_NOT_LONG         0x0B
#define FLOSS_ATT_ERR_INSUFFICIENT_KEY_SIZE 0x0C
#define FLOSS_ATT_ERR_INVALID_VALUE_LEN     0x0D
#define FLOSS_ATT_ERR_UNLIKELY              0x0E
#define FLOSS_ATT_ERR_INSUFFICIENT_ENC      0x0F
#define FLOSS_ATT_ERR_UNSUPPORTED_GROUP     0x10
#define FLOSS_ATT_ERR_INSUFFICIENT_RESOURCES 0x11
#define FLOSS_ATT_ERR_VALUE_NOT_ALLOWED     0x13
/** Common profile error: CCCD improperly configured. */
#define FLOSS_ATT_ERR_CCCD_IMPROPER         0xFD

/* ------------------------------------------------------------------------ */
/* Assigned numbers used by FLOSS                                           */
/* ------------------------------------------------------------------------ */

/* Declarations and descriptors */
#define FLOSS_GATT_UUID_PRIMARY_SERVICE    0x2800
#define FLOSS_GATT_UUID_SECONDARY_SERVICE  0x2801
#define FLOSS_GATT_UUID_INCLUDE            0x2802
#define FLOSS_GATT_UUID_CHARACTERISTIC     0x2803
#define FLOSS_GATT_UUID_CHAR_EXT_PROPS     0x2900
#define FLOSS_GATT_UUID_CHAR_USER_DESC     0x2901
#define FLOSS_GATT_UUID_CCCD               0x2902
#define FLOSS_GATT_UUID_EXT_REPORT_REF     0x2907
#define FLOSS_GATT_UUID_REPORT_REF         0x2908

/* Services */
#define FLOSS_GATT_SVC_GAP                 0x1800
#define FLOSS_GATT_SVC_GATT                0x1801
#define FLOSS_GATT_SVC_DEVICE_INFO         0x180A
#define FLOSS_GATT_SVC_BATTERY             0x180F
#define FLOSS_GATT_SVC_HID                 0x1812

/* Characteristics */
#define FLOSS_GATT_CHR_DEVICE_NAME         0x2A00
#define FLOSS_GATT_CHR_APPEARANCE          0x2A01
#define FLOSS_GATT_CHR_PPCP                0x2A04
#define FLOSS_GATT_CHR_SERVICE_CHANGED     0x2A05
#define FLOSS_GATT_CHR_BATTERY_LEVEL       0x2A19
#define FLOSS_GATT_CHR_MODEL_NUMBER        0x2A24
#define FLOSS_GATT_CHR_SERIAL_NUMBER       0x2A25
#define FLOSS_GATT_CHR_FIRMWARE_REV        0x2A26
#define FLOSS_GATT_CHR_MANUFACTURER_NAME   0x2A29
#define FLOSS_GATT_CHR_BOOT_KB_INPUT       0x2A22
#define FLOSS_GATT_CHR_BOOT_KB_OUTPUT      0x2A32
#define FLOSS_GATT_CHR_BOOT_MOUSE_INPUT    0x2A33
#define FLOSS_GATT_CHR_HID_INFORMATION     0x2A4A
#define FLOSS_GATT_CHR_REPORT_MAP          0x2A4B
#define FLOSS_GATT_CHR_HID_CONTROL_POINT   0x2A4C
#define FLOSS_GATT_CHR_REPORT              0x2A4D
#define FLOSS_GATT_CHR_PROTOCOL_MODE       0x2A4E
#define FLOSS_GATT_CHR_PNP_ID              0x2A50

/* Characteristic properties (in the declaration) */
#define FLOSS_GATT_PROP_BROADCAST     0x01
#define FLOSS_GATT_PROP_READ          0x02
#define FLOSS_GATT_PROP_WRITE_NO_RSP  0x04
#define FLOSS_GATT_PROP_WRITE         0x08
#define FLOSS_GATT_PROP_NOTIFY        0x10
#define FLOSS_GATT_PROP_INDICATE      0x20
#define FLOSS_GATT_PROP_SIGNED_WRITE  0x40
#define FLOSS_GATT_PROP_EXT_PROPS     0x80

/* CCCD bits */
#define FLOSS_GATT_CCCD_NOTIFY   0x0001
#define FLOSS_GATT_CCCD_INDICATE 0x0002

/* ------------------------------------------------------------------------ */
/* Attributes                                                               */
/* ------------------------------------------------------------------------ */

/*
 * Permissions (what the server enforces; the declaration's properties are
 * only what it advertises). Security levels come from SMP:
 * ENC needs level >= 2 (encrypted), AUTHEN needs level >= 3 (MITM-protected).
 */
#define FLOSS_GATT_P_READ          0x0001
#define FLOSS_GATT_P_WRITE         0x0002
#define FLOSS_GATT_P_READ_ENC      0x0004
#define FLOSS_GATT_P_WRITE_ENC     0x0008
#define FLOSS_GATT_P_READ_AUTHEN   0x0010
#define FLOSS_GATT_P_WRITE_AUTHEN  0x0020
/* Shorthands */
#define FLOSS_GATT_P_RD_ENC   (FLOSS_GATT_P_READ | FLOSS_GATT_P_READ_ENC)
#define FLOSS_GATT_P_WR_ENC   (FLOSS_GATT_P_WRITE | FLOSS_GATT_P_WRITE_ENC)
#define FLOSS_GATT_P_RW       (FLOSS_GATT_P_READ | FLOSS_GATT_P_WRITE)
#define FLOSS_GATT_P_RW_ENC   (FLOSS_GATT_P_RD_ENC | FLOSS_GATT_P_WR_ENC)

/* Flags passed to write callbacks */
#define FLOSS_GATT_WRITE_F_CMD      0x01 /**< Write Command: no response will be sent */
#define FLOSS_GATT_WRITE_F_PREPARED 0x02 /**< result of Execute Write (long write) */

typedef struct floss_gatt_attr floss_gatt_attr_t;

/**
 * Read callback. Copy the value starting at `offset` into out (at most cap
 * bytes). Return the number of bytes copied, or -FLOSS_ATT_ERR_* (e.g.
 * -FLOSS_ATT_ERR_INVALID_OFFSET when offset is past the end).
 * attr_handle is the absolute handle; attr->user carries your context.
 */
typedef int (*floss_gatt_read_fn)(uint16_t conn, uint16_t attr_handle, const floss_gatt_attr_t *attr,
                                  uint16_t offset, uint8_t *out, uint16_t cap);

/**
 * Write callback. Return 0 to accept or a FLOSS_ATT_ERR_* code to reject.
 * flags = FLOSS_GATT_WRITE_F_*. Called before the bytes are stored (if the
 * attribute also has storage).
 */
typedef uint8_t (*floss_gatt_write_fn)(uint16_t conn, uint16_t attr_handle, const floss_gatt_attr_t *attr,
                                       uint16_t offset, const uint8_t *data, uint16_t len, uint8_t flags);

struct floss_gatt_attr {
    uint16_t type;            /**< 16-bit attribute type (ignored when uuid128 != NULL) */
    uint16_t perms;           /**< FLOSS_GATT_P_* */
    const uint8_t *uuid128;   /**< 128-bit type, little-endian (wire order), or NULL */
    void *value;              /**< storage (const data is fine for read-only attributes) */
    uint16_t max_len;         /**< storage capacity / fixed length */
    uint16_t *len;            /**< current length; NULL = always max_len */
    floss_gatt_read_fn read;  /**< optional: serve reads */
    floss_gatt_write_fn write;/**< optional: validate/consume writes */
    uint16_t aux;             /**< service decl: 16-bit UUID; characteristic decl: properties */
    void *user;               /**< callback context */
};

/** Per-link CCCD storage (index = floss_le_link_index()). */
typedef struct {
    uint16_t v[FLOSS_MAX_CONNECTIONS];
} floss_gatt_cccd_t;

/*
 * Cast helper: `value` is non-const because some values are writable RAM;
 * const data is only ever read (the server checks FLOSS_GATT_P_WRITE first).
 * A plain cast keeps the initialiser an address constant.
 */
#define FLOSS_GATT__PTR(p) ((void *)(p))

/* ---- table building macros ---------------------------------------------- */

#define FLOSS_GATT_PRIMARY_SERVICE16(uuid16) \
    { .type = FLOSS_GATT_UUID_PRIMARY_SERVICE, .perms = FLOSS_GATT_P_READ, .aux = (uuid16) }

/** uuid128: 16 bytes, little-endian. */
#define FLOSS_GATT_PRIMARY_SERVICE128(uuid128_bytes) \
    { .type = FLOSS_GATT_UUID_PRIMARY_SERVICE, .perms = FLOSS_GATT_P_READ, \
      .value = FLOSS_GATT__PTR(uuid128_bytes), .max_len = 16 }

#define FLOSS_GATT_SECONDARY_SERVICE16(uuid16) \
    { .type = FLOSS_GATT_UUID_SECONDARY_SERVICE, .perms = FLOSS_GATT_P_READ, .aux = (uuid16) }

/** Characteristic declaration; the NEXT entry must be its value attribute. */
#define FLOSS_GATT_CHARACTERISTIC(props) \
    { .type = FLOSS_GATT_UUID_CHARACTERISTIC, .perms = FLOSS_GATT_P_READ, .aux = (props) }

/** Value with storage. lenp may be NULL (fixed length = maxlen). */
#define FLOSS_GATT_VALUE16(uuid16, perm, ptr, maxlen, lenp) \
    { .type = (uuid16), .perms = (perm), .value = FLOSS_GATT__PTR(ptr), .max_len = (maxlen), .len = (lenp) }

/** Value served by callbacks (either may be NULL). maxlen: largest value (0 = unknown). */
#define FLOSS_GATT_VALUE16_CB(uuid16, perm, rd, wr, maxlen, usr) \
    { .type = (uuid16), .perms = (perm), .read = (rd), .write = (wr), .max_len = (maxlen), .user = (usr) }

/** Value with a 128-bit type (uuid128: 16 bytes little-endian). */
#define FLOSS_GATT_VALUE128(uuid128_bytes, perm, ptr, maxlen, lenp) \
    { .uuid128 = (uuid128_bytes), .perms = (perm), .value = FLOSS_GATT__PTR(ptr), .max_len = (maxlen), .len = (lenp) }

/** CCCD; store is a floss_gatt_cccd_t lvalue. Readable/writable without security. */
#define FLOSS_GATT_CCCD(store) FLOSS_GATT_CCCD_P(store, FLOSS_GATT_P_RW)

/** CCCD with explicit permissions (e.g. FLOSS_GATT_P_READ | FLOSS_GATT_P_WR_ENC). */
#define FLOSS_GATT_CCCD_P(store, perm) \
    { .type = FLOSS_GATT_UUID_CCCD, .perms = (perm), .value = FLOSS_GATT__PTR(&(store)), .max_len = 2 }

/** Constant read-only descriptor (e.g. Report Reference: 2 bytes {id, type}). */
#define FLOSS_GATT_DESCRIPTOR16(uuid16, perm, ptr, length) \
    { .type = (uuid16), .perms = (perm), .value = FLOSS_GATT__PTR(ptr), .max_len = (length) }

/* ------------------------------------------------------------------------ */
/* Database                                                                 */
/* ------------------------------------------------------------------------ */

/**
 * Append a table to the database. The table must stay valid forever.
 * @return the handle of table[0], or 0 if the table registry is full.
 * Register everything before floss_start(): handles must not change while a
 * client may have cached them.
 */
uint16_t floss_gatt_register(const floss_gatt_attr_t *table, uint16_t count);

/**
 * Empty the database, the stack's own GAP and GATT services included, so the
 * next table registered starts at handle 1: for a peer that expects another
 * device's exact handles (the table then carries its own GAP service). Only
 * while no link is up.
 */
void floss_gatt_clear(void);

/** Attribute for an absolute handle, NULL if out of range. */
const floss_gatt_attr_t *floss_gatt_attr(uint16_t handle);
/** Highest handle in use (0 if empty). */
uint16_t floss_gatt_last_handle(void);

/**
 * Find the first attribute of `type16` in [start, end]. Returns its handle or
 * 0. Typical use: locate a characteristic value by UUID after registration.
 */
uint16_t floss_gatt_find(uint16_t start, uint16_t end, uint16_t type16);

/** CCCD value of the characteristic whose value handle is `value_handle` on link `conn` (0 if none). */
uint16_t floss_gatt_cccd_get(uint16_t conn, uint16_t value_handle);

/** Storage key range for per-bond CCCD values (+ LE bond index). */
#define FLOSS_GATT_STORE_KEY_CCCD_BASE 0x0300

/**
 * Keep CCCD values of bonded peers across connections (required: a bonded
 * client does not rewrite them on reconnection). Called by LE whenever the
 * link's security changes. If the peer is bonded: a link that has CCCDs set
 * saves them, a link with none set (fresh reconnection) restores them.
 */
void floss_gatt_cccd_sync(uint16_t conn);
/** Handle of the CCCD belonging to `value_handle` (0 if none). */
uint16_t floss_gatt_cccd_handle(uint16_t value_handle);

/* ---- value access used by the ATT server (public for tests/tools) ------- */

/** Attribute type as a 16-bit UUID, or 0 if it is a 128-bit type. */
static inline uint16_t floss_gatt_type16(const floss_gatt_attr_t *a) {
    return a->uuid128 ? 0 : a->type;
}
/**
 * Read a value (generated, stored or via callback) from `offset` into out.
 * No permission checks. Returns bytes copied or -FLOSS_ATT_ERR_*.
 */
int floss_gatt_read_value(uint16_t conn, uint16_t handle, uint16_t offset, uint8_t *out, uint16_t cap);
/** Largest length the value can have (for long-read decisions); 0xFFFF if unknown. */
uint16_t floss_gatt_value_max_len(uint16_t handle);
/** Write a value (callback, then storage). No permission checks. Returns 0 or FLOSS_ATT_ERR_*. */
uint8_t floss_gatt_write_value(uint16_t conn, uint16_t handle, uint16_t offset, const uint8_t *data, uint16_t len,
                               uint8_t flags);

/* ------------------------------------------------------------------------ */
/* Server-initiated PDUs                                                    */
/* ------------------------------------------------------------------------ */

/**
 * Notify `value_handle` on link `conn` if the client enabled notifications
 * (CCCD bit 0). The value is truncated to ATT_MTU - 3.
 * @return FLOSS_OK, FLOSS_ERR_STATE (not subscribed / no link),
 *         FLOSS_ERR_BUSY (no buffer).
 */
floss_err_t floss_gatt_notify(uint16_t conn, uint16_t value_handle, const uint8_t *data, uint16_t len);

/** Notify every link that subscribed. Returns the number of links notified. */
uint8_t floss_gatt_notify_all(uint16_t value_handle, const uint8_t *data, uint16_t len);

/**
 * Zero-copy notification, step 1: allocate a buffer with the ATT header
 * written. *value points where the value goes, *cap = ATT_MTU - 3.
 * Returns NULL if no link / no buffer. Does NOT check the CCCD.
 */
floss_buf_t *floss_gatt_notify_alloc(uint16_t conn, uint16_t value_handle, uint8_t **value, uint16_t *cap);
/** Step 2: send `len` value bytes. Takes ownership. flags = FLOSS_BUF_F_*. */
floss_err_t floss_gatt_notify_send(floss_buf_t *buf, uint16_t len, uint8_t flags);

/**
 * Indicate (acknowledged). One indication may be outstanding per link:
 * FLOSS_ERR_BUSY while waiting for the confirmation. Completion is reported
 * as FLOSS_LE_EVT_INDICATION_DONE (30 s timeout closes the link per spec).
 */
floss_err_t floss_gatt_indicate(uint16_t conn, uint16_t value_handle, const uint8_t *data, uint16_t len);
/** True while an indication on `conn` awaits its confirmation. */
bool floss_gatt_indication_pending(uint16_t conn);

/* ------------------------------------------------------------------------ */
/* Standard services (floss_gatt_services.c)                                */
/* ------------------------------------------------------------------------ */

/**
 * GAP (0x1800: Device Name, Appearance, Peripheral Preferred Connection
 * Parameters) and GATT (0x1801: Service Changed + CCCD). Registered by the
 * stack at init, values come from floss_le_name()/appearance()/link policy.
 */
void floss_gatt_services_register_core(void);

/** Device Information service parameters (strings may be NULL to omit). */
typedef struct {
    const char *manufacturer;  /**< Manufacturer Name String */
    const char *model;         /**< Model Number String */
    const char *firmware;      /**< Firmware Revision String */
    uint8_t vendor_id_source;  /**< PnP ID: 1 = radio consortium assigned, 2 = USB-IF (0 = omit PnP ID) */
    uint16_t vendor_id;
    uint16_t product_id;
    uint16_t version;
} floss_gatt_dis_t;

/** Register Device Information (0x180A). cfg strings must stay valid. Once only. */
floss_err_t floss_gatt_dis_register(const floss_gatt_dis_t *cfg);

/** Register Battery (0x180F: Battery Level read + notify). Once only. */
floss_err_t floss_gatt_battery_register(uint8_t level);
/** Update the battery level (0..100) and notify subscribed links. */
void floss_gatt_battery_set(uint8_t level);
/** Value handle of the Battery Level characteristic (0 if not registered). */
uint16_t floss_gatt_battery_handle(void);

/* ------------------------------------------------------------------------ */
/* Lifecycle (called by the LE module)                                      */
/* ------------------------------------------------------------------------ */

/** Empty the database (floss_le_init calls this, then registers GAP/GATT). */
void floss_gatt_init(void);
/** Clear the per-link CCCD slot `link_index` (link down). */
void floss_gatt_on_link_down(uint8_t link_index);

void floss_att_init(void);
void floss_att_on_link_up(uint16_t handle);
void floss_att_on_link_down(uint16_t handle);
/** ATT PDU (payload after the L2CAP header) received on CID 0x0004. */
void floss_att_on_pdu(uint16_t handle, const uint8_t *pdu, uint16_t len);
/** Retry deferred responses (no buffer earlier). Called on poll and credits. */
void floss_att_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_GATT_H */
