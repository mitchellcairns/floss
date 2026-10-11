/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_gatt.c - GATT attribute database: registration, lookup, value access.
 *
 * Overview
 * --------
 * The database is a list of application-owned attribute arrays ("tables").
 * floss_gatt_register() appends a table and gives it the next free block of
 * handles, so handle -> attribute is "find the table whose block contains
 * the handle, then index" - a couple of compares for the handful of tables a
 * device has. Nothing is copied and nothing is allocated; `static const`
 * tables stay in flash.
 *
 * This file knows what attribute values ARE (stored bytes, callbacks, and
 * the few values GATT generates: service UUIDs, characteristic declarations
 * and per-link CCCDs). It does not know the ATT protocol: permission checks,
 * MTU handling and PDU formats live in floss_att.c, which calls
 * floss_gatt_read_value()/floss_gatt_write_value().
 *
 * Why characteristic declarations are generated: a declaration contains the
 * handle of its value, and handles depend on registration order. Generating
 * [properties][handle + 1][UUID of the next attribute] at read time keeps the
 * tables position independent and const.
 */
#include "floss_internal.h"
#include "floss/floss_le.h"
#include "floss/floss_gatt.h"

#if FLOSS_ENABLE_LE

typedef struct {
    const floss_gatt_attr_t *attrs;
    uint16_t count;
    uint16_t base; /* handle of attrs[0] */
} gatt_table_t;

static gatt_table_t s_tables[FLOSS_GATT_MAX_TABLES];
static uint8_t s_table_count;
static uint16_t s_last_handle;

void floss_gatt_init(void) {
    memset(s_tables, 0, sizeof(s_tables));
    s_table_count = 0;
    s_last_handle = 0;
}

uint16_t floss_gatt_register(const floss_gatt_attr_t *table, uint16_t count) {
    if (table == NULL || count == 0 || s_table_count >= FLOSS_GATT_MAX_TABLES ||
        (uint32_t)s_last_handle + count > 0xFFFFu) {
        FLOSS_LOGE(LE, "gatt: cannot register table (%u attrs, %u tables)", (unsigned)count,
                   (unsigned)s_table_count);
        return 0;
    }
    gatt_table_t *t = &s_tables[s_table_count++];
    t->attrs = table;
    t->count = count;
    t->base = (uint16_t)(s_last_handle + 1);
    s_last_handle = (uint16_t)(s_last_handle + count);
    FLOSS_LOGD(LE, "gatt: table %u handles 0x%04x-0x%04x", (unsigned)(s_table_count - 1), (unsigned)t->base,
               (unsigned)s_last_handle);
    return t->base;
}

const floss_gatt_attr_t *floss_gatt_attr(uint16_t handle) {
    if (handle == 0 || handle > s_last_handle) {
        return NULL;
    }
    for (uint8_t i = 0; i < s_table_count; i++) {
        const gatt_table_t *t = &s_tables[i];
        if (handle < t->base + t->count) {
            return &t->attrs[handle - t->base];
        }
    }
    return NULL;
}

uint16_t floss_gatt_last_handle(void) {
    return s_last_handle;
}

uint16_t floss_gatt_find(uint16_t start, uint16_t end, uint16_t type16) {
    if (start == 0) {
        start = 1;
    }
    for (uint32_t h = start; h <= end && h <= s_last_handle; h++) {
        const floss_gatt_attr_t *a = floss_gatt_attr((uint16_t)h);
        if (a && floss_gatt_type16(a) == type16) {
            return (uint16_t)h;
        }
    }
    return 0;
}

/** Declarations end a characteristic's descriptor list. */
static bool is_declaration(uint16_t type16) {
    return type16 == FLOSS_GATT_UUID_PRIMARY_SERVICE || type16 == FLOSS_GATT_UUID_SECONDARY_SERVICE ||
           type16 == FLOSS_GATT_UUID_INCLUDE || type16 == FLOSS_GATT_UUID_CHARACTERISTIC;
}

uint16_t floss_gatt_cccd_handle(uint16_t value_handle) {
    for (uint32_t h = (uint32_t)value_handle + 1; h <= s_last_handle; h++) {
        const floss_gatt_attr_t *a = floss_gatt_attr((uint16_t)h);
        uint16_t t = floss_gatt_type16(a);
        if (t == FLOSS_GATT_UUID_CCCD) {
            return (uint16_t)h;
        }
        if (is_declaration(t)) {
            break;
        }
    }
    return 0;
}

uint16_t floss_gatt_cccd_get(uint16_t conn, uint16_t value_handle) {
    int idx = floss_le_link_index(conn);
    uint16_t h = floss_gatt_cccd_handle(value_handle);
    if (idx < 0 || h == 0) {
        return 0;
    }
    const floss_gatt_cccd_t *store = (const floss_gatt_cccd_t *)floss_gatt_attr(h)->value;
    return store ? store->v[idx] : 0;
}

void floss_gatt_on_link_down(uint8_t link_index) {
    if (link_index >= FLOSS_MAX_CONNECTIONS) {
        return;
    }
    for (uint8_t i = 0; i < s_table_count; i++) {
        const gatt_table_t *t = &s_tables[i];
        for (uint16_t j = 0; j < t->count; j++) {
            const floss_gatt_attr_t *a = &t->attrs[j];
            if (floss_gatt_type16(a) == FLOSS_GATT_UUID_CCCD && a->value) {
                ((floss_gatt_cccd_t *)a->value)->v[link_index] = 0;
            }
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Value access                                                             */
/* ------------------------------------------------------------------------ */

/** Current length of a stored value (clamped to its capacity). */
static uint16_t stored_len(const floss_gatt_attr_t *a) {
    if (a->value == NULL) {
        return 0;
    }
    if (a->len == NULL) {
        return a->max_len;
    }
    return FLOSS_MIN(*a->len, a->max_len);
}

uint16_t floss_gatt_value_max_len(uint16_t handle) {
    const floss_gatt_attr_t *a = floss_gatt_attr(handle);
    if (a == NULL) {
        return 0;
    }
    if (a->read) {
        return a->max_len ? a->max_len : 0xFFFF;
    }
    switch (floss_gatt_type16(a)) {
    case FLOSS_GATT_UUID_PRIMARY_SERVICE:
    case FLOSS_GATT_UUID_SECONDARY_SERVICE:
        return a->value ? 16 : 2;
    case FLOSS_GATT_UUID_CHARACTERISTIC: {
        const floss_gatt_attr_t *v = floss_gatt_attr((uint16_t)(handle + 1));
        return (v && v->uuid128) ? 19 : 5;
    }
    case FLOSS_GATT_UUID_CCCD:
        return 2;
    default:
        return a->max_len;
    }
}

int floss_gatt_read_value(uint16_t conn, uint16_t handle, uint16_t offset, uint8_t *out, uint16_t cap) {
    const floss_gatt_attr_t *a = floss_gatt_attr(handle);
    if (a == NULL) {
        return -FLOSS_ATT_ERR_INVALID_HANDLE;
    }
    if (a->read) {
        int n = a->read(conn, handle, a, offset, out, cap);
        /* Callers size buffers by cap: never trust a callback's count past it. */
        return n > (int)cap ? (int)cap : n;
    }

    uint8_t tmp[19]; /* largest generated value: 128-bit characteristic declaration */
    const uint8_t *src;
    uint16_t len;

    switch (floss_gatt_type16(a)) {
    case FLOSS_GATT_UUID_PRIMARY_SERVICE:
    case FLOSS_GATT_UUID_SECONDARY_SERVICE:
        if (a->value) {
            src = (const uint8_t *)a->value; /* 128-bit service UUID */
            len = 16;
        } else {
            floss_put_le16(tmp, a->aux);
            src = tmp;
            len = 2;
        }
        break;
    case FLOSS_GATT_UUID_CHARACTERISTIC: {
        const floss_gatt_attr_t *v = floss_gatt_attr((uint16_t)(handle + 1));
        if (v == NULL) {
            return -FLOSS_ATT_ERR_UNLIKELY; /* malformed table: declaration without value */
        }
        tmp[0] = (uint8_t)a->aux;
        floss_put_le16(&tmp[1], (uint16_t)(handle + 1));
        if (v->uuid128) {
            memcpy(&tmp[3], v->uuid128, 16);
            len = 19;
        } else {
            floss_put_le16(&tmp[3], v->type);
            len = 5;
        }
        src = tmp;
        break;
    }
    case FLOSS_GATT_UUID_CCCD: {
        int idx = floss_le_link_index(conn);
        const floss_gatt_cccd_t *store = (const floss_gatt_cccd_t *)a->value;
        floss_put_le16(tmp, (idx >= 0 && store) ? store->v[idx] : 0);
        src = tmp;
        len = 2;
        break;
    }
    default:
        src = (const uint8_t *)a->value;
        len = stored_len(a);
        break;
    }

    if (offset > len) {
        return -FLOSS_ATT_ERR_INVALID_OFFSET;
    }
    uint16_t n = FLOSS_MIN((uint16_t)(len - offset), cap);
    if (n) {
        memcpy(out, src + offset, n);
    }
    return n;
}

/* ------------------------------------------------------------------------ */
/* CCCD persistence for bonded peers                                        */
/* ------------------------------------------------------------------------ */

/*
 * Record (stored at FLOSS_GATT_STORE_KEY_CCCD_BASE + bond index):
 *   [peer addr type][peer addr 6] then up to CCCD_MAX_SAVED x [handle 2][value 2]
 * The identity prefix guards against a bond slot that was reused by another
 * peer since the record was written.
 */
#define CCCD_MAX_SAVED 13 /* 7 + 13*4 = 59 bytes: fits the HAL's small-value limit */

/** Bond slot index of the link's peer, or -1 if the peer is not bonded. */
static int cccd_bond_slot(uint16_t conn, const floss_smp_bond_t **out) {
    const floss_le_link_t *l = floss_le_link(conn);
    if (l == NULL || l->security_level < 2) {
        return -1;
    }
    const floss_smp_bond_t *b = floss_smp_resolve(l->peer_addr_type, &l->peer);
    if (b == NULL) {
        return -1;
    }
    for (uint8_t i = 0; i < floss_smp_bond_count(); i++) {
        if (floss_smp_bond_at(i) == b) {
            *out = b;
            return i;
        }
    }
    return -1;
}

static void cccd_save(uint16_t conn, int link_index, int slot, const floss_smp_bond_t *b) {
    uint8_t rec[7 + CCCD_MAX_SAVED * 4];
    uint16_t n = 7;
    rec[0] = b->addr_type;
    memcpy(&rec[1], b->addr.b, 6);
    for (uint32_t h = 1; h <= s_last_handle && (size_t)n + 4u <= sizeof rec; h++) {
        const floss_gatt_attr_t *a = floss_gatt_attr((uint16_t)h);
        if (floss_gatt_type16(a) == FLOSS_GATT_UUID_CCCD && a->value) {
            uint16_t v = ((const floss_gatt_cccd_t *)a->value)->v[link_index];
            if (v) {
                floss_put_le16(&rec[n], (uint16_t)h);
                floss_put_le16(&rec[n + 2], v);
                n = (uint16_t)(n + 4);
            }
        }
    }
    floss_hal_store_write((uint16_t)(FLOSS_GATT_STORE_KEY_CCCD_BASE + slot), rec, n);
    FLOSS_LOGD(LE, "gatt: saved %u CCCD(s) for bond %d", (unsigned)((n - 7) / 4), slot);
}

static void cccd_restore(uint16_t conn, int link_index, int slot, const floss_smp_bond_t *b) {
    uint8_t rec[7 + CCCD_MAX_SAVED * 4];
    int n = floss_hal_store_read((uint16_t)(FLOSS_GATT_STORE_KEY_CCCD_BASE + slot), rec, sizeof rec);
    if (n < 7 || rec[0] != b->addr_type || memcmp(&rec[1], b->addr.b, 6) != 0) {
        return;
    }
    unsigned restored = 0;
    for (int off = 7; off + 4 <= n; off += 4) {
        uint16_t h = floss_get_le16(&rec[off]);
        const floss_gatt_attr_t *a = floss_gatt_attr(h);
        if (a && floss_gatt_type16(a) == FLOSS_GATT_UUID_CCCD && a->value) {
            ((floss_gatt_cccd_t *)a->value)->v[link_index] =
                (uint16_t)(floss_get_le16(&rec[off + 2]) & (FLOSS_GATT_CCCD_NOTIFY | FLOSS_GATT_CCCD_INDICATE));
            restored++;
        }
    }
    FLOSS_LOGI(LE, "gatt: restored %u CCCD(s) for bonded peer (link 0x%04x)", restored, (unsigned)conn);
}

void floss_gatt_cccd_sync(uint16_t conn) {
    int idx = floss_le_link_index(conn);
    const floss_smp_bond_t *b = NULL;
    int slot = cccd_bond_slot(conn, &b);
    if (idx < 0 || slot < 0) {
        return;
    }
    bool any = false;
    for (uint32_t h = 1; h <= s_last_handle && !any; h++) {
        const floss_gatt_attr_t *a = floss_gatt_attr((uint16_t)h);
        any = floss_gatt_type16(a) == FLOSS_GATT_UUID_CCCD && a->value &&
              ((const floss_gatt_cccd_t *)a->value)->v[idx] != 0;
    }
    if (any) {
        cccd_save(conn, idx, slot, b);
    } else {
        cccd_restore(conn, idx, slot, b);
    }
}

/** CCCD write: per-link storage, exactly 2 bytes, offset 0. */
static uint8_t write_cccd(uint16_t conn, uint16_t handle, const floss_gatt_attr_t *a, uint16_t offset,
                          const uint8_t *data, uint16_t len, uint8_t flags) {
    if (offset != 0) {
        return FLOSS_ATT_ERR_INVALID_OFFSET;
    }
    if (len != 2) {
        return FLOSS_ATT_ERR_INVALID_VALUE_LEN;
    }
    int idx = floss_le_link_index(conn);
    if (idx < 0 || a->value == NULL) {
        return FLOSS_ATT_ERR_UNLIKELY;
    }
    /* Reserved bits are ignored rather than rejected (forward compatible). */
    uint16_t v = (uint16_t)(floss_get_le16(data) & (FLOSS_GATT_CCCD_NOTIFY | FLOSS_GATT_CCCD_INDICATE));
    if (a->write) {
        uint8_t le_val[2];
        floss_put_le16(le_val, v);
        uint8_t err = a->write(conn, handle, a, 0, le_val, 2, flags);
        if (err) {
            return err;
        }
    }
    ((floss_gatt_cccd_t *)a->value)->v[idx] = v;
    FLOSS_LOGD(LE, "gatt: cccd 0x%04x = 0x%04x (link 0x%04x)", (unsigned)handle, (unsigned)v, (unsigned)conn);
    /* Bonded peer: remember it (a no-op until the link is encrypted with a bond). */
    const floss_smp_bond_t *b = NULL;
    int slot = cccd_bond_slot(conn, &b);
    if (slot >= 0) {
        cccd_save(conn, idx, slot, b);
    }
    return 0;
}

uint8_t floss_gatt_write_value(uint16_t conn, uint16_t handle, uint16_t offset, const uint8_t *data, uint16_t len,
                               uint8_t flags) {
    const floss_gatt_attr_t *a = floss_gatt_attr(handle);
    if (a == NULL) {
        return FLOSS_ATT_ERR_INVALID_HANDLE;
    }
    uint16_t t = floss_gatt_type16(a);
    if (t == FLOSS_GATT_UUID_CCCD) {
        return write_cccd(conn, handle, a, offset, data, len, flags);
    }
    if (is_declaration(t) || (a->value == NULL && a->write == NULL)) {
        return FLOSS_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    if (a->value) {
        /* Bound checks for stored values. Fixed-length values (len == NULL)
         * must be written in full; variable ones may shrink or grow up to
         * max_len, and a write at `offset` replaces everything from there. */
        if (offset > stored_len(a)) {
            return FLOSS_ATT_ERR_INVALID_OFFSET;
        }
        if ((uint32_t)offset + len > a->max_len) {
            return FLOSS_ATT_ERR_INVALID_VALUE_LEN;
        }
        if (a->len == NULL && (uint32_t)offset + len != a->max_len) {
            return FLOSS_ATT_ERR_INVALID_VALUE_LEN;
        }
    }
    if (a->write) {
        uint8_t err = a->write(conn, handle, a, offset, data, len, flags);
        if (err) {
            return err;
        }
    }
    if (a->value) {
        if (len) {
            memcpy((uint8_t *)a->value + offset, data, len);
        }
        if (a->len) {
            *a->len = (uint16_t)(offset + len);
        }
    }
    return 0;
}

#endif /* FLOSS_ENABLE_LE */
