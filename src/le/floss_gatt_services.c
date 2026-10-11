/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_gatt_services.c - Standard services: GAP, GATT, Device Information,
 * Battery.
 *
 * Overview
 * --------
 * GAP (0x1800) and GATT (0x1801) are mandatory for every LE server and are
 * registered by floss_le_init(), so they always own handles 1..9. Their
 * values are served by small read callbacks straight from the LE module's
 * state (name, appearance, preferred connection parameters), so changing
 * the name after init needs no database update.
 *
 * Service Changed is present (with its CCCD) so clients know to watch it,
 * but the database is fixed after start-up, so it is never indicated.
 * NOTE for developers iterating on a table: hosts that bonded with an older
 * firmware cache handles; remove the bond on the host after changing it.
 *
 * Device Information (0x180A) and Battery (0x180F) are what HID-over-GATT
 * hosts expect next to the HID service; floss_hogp_init() registers them,
 * other applications can do so directly.
 */
#include "floss_internal.h"
#include "floss/floss_le.h"
#include "floss/floss_gatt.h"

#if FLOSS_ENABLE_LE

/** Serve `len` bytes of src from `offset` (the common read-callback body). */
static int copy_out(const uint8_t *src, uint16_t len, uint16_t offset, uint8_t *out, uint16_t cap) {
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
/* GAP + GATT                                                               */
/* ------------------------------------------------------------------------ */

static int read_device_name(uint16_t conn, uint16_t h, const floss_gatt_attr_t *a, uint16_t offset, uint8_t *out,
                            uint16_t cap) {
    const char *name = floss_le_name();
    return copy_out((const uint8_t *)name, (uint16_t)strlen(name), offset, out, cap);
}

static int read_appearance(uint16_t conn, uint16_t h, const floss_gatt_attr_t *a, uint16_t offset, uint8_t *out,
                           uint16_t cap) {
    uint8_t v[2];
    floss_put_le16(v, floss_le_appearance());
    return copy_out(v, 2, offset, out, cap);
}

/* Peripheral Preferred Connection Parameters: some centrals (notably
 * Apple's) read this and connect with it next time, skipping an update. */
static int read_ppcp(uint16_t conn, uint16_t h, const floss_gatt_attr_t *a, uint16_t offset, uint8_t *out,
                     uint16_t cap) {
    const floss_le_conn_params_t *p = &floss_le_link_policy()->preferred;
    uint8_t v[8];
    floss_put_le16(&v[0], p->interval_min);
    floss_put_le16(&v[2], p->interval_max);
    floss_put_le16(&v[4], p->latency);
    floss_put_le16(&v[6], p->timeout);
    return copy_out(v, 8, offset, out, cap);
}

/* Affected range of a Service Changed indication: everything. */
static const uint8_t k_service_changed[4] = {0x01, 0x00, 0xFF, 0xFF};
static floss_gatt_cccd_t s_service_changed_cccd;

static const floss_gatt_attr_t k_core_services[] = {
    FLOSS_GATT_PRIMARY_SERVICE16(FLOSS_GATT_SVC_GAP),
    FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_READ),
    FLOSS_GATT_VALUE16_CB(FLOSS_GATT_CHR_DEVICE_NAME, FLOSS_GATT_P_READ, read_device_name, NULL,
                          FLOSS_LE_NAME_MAX, NULL),
    FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_READ),
    FLOSS_GATT_VALUE16_CB(FLOSS_GATT_CHR_APPEARANCE, FLOSS_GATT_P_READ, read_appearance, NULL, 2, NULL),
    FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_READ),
    FLOSS_GATT_VALUE16_CB(FLOSS_GATT_CHR_PPCP, FLOSS_GATT_P_READ, read_ppcp, NULL, 8, NULL),

    FLOSS_GATT_PRIMARY_SERVICE16(FLOSS_GATT_SVC_GATT),
    FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_INDICATE),
    FLOSS_GATT_VALUE16(FLOSS_GATT_CHR_SERVICE_CHANGED, 0 /* not readable */, k_service_changed, 4, NULL),
    FLOSS_GATT_CCCD(s_service_changed_cccd),
};

/* ------------------------------------------------------------------------ */
/* Device Information                                                       */
/* ------------------------------------------------------------------------ */

static floss_gatt_attr_t s_dis[1 + 2 * 4]; /* service + up to 4 characteristics */
static uint8_t s_pnp_id[7];
static bool s_dis_registered;

static void dis_add_string(uint16_t *n, uint16_t uuid, const char *s) {
    if (s == NULL) {
        return;
    }
    s_dis[(*n)++] = (floss_gatt_attr_t)FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_READ);
    s_dis[(*n)++] = (floss_gatt_attr_t)FLOSS_GATT_VALUE16(uuid, FLOSS_GATT_P_READ, s,
                                                          (uint16_t)FLOSS_MIN(strlen(s), FLOSS_ATT_MAX_VALUE), NULL);
}

floss_err_t floss_gatt_dis_register(const floss_gatt_dis_t *cfg) {
    if (cfg == NULL) {
        return FLOSS_ERR_INVALID;
    }
    if (s_dis_registered) {
        return FLOSS_ERR_STATE;
    }
    uint16_t n = 0;
    s_dis[n++] = (floss_gatt_attr_t)FLOSS_GATT_PRIMARY_SERVICE16(FLOSS_GATT_SVC_DEVICE_INFO);
    dis_add_string(&n, FLOSS_GATT_CHR_MANUFACTURER_NAME, cfg->manufacturer);
    dis_add_string(&n, FLOSS_GATT_CHR_MODEL_NUMBER, cfg->model);
    dis_add_string(&n, FLOSS_GATT_CHR_FIRMWARE_REV, cfg->firmware);
    if (cfg->vendor_id_source) {
        /* PnP ID: lets hosts load vendor-specific drivers/mappings. */
        s_pnp_id[0] = cfg->vendor_id_source;
        floss_put_le16(&s_pnp_id[1], cfg->vendor_id);
        floss_put_le16(&s_pnp_id[3], cfg->product_id);
        floss_put_le16(&s_pnp_id[5], cfg->version);
        s_dis[n++] = (floss_gatt_attr_t)FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_READ);
        s_dis[n++] = (floss_gatt_attr_t)FLOSS_GATT_VALUE16(FLOSS_GATT_CHR_PNP_ID, FLOSS_GATT_P_READ, s_pnp_id,
                                                           sizeof(s_pnp_id), NULL);
    }
    if (floss_gatt_register(s_dis, n) == 0) {
        return FLOSS_ERR_NOMEM;
    }
    s_dis_registered = true;
    return FLOSS_OK;
}

/* ------------------------------------------------------------------------ */
/* Battery                                                                  */
/* ------------------------------------------------------------------------ */

static uint8_t s_battery_level;
static floss_gatt_cccd_t s_battery_cccd;
static uint16_t s_battery_handle;

static const floss_gatt_attr_t k_battery[] = {
    FLOSS_GATT_PRIMARY_SERVICE16(FLOSS_GATT_SVC_BATTERY),
    FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_READ | FLOSS_GATT_PROP_NOTIFY),
    FLOSS_GATT_VALUE16(FLOSS_GATT_CHR_BATTERY_LEVEL, FLOSS_GATT_P_READ, &s_battery_level, 1, NULL),
    FLOSS_GATT_CCCD(s_battery_cccd),
};

floss_err_t floss_gatt_battery_register(uint8_t level) {
    if (s_battery_handle) {
        return FLOSS_ERR_STATE;
    }
    s_battery_level = FLOSS_MIN(level, (uint8_t)100);
    uint16_t base = floss_gatt_register(k_battery, FLOSS_ARRAY_SIZE(k_battery));
    if (base == 0) {
        return FLOSS_ERR_NOMEM;
    }
    s_battery_handle = (uint16_t)(base + 2);
    return FLOSS_OK;
}

void floss_gatt_battery_set(uint8_t level) {
    level = FLOSS_MIN(level, (uint8_t)100);
    if (level == s_battery_level) {
        return;
    }
    s_battery_level = level;
    if (s_battery_handle) {
        floss_gatt_notify_all(s_battery_handle, &s_battery_level, 1);
    }
}

uint16_t floss_gatt_battery_handle(void) {
    return s_battery_handle;
}

/* ------------------------------------------------------------------------ */
/* Init                                                                     */
/* ------------------------------------------------------------------------ */

void floss_gatt_services_register_core(void) {
    memset(&s_service_changed_cccd, 0, sizeof(s_service_changed_cccd));
    memset(&s_battery_cccd, 0, sizeof(s_battery_cccd));
    s_dis_registered = false;
    s_battery_handle = 0;
    floss_gatt_register(k_core_services, FLOSS_ARRAY_SIZE(k_core_services));
}

#endif /* FLOSS_ENABLE_LE */
