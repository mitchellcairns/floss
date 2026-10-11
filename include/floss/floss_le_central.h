/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_le_central.h - LE central role: scanning, connecting, and a minimal
 * GATT client (discover a characteristic, subscribe, write, read, receive
 * notifications). Built for receivers/dongles talking to FLOSS pads, but
 * generic enough for any peripheral.
 *
 * Connection results reuse the LE listener events in floss_le.h
 * (FLOSS_LE_EVT_CONNECTED with role 0, FLOSS_LE_EVT_CONNECT_FAILED, ...).
 * Pairing / encrypting as central: floss_smp_request_security(handle) in
 * floss_smp.h (encrypts with a stored bond, else pairs).
 *
 * All callbacks run in the stack context. At most one GATT client procedure
 * per link is in flight (ATT is strictly request/response); starting a
 * second one returns FLOSS_ERR_BUSY.
 */
#ifndef FLOSS_LE_CENTRAL_H
#define FLOSS_LE_CENTRAL_H

#include "floss_types.h"
#include "floss_le.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Scanning                                                                 */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint16_t interval;       /**< 0.625 ms units (default 0x0030 = 30 ms) */
    uint16_t window;         /**< 0.625 ms units (default 0x0030: continuous) */
    bool active;             /**< send scan requests to get scan response data (default true) */
    bool filter_duplicates;  /**< controller drops repeated reports (default false) */
} floss_le_scan_params_t;

void floss_le_scan_params_defaults(floss_le_scan_params_t *p);

typedef struct {
    uint8_t evt_type;        /**< 0 ADV_IND, 1 ADV_DIRECT_IND, 2 ADV_SCAN_IND, 3 ADV_NONCONN_IND, 4 SCAN_RSP */
    uint8_t addr_type;       /**< 0 public, 1 random */
    floss_addr_t addr;
    int8_t rssi;
    const uint8_t *data;     /**< AD structures, valid during the callback */
    uint8_t len;
} floss_le_adv_report_t;

typedef void (*floss_le_scan_fn)(const floss_le_adv_report_t *report, void *user);

floss_err_t floss_le_scan_start(const floss_le_scan_params_t *p, floss_le_scan_fn fn, void *user);
floss_err_t floss_le_scan_stop(void);
bool floss_le_scanning(void);

/** Find AD structure `ad_type` in advertising data; returns its payload (and length) or NULL. */
const uint8_t *floss_le_ad_find(const uint8_t *data, uint8_t len, uint8_t ad_type, uint8_t *out_len);
/** True if the AD data lists this 128-bit service UUID (big-endian bytes as written). */
bool floss_le_ad_has_uuid128(const uint8_t *data, uint8_t len, const uint8_t uuid_be[16]);

/* ------------------------------------------------------------------------ */
/* Connecting                                                               */
/* ------------------------------------------------------------------------ */

/** Parameters for initiating a connection as central. */
typedef struct {
    uint16_t scan_interval;        /**< 0.625 ms units, while initiating (default 0x0030) */
    uint16_t scan_window;          /**< (default 0x0030) */
    floss_le_conn_params_t conn;   /**< requested link parameters (default 6..6, latency 0, timeout 200) */
} floss_le_connect_params_t;

void floss_le_connect_params_defaults(floss_le_connect_params_t *p);

/** Initiate a connection (stops scanning). Result: FLOSS_LE_EVT_CONNECTED / CONNECT_FAILED. */
floss_err_t floss_le_connect(uint8_t addr_type, const floss_addr_t *addr, const floss_le_connect_params_t *p);
floss_err_t floss_le_connect_cancel(void);

/* Changing parameters later: floss_le_request_conn_params() in floss_le.h (works as central). */

/** Request PHYs (bit 0 1M, bit 1 2M, bit 2 Coded). Result: FLOSS_LE_EVT_PHY. */
floss_err_t floss_le_set_phy(uint16_t handle, uint8_t tx_phys, uint8_t rx_phys);

/* ------------------------------------------------------------------------ */
/* GATT client                                                              */
/* ------------------------------------------------------------------------ */

/** Generic completion: att_err 0 = success, else the ATT error code (0xFF = timeout / link lost). */
typedef void (*floss_gattc_done_fn)(uint16_t conn, uint8_t att_err, void *user);

/** Exchange MTU (client rx MTU = FLOSS_ATT_MTU). Result also emitted as FLOSS_LE_EVT_MTU. */
floss_err_t floss_gattc_exchange_mtu(uint16_t conn, floss_gattc_done_fn fn, void *user);

/** Characteristic found (or not: att_err != 0, typically 0x0A attribute not found). */
typedef void (*floss_gattc_char_fn)(uint16_t conn, uint8_t att_err, uint8_t properties, uint16_t value_handle,
                                    uint16_t cccd_handle, void *user);

/**
 * Find the first characteristic with this UUID anywhere in the server
 * (Read By Type over 0x0001-0xFFFF for the declaration) and, if it can
 * notify/indicate, its CCCD (Find Information after the value handle).
 * uuid_be: 128-bit UUID in big-endian byte order as written.
 */
floss_err_t floss_gattc_find_char128(uint16_t conn, const uint8_t uuid_be[16], floss_gattc_char_fn fn, void *user);
floss_err_t floss_gattc_find_char16(uint16_t conn, uint16_t uuid16, floss_gattc_char_fn fn, void *user);

/** Write a value: with_response = Write Request (fn called), else Write Command (fn may be NULL). */
floss_err_t floss_gattc_write(uint16_t conn, uint16_t handle, const uint8_t *data, uint16_t len, bool with_response,
                              floss_gattc_done_fn fn, void *user);

typedef void (*floss_gattc_read_fn)(uint16_t conn, uint8_t att_err, const uint8_t *data, uint16_t len, void *user);
/** Read a value (single Read Request; up to MTU-1 bytes). */
floss_err_t floss_gattc_read(uint16_t conn, uint16_t handle, floss_gattc_read_fn fn, void *user);

/** Write a CCCD: 0x0001 notify, 0x0002 indicate, 0 off. */
floss_err_t floss_gattc_subscribe(uint16_t conn, uint16_t cccd_handle, uint16_t value, floss_gattc_done_fn fn,
                                  void *user);

/** Notifications / indications from servers (indications are confirmed automatically). */
typedef void (*floss_gattc_notify_fn)(uint16_t conn, uint16_t handle, const uint8_t *data, uint16_t len,
                                      bool indication, void *user);
/** Install the (single) notification handler. */
void floss_gattc_set_notify_handler(floss_gattc_notify_fn fn, void *user);

/** True if a client procedure is in flight on this link. */
bool floss_gattc_busy(uint16_t conn);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_LE_CENTRAL_H */
