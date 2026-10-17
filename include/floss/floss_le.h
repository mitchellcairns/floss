/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_le.h - Low Energy GAP: advertising, LE links, connection parameters.
 *
 * ===========================================================================
 *  LE LAYER MAP (all of it lives in src/le, compiled when FLOSS_ENABLE_LE)
 *
 *   APP / HOGP (floss_hogp.h)        reports over GATT notifications
 *    |   floss_gatt_*                 attribute database, notify/indicate
 *   GATT DB (floss_gatt.c) + ATT (floss_att.c)    CID 0x0004
 *    |                    SMP (floss_smp*.c)      CID 0x0006
 *   LE GAP (floss_le.c): advertising, links, conn params, LE signaling (CID 5),
 *    |                   fixed channel routing
 *   HCI  (LE meta events, LE ACL with LE credits)
 *
 *  LE has no dynamic L2CAP channels here: every PDU arrives on a fixed CID
 *  and floss_le.c routes it by CID. There is no connection setup beyond the
 *  link itself, which is why an LE peripheral can deliver its first input
 *  report a few milliseconds after the central connects.
 * ===========================================================================
 *
 * Typical peripheral bring-up (before floss_start()):
 *
 *     floss_le_set_name("FLOSS LE Gamepad");
 *     floss_le_set_appearance(FLOSS_LE_APPEARANCE_GAMEPAD);
 *     static const uint16_t uuids[] = { 0x1812 };
 *     floss_le_adv_set_standard(uuids, 1);  // flags + appearance + UUIDs + name
 *     floss_le_adv_start();                 // applied once the controller is up
 *
 * Latency policy: once connected as a peripheral, after ~1 s (so the central
 * finishes service discovery at its own pace) FLOSS asks for the preferred
 * connection interval (default 7.5-15 ms, latency 0), requests the 2M PHY and
 * 251-byte data length when both controllers can do it.
 */
#ifndef FLOSS_LE_H
#define FLOSS_LE_H

#include "floss_types.h"
#include "floss_hci.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* LE controller opcodes (OGF 0x08) not already in floss_hci.h              */
/* ------------------------------------------------------------------------ */

#define FLOSS_HCI_LE_SET_RANDOM_ADDRESS        FLOSS_OPCODE(0x08, 0x0005)
#define FLOSS_HCI_LE_SET_ADV_PARAMS            FLOSS_OPCODE(0x08, 0x0006)
#define FLOSS_HCI_LE_READ_ADV_TX_POWER         FLOSS_OPCODE(0x08, 0x0007)
#define FLOSS_HCI_LE_SET_ADV_DATA              FLOSS_OPCODE(0x08, 0x0008)
#define FLOSS_HCI_LE_SET_SCAN_RSP_DATA         FLOSS_OPCODE(0x08, 0x0009)
#define FLOSS_HCI_LE_SET_ADV_ENABLE            FLOSS_OPCODE(0x08, 0x000A)
#define FLOSS_HCI_LE_CONN_UPDATE               FLOSS_OPCODE(0x08, 0x0013)
#define FLOSS_HCI_LE_READ_REMOTE_FEATURES      FLOSS_OPCODE(0x08, 0x0016)
#define FLOSS_HCI_LE_ENCRYPT                   FLOSS_OPCODE(0x08, 0x0017)
#define FLOSS_HCI_LE_RAND                      FLOSS_OPCODE(0x08, 0x0018)
#define FLOSS_HCI_LE_START_ENCRYPTION          FLOSS_OPCODE(0x08, 0x0019)
#define FLOSS_HCI_LE_LTK_REQUEST_REPLY         FLOSS_OPCODE(0x08, 0x001A)
#define FLOSS_HCI_LE_LTK_REQUEST_NEG_REPLY     FLOSS_OPCODE(0x08, 0x001B)
#define FLOSS_HCI_LE_REMOTE_CONN_PARAM_REPLY   FLOSS_OPCODE(0x08, 0x0020)
#define FLOSS_HCI_LE_REMOTE_CONN_PARAM_NEG_REPLY FLOSS_OPCODE(0x08, 0x0021)
#define FLOSS_HCI_LE_SET_DATA_LENGTH           FLOSS_OPCODE(0x08, 0x0022)
#define FLOSS_HCI_LE_READ_LOCAL_P256_KEY       FLOSS_OPCODE(0x08, 0x0025)
#define FLOSS_HCI_LE_GENERATE_DHKEY            FLOSS_OPCODE(0x08, 0x0026)
#define FLOSS_HCI_LE_SET_PHY                   FLOSS_OPCODE(0x08, 0x0032)

/* LE meta subevent codes (first byte after the 0x3E event header) */
#define FLOSS_LE_SUBEVT_CONN_COMPLETE          0x01
#define FLOSS_LE_SUBEVT_ADV_REPORT             0x02
#define FLOSS_LE_SUBEVT_CONN_UPDATE_COMPLETE   0x03
#define FLOSS_LE_SUBEVT_REMOTE_FEATURES        0x04
#define FLOSS_LE_SUBEVT_LTK_REQUEST            0x05
#define FLOSS_LE_SUBEVT_REMOTE_CONN_PARAM_REQ  0x06
#define FLOSS_LE_SUBEVT_DATA_LENGTH_CHANGE     0x07
#define FLOSS_LE_SUBEVT_P256_KEY_COMPLETE      0x08
#define FLOSS_LE_SUBEVT_DHKEY_COMPLETE         0x09
#define FLOSS_LE_SUBEVT_ENH_CONN_COMPLETE      0x0A
#define FLOSS_LE_SUBEVT_PHY_UPDATE_COMPLETE    0x0C

/* LE feature bits (floss_controller_info_t.le_features, bit n = byte n/8, bit n%8) */
#define FLOSS_LE_FEAT_ENCRYPTION               0
#define FLOSS_LE_FEAT_CONN_PARAM_REQ           1
#define FLOSS_LE_FEAT_DATA_LENGTH_EXT          5
#define FLOSS_LE_FEAT_PRIVACY                  6
#define FLOSS_LE_FEAT_2M_PHY                   8

/** True if the local controller advertises LE feature bit `bit`. */
bool floss_le_local_feature(uint8_t bit);

/* ------------------------------------------------------------------------ */
/* Fixed channels and LE signaling                                          */
/* ------------------------------------------------------------------------ */

#define FLOSS_CID_ATT            0x0004
#define FLOSS_CID_LE_SIGNALING   0x0005
#define FLOSS_CID_SMP            0x0006

/* LE signaling command codes (CID 0x0005) */
#define FLOSS_LE_SIG_COMMAND_REJECT       0x01
#define FLOSS_LE_SIG_DISCONN_REQ          0x06
#define FLOSS_LE_SIG_CONN_PARAM_REQ       0x12
#define FLOSS_LE_SIG_CONN_PARAM_RSP       0x13
#define FLOSS_LE_SIG_CREDIT_CONN_REQ      0x14
#define FLOSS_LE_SIG_CREDIT_CONN_RSP      0x15

/* ------------------------------------------------------------------------ */
/* Addresses                                                                */
/* ------------------------------------------------------------------------ */

#define FLOSS_LE_ADDR_PUBLIC  0x00
#define FLOSS_LE_ADDR_RANDOM  0x01

/** HAL store key holding our generated random static address (6 bytes). */
#define FLOSS_LE_STORE_KEY_STATIC_ADDR 0x0280

/**
 * Use a specific random static address (two top bits of b[5] must be 1).
 * NULL = generate one with floss_hal_random() on first use and keep it in
 * persistent storage, so bonds survive reboots.
 * Only takes effect with own_addr_type = FLOSS_LE_ADDR_RANDOM.
 */
floss_err_t floss_le_set_static_address(const floss_addr_t *addr);

/** Our current LE identity address (public from the controller, or the static random). */
void floss_le_own_address(floss_addr_t *addr, uint8_t *addr_type);

/* ------------------------------------------------------------------------ */
/* Identity used by advertising and the GAP service                          */
/* ------------------------------------------------------------------------ */

/** Max length of the LE device name we keep (GAP Device Name characteristic). */
#define FLOSS_LE_NAME_MAX 32

/* Appearance values (GAP Appearance characteristic / AD type 0x19) */
#define FLOSS_LE_APPEARANCE_UNKNOWN   0x0000
#define FLOSS_LE_APPEARANCE_HID       0x03C0
#define FLOSS_LE_APPEARANCE_KEYBOARD  0x03C1
#define FLOSS_LE_APPEARANCE_MOUSE     0x03C2
#define FLOSS_LE_APPEARANCE_JOYSTICK  0x03C3
#define FLOSS_LE_APPEARANCE_GAMEPAD   0x03C4

/** Device name (copied, truncated to FLOSS_LE_NAME_MAX bytes). */
floss_err_t floss_le_set_name(const char *name);
const char *floss_le_name(void);
void floss_le_set_appearance(uint16_t appearance);
uint16_t floss_le_appearance(void);

/* ------------------------------------------------------------------------ */
/* Advertising data builder                                                 */
/* ------------------------------------------------------------------------ */

/* AD types */
#define FLOSS_AD_FLAGS              0x01
#define FLOSS_AD_UUID16_SOME        0x02
#define FLOSS_AD_UUID16_ALL         0x03
#define FLOSS_AD_UUID128_ALL        0x07
#define FLOSS_AD_NAME_SHORT         0x08
#define FLOSS_AD_NAME_COMPLETE      0x09
#define FLOSS_AD_TX_POWER           0x0A
#define FLOSS_AD_CONN_INTERVAL_RANGE 0x12
#define FLOSS_AD_APPEARANCE         0x19
#define FLOSS_AD_MANUFACTURER       0xFF

/* Flags AD values */
#define FLOSS_AD_FLAG_LIMITED_DISC  0x01
#define FLOSS_AD_FLAG_GENERAL_DISC  0x02
#define FLOSS_AD_FLAG_NO_BREDR      0x04

/** Legacy advertising / scan response payload is at most 31 bytes. */
#define FLOSS_LE_ADV_DATA_MAX 31

/**
 * A 31-byte AD structure list being built. Every add returns false (and sets
 * `overflow`) instead of writing a partial structure.
 */
typedef struct {
    uint8_t data[FLOSS_LE_ADV_DATA_MAX];
    uint8_t len;
    bool overflow;
} floss_le_ad_t;

void floss_le_ad_init(floss_le_ad_t *ad);
/** Append one AD structure [len][type][data]. */
bool floss_le_ad_add(floss_le_ad_t *ad, uint8_t type, const uint8_t *data, uint8_t len);
bool floss_le_ad_flags(floss_le_ad_t *ad, uint8_t flags);
/** Complete name; if it does not fit, the remaining space is used for a shortened name. */
bool floss_le_ad_name(floss_le_ad_t *ad, const char *name);
/** Complete list of 16-bit service UUIDs. */
bool floss_le_ad_uuid16(floss_le_ad_t *ad, const uint16_t *uuids, uint8_t count);
bool floss_le_ad_appearance(floss_le_ad_t *ad, uint16_t appearance);
bool floss_le_ad_tx_power(floss_le_ad_t *ad, int8_t dbm);

/* ------------------------------------------------------------------------ */
/* Advertising                                                              */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint16_t interval_min;   /**< 0.625 ms units (0x20..0x4000). Default 0x30 (30 ms) */
    uint16_t interval_max;   /**< default 0x60 (60 ms) */
    uint8_t own_addr_type;   /**< FLOSS_LE_ADDR_PUBLIC (default) or FLOSS_LE_ADDR_RANDOM (static) */
    uint8_t channel_map;     /**< bits 0..2 = channels 37,38,39. Default 0x07 */
    uint8_t filter_policy;   /**< 0 = anyone may scan/connect (default) */
    /** Restart advertising automatically when a link drops (default true). */
    bool auto_restart;
    /** Keep advertising while a link is open, for a second central (default false). */
    bool while_connected;
    /**
     * Non-connectable, non-scannable (ADV_NONCONN_IND) instead of ADV_IND: a
     * beacon, e.g. a Classic device announcing itself over LE. Before Bluetooth
     * 5.0 its interval must be at least 100 ms (0xA0). Default false.
     */
    bool non_connectable;
} floss_le_adv_params_t;

void floss_le_adv_params_defaults(floss_le_adv_params_t *p);
/** Undirected advertising (ADV_IND, or ADV_NONCONN_IND if non_connectable) with these parameters. */
floss_err_t floss_le_adv_set_params(const floss_le_adv_params_t *p);
/** Raw advertising data (<= 31 bytes, e.g. floss_le_ad_t.data). */
floss_err_t floss_le_adv_set_data(const uint8_t *data, uint8_t len);
/** Raw scan response data (<= 31 bytes). */
floss_err_t floss_le_adv_set_scan_rsp(const uint8_t *data, uint8_t len);
/**
 * Convenience: advertising data = flags (general discoverable, LE only) +
 * appearance + complete 16-bit UUID list + name (if it fits); scan response =
 * complete name. Uses floss_le_name()/floss_le_appearance().
 */
floss_err_t floss_le_adv_set_standard(const uint16_t *uuids16, uint8_t count);

/** Start advertising (deferred until the controller is ready if called before). */
floss_err_t floss_le_adv_start(void);
/** Stop advertising (and stop auto-restart until the next floss_le_adv_start). */
floss_err_t floss_le_adv_stop(void);
/** True while the controller is advertising. */
bool floss_le_adv_active(void);

/* ------------------------------------------------------------------------ */
/* Connection parameters                                                    */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint16_t interval_min; /**< 1.25 ms units (6 = 7.5 ms) */
    uint16_t interval_max; /**< 1.25 ms units */
    uint16_t latency;      /**< peripheral latency (connection events it may skip) */
    uint16_t timeout;      /**< supervision timeout, 10 ms units */
} floss_le_conn_params_t;

typedef struct {
    /** Preferred parameters (default 6..12, latency 0, timeout 300 = 3 s). */
    floss_le_conn_params_t preferred;
    /** As peripheral, request `preferred` this long after connecting (default 1 s; 0 = never). */
    uint32_t update_delay_us;
    /** Request the 2M PHY when both sides support it (default true). */
    bool prefer_2m_phy;
    /** Request 251-byte LE data length when supported (default true). */
    bool max_data_length;
} floss_le_link_policy_t;

void floss_le_link_policy_defaults(floss_le_link_policy_t *p);
floss_err_t floss_le_set_link_policy(const floss_le_link_policy_t *p);
const floss_le_link_policy_t *floss_le_link_policy(void);

/**
 * Ask for new connection parameters now. As peripheral: LE Connection Update
 * if the controller supports the Connection Parameters Request procedure
 * (falling back to an L2CAP Connection Parameter Update Request), as central:
 * LE Connection Update directly. Result: FLOSS_LE_EVT_CONN_PARAMS.
 */
floss_err_t floss_le_request_conn_params(uint16_t handle, const floss_le_conn_params_t *p);

/* ------------------------------------------------------------------------ */
/* Links                                                                    */
/* ------------------------------------------------------------------------ */

/** Snapshot of one LE link, maintained by the LE module. */
typedef struct {
    uint16_t handle;
    floss_addr_t peer;
    uint8_t peer_addr_type;   /**< as reported by the controller (0 public, 1 random, 2/3 resolved) */
    uint8_t role;             /**< 0 central, 1 peripheral */
    uint16_t interval;        /**< 1.25 ms units */
    uint16_t latency;
    uint16_t timeout;         /**< 10 ms units */
    uint16_t att_mtu;         /**< negotiated ATT MTU (23 until Exchange MTU) */
    uint8_t security_level;   /**< 1 none, 2 encrypted, 3 authenticated, 4 SC authenticated */
    bool encrypted;
    uint16_t tx_octets;       /**< LL max TX payload (27 until Data Length Change) */
    uint8_t tx_phy, rx_phy;   /**< 1 = 1M, 2 = 2M, 3 = Coded */
} floss_le_link_t;

/** Link info, or NULL if `handle` is not an open LE link. */
const floss_le_link_t *floss_le_link(uint16_t handle);
/** Index 0..FLOSS_MAX_CONNECTIONS-1 of the LE link (per-link tables use it), -1 if none. */
int floss_le_link_index(uint16_t handle);
/** Handle of the LE link at index i, 0xFFFF if that slot is free. */
uint16_t floss_le_link_handle_at(uint8_t i);
/** Number of open LE links. */
uint8_t floss_le_link_count(void);
/** Disconnect an LE link (reason default FLOSS_HCI_ERR_REMOTE_USER_TERMINATED). */
floss_err_t floss_le_disconnect(uint16_t handle, uint8_t reason);

/* ------------------------------------------------------------------------ */
/* Events                                                                   */
/* ------------------------------------------------------------------------ */

typedef enum {
    FLOSS_LE_EVT_CONNECTED = 1,   /**< link open: handle, addr, addr_type, role, interval/latency/timeout */
    FLOSS_LE_EVT_CONNECT_FAILED,  /**< status */
    FLOSS_LE_EVT_DISCONNECTED,    /**< handle, addr, status = reason */
    FLOSS_LE_EVT_CONN_PARAMS,     /**< interval/latency/timeout changed (status != 0: update failed) */
    FLOSS_LE_EVT_MTU,             /**< mtu */
    FLOSS_LE_EVT_SECURITY,        /**< level (1..4) */
    FLOSS_LE_EVT_DATA_LENGTH,     /**< tx_octets */
    FLOSS_LE_EVT_PHY,             /**< tx_phy, rx_phy */
    FLOSS_LE_EVT_ADV_STARTED,
    FLOSS_LE_EVT_ADV_STOPPED,     /**< status = reason (0 = asked, or a link was created) */
    FLOSS_LE_EVT_INDICATION_DONE, /**< attr_handle; status 0 = confirmed, else timeout/link loss */
} floss_le_evt_type_t;

typedef struct {
    floss_le_evt_type_t type;
    uint16_t handle;
    floss_addr_t addr;
    uint8_t addr_type;
    uint8_t role;
    uint8_t status;
    uint8_t level;
    uint16_t interval, latency, timeout;
    uint16_t mtu;
    uint16_t tx_octets;
    uint8_t tx_phy, rx_phy;
    uint16_t attr_handle;
} floss_le_evt_t;

typedef void (*floss_le_evt_fn)(const floss_le_evt_t *evt, void *user);

#define FLOSS_LE_MAX_LISTENERS 4

/** Register an LE event listener (up to FLOSS_LE_MAX_LISTENERS). */
floss_err_t floss_le_add_listener(floss_le_evt_fn fn, void *user);

/* ------------------------------------------------------------------------ */
/* Stack internal (called by floss.c and the ATT/GATT/HOGP modules)         */
/* ------------------------------------------------------------------------ */

/**
 * Per-poll work for the LE modules (retry deferred ATT responses, HOGP
 * send-on-change). Called by floss_poll() when FLOSS_ENABLE_LE.
 */
void floss_le_poll(void);

/** ATT reports the negotiated MTU for a link (emits FLOSS_LE_EVT_MTU). */
void floss_le_set_att_mtu(uint16_t handle, uint16_t mtu);

/** Deliver an event to every listener (used by ATT for INDICATION_DONE). */
void floss_le_emit(const floss_le_evt_t *evt);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_LE_H */
