/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_hci.h - Host Controller Interface layer.
 *
 * Responsibilities:
 *  - Controller bring-up (reset, read capabilities, vendor quirks, event mask).
 *  - Command queue with flow control (Num_HCI_Command_Packets) and timeouts.
 *  - ACL TX with controller buffer credits (Number Of Completed Packets),
 *    per-link queues, priority insertion, fragmentation to the controller's
 *    ACL buffer size.
 *  - ACL RX reassembly of L2CAP PDUs.
 *  - The connection table (one floss_conn_t per baseband link).
 *  - Dispatch: events -> GAP, ACL -> L2CAP.
 *
 * Applications rarely call this layer directly; use GAP/HID. It is public so
 * advanced users can send arbitrary (vendor) commands.
 */
#ifndef FLOSS_HCI_H
#define FLOSS_HCI_H

#include "floss_types.h"
#include "floss_buf.h"
#include "floss_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Opcodes (OGF << 10 | OCF)                                                */
/* ------------------------------------------------------------------------ */

#define FLOSS_OPCODE(ogf, ocf) ((uint16_t)(((ogf) << 10) | (ocf)))

/* Link control (OGF 0x01) */
#define FLOSS_HCI_INQUIRY                      FLOSS_OPCODE(0x01, 0x0001)
#define FLOSS_HCI_INQUIRY_CANCEL               FLOSS_OPCODE(0x01, 0x0002)
#define FLOSS_HCI_CREATE_CONNECTION            FLOSS_OPCODE(0x01, 0x0005)
#define FLOSS_HCI_DISCONNECT                   FLOSS_OPCODE(0x01, 0x0006)
#define FLOSS_HCI_CREATE_CONNECTION_CANCEL     FLOSS_OPCODE(0x01, 0x0008)
#define FLOSS_HCI_ACCEPT_CONNECTION_REQUEST    FLOSS_OPCODE(0x01, 0x0009)
#define FLOSS_HCI_REJECT_CONNECTION_REQUEST    FLOSS_OPCODE(0x01, 0x000A)
#define FLOSS_HCI_LINK_KEY_REQUEST_REPLY       FLOSS_OPCODE(0x01, 0x000B)
#define FLOSS_HCI_LINK_KEY_REQUEST_NEG_REPLY   FLOSS_OPCODE(0x01, 0x000C)
#define FLOSS_HCI_PIN_CODE_REQUEST_REPLY       FLOSS_OPCODE(0x01, 0x000D)
#define FLOSS_HCI_PIN_CODE_REQUEST_NEG_REPLY   FLOSS_OPCODE(0x01, 0x000E)
#define FLOSS_HCI_AUTHENTICATION_REQUESTED     FLOSS_OPCODE(0x01, 0x0011)
#define FLOSS_HCI_SET_CONNECTION_ENCRYPTION    FLOSS_OPCODE(0x01, 0x0013)
#define FLOSS_HCI_REMOTE_NAME_REQUEST          FLOSS_OPCODE(0x01, 0x0019)
#define FLOSS_HCI_READ_REMOTE_FEATURES         FLOSS_OPCODE(0x01, 0x001B)
#define FLOSS_HCI_READ_REMOTE_VERSION          FLOSS_OPCODE(0x01, 0x001D)
#define FLOSS_HCI_IO_CAPABILITY_REQUEST_REPLY  FLOSS_OPCODE(0x01, 0x002B)
#define FLOSS_HCI_USER_CONFIRM_REQUEST_REPLY   FLOSS_OPCODE(0x01, 0x002C)
#define FLOSS_HCI_USER_CONFIRM_REQUEST_NEG_REPLY FLOSS_OPCODE(0x01, 0x002D)
#define FLOSS_HCI_USER_PASSKEY_REQUEST_REPLY   FLOSS_OPCODE(0x01, 0x002E)
#define FLOSS_HCI_USER_PASSKEY_REQUEST_NEG_REPLY FLOSS_OPCODE(0x01, 0x002F)
#define FLOSS_HCI_IO_CAPABILITY_REQUEST_NEG_REPLY FLOSS_OPCODE(0x01, 0x0034)

/* Link policy (OGF 0x02) */
#define FLOSS_HCI_SNIFF_MODE                   FLOSS_OPCODE(0x02, 0x0003)
#define FLOSS_HCI_EXIT_SNIFF_MODE              FLOSS_OPCODE(0x02, 0x0004)
#define FLOSS_HCI_QOS_SETUP                    FLOSS_OPCODE(0x02, 0x0007)
#define FLOSS_HCI_ROLE_DISCOVERY               FLOSS_OPCODE(0x02, 0x0009)
#define FLOSS_HCI_SWITCH_ROLE                  FLOSS_OPCODE(0x02, 0x000B)
#define FLOSS_HCI_WRITE_LINK_POLICY_SETTINGS   FLOSS_OPCODE(0x02, 0x000D)
#define FLOSS_HCI_WRITE_DEFAULT_LINK_POLICY    FLOSS_OPCODE(0x02, 0x000F)
#define FLOSS_HCI_FLOW_SPECIFICATION           FLOSS_OPCODE(0x02, 0x0010)
#define FLOSS_HCI_SNIFF_SUBRATING              FLOSS_OPCODE(0x02, 0x0011)

/* Controller & baseband (OGF 0x03) */
#define FLOSS_HCI_SET_EVENT_MASK               FLOSS_OPCODE(0x03, 0x0001)
#define FLOSS_HCI_RESET                        FLOSS_OPCODE(0x03, 0x0003)
#define FLOSS_HCI_SET_EVENT_FILTER             FLOSS_OPCODE(0x03, 0x0005)
#define FLOSS_HCI_DELETE_STORED_LINK_KEY       FLOSS_OPCODE(0x03, 0x0012)
#define FLOSS_HCI_WRITE_LOCAL_NAME             FLOSS_OPCODE(0x03, 0x0013)
#define FLOSS_HCI_READ_LOCAL_NAME              FLOSS_OPCODE(0x03, 0x0014)
#define FLOSS_HCI_WRITE_CONNECTION_ACCEPT_TIMEOUT FLOSS_OPCODE(0x03, 0x0016)
#define FLOSS_HCI_WRITE_PAGE_TIMEOUT           FLOSS_OPCODE(0x03, 0x0018)
#define FLOSS_HCI_WRITE_SCAN_ENABLE            FLOSS_OPCODE(0x03, 0x001A)
#define FLOSS_HCI_WRITE_PAGE_SCAN_ACTIVITY     FLOSS_OPCODE(0x03, 0x001C)
#define FLOSS_HCI_WRITE_INQUIRY_SCAN_ACTIVITY  FLOSS_OPCODE(0x03, 0x001E)
#define FLOSS_HCI_WRITE_AUTHENTICATION_ENABLE  FLOSS_OPCODE(0x03, 0x0020)
#define FLOSS_HCI_WRITE_CLASS_OF_DEVICE        FLOSS_OPCODE(0x03, 0x0024)
#define FLOSS_HCI_WRITE_AUTOMATIC_FLUSH_TIMEOUT FLOSS_OPCODE(0x03, 0x0028)
#define FLOSS_HCI_WRITE_LINK_SUPERVISION_TIMEOUT FLOSS_OPCODE(0x03, 0x0037)
#define FLOSS_HCI_WRITE_CURRENT_IAC_LAP        FLOSS_OPCODE(0x03, 0x003A)
#define FLOSS_HCI_WRITE_INQUIRY_SCAN_TYPE      FLOSS_OPCODE(0x03, 0x0043)
#define FLOSS_HCI_WRITE_INQUIRY_MODE           FLOSS_OPCODE(0x03, 0x0045)
#define FLOSS_HCI_WRITE_PAGE_SCAN_TYPE         FLOSS_OPCODE(0x03, 0x0047)
#define FLOSS_HCI_WRITE_EXTENDED_INQUIRY_RESPONSE FLOSS_OPCODE(0x03, 0x0052)
#define FLOSS_HCI_SET_AFH_HOST_CHANNEL_CLASSIFICATION FLOSS_OPCODE(0x03, 0x003F)
#define FLOSS_HCI_READ_AFH_CHANNEL_ASSESSMENT_MODE  FLOSS_OPCODE(0x03, 0x0048)
#define FLOSS_HCI_WRITE_AFH_CHANNEL_ASSESSMENT_MODE FLOSS_OPCODE(0x03, 0x0049)
#define FLOSS_HCI_WRITE_SIMPLE_PAIRING_MODE    FLOSS_OPCODE(0x03, 0x0056)
#define FLOSS_HCI_SET_EVENT_MASK_PAGE_2        FLOSS_OPCODE(0x03, 0x0063)
#define FLOSS_HCI_WRITE_LE_HOST_SUPPORT        FLOSS_OPCODE(0x03, 0x006D)
#define FLOSS_HCI_WRITE_SECURE_CONNECTIONS_HOST_SUPPORT FLOSS_OPCODE(0x03, 0x007A)

/* Informational (OGF 0x04) */
#define FLOSS_HCI_READ_LOCAL_VERSION           FLOSS_OPCODE(0x04, 0x0001)
#define FLOSS_HCI_READ_LOCAL_COMMANDS          FLOSS_OPCODE(0x04, 0x0002)
#define FLOSS_HCI_READ_LOCAL_FEATURES          FLOSS_OPCODE(0x04, 0x0003)
#define FLOSS_HCI_READ_BUFFER_SIZE             FLOSS_OPCODE(0x04, 0x0005)
#define FLOSS_HCI_READ_BD_ADDR                 FLOSS_OPCODE(0x04, 0x0009)

/* LE controller (OGF 0x08) - only the ones HCI bring-up uses; the LE module defines the rest */
#define FLOSS_HCI_LE_SET_EVENT_MASK            FLOSS_OPCODE(0x08, 0x0001)
#define FLOSS_HCI_LE_READ_BUFFER_SIZE          FLOSS_OPCODE(0x08, 0x0002)
#define FLOSS_HCI_LE_READ_LOCAL_FEATURES       FLOSS_OPCODE(0x08, 0x0003)

/* Status (OGF 0x05) */
#define FLOSS_HCI_READ_AFH_CHANNEL_MAP         FLOSS_OPCODE(0x05, 0x0006)
#define FLOSS_HCI_READ_CLOCK                   FLOSS_OPCODE(0x05, 0x0007)
#define FLOSS_HCI_READ_RSSI                    FLOSS_OPCODE(0x05, 0x0005)

/* ------------------------------------------------------------------------ */
/* Event codes                                                              */
/* ------------------------------------------------------------------------ */

#define FLOSS_EVT_INQUIRY_COMPLETE             0x01
#define FLOSS_EVT_INQUIRY_RESULT               0x02
#define FLOSS_EVT_CONNECTION_COMPLETE          0x03
#define FLOSS_EVT_CONNECTION_REQUEST           0x04
#define FLOSS_EVT_DISCONNECTION_COMPLETE       0x05
#define FLOSS_EVT_AUTHENTICATION_COMPLETE      0x06
#define FLOSS_EVT_REMOTE_NAME_REQUEST_COMPLETE 0x07
#define FLOSS_EVT_ENCRYPTION_CHANGE            0x08
#define FLOSS_EVT_READ_REMOTE_FEATURES_COMPLETE 0x0B
#define FLOSS_EVT_READ_REMOTE_VERSION_COMPLETE 0x0C
#define FLOSS_EVT_QOS_SETUP_COMPLETE           0x0D
#define FLOSS_EVT_COMMAND_COMPLETE             0x0E
#define FLOSS_EVT_COMMAND_STATUS               0x0F
#define FLOSS_EVT_HARDWARE_ERROR               0x10
#define FLOSS_EVT_ROLE_CHANGE                  0x12
#define FLOSS_EVT_NUMBER_OF_COMPLETED_PACKETS  0x13
#define FLOSS_EVT_MODE_CHANGE                  0x14
#define FLOSS_EVT_PIN_CODE_REQUEST             0x16
#define FLOSS_EVT_LINK_KEY_REQUEST             0x17
#define FLOSS_EVT_LINK_KEY_NOTIFICATION        0x18
#define FLOSS_EVT_MAX_SLOTS_CHANGE             0x1B
#define FLOSS_EVT_FLUSH_OCCURRED               0x11
#define FLOSS_EVT_DATA_BUFFER_OVERFLOW         0x1A
#define FLOSS_EVT_PAGE_SCAN_REPETITION_MODE_CHANGE 0x20
#define FLOSS_EVT_FLOW_SPECIFICATION_COMPLETE  0x21
#define FLOSS_EVT_INQUIRY_RESULT_WITH_RSSI     0x22
#define FLOSS_EVT_READ_REMOTE_EXT_FEATURES_COMPLETE 0x23
#define FLOSS_EVT_SNIFF_SUBRATING              0x2E
#define FLOSS_EVT_EXTENDED_INQUIRY_RESULT      0x2F
#define FLOSS_EVT_ENCRYPTION_KEY_REFRESH_COMPLETE 0x30
#define FLOSS_EVT_IO_CAPABILITY_REQUEST        0x31
#define FLOSS_EVT_IO_CAPABILITY_RESPONSE       0x32
#define FLOSS_EVT_USER_CONFIRMATION_REQUEST    0x33
#define FLOSS_EVT_USER_PASSKEY_REQUEST         0x34
#define FLOSS_EVT_SIMPLE_PAIRING_COMPLETE      0x36
#define FLOSS_EVT_LINK_SUPERVISION_TIMEOUT_CHANGED 0x38
#define FLOSS_EVT_USER_PASSKEY_NOTIFICATION    0x3B
#define FLOSS_EVT_LE_META                      0x3E
#define FLOSS_EVT_VENDOR                       0xFF

/* Common HCI status / reason codes */
#define FLOSS_HCI_SUCCESS                      0x00
#define FLOSS_HCI_ERR_UNKNOWN_CONN_ID          0x02
#define FLOSS_HCI_ERR_PAGE_TIMEOUT             0x04
#define FLOSS_HCI_ERR_AUTH_FAILURE             0x05
#define FLOSS_HCI_ERR_PIN_OR_KEY_MISSING       0x06
#define FLOSS_HCI_ERR_CONN_TIMEOUT             0x08
#define FLOSS_HCI_ERR_CONN_ACCEPT_TIMEOUT      0x10
#define FLOSS_HCI_ERR_REJECTED_LIMITED_RESOURCES 0x0D
#define FLOSS_HCI_ERR_REJECTED_SECURITY        0x0E
#define FLOSS_HCI_ERR_REMOTE_USER_TERMINATED   0x13
#define FLOSS_HCI_ERR_REMOTE_POWER_OFF         0x15
#define FLOSS_HCI_ERR_LOCAL_HOST_TERMINATED    0x16
#define FLOSS_HCI_ERR_PAIRING_NOT_ALLOWED      0x18
#define FLOSS_HCI_ERR_UNSPECIFIED              0x1F

/** Name of an HCI status code for logs ("Page Timeout", ...). */
const char *floss_hci_status_str(uint8_t status);

/* ------------------------------------------------------------------------ */
/* Controller information (valid after the stack reports READY)             */
/* ------------------------------------------------------------------------ */

typedef struct {
    floss_addr_t addr;         /**< our public device address */
    uint8_t hci_version;
    uint16_t hci_revision;
    uint8_t lmp_version;
    uint16_t manufacturer;
    uint16_t lmp_subversion;
    uint16_t acl_mtu;          /**< max ACL payload per packet the controller accepts */
    uint16_t acl_total;        /**< controller ACL buffers (credits) */
    uint16_t le_acl_mtu;       /**< LE ACL payload size (0 = LE shares the BR/EDR buffers) */
    uint16_t le_acl_total;     /**< LE ACL buffers (0 = shared) */
    uint8_t le_features[8];    /**< LE features (if LE supported) */
    uint8_t features[8];       /**< LMP features page 0 */
    uint8_t commands[64];      /**< supported commands bitmap */
} floss_controller_info_t;

const floss_controller_info_t *floss_hci_controller(void);

/** Test a bit of the supported commands bitmap (octet, bit as in the spec table). */
static inline bool floss_hci_cmd_supported(uint8_t octet, uint8_t bit) {
    return (floss_hci_controller()->commands[octet] >> bit) & 1u;
}

/* ------------------------------------------------------------------------ */
/* Connections                                                              */
/* ------------------------------------------------------------------------ */

typedef enum {
    FLOSS_CONN_FREE = 0,
    FLOSS_CONN_OUTGOING,   /**< Create_Connection issued */
    FLOSS_CONN_INCOMING,   /**< Connection_Request accepted, waiting for complete */
    FLOSS_CONN_OPEN,
    FLOSS_CONN_CLOSING,
} floss_conn_state_t;

#define FLOSS_LINK_MODE_ACTIVE 0
#define FLOSS_LINK_MODE_HOLD   1
#define FLOSS_LINK_MODE_SNIFF  2

/** One baseband link. Owned by HCI, read by GAP/L2CAP. */
typedef struct {
    floss_conn_state_t state;
    uint16_t handle;
    floss_addr_t addr;
    bool le;                 /**< Low Energy link (ACL PDUs go to floss_le_on_pdu) */
    uint8_t addr_type;       /**< LE: peer address type (0 public, 1 random) */
    uint8_t role;            /**< 0 = central (we are master), 1 = peripheral */
    uint8_t mode;            /**< FLOSS_LINK_MODE_* */
    uint16_t sniff_interval; /**< slots, when in sniff */
    bool authenticated;
    bool encrypted;
    bool initiator;          /**< we created the link */
    uint32_t pending_since_us; /**< OUTGOING/INCOMING: when the slot was created (guard timer) */

    /* ACL TX */
    floss_bufq_t txq;        /**< waiting for controller credits */
    uint16_t inflight;       /**< packets handed to controller, not yet completed */
    uint32_t inflight_t[FLOSS_ACL_INFLIGHT_MAX]; /**< send timestamps (FIFO) */
    uint8_t inflight_head;

    /* ACL RX reassembly */
    floss_buf_t *rx_buf;     /**< partial L2CAP PDU, NULL if none */
    uint16_t rx_expected;    /**< total PDU length incl. L2CAP header */
} floss_conn_t;

floss_conn_t *floss_hci_conn_by_handle(uint16_t handle);
floss_conn_t *floss_hci_conn_by_addr(const floss_addr_t *addr);
/** Iterate: returns connection at index i (0..FLOSS_MAX_CONNECTIONS-1), may be FREE. */
floss_conn_t *floss_hci_conn_at(uint8_t i);

/* ------------------------------------------------------------------------ */
/* Commands                                                                 */
/* ------------------------------------------------------------------------ */

/**
 * Completion callback.
 *  - Command Complete: status = first return byte, ret/ret_len = the rest.
 *  - Command Status:   status = status, ret = NULL, ret_len = 0.
 *  - Timeout:          status = 0xFF, ret = NULL.
 */
typedef void (*floss_hci_cmd_cb)(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len);

/**
 * Queue an HCI command. Params are copied. Sent as soon as the controller has
 * a command credit. cb may be NULL.
 * @return FLOSS_OK, FLOSS_ERR_NOMEM (queue/pool full), FLOSS_ERR_INVALID.
 */
floss_err_t floss_hci_cmd(uint16_t opcode, const void *params, uint8_t len, floss_hci_cmd_cb cb, void *ctx);

/* ------------------------------------------------------------------------ */
/* ACL data                                                                 */
/* ------------------------------------------------------------------------ */

/**
 * Send one L2CAP PDU (buf->data starts at the L2CAP basic header) on the link
 * buf->handle. HCI prepends the ACL header, fragments if larger than the
 * controller's ACL MTU, and queues if no credits. Takes ownership of buf.
 * Honours FLOSS_BUF_F_FLUSHABLE and FLOSS_BUF_F_PRIORITY.
 */
floss_err_t floss_hci_acl_send(floss_buf_t *buf);

/** True if a packet for this link would go to the controller immediately. */
bool floss_hci_acl_can_send_now(uint16_t handle);

/**
 * Newest-only send gating: true if a packet for this link would go to the
 * controller now without queueing behind earlier ones, AND fewer than
 * max_inflight x floss_hci_completion_batch() of the link's packets are
 * still in the controller. A packet waiting in the controller can no longer
 * be replaced by fresher state, so an application that sends "latest value"
 * data (input reports, controller state over RFCOMM, ...) checks this and
 * skips or coalesces a sample instead of queueing it. max_inflight 0 means
 * no in-flight cap (same as floss_hci_acl_can_send_now()). This is the rule
 * the HID and HID-over-GATT input paths use.
 */
bool floss_hci_acl_can_send_fresh(uint16_t handle, uint8_t max_inflight);

/** Free controller ACL buffers right now. */
uint16_t floss_hci_acl_credits(void);

/** Chipset's completion batching factor (>= 1), see floss_chipset_t.completion_batch. */
uint8_t floss_hci_completion_batch(void);

/* ------------------------------------------------------------------------ */
/* Lifecycle (called by floss.c)                                            */
/* ------------------------------------------------------------------------ */

void floss_hci_init(floss_transport_t *transport, const floss_chipset_t *chipset);
/** Open the transport and start the bring-up sequence. */
floss_err_t floss_hci_start(void);
/** Pump transport RX and flush TX queues. Called by floss_poll(). */
void floss_hci_poll(void);
/** True once bring-up finished. */
bool floss_hci_ready(void);

/** Optionally override our device address before floss_start() (needs chipset support). */
void floss_hci_set_addr_override(const floss_addr_t *addr);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_HCI_H */
