/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_hci_status.c - Human readable names for HCI status codes, and (in
 * debug builds) for the opcodes and events the stack uses.
 *
 * Overview
 * --------
 * Status names are always compiled in: they appear in warnings about failed
 * commands and in disconnect reasons, which matter in release logs too. The
 * opcode/event tables only exist when FLOSS_LOG_LEVEL >= DEBUG; otherwise
 * floss_hci_priv.h turns the lookups into empty strings. Names follow the
 * HCI error code table of the core specification (codes 0x00-0x45).
 */
#include "floss_hci_priv.h"

static const char *const k_status_names[] = {
    /* 0x00 */ "Success",
    /* 0x01 */ "Unknown HCI Command",
    /* 0x02 */ "Unknown Connection Identifier",
    /* 0x03 */ "Hardware Failure",
    /* 0x04 */ "Page Timeout",
    /* 0x05 */ "Authentication Failure",
    /* 0x06 */ "PIN or Key Missing",
    /* 0x07 */ "Memory Capacity Exceeded",
    /* 0x08 */ "Connection Timeout",
    /* 0x09 */ "Connection Limit Exceeded",
    /* 0x0A */ "Synchronous Connection Limit Exceeded",
    /* 0x0B */ "Connection Already Exists",
    /* 0x0C */ "Command Disallowed",
    /* 0x0D */ "Rejected: Limited Resources",
    /* 0x0E */ "Rejected: Security Reasons",
    /* 0x0F */ "Rejected: Unacceptable Address",
    /* 0x10 */ "Connection Accept Timeout",
    /* 0x11 */ "Unsupported Feature or Parameter",
    /* 0x12 */ "Invalid HCI Command Parameters",
    /* 0x13 */ "Remote User Terminated",
    /* 0x14 */ "Remote Terminated: Low Resources",
    /* 0x15 */ "Remote Terminated: Power Off",
    /* 0x16 */ "Terminated by Local Host",
    /* 0x17 */ "Repeated Attempts",
    /* 0x18 */ "Pairing Not Allowed",
    /* 0x19 */ "Unknown LMP PDU",
    /* 0x1A */ "Unsupported Remote Feature",
    /* 0x1B */ "SCO Offset Rejected",
    /* 0x1C */ "SCO Interval Rejected",
    /* 0x1D */ "SCO Air Mode Rejected",
    /* 0x1E */ "Invalid LMP/LL Parameters",
    /* 0x1F */ "Unspecified Error",
    /* 0x20 */ "Unsupported LMP/LL Parameter Value",
    /* 0x21 */ "Role Change Not Allowed",
    /* 0x22 */ "LMP/LL Response Timeout",
    /* 0x23 */ "LMP Transaction / LL Procedure Collision",
    /* 0x24 */ "LMP PDU Not Allowed",
    /* 0x25 */ "Encryption Mode Not Acceptable",
    /* 0x26 */ "Link Key Cannot Be Changed",
    /* 0x27 */ "Requested QoS Not Supported",
    /* 0x28 */ "Instant Passed",
    /* 0x29 */ "Pairing With Unit Key Not Supported",
    /* 0x2A */ "Different Transaction Collision",
    /* 0x2B */ "Reserved (0x2B)",
    /* 0x2C */ "QoS Unacceptable Parameter",
    /* 0x2D */ "QoS Rejected",
    /* 0x2E */ "Channel Classification Not Supported",
    /* 0x2F */ "Insufficient Security",
    /* 0x30 */ "Parameter Out of Mandatory Range",
    /* 0x31 */ "Reserved (0x31)",
    /* 0x32 */ "Role Switch Pending",
    /* 0x33 */ "Reserved (0x33)",
    /* 0x34 */ "Reserved Slot Violation",
    /* 0x35 */ "Role Switch Failed",
    /* 0x36 */ "Extended Inquiry Response Too Large",
    /* 0x37 */ "Simple Pairing Not Supported by Host",
    /* 0x38 */ "Host Busy - Pairing",
    /* 0x39 */ "Rejected: No Suitable Channel",
    /* 0x3A */ "Controller Busy",
    /* 0x3B */ "Unacceptable Connection Parameters",
    /* 0x3C */ "Advertising Timeout",
    /* 0x3D */ "Terminated: MIC Failure",
    /* 0x3E */ "Failed to Establish / Sync Timeout",
    /* 0x3F */ "Reserved (0x3F)",
    /* 0x40 */ "Coarse Clock Adjustment Rejected",
    /* 0x41 */ "Type0 Submap Not Defined",
    /* 0x42 */ "Unknown Advertising Identifier",
    /* 0x43 */ "Limit Reached",
    /* 0x44 */ "Operation Cancelled by Host",
    /* 0x45 */ "Packet Too Long",
};

const char *floss_hci_status_str(uint8_t status) {
    if (status < FLOSS_ARRAY_SIZE(k_status_names)) {
        return k_status_names[status];
    }
    if (status == FLOSS_HCI_STATUS_TIMEOUT) {
        return "Command Timeout (host)";
    }
    return "Unknown Status";
}

#if FLOSS_LOG_LEVEL >= FLOSS_LOG_DEBUG

typedef struct {
    uint16_t opcode;
    const char *name;
} floss_hci_op_name_t;

static const floss_hci_op_name_t k_op_names[] = {
    {FLOSS_HCI_INQUIRY, "Inquiry"},
    {FLOSS_HCI_INQUIRY_CANCEL, "Inquiry Cancel"},
    {FLOSS_HCI_CREATE_CONNECTION, "Create Connection"},
    {FLOSS_HCI_DISCONNECT, "Disconnect"},
    {FLOSS_HCI_CREATE_CONNECTION_CANCEL, "Create Connection Cancel"},
    {FLOSS_HCI_ACCEPT_CONNECTION_REQUEST, "Accept Connection Request"},
    {FLOSS_HCI_REJECT_CONNECTION_REQUEST, "Reject Connection Request"},
    {FLOSS_HCI_LINK_KEY_REQUEST_REPLY, "Link Key Request Reply"},
    {FLOSS_HCI_LINK_KEY_REQUEST_NEG_REPLY, "Link Key Request Negative Reply"},
    {FLOSS_HCI_PIN_CODE_REQUEST_REPLY, "PIN Code Request Reply"},
    {FLOSS_HCI_PIN_CODE_REQUEST_NEG_REPLY, "PIN Code Request Negative Reply"},
    {FLOSS_HCI_AUTHENTICATION_REQUESTED, "Authentication Requested"},
    {FLOSS_HCI_SET_CONNECTION_ENCRYPTION, "Set Connection Encryption"},
    {FLOSS_HCI_REMOTE_NAME_REQUEST, "Remote Name Request"},
    {FLOSS_HCI_READ_REMOTE_FEATURES, "Read Remote Features"},
    {FLOSS_HCI_READ_REMOTE_VERSION, "Read Remote Version"},
    {FLOSS_HCI_IO_CAPABILITY_REQUEST_REPLY, "IO Capability Request Reply"},
    {FLOSS_HCI_USER_CONFIRM_REQUEST_REPLY, "User Confirmation Request Reply"},
    {FLOSS_HCI_USER_CONFIRM_REQUEST_NEG_REPLY, "User Confirmation Request Negative Reply"},
    {FLOSS_HCI_USER_PASSKEY_REQUEST_REPLY, "User Passkey Request Reply"},
    {FLOSS_HCI_USER_PASSKEY_REQUEST_NEG_REPLY, "User Passkey Request Negative Reply"},
    {FLOSS_HCI_IO_CAPABILITY_REQUEST_NEG_REPLY, "IO Capability Request Negative Reply"},
    {FLOSS_HCI_SNIFF_MODE, "Sniff Mode"},
    {FLOSS_HCI_EXIT_SNIFF_MODE, "Exit Sniff Mode"},
    {FLOSS_HCI_QOS_SETUP, "QoS Setup"},
    {FLOSS_HCI_ROLE_DISCOVERY, "Role Discovery"},
    {FLOSS_HCI_SWITCH_ROLE, "Switch Role"},
    {FLOSS_HCI_WRITE_LINK_POLICY_SETTINGS, "Write Link Policy Settings"},
    {FLOSS_HCI_WRITE_DEFAULT_LINK_POLICY, "Write Default Link Policy"},
    {FLOSS_HCI_FLOW_SPECIFICATION, "Flow Specification"},
    {FLOSS_HCI_SNIFF_SUBRATING, "Sniff Subrating"},
    {FLOSS_HCI_SET_EVENT_MASK, "Set Event Mask"},
    {FLOSS_HCI_RESET, "Reset"},
    {FLOSS_HCI_SET_EVENT_FILTER, "Set Event Filter"},
    {FLOSS_HCI_DELETE_STORED_LINK_KEY, "Delete Stored Link Key"},
    {FLOSS_HCI_WRITE_LOCAL_NAME, "Write Local Name"},
    {FLOSS_HCI_READ_LOCAL_NAME, "Read Local Name"},
    {FLOSS_HCI_WRITE_CONNECTION_ACCEPT_TIMEOUT, "Write Connection Accept Timeout"},
    {FLOSS_HCI_WRITE_PAGE_TIMEOUT, "Write Page Timeout"},
    {FLOSS_HCI_WRITE_SCAN_ENABLE, "Write Scan Enable"},
    {FLOSS_HCI_WRITE_PAGE_SCAN_ACTIVITY, "Write Page Scan Activity"},
    {FLOSS_HCI_WRITE_INQUIRY_SCAN_ACTIVITY, "Write Inquiry Scan Activity"},
    {FLOSS_HCI_WRITE_AUTHENTICATION_ENABLE, "Write Authentication Enable"},
    {FLOSS_HCI_WRITE_CLASS_OF_DEVICE, "Write Class Of Device"},
    {FLOSS_HCI_WRITE_AUTOMATIC_FLUSH_TIMEOUT, "Write Automatic Flush Timeout"},
    {FLOSS_HCI_WRITE_LINK_SUPERVISION_TIMEOUT, "Write Link Supervision Timeout"},
    {FLOSS_HCI_WRITE_CURRENT_IAC_LAP, "Write Current IAC LAP"},
    {FLOSS_HCI_WRITE_INQUIRY_SCAN_TYPE, "Write Inquiry Scan Type"},
    {FLOSS_HCI_WRITE_INQUIRY_MODE, "Write Inquiry Mode"},
    {FLOSS_HCI_WRITE_PAGE_SCAN_TYPE, "Write Page Scan Type"},
    {FLOSS_HCI_WRITE_EXTENDED_INQUIRY_RESPONSE, "Write Extended Inquiry Response"},
    {FLOSS_HCI_WRITE_SIMPLE_PAIRING_MODE, "Write Simple Pairing Mode"},
    {FLOSS_HCI_SET_EVENT_MASK_PAGE_2, "Set Event Mask Page 2"},
    {FLOSS_HCI_WRITE_LE_HOST_SUPPORT, "Write LE Host Support"},
    {FLOSS_HCI_WRITE_SECURE_CONNECTIONS_HOST_SUPPORT, "Write Secure Connections Host Support"},
    {FLOSS_HCI_READ_LOCAL_VERSION, "Read Local Version"},
    {FLOSS_HCI_READ_LOCAL_COMMANDS, "Read Local Supported Commands"},
    {FLOSS_HCI_READ_LOCAL_FEATURES, "Read Local Supported Features"},
    {FLOSS_HCI_READ_BUFFER_SIZE, "Read Buffer Size"},
    {FLOSS_HCI_READ_BD_ADDR, "Read BD_ADDR"},
    {FLOSS_HCI_READ_RSSI, "Read RSSI"},
    {FLOSS_HCI_LE_SET_EVENT_MASK, "LE Set Event Mask"},
    {FLOSS_HCI_LE_READ_BUFFER_SIZE, "LE Read Buffer Size"},
    {FLOSS_HCI_LE_READ_LOCAL_FEATURES, "LE Read Local Supported Features"},
};

const char *floss_hci_opcode_str(uint16_t opcode) {
    for (size_t i = 0; i < FLOSS_ARRAY_SIZE(k_op_names); i++) {
        if (k_op_names[i].opcode == opcode) {
            return k_op_names[i].name;
        }
    }
    if ((opcode >> 10) == 0x3F) {
        return "(vendor)";
    }
    if ((opcode >> 10) == 0x08) {
        return "(LE)";
    }
    return "";
}

typedef struct {
    uint8_t code;
    const char *name;
} floss_hci_evt_name_t;

static const floss_hci_evt_name_t k_evt_names[] = {
    {FLOSS_EVT_INQUIRY_COMPLETE, "Inquiry Complete"},
    {FLOSS_EVT_INQUIRY_RESULT, "Inquiry Result"},
    {FLOSS_EVT_CONNECTION_COMPLETE, "Connection Complete"},
    {FLOSS_EVT_CONNECTION_REQUEST, "Connection Request"},
    {FLOSS_EVT_DISCONNECTION_COMPLETE, "Disconnection Complete"},
    {FLOSS_EVT_AUTHENTICATION_COMPLETE, "Authentication Complete"},
    {FLOSS_EVT_REMOTE_NAME_REQUEST_COMPLETE, "Remote Name Request Complete"},
    {FLOSS_EVT_ENCRYPTION_CHANGE, "Encryption Change"},
    {FLOSS_EVT_READ_REMOTE_FEATURES_COMPLETE, "Read Remote Features Complete"},
    {FLOSS_EVT_READ_REMOTE_VERSION_COMPLETE, "Read Remote Version Complete"},
    {FLOSS_EVT_QOS_SETUP_COMPLETE, "QoS Setup Complete"},
    {FLOSS_EVT_COMMAND_COMPLETE, "Command Complete"},
    {FLOSS_EVT_COMMAND_STATUS, "Command Status"},
    {FLOSS_EVT_HARDWARE_ERROR, "Hardware Error"},
    {FLOSS_EVT_FLUSH_OCCURRED, "Flush Occurred"},
    {FLOSS_EVT_ROLE_CHANGE, "Role Change"},
    {FLOSS_EVT_NUMBER_OF_COMPLETED_PACKETS, "Number Of Completed Packets"},
    {FLOSS_EVT_MODE_CHANGE, "Mode Change"},
    {FLOSS_EVT_PIN_CODE_REQUEST, "PIN Code Request"},
    {FLOSS_EVT_LINK_KEY_REQUEST, "Link Key Request"},
    {FLOSS_EVT_LINK_KEY_NOTIFICATION, "Link Key Notification"},
    {FLOSS_EVT_DATA_BUFFER_OVERFLOW, "Data Buffer Overflow"},
    {FLOSS_EVT_MAX_SLOTS_CHANGE, "Max Slots Change"},
    {FLOSS_EVT_PAGE_SCAN_REPETITION_MODE_CHANGE, "Page Scan Repetition Mode Change"},
    {FLOSS_EVT_FLOW_SPECIFICATION_COMPLETE, "Flow Specification Complete"},
    {FLOSS_EVT_INQUIRY_RESULT_WITH_RSSI, "Inquiry Result With RSSI"},
    {FLOSS_EVT_READ_REMOTE_EXT_FEATURES_COMPLETE, "Read Remote Extended Features Complete"},
    {FLOSS_EVT_SNIFF_SUBRATING, "Sniff Subrating"},
    {FLOSS_EVT_EXTENDED_INQUIRY_RESULT, "Extended Inquiry Result"},
    {FLOSS_EVT_ENCRYPTION_KEY_REFRESH_COMPLETE, "Encryption Key Refresh Complete"},
    {FLOSS_EVT_IO_CAPABILITY_REQUEST, "IO Capability Request"},
    {FLOSS_EVT_IO_CAPABILITY_RESPONSE, "IO Capability Response"},
    {FLOSS_EVT_USER_CONFIRMATION_REQUEST, "User Confirmation Request"},
    {FLOSS_EVT_USER_PASSKEY_REQUEST, "User Passkey Request"},
    {FLOSS_EVT_SIMPLE_PAIRING_COMPLETE, "Simple Pairing Complete"},
    {FLOSS_EVT_LINK_SUPERVISION_TIMEOUT_CHANGED, "Link Supervision Timeout Changed"},
    {FLOSS_EVT_USER_PASSKEY_NOTIFICATION, "User Passkey Notification"},
    {FLOSS_EVT_LE_META, "LE Meta"},
    {FLOSS_EVT_VENDOR, "Vendor"},
};

const char *floss_hci_event_str(uint8_t code) {
    for (size_t i = 0; i < FLOSS_ARRAY_SIZE(k_evt_names); i++) {
        if (k_evt_names[i].code == code) {
            return k_evt_names[i].name;
        }
    }
    return "";
}

#endif /* FLOSS_LOG_LEVEL >= FLOSS_LOG_DEBUG */
