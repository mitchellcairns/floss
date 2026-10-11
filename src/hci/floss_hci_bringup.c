/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_hci_bringup.c - Controller bring-up sequence.
 *
 * Overview
 * --------
 * Bring-up is a chain of commands, each issued from the completion callback
 * of the previous one, so nothing ever blocks:
 *
 *   Reset
 *   chipset init script (vendor commands, optional)
 *   Read Local Version Information
 *   Read Local Supported Commands
 *   Read Local Supported Features
 *   Read BD_ADDR
 *   [vendor Set Address + Read BD_ADDR again]    if an override is set
 *   Read Buffer Size                              -> ACL MTU and credits
 *   [Write LE Host Support, LE Read Buffer Size,
 *    LE Read Local Supported Features,
 *    LE Set Event Mask]                           if the controller does LE
 *   Set Event Mask
 *   -> READY: floss_gap_on_hci_ready()
 *
 * bu_next() is the whole sequence as a table of transitions; bu_issue()
 * encodes one step; bu_cb() stores results. Non-essential failures are
 * logged and skipped (a controller that rejects, say, LE Read Local Features
 * is still usable for Classic); only a Reset that never completes stops
 * bring-up.
 */
#include "floss_hci_priv.h"

#define G floss_hci_g

#define BU_RESET_RETRIES 3

/* LMP features page 0, byte 4 bit 6: "LE Supported (Controller)". */
#define FEAT_LE_BYTE 4
#define FEAT_LE_BIT  0x40

enum {
    BU_IDLE = 0,
    BU_RESET,
    BU_SCRIPT,
    BU_VERSION,
    BU_COMMANDS,
    BU_FEATURES,
    BU_BDADDR,
    BU_SET_ADDR,
    BU_BDADDR_VERIFY,
    BU_BUFFER,
    BU_LE_HOST,
    BU_LE_BUFFER,
    BU_LE_FEATURES,
    BU_LE_EVENT_MASK,
    BU_EVENT_MASK,
    BU_DONE,
};

static floss_addr_t s_addr_override;
static bool s_addr_override_set;

void floss_hci_set_addr_override(const floss_addr_t *addr) {
    if (addr) {
        s_addr_override = *addr;
        s_addr_override_set = true;
    } else {
        s_addr_override_set = false;
    }
}

const floss_addr_t *floss_hci_addr_override(void) {
    return s_addr_override_set ? &s_addr_override : NULL;
}

/* ------------------------------------------------------------------------ */
/* Event masks                                                              */
/* ------------------------------------------------------------------------ */

/*
 * Events the stack handles. Bit (code - 1) of the 64-bit mask enables event
 * `code`. Command Complete/Status and Number Of Completed Packets cannot be
 * masked and are not listed. Anything not listed here is left disabled so
 * the controller never waits on us for an answer we would not give (e.g.
 * Remote OOB Data Request).
 */
static const uint8_t k_classic_events[] = {
    FLOSS_EVT_INQUIRY_COMPLETE,
    FLOSS_EVT_INQUIRY_RESULT,
    FLOSS_EVT_CONNECTION_COMPLETE,
    FLOSS_EVT_CONNECTION_REQUEST,
    FLOSS_EVT_DISCONNECTION_COMPLETE,
    FLOSS_EVT_AUTHENTICATION_COMPLETE,
    FLOSS_EVT_REMOTE_NAME_REQUEST_COMPLETE,
    FLOSS_EVT_ENCRYPTION_CHANGE,
    FLOSS_EVT_READ_REMOTE_FEATURES_COMPLETE,
    FLOSS_EVT_READ_REMOTE_VERSION_COMPLETE,
    FLOSS_EVT_QOS_SETUP_COMPLETE,
    FLOSS_EVT_HARDWARE_ERROR,
    FLOSS_EVT_FLUSH_OCCURRED,
    FLOSS_EVT_ROLE_CHANGE,
    FLOSS_EVT_MODE_CHANGE,
    FLOSS_EVT_PIN_CODE_REQUEST,
    FLOSS_EVT_LINK_KEY_REQUEST,
    FLOSS_EVT_LINK_KEY_NOTIFICATION,
    FLOSS_EVT_DATA_BUFFER_OVERFLOW,
    FLOSS_EVT_MAX_SLOTS_CHANGE,
    FLOSS_EVT_PAGE_SCAN_REPETITION_MODE_CHANGE,
    FLOSS_EVT_FLOW_SPECIFICATION_COMPLETE,
    FLOSS_EVT_INQUIRY_RESULT_WITH_RSSI,
    FLOSS_EVT_READ_REMOTE_EXT_FEATURES_COMPLETE,
    FLOSS_EVT_SNIFF_SUBRATING,
    FLOSS_EVT_EXTENDED_INQUIRY_RESULT,
    FLOSS_EVT_ENCRYPTION_KEY_REFRESH_COMPLETE,
    FLOSS_EVT_IO_CAPABILITY_REQUEST,
    FLOSS_EVT_IO_CAPABILITY_RESPONSE,
    FLOSS_EVT_USER_CONFIRMATION_REQUEST,
    FLOSS_EVT_USER_PASSKEY_REQUEST,
    FLOSS_EVT_SIMPLE_PAIRING_COMPLETE,
    FLOSS_EVT_LINK_SUPERVISION_TIMEOUT_CHANGED,
    FLOSS_EVT_USER_PASSKEY_NOTIFICATION,
    0x3D, /* Remote Host Supported Features Notification (peer SSP/SC support) */
};

#if FLOSS_ENABLE_LE
/* LE Meta subevents (bit = subevent - 1 of the LE event mask). */
static const uint8_t k_le_subevents[] = {
    0x01, /* LE Connection Complete */
    0x02, /* LE Advertising Report */
    0x03, /* LE Connection Update Complete */
    0x04, /* LE Read Remote Features Complete */
    0x05, /* LE Long Term Key Request */
    0x07, /* LE Data Length Change */
    0x08, /* LE Read Local P-256 Public Key Complete */
    0x09, /* LE Generate DHKey Complete */
    0x0A, /* LE Enhanced Connection Complete */
    0x0C, /* LE PHY Update Complete */
};
#endif

static void build_mask(uint8_t out[8], const uint8_t *codes, size_t n) {
    uint64_t m = 0;
    for (size_t i = 0; i < n; i++) {
        m |= 1ull << (codes[i] - 1);
    }
    floss_put_le32(out, (uint32_t)m);
    floss_put_le32(out + 4, (uint32_t)(m >> 32));
}

/* ------------------------------------------------------------------------ */
/* Sequencing                                                               */
/* ------------------------------------------------------------------------ */

static bool controller_does_le(void) {
#if FLOSS_ENABLE_LE
    return (G.info.features[FEAT_LE_BYTE] & FEAT_LE_BIT) != 0;
#else
    return false;
#endif
}

static bool script_has_more(void) {
    return G.chipset && G.chipset->init_script && G.bu_script_off < G.chipset->init_script_len;
}

/** The sequence: which step follows `done`. */
static uint8_t bu_next(uint8_t done) {
    switch (done) {
    case BU_RESET:
    case BU_SCRIPT:
        return script_has_more() ? BU_SCRIPT : BU_VERSION;
    case BU_VERSION:
        return BU_COMMANDS;
    case BU_COMMANDS:
        return BU_FEATURES;
    case BU_FEATURES:
        return BU_BDADDR;
    case BU_BDADDR:
        return (s_addr_override_set && G.chipset && G.chipset->set_addr_opcode) ? BU_SET_ADDR : BU_BUFFER;
    case BU_SET_ADDR:
        return BU_BDADDR_VERIFY;
    case BU_BDADDR_VERIFY:
        return BU_BUFFER;
    case BU_BUFFER:
        return controller_does_le() ? BU_LE_HOST : BU_EVENT_MASK;
    case BU_LE_HOST:
        return BU_LE_BUFFER;
    case BU_LE_BUFFER:
        return BU_LE_FEATURES;
    case BU_LE_FEATURES:
        return BU_LE_EVENT_MASK;
    case BU_LE_EVENT_MASK:
        return BU_EVENT_MASK;
    default:
        return BU_DONE;
    }
}

static void bu_cb(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len);
static void bu_finish(void);

static void bu_send(uint16_t opcode, const void *params, uint8_t len) {
    if (floss_hci_cmd_internal(opcode, params, len, bu_cb, NULL) != FLOSS_OK) {
        FLOSS_LOGE(HCI, "bring-up stalled: cannot queue 0x%04x", opcode);
    }
}

/**
 * Next vendor command from the chipset script ([op_lo op_hi len params..]).
 * A malformed entry ends the script rather than sending garbage.
 */
static void bu_issue_script(void) {
    const uint8_t *s = G.chipset->init_script;
    uint16_t total = G.chipset->init_script_len;
    uint16_t off = G.bu_script_off;
    if ((uint32_t)off + 3 > total || (uint32_t)off + 3 + s[off + 2] > total) {
        FLOSS_LOGE(HCI, "chipset init script malformed at offset %u, skipping rest", off);
        G.bu_script_off = total;
        G.bu_step = BU_VERSION;
        bu_send(FLOSS_HCI_READ_LOCAL_VERSION, NULL, 0);
        return;
    }
    uint8_t plen = s[off + 2];
    G.bu_script_off = (uint16_t)(off + 3 + plen);
    bu_send(floss_get_le16(s + off), s + off + 3, plen);
}

static void bu_issue(uint8_t step) {
    G.bu_step = step;
    switch (step) {
    case BU_RESET:
        bu_send(FLOSS_HCI_RESET, NULL, 0);
        break;
    case BU_SCRIPT:
        bu_issue_script();
        break;
    case BU_VERSION:
        bu_send(FLOSS_HCI_READ_LOCAL_VERSION, NULL, 0);
        break;
    case BU_COMMANDS:
        bu_send(FLOSS_HCI_READ_LOCAL_COMMANDS, NULL, 0);
        break;
    case BU_FEATURES:
        bu_send(FLOSS_HCI_READ_LOCAL_FEATURES, NULL, 0);
        break;
    case BU_BDADDR:
    case BU_BDADDR_VERIFY:
        bu_send(FLOSS_HCI_READ_BD_ADDR, NULL, 0);
        break;
    case BU_SET_ADDR:
        /* floss_addr_t is already in wire order (LSB first). */
        bu_send(G.chipset->set_addr_opcode, s_addr_override.b, 6);
        break;
    case BU_BUFFER:
        bu_send(FLOSS_HCI_READ_BUFFER_SIZE, NULL, 0);
        break;
    case BU_LE_HOST: {
        const uint8_t p[2] = {1 /* LE_Supported_Host */, 0 /* Simultaneous_LE_Host (ignored, must be 0) */};
        bu_send(FLOSS_HCI_WRITE_LE_HOST_SUPPORT, p, sizeof(p));
        break;
    }
    case BU_LE_BUFFER:
        bu_send(FLOSS_HCI_LE_READ_BUFFER_SIZE, NULL, 0);
        break;
    case BU_LE_FEATURES:
        bu_send(FLOSS_HCI_LE_READ_LOCAL_FEATURES, NULL, 0);
        break;
#if FLOSS_ENABLE_LE
    case BU_LE_EVENT_MASK: {
        uint8_t m[8];
        build_mask(m, k_le_subevents, sizeof(k_le_subevents));
        bu_send(FLOSS_HCI_LE_SET_EVENT_MASK, m, sizeof(m));
        break;
    }
#endif
    case BU_EVENT_MASK: {
        uint8_t m[8];
        build_mask(m, k_classic_events, sizeof(k_classic_events));
        if (controller_does_le()) {
            m[7] |= 1u << ((FLOSS_EVT_LE_META - 1) - 56); /* bit 61 */
        }
        bu_send(FLOSS_HCI_SET_EVENT_MASK, m, sizeof(m));
        break;
    }
    default:
        bu_finish();
        break;
    }
}

/** Store what a step returned. Called only for successful steps. */
static void bu_store(uint8_t step, const uint8_t *r, uint16_t n) {
    floss_controller_info_t *info = &G.info;
    switch (step) {
    case BU_VERSION:
        /* HCI_Version(1) HCI_Revision(2) LMP_Version(1) Manufacturer(2) LMP_Subversion(2) */
        if (n >= 8) {
            info->hci_version = r[0];
            info->hci_revision = floss_get_le16(r + 1);
            info->lmp_version = r[3];
            info->manufacturer = floss_get_le16(r + 4);
            info->lmp_subversion = floss_get_le16(r + 6);
        }
        break;
    case BU_COMMANDS:
        memcpy(info->commands, r, FLOSS_MIN(n, sizeof(info->commands)));
        break;
    case BU_FEATURES:
        memcpy(info->features, r, FLOSS_MIN(n, sizeof(info->features)));
        break;
    case BU_BDADDR:
    case BU_BDADDR_VERIFY:
        if (n >= 6) {
            memcpy(info->addr.b, r, 6);
        }
        if (step == BU_BDADDR_VERIFY && !floss_addr_eq(&info->addr, &s_addr_override)) {
            FLOSS_LOGW(HCI, "controller ignored the address override");
        }
        break;
    case BU_BUFFER:
        /* ACL_Data_Packet_Length(2) SCO_Data_Packet_Length(1) Total_Num_ACL(2) Total_Num_SCO(2) */
        if (n >= 5) {
            info->acl_mtu = floss_get_le16(r);
            info->acl_total = floss_get_le16(r + 3);
        }
        break;
    case BU_LE_BUFFER:
        /* LE_ACL_Data_Packet_Length(2) Total_Num_LE_ACL_Data_Packets(1); 0 = shared with BR/EDR */
        if (n >= 3) {
            info->le_acl_mtu = floss_get_le16(r);
            info->le_acl_total = r[2];
            if (info->le_acl_mtu == 0 || info->le_acl_total == 0) {
                info->le_acl_mtu = 0;
                info->le_acl_total = 0;
            }
        }
        break;
    case BU_LE_FEATURES:
        memcpy(info->le_features, r, FLOSS_MIN(n, sizeof(info->le_features)));
        break;
    default:
        break;
    }
}

static void bu_cb(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len) {
    FLOSS_UNUSED(ctx);
    uint8_t step = G.bu_step;

    if (step == BU_RESET && status != FLOSS_HCI_SUCCESS) {
        if (status == FLOSS_HCI_STATUS_TIMEOUT && ++G.bu_reset_tries < BU_RESET_RETRIES) {
            FLOSS_LOGW(HCI, "no answer to Reset, retrying (%u/%u)", G.bu_reset_tries + 1, BU_RESET_RETRIES);
            bu_issue(BU_RESET);
            return;
        }
        if (status == FLOSS_HCI_STATUS_TIMEOUT) {
            FLOSS_LOGE(HCI, "controller does not respond, bring-up aborted");
            G.bu_step = BU_IDLE;
            return;
        }
        FLOSS_LOGW(HCI, "Reset failed (0x%02x), continuing", status);
    }

    if (status == FLOSS_HCI_SUCCESS) {
        bu_store(step, ret, ret ? ret_len : 0);
    } else if (step != BU_RESET) {
        FLOSS_LOGW(HCI, "bring-up step 0x%04x failed (%s), continuing", opcode, floss_hci_status_str(status));
    }
    bu_issue(bu_next(step));
}

static void bu_finish(void) {
    const floss_controller_info_t *info = &G.info;
    G.bu_step = BU_DONE;
    floss_hci_acl_reset_credits();

    char s[18];
    floss_addr_to_str(&info->addr, s);
    FLOSS_LOGI(HCI, "controller %s: HCI v%u rev 0x%04x, LMP v%u sub 0x%04x, mfr 0x%04x, ACL %ux%u, LE %ux%u%s", s,
               info->hci_version, info->hci_revision, info->lmp_version, info->lmp_subversion, info->manufacturer,
               info->acl_mtu, info->acl_total, info->le_acl_mtu, info->le_acl_total,
               controller_does_le() ? (info->le_acl_total ? "" : " (shared)") : " (no LE)");

    if (info->acl_mtu == 0 || info->acl_total == 0) {
        FLOSS_LOGE(HCI, "controller reported no ACL buffers; data cannot flow");
    }

    G.ready = true;
    floss_gap_on_hci_ready();
    floss_hci_cmd_pump(); /* release commands other layers queued before READY */
}

void floss_hci_bringup_start(void) {
    G.ready = false;
    G.bu_reset_tries = 0;
    G.bu_script_off = 0;
    bu_issue(BU_RESET);
}
