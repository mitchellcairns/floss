/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_internal.h - Cross-layer hooks. NOT part of the public API.
 *
 * Upward notifications are direct function calls (no dynamic registration),
 * which keeps the hot paths short and makes the call graph greppable: search
 * for a hook name to find both ends.
 *
 *   transport --rx--> HCI --events--> GAP --(link state)--> L2CAP --> HID/SDP
 *                         --ACL PDUs---------------------> L2CAP
 */
#ifndef FLOSS_INTERNAL_H
#define FLOSS_INTERNAL_H

#include "floss/floss.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== HCI -> GAP ======================================== */

/** Controller bring-up finished; GAP applies its settings, then emits READY. */
void floss_gap_on_hci_ready(void);

/**
 * Every HCI event that HCI does not fully consume is forwarded here
 * (connection request, pairing events, mode change, encryption change, ...).
 * HCI has already updated the connection table for CONNECTION_COMPLETE /
 * DISCONNECTION_COMPLETE / MODE_CHANGE / ENCRYPTION_CHANGE / ROLE_CHANGE
 * before forwarding. params points after the 2-byte event header.
 */
void floss_gap_on_hci_event(uint8_t code, const uint8_t *params, uint8_t len);

/* ===================== GAP -> L2CAP ====================================== */

/** A link is open (incoming or outgoing). */
void floss_l2cap_on_link_up(uint16_t handle);
/** A link is gone; close every channel on it. */
void floss_l2cap_on_link_down(uint16_t handle, uint8_t reason);
/** Link security changed: encrypted==true means channels waiting for security may proceed. status!=0 = failed. */
void floss_l2cap_on_link_security(uint16_t handle, uint8_t status, bool encrypted);

/* ===================== HCI -> L2CAP ====================================== */

/** One complete, reassembled L2CAP PDU (starts at the basic header). */
void floss_l2cap_on_pdu(uint16_t handle, const uint8_t *pdu, uint16_t len);
/** Controller freed ACL buffers (Number Of Completed Packets). */
void floss_l2cap_on_credits(void);

/* ===================== L2CAP -> GAP ====================================== */

/** L2CAP needs the link authenticated + encrypted (idempotent). */
void floss_gap_require_security(uint16_t handle);

/* ===================== SDP -> GAP ======================================== */
/* (floss_gap_eir_add_uuid16 is public) */

/* ===================== HCI -> LE (FLOSS_ENABLE_LE) ======================= */
/*
 * HCI owns the connection table for LE links too: on LE Connection Complete /
 * Enhanced Connection Complete it fills a floss_conn_t with le = true, then
 * forwards the subevent. ACL PDUs on LE links go to floss_le_on_pdu() instead
 * of L2CAP (LE uses fixed channels: 0x0004 ATT, 0x0005 LE signaling,
 * 0x0006 SMP). Disconnection of an LE link calls floss_le_on_link_down().
 */
#if FLOSS_ENABLE_LE
/** Controller bring-up done (after floss_gap_on_hci_ready settings). LE configures advertising etc. */
void floss_le_on_hci_ready(void);
/** LE Meta event (code 0x3E). params point after the subevent code byte. */
void floss_le_on_hci_event(uint8_t subevent, const uint8_t *params, uint8_t len);
/** Encryption change / key refresh on an LE link (forwarded by HCI). */
void floss_le_on_encryption(uint16_t handle, uint8_t status, bool encrypted);
/** Complete L2CAP PDU (basic header included) on an LE link. */
void floss_le_on_pdu(uint16_t handle, const uint8_t *pdu, uint16_t len);
/** LE link gone. */
void floss_le_on_link_down(uint16_t handle, uint8_t reason);
/** ACL credits freed (LE buffers or shared). */
void floss_le_on_credits(void);
/** Module init (called by floss_init). */
void floss_le_init(void);

/* ---- LE (src/le/floss_le.c, ATT/GATT) <-> SMP (src/le/floss_smp*.c) ---- */

/** Implemented by LE: send `len` bytes on fixed channel `cid` (adds L2CAP header). */
floss_err_t floss_le_send_fixed(uint16_t handle, uint16_t cid, const uint8_t *data, uint16_t len);
/**
 * Implemented by LE: SMP reports the link's new security level
 * (1 = none, 2 = encrypted unauthenticated, 3 = encrypted authenticated,
 * 4 = LE Secure Connections authenticated). ATT uses it for permission checks.
 */
void floss_le_on_security_changed(uint16_t handle, uint8_t level);

/** Implemented by SMP. Called by LE. */
void floss_smp_init(void);
/** role: 0 central, 1 peripheral. */
void floss_smp_on_link_up(uint16_t handle, uint8_t role, uint8_t peer_addr_type, const floss_addr_t *peer_addr);
void floss_smp_on_link_down(uint16_t handle);
/** SMP PDU payload (after the L2CAP header) received on CID 0x0006. */
void floss_smp_on_pdu(uint16_t handle, const uint8_t *data, uint16_t len);
/** LE meta subevents SMP needs: 0x05 LTK Request, 0x08 Read Local P-256 Public Key Complete, 0x09 Generate DHKey Complete. */
void floss_smp_on_le_event(uint8_t subevent, const uint8_t *params, uint8_t len);
/** Encryption change / key refresh on an LE link. */
void floss_smp_on_encryption(uint16_t handle, uint8_t status, bool encrypted);
#endif

/* ===================== RFCOMM (FLOSS_ENABLE_RFCOMM) ====================== */
#if FLOSS_ENABLE_RFCOMM
/** Module init (called by floss_init). Registers PSM 0x0003. */
void floss_rfcomm_init(void);
#endif

/* ===================== core -> HID ======================================= */

/**
 * Give the HID input path a turn: send mailbox reports that are due and
 * retry a deferred control response. Called by floss_poll() on every
 * iteration; costs a couple of loads when there is nothing to do.
 */
void floss_hid_poll(void);

/* ===================== core ============================================== */

void floss_debug_init(void);
/** Number of GAP listeners slot. */
#define FLOSS_GAP_MAX_LISTENERS 4

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_INTERNAL_H */
