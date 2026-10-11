/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_gatt_client.h - Internal hooks of the LE central role (scanner,
 * initiator: floss_le_central.c) and the GATT client (floss_gatt_client.c).
 * NOT part of the public API; the public contract is floss_le_central.h.
 *
 * The central-role modules sit next to the peripheral code and are reached
 * through four direct calls, each one line in the module that owns the
 * event (grep for the names to find both ends):
 *
 *   floss_le_init()            -> floss_le_central_init()       reset state
 *   floss_le_on_hci_event()    -> floss_le_central_on_le_event() advertising
 *                                 reports, our own connection attempts
 *   floss_le_on_link_down()    -> floss_gattc_on_link_down()     fail the
 *                                 outstanding client procedure (0xFF)
 *   floss_att_on_pdu()         -> floss_gattc_on_pdu()           server->client
 *                                 PDUs (responses, notifications, indications)
 *
 * A FLOSS device can be ATT client and ATT server on the same link at the
 * same time: requests from the peer stay with floss_att.c, everything the
 * peer sends as a server is claimed here.
 */
#ifndef FLOSS_GATT_CLIENT_H
#define FLOSS_GATT_CLIENT_H

#include "floss_internal.h"

#if FLOSS_ENABLE_LE

#ifdef __cplusplus
extern "C" {
#endif

/* ---- LE central: scanning and initiating (floss_le_central.c) ----------- */

/** Reset scanner, initiator and GATT client state (floss_le_init calls it). */
void floss_le_central_init(void);

/**
 * Every LE meta subevent, before the LE module handles it (params point after
 * the subevent code). Returns true when the event was consumed here
 * (advertising reports); connection complete events are only observed (to end
 * a pending connection attempt) and return false so the LE module still
 * opens the link.
 */
bool floss_le_central_on_le_event(uint8_t subevent, const uint8_t *params, uint8_t len);

/* ---- GATT client (floss_gatt_client.c) ----------------------------------- */

/** Reset every per-link client context (called by floss_le_central_init). */
void floss_gattc_init(void);

/**
 * ATT PDU on CID 0x0004 (payload after the L2CAP header). Returns true when
 * it is a server->client PDU (Error/any Response, Handle Value Notification,
 * Handle Value Indication) and was consumed by the client; false leaves it to
 * the ATT server.
 */
bool floss_gattc_on_pdu(uint16_t conn, const uint8_t *pdu, uint16_t len);

/** LE link gone: the outstanding procedure (if any) completes with 0xFF. */
void floss_gattc_on_link_down(uint16_t conn);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_ENABLE_LE */

#endif /* FLOSS_GATT_CLIENT_H */
