/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss.h - Umbrella header and top-level lifecycle.
 *
 * ===========================================================================
 *  LAYER MAP (each box is one directory; arrows are the only allowed calls)
 *
 *   APP  (examples/gamepad)          your product code
 *    |   floss_hid_*, floss_gap_*
 *   HID  (src/hid)                   HIDP transactions, report mailbox, pacing
 *    |   floss_l2cap_*, floss_sdp_*
 *   L2CAP (src/l2cap) / SDP (src/sdp) channels, signaling / service records
 *    |   floss_hci_acl_*            GAP (src/gap): visibility, pairing, bonds
 *   HCI  (src/hci)                   commands, events, ACL credits, links
 *    |   floss_transport_t
 *   TRANSPORT (transport/..)        H4 stream, CYW43 shared bus, sim, ...
 *    |
 *   HAL  (ports/<platform>)          time, storage, log, entropy, critical
 *
 *  CORE (src/core): buffer pool, timers, logs/trace/stats - used by all.
 * ===========================================================================
 *
 * Minimal program:
 *
 *     floss_transport_t *t = my_port_transport();
 *     floss_init(&(floss_init_t){ .transport = t });
 *     floss_hid_init(&hid_cfg, &hid_cb);
 *     floss_start();
 *     for (;;) {
 *         uint32_t idle_us = floss_poll();
 *         ... app work, or sleep up to idle_us ...
 *     }
 */
#ifndef FLOSS_H
#define FLOSS_H

#include "floss_config.h"
#include "floss_types.h"
#include "floss_hal.h"
#include "floss_transport.h"
#include "floss_buf.h"
#include "floss_sched.h"
#include "floss_debug.h"
#include "floss_hci.h"
#include "floss_gap.h"
#include "floss_l2cap.h"
#include "floss_sdp.h"
#include "floss_hid.h"
#if FLOSS_ENABLE_LE
#include "floss_le.h"
#include "floss_gatt.h"
#include "floss_hogp.h"
#include "floss_smp.h"
#include "floss_le_central.h"
#endif
#if FLOSS_ENABLE_RFCOMM
#include "floss_rfcomm.h"
#endif
#if FLOSS_ENABLE_LINK
#include "floss_link.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define FLOSS_VERSION_MAJOR 0
#define FLOSS_VERSION_MINOR 1
#define FLOSS_VERSION_PATCH 0
#define FLOSS_VERSION_STRING "0.1.0"

typedef struct {
    /** HCI transport (required). */
    floss_transport_t *transport;
    /** Controller quirks (optional, NULL = generic). */
    const floss_chipset_t *chipset;
} floss_init_t;

/** Initialise every layer. No radio activity yet. */
floss_err_t floss_init(const floss_init_t *init);

/** Open the transport and bring the controller up (asynchronous; READY GAP event when done). */
floss_err_t floss_start(void);

/**
 * Run the stack once: pump the transport, process events, fire due timers,
 * flush queued TX. Never blocks.
 *
 * @return microseconds until the stack next needs to run on its own (next
 *         timer). The stack also needs to run whenever the transport has
 *         data (e.g. a host-wake IRQ). Sleeping up to this value is safe;
 *         calling more often is always safe.
 */
uint32_t floss_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_H */
