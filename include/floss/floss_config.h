/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_config.h - Compile-time configuration.
 *
 * Every value here is a default. Override any of them by defining
 * FLOSS_USER_CONFIG_FILE (e.g. -DFLOSS_USER_CONFIG_FILE=\"my_floss_config.h\")
 * or by passing -DNAME=value on the compiler command line.
 *
 * All memory used by the stack is sized by these values. There is no heap use.
 */
#ifndef FLOSS_CONFIG_H
#define FLOSS_CONFIG_H

#ifdef FLOSS_USER_CONFIG_FILE
#include FLOSS_USER_CONFIG_FILE
#endif

/* ------------------------------------------------------------------------ */
/* Memory sizing                                                            */
/* ------------------------------------------------------------------------ */

/** Simultaneous baseband (ACL) links. A gamepad needs 1; 2 allows a spare. */
#ifndef FLOSS_MAX_CONNECTIONS
#define FLOSS_MAX_CONNECTIONS 2
#endif

/** Simultaneous L2CAP channels across all links (HID uses 2 + SDP 1). */
#ifndef FLOSS_MAX_L2CAP_CHANNELS
#define FLOSS_MAX_L2CAP_CHANNELS 6
#endif

/** Number of packet buffers in the shared pool (commands, ACL TX, reassembly). */
#ifndef FLOSS_BUF_COUNT
#define FLOSS_BUF_COUNT 16
#endif

/**
 * Payload capacity of one pool buffer in bytes (excluding headroom).
 * Must hold: HCI ACL header (4) + L2CAP header (4) + FLOSS_L2CAP_MTU,
 * and the largest HCI command (3 + 255).
 */
#ifndef FLOSS_BUF_SIZE
#define FLOSS_BUF_SIZE 704
#endif

/**
 * Bytes reserved in front of each buffer's data so that lower layers can
 * prepend headers without copying (zero-copy TX). Covers the largest
 * transport header (FLOSS_TRANSPORT_MAX_HEADROOM) plus HCI + L2CAP headers.
 */
#ifndef FLOSS_TRANSPORT_MAX_HEADROOM
#define FLOSS_TRANSPORT_MAX_HEADROOM 4
#endif
#define FLOSS_BUF_HEADROOM (((FLOSS_TRANSPORT_MAX_HEADROOM + 4 /*ACL*/ + 4 /*L2CAP*/ + 2 /*HIDP+ID*/) + 3) & ~3)

/** Our receive MTU for dynamic L2CAP channels (672 is the protocol default). */
#ifndef FLOSS_L2CAP_MTU
#define FLOSS_L2CAP_MTU 672
#endif

/** Max queued HCI commands waiting for controller credits. */
#ifndef FLOSS_HCI_CMD_QUEUE_LEN
#define FLOSS_HCI_CMD_QUEUE_LEN 16
#endif

/** HCI command timeout in microseconds. */
#ifndef FLOSS_HCI_CMD_TIMEOUT_US
#define FLOSS_HCI_CMD_TIMEOUT_US 2000000u
#endif

/**
 * Max PDUs queued per link waiting for controller buffers. A peer that stops
 * acknowledging while it keeps sending requests would otherwise pin every
 * pool buffer with responses; past this depth sends fail with BUSY.
 */
#ifndef FLOSS_ACL_TXQ_MAX
#define FLOSS_ACL_TXQ_MAX (FLOSS_BUF_COUNT / 2)
#endif

/** Max ACL packets we track in flight per link (for ack-latency stats). */
#ifndef FLOSS_ACL_INFLIGHT_MAX
#define FLOSS_ACL_INFLIGHT_MAX 16
#endif

/** Max bonded peers remembered in persistent storage. */
#ifndef FLOSS_MAX_BONDS
#define FLOSS_MAX_BONDS 4
#endif

/** Max SDP service records registered at once. */
#ifndef FLOSS_SDP_MAX_RECORDS
#define FLOSS_SDP_MAX_RECORDS 4
#endif

/** Max size of a single HID input/output/feature report payload (excluding ID). */
#ifndef FLOSS_HID_MAX_REPORT
#define FLOSS_HID_MAX_REPORT 64
#endif

/** Max distinct input report IDs the HID mailbox tracks (latest-wins per ID). */
#ifndef FLOSS_HID_MAX_INPUT_IDS
#define FLOSS_HID_MAX_INPUT_IDS 4
#endif

/* ------------------------------------------------------------------------ */
/* Optional protocol families                                               */
/* ------------------------------------------------------------------------ */

/** Low Energy: LE GAP (advertising/connections), ATT/GATT server, SMP, HID-over-GATT. */
#ifndef FLOSS_ENABLE_LE
#define FLOSS_ENABLE_LE 1
#endif

/** RFCOMM multiplexer + serial port service. */
#ifndef FLOSS_ENABLE_RFCOMM
#define FLOSS_ENABLE_RFCOMM 1
#endif

/** Point-to-point pad <-> dongle link profile (floss_link.h). */
#ifndef FLOSS_ENABLE_LINK
#define FLOSS_ENABLE_LINK 1
#endif

/** Max simultaneous RFCOMM data channels (DLCs). */
#ifndef FLOSS_RFCOMM_MAX_DLC
#define FLOSS_RFCOMM_MAX_DLC 2
#endif

/** ATT MTU we offer on LE links. */
#ifndef FLOSS_ATT_MTU
#define FLOSS_ATT_MTU 185
#endif

/* ------------------------------------------------------------------------ */
/* Debugging                                                                */
/* ------------------------------------------------------------------------ */

/* Log levels (see floss_debug.h). */
#define FLOSS_LOG_NONE  0
#define FLOSS_LOG_ERROR 1
#define FLOSS_LOG_WARN  2
#define FLOSS_LOG_INFO  3
#define FLOSS_LOG_DEBUG 4
#define FLOSS_LOG_TRACE 5

/** Compile-time ceiling for all modules. Messages above it compile to nothing. */
#ifndef FLOSS_LOG_LEVEL
#define FLOSS_LOG_LEVEL FLOSS_LOG_INFO
#endif

/** Max formatted length of a single log line. */
#ifndef FLOSS_LOG_LINE_MAX
#define FLOSS_LOG_LINE_MAX 160
#endif

/** Binary trace ring (cheap, timestamped event IDs). 0 disables. Power of two. */
#ifndef FLOSS_TRACE_ENTRIES
#define FLOSS_TRACE_ENTRIES 256
#endif

/** Collect counters and latency histograms (floss_stats()). */
#ifndef FLOSS_ENABLE_STATS
#define FLOSS_ENABLE_STATS 1
#endif

/** Allow an HCI packet snoop hook (for Wireshark capture via tools/). */
#ifndef FLOSS_ENABLE_SNOOP
#define FLOSS_ENABLE_SNOOP 1
#endif

/** Toggle HAL debug pins around hot paths (for oscilloscope/logic analyser). */
#ifndef FLOSS_ENABLE_DEBUG_PINS
#define FLOSS_ENABLE_DEBUG_PINS 0
#endif

#endif /* FLOSS_CONFIG_H */
