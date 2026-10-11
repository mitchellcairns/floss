/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_hci_priv.h - State and helpers shared by the files of the HCI layer.
 *
 * The HCI layer is split by concern so each file reads top to bottom:
 *
 *   floss_hci.c         transport glue, command queue + flow control,
 *                       event dispatch, connection table
 *   floss_hci_bringup.c controller bring-up sequence (Reset .. Set Event Mask)
 *   floss_hci_acl.c     ACL credits, per-link TX queues, fragmentation,
 *                       RX reassembly
 *   floss_hci_status.c  status / opcode / event names for logs
 *
 * Everything here is private to src/hci. Other layers use floss_hci.h.
 */
#ifndef FLOSS_HCI_PRIV_H
#define FLOSS_HCI_PRIV_H

#include "floss_internal.h"

/* ------------------------------------------------------------------------ */
/* Wire constants                                                           */
/* ------------------------------------------------------------------------ */

#define FLOSS_HCI_CMD_HDR_LEN 3 /* opcode(2) + parameter length(1) */
#define FLOSS_HCI_EVT_HDR_LEN 2 /* event code(1) + parameter length(1) */
#define FLOSS_HCI_ACL_HDR_LEN 4 /* handle+flags(2) + data length(2) */

/* ACL Packet_Boundary_Flag (bits 12-13 of the first header word). */
#define FLOSS_ACL_PB_FIRST_NONFLUSH 0x0 /* host->controller start, never auto-flushed (LE start too) */
#define FLOSS_ACL_PB_CONT           0x1 /* continuing fragment */
#define FLOSS_ACL_PB_FIRST_FLUSH    0x2 /* start of an automatically flushable PDU */

/* LE Meta subevents HCI itself inspects (the LE module handles the rest). */
#define FLOSS_LE_SUBEVT_CONN_COMPLETE          0x01
#define FLOSS_LE_SUBEVT_ENHANCED_CONN_COMPLETE 0x0A

/* Status passed to command callbacks when the controller never answered. */
#define FLOSS_HCI_STATUS_TIMEOUT 0xFF

/*
 * Private floss_buf_t flag bits. The public FLUSHABLE/PRIORITY bits use the
 * low bits; while HCI owns a buffer it may use the high ones.
 */
#define FLOSS_HCI_BUF_F_CONT     0x80 /* ACL TX: first fragment already sent; data/len = remainder */
#define FLOSS_HCI_BUF_F_INTERNAL 0x40 /* CMD: issued by bring-up, allowed before READY */

/* ------------------------------------------------------------------------ */
/* Layer state                                                              */
/* ------------------------------------------------------------------------ */

/** A queued command: the encoded packet plus who to tell when it completes. */
typedef struct {
    floss_buf_t *buf;
    floss_hci_cmd_cb cb;
    void *ctx;
} floss_hci_cmd_slot_t;

typedef struct {
    floss_transport_t *transport;
    const floss_chipset_t *chipset;
    floss_controller_info_t info;
    bool started; /* transport open */
    bool ready;   /* bring-up finished */

    /* --- command flow control ------------------------------------------- */
    floss_hci_cmd_slot_t cmdq[FLOSS_HCI_CMD_QUEUE_LEN]; /* ring */
    uint8_t cmdq_head;
    uint8_t cmdq_count;
    uint8_t cmd_credits;       /* Num_HCI_Command_Packets from the controller */
    bool cmd_inflight;         /* a command was sent and is awaiting its event */
    uint16_t inflight_opcode;
    floss_hci_cmd_cb inflight_cb;
    void *inflight_ctx;
    uint32_t inflight_t_us;
    floss_timer_t cmd_timer;

    /* --- links / ACL ----------------------------------------------------- */
    floss_conn_t conns[FLOSS_MAX_CONNECTIONS];
    uint16_t acl_credits;      /* free BR/EDR (or shared) controller buffers */
    uint16_t le_credits;       /* free LE buffers when the controller has a separate pool */
    uint8_t acl_rr;            /* round-robin start index for fair flushing */
    bool acl_stalled;          /* inside a credit-stall episode (counted once) */

    /* --- bring-up --------------------------------------------------------- */
    uint8_t bu_step;
    uint8_t bu_reset_tries;
    uint16_t bu_script_off;
} floss_hci_state_t;

extern floss_hci_state_t floss_hci_g;

/* ------------------------------------------------------------------------ */
/* Cross-file helpers                                                       */
/* ------------------------------------------------------------------------ */

/* floss_hci.c */
floss_err_t floss_hci_transport_send(uint8_t type, uint8_t *pkt, uint16_t len);
/** Queue a bring-up command: jumps ahead of application commands and bypasses the READY gate. */
floss_err_t floss_hci_cmd_internal(uint16_t opcode, const void *params, uint8_t len, floss_hci_cmd_cb cb, void *ctx);
/** Send queued commands if the controller has room. Safe to call any time. */
void floss_hci_cmd_pump(void);

/* floss_hci_bringup.c */
void floss_hci_bringup_start(void);
/** Override address requested by the application (NULL if none). */
const floss_addr_t *floss_hci_addr_override(void);

/* floss_hci_acl.c */
void floss_hci_acl_reset_credits(void);
void floss_hci_acl_flush(void);
void floss_hci_acl_on_rx(const uint8_t *pkt, uint16_t len);
void floss_hci_acl_on_completed(const uint8_t *params, uint8_t len);
/** Link gone: return its in-flight credits, drop queued TX and partial RX. Returns credits returned. */
uint16_t floss_hci_acl_link_down(floss_conn_t *conn);
/** Notify upper layers that credits were freed. */
void floss_hci_acl_notify_credits(void);

/* floss_hci_status.c - names for debug logs */
#if FLOSS_LOG_LEVEL >= FLOSS_LOG_DEBUG
const char *floss_hci_opcode_str(uint16_t opcode);
const char *floss_hci_event_str(uint8_t code);
#else
#define floss_hci_opcode_str(op) ((void)(op), "")
#define floss_hci_event_str(code) ((void)(code), "")
#endif

#endif /* FLOSS_HCI_PRIV_H */
