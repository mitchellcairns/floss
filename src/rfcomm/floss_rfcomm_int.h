/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_rfcomm_int.h - Private RFCOMM types shared by floss_rfcomm.c (API,
 * DLC lifecycle, data path) and floss_rfcomm_mux.c (framing, multiplexer).
 *
 * Wire format of one RFCOMM frame (TS 07.10 basic option, no flags):
 *
 *   | address | control | length (1-2) | [credits] | information | FCS |
 *
 *   address  = EA(1) | C/R << 1 | DLCI << 2
 *   control  = frame type, P/F bit 0x10
 *   length   = (len << 1) | 1 for len <= 127, else two bytes little-endian
 *              7-bit groups with EA = 0 in the first
 *   credits  = only in UIH frames with P/F = 1 on a DLC using credit based
 *              flow control (RFCOMM addition to TS 07.10)
 *   FCS      = CRC-8 over address+control (UIH) or address+control+length
 *              (all other frames); the information field is never covered.
 *
 * Not part of the public API.
 */
#ifndef FLOSS_RFCOMM_INT_H
#define FLOSS_RFCOMM_INT_H

#include "floss_internal.h"
#include "floss/floss_rfcomm.h"

#if FLOSS_ENABLE_RFCOMM

/* Logging: there is no RFCOMM module id yet, so RFCOMM logs under L2CAP with
 * an "rfcomm:" prefix (the format string must be a literal). */
#define RFC_LOGE(...) FLOSS_LOGE(L2CAP, "rfcomm: " __VA_ARGS__)
#define RFC_LOGW(...) FLOSS_LOGW(L2CAP, "rfcomm: " __VA_ARGS__)
#define RFC_LOGI(...) FLOSS_LOGI(L2CAP, "rfcomm: " __VA_ARGS__)
#define RFC_LOGD(...) FLOSS_LOGD(L2CAP, "rfcomm: " __VA_ARGS__)

/* ------------------------------------------------------------------------ */
/* Frame constants                                                          */
/* ------------------------------------------------------------------------ */

#define RFC_EA   0x01
#define RFC_CR   0x02
#define RFC_PF   0x10

#define RFC_SABM 0x2F
#define RFC_UA   0x63
#define RFC_DM   0x0F
#define RFC_DISC 0x43
#define RFC_UIH  0xEF

/* Multiplexer control command types (type octet with EA and C/R cleared). */
#define RFC_MCC_PN    0x80 /* DLC parameter negotiation */
#define RFC_MCC_PSC   0x40 /* power saving control (unsupported -> NSC) */
#define RFC_MCC_CLD   0xC0 /* multiplexer close down (unsupported -> NSC) */
#define RFC_MCC_TEST  0x20
#define RFC_MCC_FCON  0xA0
#define RFC_MCC_FCOFF 0x60
#define RFC_MCC_MSC   0xE0 /* modem status */
#define RFC_MCC_NSC   0x10 /* non supported command (response only) */
#define RFC_MCC_RPN   0x90 /* remote port negotiation */
#define RFC_MCC_RLS   0x50 /* remote line status */
#define RFC_MCC_SNC   0xD0 /* service negotiation (unsupported -> NSC) */

/* PN convergence layer field (upper nibble of octet 2). */
#define RFC_PN_CL_CFC_REQ 0xF0 /* initiator: "I support credit based flow control" */
#define RFC_PN_CL_CFC_RSP 0xE0 /* responder: "accepted" */

/** Largest frame queued on the control path (PN/RPN frames are 14 bytes). */
#define RFC_CTRL_FRAME_MAX 20

/* ------------------------------------------------------------------------ */
/* Multiplexer session                                                      */
/* ------------------------------------------------------------------------ */

typedef enum {
    RFC_MUX_FREE = 0,
    RFC_MUX_WAIT_L2CAP, /* initiator: L2CAP channel opening */
    RFC_MUX_IDLE,       /* L2CAP open, DLCI 0 not established (responder waits for SABM) */
    RFC_MUX_WAIT_UA,    /* initiator: SABM on DLCI 0 sent */
    RFC_MUX_OPEN,
    RFC_MUX_CLOSING,    /* initiator: DISC on DLCI 0 sent */
} floss_rfcomm_mux_state_t;

typedef struct floss_rfcomm_mux {
    floss_rfcomm_mux_state_t state;
    floss_l2cap_chan_t *chan;
    uint16_t handle;
    bool initiator;   /* we sent SABM on DLCI 0: decides every C/R bit */
    bool cfc;         /* credit based flow control negotiated on this session */
    bool peer_fcoff;  /* non-CFC aggregate flow control: peer sent FCoff */

    /* Control frames that L2CAP could not take yet (FIFO, sent before data). */
    uint8_t q_head;
    uint8_t q_count;
    uint8_t q_len[FLOSS_RFCOMM_CTRL_QUEUE_LEN];
    uint8_t q_frame[FLOSS_RFCOMM_CTRL_QUEUE_LEN][RFC_CTRL_FRAME_MAX];
} floss_rfcomm_mux_t;

typedef struct {
    uint8_t channel; /* 0 = free slot */
    const floss_rfcomm_ops_t *ops;
    void *user;
} floss_rfcomm_server_t;

/** All module state (no heap: sized by FLOSS_MAX_CONNECTIONS / FLOSS_RFCOMM_MAX_DLC). */
typedef struct {
    floss_rfcomm_mux_t mux[FLOSS_MAX_CONNECTIONS];
    floss_rfcomm_dlc_t dlc[FLOSS_RFCOMM_MAX_DLC];
    floss_rfcomm_server_t server[FLOSS_RFCOMM_MAX_SERVERS];
    floss_timer_t retry_timer; /* buffer pool was empty: retry senders later */
} floss_rfcomm_ctx_t;

extern floss_rfcomm_ctx_t floss_rfcomm_ctx;

/* ------------------------------------------------------------------------ */
/* floss_rfcomm_mux.c                                                       */
/* ------------------------------------------------------------------------ */

/** TS 07.10 FCS over len bytes (ready to append). */
uint8_t floss_rfcomm_fcs(const uint8_t *p, uint16_t len);

/**
 * Build a frame header into hdr (FLOSS_RFCOMM_HEADROOM bytes). credits < 0
 * means "no credit byte". Returns the header length; *fcs gets the FCS for
 * the frame (which does not depend on the information field).
 */
uint8_t floss_rfcomm_build_hdr(uint8_t *hdr, const floss_rfcomm_mux_t *mux, uint8_t dlci, uint8_t type, bool pf,
                               uint16_t info_len, int credits, uint8_t *fcs);

/**
 * Send a small frame on the control path: straight to L2CAP if nothing is
 * queued, else (or when L2CAP is busy) appended to the mux's FIFO.
 */
floss_err_t floss_rfcomm_mux_send_frame(floss_rfcomm_mux_t *mux, uint8_t dlci, uint8_t type, bool pf,
                                        const uint8_t *info, uint16_t len, int credits);

/** Send one multiplexer control message on DLCI 0. */
floss_err_t floss_rfcomm_mux_send_mcc(floss_rfcomm_mux_t *mux, uint8_t type, bool command, const uint8_t *val,
                                      uint16_t len);

/** Push queued control frames to L2CAP. Returns true when the queue is empty. */
bool floss_rfcomm_mux_flush(floss_rfcomm_mux_t *mux);

/** Send PN for an initiator DLC (DLC_WAIT_PN). */
floss_err_t floss_rfcomm_mux_send_pn(floss_rfcomm_mux_t *mux, floss_rfcomm_dlc_t *dlc, bool command);

/** Send our MSC command for a DLC. */
floss_err_t floss_rfcomm_mux_send_msc(floss_rfcomm_mux_t *mux, floss_rfcomm_dlc_t *dlc);

/** Largest N1 this L2CAP channel can carry. */
uint16_t floss_rfcomm_mux_max_mtu(const floss_rfcomm_mux_t *mux);

/** The L2CAP callbacks for PSM 0x0003 (incoming and outgoing). */
extern const floss_l2cap_ops_t floss_rfcomm_l2cap_ops;

/* ------------------------------------------------------------------------ */
/* floss_rfcomm.c                                                           */
/* ------------------------------------------------------------------------ */

floss_rfcomm_mux_t *floss_rfcomm_mux_alloc(void);
void floss_rfcomm_mux_free(floss_rfcomm_mux_t *mux);
floss_rfcomm_mux_t *floss_rfcomm_mux_by_handle(uint16_t handle);

const floss_rfcomm_server_t *floss_rfcomm_server_find(uint8_t channel);

floss_rfcomm_dlc_t *floss_rfcomm_dlc_find(const floss_rfcomm_mux_t *mux, uint8_t dlci);
/** Take a free DLC slot and bind it to mux/dlci with protocol defaults. */
floss_rfcomm_dlc_t *floss_rfcomm_dlc_alloc(floss_rfcomm_mux_t *mux, uint8_t dlci);
/** True if `dlci` names a server channel on our side of this session. */
bool floss_rfcomm_dlci_is_local(const floss_rfcomm_mux_t *mux, uint8_t dlci);

/** DLC became open: report on_open(0), send MSC, top up credits. */
void floss_rfcomm_dlc_opened(floss_rfcomm_dlc_t *dlc);
/**
 * DLC is gone: report on_close (if it was open) or on_open(status) (if it
 * never opened and was not cancelled), then free the slot.
 */
void floss_rfcomm_dlc_closed(floss_rfcomm_dlc_t *dlc, uint8_t status);

/** Received a UIH frame on an open DLC (credits already extracted). */
void floss_rfcomm_dlc_rx(floss_rfcomm_dlc_t *dlc, int credits, const uint8_t *data, uint16_t len);

/** Credits granted to the peer: auto top-up after data was consumed. */
void floss_rfcomm_dlc_replenish(floss_rfcomm_dlc_t *dlc);

/** More transmit capacity may exist: fire on_can_send for waiting DLCs on mux. */
void floss_rfcomm_mux_kick(floss_rfcomm_mux_t *mux);

/** Multiplexer reached OPEN (initiator): start pending DLCs. */
void floss_rfcomm_mux_started(floss_rfcomm_mux_t *mux);

/** Multiplexer gone (DISC 0, DM 0 or L2CAP closed): close every DLC on it. */
void floss_rfcomm_mux_close_all(floss_rfcomm_mux_t *mux, uint8_t status);

/** A DLC closed: initiator shuts the multiplexer down when none remain. */
void floss_rfcomm_mux_maybe_shutdown(floss_rfcomm_mux_t *mux);

/* ------------------------------------------------------------------------ */
/* floss_spp.c                                                              */
/* ------------------------------------------------------------------------ */

/** Forget registered serial port records (floss_init resets SDP as well). */
void floss_spp_reset(void);

#endif /* FLOSS_ENABLE_RFCOMM */

#endif /* FLOSS_RFCOMM_INT_H */
