/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_l2cap.h - Logical link control and adaptation (Classic, basic mode).
 *
 * Implements the signaling channel (CID 0x0001): connection request/response,
 * configuration (MTU, flush timeout, QoS passthrough), disconnection, echo,
 * information request (extended features / fixed channels). Dynamic channels
 * run in Basic mode, which is what HID, SDP and most profiles use.
 *
 * Usage (server):
 *     static const floss_l2cap_ops_t ops = { .on_open = ..., .on_data = ... };
 *     floss_l2cap_register_psm(0x0011, &ops, NULL, FLOSS_SEC_ENCRYPTED);
 *
 * Usage (client):
 *     floss_l2cap_connect(handle, 0x0011, &ops, NULL, &chan);
 */
#ifndef FLOSS_L2CAP_H
#define FLOSS_L2CAP_H

#include "floss_types.h"
#include "floss_buf.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Well-known PSMs */
#define FLOSS_PSM_SDP            0x0001
#define FLOSS_PSM_RFCOMM         0x0003
#define FLOSS_PSM_HID_CONTROL    0x0011
#define FLOSS_PSM_HID_INTERRUPT  0x0013

/* Fixed CIDs */
#define FLOSS_CID_SIGNALING      0x0001
#define FLOSS_CID_CONNECTIONLESS 0x0002
#define FLOSS_CID_DYNAMIC_FIRST  0x0040

/** L2CAP basic header length. */
#define FLOSS_L2CAP_HDR_LEN 4

/*
 * Statuses passed to on_open() when a channel does not open. Connection
 * Response results come from the peer (0x0001..0x00FF); the 0xFFxx values are
 * local outcomes and never collide with them or with HCI reasons.
 */
#define FLOSS_L2CAP_CONN_SUCCESS        0x0000
#define FLOSS_L2CAP_CONN_PENDING        0x0001
#define FLOSS_L2CAP_CONN_PSM_UNSUPP     0x0002 /**< peer refused: no such PSM */
#define FLOSS_L2CAP_CONN_SECURITY_BLOCK 0x0003 /**< peer refused: security */
#define FLOSS_L2CAP_CONN_NO_RESOURCES   0x0004 /**< peer refused: no resources */
#define FLOSS_L2CAP_CONN_INVALID_SCID   0x0006
#define FLOSS_L2CAP_CONN_SCID_IN_USE    0x0007

#define FLOSS_L2CAP_STATUS_TIMEOUT       0xFF01 /**< peer never answered a request (RTX) */
#define FLOSS_L2CAP_STATUS_REMOTE_CLOSED 0xFF02 /**< peer disconnected before open */
#define FLOSS_L2CAP_STATUS_CONFIG_FAILED 0xFF03 /**< configuration could not converge */
#define FLOSS_L2CAP_STATUS_REJECTED      0xFF04 /**< peer sent Command Reject */
#define FLOSS_L2CAP_STATUS_PROTOCOL      0xFF05 /**< malformed response */
#define FLOSS_L2CAP_STATUS_ABORTED       0xFF06 /**< generic local abort */

/** Security a PSM requires before the channel is accepted. */
typedef enum {
    FLOSS_SEC_NONE = 0,       /**< no requirement (SDP) */
    FLOSS_SEC_ENCRYPTED = 1,  /**< link must be authenticated + encrypted (HID default) */
} floss_sec_level_t;

typedef enum {
    FLOSS_L2CAP_CLOSED = 0,
    FLOSS_L2CAP_WAIT_SECURITY,   /**< waiting for authentication/encryption */
    FLOSS_L2CAP_WAIT_CONN_RSP,   /**< outgoing: request sent */
    FLOSS_L2CAP_CONFIG,          /**< exchanging configuration */
    FLOSS_L2CAP_OPEN,
    FLOSS_L2CAP_WAIT_DISC_RSP,
} floss_l2cap_state_t;

typedef struct floss_l2cap_chan floss_l2cap_chan_t;

/** Channel callbacks. All optional except on_data for useful channels. */
typedef struct {
    /** Channel open (status 0) or failed to open (status = L2CAP result / HCI status). */
    void (*on_open)(floss_l2cap_chan_t *chan, uint16_t status);
    /** Channel closed (by either side or link loss). The chan is freed after return. */
    void (*on_close)(floss_l2cap_chan_t *chan);
    /** Payload received (L2CAP header stripped). data valid only during the call. */
    void (*on_data)(floss_l2cap_chan_t *chan, const uint8_t *data, uint16_t len);
    /** Optional: a credit freed up after floss_l2cap_send returned BUSY. */
    void (*on_can_send)(floss_l2cap_chan_t *chan);
} floss_l2cap_ops_t;

struct floss_l2cap_chan {
    floss_l2cap_state_t state;
    uint16_t handle;        /**< ACL link */
    uint16_t psm;
    uint16_t local_cid;
    uint16_t remote_cid;
    uint16_t local_mtu;     /**< what we accept */
    uint16_t remote_mtu;    /**< what the peer accepts (we never send more) */
    uint16_t flush_timeout; /**< peer requested flush timeout (ms), 0xFFFF = infinite */
    bool initiator;         /**< we opened it */
    uint8_t cfg_flags;      /**< internal: config handshake progress */
    uint8_t sig_id;         /**< internal: identifier of our pending request */
    const floss_l2cap_ops_t *ops;
    void *user;
};

/**
 * Register a server PSM. Incoming connections are accepted automatically
 * once `sec` is satisfied (authentication is requested if needed).
 */
floss_err_t floss_l2cap_register_psm(uint16_t psm, const floss_l2cap_ops_t *ops, void *user, floss_sec_level_t sec);

/** Open a channel to `psm` on an existing link. *out set immediately (state != OPEN yet). */
floss_err_t floss_l2cap_connect(uint16_t handle, uint16_t psm, const floss_l2cap_ops_t *ops, void *user,
                                floss_sec_level_t sec, floss_l2cap_chan_t **out);

/** Request disconnection. on_close fires when done. */
floss_err_t floss_l2cap_disconnect(floss_l2cap_chan_t *chan);

/** Copying send. Returns FLOSS_ERR_BUSY when no buffer / credit (on_can_send fires later). */
floss_err_t floss_l2cap_send(floss_l2cap_chan_t *chan, const uint8_t *data, uint16_t len);

/**
 * Zero-copy send: allocate a buffer with floss_buf_alloc(), write the payload
 * at buf->data (len = payload), then call this. Takes ownership of buf in
 * all cases. `flags` = FLOSS_BUF_F_* (e.g. FLUSHABLE | PRIORITY for HID input).
 */
floss_err_t floss_l2cap_send_buf(floss_l2cap_chan_t *chan, floss_buf_t *buf, uint8_t flags);

/** True if a send on this channel would reach the controller immediately. */
bool floss_l2cap_can_send_now(const floss_l2cap_chan_t *chan);

/** Find an open channel by link and PSM (NULL if none). */
floss_l2cap_chan_t *floss_l2cap_find(uint16_t handle, uint16_t psm);

/**
 * Called when the peer addresses a dynamic channel we do not have (a
 * Configuration or Disconnection Request for an unknown CID, which is answered
 * with Command Reject "invalid CID"). A peer that restarted its stack but kept
 * the baseband link does exactly this, so an application can treat it as a
 * stale link and drop it. Runs inside floss_poll(): do not re-initialise the
 * stack from the hook. Reset by floss_init(); NULL clears it.
 */
typedef void (*floss_l2cap_unknown_cid_fn)(uint16_t handle, uint16_t cid);
void floss_l2cap_set_unknown_cid_hook(floss_l2cap_unknown_cid_fn fn);

/* ------------------------------------------------------------------------ */
/* Lifecycle (called by floss.c)                                            */
/* ------------------------------------------------------------------------ */

void floss_l2cap_init(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_L2CAP_H */
