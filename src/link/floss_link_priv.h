/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_link_priv.h - Private glue of the pad <-> dongle link profile.
 *
 * The profile is one core plus one transport per radio technology:
 *
 *   floss_link.c          state, callbacks, the pad input mailbox (seqlock),
 *                         pacing, PDU encode/decode, sequence numbers, the
 *                         dongle's timing statistics, PING/PONG, HELLO,
 *                         the stored peer record
 *   floss_link_classic.c  Classic: custom PSM, inquiry/page/bond (dongle),
 *                         discoverable/connectable + idle sniff (pad)
 *   floss_link_le.c       LE: GATT link service + advertising (pad),
 *                         scan/connect/GATT client (dongle)
 *   floss_link_hook.c     floss_link_poll() trampoline (see there)
 *
 * The core never touches a radio API directly: it talks to the active
 * transport through floss_link_tp_t, and transports report link
 * events back through the floss_link_* functions below. Nothing here is
 * visible outside src/link/ (tests may include it to inspect counters).
 */
#ifndef FLOSS_LINK_PRIV_H
#define FLOSS_LINK_PRIV_H

#include "floss_internal.h"
#include "floss/floss_link.h"

/* ------------------------------------------------------------------------ */
/* Tunables (override with -D or the user config file)                      */
/* ------------------------------------------------------------------------ */

/** Distinct input report IDs the pad mailbox tracks (latest wins per ID). */
#ifndef FLOSS_LINK_MAX_INPUT_IDS
#define FLOSS_LINK_MAX_INPUT_IDS 4
#endif

/**
 * HAL store key of the peer record (who our pad / dongle is). In the
 * application key range: applications must not use it for anything else.
 */
#ifndef FLOSS_LINK_STORE_KEY_PEER
#define FLOSS_LINK_STORE_KEY_PEER 0x8100
#endif

/** Reconnect backoff: first retry after MIN, doubling up to MAX. */
#ifndef FLOSS_LINK_RECONNECT_MIN_US
#define FLOSS_LINK_RECONNECT_MIN_US 200000u
#endif
#ifndef FLOSS_LINK_RECONNECT_MAX_US
#define FLOSS_LINK_RECONNECT_MAX_US 5000000u
#endif

/**
 * floss_link_pair() gives up after this long and goes back to the bonded
 * peer - only when there is one. Without a peer, pairing is the resting
 * state and never times out.
 */
#ifndef FLOSS_LINK_PAIR_TIMEOUT_US
#define FLOSS_LINK_PAIR_TIMEOUT_US 60000000u
#endif

/**
 * rx_jitter baseline window. The pad and dongle clocks are independent
 * crystals; their offset (t_rx - t_pad) drifts by up to ~40 us/s with
 * +-20 ppm parts. The baseline is the minimum offset seen over the current
 * and the previous window, so it follows the drift with an error of at most
 * 2 windows x drift (~160 us at 40 ppm with 2 s windows) while still
 * holding a minimum taken from hundreds of reports.
 */
#ifndef FLOSS_LINK_JITTER_WINDOW_US
#define FLOSS_LINK_JITTER_WINDOW_US 2000000u
#endif

/** Classic dongle: link supervision timeout (slots). 0x0C80 = 2 s (controller default is 20 s). */
#ifndef FLOSS_LINK_CLASSIC_SUPERVISION_SLOTS
#define FLOSS_LINK_CLASSIC_SUPERVISION_SLOTS 0x0C80u
#endif

/**
 * Classic pad: page scan while waiting for its dongle (slots, interlaced).
 * 0x0100 = every 160 ms: the dongle's reconnect page finds the pad within a
 * few hundred ms. Costs ~14 % receive duty while disconnected only.
 */
#ifndef FLOSS_LINK_PAD_PSCAN_INTERVAL
#define FLOSS_LINK_PAD_PSCAN_INTERVAL 0x0100u
#endif
#ifndef FLOSS_LINK_PAD_PSCAN_WINDOW
#define FLOSS_LINK_PAD_PSCAN_WINDOW 0x0012u
#endif

/** Classic dongle: length of one inquiry round while searching (x 1.28 s). */
#ifndef FLOSS_LINK_INQUIRY_LENGTH
#define FLOSS_LINK_INQUIRY_LENGTH 4u
#endif

/** LE: a central that has not encrypted the link after this long is dropped. */
#ifndef FLOSS_LINK_LE_SECURE_TIMEOUT_US
#define FLOSS_LINK_LE_SECURE_TIMEOUT_US 10000000u
#endif

#if FLOSS_LINK_MAX_INPUT_IDS > 32
#error "FLOSS_LINK_MAX_INPUT_IDS must be <= 32 (slot bitmasks are uint32_t)"
#endif
#if FLOSS_LINK_MAX_REPORT > 255
#error "FLOSS_LINK_MAX_REPORT must fit the u8 length fields of the PDUs"
#endif

/* ------------------------------------------------------------------------ */
/* Wire format                                                              */
/* ------------------------------------------------------------------------ */

#define FLOSS_LINK_INPUT_HDR  9u  /**< type seq(2) t_pad(4) id len */
#define FLOSS_LINK_OUTPUT_HDR 5u  /**< type seq(2) id len */
#define FLOSS_LINK_PING_LEN   5u  /**< type t_dongle(4) */
#define FLOSS_LINK_PONG_LEN   9u  /**< type t_dongle(4) t_pad(4) */
#define FLOSS_LINK_HELLO_LEN  5u  /**< type version role max_report flags (longer = future fields) */
/** Largest PDU of this build (an INPUT with a full report). */
#define FLOSS_LINK_PDU_MAX (FLOSS_LINK_INPUT_HDR + FLOSS_LINK_MAX_REPORT)

/** HELLO flags. */
#define FLOSS_LINK_HELLO_F_IDLE_SNIFF 0x01 /**< pad: will request sniff when idle (dongle must allow it) */
#define FLOSS_LINK_HELLO_F_ACKS       0x02 /**< dongle: acknowledges every INPUT (ACK PDU) */

#define FLOSS_LINK_ACK_LEN 3u /**< type seq(2) */

/** Pad: no ACK for this long while reports wait: count by completion events again. */
#ifndef FLOSS_LINK_ACK_TIMEOUT_US
#define FLOSS_LINK_ACK_TIMEOUT_US 20000u
#endif

/* ------------------------------------------------------------------------ */
/* Peer record (FLOSS_LINK_STORE_KEY_PEER)                                  */
/* ------------------------------------------------------------------------ */

/** Who our peer is. The keys themselves live in GAP's / SMP's bond database. */
typedef struct {
    uint8_t transport;   /**< floss_link_transport_t it was paired over */
    uint8_t addr_type;   /**< LE identity address type (0 on Classic) */
    floss_addr_t addr;   /**< Classic address / LE identity address */
} floss_link_peer_t;

/** Current peer, false if none (or one paired over the other transport). */
bool floss_link_peer_get(floss_link_peer_t *out);
/** Remember a new peer (persisted). */
void floss_link_peer_set(uint8_t addr_type, const floss_addr_t *addr);
/** Forget the peer record (persisted). */
void floss_link_peer_clear(void);

/* ------------------------------------------------------------------------ */
/* Transport interface                                                      */
/* ------------------------------------------------------------------------ */

typedef struct {
    const char *name;
    /** floss_link_init(): register services, listeners, identity. */
    floss_err_t (*init)(void);
    /** Controller ready: enter floss_link_rest_state() (reconnect / pair / idle). */
    void (*on_ready)(void);
    /** GAP event (the core holds the only GAP listener of the profile). */
    void (*on_gap_event)(const floss_gap_evt_t *evt);
    /** floss_link_pair(): leave any link, start pairing. */
    floss_err_t (*pair)(void);
    /** The pairing window closed without a new peer: go back to the rest state. */
    void (*on_pair_timeout)(void);
    /** floss_link_forget(): old = the peer just forgotten (NULL if none). Drop bonds and the link. */
    void (*forget)(const floss_link_peer_t *old);

    /* ---- pad input path (zero copy) -------------------------------------- */
    /** May an INPUT go to the radio now without queuing (credits + in-flight cap)? */
    bool (*can_send)(void);
    /** Buffer for one PDU: *pdu = where it goes, *cap = the most the peer accepts. NULL = none now. */
    floss_buf_t *(*input_alloc)(uint8_t **pdu, uint16_t *cap);
    /** Send `len` PDU bytes written at the alloc'ed position. Takes ownership. */
    floss_err_t (*input_send)(floss_buf_t *b, uint16_t len);

    /** Control PDU (HELLO, PING, PONG, OUTPUT), copied. FLOSS_ERR_BUSY = retry later. */
    floss_err_t (*send_ctrl)(const uint8_t *pdu, uint16_t len);
    /** Pacing period for FLOSS_LINK_INTERVAL_CONN (us). */
    uint32_t (*conn_period_us)(void);

    /* ---- optional ---------------------------------------------------------- */
    /** Pad: the content of an input report changed (idle power management). */
    void (*on_input_change)(void);
    /** The peer's HELLO arrived (flags = FLOSS_LINK_HELLO_F_*). */
    void (*on_peer_hello)(uint8_t flags);
    /** Every floss_link_poll(), after the input path. Keep it to a few loads. */
    void (*poll)(void);
} floss_link_tp_t;

extern const floss_link_tp_t floss_link_classic;
extern const floss_link_tp_t floss_link_le;

/* ------------------------------------------------------------------------ */
/* Core services for the transports                                         */
/* ------------------------------------------------------------------------ */

const floss_link_config_t *floss_link_cfg(void);
static inline bool floss_link_is_pad(const floss_link_config_t *cfg) {
    return cfg->role == FLOSS_LINK_ROLE_PAD;
}
/** Change the state (logs it; arms / stops the pairing window). */
void floss_link_set_state(floss_link_state_t st);
/** What to do while not connected: PAIRING without a peer, else CONNECTING (auto_reconnect) or IDLE. */
floss_link_state_t floss_link_rest_state(void);

/** The data path is up (channel open / subscribed): state CONNECTED, HELLO, pacing, on_connected. */
void floss_link_session_up(const floss_addr_t *peer);
/** The data path is gone: stops timers, on_disconnected if it was up. The transport sets the next state. */
void floss_link_session_down(uint8_t reason);
bool floss_link_session_active(void);

/** A PDU from the peer (any type). Malformed ones are counted and dropped. */
void floss_link_rx_pdu(const uint8_t *pdu, uint16_t len);

/** Reconnect delay for the n-th consecutive failure (0 = first retry). */
uint32_t floss_link_backoff_us(uint8_t attempt);

/** Malformed / unexpected PDUs received since init or stats reset (tests, dump). */
uint32_t floss_link_rx_malformed(void);
/** In-flight cap for the input path on `handle` (max_inflight x completion batch; 0 = none). */
bool floss_link_inflight_ok(uint16_t handle);

/** Set by floss_link_init(); floss_link_poll() (floss_link_hook.c) calls it. */
extern void (*floss_link_poll_hook)(void);

#endif /* FLOSS_LINK_PRIV_H */
