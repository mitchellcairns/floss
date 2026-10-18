/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_link.h - Point-to-point input link: a "pad" (controller) streams
 * input state to a "dongle" (USB receiver) that FLOSS runs on as well.
 *
 * When both ends run FLOSS there is no need for the HID profile on the air:
 * the link carries raw input reports with a sequence number and the pad's
 * timestamp, so the dongle can count loss and measure timing exactly, and
 * the pairing/reconnect flow is fully under our control.
 *
 * Two transports, chosen at init:
 *
 *   FLOSS_LINK_CLASSIC  dongle = central (it pages the pad; with
 *                       classic_pad_connects the pad pages the dongle and is
 *                       central), custom L2CAP PSM, active mode. The central
 *                       polls the peripheral every 2 slots: one report per
 *                       1.25 ms, 800 Hz max. classic_store_keys = false pairs
 *                       on every connection and stores nothing.
 *                       Idle power: optional sniff after inactivity (the
 *                       first input after idle waits for the next sniff
 *                       anchor plus sniff exit).
 *   FLOSS_LINK_LE       dongle = central, pad = peripheral advertising a
 *                       link service; reports are ATT notifications. Fastest
 *                       standard interval is 7.5 ms (133 Hz). Idle power:
 *                       peripheral latency lets an idle pad skip connection
 *                       events and still send at the very next event when
 *                       input changes.
 *
 * Pad input path: same latest-wins seqlock mailbox as floss_hid
 * (floss_link_set_input is ISR / other-core safe), paced or on change,
 * bounded in-flight packets (scaled by the chipset's completion_batch).
 *
 * Wire format (one PDU per L2CAP SDU / ATT value, little endian):
 *   0x01 INPUT   [seq u16][t_pad_us u32][report_id u8][len u8][data len]
 *   0x02 OUTPUT  [seq u16][report_id u8][len u8][data len]
 *   0x03 PING    [t_dongle_us u32]            (dongle -> pad)
 *   0x04 PONG    [t_dongle_us u32][t_pad_us u32] (pad -> dongle, echo)
 *   0x05 HELLO   [version u8][role u8][max_report u8][flags u8]
 * INPUT for a 13-byte report is 22 bytes + 4 L2CAP: one single-slot packet.
 */
#ifndef FLOSS_LINK_H
#define FLOSS_LINK_H

#include "floss_types.h"
#include "floss_debug.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Protocol version carried in HELLO. */
#define FLOSS_LINK_VERSION 1

/** L2CAP PSM used on Classic (dynamic range, odd, LSB of upper byte even). */
#define FLOSS_LINK_PSM 0x1001

/** Max report payload carried by one INPUT/OUTPUT PDU. */
#ifndef FLOSS_LINK_MAX_REPORT
#define FLOSS_LINK_MAX_REPORT 32
#endif

/** PDU types. */
#define FLOSS_LINK_PDU_INPUT  0x01
#define FLOSS_LINK_PDU_OUTPUT 0x02
#define FLOSS_LINK_PDU_PING   0x03
#define FLOSS_LINK_PDU_PONG   0x04
#define FLOSS_LINK_PDU_HELLO  0x05

/**
 * 128-bit UUIDs of the LE link service and its characteristics (big-endian
 * byte order as written; generated randomly for this project).
 *   service  f1055000-7a3c-4b8e-9d2f-5c6a1e0b7d10
 *   input    f1055001-...  notify          (pad -> dongle INPUT/PONG/HELLO PDUs)
 *   output   f1055002-...  write w/o resp  (dongle -> pad OUTPUT/PING/HELLO PDUs)
 */
extern const uint8_t floss_link_uuid_service[16];
extern const uint8_t floss_link_uuid_input[16];
extern const uint8_t floss_link_uuid_output[16];

typedef enum {
    FLOSS_LINK_ROLE_PAD = 0,
    FLOSS_LINK_ROLE_DONGLE = 1,
} floss_link_role_t;

typedef enum {
    FLOSS_LINK_CLASSIC = 0,
    FLOSS_LINK_LE = 1,
} floss_link_transport_t;

typedef enum {
    FLOSS_LINK_STATE_IDLE = 0,    /**< not started */
    FLOSS_LINK_STATE_PAIRING,     /**< pad: discoverable/advertising for a new dongle; dongle: searching */
    FLOSS_LINK_STATE_CONNECTING,  /**< reconnecting to the bonded peer */
    FLOSS_LINK_STATE_CONNECTED,   /**< link + channel up, reports flow */
} floss_link_state_t;

typedef struct {
    floss_link_role_t role;
    floss_link_transport_t transport;
    /**
     * Pad: name (Classic local name / LE advertised name). Dongle: name to
     * look for (prefix match; NULL = any FLOSS link pad); with
     * classic_pad_connects, the dongle's own local name instead.
     */
    const char *name;
    /**
     * Classic, the side that connects: name prefix the peer must have (NULL
     * = any FLOSS link device of the other role). Default NULL; a dongle
     * that connects falls back to `name`.
     */
    const char *peer_name;

    /* --- pad sending --------------------------------------------------- */
    /** 0 = send on change as fast as the link allows; >0 = paced period (us). LE: FLOSS_LINK_INTERVAL_CONN = one per connection event. */
    uint32_t report_interval_us;
    /** Max un-acknowledged packets (x chipset completion_batch). Default 2. */
    uint8_t max_inflight;

    /* --- Classic -------------------------------------------------------- */
    /**
     * Who connects. false (default): the dongle searches for and pages the
     * pad, which waits connectable. true: the pad searches for and pages the
     * dongle, which waits; the pad is then central. Whichever side connects
     * searches while pairing, pages its remembered peer, requests the poll
     * latency and runs the AFH policy; the waiting side is discoverable while
     * pairing, connectable otherwise, and answers neither while connected.
     * Both sides must agree. Sniff for idle power stays a pad decision.
     */
    bool classic_pad_connects;
    /**
     * Keep the peer's link key (bond). Default true. false: this side stores
     * no keys: every connection pairs again (just works), the key lives for
     * that connection only. The waiting side then accepts any link device of
     * the other role while it has no connection (no pairing window needed)
     * and remembers nobody; the connecting side remembers its last peer's
     * address only, pages it first and searches for any other one when it
     * does not answer. Turns off bonding in GAP's security settings.
     */
    bool classic_store_keys;
    /** Connecting side: poll latency requested with QoS setup after connecting (us). Default 1250. */
    uint32_t classic_poll_us;
    /**
     * Connecting side: how long a page to the remembered peer tries before the
     * search for any other one starts (ms). A waiting peer that is on answers
     * within its page scan interval (about 160 ms for a link device). 0 = GAP's
     * default (5120). Default 0.
     */
    uint16_t classic_page_timeout_ms;
    /** Power: enter sniff after this many ms without input changes (0 = never). Default 0. */
    uint32_t classic_idle_ms;
    /** Power: sniff interval (slots) used while idle. Default 16 (10 ms). */
    uint16_t classic_idle_sniff_slots;
    /**
     * Adaptive hopping policy (see docs/PERFORMANCE.md, "periodic gaps"):
     *  FLOSS_LINK_AFH_FREEZE (default): the connecting side (central; the
     *    dongle by default) lets its controller assess
     *    channels for classic_afh_learn_ms after connecting, then hands the
     *    learned map back as the host classification and turns assessment off;
     *    the waiting side never assesses. Bad channels stay avoided, no periodic scans.
     *  FLOSS_LINK_AFH_CONTROLLER: controller default (continuous assessment:
     *    ~25 ms report gaps every few seconds on the CYW43439).
     *  FLOSS_LINK_AFH_ALL_CHANNELS: no assessment at all, hop on all 79.
     */
    uint8_t classic_afh;
    /** FREEZE: learning window after each connection (ms). Default 4000. */
    uint32_t classic_afh_learn_ms;
    /** FREEZE: re-learn this often (s), 0 = only at connection. Default 0. */
    uint32_t classic_afh_refresh_s;

    /* --- LE ------------------------------------------------------------- */
    /** Connection interval min/max requested by the dongle (1.25 ms units). Default 6/6 (7.5 ms). */
    uint16_t le_interval_min;
    uint16_t le_interval_max;
    /** Peripheral latency: events an idle pad may skip. Default 0 (lowest latency); try 20-50 for power. */
    uint16_t le_peripheral_latency;
    /** Supervision timeout (10 ms units). Default 200 (2 s). */
    uint16_t le_supervision_timeout;
    /** Request the 2M PHY (shorter air time per packet). Default true. */
    bool le_2m_phy;

    /* --- both ----------------------------------------------------------- */
    /** Dongle: send a PING every N ms to measure round-trip latency (0 = off). Default 0. */
    uint32_t ping_period_ms;
    /** Reconnect to the bonded peer automatically at start and after a drop. Default true. */
    bool auto_reconnect;
} floss_link_config_t;

/** classic_afh values. */
#define FLOSS_LINK_AFH_FREEZE       0
#define FLOSS_LINK_AFH_CONTROLLER   1
#define FLOSS_LINK_AFH_ALL_CHANNELS 2

/** report_interval_us value for LE: one report per connection event. */
#define FLOSS_LINK_INTERVAL_CONN 1u

/** Per-received-input information (dongle). */
typedef struct {
    uint16_t seq;          /**< pad sequence number */
    uint16_t lost;         /**< sequence numbers skipped since the previous INPUT */
    uint32_t t_pad_us;     /**< pad clock when the report was handed to the radio */
    uint32_t t_rx_us;      /**< dongle clock when it arrived */
} floss_link_rx_info_t;

typedef struct {
    void (*on_connected)(const floss_addr_t *peer);
    void (*on_disconnected)(uint8_t reason);
    /** Dongle: an input report arrived. data valid during the call only. */
    void (*on_input)(uint8_t report_id, const uint8_t *data, uint8_t len, const floss_link_rx_info_t *info);
    /** Pad: an output report (rumble, LEDs...) arrived. */
    void (*on_output)(uint8_t report_id, const uint8_t *data, uint8_t len);
    /** Pad (paced mode): last chance to sample inputs before a report is sent. */
    void (*on_report_tick)(void);
} floss_link_callbacks_t;

/** Link statistics (dongle side is the interesting one). */
typedef struct {
    uint32_t inputs_sent;      /**< pad */
    uint32_t inputs_received;  /**< dongle */
    uint32_t inputs_lost;      /**< dongle: sequence gaps */
    uint32_t outputs_sent;
    uint32_t outputs_received;
    floss_hist_t rx_interval;  /**< dongle: time between consecutive INPUT arrivals */
    floss_hist_t rx_jitter;    /**< dongle: arrival - pad timestamp, minus the minimum seen (latency variation) */
    floss_hist_t rtt;          /**< dongle: PING -> PONG round trip */
} floss_link_stats_t;

/** Defaults for a role/transport. */
void floss_link_config_defaults(floss_link_config_t *cfg, floss_link_role_t role, floss_link_transport_t transport);

/** Initialise. Call after floss_init(), before floss_start(). cfg must stay valid. */
floss_err_t floss_link_init(const floss_link_config_t *cfg, const floss_link_callbacks_t *cb);

/**
 * Start pairing a new peer. Pad: become discoverable / advertise for
 * pairing. Dongle: search for a pad (Classic inquiry / LE scan), connect
 * and bond with the first one found (matching cfg->name).
 */
floss_err_t floss_link_pair(void);

/** Dongle, Classic, FREEZE policy: run a new channel-learning window now. */
floss_err_t floss_link_afh_relearn(void);

/** Forget the bonded peer and disconnect. */
floss_err_t floss_link_forget(void);

/**
 * Hold off connecting: no pages, inquiry or scans until released (a page or
 * inquiry in progress is cancelled). A link already up stays up. For an
 * application that wants the air for something else meanwhile, such as an LE
 * connection. Releasing goes back to the rest state (reconnect / search),
 * from a fresh back-off. Classic only.
 */
floss_err_t floss_link_hold(bool hold);

/** Pad: publish the newest input report (ISR / other-core safe). len <= FLOSS_LINK_MAX_REPORT. */
floss_err_t floss_link_set_input(uint8_t report_id, const uint8_t *data, uint8_t len);

/**
 * Pad, paced: move the report clock by delta_us (positive = later), from the next
 * report on; the period stays the same. Lets the dongle line the pad's reports up with
 * its own output clock (a USB host's polls), which the pad cannot see. A shift that
 * puts the next report in the past sends it at once. STATE when not paced or not
 * connected.
 */
floss_err_t floss_link_shift_phase(int32_t delta_us);

/**
 * Pad, paced: when the next report goes (floss_hal_time_us()), so the application
 * can build it just before instead of in on_report_tick, which delays the send by
 * however long building takes. False when not paced or not connected.
 */
bool floss_link_next_report_at(uint32_t *t_us);

/** Dongle: send an output report to the pad (copied; BUSY if no buffer/credit). */
floss_err_t floss_link_send_output(uint8_t report_id, const uint8_t *data, uint8_t len);

floss_link_state_t floss_link_state(void);
static inline bool floss_link_connected(void) {
    return floss_link_state() == FLOSS_LINK_STATE_CONNECTED;
}
const floss_link_stats_t *floss_link_stats(void);
void floss_link_stats_reset(void);
/** Print floss_link_stats() via the log. */
void floss_link_stats_dump(void);

/* Internal: called from floss_poll() (pad mailbox). */
void floss_link_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_LINK_H */
