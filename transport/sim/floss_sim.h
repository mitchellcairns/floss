/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_sim.h - Virtual controller and scripted remote peer (tests, demos).
 *
 * ===========================================================================
 *  WHAT IT IS
 *
 *  floss_sim plays BOTH ends of the air interface that the stack cannot see:
 *
 *     +-----------+  floss_transport_t  +------------------+  "air"  +--------+
 *     |  FLOSS    | <-----------------> | virtual          | <-----> | remote |
 *     |  stack    |  HCI cmd/evt/ACL    | controller (HCI) |         | peer   |
 *     +-----------+                     +------------------+         +--------+
 *                                         answers commands,          L2CAP peer,
 *                                         pairing (LMP), credits     SDP client
 *
 *  - The virtual controller answers every HCI command the way a real one
 *    does (Command Complete / Command Status, Num_HCI_Command_Packets = 1),
 *    models paging, connection acceptance, Secure Simple Pairing (or legacy
 *    PIN pairing), encryption, sniff mode, role switch, ACL buffer credits
 *    (Number Of Completed Packets) and honours the host's event masks.
 *  - The scripted remote peer is a minimal host on the other side of the
 *    link: it connects/disconnects, pairs, opens L2CAP channels, answers
 *    L2CAP signaling and runs SDP queries - all driven by the test.
 *
 *  It is deterministic and single threaded. There is no real time: the sim
 *  reads floss_hal_time_us() (the test HAL's fake clock) for anything that
 *  takes "air time" (page timeout, delayed completions) and floss_sim_pump()
 *  advances that clock through the advance_time hook.
 *
 *  It also CHECKS the stack. Every protocol rule a real controller would
 *  punish silently (sending a command without a command credit, exceeding
 *  ACL buffers or the ACL MTU, wrong packet boundary flags, data larger than
 *  the peer's L2CAP MTU, events the host forgot to unmask) is counted in
 *  floss_sim_stats(); tests assert floss_sim_violations() == 0.
 *
 *  Reading the .c file top to bottom is a tour of how HCI, pairing, L2CAP
 *  signaling and SDP actually flow on the wire.
 * ===========================================================================
 *
 * Typical test:
 *
 *     floss_sim_config_t cfg;
 *     floss_sim_config_defaults(&cfg);
 *     cfg.advance_time = floss_test_time_advance;
 *     floss_transport_t *t = floss_sim_init(&cfg);
 *     floss_init(&(floss_init_t){ .transport = t });
 *     floss_start();
 *     floss_sim_pump(100);                         // bring-up runs
 *     uint16_t h = floss_sim_remote_connect(&host, 0x00010C);
 *     floss_sim_pump(10);
 *     floss_sim_remote_pair(h);
 *     floss_sim_chan_t *ctrl = floss_sim_l2cap_connect(h, FLOSS_PSM_HID_CONTROL);
 *     floss_sim_l2cap_send(ctrl, req, sizeof req);
 *     floss_sim_pump(10);
 *     n = floss_sim_l2cap_recv(ctrl, buf, sizeof buf);
 *
 * Only one simulator instance exists (it is a test fixture, not a driver
 * you instantiate per radio).
 */
#ifndef FLOSS_SIM_H
#define FLOSS_SIM_H

#include "floss/floss.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Capacities (override with -D...)                                         */
/* ------------------------------------------------------------------------ */

/** Simultaneous baseband/LE links the virtual controller tracks. */
#ifndef FLOSS_SIM_MAX_LINKS
#define FLOSS_SIM_MAX_LINKS 4
#endif
/** Remote devices the sim knows about (each remembers its link key). */
#ifndef FLOSS_SIM_MAX_PEERS
#define FLOSS_SIM_MAX_PEERS 4
#endif
/** L2CAP channels on the remote side (dynamic + LE fixed channels). */
#ifndef FLOSS_SIM_MAX_CHANS
#define FLOSS_SIM_MAX_CHANS 12
#endif
/** Received payloads kept per channel (oldest dropped when full). */
#ifndef FLOSS_SIM_INBOX_LEN
#define FLOSS_SIM_INBOX_LEN 8
#endif
/** Largest L2CAP payload / ACL payload the sim handles. */
#ifndef FLOSS_SIM_ACL_MAX
#define FLOSS_SIM_ACL_MAX 1024
#endif
/** Packets waiting to be delivered controller -> stack. */
#ifndef FLOSS_SIM_QUEUE_LEN
#define FLOSS_SIM_QUEUE_LEN 128
#endif
/** Commands remembered in the command log (ring). */
#ifndef FLOSS_SIM_CMD_LOG_LEN
#define FLOSS_SIM_CMD_LOG_LEN 256
#endif
/** Parameter bytes stored per logged command (longer ones are truncated; len keeps the real size). */
#ifndef FLOSS_SIM_CMD_PARAMS_MAX
#define FLOSS_SIM_CMD_PARAMS_MAX 64
#endif
/** Signaling commands from the stack remembered (ring). */
#ifndef FLOSS_SIM_SIG_LOG_LEN
#define FLOSS_SIM_SIG_LOG_LEN 32
#endif
#ifndef FLOSS_SIM_SIG_DATA_MAX
#define FLOSS_SIM_SIG_DATA_MAX 64
#endif

/** Returned instead of a connection handle on failure. */
#define FLOSS_SIM_NO_HANDLE 0xFFFF

/* ------------------------------------------------------------------------ */
/* Configuration                                                            */
/* ------------------------------------------------------------------------ */

typedef struct {
    /** Public device address reported by Read BD_ADDR (vendor 0xFC01 changes it). */
    floss_addr_t addr;

    /** Read Buffer Size: max ACL payload per packet and number of ACL buffers. */
    uint16_t acl_mtu;
    uint16_t acl_total;
    /** LE Read Buffer Size. le_acl_total = 0 reports "shared with BR/EDR". */
    uint16_t le_acl_mtu;
    uint8_t le_acl_total;

    /** Max ACL payload of controller->stack packets (fragments remote PDUs to exercise reassembly). */
    uint16_t acl_rx_frag;

    /**
     * Air latency: each ACL packet from the stack is reported complete this
     * many fake microseconds after it was received. Completions that are due
     * at the same sim poll are coalesced into one Number Of Completed Packets
     * event, like a real controller. 0 = complete at the next poll.
     */
    uint32_t nocp_delay_us;

    /** Drop events the host has not enabled with Set Event Mask (real controller behaviour). */
    bool enforce_event_mask;

    /** Seed of the sim's own PRNG (link keys, numeric comparison values). */
    uint32_t seed;

    /** L2CAP MTU the remote peer proposes in its Configuration Requests (and accepts). */
    uint16_t l2cap_mtu;

    /** MaximumAttributeByteCount used by floss_sim_sdp_ssa() (small values force continuation). */
    uint16_t sdp_max_attr_bytes;

    /** Fake time advanced per floss_sim_pump() iteration. */
    uint32_t pump_step_us;
    /** Max pump iterations the blocking helpers (l2cap_connect, expect_cmd, ...) wait. */
    uint32_t wait_polls;
    /** Hook that advances the fake clock (e.g. floss_test_time_advance). NULL = time stands still. */
    void (*advance_time)(uint32_t us);
} floss_sim_config_t;

/** Defaults: addr 02:F1:05:50:00:01, ACL 1021 x 8, LE 251 x 8, frag 1021, instant credits, MTU 672. */
void floss_sim_config_defaults(floss_sim_config_t *cfg);

/**
 * Reset the whole simulator (controller, peers, logs, queues) and return the
 * transport to pass to floss_init(). cfg is copied (NULL = defaults).
 */
floss_transport_t *floss_sim_init(const floss_sim_config_t *cfg);

/** Live configuration: tests may change fields at any time (e.g. nocp_delay_us). */
floss_sim_config_t *floss_sim_cfg(void);

/* ------------------------------------------------------------------------ */
/* Run loop helpers                                                         */
/* ------------------------------------------------------------------------ */

/** Deliver pending controller->stack packets now (what the transport poll does). Returns count. */
int floss_sim_run(void);

/** n times: floss_poll(), then advance fake time by pump_step_us. */
void floss_sim_pump(uint32_t n);

/** Pump until at least `us` of fake time has passed. */
void floss_sim_pump_us(uint32_t us);

/** Pump until cond(arg) is true (checked before each poll) or max_polls (0 = wait_polls) ran out. */
bool floss_sim_pump_until(bool (*cond)(void *arg), void *arg, uint32_t max_polls);

/* ------------------------------------------------------------------------ */
/* Statistics and protocol checks                                           */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint32_t cmds;              /**< HCI commands received */
    uint32_t events;            /**< HCI events delivered to the stack */
    uint32_t acl_in;            /**< ACL packets received from the stack */
    uint32_t acl_in_flushable;  /**< ...of which started an automatically flushable PDU */
    uint32_t acl_out;           /**< ACL packets delivered to the stack */
    uint32_t nocp_events;       /**< Number Of Completed Packets events generated */

    /* Stack misbehaviour - tests expect these to stay 0 (floss_sim_violations()). */
    uint32_t cmd_credit_violations; /**< command sent with no Num_HCI_Command_Packets credit */
    uint32_t acl_credit_violations; /**< ACL sent with all controller buffers in use */
    uint32_t acl_size_violations;   /**< ACL payload larger than Read Buffer Size allowed */
    uint32_t acl_bad_handle;        /**< ACL for a handle that is not open */
    uint32_t acl_pb_violations;     /**< bad packet boundary flag / broken fragment sequence */
    uint32_t l2cap_mtu_violations;  /**< SDU larger than the MTU the peer configured */
    uint32_t l2cap_unknown_cid;     /**< data for a CID that is not open on the peer */
    uint32_t malformed;             /**< inconsistent lengths in HCI / L2CAP packets */

    /* Informational. */
    uint32_t masked_events;     /**< events dropped because the host masked them */
    uint32_t queue_overflows;   /**< sim delivery queue full (raise FLOSS_SIM_QUEUE_LEN) */
    uint32_t inbox_overflows;   /**< channel inbox full: oldest payload dropped */
} floss_sim_stats_t;

const floss_sim_stats_t *floss_sim_stats(void);
/** Sum of all violation counters above. */
uint32_t floss_sim_violations(void);

/* ------------------------------------------------------------------------ */
/* Virtual controller state (what the host configured)                      */
/* ------------------------------------------------------------------------ */

const floss_addr_t *floss_sim_addr(void);
/** Last Write Scan Enable value (bit 0 inquiry scan = discoverable, bit 1 page scan = connectable). */
uint8_t floss_sim_scan_enable(void);
/** Last Write Class Of Device value. */
uint32_t floss_sim_class_of_device(void);
/** Last Write Local Name value (NUL terminated). */
const char *floss_sim_local_name(void);
/** Write Simple Pairing Mode / Write Secure Connections Host Support values. */
bool floss_sim_ssp_enabled(void);
bool floss_sim_sc_enabled(void);

/* ------------------------------------------------------------------------ */
/* Command log                                                              */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint32_t seq;       /**< position in the log since the last clear (0 = first) */
    uint32_t t_us;      /**< fake time when received */
    uint16_t opcode;
    uint8_t len;        /**< real parameter length */
    uint8_t params[FLOSS_SIM_CMD_PARAMS_MAX];
} floss_sim_cmd_t;

void floss_sim_log_clear(void);
/** Commands logged since the last clear (may exceed the ring size). */
uint32_t floss_sim_cmd_total(void);
/** i-th oldest command still in the ring (NULL if out of range). */
const floss_sim_cmd_t *floss_sim_cmd_at(uint32_t i);
/** Occurrences of opcode still in the ring. */
uint32_t floss_sim_cmd_count(uint16_t opcode);
/** Oldest / most recent occurrence (NULL if none). */
const floss_sim_cmd_t *floss_sim_cmd_first(uint16_t opcode);
const floss_sim_cmd_t *floss_sim_cmd_last(uint16_t opcode);
/** Pump until the stack has sent opcode (since the last clear). Returns the last occurrence or NULL. */
const floss_sim_cmd_t *floss_sim_expect_cmd(uint16_t opcode);

/* ------------------------------------------------------------------------ */
/* Remote peers                                                             */
/* ------------------------------------------------------------------------ */

typedef struct {
    bool in_use;
    floss_addr_t addr;
    uint32_t cod;            /**< its class of device (sent in Connection Request) */
    bool present;            /**< in range: answers pages */
    bool ssp;                /**< supports Secure Simple Pairing (false = legacy PIN) */
    bool sc;                 /**< Secure Connections host support (P-256 keys) */
    uint8_t io_cap;          /**< FLOSS_IO_CAP_* (default NO_INPUT_NO_OUTPUT: "just works") */
    bool auto_encrypt;       /**< after it authenticates a link it initiated security on, it enables encryption */
    char pin[17];            /**< legacy PIN it enters (default "0000") */
    char name[32];           /**< answered to Remote Name Request */
    bool has_key;            /**< it holds a link key for us */
    floss_link_key_t key;
} floss_sim_peer_t;

/** Add (or return existing) peer. New peers: present, SSP+SC, NoInputNoOutput, auto_encrypt. */
floss_sim_peer_t *floss_sim_peer_add(const floss_addr_t *addr, uint32_t cod);
floss_sim_peer_t *floss_sim_peer_find(const floss_addr_t *addr);

/* ------------------------------------------------------------------------ */
/* Links                                                                    */
/* ------------------------------------------------------------------------ */

typedef enum {
    FLOSS_SIM_LINK_FREE = 0,
    FLOSS_SIM_LINK_PAGING,     /**< stack sent Create Connection, page in progress */
    FLOSS_SIM_LINK_REQUESTED,  /**< Connection Request sent to the stack, waiting for Accept */
    FLOSS_SIM_LINK_OPEN,
} floss_sim_link_state_t;

typedef struct {
    floss_sim_link_state_t state;
    uint16_t handle;
    floss_addr_t addr;       /**< the remote peer */
    bool le;
    uint8_t role;            /**< the STACK's role: 0 central, 1 peripheral */
    bool stack_initiated;    /**< the stack paged / LE-connected */
    bool authenticated;
    uint8_t encrypted;       /**< 0 off, 1 E0 / AES-CCM (LE), 2 AES-CCM (BR/EDR Secure Connections) */
    uint8_t key_type;        /**< key type of the last pairing / authentication */
    uint8_t mode;            /**< FLOSS_LINK_MODE_* */
    uint16_t interval;       /**< sniff interval (slots) */
    uint16_t link_policy;    /**< Write Link Policy Settings (starts at the default policy) */
    uint16_t flush_timeout;  /**< Write Automatic Flush Timeout value (slots of 0.625 ms) */
} floss_sim_link_t;

/** Look up a link (NULL if free / unknown). */
const floss_sim_link_t *floss_sim_link(uint16_t handle);
const floss_sim_link_t *floss_sim_link_by_addr(const floss_addr_t *addr);

/**
 * The remote device pages us: sends Connection Request to the stack (only if
 * the host enabled page scan). The link opens once the stack accepts.
 * @return the handle the link will get, or FLOSS_SIM_NO_HANDLE.
 */
uint16_t floss_sim_remote_connect(const floss_addr_t *addr, uint32_t cod);

/** The remote terminates the link (e.g. FLOSS_HCI_ERR_REMOTE_USER_TERMINATED). */
floss_err_t floss_sim_remote_disconnect(uint16_t handle, uint8_t reason);

/**
 * The remote authenticates the link, as a PC does right after connecting:
 * with a stored key it asks the stack's controller to authenticate (Link Key
 * Request), otherwise it starts Secure Simple Pairing ("just works" with
 * NoInputNoOutput). On success it turns encryption on (auto_encrypt).
 */
floss_err_t floss_sim_remote_pair(uint16_t handle);

/** Make an outgoing page from the stack succeed (true) or end in Page Timeout (false). */
void floss_sim_peer_set_present(const floss_addr_t *addr, bool present);

/**
 * An LE connection completes (LE Connection Complete, subevent 0x01).
 * role = the stack's role (0 central, 1 peripheral). Returns the handle.
 */
uint16_t floss_sim_le_connect(const floss_addr_t *addr, uint8_t addr_type, uint8_t role);

/* ------------------------------------------------------------------------ */
/* Remote L2CAP peer                                                        */
/* ------------------------------------------------------------------------ */

typedef enum {
    FLOSS_SIM_CHAN_FREE = 0,
    FLOSS_SIM_CHAN_WAIT_CONNECT,  /**< sim sent Connection Request */
    FLOSS_SIM_CHAN_CONFIG,        /**< configuration in progress */
    FLOSS_SIM_CHAN_OPEN,
    FLOSS_SIM_CHAN_WAIT_DISCONNECT,
    FLOSS_SIM_CHAN_CLOSED,        /**< refused or disconnected (inbox still readable) */
} floss_sim_chan_state_t;

/** A channel as seen by the remote peer. Read-only for tests. */
typedef struct {
    floss_sim_chan_state_t state;
    uint16_t handle;
    uint16_t psm;               /**< 0 for fixed channels */
    uint16_t sim_cid;           /**< CID on the remote (sim) side = the stack's remote_cid */
    uint16_t stack_cid;         /**< CID on the stack side = the stack's local_cid */
    uint16_t stack_mtu;         /**< MTU the stack announced (sim never sends more) */
    uint16_t sim_mtu;           /**< MTU the sim configured (stack must not send more) */
    uint16_t stack_flush_timeout; /**< flush timeout option the stack sent (0xFFFF if none) */
    bool sim_initiated;         /**< the remote opened it */
    uint32_t seq;               /**< creation order (compare to check e.g. control before interrupt) */
    uint16_t result;            /**< Connection Response result (0 ok, 2 PSM not supported, ...) */
    uint16_t cfg_reject_result; /**< last non-success Configuration Response result from the stack */
    uint16_t cfg_suggested_mtu; /**< MTU the stack suggested when it rejected our configuration */
    uint8_t cfg_rejects;        /**< how many of our Configuration Requests the stack refused */
    uint32_t rx_count;          /**< payloads received from the stack */
    uint32_t rx_dropped;        /**< payloads dropped because the inbox was full */
} floss_sim_chan_t;

/**
 * Open a channel from the remote side: Connection Request, then both
 * configuration directions (sim proposes cfg.l2cap_mtu). Pumps the stack
 * until the channel is OPEN or refused. Always returns the channel record
 * (check ->state and ->result) unless the link is unknown / no slot (NULL).
 */
floss_sim_chan_t *floss_sim_l2cap_connect(uint16_t handle, uint16_t psm);

/** Send one SDU to the stack on an open channel (fragmented per acl_rx_frag). */
floss_err_t floss_sim_l2cap_send(floss_sim_chan_t *ch, const uint8_t *data, uint16_t len);

/**
 * Pop the oldest payload the stack sent on this channel. Copies up to cap
 * bytes and returns the payload length, or -1 when the inbox is empty.
 */
int floss_sim_l2cap_recv(floss_sim_chan_t *ch, uint8_t *out, uint16_t cap);
/** Payloads waiting in the inbox. */
uint8_t floss_sim_l2cap_pending(const floss_sim_chan_t *ch);
/** Pump until a payload arrives, then pop it (-1 on timeout). */
int floss_sim_l2cap_recv_wait(floss_sim_chan_t *ch, uint8_t *out, uint16_t cap);

/** Disconnect from the remote side and pump until the stack answers. */
floss_err_t floss_sim_l2cap_disconnect(floss_sim_chan_t *ch);

/** Find the newest channel on a link by PSM (any state except FREE), NULL if none. */
floss_sim_chan_t *floss_sim_chan_find(uint16_t handle, uint16_t psm);

/** Send a raw signaling command on CID 0x0001. Returns the identifier used. */
uint8_t floss_sim_l2cap_sig_send(uint16_t handle, uint8_t code, const uint8_t *data, uint16_t len);

/** A signaling command the stack sent. */
typedef struct {
    uint32_t seq;
    uint16_t handle;
    uint8_t code;
    uint8_t id;
    uint16_t len;               /**< real data length */
    uint8_t data[FLOSS_SIM_SIG_DATA_MAX];
} floss_sim_sig_t;

/** Most recent signaling command with this code from the stack (NULL if none). */
const floss_sim_sig_t *floss_sim_sig_last(uint8_t code);
/** Most recent signaling command with this code AND identifier. */
const floss_sim_sig_t *floss_sim_sig_find(uint8_t code, uint8_t id);
void floss_sim_sig_clear(void);

/** Send a raw L2CAP PDU (basic header included) to the stack. */
floss_err_t floss_sim_acl_send_pdu(uint16_t handle, const uint8_t *pdu, uint16_t len);

/* ------------------------------------------------------------------------ */
/* Fixed channels (LE ATT / signaling / SMP) - raw bytes                    */
/* ------------------------------------------------------------------------ */

#define FLOSS_SIM_CID_ATT    0x0004
#define FLOSS_SIM_CID_LE_SIG 0x0005
#define FLOSS_SIM_CID_SMP    0x0006

floss_err_t floss_sim_fixed_send(uint16_t handle, uint16_t cid, const uint8_t *data, uint16_t len);
/** Same contract as floss_sim_l2cap_recv(). */
int floss_sim_fixed_recv(uint16_t handle, uint16_t cid, uint8_t *out, uint16_t cap);

static inline floss_err_t floss_sim_att_send(uint16_t handle, const uint8_t *data, uint16_t len) {
    return floss_sim_fixed_send(handle, FLOSS_SIM_CID_ATT, data, len);
}
static inline int floss_sim_att_recv(uint16_t handle, uint8_t *out, uint16_t cap) {
    return floss_sim_fixed_recv(handle, FLOSS_SIM_CID_ATT, out, cap);
}

/* ------------------------------------------------------------------------ */
/* SDP client                                                               */
/* ------------------------------------------------------------------------ */

/**
 * ServiceSearchAttributeRequest for uuid16 over a fresh PSM 0x0001 channel,
 * all attributes (0x0000-0xFFFF), following continuation until done.
 * Writes the concatenated AttributeLists bytes (a sequence of attribute-list
 * sequences) to out. Returns the length, or a negative floss_err_t.
 */
int floss_sim_sdp_ssa(uint16_t handle, uint16_t uuid16, uint8_t *out, uint16_t cap);

/* ------------------------------------------------------------------------ */
/* Raw injection                                                            */
/* ------------------------------------------------------------------------ */

/** Queue an arbitrary HCI event (subject to the event mask). */
floss_err_t floss_sim_inject_event(uint8_t code, const uint8_t *params, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_SIM_H */
