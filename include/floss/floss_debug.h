/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_debug.h - The debugging toolbox.
 *
 * Four tools, from most to least expensive. Use the cheapest one that answers
 * your question; see docs/DEBUGGING.md for recipes.
 *
 *  1. LOGS   FLOSS_LOGI(HCI, "...")   printf-style text, per-module level.
 *            Costs a vsnprintf + HAL write. Compiled out above FLOSS_LOG_LEVEL.
 *
 *  2. SNOOP  floss_snoop_set_hook()   every HCI packet, timestamped. Feed to
 *            tools/floss_snoop.py to get a .btsnoop file for Wireshark.
 *
 *  3. STATS  floss_stats()            counters + latency histograms that answer
 *            "why is my throughput/latency inconsistent?" Always-on, ~O(1)
 *            per event. floss_stats_dump() prints them with hints.
 *
 *  4. TRACE  FLOSS_TRACE(ID, arg)     ~10 instructions: timestamp + id + arg
 *            into a RAM ring. Safe in the hottest paths. floss_trace_dump()
 *            decodes it after the fact.
 *
 *  5. PINS   FLOSS_DBG_PIN(n, 1/0)    GPIO toggles around hot paths for a
 *            scope / logic analyser (FLOSS_ENABLE_DEBUG_PINS=1).
 */
#ifndef FLOSS_DEBUG_H
#define FLOSS_DEBUG_H

#include "floss_types.h"
#include "floss_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* 1. Logging                                                               */
/* ------------------------------------------------------------------------ */

/** Modules (each has its own runtime level). */
typedef enum {
    FLOSS_MOD_CORE = 0,
    FLOSS_MOD_TRANSPORT,
    FLOSS_MOD_HCI,
    FLOSS_MOD_GAP,
    FLOSS_MOD_L2CAP,
    FLOSS_MOD_SDP,
    FLOSS_MOD_HID,
    FLOSS_MOD_APP,
    FLOSS_MOD_COUNT
} floss_mod_t;

/** Set a module's runtime level (cannot exceed compile-time FLOSS_LOG_LEVEL). */
void floss_log_set_level(floss_mod_t mod, uint8_t level);
/** Set all modules at once. */
void floss_log_set_level_all(uint8_t level);
uint8_t floss_log_get_level(floss_mod_t mod);

/** Backend; prefer the macros. Prefixes "[sssss.uuuuuu] MOD L " and appends '\n'. */
void floss_log(floss_mod_t mod, uint8_t level, const char *fmt, ...)
#if defined(__GNUC__) || defined(__clang__)
    __attribute__((format(printf, 3, 4)))
#endif
    ;

/** Hex dump at DEBUG level: "label: 01 02 03 ..." (truncated to one line). */
void floss_log_hex(floss_mod_t mod, uint8_t level, const char *label, const uint8_t *data, uint16_t len);

#define FLOSS__LOG(mod, lvl, ...)                                \
    do {                                                         \
        if ((lvl) <= FLOSS_LOG_LEVEL) {                          \
            floss_log(FLOSS_MOD_##mod, (lvl), __VA_ARGS__);      \
        }                                                        \
    } while (0)

#define FLOSS_LOGE(mod, ...) FLOSS__LOG(mod, FLOSS_LOG_ERROR, __VA_ARGS__)
#define FLOSS_LOGW(mod, ...) FLOSS__LOG(mod, FLOSS_LOG_WARN, __VA_ARGS__)
#define FLOSS_LOGI(mod, ...) FLOSS__LOG(mod, FLOSS_LOG_INFO, __VA_ARGS__)
#define FLOSS_LOGD(mod, ...) FLOSS__LOG(mod, FLOSS_LOG_DEBUG, __VA_ARGS__)
#define FLOSS_LOGT(mod, ...) FLOSS__LOG(mod, FLOSS_LOG_TRACE, __VA_ARGS__)

#define FLOSS_HEXDUMP(mod, label, data, len)                                   \
    do {                                                                       \
        if (FLOSS_LOG_DEBUG <= FLOSS_LOG_LEVEL) {                              \
            floss_log_hex(FLOSS_MOD_##mod, FLOSS_LOG_DEBUG, label, data, len); \
        }                                                                      \
    } while (0)

/* ------------------------------------------------------------------------ */
/* 2. HCI snoop                                                             */
/* ------------------------------------------------------------------------ */

/**
 * Called for every HCI packet crossing the transport boundary.
 * is_rx = true for controller->host. Runs in the hot path: keep it fast
 * (e.g. copy into a ring and write out later).
 */
typedef void (*floss_snoop_fn)(uint8_t type, bool is_rx, const uint8_t *pkt, uint16_t len, uint32_t t_us);

/** Install (or clear with NULL) the snoop hook. */
void floss_snoop_set_hook(floss_snoop_fn fn);

/**
 * Ready-made hook: prints each packet as one log line
 *   "SNOOP <t_us> <R|T> <type> <hex...>"
 * which tools/floss_snoop.py converts into a .btsnoop capture.
 */
void floss_snoop_log_hook(uint8_t type, bool is_rx, const uint8_t *pkt, uint16_t len, uint32_t t_us);

/* Internal: called by the HCI layer. */
void floss_snoop_packet(uint8_t type, bool is_rx, const uint8_t *pkt, uint16_t len);

/* ------------------------------------------------------------------------ */
/* 3. Statistics                                                            */
/* ------------------------------------------------------------------------ */

/**
 * Log2 histogram of microsecond durations.
 * bucket[0] = < 128us, bucket[i] = [64<<i, 128<<i) us, last = overflow (>= ~131 ms).
 */
#define FLOSS_HIST_BUCKETS 12
typedef struct {
    uint32_t count;
    uint32_t min_us;
    uint32_t max_us;
    uint64_t sum_us;
    uint32_t bucket[FLOSS_HIST_BUCKETS];
} floss_hist_t;

void floss_hist_add(floss_hist_t *h, uint32_t us);
/** Approximate percentile (0..100) from the buckets (upper bucket edge). */
uint32_t floss_hist_percentile(const floss_hist_t *h, uint8_t pct);

typedef struct {
    /* --- transport / HCI ------------------------------------------------ */
    uint32_t tx_cmd;            /**< HCI commands sent */
    uint32_t tx_acl;            /**< ACL packets sent */
    uint32_t rx_evt;            /**< HCI events received */
    uint32_t rx_acl;            /**< ACL packets received */
    uint32_t tx_bytes, rx_bytes;
    uint32_t transport_busy;    /**< driver said BUSY on send (back-pressure) */
    uint32_t transport_errors;
    uint32_t cmd_timeouts;      /**< controller never answered a command */
    uint32_t cmd_failures;      /**< command completed with non-zero status */
    uint32_t acl_credit_stalls; /**< wanted to send ACL, controller had no buffers */
    uint32_t buf_alloc_failures;/**< pool exhausted -> raise FLOSS_BUF_COUNT */
    uint32_t rx_dropped;        /**< malformed / no room for reassembly */
    floss_hist_t cmd_latency;   /**< command sent -> Complete/Status event */
    floss_hist_t acl_ack_latency; /**< ACL handed to transport -> Number Of
                                       Completed Packets. Reflects air time,
                                       retransmissions and sniff intervals. */

    /* --- link -------------------------------------------------------------- */
    uint32_t connections;
    uint32_t disconnections;
    uint32_t mode_changes;       /**< active <-> sniff transitions */
    uint8_t last_disconnect_reason;
    uint8_t current_mode;        /**< 0 active, 2 sniff */
    uint16_t sniff_interval;     /**< slots (0.625 ms), when in sniff */

    /* --- HID --------------------------------------------------------------- */
    uint32_t hid_reports_sent;
    uint32_t hid_reports_coalesced; /**< app updated a report that was replaced
                                         before it could be sent (normal when the
                                         app updates faster than the link) */
    uint32_t hid_send_deferred;  /**< pacing tick with no ACL credit: report
                                      waited. High = link saturated. */
    uint32_t hid_output_reports;
    floss_hist_t hid_report_age;     /**< app floss_hid_set_input() -> transport */
    floss_hist_t hid_send_interval;  /**< spacing between consecutive input reports */

    /* --- run loop ----------------------------------------------------------- */
    uint32_t polls;
    floss_hist_t poll_gap;     /**< time between floss_poll() calls. If p99 is
                                    large the application is starving the stack. */
    floss_hist_t timer_lateness; /**< how late timers fired vs. deadline */
} floss_stats_t;

/** Live statistics (zeroed at init). Read-only for the application. */
const floss_stats_t *floss_stats(void);
/** Mutable access for stack internals. */
floss_stats_t *floss_stats_mut(void);
void floss_stats_reset(void);
/** Print every counter and histogram with tuning hints. */
void floss_stats_dump(void);

#if FLOSS_ENABLE_STATS
#define FLOSS_STAT_INC(field) (floss_stats_mut()->field++)
#define FLOSS_STAT_ADD(field, n) (floss_stats_mut()->field += (n))
#define FLOSS_STAT_HIST(field, us) floss_hist_add(&floss_stats_mut()->field, (us))
#define FLOSS_STAT_SET(field, v) (floss_stats_mut()->field = (v))
#else
#define FLOSS_STAT_INC(field) ((void)0)
#define FLOSS_STAT_ADD(field, n) ((void)0)
#define FLOSS_STAT_HIST(field, us) ((void)0)
#define FLOSS_STAT_SET(field, v) ((void)0)
#endif

/* ------------------------------------------------------------------------ */
/* 4. Trace ring                                                            */
/* ------------------------------------------------------------------------ */

/** Trace event IDs. Add new ones at the end; floss_trace_dump() names them. */
typedef enum {
    FLOSS_TR_NONE = 0,
    FLOSS_TR_POLL,           /* arg: us since previous poll */
    FLOSS_TR_TX_CMD,         /* arg: opcode */
    FLOSS_TR_CMD_DONE,       /* arg: opcode */
    FLOSS_TR_RX_EVT,         /* arg: event code */
    FLOSS_TR_TX_ACL,         /* arg: (handle << 16) | len */
    FLOSS_TR_RX_ACL,         /* arg: (handle << 16) | len */
    FLOSS_TR_ACL_DONE,       /* arg: (handle << 16) | completed count */
    FLOSS_TR_CREDIT_STALL,   /* arg: handle */
    FLOSS_TR_CONN,           /* arg: handle */
    FLOSS_TR_DISCONN,        /* arg: (handle << 16) | reason */
    FLOSS_TR_MODE,           /* arg: (mode << 16) | interval */
    FLOSS_TR_L2CAP_SIG,      /* arg: (code << 16) | identifier */
    FLOSS_TR_HID_SET_INPUT,  /* arg: report id */
    FLOSS_TR_HID_SEND,       /* arg: report age us */
    FLOSS_TR_HID_DEFER,      /* arg: 0 */
    FLOSS_TR_HID_OUTPUT,     /* arg: report id */
    FLOSS_TR_TIMER,          /* arg: lateness us */
    FLOSS_TR_USER,           /* first id free for applications */
} floss_trace_id_t;

typedef struct {
    uint32_t t_us;
    uint32_t arg;
    uint16_t id;
} floss_trace_entry_t;

#if FLOSS_TRACE_ENTRIES > 0
void floss_trace(uint16_t id, uint32_t arg);
#define FLOSS_TRACE(id, arg) floss_trace((uint16_t)(id), (uint32_t)(arg))
#else
#define FLOSS_TRACE(id, arg) ((void)0)
#endif

/** Freeze/unfreeze recording (freeze when you hit the bug, then dump). */
void floss_trace_freeze(bool frozen);
/** Print the ring oldest-first with relative timestamps. */
void floss_trace_dump(void);
/** Raw access: copies up to max entries oldest-first, returns count. */
uint16_t floss_trace_read(floss_trace_entry_t *out, uint16_t max);

/* ------------------------------------------------------------------------ */
/* 5. Debug pins                                                            */
/* ------------------------------------------------------------------------ */

#define FLOSS_DBG_PIN_POLL     0 /* high while inside floss_poll() */
#define FLOSS_DBG_PIN_HID_TX   1 /* pulse when an input report is handed to HCI */
#define FLOSS_DBG_PIN_TRANS_TX 2 /* high during transport send() */
#define FLOSS_DBG_PIN_TRANS_RX 3 /* high while delivering a received packet */

#if FLOSS_ENABLE_DEBUG_PINS
#define FLOSS_DBG_PIN(n, level) floss_hal_debug_pin((n), (level))
#else
#define FLOSS_DBG_PIN(n, level) ((void)0)
#endif

/* ------------------------------------------------------------------------ */
/* Assertions                                                               */
/* ------------------------------------------------------------------------ */

/** Called on a failed FLOSS_ASSERT (logs and halts). Never returns. */
void floss_assert_fail(const char *file, int line, const char *expr);

#ifndef FLOSS_ASSERT
#define FLOSS_ASSERT(expr)                                   \
    do {                                                     \
        if (!(expr)) {                                       \
            floss_assert_fail(__FILE__, __LINE__, #expr);    \
        }                                                    \
    } while (0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_DEBUG_H */
