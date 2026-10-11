/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_debug.c - Logging, HCI snoop, statistics, trace ring and assertions.
 *
 * Overview
 * --------
 * This is the implementation of the toolbox described in floss_debug.h.
 * Everything is static storage; nothing allocates, nothing blocks (beyond
 * whatever the HAL log sink does).
 *
 * LOGS   One line = one floss_hal_log_write() call, formatted on the stack:
 *            "[sssss.uuuuuu] MOD  L message\n"
 *        Runtime levels are stored as a *reduction* from the compile-time
 *        ceiling FLOSS_LOG_LEVEL, so the zero-initialised default is "log
 *        everything that was compiled in" and the application may tune
 *        levels before floss_init() (floss_debug_init() leaves them alone).
 *
 * SNOOP  A single hook pointer. floss_snoop_log_hook() renders a packet as
 *            "SNOOP <t_us> <R|T> <type hex> <hex bytes>\n"
 *        written in fixed-size chunks so arbitrarily long ACL packets are
 *        never truncated; tools/floss_snoop.py turns these into .btsnoop.
 *
 * STATS  One static floss_stats_t. Histograms are log2-bucketed so adding a
 *        sample is a count-leading-zeros and a few adds. floss_stats_dump()
 *        prints everything and derives tuning hints from the numbers.
 *
 * TRACE  A power-of-two ring of {t_us, arg, id}. Only the index increment is
 *        inside a critical section, so recording costs a timestamp read, an
 *        IRQ-off window of a few instructions and three stores.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "floss/floss_debug.h"
#include "floss/floss_buf.h"

/* ======================================================================== */
/* Output helpers                                                           */
/* ======================================================================== */

/** Module tags, padded to 4 columns by the formatter. Order = floss_mod_t. */
static const char *const s_mod_names[] = {
    "CORE", "TRNS", "HCI", "GAP", "L2C", "SDP", "HID", "APP", "LE", "SMP", "RFC", "LNK",
};
_Static_assert(FLOSS_ARRAY_SIZE(s_mod_names) == FLOSS_MOD_COUNT, "s_mod_names out of sync with floss_mod_t");

static const char s_level_chars[] = "-EWIDT"; /* index = FLOSS_LOG_* */

/**
 * Close a line: make sure it ends in exactly one '\n' and hand it to the HAL.
 * `len` excludes any terminator; buf must have room for len + 1 bytes.
 */
static void emit_line(char *buf, size_t len)
{
    while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) {
        len--;
    }
    buf[len++] = '\n';
    floss_hal_log_write(buf, len);
}

/**
 * vsnprintf into buf[pos..cap) and return the new length, clamped so that a
 * '\n' still fits (cap - 1 at most). Truncated lines end in "..." so a cut
 * message is never mistaken for a complete one.
 */
static size_t append_v(char *buf, size_t pos, size_t cap, const char *fmt, va_list ap)
{
    size_t room = cap - 1 - pos; /* keep one byte for '\n' */
    int n;

    if (pos >= cap - 1) {
        return pos;
    }
    n = vsnprintf(buf + pos, room, fmt, ap);
    if (n < 0) {
        return pos;
    }
    if ((size_t)n >= room) {
        size_t end = cap - 2; /* vsnprintf left a NUL at cap-2 */
        if (end >= pos + 3) {
            buf[end - 3] = '.';
            buf[end - 2] = '.';
            buf[end - 1] = '.';
        }
        return end;
    }
    return pos + (size_t)n;
}

static size_t append(char *buf, size_t pos, size_t cap, const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    pos = append_v(buf, pos, cap, fmt, ap);
    va_end(ap);
    return pos;
}

/** printf-style line without a log prefix (tables, dumps). Always printed. */
static void out(const char *fmt, ...)
#if defined(__GNUC__) || defined(__clang__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;

static void out(const char *fmt, ...)
{
    char line[FLOSS_LOG_LINE_MAX];
    va_list ap;
    size_t len;

    va_start(ap, fmt);
    len = append_v(line, 0, sizeof(line), fmt, ap);
    va_end(ap);
    emit_line(line, len);
}

/* ======================================================================== */
/* 1. Logging                                                               */
/* ======================================================================== */

/* Effective level = FLOSS_LOG_LEVEL - s_level_reduce[mod] (zero-init = max). */
static uint8_t s_level_reduce[FLOSS_MOD_COUNT];

void floss_log_set_level(floss_mod_t mod, uint8_t level)
{
    if ((unsigned)mod >= FLOSS_MOD_COUNT) {
        return;
    }
    if (level > FLOSS_LOG_LEVEL) {
        level = FLOSS_LOG_LEVEL;
    }
    s_level_reduce[mod] = (uint8_t)(FLOSS_LOG_LEVEL - level);
}

void floss_log_set_level_all(uint8_t level)
{
    for (int m = 0; m < FLOSS_MOD_COUNT; m++) {
        floss_log_set_level((floss_mod_t)m, level);
    }
}

uint8_t floss_log_get_level(floss_mod_t mod)
{
    if ((unsigned)mod >= FLOSS_MOD_COUNT) {
        return FLOSS_LOG_NONE;
    }
    return (uint8_t)(FLOSS_LOG_LEVEL - s_level_reduce[mod]);
}

static bool log_enabled(floss_mod_t mod, uint8_t level)
{
    return level != FLOSS_LOG_NONE && level <= floss_log_get_level(mod);
}

/** Write "[sssss.uuuuuu] MOD  L " and return its length. */
static size_t log_prefix(char *buf, size_t cap, floss_mod_t mod, uint8_t level)
{
    uint32_t t = floss_hal_time_us();

    return append(buf, 0, cap, "[%5lu.%06lu] %-4s %c ",
                  (unsigned long)(t / 1000000u), (unsigned long)(t % 1000000u),
                  s_mod_names[mod], s_level_chars[level <= FLOSS_LOG_TRACE ? level : 0]);
}

void floss_log(floss_mod_t mod, uint8_t level, const char *fmt, ...)
{
    char line[FLOSS_LOG_LINE_MAX];
    size_t len;
    va_list ap;

    if (!log_enabled(mod, level)) {
        return;
    }
    len = log_prefix(line, sizeof(line), mod, level);
    va_start(ap, fmt);
    len = append_v(line, len, sizeof(line), fmt, ap);
    va_end(ap);
    emit_line(line, len);
}

void floss_log_hex(floss_mod_t mod, uint8_t level, const char *label, const uint8_t *data, uint16_t len)
{
    static const char hex[] = "0123456789ABCDEF";
    char line[FLOSS_LOG_LINE_MAX];
    size_t pos;
    const size_t limit = sizeof(line) - 1; /* room for '\n' */

    if (!log_enabled(mod, level)) {
        return;
    }
    pos = log_prefix(line, sizeof(line), mod, level);
    pos = append(line, pos, sizeof(line), "%s:", label ? label : "");

    for (uint16_t i = 0; i < len; i++) {
        /* Each byte needs 3 chars; while more follow, keep room for the
         * " ...(+65535)" marker that says how much was cut. */
        if (pos + 3 + ((i + 1u < len) ? 12u : 0u) > limit) {
            pos = append(line, pos, sizeof(line), " ...(+%u)", (unsigned)(len - i));
            break;
        }
        line[pos++] = ' ';
        line[pos++] = hex[data[i] >> 4];
        line[pos++] = hex[data[i] & 0x0F];
    }
    emit_line(line, pos);
}

/* ======================================================================== */
/* 2. HCI snoop                                                             */
/* ======================================================================== */

#if FLOSS_ENABLE_SNOOP
static floss_snoop_fn s_snoop_hook;
#endif

void floss_snoop_set_hook(floss_snoop_fn fn)
{
#if FLOSS_ENABLE_SNOOP
    s_snoop_hook = fn;
#else
    FLOSS_UNUSED(fn);
#endif
}

void floss_snoop_packet(uint8_t type, bool is_rx, const uint8_t *pkt, uint16_t len)
{
#if FLOSS_ENABLE_SNOOP
    floss_snoop_fn fn = s_snoop_hook;

    if (fn != NULL) {
        fn(type, is_rx, pkt, len, floss_hal_time_us());
    }
#else
    FLOSS_UNUSED(type);
    FLOSS_UNUSED(is_rx);
    FLOSS_UNUSED(pkt);
    FLOSS_UNUSED(len);
#endif
}

/** Size of each write; small enough for the stack, big enough to be efficient. */
#define FLOSS_SNOOP_CHUNK 128

void floss_snoop_log_hook(uint8_t type, bool is_rx, const uint8_t *pkt, uint16_t len, uint32_t t_us)
{
    static const char hex[] = "0123456789ABCDEF";
    char chunk[FLOSS_SNOOP_CHUNK];
    int n;
    size_t pos;

    n = snprintf(chunk, sizeof(chunk), "SNOOP %lu %c %02X ", (unsigned long)t_us, is_rx ? 'R' : 'T',
                 (unsigned)type);
    pos = (n > 0) ? (size_t)n : 0;

    /* The line may span several HAL writes; it only ends at the final '\n'. */
    for (uint16_t i = 0; i < len; i++) {
        if (pos + 2 > sizeof(chunk)) {
            floss_hal_log_write(chunk, pos);
            pos = 0;
        }
        chunk[pos++] = hex[pkt[i] >> 4];
        chunk[pos++] = hex[pkt[i] & 0x0F];
    }
    if (pos + 1 > sizeof(chunk)) {
        floss_hal_log_write(chunk, pos);
        pos = 0;
    }
    chunk[pos++] = '\n';
    floss_hal_log_write(chunk, pos);
}

/* ======================================================================== */
/* 3. Statistics                                                            */
/* ======================================================================== */

static floss_stats_t s_stats;

/** floor(log2(v)) for v > 0. */
static unsigned log2_floor(uint32_t v)
{
#if defined(__GNUC__) || defined(__clang__)
    return 31u - (unsigned)__builtin_clz(v);
#else
    unsigned r = 0;
    while (v >>= 1) {
        r++;
    }
    return r;
#endif
}

/*
 * Bucket layout (floss_debug.h): [0] < 8 us, [i] = [4<<i, 8<<i) us,
 * [15] >= 131072 us. For us >= 8 the bucket is log2(us) - 2.
 */
void floss_hist_add(floss_hist_t *h, uint32_t us)
{
    unsigned b = 0;

    if (us >= 8u) {
        b = log2_floor(us) - 2u;
        if (b >= FLOSS_HIST_BUCKETS) {
            b = FLOSS_HIST_BUCKETS - 1;
        }
    }
    h->bucket[b]++;
    if (h->count == 0 || us < h->min_us) {
        h->min_us = us;
    }
    if (us > h->max_us) {
        h->max_us = us;
    }
    h->count++;
    h->sum_us += us;
}

/*
 * Walk buckets until the cumulative count reaches the requested rank and
 * report that bucket's upper edge. The edge is clamped into [min, max] seen,
 * which removes the worst of the log2 rounding at the extremes (p100 is the
 * real maximum, a histogram of identical samples reports the sample itself).
 */
uint32_t floss_hist_percentile(const floss_hist_t *h, uint8_t pct)
{
    uint64_t rank;
    uint64_t cum = 0;

    if (h->count == 0) {
        return 0;
    }
    if (pct > 100) {
        pct = 100;
    }
    rank = ((uint64_t)h->count * pct + 99u) / 100u; /* ceil */
    if (rank == 0) {
        return h->min_us;
    }
    for (unsigned i = 0; i < FLOSS_HIST_BUCKETS; i++) {
        cum += h->bucket[i];
        if (cum >= rank) {
            uint32_t edge = (i == FLOSS_HIST_BUCKETS - 1) ? h->max_us : (8u << i);
            if (edge > h->max_us) {
                edge = h->max_us;
            }
            if (edge < h->min_us) {
                edge = h->min_us;
            }
            return edge;
        }
    }
    return h->max_us;
}

const floss_stats_t *floss_stats(void)
{
    return &s_stats;
}

floss_stats_t *floss_stats_mut(void)
{
    return &s_stats;
}

void floss_stats_reset(void)
{
    /* Link state fields describe "now", not a count: keep them. */
    uint8_t mode = s_stats.current_mode;
    uint16_t sniff = s_stats.sniff_interval;
    uint8_t reason = s_stats.last_disconnect_reason;

    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.current_mode = mode;
    s_stats.sniff_interval = sniff;
    s_stats.last_disconnect_reason = reason;
}

static void dump_counter(const char *name, uint32_t v)
{
    out("  %-24s %10lu", name, (unsigned long)v);
}

static void dump_hist(const char *name, const floss_hist_t *h)
{
    if (h->count == 0) {
        out("  %-20s %8s", name, "-");
        return;
    }
    out("  %-20s %8lu %8lu %8lu %8lu %8lu %8lu", name, (unsigned long)h->count, (unsigned long)h->min_us,
        (unsigned long)(h->sum_us / h->count), (unsigned long)floss_hist_percentile(h, 50),
        (unsigned long)floss_hist_percentile(h, 99), (unsigned long)h->max_us);
}

/** Common disconnect reasons worth explaining in plain words. */
static const char *disconnect_reason_str(uint8_t reason)
{
    switch (reason) {
    case 0x05: return "authentication failure";
    case 0x06: return "PIN or key missing (peer lost its bond?)";
    case 0x08: return "supervision timeout (range / interference / peer stopped)";
    case 0x13: return "remote user terminated";
    case 0x14: return "remote low resources";
    case 0x15: return "remote power off";
    case 0x16: return "terminated by local host";
    case 0x22: return "LMP/LL response timeout";
    case 0x3D: return "MIC failure";
    case 0x3E: return "failed to establish";
    default:   return "see HCI error codes";
    }
}

/**
 * Turn the numbers into advice. Each rule encodes a piece of field
 * experience; thresholds are deliberately conservative so a hint means
 * something is actually off.
 */
static void dump_hints(const floss_stats_t *s)
{
    unsigned hints = 0;
    uint16_t low = floss_buf_low_watermark();

    out("-- hints --");
    if (s->buf_alloc_failures > 0) {
        out("  buf_alloc_failures=%lu -> pool exhausted, raise FLOSS_BUF_COUNT", (unsigned long)s->buf_alloc_failures);
        hints++;
    } else if (low == 0) {
        out("  buffer pool reached 0 free (no failure yet) -> consider raising FLOSS_BUF_COUNT");
        hints++;
    }
    if (s->cmd_timeouts > 0) {
        out("  cmd_timeouts=%lu -> transport problem (wiring, baud rate, RTS/CTS, controller reset/crash)",
            (unsigned long)s->cmd_timeouts);
        hints++;
    }
    if (s->cmd_failures > 0) {
        out("  cmd_failures=%lu -> controller rejected commands; enable HCI debug logs or snoop to see which",
            (unsigned long)s->cmd_failures);
        hints++;
    }
    if (s->transport_errors > 0) {
        out("  transport_errors=%lu -> driver I/O errors; check the physical link", (unsigned long)s->transport_errors);
        hints++;
    }
    if (s->transport_busy > 0 && s->transport_busy * 20u > s->tx_cmd + s->tx_acl) {
        out("  transport_busy high -> host->controller pipe is the bottleneck (raise baud / enable flow control)");
        hints++;
    }
    if (s->rx_dropped > 0) {
        out("  rx_dropped=%lu -> malformed input or reassembly overflow (UART errors? FLOSS_BUF_SIZE vs MTU?)",
            (unsigned long)s->rx_dropped);
        hints++;
    }
    if (s->acl_credit_stalls > 0 && s->acl_credit_stalls * 10u > s->tx_acl) {
        out("  acl_credit_stalls high -> controller ACL buffers often full: the air link is slower than the offered load");
        hints++;
    }
    if (s->acl_ack_latency.count > 0) {
        uint32_t p99 = floss_hist_percentile(&s->acl_ack_latency, 99);
        if (s->current_mode == 2 && s->sniff_interval > 0) {
            uint32_t sniff_us = (uint32_t)s->sniff_interval * 625u;
            if (p99 > 2u * sniff_us) {
                out("  acl_ack_latency p99 %luus > 2x sniff interval (%luus) -> retransmissions/interference",
                    (unsigned long)p99, (unsigned long)sniff_us);
                hints++;
            }
        } else if (p99 > 20000u) {
            out("  acl_ack_latency p99 %luus in active mode -> retransmissions/interference or peer busy (scanning?)",
                (unsigned long)p99);
            hints++;
        }
    }
    if (s->transport_send_time.count > 0 && s->transport_send_time.max_us > 500u) {
        out("  transport_send_time max %luus -> the bus blocks the loop per packet; check bus clock / driver waits",
            (unsigned long)s->transport_send_time.max_us);
        hints++;
    }
    if (s->transport_poll_time.count > 0 && s->transport_poll_time.max_us > 500u) {
        out("  transport_poll_time max %luus -> reading the controller is slow; consider capping packets per poll",
            (unsigned long)s->transport_poll_time.max_us);
        hints++;
    }
    if (s->hid_send_deferred > 0 && s->hid_send_deferred * 10u > s->hid_reports_sent) {
        out("  hid_send_deferred high (%lu vs %lu sent) -> link saturated, lower report rate or set flush timeout",
            (unsigned long)s->hid_send_deferred, (unsigned long)s->hid_reports_sent);
        hints++;
    }
    if (s->hid_report_age.count > 0 && s->hid_send_interval.count > 0) {
        uint32_t age99 = floss_hist_percentile(&s->hid_report_age, 99);
        uint32_t ival50 = floss_hist_percentile(&s->hid_send_interval, 50);
        if (age99 > 2u * ival50 && age99 > 2000u) {
            out("  hid_report_age p99 %luus > 2x send interval -> reports queue up; use a flush timeout / sniff off",
                (unsigned long)age99);
            hints++;
        }
    }
    if (s->hid_reports_coalesced > s->hid_reports_sent && s->hid_reports_sent > 0) {
        out("  hid_reports_coalesced > sent -> app updates faster than the link sends (harmless, latest wins)");
        hints++;
    }
    if (s->poll_gap.count > 0 && floss_hist_percentile(&s->poll_gap, 99) > 1000u) {
        out("  poll_gap p99 > 1000us -> application starves the stack; call floss_poll() more often");
        hints++;
    }
    if (s->timer_lateness.count > 0 && floss_hist_percentile(&s->timer_lateness, 99) > 500u) {
        out("  timer_lateness p99 > 500us -> sleep no longer than floss_poll()'s return value");
        hints++;
    }
    if (s->disconnections > 0) {
        out("  last disconnect reason 0x%02X: %s", s->last_disconnect_reason,
            disconnect_reason_str(s->last_disconnect_reason));
        hints++;
    }
    if (hints == 0) {
        out("  none - all numbers look healthy");
    }
}

void floss_stats_dump(void)
{
    const floss_stats_t *s = &s_stats;

    out("==== FLOSS stats ====");
    out("-- transport / HCI --");
    dump_counter("tx_cmd", s->tx_cmd);
    dump_counter("tx_acl", s->tx_acl);
    dump_counter("rx_evt", s->rx_evt);
    dump_counter("rx_acl", s->rx_acl);
    dump_counter("tx_bytes", s->tx_bytes);
    dump_counter("rx_bytes", s->rx_bytes);
    dump_counter("transport_busy", s->transport_busy);
    dump_counter("transport_errors", s->transport_errors);
    dump_counter("cmd_timeouts", s->cmd_timeouts);
    dump_counter("cmd_failures", s->cmd_failures);
    dump_counter("acl_credit_stalls", s->acl_credit_stalls);
    dump_counter("buf_alloc_failures", s->buf_alloc_failures);
    dump_counter("rx_dropped", s->rx_dropped);
    out("  %-24s %10u / %u (low watermark %u)", "buf_free", (unsigned)floss_buf_free_count(),
        (unsigned)FLOSS_BUF_COUNT, (unsigned)floss_buf_low_watermark());
    out("-- link --");
    dump_counter("connections", s->connections);
    dump_counter("disconnections", s->disconnections);
    dump_counter("mode_changes", s->mode_changes);
    out("  %-24s       0x%02X", "last_disconnect_reason", s->last_disconnect_reason);
    out("  %-24s %10s", "current_mode", s->current_mode == 2 ? "sniff" : (s->current_mode == 0 ? "active" : "other"));
    out("  %-24s %10u slots (%lu us)", "sniff_interval", (unsigned)s->sniff_interval,
        (unsigned long)s->sniff_interval * 625ul);
    out("-- HID --");
    dump_counter("hid_reports_sent", s->hid_reports_sent);
    dump_counter("hid_reports_coalesced", s->hid_reports_coalesced);
    dump_counter("hid_send_deferred", s->hid_send_deferred);
    dump_counter("hid_output_reports", s->hid_output_reports);
    out("-- run loop --");
    dump_counter("polls", s->polls);
    out("-- latency histograms (us) --");
    out("  %-20s %8s %8s %8s %8s %8s %8s", "", "count", "min", "avg", "p50", "p99", "max");
    dump_hist("cmd_latency", &s->cmd_latency);
    dump_hist("acl_ack_latency", &s->acl_ack_latency);
    dump_hist("transport_send_time", &s->transport_send_time);
    dump_hist("transport_poll_time", &s->transport_poll_time);
    dump_hist("hid_report_age", &s->hid_report_age);
    dump_hist("hid_send_interval", &s->hid_send_interval);
    dump_hist("poll_gap", &s->poll_gap);
    dump_hist("timer_lateness", &s->timer_lateness);
    dump_hints(s);
}

/* ======================================================================== */
/* 4. Trace ring                                                            */
/* ======================================================================== */

static volatile bool s_trace_frozen;

#if FLOSS_TRACE_ENTRIES > 0

/** How an id's argument is best shown: decimal, hex, or "(hi16 << 16) | lo16" pair. */
enum { ARG_DEC, ARG_HEX, ARG_PAIR };

/** Name and argument format for floss_trace_dump(), indexed by floss_trace_id_t. */
static const struct {
    const char *name;
    uint8_t arg_fmt;
} s_trace_info[] = {
    [FLOSS_TR_NONE]          = {"NONE", ARG_HEX},
    [FLOSS_TR_POLL]          = {"POLL", ARG_DEC},
    [FLOSS_TR_TX_CMD]        = {"TX_CMD", ARG_HEX},
    [FLOSS_TR_CMD_DONE]      = {"CMD_DONE", ARG_HEX},
    [FLOSS_TR_RX_EVT]        = {"RX_EVT", ARG_HEX},
    [FLOSS_TR_TX_ACL]        = {"TX_ACL", ARG_PAIR},
    [FLOSS_TR_RX_ACL]        = {"RX_ACL", ARG_PAIR},
    [FLOSS_TR_ACL_DONE]      = {"ACL_DONE", ARG_PAIR},
    [FLOSS_TR_CREDIT_STALL]  = {"CR_STALL", ARG_HEX},
    [FLOSS_TR_CONN]          = {"CONN", ARG_HEX},
    [FLOSS_TR_DISCONN]       = {"DISCONN", ARG_PAIR},
    [FLOSS_TR_MODE]          = {"MODE", ARG_PAIR},
    [FLOSS_TR_L2CAP_SIG]     = {"L2CAP_SIG", ARG_PAIR},
    [FLOSS_TR_HID_SET_INPUT] = {"HID_SET_IN", ARG_DEC},
    [FLOSS_TR_HID_SEND]      = {"HID_SEND", ARG_DEC},
    [FLOSS_TR_HID_DEFER]     = {"HID_DEFER", ARG_DEC},
    [FLOSS_TR_HID_OUTPUT]    = {"HID_OUTPUT", ARG_DEC},
    [FLOSS_TR_TIMER]         = {"TIMER", ARG_DEC},
};
_Static_assert(FLOSS_ARRAY_SIZE(s_trace_info) == FLOSS_TR_USER, "s_trace_info out of sync with floss_trace_id_t");

_Static_assert((FLOSS_TRACE_ENTRIES & (FLOSS_TRACE_ENTRIES - 1)) == 0, "FLOSS_TRACE_ENTRIES must be a power of two");
#define TRACE_MASK ((uint32_t)FLOSS_TRACE_ENTRIES - 1u)

static floss_trace_entry_t s_trace[FLOSS_TRACE_ENTRIES];
static uint32_t s_trace_head; /* total entries ever written (wraps harmlessly) */
static bool s_trace_wrapped;  /* head has passed FLOSS_TRACE_ENTRIES at least once */

void floss_trace(uint16_t id, uint32_t arg)
{
    uint32_t t, idx, cs;
    floss_trace_entry_t *e;

    if (s_trace_frozen) {
        return;
    }
    t = floss_hal_time_us();
    cs = floss_hal_critical_enter();
    idx = s_trace_head++;
    if (idx >= TRACE_MASK) {
        s_trace_wrapped = true;
    }
    floss_hal_critical_exit(cs);

    e = &s_trace[idx & TRACE_MASK];
    e->t_us = t;
    e->arg = arg;
    e->id = id;
}

static uint32_t trace_count(void)
{
    return s_trace_wrapped ? FLOSS_TRACE_ENTRIES : s_trace_head;
}
#endif

void floss_trace_freeze(bool frozen)
{
    s_trace_frozen = frozen;
}

/** Returns the newest `max` entries (or all, if fewer), oldest first. */
uint16_t floss_trace_read(floss_trace_entry_t *out_entries, uint16_t max)
{
#if FLOSS_TRACE_ENTRIES > 0
    uint32_t cs = floss_hal_critical_enter();
    uint32_t head = s_trace_head;
    uint32_t n = trace_count();
    floss_hal_critical_exit(cs);

    if (n > max) {
        n = max;
    }
    for (uint32_t i = 0; i < n; i++) {
        out_entries[i] = s_trace[(head - n + i) & TRACE_MASK];
    }
    return (uint16_t)n;
#else
    FLOSS_UNUSED(out_entries);
    FLOSS_UNUSED(max);
    return 0;
#endif
}

void floss_trace_dump(void)
{
#if FLOSS_TRACE_ENTRIES > 0
    bool was_frozen = s_trace_frozen;
    uint32_t head, n, t0, prev;

    /* Freeze while printing: the dump itself must not scroll the ring. */
    s_trace_frozen = true;
    head = s_trace_head;
    n = trace_count();
    out("==== FLOSS trace (%lu entries, oldest first%s) ====", (unsigned long)n, was_frozen ? ", frozen" : "");
    out("  %10s %8s  %-11s %s", "t_rel_us", "dt_us", "event", "arg");
    if (n > 0) {
        t0 = s_trace[(head - n) & TRACE_MASK].t_us;
        prev = t0;
        for (uint32_t i = 0; i < n; i++) {
            const floss_trace_entry_t *e = &s_trace[(head - n + i) & TRACE_MASK];
            char name[16];
            uint8_t fmt = ARG_HEX;
            uint32_t rel = e->t_us - t0, dt = e->t_us - prev;

            if (e->id < FLOSS_TR_USER) {
                snprintf(name, sizeof(name), "%s", s_trace_info[e->id].name);
                fmt = s_trace_info[e->id].arg_fmt;
            } else {
                snprintf(name, sizeof(name), "USER+%u", (unsigned)(e->id - FLOSS_TR_USER));
            }
            if (fmt == ARG_DEC) {
                out("  %10lu %8lu  %-11s %lu", (unsigned long)rel, (unsigned long)dt, name, (unsigned long)e->arg);
            } else if (fmt == ARG_PAIR) {
                out("  %10lu %8lu  %-11s 0x%04lX / %lu", (unsigned long)rel, (unsigned long)dt, name,
                    (unsigned long)(e->arg >> 16), (unsigned long)(e->arg & 0xFFFFu));
            } else {
                out("  %10lu %8lu  %-11s 0x%04lX", (unsigned long)rel, (unsigned long)dt, name,
                    (unsigned long)e->arg);
            }
            prev = e->t_us;
        }
    }
    s_trace_frozen = was_frozen;
#else
    out("==== FLOSS trace disabled (FLOSS_TRACE_ENTRIES = 0) ====");
#endif
}

/* ======================================================================== */
/* Init + assertions                                                        */
/* ======================================================================== */

/*
 * Resets counters and the trace ring. Log levels and the snoop hook are
 * deliberately preserved so an application can configure them before
 * floss_init() and still see the very first packets.
 */
void floss_debug_init(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
#if FLOSS_TRACE_ENTRIES > 0
    s_trace_head = 0;
    s_trace_wrapped = false;
#endif
    s_trace_frozen = false;
}

/* Hosted builds (desktop) abort so debuggers and CI see a crash; on bare
 * metal we park the CPU so a debugger can inspect the state. */
#ifndef FLOSS_ASSERT_ABORTS
#if defined(_WIN32) || defined(__unix__) || defined(__APPLE__)
#define FLOSS_ASSERT_ABORTS 1
#else
#define FLOSS_ASSERT_ABORTS 0
#endif
#endif

void floss_assert_fail(const char *file, int line, const char *expr)
{
    static bool s_in_assert;

    floss_trace_freeze(true); /* keep the history that led here */
    if (!s_in_assert) {
        s_in_assert = true; /* an assert inside the dump must not recurse */
        out("ASSERT FAILED %s:%d: %s", file ? file : "?", line, expr ? expr : "?");
        floss_trace_dump();
    }
#if FLOSS_ASSERT_ABORTS
    abort();
#else
    for (;;) {
        /* halted: attach a debugger, inspect floss_stats() / the trace ring */
    }
#endif
}
