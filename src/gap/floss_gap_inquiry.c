/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_gap_inquiry.c - Discovering other devices (inquiry, central side).
 *
 * Design
 * ------
 * An inquiry is the Classic "who is out there?" broadcast. GAP sends HCI
 * Inquiry with the General Inquiry Access Code; every discoverable device
 * that hears it answers, and the controller reports each answer as one of
 * three events depending on the inquiry mode (GAP selects mode 2 during
 * bring-up, so modern controllers use the last one):
 *
 *   0x02 Inquiry Result            addr, page scan mode, CoD, clock offset
 *   0x22 Inquiry Result with RSSI  ...plus RSSI
 *   0x2F Extended Inquiry Result   ...plus RSSI and the 240-byte EIR
 *                                  (name, service UUIDs) of ONE device
 *
 * 0x02 and 0x22 may carry several responses; their parameters are arrays
 * laid out field by field (all addresses, then all page scan modes, ...),
 * as the specification describes them. Every response becomes one
 * FLOSS_GAP_EVT_INQUIRY_RESULT with the EIR pointing into the event (valid
 * during the callback only).
 *
 * End of inquiry: Inquiry Complete, or the Command Complete of Inquiry
 * Cancel (which per specification produces no Inquiry Complete; some
 * controllers send one anyway). `active` makes sure listeners see exactly
 * one FLOSS_GAP_EVT_INQUIRY_COMPLETE per floss_gap_inquiry_start(), and that
 * stale results queued behind a cancel are dropped.
 *
 * Page hints: an answer tells us the device's page scan repetition mode and
 * its clock offset relative to ours. Paging with these (instead of "R1,
 * offset unknown") lets the controller hit the device's page scan window
 * almost immediately. The last few answers are cached for floss_gap_connect().
 */
#include "floss_gap_internal.h"

/** General Inquiry Access Code (LAP 0x9E8B33). */
#define INQ_GIAC_LAP 0x9E8B33u
/** Spec range of Inquiry_Length (x 1.28 s). */
#define INQ_LENGTH_MAX 0x30u
/** Bytes per response in Inquiry Result / with RSSI (all fields together). */
#define INQ_RESULT_BYTES 14u
/** Extended Inquiry Result: fixed part before the EIR data. */
#define INQ_EXT_FIXED 15u
/** Devices whose page parameters we remember. */
#define INQ_HINTS 4u

typedef struct {
    bool used;
    floss_addr_t addr;
    uint8_t psrm;           /* page scan repetition mode */
    uint16_t clock_offset;  /* bits 14..0 as reported */
    uint32_t seq;           /* recency */
} inq_hint_t;

static struct {
    bool active;
    inq_hint_t hint[INQ_HINTS];
    uint32_t hint_seq;
} s_inq;

void floss_gap_inquiry_init(void)
{
    memset(&s_inq, 0, sizeof(s_inq));
}

bool floss_gap_inquiry_active(void)
{
    return s_inq.active;
}

/* ------------------------------------------------------------------------ */
/* Page hints                                                               */
/* ------------------------------------------------------------------------ */

static void hint_store(const floss_addr_t *addr, uint8_t psrm, uint16_t clock_offset)
{
    inq_hint_t *slot = NULL;
    for (unsigned i = 0; i < INQ_HINTS && !slot; i++) {
        if (s_inq.hint[i].used && floss_addr_eq(&s_inq.hint[i].addr, addr)) {
            slot = &s_inq.hint[i];
        }
    }
    for (unsigned i = 0; i < INQ_HINTS && !slot; i++) {
        if (!s_inq.hint[i].used) {
            slot = &s_inq.hint[i];
        }
    }
    if (!slot) {
        slot = &s_inq.hint[0]; /* evict the oldest */
        for (unsigned i = 1; i < INQ_HINTS; i++) {
            if (s_inq.hint[i].seq < slot->seq) {
                slot = &s_inq.hint[i];
            }
        }
    }
    slot->used = true;
    slot->addr = *addr;
    slot->psrm = psrm;
    slot->clock_offset = (uint16_t)(clock_offset & 0x7FFFu);
    slot->seq = ++s_inq.hint_seq;
}

bool floss_gap_inquiry_page_hint(const floss_addr_t *addr, uint8_t *psrm, uint16_t *clock_offset)
{
    for (unsigned i = 0; i < INQ_HINTS; i++) {
        const inq_hint_t *h = &s_inq.hint[i];
        if (h->used && floss_addr_eq(&h->addr, addr)) {
            *psrm = h->psrm;
            *clock_offset = (uint16_t)(h->clock_offset | 0x8000u); /* Clock_Offset_Valid_Flag */
            return true;
        }
    }
    return false;
}

/* ------------------------------------------------------------------------ */
/* Commands                                                                 */
/* ------------------------------------------------------------------------ */

static void inquiry_finish(uint8_t status)
{
    if (!s_inq.active) {
        return;
    }
    s_inq.active = false;
    FLOSS_LOGI(GAP, "inquiry complete (%s)", floss_hci_status_str(status));
    floss_gap_emit_simple(FLOSS_GAP_EVT_INQUIRY_COMPLETE, FLOSS_GAP_NO_HANDLE, NULL, status);
}

/** Command Status of Inquiry: only a failure needs handling. */
static void inquiry_cmd_done(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len)
{
    if (status != FLOSS_HCI_SUCCESS) {
        FLOSS_LOGW(GAP, "inquiry rejected: %s", floss_hci_status_str(status));
        inquiry_finish(status);
    }
}

floss_err_t floss_gap_inquiry_start(uint8_t duration_1_28s, uint8_t max_results)
{
    if (duration_1_28s == 0 || duration_1_28s > INQ_LENGTH_MAX) {
        return FLOSS_ERR_INVALID;
    }
    if (!floss_gap_hci_live()) {
        return FLOSS_ERR_STATE;
    }
    if (s_inq.active) {
        return FLOSS_ERR_BUSY;
    }
    uint8_t p[5];
    floss_put_le24(&p[0], INQ_GIAC_LAP);
    p[3] = duration_1_28s;
    p[4] = max_results;
    s_inq.active = true; /* before sending: results may follow the Command Status closely */
    floss_err_t err = floss_hci_cmd(FLOSS_HCI_INQUIRY, p, sizeof(p), inquiry_cmd_done, NULL);
    if (err != FLOSS_OK) {
        s_inq.active = false;
        return err;
    }
    FLOSS_LOGI(GAP, "inquiry started (%u x 1.28 s, max %u)", duration_1_28s, max_results);
    return FLOSS_OK;
}

static void inquiry_cancel_done(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len)
{
    if (status == FLOSS_HCI_SUCCESS) {
        inquiry_finish(FLOSS_HCI_SUCCESS);
    } else {
        /* Usually "Command Disallowed": the inquiry ended on its own and its
         * Inquiry Complete is (or was) delivered. */
        FLOSS_LOGD(GAP, "inquiry cancel: %s", floss_hci_status_str(status));
    }
}

floss_err_t floss_gap_inquiry_cancel(void)
{
    if (!s_inq.active) {
        return FLOSS_ERR_STATE;
    }
    return floss_hci_cmd(FLOSS_HCI_INQUIRY_CANCEL, NULL, 0, inquiry_cancel_done, NULL);
}

/* ------------------------------------------------------------------------ */
/* Events                                                                   */
/* ------------------------------------------------------------------------ */

static void emit_result(const floss_addr_t *addr, uint32_t cod, int8_t rssi, const uint8_t *eir, uint8_t eir_len)
{
    floss_gap_evt_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = FLOSS_GAP_EVT_INQUIRY_RESULT;
    evt.handle = FLOSS_GAP_NO_HANDLE;
    evt.addr = *addr;
    evt.cod = cod;
    evt.rssi = rssi;
    evt.eir = eir_len ? eir : NULL;
    evt.eir_len = eir_len;
    if (FLOSS_LOG_DEBUG <= FLOSS_LOG_LEVEL) {
        char s[18];
        floss_addr_to_str(addr, s);
        FLOSS_LOGD(GAP, "inquiry result %s CoD 0x%06lX rssi %d eir %u", s, (unsigned long)cod, rssi, eir_len);
    }
    floss_gap_emit(&evt);
}

/**
 * Inquiry Result (with_rssi false) / Inquiry Result with RSSI: Num_Responses
 * then field arrays. The reserved field is 2 bytes per response in the
 * first, 1 in the second (whose RSSI array ends the event).
 */
static void on_inquiry_result(const uint8_t *p, uint8_t len, bool with_rssi)
{
    if (len < 1) {
        return;
    }
    uint8_t n = p[0];
    if (n == 0 || 1u + INQ_RESULT_BYTES * (uint32_t)n > len) {
        FLOSS_LOGW(GAP, "malformed inquiry result (%u responses, %u bytes)", n, len);
        FLOSS_STAT_INC(rx_dropped);
        return;
    }
    const uint8_t *addrs = p + 1;
    const uint8_t *psrm = addrs + 6u * n;
    const uint8_t *cods = psrm + n + (with_rssi ? 1u : 2u) * n;
    const uint8_t *clocks = cods + 3u * n;
    const uint8_t *rssis = clocks + 2u * n;
    for (uint8_t i = 0; i < n && s_inq.active; i++) {
        floss_addr_t addr;
        memcpy(addr.b, addrs + 6u * i, 6);
        hint_store(&addr, psrm[i], floss_get_le16(clocks + 2u * i));
        emit_result(&addr, floss_get_le24(cods + 3u * i), with_rssi ? (int8_t)rssis[i] : FLOSS_GAP_RSSI_UNKNOWN,
                    NULL, 0);
    }
}

/**
 * Extended Inquiry Result: Num_Responses (always 1), addr(6), page scan
 * repetition mode(1), reserved(1), CoD(3), clock offset(2), RSSI(1), EIR(240).
 */
static void on_extended_inquiry_result(const uint8_t *p, uint8_t len)
{
    if (len < INQ_EXT_FIXED || p[0] != 1) {
        FLOSS_LOGW(GAP, "malformed extended inquiry result (%u bytes)", len);
        FLOSS_STAT_INC(rx_dropped);
        return;
    }
    floss_addr_t addr;
    memcpy(addr.b, p + 1, 6);
    uint8_t psrm = p[7];
    uint32_t cod = floss_get_le24(p + 9);
    uint16_t clock = floss_get_le16(p + 12);
    int8_t rssi = (int8_t)p[14];
    const uint8_t *eir = p + INQ_EXT_FIXED;
    uint8_t eir_cap = (uint8_t)FLOSS_MIN((uint16_t)(len - INQ_EXT_FIXED), (uint16_t)FLOSS_GAP_EIR_LEN);

    /* Significant part: up to the first zero-length structure, never past the end. */
    uint8_t eir_len = 0;
    while (eir_len < eir_cap && eir[eir_len] != 0 && (uint16_t)(eir_len + 1u + eir[eir_len]) <= eir_cap) {
        eir_len = (uint8_t)(eir_len + 1u + eir[eir_len]);
    }
    if (!s_inq.active) {
        return;
    }
    hint_store(&addr, psrm, clock);
    emit_result(&addr, cod, rssi, eir, eir_len);
}

bool floss_gap_inquiry_on_event(uint8_t code, const uint8_t *params, uint8_t len)
{
    switch (code) {
    case FLOSS_EVT_INQUIRY_COMPLETE:
        inquiry_finish(len >= 1 ? params[0] : FLOSS_HCI_SUCCESS);
        return true;
    case FLOSS_EVT_INQUIRY_RESULT:
        on_inquiry_result(params, len, false);
        return true;
    case FLOSS_EVT_INQUIRY_RESULT_WITH_RSSI:
        on_inquiry_result(params, len, true);
        return true;
    case FLOSS_EVT_EXTENDED_INQUIRY_RESULT:
        on_extended_inquiry_result(params, len);
        return true;
    default:
        return false;
    }
}
