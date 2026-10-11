/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_gap.c - Classic GAP core: settings, bring-up, links, link policy.
 *
 * Design
 * ------
 * Settings. Every setter stores its value first. If the controller is
 * already up it also sends the HCI command right away; otherwise the value
 * is applied by the bring-up sequence. So the application may configure GAP
 * at any time after floss_init() without caring about ordering.
 *
 * Bring-up. floss_gap_on_hci_ready() runs a small step machine: each step
 * sends one command whose completion callback sends the next. This keeps at
 * most one GAP command in HCI's queue (it never overflows, whatever its
 * length) and makes the order on the wire exactly the table order. A failing
 * step is logged and skipped - an old controller rejecting e.g. Write
 * Inquiry Scan Type must not stop the device from becoming usable. When the
 * last step completes GAP emits READY, then hands over to LE.
 *
 * Links. HCI owns the connection table (floss_conn_t). GAP keeps a parallel
 * floss_gap_link_t per Classic link with what only GAP cares about: the
 * peer's Class of Device, whether we paged it, and pairing progress. Slots
 * are keyed by address because they exist before the handle does
 * (Connection Request, outgoing page) and because pairing events only carry
 * the address.
 *
 * Role. For incoming connections we stay peripheral (Accept with role 0x01):
 * a gamepad asking for a role switch costs the host a reconfiguration and
 * some hosts reject it outright. Outgoing pages allow the host to take the
 * central role (allow role switch = 1), which is what hosts expect.
 */
#include "floss_gap_internal.h"

/* Defaults */
#define GAP_DEFAULT_NAME            "FLOSS Device"
#define GAP_DEFAULT_PSCAN_INTERVAL  0x0400
#define GAP_DEFAULT_PSCAN_WINDOW    0x0012
#define GAP_ISCAN_INTERVAL          0x0800
#define GAP_ISCAN_WINDOW            0x0012
#define GAP_PAGE_TIMEOUT            0x2000 /* 5.12 s */
#define GAP_CONN_ACCEPT_TIMEOUT     0x1FA0 /* 5.06 s */
#define GAP_NAME_LEN                248u

/* Create Connection parameters for paging a host. */
#define GAP_ACL_PACKET_TYPES        0xCC18 /* DM1 DH1 DM3 DH3 DM5 DH5 */
#define GAP_PSCAN_REP_MODE_R1       0x01

/* HCI link types in Connection Request / Complete. */
#define GAP_LINK_TYPE_ACL           0x01

/* Bring-up retry when HCI's command queue is momentarily full. */
#define GAP_READY_RETRY_US          1000u
/* Retry period for a Write Scan Enable the HCI command queue had no room for. */
#define GAP_SCAN_RETRY_US           10000u

/** Bring-up steps, in wire order. */
typedef enum {
    STEP_SIMPLE_PAIRING = 0,
    STEP_SC_HOST_SUPPORT,
    STEP_INQUIRY_MODE,
    STEP_PAGE_SCAN_TYPE,
    STEP_PAGE_SCAN_ACTIVITY,
    STEP_INQUIRY_SCAN_TYPE,
    STEP_INQUIRY_SCAN_ACTIVITY,
    STEP_DEFAULT_LINK_POLICY,
    STEP_CLASS_OF_DEVICE,
    STEP_LOCAL_NAME,
    STEP_EIR,
    STEP_PAGE_TIMEOUT,
    STEP_CONN_ACCEPT_TIMEOUT,
    STEP_SCAN_ENABLE,
    STEP_DONE
} gap_step_t;

static struct {
    /* settings */
    uint8_t name[GAP_NAME_LEN]; /* zero padded, exactly as Write Local Name wants it */
    uint8_t name_len;
    uint32_t cod;
    bool discoverable;
    bool connectable;
    uint16_t pscan_interval;
    uint16_t pscan_window;
    bool pscan_interlaced;
    uint16_t default_policy;

    /* lifecycle */
    gap_step_t step;
    bool applying;   /* bring-up sequence started: setters send directly */
    bool ready;      /* READY emitted */
    floss_timer_t retry_timer;
    floss_timer_t scan_timer; /* re-sends Write Scan Enable the HCI queue refused */

    /* listeners */
    struct {
        floss_gap_evt_fn fn;
        void *user;
    } listener[FLOSS_GAP_MAX_LISTENERS];

    floss_gap_link_t link[FLOSS_MAX_CONNECTIONS];
} s_gap;

/* ======================================================================== */
/* Listeners / events                                                       */
/* ======================================================================== */

floss_err_t floss_gap_add_listener(floss_gap_evt_fn fn, void *user)
{
    if (!fn) {
        return FLOSS_ERR_INVALID;
    }
    for (int i = 0; i < FLOSS_GAP_MAX_LISTENERS; i++) {
        if (!s_gap.listener[i].fn) {
            s_gap.listener[i].fn = fn;
            s_gap.listener[i].user = user;
            return FLOSS_OK;
        }
    }
    return FLOSS_ERR_NOMEM;
}

void floss_gap_emit(const floss_gap_evt_t *evt)
{
    for (int i = 0; i < FLOSS_GAP_MAX_LISTENERS; i++) {
        if (s_gap.listener[i].fn) {
            s_gap.listener[i].fn(evt, s_gap.listener[i].user);
        }
    }
}

void floss_gap_emit_simple(floss_gap_evt_type_t type, uint16_t handle, const floss_addr_t *addr, uint8_t status)
{
    floss_gap_evt_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = type;
    evt.handle = handle;
    if (addr) {
        evt.addr = *addr;
    }
    evt.status = status;
    floss_gap_emit(&evt);
}

/* ======================================================================== */
/* Link slots                                                               */
/* ======================================================================== */

floss_gap_link_t *floss_gap_link_by_addr(const floss_addr_t *addr)
{
    for (int i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        floss_gap_link_t *l = &s_gap.link[i];
        if (l->used && floss_addr_eq(&l->addr, addr)) {
            return l;
        }
    }
    return NULL;
}

floss_gap_link_t *floss_gap_link_by_handle(uint16_t handle)
{
    for (int i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        floss_gap_link_t *l = &s_gap.link[i];
        if (l->used && l->has_handle && l->handle == handle) {
            return l;
        }
    }
    return NULL;
}

floss_gap_link_t *floss_gap_link_get(const floss_addr_t *addr)
{
    floss_gap_link_t *l = floss_gap_link_by_addr(addr);
    if (l) {
        return l;
    }
    for (int i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        l = &s_gap.link[i];
        if (!l->used) {
            memset(l, 0, sizeof(*l));
            l->used = true;
            l->addr = *addr;
            l->handle = FLOSS_GAP_NO_HANDLE;
            floss_timer_init(&l->sec_timer, floss_gap_sec_timer_fn, l);
            return l;
        }
    }
    return NULL;
}

floss_gap_link_t *floss_gap_link_for_event(const floss_addr_t *addr)
{
    floss_gap_link_t *l = floss_gap_link_by_addr(addr);
    if (l) {
        return l;
    }
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        const floss_conn_t *c = floss_hci_conn_at(i);
        if (c->state != FLOSS_CONN_FREE && !c->le && floss_addr_eq(&c->addr, addr)) {
            return floss_gap_link_get(addr);
        }
    }
    return NULL;
}

static void link_free(floss_gap_link_t *l)
{
    floss_timer_stop(&l->sec_timer);
    memset(l, 0, sizeof(*l));
}

uint16_t floss_gap_handle_for_addr(const floss_addr_t *addr)
{
    floss_conn_t *c = floss_hci_conn_by_addr(addr);
    if (c && !c->le && (c->state == FLOSS_CONN_OPEN || c->state == FLOSS_CONN_CLOSING)) {
        return c->handle;
    }
    floss_gap_link_t *l = floss_gap_link_by_addr(addr);
    return (l && l->has_handle) ? l->handle : FLOSS_GAP_NO_HANDLE;
}

/** Classic link for handle (LE links are LE's business), or NULL. */
static floss_conn_t *classic_conn(uint16_t handle)
{
    floss_conn_t *c = floss_hci_conn_by_handle(handle);
    return (c && !c->le) ? c : NULL;
}

/* ======================================================================== */
/* Settings -> HCI commands (shared by setters and the bring-up sequence)   */
/* ======================================================================== */

bool floss_gap_hci_live(void)
{
    return s_gap.applying;
}

const uint8_t *floss_gap_local_name(uint8_t *len)
{
    *len = s_gap.name_len;
    return s_gap.name;
}

static floss_err_t send_u8(uint16_t opcode, uint8_t v, floss_hci_cmd_cb cb)
{
    return floss_hci_cmd(opcode, &v, 1, cb, NULL);
}

static floss_err_t send_u16(uint16_t opcode, uint16_t v, floss_hci_cmd_cb cb)
{
    uint8_t p[2];
    floss_put_le16(p, v);
    return floss_hci_cmd(opcode, p, sizeof(p), cb, NULL);
}

static floss_err_t send_u16x2(uint16_t opcode, uint16_t a, uint16_t b, floss_hci_cmd_cb cb)
{
    uint8_t p[4];
    floss_put_le16(&p[0], a);
    floss_put_le16(&p[2], b);
    return floss_hci_cmd(opcode, p, sizeof(p), cb, NULL);
}

static floss_err_t send_scan_enable(floss_hci_cmd_cb cb)
{
    /* bit 0 = inquiry scan (discoverable), bit 1 = page scan (connectable) */
    uint8_t v = (uint8_t)((s_gap.discoverable ? 0x01 : 0x00) | (s_gap.connectable ? 0x02 : 0x00));
    return send_u8(FLOSS_HCI_WRITE_SCAN_ENABLE, v, cb);
}

static floss_err_t send_class(floss_hci_cmd_cb cb)
{
    uint8_t p[3];
    floss_put_le24(p, s_gap.cod);
    return floss_hci_cmd(FLOSS_HCI_WRITE_CLASS_OF_DEVICE, p, sizeof(p), cb, NULL);
}

static floss_err_t send_name(floss_hci_cmd_cb cb)
{
    return floss_hci_cmd(FLOSS_HCI_WRITE_LOCAL_NAME, s_gap.name, GAP_NAME_LEN, cb, NULL);
}

/* ======================================================================== */
/* Bring-up sequence                                                        */
/* ======================================================================== */

static void ready_next(void);
static void scan_retry(floss_timer_t *t, void *arg);

static void ready_cmd_done(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len)
{
    if (status != FLOSS_HCI_SUCCESS) {
        FLOSS_LOGW(GAP, "bring-up: opcode 0x%04X failed: %s (continuing)", opcode, floss_hci_status_str(status));
    }
    s_gap.step++;
    ready_next();
}

static void ready_retry(floss_timer_t *t, void *arg)
{
    ready_next();
}

/**
 * Send the command for the current step (skipping steps that do not apply).
 * Its completion callback advances to the next step.
 */
static void ready_next(void)
{
    for (;;) {
        floss_err_t err = FLOSS_OK;
        bool sent = true;

        switch (s_gap.step) {
        case STEP_SIMPLE_PAIRING:
            err = send_u8(FLOSS_HCI_WRITE_SIMPLE_PAIRING_MODE, 0x01, ready_cmd_done);
            break;
        case STEP_SC_HOST_SUPPORT:
            /* Octet 32 bit 3 of the supported-commands bitmap. */
            if (floss_gap_sec_want_sc() && floss_hci_cmd_supported(32, 3)) {
                err = send_u8(FLOSS_HCI_WRITE_SECURE_CONNECTIONS_HOST_SUPPORT, 0x01, ready_cmd_done);
            } else {
                sent = false;
            }
            break;
        case STEP_INQUIRY_MODE:
            err = send_u8(FLOSS_HCI_WRITE_INQUIRY_MODE, 0x02 /* extended (EIR) */, ready_cmd_done);
            break;
        case STEP_PAGE_SCAN_TYPE:
            err = send_u8(FLOSS_HCI_WRITE_PAGE_SCAN_TYPE, s_gap.pscan_interlaced ? 1 : 0, ready_cmd_done);
            break;
        case STEP_PAGE_SCAN_ACTIVITY:
            err = send_u16x2(FLOSS_HCI_WRITE_PAGE_SCAN_ACTIVITY, s_gap.pscan_interval, s_gap.pscan_window,
                             ready_cmd_done);
            break;
        case STEP_INQUIRY_SCAN_TYPE:
            err = send_u8(FLOSS_HCI_WRITE_INQUIRY_SCAN_TYPE, 0x01 /* interlaced */, ready_cmd_done);
            break;
        case STEP_INQUIRY_SCAN_ACTIVITY:
            err = send_u16x2(FLOSS_HCI_WRITE_INQUIRY_SCAN_ACTIVITY, GAP_ISCAN_INTERVAL, GAP_ISCAN_WINDOW,
                             ready_cmd_done);
            break;
        case STEP_DEFAULT_LINK_POLICY:
            err = send_u16(FLOSS_HCI_WRITE_DEFAULT_LINK_POLICY, s_gap.default_policy, ready_cmd_done);
            break;
        case STEP_CLASS_OF_DEVICE:
            err = send_class(ready_cmd_done);
            break;
        case STEP_LOCAL_NAME:
            err = send_name(ready_cmd_done);
            break;
        case STEP_EIR:
            err = floss_gap_eir_send(ready_cmd_done, NULL);
            break;
        case STEP_PAGE_TIMEOUT:
            err = send_u16(FLOSS_HCI_WRITE_PAGE_TIMEOUT, GAP_PAGE_TIMEOUT, ready_cmd_done);
            break;
        case STEP_CONN_ACCEPT_TIMEOUT:
            err = send_u16(FLOSS_HCI_WRITE_CONNECTION_ACCEPT_TIMEOUT, GAP_CONN_ACCEPT_TIMEOUT, ready_cmd_done);
            break;
        case STEP_SCAN_ENABLE:
            err = send_scan_enable(ready_cmd_done);
            break;
        case STEP_DONE:
        default:
            s_gap.ready = true;
            FLOSS_LOGI(GAP, "ready (scan: %s%s)", s_gap.discoverable ? "discoverable " : "",
                       s_gap.connectable ? "connectable" : "");
            floss_hid_on_controller_ready();
            floss_gap_emit_simple(FLOSS_GAP_EVT_READY, FLOSS_GAP_NO_HANDLE, NULL, 0);
#if FLOSS_ENABLE_LE
            floss_le_on_hci_ready();
#endif
            return;
        }

        if (!sent) {
            s_gap.step++;
            continue;
        }
        if (err != FLOSS_OK) {
            /* HCI command queue full: try this step again shortly. */
            FLOSS_LOGD(GAP, "bring-up step %d deferred (%s)", (int)s_gap.step, floss_err_str(err));
            floss_timer_start(&s_gap.retry_timer, GAP_READY_RETRY_US);
        }
        return;
    }
}

void floss_gap_on_hci_ready(void)
{
    /* A (re)started controller has no links: drop any stale GAP link state. */
    for (int i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        if (s_gap.link[i].used) {
            link_free(&s_gap.link[i]);
        }
    }
    floss_gap_inquiry_init(); /* a reset controller runs no inquiry */
    s_gap.ready = false;
    s_gap.applying = true;
    s_gap.step = STEP_SIMPLE_PAIRING;
    floss_timer_stop(&s_gap.retry_timer);
    FLOSS_LOGI(GAP, "applying settings");
    ready_next();
}

/* ======================================================================== */
/* Lifecycle                                                                */
/* ======================================================================== */

void floss_gap_init(void)
{
    memset(&s_gap, 0, sizeof(s_gap));
    s_gap.name_len = (uint8_t)strlen(GAP_DEFAULT_NAME);
    memcpy(s_gap.name, GAP_DEFAULT_NAME, s_gap.name_len);
    s_gap.pscan_interval = GAP_DEFAULT_PSCAN_INTERVAL;
    s_gap.pscan_window = GAP_DEFAULT_PSCAN_WINDOW;
    s_gap.pscan_interlaced = true; /* interlaced halves the time a host needs to page us */
    s_gap.default_policy = FLOSS_LINK_POLICY_ROLE_SWITCH | FLOSS_LINK_POLICY_SNIFF;
    floss_timer_init(&s_gap.retry_timer, ready_retry, NULL);
    floss_timer_init(&s_gap.scan_timer, scan_retry, NULL);

    floss_gap_sec_init();
    floss_gap_eir_init();
    floss_gap_inquiry_init();
    floss_gap_bonds_load();
}

/* ======================================================================== */
/* Identity & visibility                                                    */
/* ======================================================================== */

floss_err_t floss_gap_set_name(const char *name)
{
    if (!name) {
        return FLOSS_ERR_INVALID;
    }
    size_t n = strlen(name);
    if (n > GAP_NAME_LEN) {
        return FLOSS_ERR_INVALID;
    }
    memset(s_gap.name, 0, sizeof(s_gap.name));
    memcpy(s_gap.name, name, n);
    s_gap.name_len = (uint8_t)n;
    if (!s_gap.applying) {
        return FLOSS_OK;
    }
    floss_err_t err = send_name(NULL);
    floss_err_t err2 = floss_gap_update_eir();
    return err != FLOSS_OK ? err : err2;
}

floss_err_t floss_gap_set_class(uint32_t cod)
{
    if (cod > 0xFFFFFFu) {
        return FLOSS_ERR_INVALID;
    }
    s_gap.cod = cod;
    return s_gap.applying ? send_class(NULL) : FLOSS_OK;
}

/**
 * Scan enable is the one setting whose loss is unrecoverable (no host could
 * ever reach us again), so a write the HCI command queue had no room for
 * (e.g. while a misbehaving controller floods us) is retried until it fits.
 */
static void scan_apply(void)
{
    if (s_gap.applying && send_scan_enable(NULL) == FLOSS_ERR_NOMEM) {
        floss_timer_start(&s_gap.scan_timer, GAP_SCAN_RETRY_US);
    }
}

static void scan_retry(floss_timer_t *t, void *arg)
{
    FLOSS_UNUSED(t);
    FLOSS_UNUSED(arg);
    scan_apply();
}

floss_err_t floss_gap_set_scan(bool discoverable, bool connectable)
{
    s_gap.discoverable = discoverable;
    s_gap.connectable = connectable;
    FLOSS_LOGD(GAP, "scan: discoverable=%d connectable=%d", discoverable, connectable);
    scan_apply();
    return FLOSS_OK;
}

floss_err_t floss_gap_set_page_scan(uint16_t interval, uint16_t window, bool interlaced)
{
    /* Spec ranges: interval 0x0012..0x1000, window 0x0011..0x1000, window <= interval. */
    if (interval < 0x0012 || interval > 0x1000 || window < 0x0011 || window > interval) {
        return FLOSS_ERR_INVALID;
    }
    s_gap.pscan_interval = interval;
    s_gap.pscan_window = window;
    s_gap.pscan_interlaced = interlaced;
    if (!s_gap.applying) {
        return FLOSS_OK;
    }
    floss_err_t err = send_u8(FLOSS_HCI_WRITE_PAGE_SCAN_TYPE, interlaced ? 1 : 0, NULL);
    if (err != FLOSS_OK) {
        return err;
    }
    return send_u16x2(FLOSS_HCI_WRITE_PAGE_SCAN_ACTIVITY, interval, window, NULL);
}

/* ======================================================================== */
/* Connections                                                              */
/* ======================================================================== */

static void create_conn_done(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len)
{
    if (status == FLOSS_HCI_SUCCESS) {
        return; /* Command Status OK: the page is running, Connection Complete follows */
    }
    floss_gap_link_t *l = (floss_gap_link_t *)ctx;
    if (!l->used || !l->outgoing || l->has_handle) {
        return; /* slot was recycled meanwhile */
    }
    floss_addr_t addr = l->addr;
    FLOSS_LOGW(GAP, "page rejected: %s", floss_hci_status_str(status));
    link_free(l);
    floss_gap_emit_simple(FLOSS_GAP_EVT_CONNECT_FAILED, FLOSS_GAP_NO_HANDLE, &addr, status);
}

floss_err_t floss_gap_connect(const floss_addr_t *addr)
{
    if (!addr || floss_addr_is_zero(addr)) {
        return FLOSS_ERR_INVALID;
    }
    if (!s_gap.applying) {
        return FLOSS_ERR_STATE;
    }
    if (floss_gap_link_by_addr(addr)) {
        return FLOSS_ERR_STATE; /* already connected or connecting */
    }
    if (floss_gap_inquiry_active()) {
        return FLOSS_ERR_BUSY; /* controllers cannot page while inquiring: cancel first */
    }
    floss_gap_link_t *l = floss_gap_link_get(addr);
    if (!l) {
        return FLOSS_ERR_NOMEM;
    }
    l->outgoing = true;

    /* A recent inquiry answer tells us how and when the device scans. */
    uint8_t psrm = GAP_PSCAN_REP_MODE_R1;
    uint16_t clock_offset = 0; /* unknown */
    (void)floss_gap_inquiry_page_hint(addr, &psrm, &clock_offset);

    uint8_t p[13];
    memcpy(&p[0], addr->b, 6);
    floss_put_le16(&p[6], GAP_ACL_PACKET_TYPES);
    p[8] = psrm;
    p[9] = 0x00;                 /* reserved */
    floss_put_le16(&p[10], clock_offset);
    p[12] = 0x01;                /* allow role switch: the host may become central */
    floss_err_t err = floss_hci_cmd(FLOSS_HCI_CREATE_CONNECTION, p, sizeof(p), create_conn_done, l);
    if (err != FLOSS_OK) {
        link_free(l);
        return err;
    }
    char s[18];
    floss_addr_to_str(addr, s);
    FLOSS_LOGI(GAP, "paging %s", s);
    return FLOSS_OK;
}

floss_err_t floss_gap_connect_cancel(const floss_addr_t *addr)
{
    if (!addr) {
        return FLOSS_ERR_INVALID;
    }
    floss_gap_link_t *l = floss_gap_link_by_addr(addr);
    if (!l || !l->outgoing || l->has_handle) {
        return FLOSS_ERR_NOT_FOUND;
    }
    /* Connection Complete (status Unknown Connection / Page Timeout...) frees the slot. */
    return floss_hci_cmd(FLOSS_HCI_CREATE_CONNECTION_CANCEL, addr->b, 6, NULL, NULL);
}

floss_err_t floss_gap_disconnect(uint16_t handle, uint8_t reason)
{
    if (!classic_conn(handle)) {
        return FLOSS_ERR_NOT_FOUND;
    }
    uint8_t p[3];
    floss_put_le16(p, handle);
    p[2] = reason ? reason : FLOSS_HCI_ERR_REMOTE_USER_TERMINATED;
    return floss_hci_cmd(FLOSS_HCI_DISCONNECT, p, sizeof(p), NULL, NULL);
}

/* ======================================================================== */
/* Link policy / power                                                      */
/* ======================================================================== */

floss_err_t floss_gap_set_default_link_policy(uint16_t policy)
{
    s_gap.default_policy = policy;
    return s_gap.applying ? send_u16(FLOSS_HCI_WRITE_DEFAULT_LINK_POLICY, policy, NULL) : FLOSS_OK;
}

floss_err_t floss_gap_set_link_policy(uint16_t handle, uint16_t policy)
{
    if (!classic_conn(handle)) {
        return FLOSS_ERR_NOT_FOUND;
    }
    return send_u16x2(FLOSS_HCI_WRITE_LINK_POLICY_SETTINGS, handle, policy, NULL);
}

floss_err_t floss_gap_sniff(uint16_t handle, uint16_t min_interval, uint16_t max_interval, uint16_t attempt,
                            uint16_t timeout)
{
    if (!classic_conn(handle)) {
        return FLOSS_ERR_NOT_FOUND;
    }
    if (min_interval < 2 || min_interval > max_interval || attempt == 0 || attempt > 0x7FFF || timeout > 0x7FFF) {
        return FLOSS_ERR_INVALID;
    }
    uint8_t p[10];
    floss_put_le16(&p[0], handle);
    floss_put_le16(&p[2], max_interval); /* note: the command takes max before min */
    floss_put_le16(&p[4], min_interval);
    floss_put_le16(&p[6], attempt);
    floss_put_le16(&p[8], timeout);
    return floss_hci_cmd(FLOSS_HCI_SNIFF_MODE, p, sizeof(p), NULL, NULL);
}

floss_err_t floss_gap_sniff_exit(uint16_t handle)
{
    if (!classic_conn(handle)) {
        return FLOSS_ERR_NOT_FOUND;
    }
    return send_u16(FLOSS_HCI_EXIT_SNIFF_MODE, handle, NULL);
}

floss_err_t floss_gap_sniff_subrating(uint16_t handle, uint16_t max_latency, uint16_t min_remote_timeout,
                                      uint16_t min_local_timeout)
{
    if (!classic_conn(handle)) {
        return FLOSS_ERR_NOT_FOUND;
    }
    uint8_t p[8];
    floss_put_le16(&p[0], handle);
    floss_put_le16(&p[2], max_latency);
    floss_put_le16(&p[4], min_remote_timeout);
    floss_put_le16(&p[6], min_local_timeout);
    return floss_hci_cmd(FLOSS_HCI_SNIFF_SUBRATING, p, sizeof(p), NULL, NULL);
}

floss_err_t floss_gap_set_supervision_timeout(uint16_t handle, uint16_t slots)
{
    if (!classic_conn(handle)) {
        return FLOSS_ERR_NOT_FOUND;
    }
    return send_u16x2(FLOSS_HCI_WRITE_LINK_SUPERVISION_TIMEOUT, handle, slots, NULL);
}

floss_err_t floss_gap_set_flush_timeout(uint16_t handle, uint16_t ms)
{
    if (!classic_conn(handle)) {
        return FLOSS_ERR_NOT_FOUND;
    }
    /* Units of 0.625 ms: slots = ms * 1000 / 625 = ms * 8 / 5 (rounded).
     * 0 means "never flush"; the controller maximum is 0x07FF (~1279 ms). */
    uint32_t slots = 0;
    if (ms > 0) {
        slots = ((uint32_t)ms * 8u + 2u) / 5u;
        slots = FLOSS_MAX(slots, 1u);
        slots = FLOSS_MIN(slots, 0x07FFu);
    }
    return send_u16x2(FLOSS_HCI_WRITE_AUTOMATIC_FLUSH_TIMEOUT, handle, (uint16_t)slots, NULL);
}

floss_err_t floss_gap_set_afh_assessment(bool enabled)
{
    uint8_t mode = enabled ? 1u : 0u;
    return floss_hci_cmd(FLOSS_HCI_WRITE_AFH_CHANNEL_ASSESSMENT_MODE, &mode, 1, NULL, NULL);
}

floss_err_t floss_gap_set_afh_host_channels(const uint8_t map[10])
{
    uint8_t m[10];
    memcpy(m, map, 10);
    m[9] &= 0x7F; /* bit 79 is reserved */
    return floss_hci_cmd(FLOSS_HCI_SET_AFH_HOST_CHANNEL_CLASSIFICATION, m, sizeof m, NULL, NULL);
}

typedef struct {
    floss_gap_afh_map_fn fn;
    void *user;
    uint16_t handle;
} afh_read_t;
static afh_read_t s_afh_read; /* one read at a time is plenty */

static void afh_map_done(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t len)
{
    afh_read_t *r = (afh_read_t *)ctx;
    static const uint8_t none[10];
    /* Return parameters after status: handle(2) mode(1) map(10). */
    bool ok = status == FLOSS_HCI_SUCCESS && ret != NULL && len >= 13;
    floss_gap_afh_map_fn fn = r->fn;
    r->fn = NULL;
    if (fn) {
        fn(r->handle, ok ? 0 : (status ? status : FLOSS_HCI_ERR_UNSPECIFIED), ok ? ret[2] : 0, ok ? &ret[3] : none,
           r->user);
    }
}

floss_err_t floss_gap_read_afh_map(uint16_t handle, floss_gap_afh_map_fn fn, void *user)
{
    if (!classic_conn(handle)) {
        return FLOSS_ERR_NOT_FOUND;
    }
    if (s_afh_read.fn) {
        return FLOSS_ERR_BUSY;
    }
    uint8_t p[2];
    floss_put_le16(p, handle);
    s_afh_read.fn = fn;
    s_afh_read.user = user;
    s_afh_read.handle = handle;
    floss_err_t err = floss_hci_cmd(FLOSS_HCI_READ_AFH_CHANNEL_MAP, p, sizeof p, afh_map_done, &s_afh_read);
    if (err != FLOSS_OK) {
        s_afh_read.fn = NULL;
    }
    return err;
}

floss_err_t floss_gap_qos_setup(uint16_t handle, uint32_t latency_us, uint32_t token_rate)
{
    if (!classic_conn(handle)) {
        return FLOSS_ERR_NOT_FOUND;
    }
    /* Connection_Handle(2) Unused(1) Service_Type(1) Token_Rate(4)
     * Peak_Bandwidth(4) Latency(4) Delay_Variation(4). */
    uint8_t p[20];
    floss_put_le16(&p[0], handle);
    p[2] = 0x00;
    p[3] = 0x02; /* guaranteed */
    floss_put_le32(&p[4], token_rate);
    floss_put_le32(&p[8], 0);        /* peak bandwidth: not specified */
    floss_put_le32(&p[12], latency_us);
    floss_put_le32(&p[16], 0xFFFFFFFFu); /* delay variation: don't care */
    FLOSS_LOGI(GAP, "QoS setup handle 0x%03X latency %lu us", handle, (unsigned long)latency_us);
    return floss_hci_cmd(FLOSS_HCI_QOS_SETUP, p, sizeof(p), NULL, NULL);
}

floss_err_t floss_gap_switch_role(uint16_t handle, uint8_t role)
{
    floss_conn_t *c = classic_conn(handle);
    if (!c) {
        return FLOSS_ERR_NOT_FOUND;
    }
    if (role > 1) {
        return FLOSS_ERR_INVALID;
    }
    uint8_t p[7];
    memcpy(&p[0], c->addr.b, 6);
    p[6] = role;
    return floss_hci_cmd(FLOSS_HCI_SWITCH_ROLE, p, sizeof(p), NULL, NULL);
}

static void read_rssi_done(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len)
{
    /* Command Complete return: status (already split off), handle(2), rssi(1, signed dB). */
    uint16_t handle = (uint16_t)(uintptr_t)ctx;
    if (status == FLOSS_HCI_SUCCESS && ret && ret_len >= 3) {
        handle = floss_get_le16(ret);
    } else if (status == FLOSS_HCI_SUCCESS) {
        return; /* Command Status for a Command Complete opcode: wait for the real result */
    }
    floss_gap_evt_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = FLOSS_GAP_EVT_RSSI;
    evt.handle = handle;
    evt.status = status;
    evt.rssi = (status == FLOSS_HCI_SUCCESS) ? (int8_t)ret[2] : 0;
    floss_conn_t *c = floss_hci_conn_by_handle(handle);
    if (c) {
        evt.addr = c->addr;
    }
    floss_gap_emit(&evt);
}

floss_err_t floss_gap_read_rssi(uint16_t handle)
{
    if (!classic_conn(handle)) {
        return FLOSS_ERR_NOT_FOUND;
    }
    uint8_t p[2];
    floss_put_le16(p, handle);
    return floss_hci_cmd(FLOSS_HCI_READ_RSSI, p, sizeof(p), read_rssi_done, (void *)(uintptr_t)handle);
}

/* ======================================================================== */
/* Connection events                                                        */
/* ======================================================================== */

/** Connection Request: addr(6) cod(3) link_type(1). */
static void on_connection_request(const uint8_t *p, uint8_t len)
{
    if (len < 10) {
        return;
    }
    floss_addr_t addr;
    memcpy(addr.b, p, 6);
    uint32_t cod = floss_get_le24(&p[6]);
    uint8_t link_type = p[9];

    char s[18];
    floss_addr_to_str(&addr, s);

    /* HCI allocated an INCOMING slot if one was free; we only decide. */
    floss_conn_t *c = floss_hci_conn_by_addr(&addr);
    floss_gap_link_t *l = NULL;
    if (link_type == GAP_LINK_TYPE_ACL && c) {
        l = floss_gap_link_get(&addr);
    }
    if (!l) {
        FLOSS_LOGW(GAP, "rejecting connection from %s (link type %u)", s, link_type);
        uint8_t r[7];
        memcpy(r, addr.b, 6);
        r[6] = FLOSS_HCI_ERR_REJECTED_LIMITED_RESOURCES;
        floss_hci_cmd(FLOSS_HCI_REJECT_CONNECTION_REQUEST, r, sizeof(r), NULL, NULL);
        return;
    }
    l->cod = cod;
    l->outgoing = false;

    /* Accept even unknown peers when not pairable: the security layer
     * refuses the pairing itself, which hosts handle more gracefully. */
    FLOSS_LOGI(GAP, "accepting connection from %s (CoD 0x%06lX)", s, (unsigned long)cod);
    uint8_t r[7];
    memcpy(r, addr.b, 6);
    r[6] = 0x01; /* remain peripheral: no role switch forced on the host */
    floss_hci_cmd(FLOSS_HCI_ACCEPT_CONNECTION_REQUEST, r, sizeof(r), NULL, NULL);
}

/** Connection Complete: status(1) handle(2) addr(6) link_type(1) encryption(1). */
static void on_connection_complete(const uint8_t *p, uint8_t len)
{
    if (len < 11) {
        return;
    }
    uint8_t status = p[0];
    uint16_t handle = floss_get_le16(&p[1]) & 0x0FFF;
    floss_addr_t addr;
    memcpy(addr.b, &p[3], 6);
    if (p[9] != GAP_LINK_TYPE_ACL) {
        return; /* SCO/eSCO are not handled */
    }

    char s[18];
    floss_addr_to_str(&addr, s);
    floss_gap_link_t *l = floss_gap_link_by_addr(&addr);

    if (status != FLOSS_HCI_SUCCESS) {
        bool ours = l && l->outgoing;
        FLOSS_LOGW(GAP, "connection with %s failed: %s", s, floss_hci_status_str(status));
        if (l) {
            link_free(l);
        }
        if (ours) {
            floss_gap_emit_simple(FLOSS_GAP_EVT_CONNECT_FAILED, FLOSS_GAP_NO_HANDLE, &addr, status);
        }
        return;
    }

    if (!l) {
        l = floss_gap_link_get(&addr); /* e.g. controller auto-accepted */
    }
    if (l) {
        l->has_handle = true;
        l->handle = handle;
    } else {
        FLOSS_LOGE(GAP, "no GAP link slot for handle 0x%03X", handle);
    }
    FLOSS_LOGI(GAP, "connected %s handle 0x%03X (%s)", s, handle, (l && l->outgoing) ? "outgoing" : "incoming");

    send_u16x2(FLOSS_HCI_WRITE_LINK_POLICY_SETTINGS, handle, s_gap.default_policy, NULL);
    floss_l2cap_on_link_up(handle);
    floss_gap_emit_simple(FLOSS_GAP_EVT_CONNECTED, handle, &addr, 0);
}

/** Disconnection Complete: status(1) handle(2) reason(1). */
static void on_disconnection_complete(const uint8_t *p, uint8_t len)
{
    if (len < 4 || p[0] != FLOSS_HCI_SUCCESS) {
        return; /* a failed Disconnect leaves the link up */
    }
    uint16_t handle = floss_get_le16(&p[1]) & 0x0FFF;
    uint8_t reason = p[3];

    floss_conn_t *c = floss_hci_conn_by_handle(handle);
    if (c && c->le) {
        return; /* LE link: HCI notifies the LE layer */
    }
    floss_gap_link_t *l = floss_gap_link_by_handle(handle);
    if (!l && c) {
        l = floss_gap_link_by_addr(&c->addr); /* entry created before the handle was known */
    }
    floss_addr_t addr;
    memset(&addr, 0, sizeof(addr));
    if (c) {
        addr = c->addr;
    } else if (l) {
        addr = l->addr;
    }
    FLOSS_LOGI(GAP, "disconnected handle 0x%03X: %s", handle, floss_hci_status_str(reason));

    floss_l2cap_on_link_down(handle, reason);
    if (l) {
        link_free(l); /* clears pairing state too */
    }
    floss_gap_emit_simple(FLOSS_GAP_EVT_DISCONNECTED, handle, &addr, reason);
}

/** Mode Change: status(1) handle(2) mode(1) interval(2). */
static void on_mode_change(const uint8_t *p, uint8_t len)
{
    if (len < 6) {
        return;
    }
    floss_gap_evt_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = FLOSS_GAP_EVT_MODE_CHANGE;
    evt.status = p[0];
    evt.handle = floss_get_le16(&p[1]) & 0x0FFF;
    evt.mode = p[3];
    evt.interval = floss_get_le16(&p[4]);
    floss_conn_t *c = floss_hci_conn_by_handle(evt.handle);
    if (c) {
        evt.addr = c->addr;
    }
    FLOSS_LOGD(GAP, "handle 0x%03X mode %u interval %u", evt.handle, evt.mode, evt.interval);
    floss_gap_emit(&evt);
}

/** Role Change: status(1) addr(6) role(1). */
static void on_role_change(const uint8_t *p, uint8_t len)
{
    if (len < 8) {
        return;
    }
    floss_gap_evt_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = FLOSS_GAP_EVT_ROLE_CHANGE;
    evt.status = p[0];
    memcpy(evt.addr.b, &p[1], 6);
    evt.mode = p[7];
    evt.handle = floss_gap_handle_for_addr(&evt.addr);
    FLOSS_LOGD(GAP, "role change: %s (%s)", evt.mode ? "peripheral" : "central", floss_hci_status_str(evt.status));
    floss_gap_emit(&evt);
}

void floss_gap_on_hci_event(uint8_t code, const uint8_t *params, uint8_t len)
{
    if (floss_gap_sec_on_event(code, params, len)) {
        return;
    }
    if (floss_gap_inquiry_on_event(code, params, len)) {
        return;
    }
    switch (code) {
    case FLOSS_EVT_CONNECTION_REQUEST:
        on_connection_request(params, len);
        break;
    case FLOSS_EVT_CONNECTION_COMPLETE:
        on_connection_complete(params, len);
        break;
    case FLOSS_EVT_DISCONNECTION_COMPLETE:
        on_disconnection_complete(params, len);
        break;
    case FLOSS_EVT_MODE_CHANGE:
        on_mode_change(params, len);
        break;
    case FLOSS_EVT_ROLE_CHANGE:
        on_role_change(params, len);
        break;
    case FLOSS_EVT_HARDWARE_ERROR:
        FLOSS_LOGE(GAP, "controller hardware error 0x%02X", len ? params[0] : 0);
        floss_gap_emit_simple(FLOSS_GAP_EVT_HARDWARE_ERROR, FLOSS_GAP_NO_HANDLE, NULL, len ? params[0] : 0);
        break;
    default:
        FLOSS_LOGT(GAP, "event 0x%02X ignored", code);
        break;
    }
}
