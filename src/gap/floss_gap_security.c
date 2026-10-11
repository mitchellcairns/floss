/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_gap_security.c - Classic pairing, authentication and encryption.
 *
 * Design
 * ------
 * The controller runs every cryptographic protocol (Secure Simple Pairing,
 * legacy PIN pairing, authentication challenge, encryption start). The host
 * only answers its questions:
 *
 *   Link Key Request        -> "here is the key I stored for this peer" / "none"
 *   IO Capability Request   -> our IO capability + authentication requirements
 *   User Confirmation Req.  -> yes/no to the numeric comparison
 *   PIN Code Request        -> legacy PIN (pre-2.1 hosts)
 *   Link Key Notification   -> new key: store it (bonding)
 *
 * A typical first pairing with a host (peripheral side, host initiates):
 *
 *   host connects -> Link Key Request (no bond: negative reply)
 *   -> IO Capability Request/Response -> User Confirmation Request (auto yes)
 *   -> Simple Pairing Complete -> Link Key Notification (bond stored)
 *   -> Encryption Change (on) -> L2CAP lets HID channels open.
 *
 * A reconnect is just Link Key Request (reply with the bond) -> Encryption
 * Change. If the host deleted its key, authentication fails with "PIN or Key
 * Missing": we drop our stale bond so the next attempt pairs from scratch
 * instead of failing forever.
 *
 * When L2CAP needs a secure link (floss_gap_require_security) and the link
 * is already encrypted, the answer is posted through a zero-delay timer so
 * L2CAP never gets a callback from inside its own call.
 */
#include "floss_gap_internal.h"

/* Authentication requirements (IO Capability Request Reply). */
#define AUTH_REQ_NO_BONDING          0x00
#define AUTH_REQ_NO_BONDING_MITM     0x01
#define AUTH_REQ_GENERAL_BONDING     0x04
#define AUTH_REQ_GENERAL_BONDING_MITM 0x05

#define LEGACY_PIN_MAX 16u

static struct {
    floss_gap_security_t cfg;
    char pin[LEGACY_PIN_MAX + 1]; /* our copy; cfg.legacy_pin points here or is NULL */
} s_sec;

/* ======================================================================== */
/* Settings                                                                 */
/* ======================================================================== */

void floss_gap_security_defaults(floss_gap_security_t *sec)
{
    if (!sec) {
        return;
    }
    memset(sec, 0, sizeof(*sec));
    sec->io_capability = FLOSS_IO_CAP_NO_INPUT_NO_OUTPUT;
    sec->bondable = true;
    sec->mitm_required = false;
    sec->auto_accept_confirm = true;
    sec->secure_connections = true;
    sec->legacy_pin = "0000";
    sec->pairable = true;
}

void floss_gap_sec_init(void)
{
    memset(&s_sec, 0, sizeof(s_sec));
    floss_gap_security_defaults(&s_sec.cfg);
    memcpy(s_sec.pin, "0000", 5);
    s_sec.cfg.legacy_pin = s_sec.pin;
}

bool floss_gap_sec_want_sc(void)
{
    return s_sec.cfg.secure_connections;
}

floss_err_t floss_gap_set_security(const floss_gap_security_t *sec)
{
    if (!sec || sec->io_capability > FLOSS_IO_CAP_NO_INPUT_NO_OUTPUT) {
        return FLOSS_ERR_INVALID;
    }
    size_t pin_len = sec->legacy_pin ? strlen(sec->legacy_pin) : 0;
    if (pin_len > LEGACY_PIN_MAX || (sec->legacy_pin && pin_len == 0)) {
        return FLOSS_ERR_INVALID;
    }
    bool sc_changed = sec->secure_connections != s_sec.cfg.secure_connections;

    s_sec.cfg = *sec;
    memset(s_sec.pin, 0, sizeof(s_sec.pin));
    if (sec->legacy_pin) {
        memcpy(s_sec.pin, sec->legacy_pin, pin_len);
        s_sec.cfg.legacy_pin = s_sec.pin;
    }

    /* SC host support is the only security setting the controller holds.
     * The controller only honours a change while no link exists. */
    if (sc_changed && floss_gap_hci_live() && floss_hci_cmd_supported(32, 3)) {
        uint8_t v = sec->secure_connections ? 1 : 0;
        return floss_hci_cmd(FLOSS_HCI_WRITE_SECURE_CONNECTIONS_HOST_SUPPORT, &v, 1, NULL, NULL);
    }
    return FLOSS_OK;
}

/** New pairings need `pairable`; peers we already bonded with may always re-pair. */
static bool pairing_allowed(const floss_addr_t *addr)
{
    return s_sec.cfg.pairable || floss_gap_bond_find(addr) != NULL;
}

/* ======================================================================== */
/* Small command helpers                                                    */
/* ======================================================================== */

static void cmd_addr(uint16_t opcode, const floss_addr_t *addr)
{
    floss_hci_cmd(opcode, addr->b, 6, NULL, NULL);
}

static void cmd_addr_u8(uint16_t opcode, const floss_addr_t *addr, uint8_t v)
{
    uint8_t p[7];
    memcpy(p, addr->b, 6);
    p[6] = v;
    floss_hci_cmd(opcode, p, sizeof(p), NULL, NULL);
}

static floss_conn_t *classic_conn(uint16_t handle)
{
    floss_conn_t *c = floss_hci_conn_by_handle(handle);
    return (c && !c->le) ? c : NULL;
}

/** Command Status of Authentication Requested / Set Connection Encryption. */
static void sec_cmd_done(void *ctx, uint16_t opcode, uint8_t status, const uint8_t *ret, uint16_t ret_len)
{
    if (status == FLOSS_HCI_SUCCESS) {
        return; /* running; Authentication Complete / Encryption Change follows */
    }
    uint16_t handle = (uint16_t)(uintptr_t)ctx;
    FLOSS_LOGW(GAP, "handle 0x%03X: security command 0x%04X failed: %s", handle, opcode,
               floss_hci_status_str(status));
    floss_gap_link_t *l = floss_gap_link_by_handle(handle);
    if (l) {
        l->auth_pending = false;
        l->encrypt_pending = false;
    }
    floss_l2cap_on_link_security(handle, status, false);
}

static floss_err_t start_encryption(floss_gap_link_t *l)
{
    uint8_t p[3];
    floss_put_le16(p, l->handle);
    p[2] = 0x01;
    floss_err_t err = floss_hci_cmd(FLOSS_HCI_SET_CONNECTION_ENCRYPTION, p, sizeof(p), sec_cmd_done,
                                    (void *)(uintptr_t)l->handle);
    if (err == FLOSS_OK) {
        l->encrypt_pending = true;
    }
    return err;
}

/* ======================================================================== */
/* Securing a link (L2CAP / application request)                            */
/* ======================================================================== */

/** Retry delay when HCI's command queue is momentarily full. */
#define SEC_RETRY_US 1000u

static floss_err_t secure_link(uint16_t handle);

/**
 * sec_timer: either reports an already-encrypted link to L2CAP (outside
 * L2CAP's own call stack), or retries a security request that could not be
 * queued.
 */
void floss_gap_sec_timer_fn(floss_timer_t *t, void *arg)
{
    floss_gap_link_t *l = (floss_gap_link_t *)arg;
    if (!l->used || !l->has_handle) {
        return;
    }
    floss_conn_t *c = classic_conn(l->handle);
    if (!c) {
        return;
    }
    if (c->encrypted) {
        floss_l2cap_on_link_security(l->handle, FLOSS_HCI_SUCCESS, true);
    } else if (secure_link(l->handle) != FLOSS_OK) {
        floss_timer_start(&l->sec_timer, SEC_RETRY_US);
    }
}

/**
 * Bring a link to "authenticated + encrypted". Idempotent: requests already
 * in flight are not repeated; an already-secure link is reported (deferred).
 */
static floss_err_t secure_link(uint16_t handle)
{
    floss_conn_t *c = classic_conn(handle);
    if (!c) {
        return FLOSS_ERR_NOT_FOUND;
    }
    floss_gap_link_t *l = floss_gap_link_by_handle(handle);
    if (!l) {
        l = floss_gap_link_get(&c->addr);
        if (!l) {
            return FLOSS_ERR_NOMEM;
        }
        l->has_handle = true;
        l->handle = handle;
    }

    if (c->encrypted) {
        floss_timer_start(&l->sec_timer, 0);
        return FLOSS_OK;
    }
    if (l->auth_pending || l->encrypt_pending) {
        return FLOSS_OK;
    }
    if (l->authenticated || c->authenticated) {
        return start_encryption(l);
    }

    uint8_t p[2];
    floss_put_le16(p, handle);
    floss_err_t err = floss_hci_cmd(FLOSS_HCI_AUTHENTICATION_REQUESTED, p, sizeof(p), sec_cmd_done,
                                    (void *)(uintptr_t)handle);
    if (err == FLOSS_OK) {
        l->auth_pending = true;
        FLOSS_LOGD(GAP, "handle 0x%03X: authentication requested", handle);
    }
    return err;
}

void floss_gap_require_security(uint16_t handle)
{
    floss_err_t err = secure_link(handle);
    if (err == FLOSS_OK) {
        return;
    }
    floss_gap_link_t *l = floss_gap_link_by_handle(handle);
    if (l) {
        /* Command queue full: retry shortly rather than failing the channel. */
        FLOSS_LOGD(GAP, "handle 0x%03X: security request deferred (%s)", handle, floss_err_str(err));
        floss_timer_start(&l->sec_timer, SEC_RETRY_US);
    } else {
        FLOSS_LOGW(GAP, "handle 0x%03X: cannot secure link (%s)", handle, floss_err_str(err));
    }
}

floss_err_t floss_gap_authenticate(uint16_t handle)
{
    return secure_link(handle);
}

floss_err_t floss_gap_user_confirm(uint16_t handle, bool accept)
{
    floss_gap_link_t *l = floss_gap_link_by_handle(handle);
    if (!l) {
        return FLOSS_ERR_NOT_FOUND;
    }
    if (!l->confirm_pending) {
        return FLOSS_ERR_STATE;
    }
    l->confirm_pending = false;
    uint16_t op = accept ? FLOSS_HCI_USER_CONFIRM_REQUEST_REPLY : FLOSS_HCI_USER_CONFIRM_REQUEST_NEG_REPLY;
    return floss_hci_cmd(op, l->addr.b, 6, NULL, NULL);
}

/* ======================================================================== */
/* Pairing events (all start with, or carry, the peer address)              */
/* ======================================================================== */

/** Link Key Request: addr(6). */
static void on_link_key_request(const floss_addr_t *addr)
{
    const floss_bond_t *b = floss_gap_bond_find(addr);
    const floss_link_key_t *key = b ? &b->key : NULL;
    floss_gap_link_t *l = floss_gap_link_by_addr(addr);
    if (!key && l && l->have_session_key) {
        key = &l->session_key; /* non-bondable pairing earlier on this link */
    }
    if (!key) {
        FLOSS_LOGD(GAP, "link key request: no key");
        cmd_addr(FLOSS_HCI_LINK_KEY_REQUEST_NEG_REPLY, addr);
        return;
    }
    FLOSS_LOGD(GAP, "link key request: using stored key (type %u)", key->type);
    uint8_t p[22];
    memcpy(&p[0], addr->b, 6);
    memcpy(&p[6], key->key, 16);
    floss_hci_cmd(FLOSS_HCI_LINK_KEY_REQUEST_REPLY, p, sizeof(p), NULL, NULL);
}

/** Link Key Notification: addr(6) key(16) type(1). */
static void on_link_key_notification(const floss_addr_t *addr, const uint8_t *p, uint8_t len)
{
    if (len < 23) {
        return;
    }
    floss_link_key_t key;
    memcpy(key.key, &p[6], 16);
    key.type = p[22];
    if (key.type == 0x03) {
        FLOSS_LOGW(GAP, "peer uses a DEBUG link key (SSP debug mode): link is not private");
    }

    floss_gap_link_t *l = floss_gap_link_for_event(addr);
    if (l) {
        l->pairing = false;
        l->session_key = key;
        l->have_session_key = true;
    }
    /* Replace an existing bond even when not bondable: the old key is now stale. */
    if (s_sec.cfg.bondable || floss_gap_bond_find(addr)) {
        floss_gap_bond_save(addr, &key, l ? l->cod : 0);
        FLOSS_LOGI(GAP, "bonded (key type %u)", key.type);
    }
    floss_gap_emit_simple(FLOSS_GAP_EVT_PAIRING_COMPLETE, floss_gap_handle_for_addr(addr), addr, 0);
}

/** IO Capability Request: addr(6). */
static void on_io_cap_request(const floss_addr_t *addr)
{
    if (!pairing_allowed(addr)) {
        FLOSS_LOGI(GAP, "pairing refused: not pairable");
        cmd_addr_u8(FLOSS_HCI_IO_CAPABILITY_REQUEST_NEG_REPLY, addr, FLOSS_HCI_ERR_PAIRING_NOT_ALLOWED);
        return;
    }
    const floss_gap_security_t *cfg = &s_sec.cfg;
    uint8_t auth = cfg->bondable ? (cfg->mitm_required ? AUTH_REQ_GENERAL_BONDING_MITM : AUTH_REQ_GENERAL_BONDING)
                                 : (cfg->mitm_required ? AUTH_REQ_NO_BONDING_MITM : AUTH_REQ_NO_BONDING);
    uint8_t p[9];
    memcpy(p, addr->b, 6);
    p[6] = cfg->io_capability;
    p[7] = 0x00; /* no OOB data */
    p[8] = auth;
    floss_hci_cmd(FLOSS_HCI_IO_CAPABILITY_REQUEST_REPLY, p, sizeof(p), NULL, NULL);

    floss_gap_link_t *l = floss_gap_link_for_event(addr);
    if (l) {
        l->pairing = true;
        l->pairing_fail_reported = false;
    }
    FLOSS_LOGI(GAP, "pairing: io cap %u auth req 0x%02X", cfg->io_capability, auth);
    floss_gap_emit_simple(FLOSS_GAP_EVT_PAIRING_STARTED, floss_gap_handle_for_addr(addr), addr, 0);
}

/** User Confirmation Request: addr(6) numeric value(4). */
static void on_user_confirm_request(const floss_addr_t *addr, const uint8_t *p, uint8_t len)
{
    uint32_t value = len >= 10 ? floss_get_le32(&p[6]) : 0;
    if (s_sec.cfg.auto_accept_confirm || s_sec.cfg.io_capability == FLOSS_IO_CAP_NO_INPUT_NO_OUTPUT) {
        /* Just Works (or the app opted out of confirming): accept. */
        cmd_addr(FLOSS_HCI_USER_CONFIRM_REQUEST_REPLY, addr);
        return;
    }
    floss_gap_link_t *l = floss_gap_link_for_event(addr);
    if (!l) {
        cmd_addr(FLOSS_HCI_USER_CONFIRM_REQUEST_NEG_REPLY, addr);
        return;
    }
    l->confirm_pending = true;
    floss_gap_evt_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = FLOSS_GAP_EVT_USER_CONFIRM;
    evt.handle = floss_gap_handle_for_addr(addr);
    evt.addr = *addr;
    evt.passkey = value;
    floss_gap_emit(&evt);
}

/** User Passkey Notification: addr(6) passkey(4). */
static void on_passkey_notification(const floss_addr_t *addr, const uint8_t *p, uint8_t len)
{
    if (len < 10) {
        return;
    }
    floss_gap_evt_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = FLOSS_GAP_EVT_PASSKEY_DISPLAY;
    evt.handle = floss_gap_handle_for_addr(addr);
    evt.addr = *addr;
    evt.passkey = floss_get_le32(&p[6]);
    floss_gap_emit(&evt);
}

/** PIN Code Request: addr(6). Legacy (pre Simple Pairing) hosts only. */
static void on_pin_code_request(const floss_addr_t *addr)
{
    const char *pin = s_sec.cfg.legacy_pin;
    if (!pin || !pairing_allowed(addr)) {
        FLOSS_LOGI(GAP, "legacy PIN pairing refused");
        cmd_addr(FLOSS_HCI_PIN_CODE_REQUEST_NEG_REPLY, addr);
        return;
    }
    uint8_t p[23];
    memset(p, 0, sizeof(p));
    memcpy(&p[0], addr->b, 6);
    p[6] = (uint8_t)strlen(pin);
    memcpy(&p[7], pin, p[6]);
    floss_hci_cmd(FLOSS_HCI_PIN_CODE_REQUEST_REPLY, p, sizeof(p), NULL, NULL);

    floss_gap_link_t *l = floss_gap_link_for_event(addr);
    if (l) {
        l->pairing = true;
        l->pairing_fail_reported = false;
    }
    FLOSS_LOGI(GAP, "legacy PIN pairing");
    floss_gap_emit_simple(FLOSS_GAP_EVT_PAIRING_STARTED, floss_gap_handle_for_addr(addr), addr, 0);
}

/** Simple Pairing Complete: status(1) addr(6). */
static void on_simple_pairing_complete(const uint8_t *p, uint8_t len)
{
    if (len < 7) {
        return;
    }
    uint8_t status = p[0];
    floss_addr_t addr;
    memcpy(addr.b, &p[1], 6);
    floss_gap_link_t *l = floss_gap_link_by_addr(&addr);
    if (l) {
        l->confirm_pending = false;
    }
    if (status == FLOSS_HCI_SUCCESS) {
        FLOSS_LOGD(GAP, "simple pairing complete");
        return; /* PAIRING_COMPLETE is emitted with the link key */
    }
    FLOSS_LOGW(GAP, "simple pairing failed: %s", floss_hci_status_str(status));
    if (l) {
        l->pairing = false;
        l->pairing_fail_reported = true;
    }
    floss_gap_emit_simple(FLOSS_GAP_EVT_PAIRING_FAILED, floss_gap_handle_for_addr(&addr), &addr, status);
}

/* ======================================================================== */
/* Authentication / encryption results                                      */
/* ======================================================================== */

/** Authentication Complete: status(1) handle(2). */
static void on_auth_complete(const uint8_t *p, uint8_t len)
{
    if (len < 3) {
        return;
    }
    uint8_t status = p[0];
    uint16_t handle = floss_get_le16(&p[1]) & 0x0FFF;
    floss_conn_t *c = classic_conn(handle);
    floss_gap_link_t *l = floss_gap_link_by_handle(handle);
    floss_addr_t addr;
    memset(&addr, 0, sizeof(addr));
    if (c) {
        addr = c->addr;
    } else if (l) {
        addr = l->addr;
    }
    if (l) {
        l->auth_pending = false;
        l->pairing = false;
    }

    if (status == FLOSS_HCI_SUCCESS) {
        FLOSS_LOGI(GAP, "handle 0x%03X authenticated", handle);
        if (l) {
            l->authenticated = true;
        }
        if (c && c->encrypted) {
            /* Re-authentication of an encrypted link: nothing more to do. */
            floss_l2cap_on_link_security(handle, FLOSS_HCI_SUCCESS, true);
        } else if (l && !l->encrypt_pending) {
            if (start_encryption(l) != FLOSS_OK) {
                floss_l2cap_on_link_security(handle, FLOSS_HCI_ERR_UNSPECIFIED, false);
            }
        }
        floss_gap_emit_simple(FLOSS_GAP_EVT_AUTHENTICATED, handle, &addr, status);
        return;
    }

    FLOSS_LOGW(GAP, "handle 0x%03X authentication failed: %s", handle, floss_hci_status_str(status));
    if ((status == FLOSS_HCI_ERR_PIN_OR_KEY_MISSING || status == FLOSS_HCI_ERR_AUTH_FAILURE) &&
        floss_gap_bond_find(&addr)) {
        /* The host no longer has our key: forget ours so the next attempt pairs afresh. */
        floss_gap_bond_delete(&addr);
    }
    if (l) {
        l->have_session_key = false;
    }
    if (!l || !l->pairing_fail_reported) {
        floss_gap_emit_simple(FLOSS_GAP_EVT_PAIRING_FAILED, handle, &addr, status);
    }
    if (l) {
        l->pairing_fail_reported = false;
    }
    floss_l2cap_on_link_security(handle, status, false);
    floss_gap_emit_simple(FLOSS_GAP_EVT_AUTHENTICATED, handle, &addr, status);
}

/**
 * Encryption Change: status(1) handle(2) enabled(1).
 * Encryption Key Refresh Complete: status(1) handle(2).
 */
static void on_encryption(uint8_t code, const uint8_t *p, uint8_t len)
{
    if (len < 3) {
        return;
    }
    uint8_t status = p[0];
    uint16_t handle = floss_get_le16(&p[1]) & 0x0FFF;
    floss_conn_t *c = floss_hci_conn_by_handle(handle);
    if (c && c->le) {
        return; /* LE link: HCI routes it to the LE layer */
    }

    bool encrypted;
    if (status != FLOSS_HCI_SUCCESS) {
        encrypted = c ? c->encrypted : false;
    } else if (code == FLOSS_EVT_ENCRYPTION_CHANGE) {
        encrypted = len >= 4 && p[3] != 0;
    } else {
        encrypted = true; /* key refresh only happens on encrypted links */
    }

    floss_gap_link_t *l = floss_gap_link_by_handle(handle);
    if (l) {
        l->encrypt_pending = false;
        l->auth_pending = false;
        if (encrypted) {
            l->authenticated = true; /* encryption implies an authenticated link key */
        }
    }
    FLOSS_LOGI(GAP, "handle 0x%03X encryption %s (%s)", handle, encrypted ? "on" : "off",
               floss_hci_status_str(status));

    /* A successful encrypted reconnect makes this host the one to page next. */
    if (status == FLOSS_HCI_SUCCESS && encrypted && c) {
        floss_gap_bond_touch(&c->addr);
    }

    floss_l2cap_on_link_security(handle, status, encrypted);

    floss_gap_evt_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = FLOSS_GAP_EVT_ENCRYPTION_CHANGE;
    evt.handle = handle;
    if (c) {
        evt.addr = c->addr;
    }
    evt.status = status;
    evt.encrypted = encrypted;
    floss_gap_emit(&evt);
}

/* ======================================================================== */
/* Dispatch                                                                 */
/* ======================================================================== */

bool floss_gap_sec_on_event(uint8_t code, const uint8_t *params, uint8_t len)
{
    floss_addr_t addr;

    switch (code) {
    case FLOSS_EVT_AUTHENTICATION_COMPLETE:
        on_auth_complete(params, len);
        return true;
    case FLOSS_EVT_ENCRYPTION_CHANGE:
    case FLOSS_EVT_ENCRYPTION_KEY_REFRESH_COMPLETE:
        on_encryption(code, params, len);
        return true;
    case FLOSS_EVT_SIMPLE_PAIRING_COMPLETE:
        on_simple_pairing_complete(params, len);
        return true;
    case FLOSS_EVT_LINK_KEY_REQUEST:
    case FLOSS_EVT_LINK_KEY_NOTIFICATION:
    case FLOSS_EVT_IO_CAPABILITY_REQUEST:
    case FLOSS_EVT_IO_CAPABILITY_RESPONSE:
    case FLOSS_EVT_USER_CONFIRMATION_REQUEST:
    case FLOSS_EVT_USER_PASSKEY_REQUEST:
    case FLOSS_EVT_USER_PASSKEY_NOTIFICATION:
    case FLOSS_EVT_PIN_CODE_REQUEST:
        break; /* address-first events, below */
    default:
        return false;
    }

    if (len < 6) {
        return true;
    }
    memcpy(addr.b, params, 6);

    switch (code) {
    case FLOSS_EVT_LINK_KEY_REQUEST:
        on_link_key_request(&addr);
        break;
    case FLOSS_EVT_LINK_KEY_NOTIFICATION:
        on_link_key_notification(&addr, params, len);
        break;
    case FLOSS_EVT_IO_CAPABILITY_REQUEST:
        on_io_cap_request(&addr);
        break;
    case FLOSS_EVT_IO_CAPABILITY_RESPONSE:
        if (len >= 9) {
            FLOSS_LOGI(GAP, "peer io cap %u oob %u auth req 0x%02X", params[6], params[7], params[8]);
        }
        break;
    case FLOSS_EVT_USER_CONFIRMATION_REQUEST:
        on_user_confirm_request(&addr, params, len);
        break;
    case FLOSS_EVT_USER_PASSKEY_REQUEST:
        /* We would have to type the passkey shown by the host. FLOSS devices
         * advertise no keyboard, so a controller only asks this when the app
         * configured KEYBOARD_ONLY.
         * TODO: emit an event + floss_gap_passkey_reply() for keyboard devices. */
        FLOSS_LOGW(GAP, "passkey entry not supported: rejecting");
        cmd_addr(FLOSS_HCI_USER_PASSKEY_REQUEST_NEG_REPLY, &addr);
        break;
    case FLOSS_EVT_USER_PASSKEY_NOTIFICATION:
        on_passkey_notification(&addr, params, len);
        break;
    case FLOSS_EVT_PIN_CODE_REQUEST:
        on_pin_code_request(&addr);
        break;
    default:
        break;
    }
    return true;
}
