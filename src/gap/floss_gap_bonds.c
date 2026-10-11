/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_gap_bonds.c - Persistent Classic bond database (link keys).
 *
 * Design
 * ------
 * FLOSS_MAX_BONDS fixed slots, slot i <-> HAL store key
 * FLOSS_STORE_KEY_BOND_BASE + i. A slot never moves, so updating one bond
 * rewrites exactly one small store value (flash friendly).
 *
 * Recency is a monotonically increasing 32-bit sequence number: every time a
 * bond is created or used for an authenticated reconnect it gets
 * ++global_seq. The "last host" (who a gamepad pages at power-up) is simply
 * the highest seq, and the eviction victim when full is the lowest seq. At
 * one increment per connection a 32-bit counter never wraps in practice.
 *
 * Record layout (little endian, versioned so the format can evolve):
 *
 *     [0]      version (FLOSS_GAP_BOND_REC_VERSION)
 *     [1..6]   peer address (wire order)
 *     [7..22]  link key
 *     [23]     link key type (as reported in Link Key Notification)
 *     [24..26] peer Class of Device
 *     [27..30] seq
 *
 * LE bonds (LTK/IRK) are not stored here; SMP keeps its own records.
 */
#include "floss_gap_internal.h"

static struct {
    floss_bond_t bond[FLOSS_MAX_BONDS];
    bool used[FLOSS_MAX_BONDS];
    uint32_t seq; /* highest seq handed out so far */
} s_bonds;

/* ------------------------------------------------------------------------ */
/* Serialisation                                                            */
/* ------------------------------------------------------------------------ */

uint16_t floss_gap_bond_encode(const floss_bond_t *bond, uint8_t out[FLOSS_GAP_BOND_REC_LEN])
{
    out[0] = FLOSS_GAP_BOND_REC_VERSION;
    memcpy(&out[1], bond->addr.b, 6);
    memcpy(&out[7], bond->key.key, 16);
    out[23] = bond->key.type;
    floss_put_le24(&out[24], bond->cod);
    floss_put_le32(&out[27], bond->seq);
    return FLOSS_GAP_BOND_REC_LEN;
}

bool floss_gap_bond_decode(const uint8_t *in, int len, floss_bond_t *bond)
{
    if (len != (int)FLOSS_GAP_BOND_REC_LEN || in[0] != FLOSS_GAP_BOND_REC_VERSION) {
        return false;
    }
    memcpy(bond->addr.b, &in[1], 6);
    if (floss_addr_is_zero(&bond->addr)) {
        return false;
    }
    memcpy(bond->key.key, &in[7], 16);
    bond->key.type = in[23];
    bond->cod = floss_get_le24(&in[24]);
    bond->seq = floss_get_le32(&in[27]);
    return true;
}

/* ------------------------------------------------------------------------ */
/* Internals                                                                */
/* ------------------------------------------------------------------------ */

static int bond_slot(const floss_addr_t *addr)
{
    for (int i = 0; i < FLOSS_MAX_BONDS; i++) {
        if (s_bonds.used[i] && floss_addr_eq(&s_bonds.bond[i].addr, addr)) {
            return i;
        }
    }
    return -1;
}

static int bond_slot_last(void)
{
    int best = -1;
    for (int i = 0; i < FLOSS_MAX_BONDS; i++) {
        if (s_bonds.used[i] && (best < 0 || s_bonds.bond[i].seq > s_bonds.bond[best].seq)) {
            best = i;
        }
    }
    return best;
}

static void bond_persist(int slot)
{
    uint8_t rec[FLOSS_GAP_BOND_REC_LEN];
    uint16_t n = floss_gap_bond_encode(&s_bonds.bond[slot], rec);
    floss_err_t err = floss_hal_store_write((uint16_t)(FLOSS_STORE_KEY_BOND_BASE + slot), rec, n);
    if (err != FLOSS_OK) {
        FLOSS_LOGW(GAP, "bond slot %d: store write failed (%s)", slot, floss_err_str(err));
    }
}

/** Keep FLOSS_STORE_KEY_LAST_HOST in sync with the most recent bond. */
static void bond_record_last_host(void)
{
    int last = bond_slot_last();
    if (last < 0) {
        floss_hal_store_delete(FLOSS_STORE_KEY_LAST_HOST);
        return;
    }
    floss_hal_store_write(FLOSS_STORE_KEY_LAST_HOST, s_bonds.bond[last].addr.b, 6);
}

/* ------------------------------------------------------------------------ */
/* Internal API                                                             */
/* ------------------------------------------------------------------------ */

void floss_gap_bonds_load(void)
{
    memset(&s_bonds, 0, sizeof(s_bonds));
    for (int i = 0; i < FLOSS_MAX_BONDS; i++) {
        uint8_t rec[FLOSS_GAP_BOND_REC_LEN + 1]; /* +1 detects oversized records */
        int n = floss_hal_store_read((uint16_t)(FLOSS_STORE_KEY_BOND_BASE + i), rec, sizeof(rec));
        if (n <= 0) {
            continue;
        }
        floss_bond_t b;
        if (!floss_gap_bond_decode(rec, n, &b)) {
            /* Unknown version or garbage: leave the slot free so it gets
             * overwritten; do not delete it (a newer firmware may own it). */
            FLOSS_LOGW(GAP, "bond slot %d: unreadable record (len %d) ignored", i, n);
            continue;
        }
        if (bond_slot(&b.addr) >= 0) {
            FLOSS_LOGW(GAP, "bond slot %d: duplicate address ignored", i);
            continue;
        }
        s_bonds.bond[i] = b;
        s_bonds.used[i] = true;
        if (b.seq > s_bonds.seq) {
            s_bonds.seq = b.seq;
        }
    }
    FLOSS_LOGI(GAP, "%u bond(s) loaded", (unsigned)floss_gap_bond_count());
}

const floss_bond_t *floss_gap_bond_save(const floss_addr_t *addr, const floss_link_key_t *key, uint32_t cod)
{
    int slot = bond_slot(addr);
    if (slot < 0) {
        /* Free slot first, otherwise evict the least recently used bond. */
        for (int i = 0; i < FLOSS_MAX_BONDS && slot < 0; i++) {
            if (!s_bonds.used[i]) {
                slot = i;
            }
        }
        if (slot < 0) {
            slot = 0;
            for (int i = 1; i < FLOSS_MAX_BONDS; i++) {
                if (s_bonds.bond[i].seq < s_bonds.bond[slot].seq) {
                    slot = i;
                }
            }
            char s[18];
            floss_addr_to_str(&s_bonds.bond[slot].addr, s);
            FLOSS_LOGI(GAP, "bond table full: evicting %s", s);
        }
    } else if (cod == 0) {
        cod = s_bonds.bond[slot].cod; /* keep a CoD we learned earlier */
    }

    floss_bond_t *b = &s_bonds.bond[slot];
    b->addr = *addr;
    b->key = *key;
    b->cod = cod & 0xFFFFFFu;
    b->seq = ++s_bonds.seq;
    s_bonds.used[slot] = true;

    bond_persist(slot);
    bond_record_last_host();
    return b;
}

void floss_gap_bond_touch(const floss_addr_t *addr)
{
    int slot = bond_slot(addr);
    if (slot < 0 || slot == bond_slot_last()) {
        return; /* unknown, or already the most recent: avoid a flash write */
    }
    s_bonds.bond[slot].seq = ++s_bonds.seq;
    bond_persist(slot);
    bond_record_last_host();
}

/* ------------------------------------------------------------------------ */
/* Public API                                                               */
/* ------------------------------------------------------------------------ */

uint8_t floss_gap_bond_count(void)
{
    uint8_t n = 0;
    for (int i = 0; i < FLOSS_MAX_BONDS; i++) {
        n += s_bonds.used[i] ? 1u : 0u;
    }
    return n;
}

const floss_bond_t *floss_gap_bond_at(uint8_t idx)
{
    for (int i = 0; i < FLOSS_MAX_BONDS; i++) {
        if (s_bonds.used[i]) {
            if (idx == 0) {
                return &s_bonds.bond[i];
            }
            idx--;
        }
    }
    return NULL;
}

const floss_bond_t *floss_gap_bond_last(void)
{
    int slot = bond_slot_last();
    return slot < 0 ? NULL : &s_bonds.bond[slot];
}

const floss_bond_t *floss_gap_bond_find(const floss_addr_t *addr)
{
    if (!addr) {
        return NULL;
    }
    int slot = bond_slot(addr);
    return slot < 0 ? NULL : &s_bonds.bond[slot];
}

floss_err_t floss_gap_bond_delete(const floss_addr_t *addr)
{
    if (!addr) {
        return FLOSS_ERR_INVALID;
    }
    int slot = bond_slot(addr);
    if (slot < 0) {
        return FLOSS_ERR_NOT_FOUND;
    }
    s_bonds.used[slot] = false;
    memset(&s_bonds.bond[slot], 0, sizeof(s_bonds.bond[slot]));
    floss_hal_store_delete((uint16_t)(FLOSS_STORE_KEY_BOND_BASE + slot));
    bond_record_last_host();

    char s[18];
    floss_addr_to_str(addr, s);
    FLOSS_LOGI(GAP, "bond %s deleted", s);
    return FLOSS_OK;
}

floss_err_t floss_gap_bond_delete_all(void)
{
    for (int i = 0; i < FLOSS_MAX_BONDS; i++) {
        floss_hal_store_delete((uint16_t)(FLOSS_STORE_KEY_BOND_BASE + i));
    }
    floss_hal_store_delete(FLOSS_STORE_KEY_LAST_HOST);
    memset(&s_bonds, 0, sizeof(s_bonds));
    FLOSS_LOGI(GAP, "all bonds deleted");
    return FLOSS_OK;
}
