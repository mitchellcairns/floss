/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_gap_eir.c - Extended Inquiry Response builder and parser.
 *
 * Design
 * ------
 * The EIR is what a host sees in its "add device" list before connecting:
 * the name and the service UUIDs, without an extra Remote Name Request or
 * SDP round trip. It is 240 bytes of [len][type][data...] structures,
 * zero padded (a zero length terminates the significant part).
 *
 * Layout chosen here:
 *   1. Complete List of 16-bit Service UUIDs (0x03), if any were added;
 *   2. Complete List of 128-bit Service UUIDs (0x07), if any were added;
 *      both reserved first because they are small and hosts filter on them.
 *   3. Local name: Complete (0x09) if it fits in what is left, otherwise
 *      Shortened (0x08), cut on a UTF-8 character boundary.
 * The name structure is emitted before the UUID lists on the wire (hosts
 * conventionally expect it first); only the space budget is computed with
 * the UUIDs reserved. TX power level is not included.
 *
 * SDP registers its service classes through floss_gap_eir_add_uuid16();
 * profiles with their own 128-bit service (the pad <-> dongle link) use
 * floss_gap_eir_add_uuid128(). The EIR is rebuilt and resent automatically
 * when the controller is up.
 *
 * The parser (floss_gap_eir_find) reads what other devices send us in
 * Extended Inquiry Results; LE advertising data uses the same format.
 */
#include "floss_gap_internal.h"

static struct {
    uint16_t uuid[FLOSS_GAP_EIR_MAX_UUIDS];
    uint8_t n_uuid;
    uint8_t uuid128[FLOSS_GAP_EIR_MAX_UUID128][16]; /* little-endian (wire order) */
    uint8_t n_uuid128;
} s_eir;

/* Scratch for the HCI command (FEC byte + 240). Params are copied by floss_hci_cmd. */
static uint8_t s_eir_cmd[1 + FLOSS_GAP_EIR_LEN];

void floss_gap_eir_init(void)
{
    memset(&s_eir, 0, sizeof(s_eir));
}

/** Largest n <= max such that name[0..n) does not end inside a UTF-8 sequence. */
static uint16_t utf8_trim(const uint8_t *name, uint16_t max)
{
    uint16_t n = max;
    /* Back up over continuation bytes (10xxxxxx) to the lead byte... */
    while (n > 0 && (name[n] & 0xC0u) == 0x80u) {
        n--;
    }
    return n; /* ...and cut before it: name[n] starts the character we drop. */
}

uint16_t floss_gap_eir_build(uint8_t *out, uint16_t cap, const uint8_t *name, uint16_t name_len,
                             const uint16_t *uuids, uint8_t n_uuids, const uint8_t *uuid128, uint8_t n_uuid128)
{
    memset(out, 0, cap);

    uint16_t uuid_field = n_uuids ? (uint16_t)(2u + 2u * n_uuids) : 0u;
    if (uuid_field > cap) {
        n_uuids = 0;
        uuid_field = 0;
    }
    uint16_t uuid128_field = n_uuid128 ? (uint16_t)(2u + 16u * n_uuid128) : 0u;
    if ((uint32_t)uuid_field + uuid128_field > cap) {
        n_uuid128 = 0;
        uuid128_field = 0;
    }
    uint16_t reserved = (uint16_t)(uuid_field + uuid128_field);
    uint16_t pos = 0;

    /* Name: whatever is left after reserving the UUID lists (2 header bytes). */
    if (name_len > 0 && cap >= reserved + 3u) {
        uint16_t room = (uint16_t)(cap - reserved - 2u);
        uint8_t type = FLOSS_EIR_NAME_COMPLETE;
        uint16_t n = name_len;
        if (n > room) {
            type = FLOSS_EIR_NAME_SHORT;
            n = utf8_trim(name, room);
        }
        if (n > 0) {
            out[pos++] = (uint8_t)(n + 1u);
            out[pos++] = type;
            memcpy(&out[pos], name, n);
            pos = (uint16_t)(pos + n);
        }
    }

    if (n_uuids) {
        out[pos++] = (uint8_t)(1u + 2u * n_uuids);
        out[pos++] = FLOSS_EIR_UUID16_ALL;
        for (uint8_t i = 0; i < n_uuids; i++) {
            floss_put_le16(&out[pos], uuids[i]);
            pos = (uint16_t)(pos + 2u);
        }
    }
    if (n_uuid128) {
        out[pos++] = (uint8_t)(1u + 16u * n_uuid128);
        out[pos++] = FLOSS_EIR_UUID128_ALL;
        memcpy(&out[pos], uuid128, 16u * n_uuid128);
        pos = (uint16_t)(pos + 16u * n_uuid128);
    }
    return pos;
}

floss_err_t floss_gap_eir_send(floss_hci_cmd_cb cb, void *ctx)
{
    uint8_t name_len;
    const uint8_t *name = floss_gap_local_name(&name_len);

    s_eir_cmd[0] = 0x01; /* FEC required: more robust, and the data still fits */
    uint16_t used = floss_gap_eir_build(&s_eir_cmd[1], FLOSS_GAP_EIR_LEN, name, name_len, s_eir.uuid, s_eir.n_uuid,
                                        &s_eir.uuid128[0][0], s_eir.n_uuid128);
    FLOSS_LOGD(GAP, "EIR: %u bytes, %u uuid16, %u uuid128", (unsigned)used, (unsigned)s_eir.n_uuid,
               (unsigned)s_eir.n_uuid128);
    return floss_hci_cmd(FLOSS_HCI_WRITE_EXTENDED_INQUIRY_RESPONSE, s_eir_cmd, sizeof(s_eir_cmd), cb, ctx);
}

floss_err_t floss_gap_update_eir(void)
{
    if (!floss_gap_hci_live()) {
        return FLOSS_OK; /* applied by the bring-up sequence */
    }
    return floss_gap_eir_send(NULL, NULL);
}

floss_err_t floss_gap_eir_add_uuid16(uint16_t uuid)
{
    for (uint8_t i = 0; i < s_eir.n_uuid; i++) {
        if (s_eir.uuid[i] == uuid) {
            return FLOSS_OK;
        }
    }
    if (s_eir.n_uuid >= FLOSS_GAP_EIR_MAX_UUIDS) {
        FLOSS_LOGW(GAP, "EIR UUID list full, 0x%04X not advertised", uuid);
        return FLOSS_ERR_NOMEM;
    }
    s_eir.uuid[s_eir.n_uuid++] = uuid;
    return floss_gap_update_eir();
}

floss_err_t floss_gap_eir_add_uuid128(const uint8_t uuid_be[16])
{
    if (!uuid_be) {
        return FLOSS_ERR_INVALID;
    }
    uint8_t le[16];
    for (unsigned i = 0; i < 16; i++) {
        le[i] = uuid_be[15 - i]; /* EIR carries UUIDs least significant byte first */
    }
    for (uint8_t i = 0; i < s_eir.n_uuid128; i++) {
        if (memcmp(s_eir.uuid128[i], le, 16) == 0) {
            return FLOSS_OK;
        }
    }
    if (s_eir.n_uuid128 >= FLOSS_GAP_EIR_MAX_UUID128) {
        FLOSS_LOGW(GAP, "EIR 128-bit UUID list full");
        return FLOSS_ERR_NOMEM;
    }
    memcpy(s_eir.uuid128[s_eir.n_uuid128++], le, 16);
    return floss_gap_update_eir();
}

/* ------------------------------------------------------------------------ */
/* Parsing (EIR received from other devices; also LE advertising data)      */
/* ------------------------------------------------------------------------ */

const uint8_t *floss_gap_eir_find(const uint8_t *eir, uint8_t len, uint8_t type, uint8_t *out_len)
{
    if (!eir) {
        return NULL;
    }
    uint16_t pos = 0;
    while (pos < len) {
        uint8_t flen = eir[pos];
        if (flen == 0) {
            break; /* end of the significant part (zero padding) */
        }
        if ((uint16_t)(pos + 1u + flen) > len) {
            break; /* structure runs past the end: malformed, stop here */
        }
        if (eir[pos + 1u] == type) {
            if (out_len) {
                *out_len = (uint8_t)(flen - 1u);
            }
            return &eir[pos + 2u];
        }
        pos = (uint16_t)(pos + 1u + flen);
    }
    return NULL;
}

bool floss_gap_eir_has_uuid128(const uint8_t *eir, uint8_t len, const uint8_t uuid_be[16])
{
    static const uint8_t types[2] = {FLOSS_EIR_UUID128_ALL, FLOSS_EIR_UUID128_SOME};
    for (unsigned t = 0; t < 2; t++) {
        uint8_t n = 0;
        const uint8_t *p = floss_gap_eir_find(eir, len, types[t], &n);
        for (uint16_t off = 0; p && off + 16u <= n; off = (uint16_t)(off + 16u)) {
            bool same = true;
            for (unsigned i = 0; i < 16 && same; i++) {
                same = p[off + i] == uuid_be[15 - i];
            }
            if (same) {
                return true;
            }
        }
    }
    return false;
}
