/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_gap_eir.c - Extended Inquiry Response builder.
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
 *      reserved first because it is small and hosts filter on it.
 *   2. Local name: Complete (0x09) if it fits in what is left, otherwise
 *      Shortened (0x08), cut on a UTF-8 character boundary.
 * The name structure is emitted before the UUID list on the wire (hosts
 * conventionally expect it first); only the space budget is computed with
 * the UUIDs reserved. TX power level is not included.
 *
 * SDP registers its service classes through floss_gap_eir_add_uuid16();
 * the EIR is rebuilt and resent automatically when the controller is up.
 */
#include "floss_gap_internal.h"

#define EIR_TYPE_UUID16_COMPLETE 0x03
#define EIR_TYPE_NAME_SHORT      0x08
#define EIR_TYPE_NAME_COMPLETE   0x09

static struct {
    uint16_t uuid[FLOSS_GAP_EIR_MAX_UUIDS];
    uint8_t n_uuid;
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
                             const uint16_t *uuids, uint8_t n_uuids)
{
    memset(out, 0, cap);

    uint16_t uuid_field = n_uuids ? (uint16_t)(2u + 2u * n_uuids) : 0u;
    if (uuid_field > cap) {
        n_uuids = 0;
        uuid_field = 0;
    }
    uint16_t pos = 0;

    /* Name: whatever is left after reserving the UUID list (2 header bytes). */
    if (name_len > 0 && cap >= uuid_field + 3u) {
        uint16_t room = (uint16_t)(cap - uuid_field - 2u);
        uint8_t type = EIR_TYPE_NAME_COMPLETE;
        uint16_t n = name_len;
        if (n > room) {
            type = EIR_TYPE_NAME_SHORT;
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
        out[pos++] = EIR_TYPE_UUID16_COMPLETE;
        for (uint8_t i = 0; i < n_uuids; i++) {
            floss_put_le16(&out[pos], uuids[i]);
            pos = (uint16_t)(pos + 2u);
        }
    }
    return pos;
}

floss_err_t floss_gap_eir_send(floss_hci_cmd_cb cb, void *ctx)
{
    uint8_t name_len;
    const uint8_t *name = floss_gap_local_name(&name_len);

    s_eir_cmd[0] = 0x01; /* FEC required: more robust, and the data still fits */
    uint16_t used = floss_gap_eir_build(&s_eir_cmd[1], FLOSS_GAP_EIR_LEN, name, name_len, s_eir.uuid, s_eir.n_uuid);
    FLOSS_LOGD(GAP, "EIR: %u bytes, %u uuid(s)", (unsigned)used, (unsigned)s_eir.n_uuid);
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
