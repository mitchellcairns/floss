/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_util.c - Small helpers declared in floss_types.h: error names and
 * device address formatting/parsing.
 *
 * Overview
 * --------
 * Device addresses travel over HCI least-significant byte first, which is how
 * floss_addr_t stores them. Humans (and every tool that prints addresses)
 * expect the most-significant byte first, so the string helpers reverse the
 * byte order. Nothing here touches the HAL, so these are safe anywhere.
 */
#include "floss/floss_types.h"

const char *floss_err_str(floss_err_t err)
{
    switch (err) {
    case FLOSS_OK:              return "OK";
    case FLOSS_ERR_NOMEM:       return "NOMEM";
    case FLOSS_ERR_BUSY:        return "BUSY";
    case FLOSS_ERR_INVALID:     return "INVALID";
    case FLOSS_ERR_STATE:       return "STATE";
    case FLOSS_ERR_NOT_FOUND:   return "NOT_FOUND";
    case FLOSS_ERR_TIMEOUT:     return "TIMEOUT";
    case FLOSS_ERR_IO:          return "IO";
    case FLOSS_ERR_UNSUPPORTED: return "UNSUPPORTED";
    case FLOSS_ERR_PROTOCOL:    return "PROTOCOL";
    default:                    return "UNKNOWN";
    }
}

void floss_addr_to_str(const floss_addr_t *addr, char out[18])
{
    static const char hex[] = "0123456789ABCDEF";
    char *p = out;

    /* Wire order is LSB first; print MSB first. */
    for (int i = 5; i >= 0; i--) {
        *p++ = hex[addr->b[i] >> 4];
        *p++ = hex[addr->b[i] & 0x0F];
        *p++ = (i > 0) ? ':' : '\0';
    }
}

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

/**
 * Accepts exactly "XX:XX:XX:XX:XX:XX" (either case, ':' or '-' separators).
 * The output is only written when the whole string is valid, so callers can
 * pass their live address without risking a half-parsed result.
 */
bool floss_addr_from_str(const char *str, floss_addr_t *out)
{
    floss_addr_t tmp;

    if (str == NULL || out == NULL) {
        return false;
    }
    for (int i = 0; i < 6; i++) {
        int hi = hex_nibble(str[0]);
        int lo = (hi < 0) ? -1 : hex_nibble(str[1]);
        if (lo < 0) {
            return false;
        }
        tmp.b[5 - i] = (uint8_t)((hi << 4) | lo);
        str += 2;
        if (i < 5) {
            if (*str != ':' && *str != '-') {
                return false;
            }
            str++;
        }
    }
    if (*str != '\0') {
        return false;
    }
    *out = tmp;
    return true;
}
