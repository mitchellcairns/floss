/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * link_diag.c - Radio diagnostics for the pad/dongle firmwares.
 *
 * Built to explain periodic report gaps on a Classic link between two
 * CYW43439 radios. Everything here asks the controller what it is doing
 * through standard HCI commands and prints the answers with timestamps, so
 * the output can be lined up against the dongle's gap log ('G').
 *
 * Console keys:
 *   A  capability dump: local + remote LMP feature bits related to adaptive
 *      hopping, current channel assessment mode, link quality, RSSI
 *   M  toggle the hopping-map monitor: Read AFH Channel Map every 25 ms on
 *      the Classic link, print every change of mode or map
 *   E  write channel assessment mode = enabled (does the command work at all?)
 *   D  write channel assessment mode = disabled
 *   K  toggle a counter monitor: Read Failed Contact Counter + Link Quality
 *      every 100 ms, print when they change
 */
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "floss/floss.h"
#include "link_diag.h"

#define OP_READ_AFH_ASSESSMENT_MODE  FLOSS_HCI_READ_AFH_CHANNEL_ASSESSMENT_MODE
#define OP_READ_FAILED_CONTACT       FLOSS_OPCODE(0x05, 0x0001)
#define OP_READ_LINK_QUALITY         FLOSS_OPCODE(0x05, 0x0003)
#define OP_READ_AFH_CHANNEL_MAP      FLOSS_OPCODE(0x05, 0x0006)
#define OP_READ_REMOTE_FEATURES      FLOSS_OPCODE(0x01, 0x001B)

#define MAP_PERIOD_US     25000u
#define COUNTER_PERIOD_US 100000u

static bool map_on, counters_on, map_busy, counters_busy;
static uint32_t map_next, counters_next;
static uint8_t last_mode = 0xFF;
static uint8_t last_map[10];
static bool have_map;
static uint16_t last_fcc = 0xFFFF;
static uint8_t last_lq = 0xFF;

/** Handle of the (first) open Classic link, 0xFFFF if none. */
static uint16_t classic_handle(void) {
    for (uint8_t i = 0; i < FLOSS_MAX_CONNECTIONS; i++) {
        const floss_conn_t *c = floss_hci_conn_at(i);
        if (c->state == FLOSS_CONN_OPEN && !c->le) {
            return c->handle;
        }
    }
    return 0xFFFF;
}

static uint32_t now_ms(void) {
    return time_us_32() / 1000u;
}

static unsigned popcount_map(const uint8_t m[10]) {
    unsigned n = 0;
    for (int i = 0; i < 10; i++) {
        for (uint8_t b = m[i]; b; b &= (uint8_t)(b - 1)) {
            n++;
        }
    }
    return n;
}

static void print_afh_features(const char *who, const uint8_t f[8]) {
    printf("%s features: AFH capable slave %u, classification slave %u, capable master %u, classification master %u, "
           "sniff subrating %u, EPR %u, SSP %u\n",
           who, (f[4] >> 3) & 1u, (f[4] >> 4) & 1u, (f[5] >> 3) & 1u, (f[5] >> 4) & 1u, (f[5] >> 1) & 1u,
           (f[5] >> 2) & 1u, (f[6] >> 3) & 1u);
}

/* ---- capability dump ---------------------------------------------------- */

static void on_assess_mode(void *ctx, uint16_t op, uint8_t status, const uint8_t *ret, uint16_t len) {
    if (status == 0 && len >= 1) {
        printf("AFH channel assessment mode: %u (%s)\n", ret[0], ret[0] ? "enabled" : "disabled");
    } else {
        printf("Read AFH Channel Assessment Mode failed: 0x%02x %s\n", status, floss_hci_status_str(status));
    }
}

static void on_link_quality_once(void *ctx, uint16_t op, uint8_t status, const uint8_t *ret, uint16_t len) {
    if (status == 0 && len >= 3) {
        printf("link quality: %u / 255\n", ret[2]);
    }
}

static void on_map_once(void *ctx, uint16_t op, uint8_t status, const uint8_t *ret, uint16_t len) {
    if (status == 0 && len >= 13) {
        printf("AFH: mode %u, %u of 79 channels in use, map", ret[2], popcount_map(&ret[3]));
        for (int i = 0; i < 10; i++) {
            printf(" %02x", ret[3 + i]);
        }
        printf("\n");
    } else {
        printf("Read AFH Channel Map failed: 0x%02x %s\n", status, floss_hci_status_str(status));
    }
}

static void on_cmd_status_only(void *ctx, uint16_t op, uint8_t status, const uint8_t *ret, uint16_t len) {
    printf("cmd 0x%04x -> 0x%02x %s\n", op, status, floss_hci_status_str(status));
}

/* Remote features arrive as an event, not in the command result; GAP does
 * not surface it, so we only report that the request was accepted and
 * rely on the HCI debug log ('v') for the event bytes. */

static void capability_dump(void) {
    print_afh_features("local", floss_hci_controller()->features);
    floss_hci_cmd(OP_READ_AFH_ASSESSMENT_MODE, NULL, 0, on_assess_mode, NULL);
    uint16_t h = classic_handle();
    if (h == 0xFFFF) {
        printf("no Classic link\n");
        return;
    }
    uint8_t p[2];
    floss_put_le16(p, h);
    floss_hci_cmd(OP_READ_AFH_CHANNEL_MAP, p, 2, on_map_once, NULL);
    floss_hci_cmd(OP_READ_LINK_QUALITY, p, 2, on_link_quality_once, NULL);
    floss_hci_cmd(OP_READ_REMOTE_FEATURES, p, 2, on_cmd_status_only, NULL);
    printf("(remote features: event 0x0B in the HCI debug log, enable with 'v')\n");
}

/* ---- monitors ----------------------------------------------------------- */

static void on_map_poll(void *ctx, uint16_t op, uint8_t status, const uint8_t *ret, uint16_t len) {
    map_busy = false;
    if (status != 0 || len < 13) {
        return;
    }
    uint8_t mode = ret[2];
    const uint8_t *map = &ret[3];
    if (mode != last_mode || !have_map || memcmp(map, last_map, 10) != 0) {
        printf("[%lu ms] AFH mode %u, %u channels:", (unsigned long)now_ms(), mode, popcount_map(map));
        for (int i = 0; i < 10; i++) {
            printf(" %02x", map[i]);
        }
        printf("\n");
        last_mode = mode;
        memcpy(last_map, map, 10);
        have_map = true;
    }
}

static void on_fcc_poll(void *ctx, uint16_t op, uint8_t status, const uint8_t *ret, uint16_t len) {
    if (status == 0 && len >= 4) {
        uint16_t fcc = floss_get_le16(&ret[2]);
        if (fcc != last_fcc) {
            printf("[%lu ms] failed contact counter %u\n", (unsigned long)now_ms(), fcc);
            last_fcc = fcc;
        }
    }
}

static void on_lq_poll(void *ctx, uint16_t op, uint8_t status, const uint8_t *ret, uint16_t len) {
    counters_busy = false;
    if (status == 0 && len >= 3 && ret[2] != last_lq) {
        printf("[%lu ms] link quality %u\n", (unsigned long)now_ms(), ret[2]);
        last_lq = ret[2];
    }
}

void link_diag_task(uint32_t now_us) {
    uint16_t h = classic_handle();
    if (h == 0xFFFF) {
        return;
    }
    uint8_t p[2];
    floss_put_le16(p, h);
    if (map_on && !map_busy && floss_time_diff(now_us, map_next) >= 0) {
        map_next = now_us + MAP_PERIOD_US;
        if (floss_hci_cmd(OP_READ_AFH_CHANNEL_MAP, p, 2, on_map_poll, NULL) == FLOSS_OK) {
            map_busy = true;
        }
    }
    if (counters_on && !counters_busy && floss_time_diff(now_us, counters_next) >= 0) {
        counters_next = now_us + COUNTER_PERIOD_US;
        if (floss_hci_cmd(OP_READ_FAILED_CONTACT, p, 2, on_fcc_poll, NULL) == FLOSS_OK &&
            floss_hci_cmd(OP_READ_LINK_QUALITY, p, 2, on_lq_poll, NULL) == FLOSS_OK) {
            counters_busy = true;
        }
    }
}

bool link_diag_key(int c) {
    uint8_t mode;
    switch (c) {
    case 'A':
        capability_dump();
        return true;
    case 'M':
        map_on = !map_on;
        have_map = false;
        printf("AFH map monitor %s\n", map_on ? "on" : "off");
        return true;
    case 'K':
        counters_on = !counters_on;
        last_fcc = 0xFFFF;
        last_lq = 0xFF;
        printf("counter monitor %s\n", counters_on ? "on" : "off");
        return true;
    case 'E':
    case 'D':
        mode = c == 'E' ? 1u : 0u;
        floss_hci_cmd(FLOSS_HCI_WRITE_AFH_CHANNEL_ASSESSMENT_MODE, &mode, 1, on_cmd_status_only, NULL);
        return true;
    default:
        return false;
    }
}
