/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * serial_echo_app.c - Serial port echo: uppercases and returns what it gets.
 *
 * Design: an echo server receives exactly as fast as the peer sends, but can
 * only transmit as fast as the peer grants it credits. Echoing straight from
 * on_data would therefore either drop data or need unbounded memory. So:
 *
 *   on_data  -> uppercase into a ring buffer -> send as much as credits allow
 *   on_can_send -> send more from the ring
 *
 * and the DLC runs in manual credit mode: we only hand the peer a credit
 * (= permission to send one frame of up to dlc->mtu bytes) when the ring has
 * room for that frame. That is RFCOMM back-pressure end to end, with a fixed
 * amount of RAM. Peers without credit flow control cannot be throttled this
 * way; for them a full ring drops bytes and says so in the log.
 */
#include "serial_echo_app.h"

#include "floss/floss_rfcomm.h"

#if FLOSS_ENABLE_RFCOMM

#define ECHO_CHANNEL      1
#define ECHO_SERVICE_NAME "FLOSS Serial"
#define ECHO_DEVICE_NAME  "FLOSS Serial Echo"

/** Echo backlog. Should hold FLOSS_RFCOMM_RX_CREDITS (initial window) frames. */
#ifndef ECHO_RING_SIZE
#define ECHO_RING_SIZE 4096
#endif

static struct {
    floss_rfcomm_dlc_t *dlc;
    uint16_t head;  /* oldest byte */
    uint16_t count; /* bytes waiting to be echoed */
    uint32_t dropped;
    uint8_t ring[ECHO_RING_SIZE];
} s_echo;

/** Hand out credits for as many whole frames as the ring can still absorb. */
static void echo_grant(void) {
    floss_rfcomm_dlc_t *dlc = s_echo.dlc;
    if (dlc == NULL || !dlc->cfc || dlc->mtu == 0) {
        return;
    }
    uint32_t room_frames = (uint32_t)(ECHO_RING_SIZE - s_echo.count) / dlc->mtu;
    uint32_t promised = (uint32_t)dlc->rx_credits + dlc->grant_pending;
    if (room_frames > promised) {
        floss_rfcomm_grant(dlc, (uint8_t)FLOSS_MIN(room_frames - promised, 255u));
    }
}

/** Send from the ring until it is empty or RFCOMM pushes back. */
static void echo_flush(void) {
    while (s_echo.count > 0 && s_echo.dlc != NULL) {
        uint16_t chunk = FLOSS_MIN(s_echo.count, (uint16_t)(ECHO_RING_SIZE - s_echo.head));
        int n = floss_rfcomm_send(s_echo.dlc, &s_echo.ring[s_echo.head], chunk);
        if (n <= 0) {
            break; /* BUSY: on_can_send will call us again */
        }
        s_echo.head = (uint16_t)((s_echo.head + n) % ECHO_RING_SIZE);
        s_echo.count = (uint16_t)(s_echo.count - n);
        if (n < chunk) {
            break;
        }
    }
    echo_grant();
}

static void echo_on_open(floss_rfcomm_dlc_t *dlc, uint8_t status) {
    if (status != FLOSS_RFCOMM_STATUS_OK) {
        return;
    }
    if (s_echo.dlc != NULL) {
        /* One echo session at a time keeps the example simple. */
        FLOSS_LOGW(APP, "echo: already serving a peer, closing new DLC");
        floss_rfcomm_disconnect(dlc);
        return;
    }
    s_echo.dlc = dlc;
    s_echo.head = 0;
    s_echo.count = 0;
    s_echo.dropped = 0;
    floss_rfcomm_set_manual_credits(dlc, true);
    echo_grant();
    FLOSS_LOGI(APP, "echo: serial port open (link 0x%03X, mtu %u)", dlc->handle, dlc->mtu);
}

static void echo_on_data(floss_rfcomm_dlc_t *dlc, const uint8_t *data, uint16_t len) {
    if (dlc != s_echo.dlc) {
        return;
    }
    for (uint16_t i = 0; i < len; i++) {
        if (s_echo.count == ECHO_RING_SIZE) {
            s_echo.dropped += (uint32_t)(len - i);
            FLOSS_LOGW(APP, "echo: backlog full, %u bytes dropped", (unsigned)(len - i));
            break;
        }
        uint8_t c = data[i];
        if (c >= 'a' && c <= 'z') {
            c = (uint8_t)(c - 'a' + 'A');
        }
        s_echo.ring[(s_echo.head + s_echo.count) % ECHO_RING_SIZE] = c;
        s_echo.count++;
    }
    echo_flush();
}

static void echo_on_can_send(floss_rfcomm_dlc_t *dlc) {
    if (dlc == s_echo.dlc) {
        echo_flush();
    }
}

static void echo_on_close(floss_rfcomm_dlc_t *dlc) {
    if (dlc != s_echo.dlc) {
        return;
    }
    FLOSS_LOGI(APP, "echo: serial port closed (%u bytes unsent, %lu dropped)", s_echo.count,
               (unsigned long)s_echo.dropped);
    s_echo.dlc = NULL;
    s_echo.count = 0;
}

static const floss_rfcomm_ops_t s_echo_ops = {
    .on_open = echo_on_open,
    .on_data = echo_on_data,
    .on_close = echo_on_close,
    .on_can_send = echo_on_can_send,
};

floss_err_t serial_echo_app_init(void) {
    floss_err_t err = floss_spp_init(ECHO_CHANNEL, ECHO_SERVICE_NAME, &s_echo_ops, NULL);
    if (err != FLOSS_OK) {
        FLOSS_LOGE(APP, "echo: serial port service failed (%s)", floss_err_str(err));
        return err;
    }
    floss_gap_set_name(ECHO_DEVICE_NAME);
    floss_gap_set_scan(true, true);
    FLOSS_LOGI(APP, "echo: ready on channel %u", ECHO_CHANNEL);
    return FLOSS_OK;
}

#else /* !FLOSS_ENABLE_RFCOMM */

floss_err_t serial_echo_app_init(void) {
    FLOSS_LOGE(APP, "echo: build with FLOSS_ENABLE_RFCOMM=1");
    return FLOSS_ERR_UNSUPPORTED;
}

#endif /* FLOSS_ENABLE_RFCOMM */
