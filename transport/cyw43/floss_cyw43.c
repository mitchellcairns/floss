/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_cyw43.c - HCI transport for the Infineon CYW43439 (Pico W / Pico 2 W).
 *
 * Overview
 * --------
 * The radio shares one SPI bus with WLAN. The Pico SDK's cyw43 driver owns
 * that bus, downloads the radio firmware and exposes a packet interface:
 *
 *   cyw43_bluetooth_hci_write(buf, len)   buf = [len0 len1 len2 type][HCI pkt]
 *   cyw43_bluetooth_hci_read(buf, max, &len)   same layout, len includes the
 *                                              4 header bytes, 0 = nothing
 *
 * TX is zero-copy: the stack leaves 4 bytes of headroom in front of every
 * packet, we put the type in byte 3 and the driver fills the length. The
 * bus code moves whole 32-bit words, so if the frame start is not 4-byte
 * aligned (it depends on how many headers upper layers pushed) we bounce it
 * through an aligned static buffer instead.
 *
 * RX is interrupt driven by the cyw43 driver: when the radio has data its
 * poll worker calls cyw43_bluetooth_hci_process(), which we define to set a
 * flag. floss poll() reads only when the flag is set, plus a cheap safety
 * read every RX_SAFETY_POLL_US in case a notification was missed. Reads are
 * capped per poll to keep floss_poll() latency bounded.
 *
 * Threading: bus access is wrapped in CYW43_THREAD_ENTER/EXIT (the SDK's
 * lock). Packets are delivered to the stack outside the lock, so the stack
 * may send from its RX handlers.
 */
#include "floss_cyw43.h"
#include "floss/floss_debug.h"
#include "floss/floss_hal.h"

#include "cyw43.h"

/** Shared-bus header in front of every HCI packet. */
#define CYW43_BT_HDR_LEN 4
/** Max packets delivered by one poll. */
#define CYW43_MAX_PACKETS_PER_POLL 8
/** Read even without a notification this often (missed-wakeup safety net). */
#define CYW43_RX_SAFETY_POLL_US 10000u

/* Set from the cyw43 driver's worker context, consumed by poll(). */
static volatile bool s_rx_pending;
static uint32_t s_last_read_us;

/* Word-aligned frames (+4 slack: the bus rounds transfers up to whole words). */
static uint32_t s_rx_frame[(CYW43_BT_HDR_LEN + FLOSS_BUF_SIZE + 4 + 3) / 4];
static uint32_t s_tx_bounce[(CYW43_BT_HDR_LEN + FLOSS_BUF_SIZE + 4 + 3) / 4];

static floss_transport_t s_transport;

/**
 * Called by the cyw43 driver (from its poll worker) when the radio has
 * something for us. Only sets a flag: the stack drains it from floss_poll().
 */
void cyw43_bluetooth_hci_process(void) {
    s_rx_pending = true;
}

static floss_err_t cyw43_open(floss_transport_t *t) {
    FLOSS_UNUSED(t);
    int rc = cyw43_bluetooth_hci_init();
    if (rc != 0) {
        FLOSS_LOGE(TRANSPORT, "cyw43: radio firmware load / HCI init failed (%d)", rc);
        return FLOSS_ERR_IO;
    }
    s_rx_pending = true; /* the controller may already have something queued */
    s_last_read_us = floss_hal_time_us();
    FLOSS_LOGI(TRANSPORT, "cyw43: HCI up");
    return FLOSS_OK;
}

static void cyw43_close(floss_transport_t *t) {
    FLOSS_UNUSED(t);
    s_rx_pending = false;
}

static floss_err_t cyw43_send(floss_transport_t *t, uint8_t type, uint8_t *pkt, uint16_t len) {
    FLOSS_UNUSED(t);
    uint8_t *frame = pkt - CYW43_BT_HDR_LEN;
    size_t frame_len = (size_t)len + CYW43_BT_HDR_LEN;

    if (((uintptr_t)frame & 3u) != 0) {
        if (frame_len > sizeof(s_tx_bounce) - 4) {
            FLOSS_LOGE(TRANSPORT, "cyw43: packet too large (%u)", len);
            return FLOSS_ERR_INVALID;
        }
        frame = (uint8_t *)s_tx_bounce;
        memcpy(frame + CYW43_BT_HDR_LEN, pkt, len);
    }
    frame[3] = type; /* bytes 0..2 (length) are filled in by the driver */

    CYW43_THREAD_ENTER;
    int rc = cyw43_bluetooth_hci_write(frame, frame_len);
    CYW43_THREAD_EXIT;

    if (rc != 0) {
        /* Most likely the radio's host-to-controller ring is full: let the stack retry. */
        FLOSS_LOGD(TRANSPORT, "cyw43: write failed (%d), retrying later", rc);
        return FLOSS_ERR_BUSY;
    }
    return FLOSS_OK;
}

static int cyw43_poll(floss_transport_t *t) {
    uint32_t now = floss_hal_time_us();
    if (!s_rx_pending && floss_time_diff(now, s_last_read_us) < (int32_t)CYW43_RX_SAFETY_POLL_US) {
        return 0;
    }
    /* Clear before reading: a notification arriving meanwhile re-arms it. */
    s_rx_pending = false;
    s_last_read_us = now;

    uint8_t *frame = (uint8_t *)s_rx_frame;
    int delivered = 0;
    while (delivered < CYW43_MAX_PACKETS_PER_POLL) {
        uint32_t len = 0;
        CYW43_THREAD_ENTER;
        int rc = cyw43_bluetooth_hci_read(frame, sizeof(s_rx_frame), &len);
        CYW43_THREAD_EXIT;

        if (rc != 0) {
            /* Leave the flag clear: the safety-net read retries in RX_SAFETY_POLL_US. */
            FLOSS_STAT_INC(transport_errors);
            FLOSS_LOGE(TRANSPORT, "cyw43: read failed (%d)", rc);
            return delivered;
        }
        if (len == 0) {
            return delivered; /* drained */
        }
        if (len <= CYW43_BT_HDR_LEN || len > sizeof(s_rx_frame)) {
            FLOSS_STAT_INC(transport_errors);
            FLOSS_LOGW(TRANSPORT, "cyw43: bad frame length %u", (unsigned)len);
            return delivered;
        }
        floss_transport_deliver(t, frame[3], frame + CYW43_BT_HDR_LEN, (uint16_t)(len - CYW43_BT_HDR_LEN));
        delivered++;
    }
    /* Hit the per-poll cap: more may be waiting, come straight back next poll. */
    s_rx_pending = true;
    return delivered;
}

static const floss_transport_ops_t k_cyw43_ops = {
    .name = "cyw43",
    .headroom = CYW43_BT_HDR_LEN,
    .open = cyw43_open,
    .close = cyw43_close,
    .send = cyw43_send,
    .poll = cyw43_poll,
};

floss_transport_t *floss_cyw43_transport(void) {
    s_transport.ops = &k_cyw43_ops;
    s_transport.ctx = NULL;
    return &s_transport;
}

const floss_chipset_t floss_chipset_cyw43 = {
    .name = "cyw43",
    .set_addr_opcode = 0xFC01, /* vendor: write device address, 6 bytes LSB first */
    .init_script = NULL,
    .init_script_len = 0,
};

void floss_cyw43_default_addr(floss_addr_t *addr) {
    uint8_t mac[6]; /* MSB first */
    /*
     * cyw43_state.mac is only filled once the chip has been powered and its
     * OTP read, which happens inside the (idempotent) radio bring-up. Do it
     * here so callers may ask for the address before floss_start().
     */
    (void)cyw43_bluetooth_hci_init();
    cyw43_hal_get_mac(0, mac);
    mac[5]++;
    for (int i = 0; i < 6; i++) {
        addr->b[i] = mac[5 - i];
    }
}
