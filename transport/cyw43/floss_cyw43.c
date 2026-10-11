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
 *
 * Fast bus path (FLOSS_CYW43_FAST_BUS, default on)
 * ------------------------------------------------
 * Under the packet API the radio exchanges data through two 4 KB rings in
 * its memory, each with in/out indices; every ring entry is
 * [len24][type][payload padded to 4 bytes]. Each SDK read/write call does
 * a full handshake (wake request + wake check + index read + data + index
 * write + interrupt toggle), and a read of one packet takes TWO such calls
 * (header, then payload) plus a third that finds the ring empty. Measured
 * on a Pico W: ~250 us per TX packet and ~600 us per RX packet, all of it
 * with the main loop blocked.
 *
 * The fast path drives the same rings through the SDK's shared-bus driver
 * primitives (cybt_shared_bus_driver.h, BSD-3-Clause):
 *   RX: read all four indices in one 16-byte transfer; if the ring is empty
 *       stop there (one bus transaction); else read every pending byte in
 *       one (or two, when wrapping) bulk transfers, parse all whole packets,
 *       write the out index once, toggle the interrupt once.
 *   TX: index read, data write, in-index write, interrupt toggle. The wake
 *       request is made once at open (the SDK never releases it).
 */
#include "floss_cyw43.h"
#include "floss/floss_debug.h"
#include "floss/floss_hal.h"

#include "cyw43.h"

#ifndef FLOSS_CYW43_FAST_BUS
#define FLOSS_CYW43_FAST_BUS 1
#endif
#if FLOSS_CYW43_FAST_BUS
#include "cybt_shared_bus_driver.h"
/** Ring size in radio memory (both directions). */
#define CYW43_RING_SIZE BTSDIO_FWBUF_SIZE
#define CYW43_ROUND4(x) (((x) + 3u) & ~3u)
/** Bulk RX buffer: must hold the largest entry (4 + ACL 4 + 1021, rounded). */
static uint32_t s_rx_bulk[2048 / 4];
#endif

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

static floss_err_t floss_cyw43_open(floss_transport_t *t) {
    FLOSS_UNUSED(t);
    int rc = cyw43_bluetooth_hci_init();
    if (rc != 0) {
        FLOSS_LOGE(TRANSPORT, "cyw43: radio firmware load / HCI init failed (%d)", rc);
        return FLOSS_ERR_IO;
    }
    s_rx_pending = true; /* the controller may already have something queued */
    s_last_read_us = floss_hal_time_us();
#if FLOSS_CYW43_FAST_BUS
    /* Ask the radio core to stay awake for the fast path (the SDK path does
     * this before every transfer and never releases it either). */
    CYW43_THREAD_ENTER;
    cybt_set_bt_awake(1);
    for (int i = 0; i < 1000 && !cybt_awake(); i++) {
        cyw43_delay_us(10);
    }
    CYW43_THREAD_EXIT;
    FLOSS_LOGI(TRANSPORT, "cyw43: HCI up (fast bus path)");
#else
    FLOSS_LOGI(TRANSPORT, "cyw43: HCI up");
#endif
    return FLOSS_OK;
}

static void floss_cyw43_close(floss_transport_t *t) {
    FLOSS_UNUSED(t);
    s_rx_pending = false;
}

static floss_err_t floss_cyw43_send(floss_transport_t *t, uint8_t type, uint8_t *pkt, uint16_t len) {
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
#if FLOSS_CYW43_FAST_BUS
    floss_put_le24(frame, len); /* ring entry length excludes the 4-byte header */
    frame[3] = type;
    uint32_t total = CYW43_ROUND4((uint32_t)frame_len); /* reads up to 3 bytes of slack */
    int rc = 0;

    CYW43_THREAD_ENTER;
    cybt_fw_membuf_index_t idx;
    cybt_get_bt_buf_index(&idx);
    uint32_t in = idx.host2bt_in_val;
    uint32_t space = (idx.host2bt_out_val - in - 4u) & (CYW43_RING_SIZE - 1u);
    if (total > space) {
        rc = -1; /* ring full: the controller has not consumed earlier packets yet */
    } else {
        uint32_t first = FLOSS_MIN(total, CYW43_RING_SIZE - in);
        cybt_mem_write_idx(H2B_BUF_ADDR_IDX, in, frame, first);
        if (total > first) {
            cybt_mem_write_idx(H2B_BUF_ADDR_IDX, 0, frame + first, total - first);
        }
        cybt_reg_write_idx(H2B_BUF_IN_ADDR_IDX, (in + total) & (CYW43_RING_SIZE - 1u));
        cybt_toggle_bt_intr();
    }
    CYW43_THREAD_EXIT;
#else
    frame[3] = type; /* bytes 0..2 (length) are filled in by the driver */

    CYW43_THREAD_ENTER;
    int rc = cyw43_bluetooth_hci_write(frame, frame_len);
    CYW43_THREAD_EXIT;
#endif

    if (rc != 0) {
        /* Most likely the radio's host-to-controller ring is full: let the stack retry. */
        FLOSS_LOGD(TRANSPORT, "cyw43: write failed (%d), retrying later", rc);
        return FLOSS_ERR_BUSY;
    }
    return FLOSS_OK;
}

#if FLOSS_CYW43_FAST_BUS
/*
 * Drain the radio->host ring: one index read, bulk data read, one index
 * write, one interrupt toggle - however many packets were waiting.
 */
static int cyw43_fast_rx(floss_transport_t *t) {
    uint8_t *buf = (uint8_t *)s_rx_bulk;
    uint32_t got;
    uint32_t used = 0;

    CYW43_THREAD_ENTER;
    cybt_fw_membuf_index_t idx;
    cybt_get_bt_buf_index(&idx);
    uint32_t out = idx.bt2host_out_val;
    uint32_t avail = (idx.bt2host_in_val - out) & (CYW43_RING_SIZE - 1u);
    if (avail == 0) {
        CYW43_THREAD_EXIT;
        return 0;
    }
    got = FLOSS_MIN(avail, (uint32_t)sizeof(s_rx_bulk));
    uint32_t first = FLOSS_MIN(got, CYW43_RING_SIZE - out);
    cybt_mem_read_idx(B2H_BUF_ADDR_IDX, out, buf, first);
    if (got > first) {
        cybt_mem_read_idx(B2H_BUF_ADDR_IDX, 0, buf + first, got - first);
    }
    /* Consume only whole entries; a partial one is re-read next poll. */
    while (used + CYW43_BT_HDR_LEN <= got) {
        uint32_t entry = CYW43_BT_HDR_LEN + CYW43_ROUND4(floss_get_le24(buf + used));
        if (used + entry > got) {
            break;
        }
        used += entry;
    }
    if (used) {
        cybt_reg_write_idx(B2H_BUF_OUT_ADDR_IDX, (out + used) & (CYW43_RING_SIZE - 1u));
        cybt_toggle_bt_intr();
    }
    CYW43_THREAD_EXIT;

    if (used < got || got < avail) {
        s_rx_pending = true; /* more waiting: come straight back next poll */
    }

    /* Deliver outside the bus lock: handlers may transmit. */
    int delivered = 0;
    for (uint32_t off = 0; off < used;) {
        uint32_t len = floss_get_le24(buf + off);
        if (len == 0 || len > FLOSS_BUF_SIZE + 4u) {
            FLOSS_STAT_INC(transport_errors);
            FLOSS_LOGW(TRANSPORT, "cyw43: bad ring entry length %lu", (unsigned long)len);
            break;
        }
        floss_transport_deliver(t, buf[off + 3], buf + off + CYW43_BT_HDR_LEN, (uint16_t)len);
        delivered++;
        off += CYW43_BT_HDR_LEN + CYW43_ROUND4(len);
    }
    return delivered;
}
#endif

static int floss_cyw43_poll(floss_transport_t *t) {
#if FLOSS_CYW43_BARE
    /* No async_context: the driver's own work (host-wake, bus sleep) runs here. */
    floss_cyw43_driver_service();
#endif
    uint32_t now = floss_hal_time_us();
    if (!s_rx_pending && floss_time_diff(now, s_last_read_us) < (int32_t)CYW43_RX_SAFETY_POLL_US) {
        return 0;
    }
    /* Clear before reading: a notification arriving meanwhile re-arms it. */
    s_rx_pending = false;
    s_last_read_us = now;

#if FLOSS_CYW43_FAST_BUS
    return cyw43_fast_rx(t);
#endif
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
    .open = floss_cyw43_open,
    .close = floss_cyw43_close,
    .send = floss_cyw43_send,
    .poll = floss_cyw43_poll,
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
    /* Measured: completions are reported two packets at a time, 3-8 ms after
     * the packet left (a single outstanding packet waits ~250 ms). */
    .completion_batch = 2,
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
