/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * usb_gamepad.h - The link dongle's USB HID gamepad: radio reports in,
 * USB reports out, rumble back.
 *
 * Input path (latest wins, never queues):
 *
 *   floss_link on_input --> usb_gamepad_input() --> one pending slot
 *                                                       |
 *        tud_hid_ready()? (endpoint free) --> tud_hid_report()  --> host
 *
 * A report is handed to USB as soon as the interrupt IN endpoint is free -
 * at the latest at the host's next 1 ms poll after the previous one was
 * read. If a newer radio report arrives while the slot is still waiting,
 * it replaces the older one ("replaced" in the stats): the host always
 * gets the freshest state, never a backlog.
 *
 * Output path: SET_REPORT (control) or interrupt OUT with report ID 2 is
 * kept in one slot (latest wins) and sent to the pad with
 * floss_link_send_output() from usb_gamepad_task(), retried while the
 * link is busy, dropped when no pad is connected.
 *
 * All functions run in the main loop context (floss_poll callbacks and
 * tud_task callbacks alike), so no locking.
 */
#ifndef USB_GAMEPAD_H
#define USB_GAMEPAD_H

#include <stdbool.h>
#include <stdint.h>

#include "link_meter.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t forwarded;     /**< reports handed to USB (tud_hid_report) */
    uint32_t delivered;     /**< ... and read by the host (IN transfer complete) */
    uint32_t replaced;      /**< radio report replaced by a newer one before USB took it (USB up) */
    uint32_t bad_input;     /**< radio report with an unexpected ID / length (not forwarded) */
    uint32_t outputs_received;  /**< output reports from the host */
    uint32_t outputs_forwarded; /**< ... sent to the pad */
    uint32_t outputs_dropped;   /**< ... dropped (no pad, bad ID / length) */
} usb_gamepad_stats_t;

/** Start with a neutral report (answers GET_REPORT before any radio input). */
void usb_gamepad_init(void);

/** A radio input report arrived (t_rx_us: dongle arrival time). */
void usb_gamepad_input(uint8_t report_id, const uint8_t *data, uint8_t len, uint32_t t_rx_us);

/** The pad went away: release every control (neutral report). */
void usb_gamepad_neutral(void);

/** Push the pending report into a free endpoint; forward pending output. Every loop. */
void usb_gamepad_task(void);

const usb_gamepad_stats_t *usb_gamepad_stats(void);
void usb_gamepad_stats_reset(void);

/**
 * Host wait window: radio arrival -> host read of the same report, i.e.
 * the dongle + USB polling share of the end-to-end latency (25 us bins).
 * Returns the number of samples; reading it starts a new window.
 */
uint32_t usb_gamepad_wait_window(uint32_t *p50_us, uint32_t *p99_us, uint32_t *max_us);

#ifdef __cplusplus
}
#endif

#endif /* USB_GAMEPAD_H */
