/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * usb_gamepad.c - Radio <-> USB HID bridge of the link dongle.
 *
 * The pad sends exactly the report the USB interface declares
 * (examples/gamepad/gamepad_state.h: input ID 1, 13 bytes), so forwarding
 * is a copy - no translation, and the millisecond-counter axis of the demo
 * reaches the PC as the pad produced it.
 *
 * One pending slot plus whatever sits in the IN endpoint is the whole
 * buffering: at most one report waits for the host's next poll, and a newer
 * radio report always replaces a waiting one.
 */
#include "pico/stdlib.h"
#include "tusb.h"

#include "floss/floss.h"
#include "floss/floss_link.h"
#include "gamepad_state.h"
#include "usb_gamepad.h"

static struct {
    /* input: radio -> USB */
    uint8_t pending[GAMEPAD_INPUT_REPORT_LEN];
    bool have_pending;
    bool pending_measured;   /* came from the radio (time the host wait) */
    uint32_t pending_t_rx;
    uint8_t current[GAMEPAD_INPUT_REPORT_LEN]; /* last report handed to USB: GET_REPORT answer */
    bool in_flight;          /* a measured report sits in the IN endpoint */
    uint32_t in_flight_t_rx;
    /* output: USB -> radio */
    uint8_t out[GAMEPAD_OUTPUT_REPORT_LEN];
    bool have_out;
    usb_gamepad_stats_t st;
    meter_hist_t wait;
} ug;

static bool usb_up(void) {
    return tud_mounted() && !tud_suspended();
}

/* ------------------------------------------------------------------------ */
/* Input: radio -> USB                                                      */
/* ------------------------------------------------------------------------ */

/* Hand the pending report to the IN endpoint if it is free. */
static void push_pending(void) {
    if (!ug.have_pending || !tud_hid_ready()) {
        return;
    }
    if (!tud_hid_report(GAMEPAD_REPORT_ID_INPUT, ug.pending, GAMEPAD_INPUT_REPORT_LEN)) {
        return; /* endpoint taken meanwhile: next loop */
    }
    memcpy(ug.current, ug.pending, sizeof(ug.current));
    ug.have_pending = false;
    ug.in_flight = ug.pending_measured;
    ug.in_flight_t_rx = ug.pending_t_rx;
    ug.st.forwarded++;
}

static void queue_report(const uint8_t *data, bool measured, uint32_t t_rx_us) {
    if (ug.have_pending && measured) {
        if (usb_up()) {
            ug.st.replaced++; /* USB had not taken the previous one yet */
        }
    }
    memcpy(ug.pending, data, GAMEPAD_INPUT_REPORT_LEN);
    ug.have_pending = true;
    ug.pending_measured = measured;
    ug.pending_t_rx = t_rx_us;
    push_pending(); /* endpoint idle: no need to wait for the next loop */
}

void usb_gamepad_input(uint8_t report_id, const uint8_t *data, uint8_t len, uint32_t t_rx_us) {
    if (report_id != GAMEPAD_REPORT_ID_INPUT || len != GAMEPAD_INPUT_REPORT_LEN) {
        ug.st.bad_input++;
        return;
    }
    queue_report(data, true, t_rx_us);
}

void usb_gamepad_neutral(void) {
    gamepad_state_t s;
    uint8_t report[GAMEPAD_INPUT_REPORT_LEN];
    gamepad_state_init(&s);
    gamepad_state_build_report(&s, report);
    queue_report(report, false, 0);
}

/* IN transfer done: the host has read the report. */
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report, uint16_t len) {
    (void)instance;
    (void)report;
    (void)len;
    if (ug.in_flight) {
        meter_hist_add(&ug.wait, time_us_32() - ug.in_flight_t_rx);
        ug.in_flight = false;
        ug.st.delivered++;
    }
}

/* GET_REPORT: the input report the host would see now. */
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
    (void)instance;
    if (report_type != HID_REPORT_TYPE_INPUT || report_id != GAMEPAD_REPORT_ID_INPUT) {
        return 0; /* nothing but the report ID */
    }
    uint16_t n = reqlen < GAMEPAD_INPUT_REPORT_LEN ? reqlen : GAMEPAD_INPUT_REPORT_LEN;
    memcpy(buffer, ug.current, n);
    return n;
}

/* ------------------------------------------------------------------------ */
/* Output: USB -> radio                                                     */
/* ------------------------------------------------------------------------ */

/*
 * SET_REPORT on control arrives with the report ID split off; interrupt
 * OUT arrives with report_id 0 and the ID as the first byte (the
 * descriptor uses report IDs). Only stored here (TinyUSB callbacks must not
 * print or block); usb_gamepad_task() sends it.
 */
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize) {
    (void)instance;
    if (report_id == 0 && bufsize > 0) {
        report_id = buffer[0];
        buffer++;
        bufsize--;
    }
    if (report_type != HID_REPORT_TYPE_OUTPUT) {
        return; /* no feature reports */
    }
    ug.st.outputs_received++;
    if (report_id != GAMEPAD_REPORT_ID_OUTPUT || bufsize < GAMEPAD_OUTPUT_REPORT_LEN) {
        ug.st.outputs_dropped++;
        return;
    }
    if (ug.have_out) {
        ug.st.outputs_dropped++; /* superseded before the radio took it */
    }
    memcpy(ug.out, buffer, GAMEPAD_OUTPUT_REPORT_LEN);
    ug.have_out = true;
}

static void forward_output(void) {
    if (!ug.have_out) {
        return;
    }
    if (!floss_link_connected()) {
        ug.have_out = false;
        ug.st.outputs_dropped++;
        return;
    }
    floss_err_t err = floss_link_send_output(GAMEPAD_REPORT_ID_OUTPUT, ug.out, GAMEPAD_OUTPUT_REPORT_LEN);
    if (err == FLOSS_ERR_BUSY || err == FLOSS_ERR_NOMEM) {
        return; /* no buffer / credit right now: retry next loop */
    }
    ug.have_out = false;
    if (err == FLOSS_OK) {
        ug.st.outputs_forwarded++;
    } else {
        ug.st.outputs_dropped++;
    }
}

/* ------------------------------------------------------------------------ */
/* Task, stats                                                              */
/* ------------------------------------------------------------------------ */

void usb_gamepad_init(void) {
    memset(&ug, 0, sizeof(ug));
    gamepad_state_t s;
    gamepad_state_init(&s);
    gamepad_state_build_report(&s, ug.current);
}

void usb_gamepad_task(void) {
    push_pending();
    forward_output();
}

const usb_gamepad_stats_t *usb_gamepad_stats(void) {
    return &ug.st;
}

void usb_gamepad_stats_reset(void) {
    memset(&ug.st, 0, sizeof(ug.st));
    meter_hist_reset(&ug.wait);
}

uint32_t usb_gamepad_wait_window(uint32_t *p50_us, uint32_t *p99_us, uint32_t *max_us) {
    uint32_t n = ug.wait.count;
    *p50_us = meter_hist_percentile(&ug.wait, 50);
    *p99_us = meter_hist_percentile(&ug.wait, 99);
    *max_us = ug.wait.max_us;
    meter_hist_reset(&ug.wait);
    return n;
}
