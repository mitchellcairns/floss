/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * test_common.h - Shared fixtures for the integration suites (stack + sim).
 *
 * The helpers script the common opening of every scenario so individual
 * tests read like the story they check:
 *
 *     floss_transport_t *t = floss_tc_sim_init(NULL);
 *     floss_tc_hid_config(&cfg, 1000);
 *     floss_tc_stack_start(t, &cfg);           // init + start, until READY
 *     uint16_t h = floss_tc_host_connect();    // the PC pages us
 *     floss_tc_host_pair(h);                   // just-works SSP, encryption on
 *     floss_tc_hid_open(h, &ctrl, &intr);      // PSM 0x11 then 0x13
 *
 * All of them ASSERT internally (the framework's longjmp makes that safe).
 */
#ifndef FLOSS_TEST_COMMON_H
#define FLOSS_TEST_COMMON_H

#include "floss/floss.h"
#include "floss_sim.h"
#include "floss_test.h"

#ifdef __cplusplus
extern "C" {
#endif

/** The remote "PC" used by the scenarios: AA:BB:CC:DD:EE:01, class computer/laptop. */
extern const floss_addr_t floss_tc_host;
#define FLOSS_TC_HOST_COD 0x00010Cu

/** Test gamepad: input report ID 1 (16 buttons + 4 axes = 6 bytes), output report ID 2 (2 bytes). */
extern const uint8_t floss_tc_report_desc[];
extern const uint16_t floss_tc_report_desc_len;
#define FLOSS_TC_INPUT_ID   1
#define FLOSS_TC_INPUT_LEN  6
#define FLOSS_TC_OUTPUT_ID  2
#define FLOSS_TC_OUTPUT_LEN 2

/** What the HID callbacks observed (zeroed by floss_tc_stack_start). */
typedef struct {
    unsigned connected;
    floss_addr_t host;
    unsigned disconnected;
    uint8_t disconnect_reason;
    unsigned output_reports;
    uint8_t output_id;
    uint8_t output[64];
    uint16_t output_len;
    unsigned set_reports;
    unsigned set_protocol;
    uint8_t protocol;
    unsigned control_ops;
    uint8_t control_op;
    unsigned report_ticks;
} floss_tc_hid_log_t;

extern floss_tc_hid_log_t floss_tc_hid;
extern const floss_hid_callbacks_t floss_tc_hid_callbacks;

/** Every GAP event since floss_tc_stack_start. */
#define FLOSS_TC_GAP_LOG 64
typedef struct {
    floss_gap_evt_t evt[FLOSS_TC_GAP_LOG];
    unsigned count;
} floss_tc_gap_log_t;
extern floss_tc_gap_log_t floss_tc_gap;
/** Most recent GAP event of this type, NULL if none. */
const floss_gap_evt_t *floss_tc_gap_last(floss_gap_evt_type_t type);
unsigned floss_tc_gap_count(floss_gap_evt_type_t type);

/** Sim defaults wired to the test clock. */
void floss_tc_sim_config(floss_sim_config_t *cfg);
/** floss_sim_init(cfg or floss_tc_sim_config defaults). */
floss_transport_t *floss_tc_sim_init(const floss_sim_config_t *cfg);

/** Gamepad config for the test descriptor. cfg must stay valid while the stack runs (use static storage). */
void floss_tc_hid_config(floss_hid_config_t *cfg, uint32_t report_interval_us);

/**
 * floss_init + GAP listener + (HID init | make connectable) + floss_start,
 * then pump until GAP READY. hid == NULL runs the stack without HID.
 */
void floss_tc_stack_start(floss_transport_t *t, const floss_hid_config_t *hid);

/** The host pages the stack; returns the open handle. */
uint16_t floss_tc_host_connect(void);
/** The host pairs/authenticates and enables encryption. */
void floss_tc_host_pair(uint16_t handle);
/** The host opens HID control then interrupt; waits for HID CONNECTED. */
void floss_tc_hid_open(uint16_t handle, floss_sim_chan_t **ctrl, floss_sim_chan_t **intr);
/** connect + pair + open in one go. */
uint16_t floss_tc_hid_session(floss_sim_chan_t **ctrl, floss_sim_chan_t **intr);

/** Fail with details if the sim saw any protocol violation from the stack. */
void floss_tc_assert_sim_clean(void);

/** Pump until floss_hid_connected() (bounded). */
bool floss_tc_wait_hid_connected(uint32_t max_polls);

#ifdef __cplusplus
}
#endif

#endif /* FLOSS_TEST_COMMON_H */
