/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * link_diag.h - Radio diagnostics console for the pad/dongle firmwares
 * (see link_diag.c for the keys).
 */
#ifndef LINK_DIAG_H
#define LINK_DIAG_H

#include <stdbool.h>
#include <stdint.h>

/** Handle a diagnostics console key; false if it is not one of ours. */
bool link_diag_key(int c);
/** Run the monitors (call every main-loop pass). */
void link_diag_task(uint32_t now_us);

#endif /* LINK_DIAG_H */
