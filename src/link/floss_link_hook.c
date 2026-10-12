/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_link_hook.c - floss_link_poll(): the run loop's entry into the link
 * profile.
 *
 * floss_poll() calls floss_link_poll() on every iteration. If that call
 * went straight into floss_link.c, every firmware (a Classic HID gamepad
 * too) would link the whole profile: core, both transports, their tables.
 * So it lives alone in this tiny object and jumps through a pointer that
 * floss_link_init() sets: firmwares that never call floss_link_init() pull
 * in eight bytes, and the hot path costs one load and an indirect call.
 */
#include "floss_link_priv.h"

#if FLOSS_ENABLE_LINK

void (*floss_link_poll_hook)(void);

void floss_link_poll(void)
{
    void (*fn)(void) = floss_link_poll_hook;
    if (fn) {
        fn();
    }
}

void floss_link_reset(void)
{
    floss_link_poll_hook = NULL;
}

#endif /* FLOSS_ENABLE_LINK */
