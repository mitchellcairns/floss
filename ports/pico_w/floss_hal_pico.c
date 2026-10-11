/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_hal_pico.c - HAL for the Raspberry Pi Pico W (RP2040 / RP2350).
 *
 * Design notes:
 *
 *  TIME      time_us_32(): one load from the free-running 1 MHz timer,
 *            callable from any core or interrupt.
 *
 *  CRITICAL  A hardware spin lock taken with interrupts disabled
 *            (spin_lock_blocking). That excludes interrupts on this core AND
 *            the other core, which is what floss_hid_set_input() needs when
 *            it is called from core 1. We use the raw spin lock rather than
 *            critical_section_t because the HAL API returns the saved
 *            interrupt state to the caller, which spin_lock_blocking does.
 *
 *  STORE     Bonds are a handful of small records, written only when a
 *            pairing completes or a bond is deleted. They live in the LAST
 *            4 KB flash sector as one image (header + fixed table of
 *            key/value entries) that is mirrored in RAM: reads never touch
 *            flash, and a write rewrites the sector only if the content
 *            really changed. The image carries a magic, a layout version
 *            and a checksum; an interrupted erase/program leaves an invalid
 *            image, which loads as "no bonds" (re-pair) rather than garbage.
 *            Erasing a sector takes ~50 ms with interrupts off (XIP is
 *            unavailable meanwhile) - acceptable because it only happens
 *            during pairing, never while reports flow.
 *
 *  LOG       USB CDC via stdio. Dropped entirely when no terminal is
 *            attached; when one is attached but not reading, the SDK's
 *            stdout timeout (PICO_STDIO_USB_STDOUT_TIMEOUT_US, set to a few
 *            ms in CMakeLists.txt) bounds the stall.
 *
 *  ENTROPY   get_rand_32() (ring oscillator + timer seeded PRNG): plenty
 *            for Classic HID, which leaves key generation to the controller.
 */
#include <string.h>

#include "pico/stdlib.h"
#include "pico/flash.h"
#include "pico/rand.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#if LIB_PICO_STDIO_USB
#include "pico/stdio_usb.h"
#endif

#include "floss/floss.h"
#include "floss_hal_pico.h"

/* ------------------------------------------------------------------------ */
/* Time                                                                     */
/* ------------------------------------------------------------------------ */

uint32_t floss_hal_time_us(void) {
    return time_us_32();
}

/* ------------------------------------------------------------------------ */
/* Critical sections                                                        */
/* ------------------------------------------------------------------------ */

static spin_lock_t *hal_lock;

uint32_t floss_hal_critical_enter(void) {
    if (hal_lock) {
        return spin_lock_blocking(hal_lock);
    }
    return save_and_disable_interrupts(); /* before floss_pico_hal_init() */
}

void floss_hal_critical_exit(uint32_t state) {
    if (hal_lock) {
        spin_unlock(hal_lock, state);
    } else {
        restore_interrupts(state);
    }
}

/* ------------------------------------------------------------------------ */
/* Key/value store in the last flash sector                                 */
/* ------------------------------------------------------------------------ */

#define STORE_MAGIC        0x53534C46u /* "FLSS" */
#define STORE_VERSION      1u
#define STORE_ENTRIES      16u
#define STORE_VALUE_MAX    64u
#define STORE_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

typedef struct {
    uint16_t key;
    uint8_t used;
    uint8_t len;
    uint8_t value[STORE_VALUE_MAX];
} store_entry_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t entry_count; /* layout check: STORE_ENTRIES */
    uint32_t checksum;    /* FNV-1a over entries[] */
    uint32_t reserved;
    store_entry_t entries[STORE_ENTRIES];
} store_image_t;

/* flash_range_program() writes whole 256-byte pages. */
#define STORE_IMAGE_BYTES (((sizeof(store_image_t) + FLASH_PAGE_SIZE - 1u) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE)

static union {
    store_image_t img;
    uint8_t raw[STORE_IMAGE_BYTES];
} store __attribute__((aligned(4)));

static uint32_t store_checksum(const store_image_t *img) {
    const uint8_t *p = (const uint8_t *)img->entries;
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < sizeof(img->entries); i++) {
        h = (h ^ p[i]) * 16777619u;
    }
    return h;
}

static void store_load(void) {
    const store_image_t *flash = (const store_image_t *)(XIP_BASE + STORE_FLASH_OFFSET);
    if (flash->magic == STORE_MAGIC && flash->version == STORE_VERSION && flash->entry_count == STORE_ENTRIES &&
        flash->checksum == store_checksum(flash)) {
        memcpy(&store.img, flash, sizeof(store_image_t));
    } else {
        memset(&store, 0, sizeof(store));
    }
}

/* Runs with interrupts off and the other core parked (flash_safe_execute). */
static void store_flash_op(void *param) {
    (void)param;
    flash_range_erase(STORE_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(STORE_FLASH_OFFSET, store.raw, sizeof(store.raw));
}

static floss_err_t store_commit(void) {
    store.img.magic = STORE_MAGIC;
    store.img.version = STORE_VERSION;
    store.img.entry_count = STORE_ENTRIES;
    store.img.checksum = store_checksum(&store.img);
    int rc = flash_safe_execute(store_flash_op, NULL, 100);
    return rc == PICO_OK ? FLOSS_OK : FLOSS_ERR_IO;
}

static store_entry_t *store_find(uint16_t key) {
    for (unsigned i = 0; i < STORE_ENTRIES; i++) {
        store_entry_t *e = &store.img.entries[i];
        if (e->used && e->key == key) {
            return e;
        }
    }
    return NULL;
}

int floss_hal_store_read(uint16_t key, void *buf, uint16_t len) {
    const store_entry_t *e = store_find(key);
    if (!e) {
        return FLOSS_ERR_NOT_FOUND;
    }
    uint16_t n = len < e->len ? len : e->len;
    memcpy(buf, e->value, n);
    return n;
}

floss_err_t floss_hal_store_write(uint16_t key, const void *buf, uint16_t len) {
    if (len > STORE_VALUE_MAX || (len && !buf)) {
        return FLOSS_ERR_INVALID;
    }
    store_entry_t *e = store_find(key);
    if (e && e->len == len && memcmp(e->value, buf, len) == 0) {
        return FLOSS_OK; /* unchanged: spare the flash */
    }
    for (unsigned i = 0; !e && i < STORE_ENTRIES; i++) {
        if (!store.img.entries[i].used) {
            e = &store.img.entries[i];
        }
    }
    if (!e) {
        return FLOSS_ERR_NOMEM;
    }
    memset(e, 0, sizeof(*e));
    e->key = key;
    e->used = 1;
    e->len = (uint8_t)len;
    if (len) {
        memcpy(e->value, buf, len);
    }
    return store_commit();
}

floss_err_t floss_hal_store_delete(uint16_t key) {
    store_entry_t *e = store_find(key);
    if (!e) {
        return FLOSS_OK;
    }
    memset(e, 0, sizeof(*e));
    return store_commit();
}

/* ------------------------------------------------------------------------ */
/* Entropy                                                                  */
/* ------------------------------------------------------------------------ */

void floss_hal_random(uint8_t *buf, size_t len) {
    while (len) {
        uint32_t r = get_rand_32();
        size_t n = len < 4 ? len : 4;
        memcpy(buf, &r, n);
        buf += n;
        len -= n;
    }
}

/* ------------------------------------------------------------------------ */
/* Log output                                                               */
/* ------------------------------------------------------------------------ */

void floss_hal_log_write(const char *text, size_t len) {
#if LIB_PICO_STDIO_USB
    if (!stdio_usb_connected()) {
        return; /* nobody listening: drop instead of stalling */
    }
#endif
    stdio_put_string(text, (int)len, false, true);
}

/* ------------------------------------------------------------------------ */
/* Debug pins                                                               */
/* ------------------------------------------------------------------------ */

void floss_hal_debug_pin(uint8_t pin, bool level) {
    if (pin < 4) {
        gpio_put(FLOSS_PICO_DBG_PIN_BASE + pin, level);
    }
}

/* ------------------------------------------------------------------------ */
/* Init                                                                     */
/* ------------------------------------------------------------------------ */

void floss_pico_hal_init(void) {
    if (!hal_lock) {
        hal_lock = spin_lock_init((uint)spin_lock_claim_unused(true));
    }
    store_load();
#if FLOSS_ENABLE_DEBUG_PINS
    for (uint i = 0; i < 4; i++) {
        gpio_init(FLOSS_PICO_DBG_PIN_BASE + i);
        gpio_set_dir(FLOSS_PICO_DBG_PIN_BASE + i, GPIO_OUT);
        gpio_put(FLOSS_PICO_DBG_PIN_BASE + i, 0);
    }
#endif
}
