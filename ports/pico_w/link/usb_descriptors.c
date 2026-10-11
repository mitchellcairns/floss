/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * usb_descriptors.c - USB identity of the link dongle: a composite device
 * with a CDC console and an HID gamepad.
 *
 *   interface 0+1  CDC ACM (IAD)  console / log, 1200-baud reboot request
 *   interface 2    HID            gamepad: examples/gamepad report descriptor
 *                                 (input report 1, output report 2),
 *                                 interrupt IN + OUT, bInterval 1 ms
 *
 * VID 0x1209 is the pid.codes open-source vendor ID. PID 0x0F11 is a
 * PLACEHOLDER (next to the Classic gamepad's 0x0F10): a real product must
 * register its own product ID at https://pid.codes before shipping.
 *
 * The serial number is the flash chip's unique ID, the same string the
 * SDK's USB stdio reports, so a board keeps its serial whether it runs the
 * dongle or another firmware (tools/pico_flash.ps1 -Serial relies on it).
 */
#include "pico/unique_id.h"
#include "tusb.h"

#include "gamepad_state.h"

#define USBD_VID 0x1209u /* pid.codes */
#define USBD_PID 0x0F11u /* placeholder, see above */
#define USBD_BCD_DEVICE 0x0100u
#define USBD_MAX_POWER_MA 250u

enum {
    ITF_CDC = 0, /* + ITF_CDC_DATA */
    ITF_CDC_DATA,
    ITF_HID,
    ITF_COUNT
};

#define EP_CDC_NOTIF 0x81u
#define EP_CDC_OUT   0x02u
#define EP_CDC_IN    0x82u
#define EP_HID_OUT   0x03u
#define EP_HID_IN    0x83u

#define EP_CDC_NOTIF_SIZE 8u
#define EP_CDC_DATA_SIZE  64u
#define HID_POLL_MS       1u /* full speed: bInterval in frames (1 ms) */

enum {
    STR_LANGID = 0,
    STR_MANUFACTURER,
    STR_PRODUCT,
    STR_SERIAL,
    STR_CDC,
    STR_HID,
    STR_COUNT
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_HID_INOUT_DESC_LEN)

/* ------------------------------------------------------------------------ */
/* Device                                                                   */
/* ------------------------------------------------------------------------ */

static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    /* Composite with an IAD (needed for the two-interface CDC function). */
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USBD_VID,
    .idProduct = USBD_PID,
    .bcdDevice = USBD_BCD_DEVICE,
    .iManufacturer = STR_MANUFACTURER,
    .iProduct = STR_PRODUCT,
    .iSerialNumber = STR_SERIAL,
    .bNumConfigurations = 1,
};

const uint8_t *tud_descriptor_device_cb(void) {
    return (const uint8_t *)&desc_device;
}

/* ------------------------------------------------------------------------ */
/* Configuration                                                            */
/* ------------------------------------------------------------------------ */

static const uint8_t desc_configuration[CONFIG_TOTAL_LEN] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_TOTAL_LEN, 0, USBD_MAX_POWER_MA),

    TUD_CDC_DESCRIPTOR(ITF_CDC, STR_CDC, EP_CDC_NOTIF, EP_CDC_NOTIF_SIZE, EP_CDC_OUT, EP_CDC_IN, EP_CDC_DATA_SIZE),

    TUD_HID_INOUT_DESCRIPTOR(ITF_HID, STR_HID, HID_ITF_PROTOCOL_NONE, GAMEPAD_REPORT_DESCRIPTOR_LEN, EP_HID_OUT,
                             EP_HID_IN, CFG_TUD_HID_EP_BUFSIZE, HID_POLL_MS),
};

const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

/* The gamepad's report descriptor, byte for byte the one the Classic HID
 * gamepad registers over the air. */
const uint8_t *tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    return gamepad_report_descriptor;
}

/* ------------------------------------------------------------------------ */
/* Strings                                                                  */
/* ------------------------------------------------------------------------ */

static char serial_str[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];

static const char *const desc_strings[STR_COUNT] = {
    [STR_LANGID] = NULL, /* handled below */
    [STR_MANUFACTURER] = "FLOSS",
    [STR_PRODUCT] = "FLOSS Link Dongle",
    [STR_SERIAL] = serial_str,
    [STR_CDC] = "FLOSS Link Console",
    [STR_HID] = "FLOSS Link Gamepad",
};

#define STR_MAX_CHARS 32

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    static uint16_t desc[1 + STR_MAX_CHARS]; /* header + UTF-16 code units */
    unsigned len;

    if (index == STR_LANGID) {
        desc[1] = 0x0409; /* English (US) */
        len = 1;
    } else {
        if (index >= STR_COUNT) {
            return NULL;
        }
        if (index == STR_SERIAL && !serial_str[0]) {
            pico_get_unique_board_id_string(serial_str, sizeof(serial_str));
        }
        const char *s = desc_strings[index];
        for (len = 0; len < STR_MAX_CHARS && s[len]; len++) {
            desc[1 + len] = (uint8_t)s[len]; /* ASCII -> UTF-16 */
        }
    }
    desc[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2u * len + 2u));
    return desc;
}
