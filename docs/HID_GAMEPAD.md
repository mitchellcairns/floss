# Building a HID gamepad with FLOSS

This guide covers the Classic HID device role (`include/floss/floss_hid.h`).
For HID-over-GATT on LE see [LE.md](LE.md).

## 1. Minimal setup

```c
static const floss_hid_callbacks_t k_cb = {
    .on_connected     = app_connected,
    .on_disconnected  = app_disconnected,
    .on_output_report = app_rumble,      /* optional */
    .on_report_tick   = app_sample,      /* optional, paced mode */
};

floss_hid_config_t cfg;
floss_hid_config_defaults(&cfg);         /* gamepad-friendly defaults */
cfg.name = "FLOSS Gamepad";
cfg.report_descriptor = k_desc;
cfg.report_descriptor_len = sizeof k_desc;
cfg.subclass = FLOSS_HID_SUBCLASS_GAMEPAD;
cfg.report_interval_us = 1000;           /* 1 kHz paced; 0 = send on change */
floss_hid_init(&cfg, &k_cb);             /* after floss_init(), before floss_start() */
```

`floss_hid_init()` registers PSM 0x0011 (control) and 0x0013 (interrupt),
the HID and Device ID SDP records, the Class of Device (derived from
`subclass` when `class_of_device == 0`) and the EIR. `cfg`, the descriptor
and the callbacks must stay valid for the life of the stack.

## 2. Report descriptor design

Generic hosts (Windows, Linux, Android, macOS) map a gamepad well when the
descriptor uses standard usages:

- Top-level collection **Generic Desktop / Game Pad** (0x01 / 0x05). Joystick
  (0x04) also works; Game Pad is the better default.
- **Buttons** from the Button page (0x09), 1-bit each, contiguous.
- **Sticks** as Generic Desktop X, Y (left) and Z, Rz (right) with a
  logical range (0..255 or -32768..32767).
- **Hat switch** (0x39) with a null state for the D-pad.
- **Triggers** as Simulation Controls Accelerator / Brake (0x02 / 0xC4, 0xC5),
  or as Rx / Ry.
- Pad to whole bytes with constant fields.
- Keep the input report small (see packet sizes in DEBUGGING.md: <= 21 bytes
  of payload fits a single-slot DH1 packet).

Example (report ID 1 input = 9 bytes, report ID 2 output = 2 bytes rumble):

```c
static const uint8_t k_desc[] = {
    0x05, 0x01,        /* Usage Page (Generic Desktop)            */
    0x09, 0x05,        /* Usage (Game Pad)                        */
    0xA1, 0x01,        /* Collection (Application)                */
    0x85, 0x01,        /*   Report ID (1)                         */
    0x05, 0x09,        /*   Usage Page (Button)                   */
    0x19, 0x01,        /*   Usage Minimum (1)                     */
    0x29, 0x10,        /*   Usage Maximum (16)                    */
    0x15, 0x00,        /*   Logical Minimum (0)                   */
    0x25, 0x01,        /*   Logical Maximum (1)                   */
    0x75, 0x01,        /*   Report Size (1)                       */
    0x95, 0x10,        /*   Report Count (16)                     */
    0x81, 0x02,        /*   Input (Data,Var,Abs)        2 bytes   */
    0x05, 0x01,        /*   Usage Page (Generic Desktop)          */
    0x09, 0x39,        /*   Usage (Hat switch)                    */
    0x15, 0x00,        /*   Logical Minimum (0)                   */
    0x25, 0x07,        /*   Logical Maximum (7)                   */
    0x75, 0x04,        /*   Report Size (4)                       */
    0x95, 0x01,        /*   Report Count (1)                      */
    0x81, 0x42,        /*   Input (Data,Var,Abs,Null)             */
    0x75, 0x04,        /*   Report Size (4)                       */
    0x95, 0x01,        /*   Report Count (1)                      */
    0x81, 0x03,        /*   Input (Const) padding       1 byte    */
    0x09, 0x30,        /*   Usage (X)                             */
    0x09, 0x31,        /*   Usage (Y)                             */
    0x09, 0x32,        /*   Usage (Z)                             */
    0x09, 0x35,        /*   Usage (Rz)                            */
    0x15, 0x00,        /*   Logical Minimum (0)                   */
    0x26, 0xFF, 0x00,  /*   Logical Maximum (255)                 */
    0x75, 0x08,        /*   Report Size (8)                       */
    0x95, 0x04,        /*   Report Count (4)                      */
    0x81, 0x02,        /*   Input (Data,Var,Abs)        4 bytes   */
    0x05, 0x02,        /*   Usage Page (Simulation Controls)      */
    0x09, 0xC4,        /*   Usage (Accelerator)                   */
    0x09, 0xC5,        /*   Usage (Brake)                         */
    0x95, 0x02,        /*   Report Count (2)                      */
    0x81, 0x02,        /*   Input (Data,Var,Abs)        2 bytes   */
    0x85, 0x02,        /*   Report ID (2)                         */
    0x06, 0x00, 0xFF,  /*   Usage Page (Vendor Defined 0xFF00)    */
    0x09, 0x01,        /*   Usage (0x01)                          */
    0x95, 0x02,        /*   Report Count (2)                      */
    0x91, 0x02,        /*   Output (Data,Var,Abs) rumble 2 bytes  */
    0xC0,              /* End Collection                          */
};
```

### Report IDs

- If the descriptor declares **no** `Report ID` items, use `report_id = 0`
  everywhere; no ID byte is sent.
- If it declares any, every report has an ID and FLOSS prepends it on the
  wire. Pass data **without** the ID byte to `floss_hid_set_input()` and in
  `on_get_report`.
- The mailbox tracks up to `FLOSS_HID_MAX_INPUT_IDS` input IDs; each payload
  is at most `FLOSS_HID_MAX_REPORT` bytes.

## 3. The input mailbox

```c
uint8_t r[9];
pack_state(r);
floss_hid_set_input(1, r, sizeof r);   /* any context: main loop, ISR, core 1 */
```

- **Latest wins.** Each report ID has one slot. A new write replaces an unsent
  one (counted in `hid_reports_coalesced`). The host always gets the newest
  state; a gamepad never queues stale input.
- **Never blocks, never allocates.** Safe from ISRs and the other core
  (seqlock + a few instructions under `floss_hal_critical_enter`).
- `floss_hid_send_input_now()` bypasses the mailbox (stack context only) and
  returns `FLOSS_ERR_BUSY` if no controller credit is free. Use it for one-off
  reports, not for the main stream.
- If `on_get_report` is NULL, GET_REPORT(INPUT) is answered from the mailbox.

## 4. Pacing vs on-change

| `report_interval_us` | Behaviour | Use for |
|---|---|---|
| `0` | Send on change as soon as a controller ACL credit is free | Lowest latency for sparse input (buttons) |
| `> 0` | Drift-free timer every interval; send if changed (or always with `send_unchanged`) | Stable cadence, analog sticks, hosts that expect a steady stream |

Change at runtime with `floss_hid_set_report_interval()`.

### Just-in-time sampling with `on_report_tick`

In paced mode `on_report_tick()` runs immediately before each report is
built: the last moment to call `floss_hid_set_input()` for this frame.
Sampling there minimises report age (`hid_report_age` stat) and avoids
polling inputs faster than you send them:

```c
static void app_sample(void) {
    uint8_t r[9];
    read_buttons_and_adc(r);              /* keep this short: it runs in the stack */
    floss_hid_set_input(1, r, sizeof r);
}
```

If sampling is slow (I2C, many ADC channels), sample continuously on another
core and just publish from there instead.

## 5. Latency trade-offs

### Flush timeout

`cfg.flush_timeout_ms` (or `floss_gap_set_flush_timeout()`). Input reports are
sent FLUSHABLE.

| Value | Effect |
|---|---|
| 0 (default) | Reliable: the controller retransmits until delivered. A fade delays every later report. |
| Finite (e.g. 2-3x the report interval) | The controller drops a report it could not deliver in time; the next, fresher report goes out instead. Best for continuous state. |

Output reports and control traffic are unaffected (not flushable).

### Sniff vs active

| Mode | Latency floor | Power |
|---|---|---|
| Active (`sniff_max_interval = 0`) | The central's poll interval (a few slots for a typical HID host) | Highest |
| Sniff, max interval N slots | N x 0.625 ms (e.g. 6 slots = 3.75 ms) | Lower |

In sniff mode the device can only transmit at sniff anchor points: **the latency
floor is the sniff interval**, and reports produced between anchors leave
together. The host may impose its own sniff parameters regardless of what we
request; check `current_mode` / `sniff_interval` in the stats.
`floss_gap_sniff_subrating()` reduces power further when idle at the cost of
first-report latency.

### Supervision timeout

How long a silent link survives before both sides drop it (default 0x7D00 slots
= 20 s). `floss_gap_set_supervision_timeout()` only takes effect when we are
central on the link; otherwise the host decides. Shorter = faster detection of a
lost host (and faster reconnect), but more spurious drops under interference.
Advertise a preference via the SDP `supervision_timeout` field.

## 6. Connection lifecycle

### Reconnect

- With a bond and `auto_reconnect`, HID pages the most recent bond
  (`floss_gap_bond_last()`) at start-up, opening control then interrupt.
- `floss_hid_connect(NULL)` reconnects on demand (e.g. a button press);
  `floss_hid_connect(&addr)` targets a specific host.
- Page Timeout (0x04) means the host is not listening: the device stays
  connectable and the host can connect instead.
- With no bond and `auto_discoverable`, the device becomes discoverable.
- `floss_hid_state()` / `floss_hid_connected()` tell you where you are.

### Virtual cable unplug

`floss_hid_unplug()` sends VIRTUAL_CABLE_UNPLUG, disconnects and deletes the
bond: "forget this host". When the host sends it, `on_control` receives
`FLOSS_HID_CTRL_VIRTUAL_CABLE_UNPLUG` and the bond is removed. Hosts honour this
only when `virtual_cable` is set (default true). Suspend / exit suspend also
arrive via `on_control`: a good moment to slow down pacing.

## 7. Device ID record and VID/PID

`vendor_id_source`, `vendor_id`, `product_id`, `version` produce a Device ID
(PnP Information, UUID 0x1200) record. `vendor_id_source = 0` disables it.

| Source | Meaning |
|---|---|
| 1 | Vendor ID assigned by the radio standard's consortium |
| 2 | USB Implementers Forum vendor ID |

Hosts use VID/PID to select drivers and mappings. For open hardware,
[pid.codes](https://pid.codes) offers free PIDs under the USB VID `0x1209`
(`vendor_id_source = 2, vendor_id = 0x1209`). pid.codes reserves a few test
PIDs for development; request your own PID before shipping.

## 8. Host notes

| Host | Notes |
|---|---|
| Windows | Generic gamepads appear as HID game controllers (DirectInput / raw input). Games using XInput need a separate mapping layer. Expect the host to request sniff after idle. |
| Linux | The kernel HID host binds any descriptor; the generic driver maps standard usages to evdev axes/buttons. VID/PID selects special drivers. |
| Android | Uses Linux input with key layout files per VID/PID; standard Game Pad usages map to the gamepad layout. Triggers as Accelerator/Brake or Rx/Ry are recognised. |
| macOS | Generic HID gamepads are supported; the game controller framework recognises only some devices, others are visible as plain HID. |
| Consoles | Typically require specific descriptors, identities, and sometimes authentication. FLOSS provides the generic HID mechanisms; reproducing a particular vendor's controller identity is the integrator's responsibility (see [LEGAL.md](LEGAL.md)). |

Pairing works with all of the above using the default security settings
(Secure Simple Pairing "just works", legacy PIN `0000` fallback for very old
hosts).

## 9. Keyboards, mice and boot protocol

The same HID layer serves keyboards, mice and combo devices;
`examples/keyboard/` is a complete keyboard + mouse combo (subclass
`FLOSS_HID_SUBCLASS_COMBO`, LEDs, typing / pointer demo).

**Boot protocol.** Set `cfg.boot_device = true` and a host that cannot parse
report descriptors (PC firmware set-up screens, minimal hosts) may switch the
device to boot protocol with SET_PROTOCOL. The fixed boot formats (USB HID
1.11 boot appendix) are:

| Report | ID | Payload |
|---|---|---|
| Keyboard input | 1 | modifiers, reserved, 6 key codes (8 bytes) |
| Keyboard output | 1 | LEDs (1 byte) |
| Mouse input | 2 | buttons, X, Y, optional wheel (3+ bytes) |

The subclass decides which boot reports exist (KEYBOARD, MOUSE or both for
COMBO). Boot reports have their own mailbox slots:

```c
floss_hid_set_input(1, report, 8);                           /* report protocol */
floss_hid_set_boot_input(FLOSS_HID_BOOT_KEYBOARD, report, 8); /* boot protocol */
```

The stack sends only the slots of the protocol the host selected and, after a
switch, sends the newest state of the new protocol once. Publishing both
formats on every change is the simplest correct application; `on_set_protocol`
and `floss_hid_protocol()` are there if you prefer to publish one. Every
connection starts in report protocol; SET_PROTOCOL(boot) on a device without
`boot_device` is answered with ERR_INVALID_PARAMETER. In boot protocol the ID
byte is always on the wire, so the LED report arrives at `on_output_report`
as report ID 1 and GET_REPORT(input) answers from the boot slots.

HID over GATT mirrors this: `cfg.boot_keyboard` / `cfg.boot_mouse` add the
Boot Keyboard Input/Output and Boot Mouse Input characteristics (plus Protocol
Mode), filled with `floss_hogp_set_boot_input()`.

**Latest wins vs keys and relative motion.** The mailbox sends the newest
state, so a state replaced before the link carried it is never sent. Hold each
key state for at least a few milliseconds (the example uses 30 ms per press /
release when typing), publish relative mouse motion at most once per
report period, and end a movement with a zero-motion report: reconnects and
protocol switches re-send the current state.

**Large reports.** Reports larger than the control channel MTU are fragmented
with HIDP DATC packets in both directions (SET_REPORT reassembly up to report
ID + `FLOSS_HID_MAX_REPORT` bytes, GET_REPORT answers split to the host's MTU).
