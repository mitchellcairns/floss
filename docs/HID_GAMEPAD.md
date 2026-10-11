# Building a HID gamepad with FLOSS

This guide covers the Classic HID device role (`include/floss/floss_hid.h`).
For HID over GATT on LE see [LE.md](LE.md). The reference application is
`examples/gamepad` (built for the Pico W as `floss_gamepad`); measured
timing and the reasoning behind every default are in
[PERFORMANCE.md](PERFORMANCE.md).

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
cfg.qos_latency_us = 1250;               /* ask the host to poll every 2 slots (optional) */
/* cfg.max_inflight stays 2: see section 5 */
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

Measured on a Pico W (PERFORMANCE.md, finding 6): pacing at 1000 us gives the
most even delivery (jitter p50/p90 0.82/1.32 ms); send-on-change gets data to
the radio sooner (report age 708 vs 947 us) but follows the controller's
batched completions. Pacing at the air rate (1250 us) measured the same as
1000 us.

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

### Recommended settings

From [PERFORMANCE.md](PERFORMANCE.md) ("Result: defaults"):

| Setting | Value | Why |
|---|---|---|
| `max_inflight` | 2 (default) | Reports queued in the controller cannot be superseded; the cap bounds latency |
| `report_interval_us` | 1000 | Most even delivery |
| `qos_latency_us` | 1250 | Harmless; helps hosts that poll slowly by default |
| `flush_timeout_ms` | 0 (default) | Short flush timeouts stalled the CYW43439 |
| `sniff_max_interval` | 0 (default) | Active mode: lowest latency |
| `FLOSS_HID_CONNECTABLE_WHILE_CONNECTED` | 0 (default) | Page scan steals air time |

Result on a Pico W with Windows: ~715-719 reports/s (90 % of the 800/s
ceiling of a 1.25 ms poll), jitter p50 0.8 ms, p99 8.5 ms.

### In-flight cap (`max_inflight`)

The most important latency setting. A report handed to the controller can
no longer be replaced by newer state, so every report queued there adds a
full air round of delay. `max_inflight` caps un-acknowledged input reports
on the link (0 = as many as the controller has buffers; default 2). The
effective cap is multiplied by the chipset's `completion_batch`
(`floss_hci_completion_batch()`): the CYW43439 reports completions two at a
time, a few ms late, so the Pico W runs with 4; a controller that reports
packets one by one runs with 2. Too low starves the link (an effective cap
of 1 packet on the Pico W delivered 4 reports/s), too high grows jitter
(unlimited: p99 27 ms).
Change at runtime with `floss_hid_set_max_inflight()`.

### Poll latency (`qos_latency_us`)

A Classic peripheral transmits only when the central polls it. If
`qos_latency_us` is non-zero, HID calls `floss_gap_qos_setup(handle,
qos_latency_us, 0)` after connecting, asking the central for that poll
interval (1250 us = every 2 slots). Windows already polls HID links at
1.25 ms, so it changed nothing measurable there; other hosts default to
much slower polling.

### Page scan while connected

`FLOSS_HID_CONNECTABLE_WHILE_CONNECTED` (default 0, override like any
config macro) keeps page scan on while a host is connected, so a second
host can reach the device. Page scan windows take the radio away from the
link: measured 648 instead of 719 reports/s and p99 jitter 22 instead of
8.5 ms. Page scan is re-enabled when the link drops, so reconnects are
unaffected.

### Flush timeout

`cfg.flush_timeout_ms` (or `floss_gap_set_flush_timeout()`). Input reports are
sent FLUSHABLE.

| Value | Effect |
|---|---|
| 0 (default) | Reliable: the controller retransmits until delivered. A fade delays every later report. |
| Finite (e.g. 2-3x the report interval) | The controller drops a report it could not deliver in time; the next, fresher report goes out instead. Best for continuous state. |

Output reports and control traffic are unaffected (not flushable). With the
CYW43439 a 2-5 ms flush timeout discarded most reports and caused
multi-second stalls on the host; keep 0 there and let `max_inflight` bound
staleness.

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

### Classic or LE?

| | Classic HID | LE HID over GATT |
|---|---|---|
| Transmit opportunity | Every central poll, down to 1.25 ms (800/s ceiling) | Once per connection interval (Windows grants 15 ms: 66.7/s) |
| Measured | ~720 reports/s, jitter p50 0.8 ms | One report per connection event, jitter p50 1.1 ms |
| Use when | Lowest latency matters | The host or the power budget requires LE |

### Measuring: the millisecond-counter axis

In demo mode `examples/gamepad` (and `examples/le_gamepad`) put a
millisecond counter on the right stick X axis (`Z`): every report says when
it was built. `tools/hostprobe` (`floss_hostprobe pads <seconds>`) decodes it
on the host to get delivered reports/s, coalescing, inter-arrival and
latency jitter. On the Pico W console, keys `0`-`8`, `Q`, `F`, `I`, `P`
change `max_inflight`, QoS, flush timeout, interval and page scan live
(see [DEBUGGING.md](DEBUGGING.md), section 6).

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
