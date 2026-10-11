# Report timing: measurements and tuning

How fast and how *consistently* can a FLOSS gamepad deliver input reports?
This document records the investigation done on real hardware: the
method, the numbers, what each knob does, and the defaults that came out of it.

## Setup

| | |
|---|---|
| Device | Raspberry Pi Pico W (RP2040 @ 125 MHz, CYW43439 radio), `floss_gamepad` firmware |
| Host | Windows 11, Intel Wireless-AC 3168 adapter, Ethernet (Wi-Fi idle) |
| Link | Classic ACL, device is peripheral, active mode (no sniff), encrypted |
| Report | 13-byte input report (ID 1), DATA header, fits a single-slot packet |
| Load | Demo mode: right stick X is a **millisecond counter**, so every report says when it was made |

### Method

`tools/hostprobe` (`floss_hostprobe pads <s>`) reads the controller through
the OS game-input API, spinning on the reading, and decodes the counter axis.
For every report that arrives it knows the host arrival time and the device
time it was generated, which gives:

* **delivered/s** and **coalesced %**: device milliseconds that never reached
  the host as their own report (superseded in the mailbox: latest wins);
* **host inter-arrival**: time between consecutive reports at the host;
* **latency jitter**: host time - device time, with clock drift removed by a
  linear fit, offset so 0 = the fastest delivery seen. This is the variation
  of end-to-end latency, the number that makes a controller feel inconsistent.

`tools/hostprobe/experiment.ps1` flashes, then applies console tuning keys
(see ports/pico_w/README.md), resets device stats, measures and prints the
device's own `floss_stats` next to the host view.

## The ceiling

A Classic peripheral can only transmit when the central polls it. The fastest
poll is every slot pair, **1.25 ms**, so the hard limit is **800 reports/s**.
Windows already polls an HID link at that rate (the QoS request below changed
nothing measurable), so reaching 800/s is about not wasting poll opportunities.

## Findings

### 1. Controller queue depth decides latency

Reports handed to the controller can no longer be replaced by newer state.
With an unlimited in-flight count, ~8 reports sat in the controller and latency
jitter reached p99 27 ms. Capping in-flight reports trades a little throughput
for latency:

| In-flight cap (packets) | Delivered | Jitter p50 | p90 | p99 |
|---|---|---|---|---|
| unlimited | 718/s | 3.0 ms | 6.2 ms | 27 ms |
| 6 | 718/s | 3.1 ms | 3.6 ms | 21 ms |
| 5 | 714/s | 2.0 ms | 2.6 ms | 14 ms |
| **4** | **719/s** | **1.0 ms** | **1.3 ms** | **8.5 ms** |
| 3 | 465/s | 0.8 ms | 1.3 ms | 14 ms |
| 2 | 347/s | 0.8 ms | 1.2 ms | 12 ms |
| 1 | 4/s | - | - | - |

### 2. The CYW43439 reports completions in pairs, late

The Number Of Completed Packets event arrives for **two packets at a time**,
3-8 ms after they left; with a single packet outstanding it waits **~250 ms**.
A cap below 4 therefore starves the link (the stack believes packets are still
queued when they are already delivered). FLOSS models this as a chipset quirk:
`floss_chipset_t.completion_batch` (CYW43 = 2), and the HID cap
`max_inflight` (default 2) is multiplied by it, giving 4 on the Pico W and 2
on controllers that report packets one by one.

### 3. Page scan while connected costs 10 % and triples the tail

Staying connectable while connected makes the radio leave the link for page
scan windows:

| | Delivered | Inter-arrival p99 / max | Jitter p99 |
|---|---|---|---|
| page scan on | 648/s | 8.7 / 30 ms | 22 ms |
| **page scan off** | **719/s** | **6.6 / 18 ms** | **8.5 ms** |

`FLOSS_HID_CONNECTABLE_WHILE_CONNECTED` now defaults to 0. Page scan is
re-enabled when the link drops, so reconnects are unaffected.

### 4. The remaining gaps are the host's radio schedule

With the device tuned, gaps over 3 ms still occur ~17 times per second. Their
timing is periodic: **6.3 ms every ~105 ms** (a 10-slot window every 168
slots, an LE scan schedule), plus occasional runs of ~8.8 ms gaps. Only the
FLOSS device was connected and the host was on Ethernet, so this is the host
adapter's own background scanning (nearby-device discovery). It costs ~6 % of
poll opportunities and is outside the device's control. On Windows, turning
off "Show notifications to connect using Swift Pair" and closing apps that
watch for nearby devices reduces it.

### 5. Bus cost on the Pico W

`floss_stats` now records `transport_send_time` and `transport_poll_time`.
Through the SDK packet API each TX packet cost ~250 us and each RX event
~600 us (worst 1 ms), with the main loop blocked. The CYW43 transport's fast
bus path (`FLOSS_CYW43_FAST_BUS`, default on) reads the controller's ring
indices once, pulls everything pending in one bulk transfer and acknowledges
once:

| | SDK API | Fast path |
|---|---|---|
| RX per event, avg / max | 587 / 1021 us | **269 / 420 us** |
| Timer lateness avg / p99 | 95 / 1024 us | **29 / 512 us** |

### 6. Pacing vs send-on-change

| Mode | Report age avg (device) | Jitter p50 / p90 |
|---|---|---|
| paced 1000 us | 947 us | **0.82 / 1.32 ms** |
| send on change (`report_interval_us = 0`) | **708 us** | 1.27 / 1.77 ms |

Pacing at 1 kHz gives the most even delivery; send-on-change gets data to the
radio sooner but its sends follow the controller's batched completions.
Pacing at exactly the air rate (1250 us) measured the same as 1000 us.

### 7. Flush timeout: not with this controller

A short automatic flush timeout (2-5 ms) discarded most reports and produced
multi-second stalls on the host with the CYW43439. Leave `flush_timeout_ms`
at 0; the in-flight cap already bounds staleness.

## Result: defaults

| Setting | Default | Why |
|---|---|---|
| `max_inflight` | 2 (x `completion_batch`) | finding 1, 2 |
| `FLOSS_HID_CONNECTABLE_WHILE_CONNECTED` | 0 | finding 3 |
| `report_interval_us` (example) | 1000 | finding 6 |
| `qos_latency_us` (example) | 1250 | harmless; helps hosts that poll slowly by default |
| `flush_timeout_ms` | 0 | finding 7 |
| `FLOSS_CYW43_FAST_BUS` | 1 | finding 5 |

Measured with these defaults over 15 s:

| Metric | Value |
|---|---|
| Delivered | **715-719 reports/s** (90 % of the 800/s ceiling) |
| Host inter-arrival | p50 **1.25 ms**, p90 **1.30 ms**, p99 6.6 ms |
| Latency jitter | p50 **0.8 ms**, p90 **1.3 ms**, p99 8.5 ms |
| Device report age (mailbox -> bus) | avg 0.95 ms, max 1.8 ms |

## LE (HID over GATT)

Same board with the `floss_le_gamepad` firmware, paired from Windows through
FLOSS's SMP (LE Secure Connections, just works, controller-assisted P-256).
Windows settles on a **15 ms connection interval** for this device, so the
ceiling is 66.7 reports/s; LE moves data only at connection events.

| Configuration | Delivered | Host inter-arrival p50 / p90 / p99 | Pattern |
|---|---|---|---|
| send on change, 1 in flight | 65/s | 15 / 29 / 44 ms | pairs then empty events |
| paced per event, 1 in flight | 65/s | 21 / 29 / 44 ms | same: pacer stalls |
| **paced per event, 2 in flight** | **66.7/s** | **15.00 / 15.09 / 29.8 ms** | one report every event |

Why: a notification is acknowledged only at the *next* connection event, and
the CYW43439 reports completions in pairs, so a completion arrives ~2
intervals after sending. With one packet in flight the stack skips events.
Defaults now: `max_inflight = 2` (x `completion_batch`) and the example paces
with `FLOSS_HOGP_INTERVAL_CONN` (one fresh report per connection interval,
following interval updates). Latency jitter: p50 1.1 ms, p90 1.9 ms.

Bonded reconnection: hosts do not rewrite notification subscriptions
(CCCDs) after reconnecting to a bonded device, so FLOSS stores them per bond
(`floss_gatt_cccd_sync`, store keys from `FLOSS_GATT_STORE_KEY_CCCD_BASE`);
after a device reboot reports resume without any host action.

Classic vs LE for a controller: Classic polls every 1.25 ms (up to ~720
reports/s measured, 800/s ceiling) while LE is bounded by the connection
interval the host grants (15 ms here). For lowest latency use Classic; LE is
for hosts or power budgets that require it.

## Reproduce

```powershell
cd tools/hostprobe; dotnet build -c Release -o out; cd ../..
powershell -File tools/hostprobe/experiment.ps1 -Flash -Configs m -Seconds 15
# compare knobs live: 0-8 in-flight cap, P page scan, I interval, Q QoS, F flush
powershell -File tools/hostprobe/experiment.ps1 -Configs 4,P,IIIII -Seconds 10
```

## Ideas not yet explored

* Measure absolute latency with an output-report round trip (host writes
  rumble, device echoes it in the next input report).
* Batch several queued TX packets into one ring write on the CYW43.
* Run the application on the second RP2040 core so sampling never waits on
  the bus (`floss_hid_set_input` is already cross-core safe).
