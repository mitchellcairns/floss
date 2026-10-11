# Debugging FLOSS

## 1. The toolbox

All tools are declared in `include/floss/floss_debug.h`. Use the cheapest one
that answers your question.

| Tool | API | Cost | Use it when |
|---|---|---|---|
| **Logs** | `FLOSS_LOGE/W/I/D/T(MOD, ...)`, `FLOSS_HEXDUMP` | vsnprintf + HAL write per line | Following control flow: bring-up, pairing, channel setup. Not for per-packet timing. |
| **Snoop** | `floss_snoop_set_hook()`, `floss_snoop_log_hook` | One callback per HCI packet | Protocol questions: "what did the host actually send?" Wireshark decodes everything. |
| **Stats** | `floss_stats()`, `floss_stats_dump()`, `floss_stats_reset()` | ~O(1) per event, always on | "Why is latency / throughput inconsistent?" Start here for performance. |
| **Trace** | `FLOSS_TRACE(id, arg)`, `floss_trace_freeze()`, `floss_trace_dump()`, `floss_trace_read()` | ~10 instructions | Exact ordering and timing in hot paths, after the fact. Freeze when the bug happens, then dump. |
| **Debug pins** | `FLOSS_DBG_PIN(n, level)` (`FLOSS_ENABLE_DEBUG_PINS=1`) | One GPIO write | End-to-end latency with a logic analyser; correlating with the radio or the host. |

## 2. Logs

Line format: `[sssss.uuuuuu] MOD L message`.

- **Compile-time ceiling**: `FLOSS_LOG_LEVEL` (default `FLOSS_LOG_INFO`).
  Anything above it compiles to nothing.
- **Runtime per-module level**: `floss_log_set_level(FLOSS_MOD_HCI, FLOSS_LOG_DEBUG)`,
  `floss_log_set_level_all(level)`, `floss_log_get_level(mod)`. Cannot exceed the ceiling.

| Module | Covers |
|---|---|
| `FLOSS_MOD_CORE` | init, buffers, timers |
| `FLOSS_MOD_TRANSPORT` | driver open, framing errors, resyncs |
| `FLOSS_MOD_HCI` | commands, events, credits, connection table |
| `FLOSS_MOD_GAP` | scan, pairing, bonds, link policy |
| `FLOSS_MOD_L2CAP` | signaling, channel state |
| `FLOSS_MOD_SDP` | requests and responses |
| `FLOSS_MOD_HID` | HIDP transactions, reconnect, mailbox |
| `FLOSS_MOD_APP` | your code |

(LE, SMP and RFCOMM get their own modules when they land.)

Typical session: build with `FLOSS_LOG_LEVEL=FLOSS_LOG_DEBUG`, run at INFO,
raise one module to DEBUG while investigating it.

## 3. Capturing a Wireshark trace

1. Install the hook early (before `floss_start()` to capture bring-up):
   ```c
   floss_snoop_set_hook(floss_snoop_log_hook);
   ```
   Each packet becomes one log line: `SNOOP <t_us> <R|T> <type> <hex...>`.
2. Capture the log output:
   - **After the fact**: save the serial output to a file, then convert:
     ```sh
     python tools/floss_snoop.py capture.log -o capture.btsnoop
     ```
   - **Live**: `tools/floss_monitor.py` reads the serial port, prints normal log
     lines and writes the SNOOP lines into a capture as they arrive.
   Run either with `--help` for the exact options.
3. Open the `.btsnoop` file in Wireshark. Useful display filters:
   `bthci_evt`, `bthci_acl`, `btl2cap`, `btsdp`, `bthid`,
   `bthci_evt.code == 0x13` (Number Of Completed Packets).

The log hook is convenient but costs a log line per packet. For heavy traffic
write your own `floss_snoop_fn` that copies into a RAM ring and drains it from
idle time; keep the hook fast, it runs in the hot path.

## 4. Reading `floss_stats_dump()`

Call it from a button press, a serial command, or every few seconds while
testing. `floss_stats_reset()` starts a fresh measurement window.

### Histograms (`floss_hist_t`)

Every histogram has `count`, `min_us`, `max_us`, `sum_us` (mean = sum / count)
and 16 log2 buckets, fine-grained at the low end where sub-millisecond
behaviour lives:

| Bucket | Range |
|---|---|
| 0 | < 8 us |
| i (1..14) | [4 << i, 8 << i) us, i.e. 8-16 us, 16-32 us, ..., 65.5-131 ms |
| 15 | >= ~131 ms (overflow) |

`floss_hist_percentile(h, 99)` returns the upper edge of the bucket containing
that percentile (an upper bound, within 2x), clamped to the observed min/max.

### Transport / HCI

| Field | Meaning | Healthy | If not |
|---|---|---|---|
| `tx_cmd`, `rx_evt` | Commands sent, events received | | |
| `tx_acl`, `rx_acl` | ACL packets each way | | |
| `tx_bytes`, `rx_bytes` | Bytes each way (HCI level) | | |
| `transport_busy` | Driver returned BUSY on send | 0 or rare | Bus/UART too slow, no flow control, or driver queue too small |
| `transport_errors` | Driver returned an error | 0 | Wiring, baud rate, firmware download |
| `cmd_timeouts` | Controller never answered a command | 0 | See "controller hangs" |
| `cmd_failures` | Command completed with non-zero status | Few at bring-up (unsupported optional commands) | Check log for the opcode and `floss_hci_status_str()` |
| `acl_credit_stalls` | Wanted to send ACL, no controller buffers | Low | Link saturated or air-time trouble; see `acl_ack_latency` |
| `buf_alloc_failures` | Pool exhausted | 0 | Raise `FLOSS_BUF_COUNT`; check `floss_buf_low_watermark()` |
| `rx_dropped` | Malformed or no room for reassembly | 0 | Peer sends PDUs larger than `FLOSS_BUF_SIZE` / MTU, or pool exhausted |
| `cmd_latency` | Command sent -> Command Complete/Status | p99 < ~2 ms | Slow transport or busy controller (WLAN coexistence on combo chips) |
| `acl_ack_latency` | ACL handed to transport -> Number Of Completed Packets | Active mode: ~1-3 ms. Sniff: ~ sniff interval | Retransmissions, interference, sniff, host scheduling. Long tail = air trouble |

### Link

| Field | Meaning |
|---|---|
| `connections`, `disconnections` | Link counts. Disconnections >> 1 per session = instability. |
| `last_disconnect_reason` | HCI reason: 0x08 Connection Timeout (supervision, radio lost), 0x13 remote user terminated, 0x16 local host terminated, 0x05 auth failure. |
| `mode_changes` | Active <-> sniff transitions. Frequent changes = host or our policy toggling. |
| `current_mode` | 0 active, 2 sniff. |
| `sniff_interval` | Slots (0.625 ms) while in sniff. Latency floor. |

### HID

| Field | Meaning |
|---|---|
| `hid_reports_sent` | Input reports handed to HCI. |
| `hid_reports_coalesced` | Mailbox overwritten before sending. Normal when the app updates faster than the link; "lost" intermediate states are intentional. |
| `hid_send_deferred` | Pacing tick found no ACL credit. High = link saturated (sending faster than the air delivers). |
| `hid_output_reports` | Output reports received (rumble, LEDs). |
| `hid_report_age` | `floss_hid_set_input()` -> transport. Your share of end-to-end latency inside the device. With pacing and `on_report_tick`, should be tiny. |
| `hid_send_interval` | Spacing between consecutive reports. Should cluster tightly at `report_interval_us`. Bimodal = bursts. |

### Run loop

| Field | Meaning |
|---|---|
| `polls` | `floss_poll()` calls. |
| `poll_gap` | Time between polls. p99 large = the application starves the stack. |
| `timer_lateness` | How late timers fired. Tracks `poll_gap` and your sleep strategy. |

## 5. Troubleshooting recipes

### Reports arrive in bursts

Symptoms: `hid_send_interval` bimodal; host sees several reports at once,
then nothing.

1. `current_mode == 2`? The link is in **sniff**: the controller can only
   transmit at sniff anchor points, so reports produced between anchors leave
   together. Check `sniff_interval`; set `sniff_max_interval = 0` in
   `floss_hid_config_t` or call `floss_gap_sniff_exit()`. The host may still
   request sniff.
2. `acl_credit_stalls` / `hid_send_deferred` rising: the link cannot keep up
   at this rate. Reduce report rate or size.
3. `poll_gap` p99 large: the stack is not running often enough; reports are
   produced late, then flushed together.

### Latency spikes every N ms

- Spikes at a fixed period that matches `sniff_interval * 0.625 ms`: sniff
  mode (see above). With sniff subrating, spikes can grow by the subrate
  factor after idle periods.
- Spikes at ~ tens of ms regardless of mode: host-side scanning (the host
  searching for other devices) or WLAN coexistence on combo chips. Correlate
  with `acl_ack_latency` tail and timestamps in a snoop capture.
- Spikes matching your log rate: the log sink blocks (see below).

### Throughput or rate inconsistent

Work down this list; each item names the stat that confirms it.

| Cause | Confirm with | Fix |
|---|---|---|
| Controller credits exhausted | `acl_credit_stalls`, `floss_hci_acl_credits()`, `acl_total` | Fewer/smaller packets; let the mailbox coalesce instead of queuing |
| Retransmissions / interference | `acl_ack_latency` long tail, RSSI (`floss_gap_read_rssi`) | Antenna, distance, 2.4 GHz congestion (WLAN, other links); shorter flush timeout |
| Flush timeout too long / infinite | Old reports delivered late after a fade | Set `flush_timeout_ms` (e.g. 2-3x report interval); reports become FLUSHABLE drops |
| Flush timeout too short | Missing reports at the host | Increase it; 0 = reliable |
| Report doesn't fit a 1-slot packet | Payload size vs table below | Shrink the report or accept multi-slot packets |
| Poll starvation | `poll_gap`, `timer_lateness` p99 | Call `floss_poll()` more often; move heavy app work off the stack context |
| Log sink blocking | Spikes line up with log lines; USB serial with no reader | Make `floss_hal_log_write()` drop; lower log levels |
| Transport back-pressure | `transport_busy` | Faster UART/bus, hardware flow control |

**Packet sizes.** An input report occupies `4 (L2CAP) + 1 (HIDP) + 1 (ID) +
payload` bytes of ACL payload. Baseband packet capacity (bytes of payload):

| Packet | Slots | Max payload |
|---|---|---|
| DM1 / DH1 | 1 | 17 / 27 |
| 2-DH1 / 3-DH1 | 1 | 54 / 83 |
| DM3 / DH3 | 3 | 121 / 183 |
| 2-DH3 / 3-DH3 | 3 | 367 / 552 |
| DH5 / 2-DH5 / 3-DH5 | 5 | 339 / 679 / 1021 |

A report with up to 21 bytes of payload fits a DH1; up to 48 fits a 2-DH1.
Exceeding one slot means the controller needs multi-slot packets (which the
host may not allow) or more air time per report.

### Host does not see the device

1. `FLOSS_GAP_EVT_READY` received? If not, see "controller hangs".
2. `floss_gap_set_scan(true, true)` called (or `auto_discoverable` with no
   bond)? Snoop: look for Write Scan Enable with value 0x03.
3. Class of Device: hosts filter "add device" lists by class. Gamepad =
   `FLOSS_COD_GAMEPAD` (0x002508). Snoop: Write Class Of Device.
4. EIR: name and service UUIDs (0x1124 for HID). Snoop: Write Extended Inquiry
   Response. A missing name shows as an address only on some hosts.
5. Controller address all zeros or duplicated across boards: use
   `floss_hci_set_addr_override()` (CYW43: `floss_cyw43_default_addr()`).

### Pairing loops (pairs, disconnects, pairs again)

- **Stale link key**: the device kept a bond but the host forgot it (or vice
  versa). Authentication fails with 0x06 *PIN or Key Missing* or 0x05
  *Authentication Failure*. GAP deletes the stale bond automatically on 0x06
  so the next attempt pairs from scratch (the GAP log says so). For 0x05, or
  if you want a stricter policy, call `floss_gap_bond_delete(&addr)` from your
  listener on `FLOSS_GAP_EVT_AUTHENTICATED` / `FLOSS_GAP_EVT_PAIRING_FAILED`.
  Also remove the device on the host side, so both ends start clean.
- **Storage not persisting**: `floss_hal_store_write()` failing; after reset
  the device has no key and the host does. Check GAP logs at boot for the bond
  count (`floss_gap_bond_count()`).
- **Security mismatch**: `mitm_required = true` with NoInputNoOutput cannot
  succeed. Keep the defaults from `floss_gap_security_defaults()`.

### Reconnect fails

- `FLOSS_GAP_EVT_CONNECT_FAILED` with status 0x04 *Page Timeout*: the host is
  not page-scanning (off, out of range, or its radio is off). The device
  stays connectable; the host can connect instead.
- Host connects but channels never open: host deleted the bond -> see pairing
  loops.
- Slow reconnect when the host initiates: tighten page scan with
  `floss_gap_set_page_scan(0x0100, 0x0012, true)` while waiting.
- Some hosts only accept a reconnect from devices with `reconnect_initiate`
  and `normally_connectable` set consistently in the SDP record.

### Controller hangs

- `cmd_timeouts > 0`: the log names the opcode. At bring-up this usually means
  the transport is wrong (baud rate, framing, firmware not loaded): enable
  `FLOSS_MOD_TRANSPORT` DEBUG and check `transport_errors` and H4 resyncs.
- Hangs after a specific vendor command: check the chipset `init_script`.
- `FLOSS_GAP_EVT_HARDWARE_ERROR`: the controller crashed. Restart with
  `floss_start()` after a transport reset, and capture a snoop of what preceded
  it.
- `cmd_latency` max close to `FLOSS_HCI_CMD_TIMEOUT_US`: the controller is
  overloaded (combo chip with busy WLAN).

## 6. Measuring end-to-end latency with debug pins

Build with `-DFLOSS_ENABLE_DEBUG_PINS=1` and map the logical pins in
`floss_hal_debug_pin()`.

| Pin | Meaning |
|---|---|
| `FLOSS_DBG_PIN_POLL` (0) | High while inside `floss_poll()` |
| `FLOSS_DBG_PIN_HID_TX` (1) | Pulse when an input report is handed to HCI |
| `FLOSS_DBG_PIN_TRANS_TX` (2) | High during transport `send()` |
| `FLOSS_DBG_PIN_TRANS_RX` (3) | High while delivering a received packet |

Setup:

1. Wire a physical button to a GPIO and to logic-analyser channel A.
2. Channels B-E: the four debug pins.
3. On the host, run a program that toggles something observable (screen
   flash with a photodiode, or a GPIO on a host-side board via a USB HID
   reader) when it sees the button in a report: channel F.

Read the segments:

| Segment | Measures |
|---|---|
| A edge -> HID_TX pulse | Device-side: sampling + mailbox + pacing wait (compare `hid_report_age`) |
| HID_TX -> TRANS_TX end | Stack + transport (should be tens of us) |
| TRANS_TX -> next TRANS_RX (Number Of Completed Packets) | Air time + ack (compare `acl_ack_latency`) |
| A edge -> F edge | Total end-to-end, including the host's input stack |

Collect a few thousand presses and look at the distribution, not the mean.
`FLOSS_TRACE(FLOSS_TR_USER + n, arg)` is the software equivalent when no pins
are available.
