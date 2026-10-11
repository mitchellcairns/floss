# FLOSS architecture

This document explains how the layers fit together, who may call whom, how
memory and threads are handled, and what happens to a packet on its way
through the stack. Headers are the source of truth; every name below exists
in `include/floss/*.h` or `src/floss_internal.h`.

## 1. Layers

```
 +-------------------------------------------------------------------+
 |  APP  (examples/gamepad, le_gamepad, serial_echo, your product)   |
 +-------------------------------------------------------------------+
     | floss_hid_*  floss_gap_*                      ^ callbacks
     v                                               |
 +--------------------------+   +------------------------------------+
 |  HID   (src/hid)         |   |  GAP  (src/gap)                    |
 |  HIDP, mailbox, pacing   |   |  visibility, pairing, bonds,       |
 +--------------------------+   |  link policy                       |
     | floss_l2cap_*            +------------------------------------+
     v   floss_sdp_*                 ^ floss_gap_on_hci_event  | floss_hci_cmd
 +--------------------------+        |                         v
 |  L2CAP (src/l2cap)       |  SDP (src/sdp)                       |
 |  signaling, channels     |  service records                     |
 +--------------------------+                                      |
     | floss_hci_acl_send     ^ floss_l2cap_on_pdu / _on_credits   |
     v                        |                                    v
 +-------------------------------------------------------------------+
 |  HCI  (src/hci)  commands, events, ACL credits, connection table  |
 +-------------------------------------------------------------------+
     | ops->send                ^ floss_transport_deliver (t->rx)
     v                          |
 +-------------------------------------------------------------------+
 |  TRANSPORT (transport/h4 | cyw43 | sim | yours)                   |
 +-------------------------------------------------------------------+
     |  wire: UART / USB / SPI shared bus / memory
 +-------------------------------------------------------------------+
 |  HAL (ports/<platform>): time, storage, log, entropy, critical    |
 +-------------------------------------------------------------------+

 CORE (src/core): buffer pool, timers, logs/trace/stats - used by all.
 LE   (src/le):   LE GAP, ATT/GATT, SMP, HID over GATT - beside L2CAP on LE links.
 RFCOMM (src/rfcomm): multiplexer + serial port, on top of L2CAP PSM 0x0003.
```

### Source map

| Directory | Files | Contents |
|---|---|---|
| `src/` | `floss.c`, `floss_internal.h` | `floss_init()` / `floss_start()` / `floss_poll()`; upward hooks |
| `src/core/` | `floss_buf.c`, `floss_sched.c`, `floss_debug.c`, `floss_util.c` | Buffer pool, timers, logs/snoop/stats/trace, helpers |
| `src/hci/` | `floss_hci.c`, `floss_hci_acl.c`, `floss_hci_bringup.c`, `floss_hci_status.c`, `floss_hci_priv.h` | Command queue, event dispatch, connection table; ACL credits, fragmentation, reassembly; controller bring-up; status/opcode names |
| `src/gap/` | `floss_gap.c`, `floss_gap_security.c`, `floss_gap_bonds.c`, `floss_gap_eir.c`, `floss_gap_internal.h` | Scan modes, link policy, connections; pairing (SSP, legacy PIN); bond store; EIR |
| `src/l2cap/` | `floss_l2cap.c`, `floss_l2cap_sig.c`, `floss_l2cap_priv.h` | Channels and data path; signaling |
| `src/sdp/` | `floss_sdp_server.c`, `floss_sdp_records.c`, `floss_sdp_de.c`, `floss_sdp_priv.h` | SDP server, HID/DID record builders, data elements |
| `src/hid/` | `floss_hid.c` | Classic HID device (HIDP, mailbox, pacing, reconnect) |
| `src/le/` | `floss_le.c`, `floss_att.c`, `floss_gatt.c`, `floss_gatt_services.c`, `floss_hogp.c`, `floss_smp.c`, `floss_smp_crypto.c/.h`, `floss_aes.c/.h` | See [LE.md](LE.md#1-map) |
| `src/rfcomm/` | `floss_rfcomm.c`, `floss_rfcomm_mux.c`, `floss_spp.c`, `floss_rfcomm_int.h` | See [RFCOMM.md](RFCOMM.md#1-model) |
| `transport/` | `h4/floss_h4.c`, `cyw43/floss_cyw43.c`, `sim/floss_sim.c` (+ headers) | HCI transports |

`cmake/floss.cmake` compiles every `src/**/*.c` into the `floss` library;
`FLOSS_ENABLE_LE` / `FLOSS_ENABLE_RFCOMM` compile the optional families out.

## 2. Who calls whom

There are exactly two kinds of cross-layer calls.

| Direction | Mechanism | Where declared | Example |
|---|---|---|---|
| **Downward** (request) | Public API of the lower layer | `include/floss/*.h` | HID calls `floss_l2cap_send_buf()`; L2CAP calls `floss_hci_acl_send()`; GAP calls `floss_hci_cmd()` |
| **Upward** (notification) | Direct call to a named internal hook, no registration | `src/floss_internal.h` | HCI calls `floss_l2cap_on_pdu()`, `floss_gap_on_hci_event()`, `floss_l2cap_on_credits()` |
| **To the app** | Callbacks registered at init | `floss_hid_callbacks_t`, `floss_hogp_callbacks_t`, `floss_rfcomm_ops_t`, `floss_l2cap_ops_t`, `floss_gap_add_listener()`, `floss_le_add_listener()`, `floss_smp_add_listener()` | `on_connected`, `FLOSS_GAP_EVT_READY` |

Rules:

1. A layer never calls *up* through a public API, and never calls *down* through
   a hook.
2. Upward hooks are plain functions, so `grep floss_l2cap_on_link_up` finds
   both ends. No function-pointer tables in the hot path.
3. Lateral calls are allowed only where `floss_internal.h` declares them
   (e.g. L2CAP -> GAP `floss_gap_require_security()`; SDP -> GAP via the public
   `floss_gap_eir_add_uuid16()`).
4. Only HCI touches the transport. Only the transport touches the wire.

The full hook list (`src/floss_internal.h`):

| From -> To | Hooks | Called from |
|---|---|---|
| HCI -> GAP | `floss_gap_on_hci_ready`, `floss_gap_on_hci_event` | `src/hci/floss_hci_bringup.c`, `src/hci/floss_hci.c` |
| GAP -> L2CAP | `floss_l2cap_on_link_up`, `floss_l2cap_on_link_down`, `floss_l2cap_on_link_security` | `src/gap/floss_gap.c`, `src/gap/floss_gap_security.c` |
| HCI -> L2CAP | `floss_l2cap_on_pdu`, `floss_l2cap_on_credits` | `src/hci/floss_hci_acl.c` |
| L2CAP -> GAP | `floss_gap_require_security` | `src/l2cap/floss_l2cap.c`, `floss_l2cap_sig.c` |
| HCI -> LE | `floss_le_on_hci_event`, `floss_le_on_encryption`, `floss_le_on_pdu`, `floss_le_on_link_down`, `floss_le_on_credits` | `src/hci/floss_hci.c`, `floss_hci_acl.c` |
| GAP -> LE | `floss_le_on_hci_ready` (declared under "HCI -> LE"; GAP calls it after emitting `FLOSS_GAP_EVT_READY`) | `src/gap/floss_gap.c` |
| init -> LE, SMP | `floss_le_init`, `floss_smp_init` | `src/floss.c` |
| SMP -> LE | `floss_le_send_fixed`, `floss_le_on_security_changed` | `src/le/floss_smp.c` |
| LE -> SMP | `floss_smp_on_link_up`, `floss_smp_on_link_down`, `floss_smp_on_pdu`, `floss_smp_on_le_event`, `floss_smp_on_encryption` | `src/le/floss_le.c` |
| init -> RFCOMM | `floss_rfcomm_init` (registers PSM 0x0003) | `src/floss.c` |
| core -> HID | `floss_hid_poll` (every `floss_poll()`) | `src/floss.c` |
| core | `floss_debug_init` | `src/floss.c` |

The LE and RFCOMM hooks exist only when `FLOSS_ENABLE_LE` /
`FLOSS_ENABLE_RFCOMM` is set. `floss_internal.h` also defines
`FLOSS_GAP_MAX_LISTENERS` (4).

Inside the LE family the wiring uses functions from the "Lifecycle" /
"Stack internal" sections of the public LE headers (not `floss_internal.h`):

| From -> To | Functions |
|---|---|
| core -> LE | `floss_le_poll()` (every `floss_poll()`) |
| LE GAP -> ATT/GATT | `floss_gatt_init()`, `floss_att_init()`, `floss_gatt_services_register_core()`, `floss_att_on_link_up()`, `floss_att_on_link_down()`, `floss_att_on_pdu()`, `floss_att_poll()`, `floss_gatt_on_link_down()`, `floss_gatt_cccd_sync()` |
| LE GAP -> HOGP | `floss_hogp_reset()`, `floss_hogp_on_security()`, `floss_hogp_on_credits()`, `floss_hogp_poll()`, `floss_hogp_on_link_down()` |
| ATT -> LE GAP | `floss_le_set_att_mtu()`, `floss_le_emit()` |

## 3. Threading model

The stack is **single-threaded and cooperative**. One execution context (the
"stack context") calls `floss_poll()`, and every `floss_*` API must be called
from that same context. Callbacks (HID, GAP listeners, L2CAP ops, timers) run
inside `floss_poll()` in that context, so they may call any API.

`floss_poll()` (`src/floss.c`) never blocks. It:

1. records `poll_gap` (time since the previous call),
2. `floss_hci_poll()`: pumps the transport (`ops->poll`, which delivers RX
   packets and dispatches events and ACL data upward), then retries TX the
   transport refused earlier,
3. `floss_hid_poll()`: sends Classic HID mailbox reports that are due,
4. `floss_le_poll()` (`FLOSS_ENABLE_LE`): retries deferred ATT responses and
   lets HOGP send on change,
5. `floss_sched_run()`: fires due timers (HCI runs first so timer callbacks
   such as the HID pacing tick see the freshest credits),
6. returns the microseconds until the next timer deadline.

Steps 2-4 run only after `floss_start()`.

### Calls that are safe from other contexts

| Function | Safe from | Notes |
|---|---|---|
| `floss_hid_set_input()` | ISR, other core, other thread | Copies into a mailbox slot under a seqlock; uses `floss_hal_critical_enter/exit` only for a few instructions. Never blocks, never allocates. |
| `floss_hogp_set_input()` | ISR, other core, other thread | Same seqlock mailbox for HID over GATT; one writer per report ID. |
| `floss_hal_time_us()` | anywhere | HAL requirement. |

Everything else, including `floss_hid_send_input_now()`, `floss_buf_alloc()` and
the timer API, is stack-context only. If another thread needs the stack to do
something, post it to your own queue and drain that queue next to
`floss_poll()`.

## 4. Memory model

No heap. Everything is static and sized by `floss_config.h`.

### Buffer pool

`FLOSS_BUF_COUNT` buffers of `floss_buf_t`, each with `FLOSS_BUF_SIZE` bytes of
payload capacity plus `FLOSS_BUF_HEADROOM` bytes in front.
`floss_buf_alloc()` / `floss_buf_free()` are O(1). Used for HCI commands,
ACL TX and ACL RX reassembly.

```
 mem: [ headroom (FLOSS_BUF_HEADROOM) ............ | payload ............ ]
                                                   ^ data after alloc, len = 0
 HID:                                  push 2 -> [HIDP][ID][payload]
 L2CAP:                         push 4 -> [L2CAP hdr][HIDP][ID][payload]
 HCI:                    push 4 -> [ACL hdr][L2CAP hdr]...
 transport writes into data[-headroom .. -1]   (e.g. H4 type, CYW43 4-byte header)
```

`FLOSS_BUF_HEADROOM` = transport headroom + ACL (4) + L2CAP (4) + HIDP+ID (2),
rounded up to 4 (16 bytes with the defaults). `floss_buf_push()` asserts if a
layer runs out of headroom; `floss_buf_tailroom()` reports what is left at the
end.

### Ownership rules

| Call | Ownership |
|---|---|
| `floss_hci_acl_send(buf)` | Takes ownership (queues or sends, then frees). |
| `floss_l2cap_send_buf(chan, buf, flags)` | Takes ownership **in all cases**, including errors. |
| `floss_l2cap_send(chan, data, len)` | Copies; caller keeps `data`. |
| `floss_hci_cmd(opcode, params, ...)` | Copies params. |
| `ops->send(t, type, pkt, len)` | Synchronous: the driver must be finished with `pkt` when it returns `FLOSS_OK`. `pkt[-headroom..-1]` is scratch for the driver. |
| `floss_transport_deliver(t, type, pkt, len)` | `pkt` is borrowed for the duration of the call; the stack copies what it keeps. |
| `on_data(chan, data, len)`, `on_output_report(...)` | `data` is valid only during the callback. |
| `floss_sdp_register(rec)`, `floss_hid_init(cfg, cb)` | Pointers are kept: storage must outlive registration. |
| `floss_timer_t` | Intrusive; caller owns the storage. |

## 5. Packet walk-through: an input report to the radio

```
 app (any context)        stack context (inside floss_poll)                    controller
 -----------------        ---------------------------------                    ----------
 floss_hid_set_input(1, d, 9)
   seqlock write slot[id 1]
   trace HID_SET_INPUT
                          (a) pacing timer fires (report_interval_us > 0)
                              on_report_tick()  <- last chance to update
                           or credit/poll path (report_interval_us == 0,
                              floss_hid_poll / floss_l2cap_on_credits)
                          HID: mailbox changed? (or send_unchanged)
                          HID: link ready? L2CAP can send and
                               conn->inflight < max_inflight x completion_batch
                          floss_buf_alloc(); copy 9 bytes at data
                          push 2: [0xA1 DATA|Input][0x01 ID]
                          floss_l2cap_send_buf(chan, buf, FLUSHABLE|PRIORITY)
                          L2CAP: push 4: [len][remote_cid]
                          floss_hci_acl_send(buf)
                          HCI: push 4: [handle|PB=10 (flushable start)][len]
                               credit free? no -> head of conn->txq (PRIORITY),
                                                  stat acl_credit_stalls
                               yes -> credits--, inflight_t[] = now
                          snoop hook, stats tx_acl / hid_report_age
                          DBG_PIN HID_TX, TRANS_TX
                          ops->send(t, ACL, pkt, 19)
                          transport writes framing into pkt[-headroom]
                                                               ----wire---->  air
                                                                              (ack or flush)
                          <---------------- Number Of Completed Packets ---  event
                          HCI: credits += n, pop inflight_t -> acl_ack_latency
                          floss_l2cap_on_credits() -> HID: newer report? send it
```

Details:

- **Mailbox.** One slot per report ID (`FLOSS_HID_MAX_INPUT_IDS`). A write that
  lands before the previous value was sent replaces it and increments
  `hid_reports_coalesced`.
- **When to send.** `report_interval_us == 0`: send as soon as a controller
  credit is free (the credit hook and `floss_hid_poll()` check). `> 0`: an absolute
  deadline timer (`floss_timer_start_at`, `deadline += interval`) fires,
  calls `on_report_tick()`, and sends if the data changed or `send_unchanged`.
  If the tick finds no credit, `hid_send_deferred` increments and the report
  goes out on the next credit.
- **PRIORITY** puts the buffer at the head of the link's TX queue so it never
  waits behind bulk traffic. **FLUSHABLE** selects the automatically-flushable
  packet boundary flag so the controller may discard it after the flush
  timeout (`floss_gap_set_flush_timeout`).
- **Fragmentation.** HCI fragments PDUs larger than the controller's `acl_mtu`.
  Gamepad reports never need it.
- **In-flight cap.** At most `max_inflight x floss_hci_completion_batch()`
  input reports wait in the controller (`floss_conn_t.inflight`); otherwise
  the report stays in the mailbox and newer state replaces it. See
  [HID_GAMEPAD.md](HID_GAMEPAD.md#in-flight-cap-max_inflight).
- **Ack latency.** `acl_ack_latency` measures send -> Number Of Completed
  Packets: air time, retransmissions and sniff waits all show up here.

## 6. Sequence: incoming connection, pairing and channel setup

A host pairs with a new gamepad (Secure Simple Pairing, NoInputNoOutput).
Events shown are HCI events unless marked.

```
 Host                 Controller              HCI            GAP                 L2CAP / HID / SDP
  | page ----------->  |
  |                    | Connection Request -> | ----------> | Accept Connection Request (role switch allowed)
  |                    | Connection Complete-> | conn table  |
  |                    |                       | ----------> | floss_l2cap_on_link_up(h); EVT_CONNECTED
  | L2CAP Conn Req PSM 0x0001 (SDP, no security) ---------------------------------> open, config
  | SDP ServiceSearchAttribute ------------------------------------------------------> answer from records
  | L2CAP Conn Req PSM 0x0011 (HID control) -----------------------------------------> needs ENCRYPTED:
  |                    |                       |             | <-- floss_gap_require_security(h)
  |                    |                       |             | Authentication Requested
  |                    | Link Key Request ---> | ----------> | no bond -> Link Key Request Negative Reply
  |                    | IO Capability Req --> | ----------> | IO Capability Reply (NoInputNoOutput, bonding)
  |                    | IO Capability Resp -> | ----------> | (peer caps)
  |                    | User Confirm Req ---> | ----------> | auto_accept_confirm -> User Confirm Reply
  |                    | Simple Pairing Compl->| ----------> | EVT_PAIRING_COMPLETE
  |                    | Link Key Notification>| ----------> | store bond (FLOSS_STORE_KEY_BOND_BASE + i)
  |                    | Authentication Compl->| ----------> | Set Connection Encryption
  |                    | Encryption Change --> | conn.encrypted = true
  |                    |                       | ----------> | floss_l2cap_on_link_security(h, 0, true)
  | <---------------------------------------------------------------------- Conn Rsp success, Config both ways
  | L2CAP Conn Req PSM 0x0013 (HID interrupt) -----------------------------------------> already encrypted: accept
  |                                                                         both channels OPEN -> on_connected()
  | input reports <---------------------------------------------------------------------- DATA on interrupt
```

A **bonded** reconnect skips IO capability / confirm: on Link Key Request GAP
answers with the stored key (Link Key Request Reply). If the host lost its key,
authentication fails with `PIN or Key Missing` (0x06); see the "pairing loops"
recipe in DEBUGGING.md.

When **the device reconnects** (`auto_reconnect`), roles invert: GAP pages the
last bond (`floss_gap_connect`), HID opens control (0x0011) first, then
interrupt (0x0013), with authentication + encryption before the first channel
opens.

## 7. Sequence: controller bring-up

`floss_start()` -> `floss_hci_start()`; everything is asynchronous (each
command is issued from the previous one's completion) and ends with
`FLOSS_GAP_EVT_READY`. Sequence from `src/hci/floss_hci_bringup.c` and
`src/gap/floss_gap.c`:

| Step | Who | Action |
|---|---|---|
| 1 | Transport | `ops->open()`: power, firmware download (CYW43), UART setup. May block. |
| 2 | HCI | `HCI_Reset` (retried) |
| 3 | HCI | Chipset `init_script` commands, if any (`floss_chipset_t`) |
| 4 | HCI | Read Local Version, Local Supported Commands, Local Supported Features, BD_ADDR |
| 5 | HCI | Vendor set-address (`set_addr_opcode`) + Read BD_ADDR again, if `floss_hci_set_addr_override()` was used |
| 6 | HCI | Read Buffer Size -> `floss_controller_info_t` (`acl_mtu`, `acl_total`) |
| 7 | HCI | If the controller does LE: Write LE Host Support, LE Read Buffer Size, LE Read Local Features, LE Set Event Mask |
| 8 | HCI | Set Event Mask -> `floss_gap_on_hci_ready()` |
| 9 | GAP | Simple Pairing Mode, Secure Connections Host Support, inquiry mode, page/inquiry scan type and activity, default link policy, Class of Device, local name, EIR, page timeout, connection accept timeout, scan enable |
| 10 | GAP | `FLOSS_GAP_EVT_READY` to listeners (HID starts reconnecting or becomes discoverable), then `floss_le_on_hci_ready()`: LE applies address, advertising parameters/data and enables advertising if started |

Non-essential failures are logged and skipped (a controller that rejects an
LE command still works for Classic).

Each command is subject to `FLOSS_HCI_CMD_TIMEOUT_US`; a timeout calls the
command's callback with status `0xFF` and increments `cmd_timeouts`.

## 8. Timers and scheduling

- Timers (`floss_timer_t`) are intrusive and kept sorted by deadline; firing
  looks only at the head (O(1)), arming is O(active timers).
- A timer fires on the first `floss_poll()` at or after its deadline. Lateness
  is recorded in the `timer_lateness` histogram.
- Periodic work uses `floss_timer_start_at(t, deadline += period)`, so jitter
  in one period never accumulates into drift.
- Time is `uint32_t` microseconds that wraps every ~71.6 min; always compare
  with `floss_time_diff(a, b)`.

### How the return value gives sub-ms precision

`floss_poll()` returns the microseconds until the next timer deadline (or a
very large value if none). The run loop decides how to wait:

```c
for (;;) {
    uint32_t idle_us = floss_poll();
    app_work();
    if (idle_us > 200) {
        port_sleep_until_irq_or(idle_us - 50);   /* wake a bit early */
    }                                            /* else: just loop (spin) */
}
```

The stack must also run when the transport has data (UART RX, host-wake IRQ).
Sleeping only up to `idle_us`, and waking on transport interrupts, gives
timer precision bounded by your wake-up latency rather than by a tick rate.
Calling `floss_poll()` more often is always safe. The `poll_gap` histogram
shows whether the app is starving the stack.

## 9. LE architecture

LE runs beside the Classic L2CAP layer, sharing HCI and the connection table:

```
            HCI (connection table: floss_conn_t.le = true)
             |  LE Meta events            | ACL PDUs on LE links
             v                            v
    floss_le_on_hci_event()       floss_le_on_pdu()
             |                            |
             |              +-------------+-------------+
             |              | CID 0x0004  | 0x0005      | 0x0006
             |              v             v             v
             |            ATT/GATT    LE signaling     SMP  (floss_smp_on_pdu)
             +-- subevents 0x05/0x08/0x09 --------------> floss_smp_on_le_event
```

Contract (from `floss_internal.h`):

- On LE (Enhanced) Connection Complete, HCI fills a `floss_conn_t` with
  `le = true` and `addr_type`, then forwards the subevent to
  `floss_le_on_hci_event()`. LE fills its `floss_le_link_t`, then calls
  `floss_att_on_link_up()` and `floss_smp_on_link_up()`.
- ACL PDUs on LE links go to `floss_le_on_pdu()` (basic header included), never
  to Classic L2CAP. LE uses fixed channels only: 0x0004 ATT, 0x0005 LE
  signaling, 0x0006 SMP.
- SMP sends through `floss_le_send_fixed()` and reports the resulting level via
  `floss_le_on_security_changed()` (1 none, 2 encrypted unauthenticated,
  3 encrypted authenticated, 4 Secure Connections authenticated). ATT checks
  attribute permissions against that level; LE then runs
  `floss_gatt_cccd_sync()` (save or restore the bonded peer's CCCDs) and
  `floss_hogp_on_security()`.
- SMP receives the LE meta subevents it needs: 0x05 LTK Request, 0x08 Read
  Local P-256 Public Key Complete, 0x09 Generate DHKey Complete; and encryption
  changes via `floss_smp_on_encryption()`.
- Disconnection calls `floss_le_on_link_down()`, which fans out to SMP,
  ATT, GATT (CCCD slot) and HOGP. Credits call `floss_le_on_credits()` (LE
  buffers, or shared when `le_acl_total == 0`), which retries ATT and lets
  HOGP send.
- Every `floss_poll()` calls `floss_le_poll()` (`floss_att_poll()` +
  `floss_hogp_poll()`).

See [LE.md](LE.md) for the APIs.
