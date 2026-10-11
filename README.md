# FLOSS

**F**ree, **L**ightweight, **O**pen **S**hort-range **S**tack.

The name is a nod to free/libre open-source software (FLOSS) and to dental
floss: thin, strong, and it gets into tight places.

FLOSS is a public-domain (Unlicense) host stack for the 2.4 GHz short-range
radio standard, covering both Classic (BR/EDR) and Low Energy (LE). It is
written in portable C11, uses no heap, and is built embedded-first with
desktop as a first-class port. The first milestone is a Classic HID gamepad
on a Raspberry Pi Pico W (CYW43439) with sub-millisecond report pacing. LE
(GATT + HID-over-GATT) and RFCOMM are being developed in parallel. FLOSS
exists because people are tired of proprietary host stacks with awkward
licensing; it is written by people who are not members of the standard's
industry consortium, from public specifications, and it interoperates with
controllers and hosts implementing the Core Specification HCI.

## Feature matrix

| Area | Feature | Status |
|---|---|---|
| **Classic** | HCI: command queue + flow control, ACL credits, fragmentation/reassembly, connection table | Implementing (milestone 1) |
| | GAP: discoverable/connectable, Class of Device, EIR, Secure Simple Pairing, bonding, legacy PIN fallback, sniff/role/supervision/flush timeout | Implementing (milestone 1) |
| | L2CAP basic mode: signaling, configuration, MTU, flush timeout, echo, info | Implementing (milestone 1) |
| | SDP server: pre-serialised records, continuation, HID + Device ID builders | Implementing (milestone 1) |
| | HID device: all HIDP transactions, latest-wins input mailbox, paced or on-change reports, reconnect, virtual cable | Implementing (milestone 1) |
| **LE** | LE GAP: advertising, connections, connection parameter updates (7.5 ms) | In progress |
| | ATT/GATT server: static attribute table, GAP/GATT/DIS/Battery services | In progress |
| | SMP: legacy + Secure Connections (P-256 ECDH via controller, software AES-CMAC) | In progress |
| | HID-over-GATT | In progress |
| **RFCOMM** | Multiplexer with credit-based flow control + serial port service | In progress |
| **Transports** | H4 over any non-blocking byte stream (UART, USB-CDC, TCP, pipe) | Implementing |
| | CYW43 shared bus (Pico W / Pico 2 W) | Implementing |
| | Simulated controller (tests, desktop demos) | Implementing |
| **Ports** | Raspberry Pi Pico W / Pico 2 W | Implementing |
| | POSIX / Windows desktop | Implementing |

See [docs/LE.md](docs/LE.md) and [docs/RFCOMM.md](docs/RFCOMM.md) for the
designs of the in-progress parts.

## Quick start

### Pico W (gamepad example)

Requirements: Pico SDK 2.x (it ships the cyw43 driver and the radio
firmware), `arm-none-eabi-gcc`, CMake >= 3.13, Ninja.

```sh
export PICO_SDK_PATH=/path/to/pico-sdk
cmake -S ports/pico_w -B build-pico -G Ninja -DPICO_BOARD=pico_w   # pico2_w for Pico 2 W
cmake --build build-pico
```

Flash: hold BOOTSEL, plug in the board, copy the generated `.uf2` from
`build-pico/` to the `RPI-RP2` drive (or `picotool load -x <file>.uf2`).
The gamepad becomes discoverable; pair it from the host's "add device"
dialog. Logs appear on the USB serial port.

### Desktop (tests)

```sh
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

Options: `-DFLOSS_BUILD_TESTS=OFF`, `-DFLOSS_BUILD_POSIX=OFF`,
`-DFLOSS_WERROR=OFF`. The tests run the whole stack against the simulated
controller (`transport/sim`), so no radio is needed.

### Minimal program

```c
#include "floss/floss.h"

int main(void) {
    floss_transport_t *t = my_port_transport();       /* H4, CYW43, sim, ... */
    floss_init(&(floss_init_t){ .transport = t });

    floss_hid_config_t cfg;
    floss_hid_config_defaults(&cfg);
    cfg.name = "FLOSS Gamepad";
    cfg.report_descriptor = my_descriptor;
    cfg.report_descriptor_len = sizeof my_descriptor;
    floss_hid_init(&cfg, &my_callbacks);

    floss_start();
    for (;;) {
        uint32_t idle_us = floss_poll();   /* never blocks */
        /* sample inputs, floss_hid_set_input(...), or sleep up to idle_us */
    }
}
```

## Repository map

```
include/floss/        Public API, one header per layer (floss.h is the umbrella)
  floss_config.h        every compile-time knob and its default
  floss_hal.h           the entire platform contract (time, storage, log, entropy, critical)
  floss_transport.h     HCI transport driver interface, byte streams, chipset quirks
  floss_buf.h           buffer pool with headroom, FIFO queues
  floss_sched.h         microsecond timers
  floss_debug.h         logs, snoop, stats, trace, debug pins
  floss_hci.h  floss_gap.h  floss_l2cap.h  floss_sdp.h  floss_hid.h
src/                  Stack implementation
  floss_internal.h      cross-layer upward hooks (not public API)
  core/                 buffer pool, timers, logs/trace/stats
  hci/ gap/ l2cap/ sdp/ hid/   one directory per Classic layer
  le/                   LE GAP, ATT/GATT, SMP, HID-over-GATT (in progress)
  rfcomm/               RFCOMM multiplexer + serial port (in progress)
transport/
  h4/                   H4 framing over any floss_stream_t
  cyw43/                CYW43439 shared-bus transport (Pico W)
  sim/                  in-memory virtual controller for tests
ports/
  posix/                desktop HAL (POSIX and Windows) + demo
  pico_w/               Pico W / Pico 2 W HAL, build and gamepad firmware
examples/gamepad/     the reference HID gamepad application
tests/                unit + integration tests (ctest)
tools/
  floss_snoop.py        log -> .btsnoop capture for Wireshark
  floss_monitor.py      live serial monitor / capture
cmake/floss.cmake     drop-in CMake targets: floss, floss_h4, floss_sim
docs/                 architecture, porting, debugging, HID, LE, RFCOMM, legal
```

## Performance philosophy

| Principle | What it means in practice |
|---|---|
| **No heap** | All memory is sized at compile time in `floss_config.h`. Allocation is O(1) from a fixed pool; exhaustion is a counted, recoverable error (`buf_alloc_failures`). |
| **Zero-copy headroom buffers** | Each buffer reserves `FLOSS_BUF_HEADROOM` bytes in front of the payload. HID, L2CAP and HCI `floss_buf_push()` their headers; the transport writes its framing into the bytes before the packet. The payload is written once. |
| **Latest-wins HID mailbox** | `floss_hid_set_input()` overwrites a per-report-ID slot (ISR/other-core safe). Stale state is never queued; superseded reports are counted as coalesced. |
| **Microsecond timers** | `floss_poll()` returns the microseconds until the next deadline; periodic work uses absolute, drift-free deadlines. |
| **Credits-aware sending** | The stack knows exactly how many controller ACL buffers are free and sends the newest report the moment one frees up, instead of queuing behind old ones. |
| **Flushable reports** | Input reports are marked FLUSHABLE + PRIORITY; with a flush timeout the controller drops a report it could not deliver in time rather than delaying the next one. |

## Debugging

Five tools, cheapest first: per-module **logs**, HCI **snoop** to Wireshark,
always-on **stats** with latency histograms (`floss_stats_dump()` prints
tuning hints), a binary **trace** ring cheap enough for hot paths, and
**debug pins** for a logic analyser. "Reports arrive in bursts?" Check the
`acl_ack_latency` histogram and the link mode. See
[docs/DEBUGGING.md](docs/DEBUGGING.md).

## Documentation

| Doc | Contents |
|---|---|
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | layers, call rules, threading, memory, packet walk-throughs |
| [PORTING.md](docs/PORTING.md) | HAL contract, writing a transport, configuration table |
| [DEBUGGING.md](docs/DEBUGGING.md) | tools, stats reference, troubleshooting recipes |
| [HID_GAMEPAD.md](docs/HID_GAMEPAD.md) | building a gamepad: descriptors, pacing, latency trade-offs |
| [LE.md](docs/LE.md), [RFCOMM.md](docs/RFCOMM.md) | in-progress designs |
| [LEGAL.md](docs/LEGAL.md) | licensing, naming, clean-room policy |
| [CONTRIBUTING.md](CONTRIBUTING.md) | style, tests, commits |

## License

FLOSS is released into the public domain under the
[Unlicense](LICENSE). Platform SDKs and radio firmware used by some ports
have their own licenses; see [docs/LEGAL.md](docs/LEGAL.md).
