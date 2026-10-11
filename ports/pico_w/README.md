# FLOSS on the Raspberry Pi Pico W

Two gamepad firmwares for the Pico W / Pico 2 W (CYW43439 radio), built from
one board `main.c` and a portable app, plus a pad + USB dongle pair that talk
to each other over FLOSS's own point-to-point link profile
([Pad + dongle](#pad--dongle)):

| Firmware | App | Host sees | Link |
|---|---|---|---|
| `floss_gamepad` | `examples/gamepad` | **"FLOSS Gamepad"**: 16 buttons, hat, two sticks, two analog triggers, rumble/LED output report | Classic HID, up to ~720 reports/s |
| `floss_le_gamepad` | `examples/le_gamepad` | **"FLOSS LE Gamepad"**: same controls over HID over GATT | LE, one report per connection event |
| `floss_link_pad` | `link/main_pad.c` | (only its serial console) | link profile to the dongle, Classic or LE |
| `floss_link_dongle` | `link/main_dongle.c` | **"FLOSS Link Dongle"**: USB HID gamepad (same report descriptor as "FLOSS Gamepad", 1 ms polling) + serial console | USB to the PC, link profile to the pad |

Both run a demo pattern out of the box, so you can watch them in any gamepad
tester without wiring anything. In the Classic firmware the **BOOTSEL**
button is button 1. The demo puts a millisecond counter on the right stick X
axis so `tools/hostprobe` can measure delivery (see
[docs/PERFORMANCE.md](../../docs/PERFORMANCE.md)).

## Files

| File | Purpose |
|---|---|
| `CMakeLists.txt` | Standalone Pico SDK project; `floss_pico_firmware()` builds both firmwares (`FLOSS_PICO_APP_LE` selects the app) |
| `pico_sdk_import.cmake` | Finds the SDK, ARM toolchain, pioasm and picotool |
| `floss_pico_config.h` | FLOSS compile-time configuration for this board (`FLOSS_USER_CONFIG_FILE`) |
| `floss_hal_pico.c/.h` | HAL: time, spin-lock critical sections, flash key/value store, log |
| `main.c` | Bring-up, main loop with profiler, BOOTSEL, LED, serial console, live tuning |
| `link/main_pad.c` | Link pad: gamepad state -> `floss_link_set_input()`, console, status |
| `link/main_dongle.c` | Link dongle: `floss_link` (dongle role) -> USB, console, status |
| `link/link_board.c/.h` | Shared by pad and dongle: radio bring-up, BOOTSEL, LED, transport in flash, debug keys |
| `link/usb_descriptors.c` | Dongle USB identity: CDC + HID composite, VID/PID, serial = flash unique ID |
| `link/usb_stdio.c/.h` | Dongle: TinyUSB task + stdio driver over its CDC, 1200-baud reboot to BOOTSEL |
| `link/usb_gamepad.c/.h` | Dongle: radio report -> USB HID IN (latest wins), rumble -> pad |
| `link/link_meter.c/.h` | Dongle: 25 us-resolution interval / jitter windows for the status line |
| `link/tusb_config.h` | Dongle: TinyUSB configuration (CDC + HID device) |

The gamepad report descriptor, report layout and demo animation are shared
by `floss_gamepad`, the link pad and the link dongle
(`examples/gamepad/gamepad_state.c/.h`).

The radio transport is `transport/cyw43/` (with the fast bus path,
`FLOSS_CYW43_FAST_BUS`, see [docs/PORTING.md](../../docs/PORTING.md#6-how-the-pico-w-port-works)).

## Prerequisites

The easiest route is the Raspberry Pi Pico VS Code extension, which installs
everything under `~/.pico-sdk/` (`pico_sdk_import.cmake` looks there):

- Pico SDK 2.2.0 (`~/.pico-sdk/sdk/2.2.0`)
- Arm GNU toolchain 14.2 (`~/.pico-sdk/toolchain/14_2_Rel1`)
- Ninja, CMake >= 3.13
- prebuilt `pioasm` and `picotool` (`~/.pico-sdk/tools/2.2.0`, `~/.pico-sdk/picotool/2.2.0`)

Anything else: pass `-DPICO_SDK_PATH=...` / `-DPICO_TOOLCHAIN_PATH=...` or set
the environment variables of the same names.

## Build

From the repository root:

```sh
cmake -G Ninja -S ports/pico_w -B build-pico
cmake --build build-pico
```

Windows (Git Bash) with the VS Code extension's SDK, toolchain and tools. The
prebuilt `pioasm`/`picotool` matter: without them CMake tries to compile them
with your host compiler, which fails with some clang/MSVC combinations.

```sh
export PICO_SDK_PATH="$HOME/.pico-sdk/sdk/2.2.0"
export PATH="$HOME/.pico-sdk/toolchain/14_2_Rel1/bin:$HOME/.pico-sdk/ninja/v1.12.1:$PATH"
cmake -G Ninja -S ports/pico_w -B build-pico \
    -Dpioasm_DIR="$HOME/.pico-sdk/tools/2.2.0/pioasm" \
    -Dpicotool_DIR="$HOME/.pico-sdk/picotool/2.2.0/picotool"
cmake --build build-pico
```

| CMake option | Effect |
|---|---|
| `-DPICO_BOARD=pico2_w` | Build for the Pico 2 W (RP2350) |
| `-DFLOSS_PICO_SNOOP=ON` | Log every HCI packet from boot (also toggled with `p`) |

Outputs: `build-pico/floss_gamepad.uf2`, `floss_le_gamepad.uf2`,
`floss_link_pad.uf2` and `floss_link_dongle.uf2` (plus `.elf` files for
debugging). The link firmwares need the link profile (`src/link/`) in the
stack; `cmake --build build-pico --target floss_gamepad` builds one firmware.

## Flash

Hold **BOOTSEL** while plugging the board in and copy the `.uf2` onto the
`RPI-RP2` (Pico 2 W: `RP2350`) drive, or with picotool:

```sh
picotool load -x build-pico/floss_gamepad.uf2 -f
```

(`-f` reboots a running board into BOOTSEL first, `-x` starts the firmware.)

On Windows picotool usually cannot reboot a running board (its reset
interface needs a WinUSB driver). Use the 1200-baud helper instead: opening
the board's USB serial port at 1200 baud makes the SDK's USB stdio reboot
into the ROM bootloader; the script waits for the `RPI-RP2` drive, copies
the `.uf2`, then shows the console for `-ReadMs` milliseconds (default 8000,
0 = don't):

```powershell
powershell -File tools/pico_flash.ps1 -Uf2 build-pico/floss_gamepad.uf2
```

The script also recognises the Pico 2 W's `RP2350` drive. With several
boards connected, pick one with `-Serial` or `-Port` (see
[Two boards on one PC](#two-boards-on-one-pc)).

## Use

1. Open the USB serial port (115200, any terminal, or
   `tools/floss_monitor.py`). The boot banner prints the firmware, device
   address, bond count and the command list; press `h` to see it again.
2. On the host, open the add-device list and pick **FLOSS Gamepad** or
   **FLOSS LE Gamepad**. Pairing is "just works" (Classic: Secure Simple
   Pairing; LE: LE Secure Connections).
3. Test it: Windows `joy.cpl`, Linux `evtest` / `jstest-gtk`, Android or
   macOS with any gamepad tester app, or a browser gamepad tester page.

Bonds live in the last 4 KB flash sector. After a reset the Classic firmware
pages the bonded host to reconnect by itself (with no bond it becomes
discoverable again); the LE firmware advertises and the bonded host
reconnects, with its notification subscriptions restored from flash.

LED (driven from Classic HID state): fast blink = no Classic bond, waiting
to pair; slow blink = bonded, waiting / reconnecting; solid = connected.
The LE firmware keeps fast-blinking; its LED does not track LE links.

### Serial console

| Key | Action |
|-----|--------|
| `s` | dump statistics (with tuning hints) |
| `t` | dump the trace ring |
| `r` | reset statistics |
| `d` | become discoverable (Classic) |
| `x` | delete all Classic bonds (`floss_gap_bond_delete_all()`), disconnect HID |
| `u` | HID virtual-cable unplug (host forgets us; Classic) |
| `v` | log level DEBUG |
| `q` | log level INFO |
| `p` | toggle HCI snoop logging |
| `h`, `?` | banner + help |

**Latency tuning keys** (Classic firmware only, not listed by `h`). Each
applies immediately and prints `tune: inflight .. qos .. us flush .. ms interval .. us`:

| Key | Effect |
|-----|--------|
| `0`-`8` | In-flight cap: `floss_hid_set_max_inflight(n)` (0 = no cap; effective cap x2 on the CYW43439) |
| `Q` | QoS poll latency, cycles 625 / 1250 / 2500 / 5000 us (`floss_gap_qos_setup()`, needs a connected host) |
| `F` | Flush timeout, cycles 0 / 2 / 3 / 5 / 10 / 20 ms (`floss_gap_set_flush_timeout()`, needs a connected host) |
| `I` | Report interval, cycles 1000 / 1250 / 2000 / 4000 / 8000 / 0 us (`floss_hid_set_report_interval()`) |
| `P` | Toggle page scan while connected (`floss_gap_set_scan(false, on)`; starts off) |
| `m` | Show the current settings |

`tools/hostprobe/experiment.ps1` drives these keys and measures the effect
from the host.

### Status lines

While a Classic host is connected, every 5 s:

```
status: connected, <n> reports/s, ack p50 <us> us p99 <us> us, deferred <n>, coalesced <n>
loop max us: cyw43 <us> floss <us> bootsel <us> app <us> led <us> console <us>
```

The second line is the main-loop profiler: worst-case time per loop section
since the previous status line. See
[docs/DEBUGGING.md](../../docs/DEBUGGING.md#6-pico-w-console-and-host-side-measurement).

## Pad + dongle

`floss_link_pad` and `floss_link_dongle` are a controller and its USB
receiver, both running FLOSS. Between them there is no HID profile on the
air but the point-to-point link profile
([include/floss/floss_link.h](../../include/floss/floss_link.h)): raw input
reports with a sequence number and the pad's timestamp, so the dongle counts
loss and measures timing exactly. The PC only sees the dongle: a USB HID
gamepad that needs no driver and no pairing on the PC side.

```
pad                                  dongle                              PC
demo + BOOTSEL -> 13-byte report ID 1
  floss_link_set_input() --radio-->  on_input() -> USB HID IN endpoint --> game input API
                                     (latest wins, 1 ms polling)
log "output: rumble ..." <--radio--  floss_link_send_output() <-- output report ID 2
```

- **Same report everywhere.** The pad publishes exactly the Classic HID
  gamepad's input report (ID 1, 13 bytes, `examples/gamepad/gamepad_state.h`)
  and the dongle's USB interface declares the same report descriptor, so
  reports are forwarded byte for byte. Output report 2 (rumble strong, rumble
  weak, LED) from the PC - SET_REPORT or interrupt OUT - goes to the pad,
  which logs it.
- **Latest wins.** The dongle hands a report to USB as soon as the HID IN
  endpoint is free. If a newer radio report arrives while one is still
  waiting for the host's poll, the newer one replaces it ("replaced" in the
  stats): the PC always reads the freshest state, never a backlog.
- **Pad inputs.** The demo pattern of the Classic gamepad (left stick circle,
  triggers, buttons cycling, millisecond counter on the right stick X) plus
  **BOOTSEL as button 1**; `d` toggles the demo.
- **Dongle USB identity.** VID `0x1209` (pid.codes) / PID `0x0F11` - a
  PLACEHOLDER, register your own before shipping - product "FLOSS Link
  Dongle", serial number = the flash unique ID. Its serial console is its own
  TinyUSB CDC interface (the SDK's USB stdio is not used); output is dropped
  rather than waited for when no terminal reads it, and the 1200-baud reboot
  to BOOTSEL works as on the other firmwares.

### Transport

Both boards must use the same transport, chosen at runtime and kept in flash
(key `0x8200`, default Classic):

| | Classic (`c`) | LE (`e`) |
|---|---|---|
| Who connects | dongle pages the pad | dongle scans, pad advertises |
| Reports | one per 1.25 ms poll, up to 800/s | one per 7.5 ms connection event, 133/s |
| Data | custom L2CAP channel | notifications of the link service |

Press `c` or `e` on **each** board: the choice is saved and the board
reboots into it. Pair again on the new transport (below) if the boards were
only bonded on the other one.

### Pairing

1. Flash `floss_link_pad.uf2` to one board and `floss_link_dongle.uf2` to the
   other ([Two boards on one PC](#two-boards-on-one-pc)).
2. Put the pad in pairing mode: an unbonded pad does this by itself about
   1.5 s after boot; otherwise press `P` on the pad. Its LED blinks fast.
3. Press `P` on the dongle: it searches for a FLOSS link pad (Classic
   inquiry / LE scan), connects and bonds with the first one it finds.

In short: press `P` on both, or `P` on the dongle while the pad is in
pairing mode. When the link is up both LEDs are solid and both consoles log
`link up`. From then on the dongle reconnects to its pad by itself after a
reset or a dropped link (slow blink while reconnecting). `x` forgets the
peer on that board.

LED (both boards): fast blink = pairing, slow blink = reconnecting to the
bonded peer, solid = connected, short blip every 2 s = idle (no bond, not
pairing).

### Console keys

| Key | Pad | Dongle |
|-----|-----|--------|
| `P` | pair: become visible to a dongle (`floss_link_pair()`) | search for a pad and bond with the first one found |
| `x` | forget the dongle (`floss_link_forget()`) | forget the pad |
| `c` / `e` | transport Classic / LE: save to flash, reboot | same |
| `s` | `floss_stats_dump()` + `floss_link_stats_dump()` | same + USB counters (forwarded, read by host, replaced, outputs, dropped console bytes) |
| `r` | reset statistics | same, incl. USB counters |
| `d` | demo pattern on/off | - |
| `t` | dump the trace ring | same |
| `v` / `q` | log level DEBUG / INFO | same |
| `p` | toggle HCI snoop logging | same |
| `h`, `?` | banner (address, transport, link state; dongle: USB serial) + help | same |

### Status lines

Every 5 s while the link is up. Pad:

```
status: connected (Classic), <n> reports/s sent, <n> outputs received
loop max us: cyw43 <us> floss <us> bootsel <us> pad <us> led <us> console <us>
```

Dongle (`ping_period_ms = 100`, so the round trip is measured all the time):

```
status: connected (Classic), <n> reports/s, lost <n>, rtt p50 <us> avg <us> us
rx interval p50 <us> p99 <us> max <us> us, jitter p50 <us> p99 <us> max <us> us
usb: <n> reports/s to host, replaced <n>, host wait p50 <us> p99 <us> max <us> us (<samples>)
loop max us: cyw43 <us> floss <us> usb <us> app <us> led <us> console <us>
```

- **reports/s, lost**: INPUTs received in the last 5 s and sequence numbers
  missing between them.
- **rtt**: PING -> PONG round trip from `floss_link_stats()` (log2 buckets:
  p50 is a bucket edge, avg is exact).
- **rx interval**: time between consecutive arrivals at the dongle.
- **jitter**: end-to-end latency variation, arrival time minus the pad's
  send timestamp with the crystal drift between the boards removed, 0 = the
  fastest delivery of the window. Interval and jitter are measured by the
  dongle itself per 5 s window at 25 us resolution (`link/link_meter.c`);
  `s` shows the link profile's cumulative histograms.
- **usb**: reports handed to the HID endpoint per second, radio reports
  replaced before USB took them, and the **host wait** from radio arrival to
  the host reading that report (the dongle + USB polling share of the
  latency; at most about one 1 ms poll interval when the host keeps up).
- **loop max us**: the main-loop profiler as in `main.c` (worst case per
  section since the previous line).

### Measuring end to end

The dongle's USB gamepad carries the pad's report unchanged, including the
demo's millisecond counter, so `tools/hostprobe` measures the whole path -
pad -> radio -> dongle -> USB -> Windows - exactly as it measures the
Classic gamepad:

```powershell
cd tools/hostprobe; dotnet build -c Release -o out; cd ../..
tools/hostprobe/out/floss_hostprobe.exe pads 10
```

`pads` reads the **first** game controller the OS lists: it should print
`controller: 'FLOSS Link Dongle' vid=0x1209 pid=0x0F11`. Disconnect other
controllers (for example a paired "FLOSS Gamepad") first. "device ms" are
the pad's milliseconds; delivered/s, coalesced %, host inter-arrival and
latency jitter mean the same as in [docs/PERFORMANCE.md](../../docs/PERFORMANCE.md).
Read them next to the dongle's status lines to split the latency between
the radio (`rx interval`, `jitter`, `rtt`) and USB (`host wait`, `replaced`).

### Two boards on one PC

Each board's USB serial number is its flash unique ID, the same under every
FLOSS firmware (the bootloader's BOOTSEL drive uses a different one).
`tools/pico_flash.ps1` uses it to tell the boards apart:

```powershell
powershell -File tools/pico_flash.ps1 -List
#   COM3   serial E661640843406A29   Pico SDK USB stdio
#   COM5   serial E66368254F427A39   FLOSS Link Dongle
powershell -File tools/pico_flash.ps1 -Uf2 build-pico/floss_link_dongle.uf2 -Serial E661640843406A29
powershell -File tools/pico_flash.ps1 -Uf2 build-pico/floss_link_pad.uf2 -Serial E66368254F427A39
```

- `-Serial <substring>` picks the running board whose serial contains it;
  `-Port COMx` picks by port. With more than one board connected, one of the
  two is required.
- The console afterwards is read from the same board, found again by its
  serial: the COM port number usually changes when a board switches between
  the pad (Raspberry Pi VID) and the dongle (VID 0x1209) firmware.
- The script refuses rather than guesses when a board is already in BOOTSEL
  mode or two `RPI-RP2` / `RP2350` drives appear; flash or unplug the other
  board first. A board that is in BOOTSEL already (no serial port) is flashed
  only when it is the one board connected.
- `tools/hostprobe/experiment.ps1` still assumes a single board.

## Tuning

- Classic defaults come from [docs/PERFORMANCE.md](../../docs/PERFORMANCE.md):
  `GAMEPAD_REPORT_INTERVAL_US` (1000), `GAMEPAD_MAX_INFLIGHT` (2),
  `GAMEPAD_QOS_LATENCY_US` (1250), `GAMEPAD_FLUSH_TIMEOUT_MS` (0) in
  `examples/gamepad/gamepad_app.h`.
- `FLOSS_HID_CONNECTABLE_WHILE_CONNECTED` defaults to 0: no page scan while
  a host is connected (most consistent report timing).
- LE: `examples/le_gamepad` paces with `FLOSS_HOGP_INTERVAL_CONN` and the
  default `max_inflight` (2).
- Feeding real inputs (Classic): call `gamepad_app_set_demo(false)` and the
  `gamepad_app_set_*()` setters from the main loop; or call
  `floss_hid_set_input()` / `floss_hogp_set_input()` directly from an ISR or
  core 1 (lock-free mailbox).

## Notes

- The CYW43 driver is used directly, not through `pico_cyw43_arch` (which
  would pull in another host stack). Default `FLOSS_PICO_BARE=ON`: no
  async_context, no RTOS, no interrupts - `transport/cyw43/floss_cyw43_bare.c`
  services the driver inside `floss_poll()`, so the main loop is just
  `floss_poll()` plus your code. `-DFLOSS_PICO_BARE=OFF` uses the SDK's
  `pico_cyw43_driver` + `async_context_poll` instead. The firmware targets define the cyw43
  driver's radio-enable option so the combined WLAN + radio firmware is
  loaded (see `CMakeLists.txt`).
- The flash store holds 16 entries of up to 64 bytes: enough for every
  Classic and LE key with `FLOSS_MAX_BONDS = 4` (key map in
  [docs/PORTING.md](../../docs/PORTING.md#persistent-storage-required-may-be-ram-only)).
  Writing erases the sector (~50 ms, interrupts off). That happens while
  pairing, and on an LE bonded reconnection whose subscriptions changed; it
  shows up in `transport_poll_time`.
- `floss_pico_config.h` builds with `FLOSS_LOG_LEVEL = FLOSS_LOG_DEBUG`; the
  firmware runs at INFO (`v` / `q` switch).
- Device IDs use vendor 0x1209 (pid.codes) with placeholder product IDs
  (Classic 0x0F10, LE 0x0001, link dongle USB 0x0F11); register your own
  before shipping a device.
