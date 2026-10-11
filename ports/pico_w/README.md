# FLOSS on the Raspberry Pi Pico W

Two gamepad firmwares for the Pico W / Pico 2 W (CYW43439 radio), built from
one board `main.c` and a portable app:

| Firmware | App | Host sees | Link |
|---|---|---|---|
| `floss_gamepad` | `examples/gamepad` | **"FLOSS Gamepad"**: 16 buttons, hat, two sticks, two analog triggers, rumble/LED output report | Classic HID, up to ~720 reports/s |
| `floss_le_gamepad` | `examples/le_gamepad` | **"FLOSS LE Gamepad"**: same controls over HID over GATT | LE, one report per connection event |

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

Outputs: `build-pico/floss_gamepad.uf2` and `build-pico/floss_le_gamepad.uf2`
(plus `.elf` files for debugging).

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

The script looks for the drive label `RPI-RP2` only; on a Pico 2 W copy the
file by hand.

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

- The CYW43 driver is used directly (`pico_cyw43_driver` +
  `pico_async_context_poll`), not through `pico_cyw43_arch`, which would
  pull in another host stack. The firmware targets define the cyw43
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
  (Classic 0x0F10, LE 0x0001); register your own before shipping a device.
