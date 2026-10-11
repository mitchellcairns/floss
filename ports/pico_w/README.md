# FLOSS on the Raspberry Pi Pico W

A Classic (BR/EDR) HID gamepad running on the Pico W's CYW43439 radio.
It shows up on the host as **"FLOSS Gamepad"**: 16 buttons, a hat switch,
two sticks and two analog triggers, plus a rumble/LED output report.

Out of the box the left stick turns a slow circle and the buttons cycle (demo
mode), so you can watch it in any gamepad tester without wiring anything.
The **BOOTSEL** button is button 1.

## Files

| File                    | Purpose                                                       |
|-------------------------|---------------------------------------------------------------|
| `CMakeLists.txt`        | Standalone Pico SDK project (`floss_gamepad` firmware)         |
| `pico_sdk_import.cmake` | Finds the SDK, ARM toolchain, pioasm and picotool             |
| `floss_pico_config.h`   | FLOSS compile-time configuration for this board               |
| `floss_hal_pico.c/.h`   | HAL: time, spin-lock critical sections, flash bond store, log |
| `main.c`                | Bring-up, main loop, BOOTSEL, LED, serial console             |

The portable gamepad logic lives in `examples/gamepad/`, the radio transport
in `transport/cyw43/`.

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

Windows (Git Bash or PowerShell) with the VS Code extension's Ninja:

```sh
export PATH="$HOME/.pico-sdk/ninja/v1.12.1:$PATH"
cmake -G Ninja -S ports/pico_w -B build-pico
cmake --build build-pico
```

Options:

| CMake option                 | Effect                                                  |
|------------------------------|---------------------------------------------------------|
| `-DPICO_BOARD=pico2_w`       | Build for the Pico 2 W (RP2350)                          |
| `-DFLOSS_PICO_SNOOP=ON`      | Log every HCI packet from boot (also toggled with `p`)  |

Output: `build-pico/floss_gamepad.uf2` (and `.elf` for debugging).

## Flash

Either hold **BOOTSEL** while plugging the board in and copy the `.uf2` onto
the `RPI-RP2` (Pico 2 W: `RP2350`) drive, or with picotool:

```sh
picotool load -x build-pico/floss_gamepad.uf2 -f
```

(`-f` reboots a running board into BOOTSEL first, `-x` starts the firmware.)

## Use

1. Open the USB serial port (115200, any terminal). The boot banner prints the
   device address and the command list; press `h` to see it again.
2. On the host, open the add-device list and pick **FLOSS Gamepad**.
   Pairing is "just works" (no PIN).
3. Test it: Windows `joy.cpl`, Linux `evtest` / `jstest-gtk`, Android or macOS
   with any gamepad tester app, or a browser gamepad tester page.

The board remembers the host (bond in the last 4 KB flash sector). After a
reset it pages the host to reconnect by itself; with no bond it becomes
discoverable again.

LED: fast blink = waiting to pair, slow blink = bonded and waiting /
reconnecting, solid = connected.

### Serial console

| Key | Action                                   |
|-----|------------------------------------------|
| `s` | dump statistics (with tuning hints)       |
| `t` | dump the trace ring                       |
| `r` | reset statistics                          |
| `d` | become discoverable                       |
| `x` | delete all bonds                          |
| `u` | HID virtual-cable unplug (host forgets us)|
| `v` | log level DEBUG                           |
| `q` | log level INFO                            |
| `p` | toggle HCI snoop logging                  |
| `h` | help                                      |

While connected a status line appears every 5 s: reports per second, ACL
acknowledgement latency p50/p99, deferred and coalesced report counts.

## Tuning

- `GAMEPAD_REPORT_INTERVAL_US` (default 1000 = 1 kHz pacing; 0 = send on
  change) and `GAMEPAD_FLUSH_TIMEOUT_MS` (default 0 = reliable) in
  `examples/gamepad/gamepad_app.h`.
- `FLOSS_HID_CONNECTABLE_WHILE_CONNECTED=0` stops page scanning while a host
  is connected, which removes periodic radio interruptions (most consistent
  report timing).
- Feeding real inputs: call `gamepad_app_set_demo(false)` and the
  `gamepad_app_set_*()` setters from the main loop; or call
  `floss_hid_set_input()` directly from an ISR or core 1 (it is lock-free).

## Notes

- The CYW43 driver is used directly (`pico_cyw43_driver` +
  `pico_async_context_poll`), not through `pico_cyw43_arch`, which would pull
  in another host stack. `CYW43_ENABLE_BLUETOOTH=1` selects the combined
  WLAN + radio firmware.
- Writing a bond erases one flash sector (~50 ms with interrupts off). That
  only happens while pairing.
- Device ID uses vendor 0x1209 (pid.codes) with a placeholder product ID
  0x0F10; register your own product ID before shipping a real device.
