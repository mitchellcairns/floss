# FLOSS tools

Host-side helpers for debugging and measuring FLOSS.

| Tool | Runs on | What it does |
|---|---|---|
| `floss_snoop.py` | Python 3 (standard library) | Converts `SNOOP` log lines into a `.btsnoop` file for Wireshark |
| `floss_monitor.py` | Python 3 + [pyserial](https://pypi.org/project/pyserial/) (`python -m pip install pyserial`) | Serial console: shows and saves the log, captures HCI live, sends keys |
| `pico_flash.ps1` | Windows PowerShell | Flashes a running Pico W via the 1200-baud bootloader request, then shows its console |
| `hostprobe/` | Windows 10+, .NET SDK | `floss_hostprobe`: pair/scan helpers and host-side report timing analysis; `experiment.ps1` automates latency experiments |

## Getting HCI packets into the log

Turn on the snoop hook in the firmware (or desktop program) before
`floss_start()`:

```c
floss_snoop_set_hook(floss_snoop_log_hook);
```

From then on, every HCI packet crossing the transport is printed as one line:

```
SNOOP <t_us> <R|T> <type> <hex bytes>
SNOOP 1520331 T 01 030C00
SNOOP 1520874 R 04 0E0401030C00
```

* `t_us` is `floss_hal_time_us()`, a 32-bit microsecond clock that wraps
  every ~71.6 minutes. The tools unwrap it.
* `R` = controller to host, `T` = host to controller.
* `type` is the H4 packet type: `01` command, `02` ACL, `03` SCO,
  `04` event, `05` ISO.

Long packets are written in several chunks but always end up as one line.
Printing every packet costs time, especially over a slow UART: at
115200 baud a 64-byte ACL packet becomes a ~160-character line, which takes
~14 ms to print. Use a fast console
(USB CDC) or only turn snooping on while you are chasing a problem.

The desktop host (`ports/posix`, `floss_gamepad_host --snoop --log run.log`)
writes the same lines to stderr and, with `--log`, to a file.

## floss_snoop.py: convert a saved log

```
python tools/floss_snoop.py run.log -o run.btsnoop
python tools/floss_snoop.py - -o run.btsnoop   < run.log    # from stdin
```

Then open `run.btsnoop` in Wireshark.

* Other log lines, log prefixes and ANSI colours are ignored.
* If a packet's length does not match its HCI header (for example the log
  sink dropped bytes), the packet is skipped and a warning is printed.
  `--keep-bad` keeps such packets anyway.
* Timestamps are anchored to the time of conversion. Pass
  `--start-time <unix seconds>` to choose the time of the first packet.

## floss_monitor.py: live console and capture

```
python tools/floss_monitor.py COM7                 # Windows
python tools/floss_monitor.py /dev/ttyACM0         # Linux / macOS
```

* Prints the device log on the console. `SNOOP` lines are hidden unless you
  pass `--show-snoop`.
* Appends every raw line to `floss_<date-time>.log` (`--log FILE`,
  `--no-log`).
* Writes HCI packets to `floss_<date-time>.btsnoop` as they arrive
  (`--snoop FILE`, `--no-snoop`). The file is flushed after every packet, so
  you can open it in Wireshark while the capture is still running (reload to
  see newer packets).
* Each key you press is sent to the device right away, without Enter. The
  example firmware uses `s` for a stats dump and `t` for a trace dump.
  Pass `--no-keys` to turn this off.
* Quit with **Ctrl-]** or **Ctrl-C**.

Other options: `-b/--baud` (ignored by USB CDC ports) and `--rtscts`.

## Reading the other debug output

* `floss_stats_dump()` prints counters and latency histograms
  (count/min/avg/p50/p99/max, in microseconds), followed by tuning hints
  worked out from the numbers, such as "poll_gap p99 > 1000us -> application
  starves the stack".
* `floss_trace_dump()` prints the binary trace ring, oldest entry first,
  with times relative to the first entry and the gap to the previous one.
  Call `floss_trace_freeze(true)` as soon as you detect the problem, so the
  history that led up to it is not overwritten.

## pico_flash.ps1: flash without BOOTSEL (Windows)

```powershell
powershell -File tools/pico_flash.ps1 -Uf2 build-pico/floss_gamepad.uf2 [-ReadMs 8000]
```

1. Finds the board's USB serial port (USB vendor ID 0x2E8A) and opens it at
   1200 baud: the Pico SDK's USB stdio reboots into the ROM bootloader.
2. Waits for the `RPI-RP2` drive and copies the `.uf2` (flash + restart).
3. Waits for the serial port to come back and prints the console for
   `-ReadMs` ms (0 = skip).

Needed because picotool cannot reset a running board on a stock Windows
install (no WinUSB driver on the reset interface). If no drive appears, hold
BOOTSEL while plugging in and retry. The script only recognises the
`RPI-RP2` label (Pico W), not the Pico 2 W's `RP2350`.

## hostprobe/: measure report timing from the host (Windows)

Build once (net9.0 Windows target, see `floss_hostprobe.csproj`):

```powershell
cd tools/hostprobe; dotnet build -c Release -o out; cd ../..
```

`tools/hostprobe/out/floss_hostprobe.exe` commands:

| Command | Does |
|---|---|
| `scan <name> [--le]` | Find an unpaired device whose name contains `<name>` (default "FLOSS") |
| `pair <name> [--le]` | Pair it, auto-accepting the confirmation |
| `unpair <name> [--le]` / `paired <name> [--le]` | Forget / find a paired device |
| `conns` | List paired Classic and LE devices and whether they are connected |
| `pads <seconds>` | Read the first game controller through the OS game-input API and analyse report timing |

`pads` needs the example firmware's demo mode: one axis carries a
millisecond counter, which the probe finds automatically. It prints
distinct reports/s, delivered vs generated (coalesced %), host inter-arrival,
device milliseconds between delivered reports, latency jitter (host time
minus device time with clock drift removed, 0 = fastest delivery seen) as
min/p50/p90/p99/max, and the structure of gaps longer than 3 ms.

`experiment.ps1` runs a series of configurations against the Classic
firmware:

```powershell
powershell -File tools/hostprobe/experiment.ps1 -Flash -Configs m,4,P -Seconds 10
```

For each comma-separated config it types those console tuning keys (see
[ports/pico_w/README.md](../ports/pico_w/README.md#serial-console)), resets the
device stats (`r`), runs `floss_hostprobe pads`, then dumps the device stats
(`s`) and prints the relevant lines next to the host view. `-Flash` first
flashes `build-pico/floss_gamepad.uf2` with `pico_flash.ps1` and waits for
the bonded host to reconnect. Results and method:
[docs/PERFORMANCE.md](../docs/PERFORMANCE.md).
