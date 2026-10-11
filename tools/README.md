# FLOSS tools

Host-side helpers for debugging FLOSS. They are plain Python 3 scripts and
need nothing beyond the standard library, except `floss_monitor.py`, which
needs [pyserial](https://pypi.org/project/pyserial/)
(`python -m pip install pyserial`).

| Script             | What it does                                                        |
|--------------------|---------------------------------------------------------------------|
| `floss_snoop.py`   | Converts `SNOOP` log lines into a `.btsnoop` file for Wireshark      |
| `floss_monitor.py` | Serial console: shows and saves the log, captures HCI live, sends keys |

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
