#!/usr/bin/env python3
# FLOSS - Free, Lightweight, Open Short-range Stack
# SPDX-License-Identifier: Unlicense
#
# floss_monitor.py - Serial console for FLOSS devices with live HCI capture.
#
# Overview
# --------
# Opens the device's log serial port (e.g. the Pico W's USB CDC console) and:
#   * prints every line to the console (SNOOP lines hidden unless --show-snoop),
#   * appends every raw line to a log file,
#   * converts SNOOP lines into a .btsnoop file on the fly (flushed per packet,
#     so Wireshark can open it at any moment; re-open to refresh),
#   * forwards single keystrokes to the device without waiting for Enter, so
#     firmware console commands (e.g. 's' = stats dump, 't' = trace dump in
#     the examples) work interactively.
#
# Quit with Ctrl-] or Ctrl-C.
#
# Requires pyserial:  python -m pip install pyserial

import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from floss_snoop import BtsnoopWriter, SnoopConverter  # noqa: E402

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover - depends on the environment
    serial = None

QUIT_KEY = "\x1d"  # Ctrl-]


class KeyReader:
    """Non-blocking single-key input for Windows and POSIX terminals."""

    def __init__(self):
        self.old = None
        if os.name != "nt" and sys.stdin.isatty():
            import termios
            import tty
            self.old = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())

    def get(self, timeout=0.05):
        """Return one key as str, or None after timeout."""
        if os.name == "nt":
            import msvcrt
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                if msvcrt.kbhit():
                    ch = msvcrt.getwch()
                    if ch in ("\x00", "\xe0"):  # function/arrow key prefix: swallow
                        msvcrt.getwch()
                        return None
                    return ch
                time.sleep(0.01)
            return None
        import select
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if r:
            return sys.stdin.read(1)
        return None

    def close(self):
        if self.old is not None:
            import termios
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.old)
            self.old = None


class Monitor:
    def __init__(self, ser, log_file, snoop_file, show_snoop):
        self.ser = ser
        self.log = log_file
        self.show_snoop = show_snoop
        self.stop = threading.Event()
        self.lines = 0
        self.conv = None
        if snoop_file is not None:
            self.conv = SnoopConverter(BtsnoopWriter(snoop_file), flush=True,
                                       warn=lambda msg: self.status("snoop: " + msg))

    def status(self, msg):
        sys.stdout.write("\r[monitor] %s\n" % msg)
        sys.stdout.flush()

    def handle_line(self, raw):
        text = raw.decode("utf-8", errors="replace")
        self.lines += 1
        if self.log is not None:
            self.log.write(raw)
            self.log.flush()
        is_snoop = "SNOOP" in text
        if self.conv is not None:
            self.conv.feed_line(text)  # every line, so warnings show real line numbers
        if not is_snoop or self.show_snoop:
            sys.stdout.write(text if text.endswith("\n") else text + "\n")
            sys.stdout.flush()

    def reader(self):
        pending = b""
        while not self.stop.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception as exc:  # device unplugged, etc.
                self.status("serial error: %s" % exc)
                self.stop.set()
                break
            if not chunk:
                continue
            pending += chunk
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                self.handle_line(line + b"\n")
        if pending:
            self.handle_line(pending)

    def run(self, keys_enabled):
        t = threading.Thread(target=self.reader, daemon=True)
        t.start()
        keys = KeyReader() if keys_enabled else None
        try:
            while not self.stop.is_set():
                if keys is None:
                    time.sleep(0.1)
                    continue
                ch = keys.get()
                if ch is None:
                    continue
                if ch == QUIT_KEY:
                    break
                self.ser.write(ch.encode("utf-8"))
        except KeyboardInterrupt:
            pass
        finally:
            self.stop.set()
            if keys is not None:
                keys.close()
            t.join(timeout=1.0)


def main(argv=None):
    stamp = time.strftime("%Y%m%d-%H%M%S")
    ap = argparse.ArgumentParser(description="FLOSS serial monitor with live btsnoop capture.")
    ap.add_argument("port", help="serial port (COM5, /dev/ttyACM0, ...)")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="baud rate (ignored by USB CDC)")
    ap.add_argument("--rtscts", action="store_true", help="hardware flow control")
    ap.add_argument("--log", default="floss_%s.log" % stamp, help="log file (default: floss_<time>.log)")
    ap.add_argument("--no-log", action="store_true", help="do not write a log file")
    ap.add_argument("--snoop", default="floss_%s.btsnoop" % stamp,
                    help="btsnoop output (default: floss_<time>.btsnoop)")
    ap.add_argument("--no-snoop", action="store_true", help="do not write a btsnoop file")
    ap.add_argument("--show-snoop", action="store_true", help="also print SNOOP lines on the console")
    ap.add_argument("--no-keys", action="store_true", help="do not forward keystrokes to the device")
    args = ap.parse_args(argv)

    if serial is None:
        print("floss_monitor.py needs pyserial. Install it with:\n"
              "    python -m pip install pyserial\n"
              "(To convert an existing log instead, use floss_snoop.py, which has no dependencies.)",
              file=sys.stderr)
        return 1

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1, rtscts=args.rtscts)
    except serial.SerialException as exc:
        print("cannot open %s: %s" % (args.port, exc), file=sys.stderr)
        return 1

    log = None if args.no_log else open(args.log, "ab")
    snoop = None if args.no_snoop else open(args.snoop, "wb")
    mon = Monitor(ser, log, snoop, args.show_snoop)
    mon.status("%s @ %d  log=%s  snoop=%s  (Ctrl-] quits)"
               % (args.port, args.baud, None if log is None else args.log, None if snoop is None else args.snoop))
    try:
        mon.run(keys_enabled=not args.no_keys)
    finally:
        ser.close()
        if log is not None:
            log.close()
        if snoop is not None:
            snoop.close()
        packets = 0 if mon.conv is None else mon.conv.writer.count
        mon.status("%d lines, %d packets captured" % (mon.lines, packets))
    return 0


if __name__ == "__main__":
    sys.exit(main())
