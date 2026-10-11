#!/usr/bin/env python3
# FLOSS - Free, Lightweight, Open Short-range Stack
# SPDX-License-Identifier: Unlicense
#
# floss_snoop.py - Convert FLOSS "SNOOP" log lines into a btsnoop capture.
#
# Overview
# --------
# With floss_snoop_set_hook(floss_snoop_log_hook) the stack prints every HCI
# packet as one log line:
#
#     SNOOP <t_us> <R|T> <type hex> <hex bytes>
#
#   t_us   floss_hal_time_us() at the transport boundary (uint32, wraps
#          every ~71.6 minutes)
#   R|T    R = controller->host, T = host->controller
#   type   H4 packet indicator (01 cmd, 02 ACL, 03 SCO, 04 event, 05 ISO)
#   bytes  the HCI packet without the indicator, hex, spaces optional
#
# Lines may be embedded in any other output (serial console noise, log
# prefixes, ANSI colours); only the SNOOP part is used. Packets whose length
# does not match their HCI header (a log sink that dropped data) are skipped
# with a warning instead of corrupting the capture.
#
# Output is btsnoop version 1, datalink 1002 (HCI UART / H4), which Wireshark
# opens directly. 32-bit timestamps are unwrapped into a monotonic 64-bit
# timeline and anchored to wall-clock time (now, or --start-time).
#
# Usage:
#     floss_snoop.py capture.log -o capture.btsnoop
#     some_command | floss_snoop.py - -o capture.btsnoop
#
# The classes here are also used by floss_monitor.py for live capture.

import argparse
import re
import struct
import sys
import time

BTSNOOP_MAGIC = b"btsnoop\0"
BTSNOOP_VERSION = 1
BTSNOOP_DATALINK_H4 = 1002
# Microseconds between 0000-01-01 (btsnoop epoch) and 1970-01-01 (Unix epoch).
BTSNOOP_EPOCH_DELTA_US = 0x00DCDDB30F2F8000

PKT_CMD, PKT_ACL, PKT_SCO, PKT_EVT, PKT_ISO = 0x01, 0x02, 0x03, 0x04, 0x05

_ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")
_SNOOP_RE = re.compile(
    r"SNOOP\s+(?P<t>\d+)\s+(?P<dir>[RT])\s+(?P<type>[0-9A-Fa-f]{1,2})(?P<hex>(?:\s*[0-9A-Fa-f]{2})*)\s*$"
)


def expected_length(pkt_type, data):
    """Total HCI packet length implied by its header, or None if unknown."""
    n = len(data)
    if pkt_type == PKT_CMD and n >= 3:
        return 3 + data[2]
    if pkt_type == PKT_ACL and n >= 4:
        return 4 + (data[2] | (data[3] << 8))
    if pkt_type == PKT_SCO and n >= 3:
        return 3 + data[2]
    if pkt_type == PKT_EVT and n >= 2:
        return 2 + data[1]
    if pkt_type == PKT_ISO and n >= 4:
        return 4 + ((data[2] | (data[3] << 8)) & 0x3FFF)
    return None


class TimestampUnwrapper:
    """Turn wrapping uint32 microsecond stamps into a monotonic int timeline.

    Each stamp is interpreted relative to the previous one as a signed 32-bit
    difference, exactly like floss_time_diff() in C. That handles the wrap at
    2^32 as well as tiny backwards steps, as long as consecutive packets are
    less than ~35 minutes apart.
    """

    def __init__(self):
        self.last_raw = None
        self.abs_us = 0
        self.wraps = 0

    def unwrap(self, raw):
        raw &= 0xFFFFFFFF
        if self.last_raw is None:
            self.abs_us = raw
        else:
            delta = ((raw - self.last_raw + 0x80000000) & 0xFFFFFFFF) - 0x80000000
            if delta > 0 and raw < self.last_raw:
                self.wraps += 1
            self.abs_us += delta
        self.last_raw = raw
        return self.abs_us


class BtsnoopWriter:
    """Minimal btsnoop v1 writer (datalink H4: payload starts with the type byte)."""

    def __init__(self, fileobj):
        self.f = fileobj
        self.count = 0
        self.f.write(BTSNOOP_MAGIC + struct.pack(">II", BTSNOOP_VERSION, BTSNOOP_DATALINK_H4))

    def write(self, pkt_type, is_rx, data, unix_us, flush=False):
        payload = bytes([pkt_type]) + bytes(data)
        # Flags: bit 0 = received (controller->host), bit 1 = command/event.
        flags = (1 if is_rx else 0) | (2 if pkt_type in (PKT_CMD, PKT_EVT) else 0)
        ts = unix_us + BTSNOOP_EPOCH_DELTA_US
        self.f.write(struct.pack(">IIIIq", len(payload), len(payload), flags, 0, ts))
        self.f.write(payload)
        self.count += 1
        if flush:
            self.f.flush()


class SnoopConverter:
    """Feed it text lines; it writes every valid SNOOP packet to a BtsnoopWriter."""

    def __init__(self, writer, start_unix_us=None, keep_bad=False, warn=None, flush=False):
        self.writer = writer
        self.unwrapper = TimestampUnwrapper()
        self.start_unix_us = start_unix_us
        self.first_abs = None
        self.keep_bad = keep_bad
        self.warn = warn or (lambda msg: print(msg, file=sys.stderr))
        self.flush = flush
        self.bad = 0
        self.lineno = 0

    @staticmethod
    def parse(line):
        """Return (t_us, is_rx, type, bytes) for a SNOOP line, else None."""
        line = _ANSI_RE.sub("", line).rstrip("\r\n")
        m = _SNOOP_RE.search(line)
        if not m:
            return None
        data = bytes.fromhex("".join(m.group("hex").split()))
        return int(m.group("t")), m.group("dir") == "R", int(m.group("type"), 16), data

    def feed_line(self, line):
        """Process one line. Returns True if a packet was written."""
        self.lineno += 1
        if "SNOOP" not in line:
            return False
        parsed = self.parse(line)
        if parsed is None:
            self.bad += 1
            self.warn("line %d: malformed SNOOP line skipped" % self.lineno)
            return False
        t_us, is_rx, pkt_type, data = parsed
        exp = expected_length(pkt_type, data)
        if exp != len(data) and not self.keep_bad:
            self.bad += 1
            self.warn("line %d: type %02X length %d != header %s, skipped (lost log data?)"
                      % (self.lineno, pkt_type, len(data), exp))
            return False
        abs_us = self.unwrapper.unwrap(t_us)
        if self.first_abs is None:
            self.first_abs = abs_us
            if self.start_unix_us is None:
                self.start_unix_us = int(time.time() * 1_000_000)
        self.writer.write(pkt_type, is_rx, data, self.start_unix_us + (abs_us - self.first_abs), self.flush)
        return True


def _iter_lines(path):
    if path == "-":
        stream = sys.stdin.buffer
        for raw in stream:
            yield raw.decode("utf-8", errors="replace")
    else:
        with open(path, "rb") as f:
            for raw in f:
                yield raw.decode("utf-8", errors="replace")


def main(argv=None):
    ap = argparse.ArgumentParser(description="Convert FLOSS SNOOP log lines to a btsnoop (Wireshark) file.")
    ap.add_argument("inputs", nargs="*", default=["-"], help="log files ('-' = stdin, default)")
    ap.add_argument("-o", "--output", required=True, help="output .btsnoop file")
    ap.add_argument("--start-time", type=float, default=None,
                    help="Unix time (seconds) of the first packet (default: now)")
    ap.add_argument("--keep-bad", action="store_true",
                    help="keep packets whose length does not match their header")
    ap.add_argument("-q", "--quiet", action="store_true", help="no per-line warnings")
    args = ap.parse_args(argv)

    start = None if args.start_time is None else int(args.start_time * 1_000_000)
    warn = (lambda msg: None) if args.quiet else None
    with open(args.output, "wb") as out:
        conv = SnoopConverter(BtsnoopWriter(out), start_unix_us=start, keep_bad=args.keep_bad, warn=warn)
        for path in args.inputs:
            for line in _iter_lines(path):
                conv.feed_line(line)
    print("%s: %d packets, %d skipped, %d clock wraps"
          % (args.output, conv.writer.count, conv.bad, conv.unwrapper.wraps), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
