# RFCOMM and serial port design

**Status: in progress.** The init hook (`floss_rfcomm_init()`, which
registers PSM 0x0003) is declared in `src/floss_internal.h`; the public API is
being written in parallel. API sections below say *see header once merged*.
Enable with `FLOSS_ENABLE_RFCOMM=1` (default).

## 1. Goals

- A serial-port emulation over Classic links: debug consoles, configuration
  tools, legacy serial-profile peers.
- Credit-based flow control so a slow consumer never drops data and a fast
  one never stalls the HID stream.
- Static memory: `FLOSS_RFCOMM_MAX_DLC` data channels (default 2).

## 2. Structure

```
   App (serial port API: open / write / on_data / on_can_write)
            |
   Serial port service  (SDP record: Serial Port 0x1101, RFCOMM channel N)
            |
   RFCOMM multiplexer   (one per link, DLCI 0 = control channel)
            |
   L2CAP channel on PSM 0x0003 (basic mode)
```

## 3. Multiplexer

| Element | Design |
|---|---|
| Session | One L2CAP channel on PSM 0x0003 per link; DLCI 0 carries multiplexer control. |
| Frames | SABM / UA / DM / DISC to open and close DLCs; UIH for data and control. FCS (CRC-8) over the header as specified. |
| Parameter negotiation (PN) | Frame size bounded by the L2CAP MTU; requests credit-based flow control. |
| Modem status (MSC) | Sent after DLC open; signals (RTC/RTR/DV) surfaced to the app. |
| Remote port negotiation (RPN) | Accepted and echoed; baud rate etc. are informational for a virtual port. |
| Flow control | Credit-based: the peer may send one UIH frame per credit we granted. We grant credits as the app consumes data; we stop sending when our credits reach 0. Aggregate flow control (FCON/FCOFF) is answered for older peers. |
| Priority | RFCOMM data is not flushable and never jumps the HID input path (no `FLOSS_BUF_F_PRIORITY`). |

## 4. Serial port service

- SDP record: Service Class ID List 0x1101 (Serial Port); Protocol Descriptor
  List L2CAP (0x0100) + RFCOMM (0x0003, server channel N); service name.
- The service accepts incoming DLCs on its server channel and requires an
  encrypted link (same GAP security as HID).

## 5. API

*See header once merged* (`include/floss/floss_rfcomm.h` or similar):
register a server channel, connect to a peer channel, credit-aware write
(`FLOSS_ERR_BUSY` + "can write" callback, like `floss_l2cap_send`), receive
callback, close.

## 6. Debugging

Wireshark decodes RFCOMM from snoop captures (`btrfcomm`). Credit starvation
shows up as a DLC with zero credits and no UIH frames carrying new credits
from the peer.
