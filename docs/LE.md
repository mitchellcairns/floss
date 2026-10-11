# Low Energy (LE) design

**Status: in progress.** The internal contracts are fixed in
`src/floss_internal.h`; the public API is being written in parallel. API
sections below say *see header once merged*. Enable with `FLOSS_ENABLE_LE=1`
(default).

## 1. Goals

- A peripheral-role LE stack sufficient for **HID-over-GATT** gamepads,
  keyboards and mice, with the same latency philosophy as Classic HID.
- 7.5 ms connection intervals, latest-wins input mailbox, no heap.
- Pairing with **legacy** and **Secure Connections** SMP, using the
  controller for P-256 ECDH so no big-number code is needed on the MCU.

## 2. Structure

```
        App / HID-over-GATT service
                 |
           GATT server  (static attribute table)
                 |
     ATT (CID 0x0004)   LE signaling (CID 0x0005)   SMP (CID 0x0006)
                 \               |                    /
                  floss_le_on_pdu()  <- HCI (LE links, conn.le = true)
                  floss_le_on_hci_event()  <- LE Meta events
```

| Piece | Source | Responsibility |
|---|---|---|
| LE GAP | `src/le/floss_le.c` | Advertising, connection handling, connection parameter updates |
| ATT / GATT | `src/le/floss_le.c` (or split) | ATT server, MTU exchange, static attribute table, notifications |
| SMP | `src/le/floss_smp*.c` | Pairing, key distribution, bonding, encryption |

Routing (see ARCHITECTURE.md section 9): HCI forwards LE meta events,
encryption changes, PDUs, link loss and credits to `floss_le_on_*`; LE and SMP
talk through `floss_le_send_fixed()`, `floss_le_on_security_changed()` and
`floss_smp_on_*`.

## 3. LE GAP

- **Advertising**: connectable undirected advertising with flags, name,
  appearance (gamepad 0x03C4) and the HID service UUID 0x1812. Fast interval
  after start-up/disconnect, slow after a timeout.
- **Connection parameters**: after connection and encryption, request a
  7.5 ms interval (6 x 1.25 ms), peripheral latency 0, and a supervision
  timeout suited to the product, via the LE signaling Connection Parameter
  Update Request (or the controller's connection-update procedure where
  supported). The central decides; the result is reported in the stats.
- **Peripheral latency trade-off**: latency > 0 saves power but adds up to
  `latency x interval` delay to the first report after idle.
- Address: public address from the controller, or a static random address.

## 4. ATT / GATT server

- **Static attribute table**: handles, types, permissions and value pointers
  live in `const` arrays built at compile time. Reads and writes are served
  directly from it; dynamic values (reports, battery level) use read/write
  callbacks.
- MTU exchange up to `FLOSS_ATT_MTU` (default 185).
- Permission checks use the link security level reported by SMP (1-4).
- Notifications are credit-aware, like Classic ACL: input reports go out as
  soon as a buffer is free, newest-wins.

### Services

| Service | UUID | Characteristics |
|---|---|---|
| Generic Access | 0x1800 | Device Name 0x2A00, Appearance 0x2A01, Peripheral Preferred Connection Parameters 0x2A04 |
| Generic Attribute | 0x1801 | Service Changed 0x2A05 |
| Device Information | 0x180A | Manufacturer Name 0x2A29, PnP ID 0x2A50 (VID source, VID, PID, version) |
| Battery | 0x180F | Battery Level 0x2A19 (notify) |
| HID | 0x1812 | HID Information 0x2A4A, Report Map 0x2A4B, HID Control Point 0x2A4C, Report 0x2A4D (+ Report Reference 0x2908, CCCD 0x2902), Protocol Mode 0x2A4E |

HID-over-GATT reuses the report descriptor, mailbox semantics and
`on_report_tick` idea from the Classic HID module. HID attributes require an
encrypted link.

## 5. SMP (peripheral)

| Feature | Approach |
|---|---|
| Legacy pairing (Just Works) | c1/s1 confirm and STK generation with AES-128 (software or the controller's LE Encrypt) |
| Secure Connections | P-256 public key from the controller (LE Read Local P-256 Public Key, subevent 0x08); DHKey via LE Generate DHKey (subevent 0x09); f4/f5/f6/g2 with software AES-CMAC |
| Encryption | LTK Request (subevent 0x05) answered from the bond store |
| Key distribution | LTK (legacy), IRK and identity address; stored via the HAL key/value store |
| Security levels | Reported to LE with `floss_le_on_security_changed()`: 1 none, 2 unauthenticated, 3 authenticated, 4 SC authenticated |

`floss_hal_random()` must be a real entropy source for LE products (PORTING.md).

## 6. API

*See header once merged* (`include/floss/floss_le.h` or similar): advertising
control, connection parameter requests, GATT service registration,
HID-over-GATT configuration, bond management.

## 7. Debugging

The snoop hook captures LE traffic like Classic (Wireshark: `btatt`,
`btsmp`, LE meta events under `bthci_evt`). LE-specific log modules and stats will be
added with the implementation.
