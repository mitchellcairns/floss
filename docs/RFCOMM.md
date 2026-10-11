# RFCOMM and the serial port service

Serial cable emulation over Classic links: debug consoles, configuration
tools, serial-profile peers. Compiled when `FLOSS_ENABLE_RFCOMM=1` (the
default). Public API: `include/floss/floss_rfcomm.h`.

**Status: implemented** (server and client DLCs, credit-based flow control,
serial port SDP record). Not yet covered by the ctest suites or verified
over the air; `examples/serial_echo` is not wired into a port build yet
(add `examples/serial_echo/serial_echo_app.c` to your firmware target).

## 1. Model

```
   App: floss_rfcomm_ops_t callbacks (on_open / on_data / on_can_send / on_close)
            |
   Serial port service   floss_spp_init(): server channel + SDP record (0x1101)
            |
   DLCs  (one per server channel and direction; DLCI = channel << 1 | dir)
            |
   Multiplexer session   one per link, DLCI 0 = control (SABM/UA, PN, MSC, RPN, RLS, ...)
            |
   L2CAP channel on PSM 0x0003 (basic mode, requires an encrypted link)
```

| File | Responsibility |
|---|---|
| `src/rfcomm/floss_rfcomm.c` | Public API, server registry, DLC lifecycle, data path, credits; `floss_rfcomm_init()` registers PSM 0x0003 with `FLOSS_SEC_ENCRYPTED` |
| `src/rfcomm/floss_rfcomm_mux.c` | Framing, FCS (CRC-8), frame dispatch, multiplexer commands, control-frame FIFO |
| `src/rfcomm/floss_spp.c` | Serial port SDP record (`floss_spp_init()`) |
| `src/rfcomm/floss_rfcomm_int.h` | Internal session/DLC structures |

## 2. Multiplexer

| Element | Behaviour |
|---|---|
| Session | One L2CAP channel on PSM 0x0003 per link (`FLOSS_MAX_CONNECTIONS` sessions). Incoming channels get a session when they open; `floss_rfcomm_connect()` creates outgoing ones. |
| Frames | SABM / UA / DM / DISC open and close DLCs; UIH carries data and control. FCS over the header. |
| Control (DLCI 0) | PN, MSC, RPN, RLS, Test, FCon/FCoff; anything else is answered with NSC. |
| Control-frame FIFO | Control frames are never dropped: when L2CAP is busy they wait in a per-session queue (`FLOSS_RFCOMM_CTRL_QUEUE_LEN`), and data is held back until it drains (so MSC always precedes data). |
| Frame size | `dlc->mtu` (N1) never exceeds the L2CAP MTU minus `FLOSS_RFCOMM_FRAME_OVERHEAD`; `FLOSS_RFCOMM_DEFAULT_MTU` (127) when the peer does not negotiate. |
| Priority | RFCOMM data is reliable (not flushable) and never jumps the HID input path. |

## 3. Flow control

Credit-based flow control (CFC) is requested on every DLC: one credit = one
frame of at most `dlc->mtu` bytes.

| Direction | Rule |
|---|---|
| TX | `floss_rfcomm_send()` never sends without a credit (`dlc->tx_credits`). Short write or `FLOSS_ERR_BUSY` -> `on_can_send()` fires once more can go. |
| RX, automatic (default) | Credits are topped up after `on_data()` returns, keeping `FLOSS_RFCOMM_RX_CREDITS` (7) outstanding; refilled when half are used, piggybacked on data when possible. |
| RX, manual | `floss_rfcomm_set_manual_credits(dlc, true)`, then `floss_rfcomm_grant(dlc, n)` as your buffer drains. This is end-to-end back-pressure. |
| Peers without CFC | FCon/FCoff and the MSC FC bit are honoured on transmit (`dlc->remote_fc`). |

## 4. API

All calls from the stack context. Server channels are 1..30
(`FLOSS_RFCOMM_CHANNEL_MIN`..`_MAX`).

| Function | Notes |
|---|---|
| `floss_spp_init(channel, service_name, ops, user)` | Registers the server channel and publishes a serial port record (class 0x1101, L2CAP + RFCOMM channel, profile 1.2, service name). The name is copied. Up to `FLOSS_SPP_MAX_INSTANCES` (1). |
| `floss_rfcomm_register_server(channel, ops, user)` / `floss_rfcomm_unregister_server(channel)` | Server channel without an SDP record. Up to `FLOSS_RFCOMM_MAX_SERVERS` (2). Call after `floss_init()`. |
| `floss_rfcomm_connect(handle, channel, ops, user, &dlc)` | Client: starts L2CAP and the multiplexer if needed; result via `on_open()`. |
| `floss_rfcomm_disconnect(dlc)` | DISC; `on_close()` when confirmed. |
| `floss_rfcomm_send(dlc, data, len)` | Copying send. Returns bytes accepted (may be short), `FLOSS_ERR_BUSY`, or another negative error. |
| `floss_rfcomm_can_send_now(dlc)` | True if at least one frame would be accepted. |
| `floss_rfcomm_buf_alloc()` + `floss_rfcomm_send_buf(dlc, buf)` | Zero-copy: write up to `dlc->mtu` bytes at `buf->data`; `send_buf` takes ownership in all cases. |
| `floss_rfcomm_set_manual_credits()`, `floss_rfcomm_grant()` | See flow control. |
| `floss_rfcomm_set_signals(dlc, v24)` | Send our V.24 signals (MSC): `FLOSS_RFCOMM_V24_RTC`, `_RTR`, `_IC`, `_DV`, `_FC`; default `FLOSS_RFCOMM_V24_DEFAULT`. |

`floss_rfcomm_ops_t`:

| Callback | When |
|---|---|
| `on_open(dlc, status)` | `FLOSS_RFCOMM_STATUS_OK`, or `_REFUSED` / `_LINK_LOST` / `_NO_MUX` (the DLC is freed after a failure). |
| `on_data(dlc, data, len)` | At most `dlc->mtu` bytes; `data` valid during the call. |
| `on_can_send(dlc)` | After a short or BUSY send, when credits/buffers are available again. |
| `on_close(dlc)` | Either side closed, or the link dropped (only after a successful open). |

`floss_rfcomm_dlc_t` fields are readable (`state`, `handle`, `channel`,
`mtu`, `tx_credits`, `rx_credits`, `grant_pending`, `remote_v24`, `port`,
`cfc`, ...); only the stack writes them.

Sizing: `FLOSS_RFCOMM_MAX_DLC` (2, in `floss_config.h`) DLC slots shared by
all sessions; the other `FLOSS_RFCOMM_*` / `FLOSS_SPP_*` limits are
overridable defaults in `floss_rfcomm.h`.

## 5. Example: `examples/serial_echo`

`serial_echo_app_init()` publishes "FLOSS Serial" on channel 1, names the
device "FLOSS Serial Echo" and makes it discoverable. Every received byte is
echoed back uppercased.

Design worth copying: the DLC runs in **manual credit mode**. `on_data`
appends to a 4 KB ring (`ECHO_RING_SIZE`) and sends what credits allow;
`on_can_send` sends more; credits are granted only for whole frames the ring
can still absorb. RAM stays fixed and nothing is dropped with CFC peers
(non-CFC peers cannot be throttled; overflow is logged). One session at a
time; a second DLC is closed.

```c
floss_init(&(floss_init_t){ .transport = t });
serial_echo_app_init();
floss_start();
for (;;) floss_poll();
```

## 6. Debugging

- Log module `FLOSS_MOD_RFCOMM`.
- Wireshark decodes RFCOMM from snoop captures (`btrfcomm`).
- Credit starvation: a DLC with `tx_credits == 0` and no incoming UIH frames
  carrying credits. In manual mode, check that the app calls
  `floss_rfcomm_grant()` as it consumes data.
