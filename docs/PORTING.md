# Porting FLOSS

A port is two things:

1. **A HAL**: one C file implementing every function in
   `include/floss/floss_hal.h` (plain link-time functions, no tables).
2. **A transport**: a `floss_transport_t` that moves HCI packets to and from
   the controller (`include/floss/floss_transport.h`). Often you reuse
   `transport/h4` and only write a `floss_stream_t`.

Reference ports: `ports/posix/` (desktop, POSIX and Windows) and
`ports/pico_w/`.

---

## 1. The HAL contract

All HAL functions are called from the stack context only, except
`floss_hal_time_us()`.

### `uint32_t floss_hal_time_us(void)` (required)

| | |
|---|---|
| Returns | Free-running monotonic microsecond counter; wraps at 2^32. |
| Context | **Any** (ISR, other core). |
| Latency | Called on every hot path: must be a register read or close to it. |
| Resolution | 1 us expected. Coarser resolution directly limits pacing precision. |
| Failure | None allowed. Must never go backwards. |

```c
uint32_t floss_hal_time_us(void) { return timer_hw->timerawl; }           /* RP2040 */

uint32_t floss_hal_time_us(void) {                                        /* POSIX */
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000000ull + ts.tv_nsec / 1000);
}
```

### Persistent storage (required; may be RAM-only)

```c
int         floss_hal_store_read(uint16_t key, void *buf, uint16_t len);
floss_err_t floss_hal_store_write(uint16_t key, const void *buf, uint16_t len);
floss_err_t floss_hal_store_delete(uint16_t key);
```

| | |
|---|---|
| Data | Small values (<= 64 bytes) under 16-bit keys. |
| Keys used | See the key map below. |
| read | Return bytes read (<= len) or `FLOSS_ERR_NOT_FOUND`. |
| write | Create or replace. Return `FLOSS_OK` or a negative error. Called when keys are created or bonds change (pairing, bonded LE reconnection that changes CCCDs), never per packet. Flash erase time is acceptable but blocks the radio loop (it shows up in the `transport_poll_time` stat, see DEBUGGING.md); prefer a write-behind or a wear-levelled sector. |
| delete | Deleting a missing key is not an error. |
| Failure | Errors are logged; the link works but the bond will not survive a reset. |

Key map (`i` = bond index `0..FLOSS_MAX_BONDS-1`):

| Key | Macro | Header | Contents | Size |
|---|---|---|---|---|
| 0x0001 | `FLOSS_STORE_KEY_LAST_HOST` | `floss_hal.h` | Address of the most recent Classic bond | 6 |
| 0x0100 + i | `FLOSS_STORE_KEY_BOND_BASE` | `floss_hal.h` | Classic bond (address, link key, key type) | 31 |
| 0x01F0 | `FLOSS_SMP_STORE_KEY_IRK` | `floss_smp.h` | Our LE Identity Resolving Key | 16 |
| 0x0200 + i | `FLOSS_SMP_STORE_KEY_BOND_BASE` | `floss_smp.h` | LE bond (identity, LTK, peer IRK) | 56 |
| 0x0280 | `FLOSS_LE_STORE_KEY_STATIC_ADDR` | `floss_le.h` | Generated LE random static address | 6 |
| 0x0300 + i | `FLOSS_GATT_STORE_KEY_CCCD_BASE` | `floss_gatt.h` | CCCD values of LE bond i | <= 59 |
| >= 0x8000 | `FLOSS_STORE_KEY_APP_BASE` | `floss_hal.h` | Free for the application | |

With `FLOSS_MAX_BONDS = 4` and LE enabled the stack needs up to 15 entries
(the Pico W HAL keeps 16 of 64 bytes).

RAM-only stub (fine for tests and bring-up):

```c
static struct { uint16_t key; uint8_t len; uint8_t v[64]; } g_kv[16];

int floss_hal_store_read(uint16_t key, void *buf, uint16_t len) {
    for (size_t i = 0; i < FLOSS_ARRAY_SIZE(g_kv); i++)
        if (g_kv[i].len && g_kv[i].key == key) {
            uint16_t n = FLOSS_MIN(len, g_kv[i].len);
            memcpy(buf, g_kv[i].v, n);
            return n;
        }
    return FLOSS_ERR_NOT_FOUND;
}
floss_err_t floss_hal_store_write(uint16_t key, const void *buf, uint16_t len) {
    if (len == 0 || len > 64) return FLOSS_ERR_INVALID;
    floss_hal_store_delete(key);
    for (size_t i = 0; i < FLOSS_ARRAY_SIZE(g_kv); i++)
        if (!g_kv[i].len) {
            g_kv[i].key = key; g_kv[i].len = (uint8_t)len; memcpy(g_kv[i].v, buf, len);
            return FLOSS_OK;
        }
    return FLOSS_ERR_NOMEM;
}
floss_err_t floss_hal_store_delete(uint16_t key) {
    for (size_t i = 0; i < FLOSS_ARRAY_SIZE(g_kv); i++)
        if (g_kv[i].len && g_kv[i].key == key) g_kv[i].len = 0;
    return FLOSS_OK;
}
```

### `void floss_hal_random(uint8_t *buf, size_t len)` (required)

Fill with random bytes. Classic HID needs no cryptographic quality (the
controller generates pairing keys). LE SMP uses it for pairing nonces and
keys, so an LE product should use a real entropy source (ROSC/TRNG,
`getrandom()`, `BCryptGenRandom()`). No failure path.

### `void floss_hal_log_write(const char *text, size_t len)` (required, may discard)

Receives a complete, formatted line including the newline. **Must not block
for long**: a stalled log sink stalls `floss_poll()`, which stalls the radio.
Drop data when the sink is busy (e.g. USB serial with no terminal attached).

```c
void floss_hal_log_write(const char *text, size_t len) {
    ring_write_or_drop(&g_log_ring, text, len);   /* drained by idle code */
}
```

### `void floss_hal_debug_pin(uint8_t pin, bool level)` (required, may be no-op)

Used only when `FLOSS_ENABLE_DEBUG_PINS=1`. `pin` is logical
(`FLOSS_DBG_PIN_POLL`, `_HID_TX`, `_TRANS_TX`, `_TRANS_RX`); map to GPIOs with a
single-cycle write (e.g. `sio_hw->gpio_set/clr` on RP2040).

### Critical sections (required)

```c
uint32_t floss_hal_critical_enter(void);
void     floss_hal_critical_exit(uint32_t state);
```

Protect only the few instructions shared with ISR/other-core-safe APIs
(`floss_hid_set_input()`). Must nest correctly via the returned state.

| Platform | Implementation |
|---|---|
| Single-core MCU | save + disable interrupts / restore |
| RP2040 / RP2350 (dual core) | `spin_lock_blocking()` / `spin_unlock()` on a claimed spinlock |
| Desktop | a mutex (state unused) |

---

## 2. Writing a transport

### The `floss_transport_ops_t` contract

| Field | Contract |
|---|---|
| `name` | Short name for logs (`"h4"`, `"cyw43"`). |
| `headroom` | Bytes the driver writes **in front of** `pkt` in `send()`. Must be `<= FLOSS_TRANSPORT_MAX_HEADROOM` (raise that macro if you need more). This makes TX zero-copy. |
| `open(t)` | Power up, download firmware, configure the bus. Blocking is OK. Return `FLOSS_OK` or an error (`FLOSS_ERR_IO`). `t->rx`/`t->rx_user` are already set. |
| `close(t)` | Shut down. |
| `send(t, type, pkt, len)` | `pkt` starts at the HCI header (no H4 byte). `pkt[-headroom..-1]` is writable scratch. **Synchronous ownership**: on `FLOSS_OK` the stack reuses the buffer immediately, so DMA drivers must copy or complete before returning. Return `FLOSS_ERR_BUSY` when you cannot accept a packet now; the stack keeps it and retries on a later poll (`transport_busy` stat). Return `FLOSS_ERR_IO` on a hard failure (`transport_errors`). |
| `poll(t)` | Called on every `floss_poll()`. Read whatever is available, call `floss_transport_deliver(t, type, pkt, len)` once per **complete** packet, return the count. Never block. |

All ops run in the stack context. If your hardware signals RX by interrupt,
the ISR should only buffer bytes / set a flag and wake the main loop; parsing
and delivery happen in `poll()`.

### Skeleton: a hypothetical SPI (or USB) transport

```c
#include "floss/floss.h"

#define MY_HDR 4   /* e.g. [len_lo len_hi type flags] framing on the bus */

typedef struct {
    uint8_t rx[MY_HDR + FLOSS_BUF_SIZE];
} my_spi_t;

static my_spi_t g_spi;

static floss_err_t my_open(floss_transport_t *t) {
    my_bus_init();                       /* clocks, pins, reset the radio */
    return my_firmware_load() ? FLOSS_OK : FLOSS_ERR_IO;
}

static void my_close(floss_transport_t *t) { my_bus_deinit(); }

static floss_err_t my_send(floss_transport_t *t, uint8_t type, uint8_t *pkt, uint16_t len) {
    if (!my_bus_tx_ready()) return FLOSS_ERR_BUSY;          /* back-pressure */
    uint8_t *frame = pkt - MY_HDR;                          /* zero-copy header */
    frame[0] = (uint8_t)len; frame[1] = (uint8_t)(len >> 8);
    frame[2] = type;          frame[3] = 0;
    my_bus_write_blocking(frame, (uint16_t)(len + MY_HDR)); /* done before return */
    return FLOSS_OK;
}

static int my_poll(floss_transport_t *t) {
    my_spi_t *s = t->ctx;
    int n = 0;
    uint16_t got;
    while ((got = my_bus_read_frame(s->rx, sizeof s->rx)) > 0) {   /* non-blocking */
        uint16_t len = floss_get_le16(s->rx);
        floss_transport_deliver(t, s->rx[2], s->rx + MY_HDR, len);
        n++;
    }
    return n;
}

static const floss_transport_ops_t my_ops = {
    .name = "myspi", .headroom = MY_HDR,
    .open = my_open, .close = my_close, .send = my_send, .poll = my_poll,
};

static floss_transport_t g_transport = { .ops = &my_ops, .ctx = &g_spi };

floss_transport_t *my_spi_transport(void) { return &g_transport; }
```

For **USB** (e.g. a USB HCI dongle with a host-mode USB stack): commands go
out on the control endpoint, ACL on bulk OUT; events come in on the interrupt
endpoint and ACL on bulk IN. `headroom` is 0 (endpoints carry the type),
`send()` returns `FLOSS_ERR_BUSY` while the endpoint's previous transfer is
pending (copy into a staging buffer if the transfer completes asynchronously),
and `poll()` delivers completed IN transfers.

### H4 over a UART: just implement `floss_stream_t`

`transport/h4` handles framing and resynchronisation. You provide a
non-blocking byte pipe:

```c
#include "floss/floss.h"
#include "floss_h4.h"

static int uart_write(void *ctx, const uint8_t *d, uint16_t len) {
    return my_uart_write_nonblocking(d, len);      /* bytes accepted, may be < len */
}
static int uart_read(void *ctx, uint8_t *buf, uint16_t max) {
    return my_uart_read_available(buf, max);       /* 0 = nothing right now */
}

static const floss_stream_t g_uart = { .ctx = NULL, .write = uart_write, .read = uart_read };
static floss_h4_t g_h4;

floss_init(&(floss_init_t){ .transport = floss_h4_init(&g_h4, &g_uart) });
```

The same works for USB-CDC, a TCP socket to a controller emulator, or a pipe.
Use hardware flow control (RTS/CTS) on real UARTs; at 115200 baud a
20-byte report costs ~1.7 ms of wire time, so run the controller UART at
1 Mbaud or more for low latency (via a chipset `init_script` if the controller
needs a vendor baud-rate command).

---

## 3. Chipset quirks: `floss_chipset_t`

Vendor knowledge that sits between the generic HCI bring-up and the radio.
Pass it as `floss_init_t.chipset` (NULL = generic).

| Field | Use |
|---|---|
| `name` | For logs. |
| `set_addr_opcode` | Vendor command that sets the public address (6 bytes, wire order). Used when the app calls `floss_hci_set_addr_override()` before `floss_start()`. 0 = unsupported. |
| `init_script`, `init_script_len` | Extra commands sent right after `HCI_Reset`, as a flat byte script of repeated `[opcode_lo, opcode_hi, param_len, params...]`. |
| `completion_batch` | How many ACL packets the controller tends to acknowledge per Number Of Completed Packets event (0/1 = one by one). Read back with `floss_hci_completion_batch()`. |

**Why `completion_batch` exists.** The HID and HOGP input paths cap the
packets waiting inside the controller (`max_inflight`), because a queued
report can no longer be replaced by newer state. Some controllers report
completions late and in batches: the CYW43439 reports them two at a time,
3-8 ms after the packets left, and a single outstanding packet waits
~250 ms. With a cap of 2 the stack would think the link is full while both
packets are long delivered, and the link would idle. The effective cap is
therefore `max_inflight x completion_batch` (4 on the Pico W). Measure your
controller with the `acl_ack_latency` histogram and a snoop capture; the
derivation is in [PERFORMANCE.md](PERFORMANCE.md).

```c
static const uint8_t k_script[] = {
    0x18, 0xFC, 0x04, 0x00, 0x10, 0x0E, 0x00,   /* hypothetical vendor command 0xFC18, 4 param bytes */
};
const floss_chipset_t my_chipset = {
    .name = "mychip", .set_addr_opcode = 0xFC01,
    .init_script = k_script, .init_script_len = sizeof k_script,
};
```

CYW43439 ships as `floss_chipset_cyw43` (set-address opcode 0xFC01,
`completion_batch = 2`). `floss_cyw43_default_addr()` derives the
recommended address (WLAN MAC + 1).

---

## 4. Configuration

`include/floss/floss_config.h` holds defaults only. Override by:

- defining `FLOSS_USER_CONFIG_FILE` (included first), e.g. in CMake:
  ```cmake
  target_compile_definitions(floss PUBLIC FLOSS_USER_CONFIG_FILE="my_floss_config.h")
  ```
- or passing `-DNAME=value` on the compiler command line.

Make sure every translation unit that includes FLOSS headers sees the same
values (use `PUBLIC` in CMake).

| Macro | Default | Meaning |
|---|---|---|
| `FLOSS_MAX_CONNECTIONS` | 2 | Simultaneous ACL links (Classic + LE). |
| `FLOSS_MAX_L2CAP_CHANNELS` | 6 | Dynamic L2CAP channels across all links (HID 2 + SDP 1). |
| `FLOSS_BUF_COUNT` | 16 | Buffers in the shared pool. |
| `FLOSS_BUF_SIZE` | 704 | Payload bytes per buffer; must hold ACL hdr + L2CAP hdr + `FLOSS_L2CAP_MTU` and the largest command (3 + 255). |
| `FLOSS_TRANSPORT_MAX_HEADROOM` | 4 | Largest `ops->headroom` of any transport you link. |
| `FLOSS_BUF_HEADROOM` | derived (16) | `(MAX_HEADROOM + 4 + 4 + 2)` rounded up to 4. Not overridable. |
| `FLOSS_L2CAP_MTU` | 672 | Our receive MTU for dynamic channels. |
| `FLOSS_HCI_CMD_QUEUE_LEN` | 16 | Commands waiting for controller command credits. |
| `FLOSS_HCI_CMD_TIMEOUT_US` | 2000000 | Per-command timeout. |
| `FLOSS_ACL_INFLIGHT_MAX` | 16 | In-flight ACL timestamps tracked per link (ack latency stats). |
| `FLOSS_MAX_BONDS` | 4 | Bonded peers in storage. |
| `FLOSS_SDP_MAX_RECORDS` | 4 | Registered SDP records. |
| `FLOSS_HID_MAX_REPORT` | 64 | Max HID report payload (excluding ID). |
| `FLOSS_HID_MAX_INPUT_IDS` | 4 | Input report IDs tracked by the mailbox. |
| `FLOSS_ENABLE_LE` | 1 | LE GAP, ATT/GATT, SMP, HID-over-GATT. |
| `FLOSS_ENABLE_RFCOMM` | 1 | RFCOMM + serial port. |
| `FLOSS_RFCOMM_MAX_DLC` | 2 | RFCOMM data channels. |
| `FLOSS_ATT_MTU` | 185 | ATT MTU offered on LE links. |
| `FLOSS_LOG_LEVEL` | `FLOSS_LOG_INFO` (3) | Compile-time log ceiling: NONE 0, ERROR 1, WARN 2, INFO 3, DEBUG 4, TRACE 5. |
| `FLOSS_LOG_LINE_MAX` | 160 | Max formatted log line. |
| `FLOSS_TRACE_ENTRIES` | 256 | Trace ring size (power of two, 0 disables). |
| `FLOSS_ENABLE_STATS` | 1 | Counters + histograms. |
| `FLOSS_ENABLE_SNOOP` | 1 | HCI snoop hook support. |
| `FLOSS_ENABLE_DEBUG_PINS` | 0 | Call `floss_hal_debug_pin()` around hot paths. |

The table above is exactly `floss_config.h`. Other defaults can be
overridden the same way (`FLOSS_USER_CONFIG_FILE` or `-D`); they live next
to the code they size:

| Macro | Default | Where | Meaning |
|---|---|---|---|
| `FLOSS_ASSERT(expr)` | calls `floss_assert_fail()` | `floss_debug.h` | Define before including to replace. |
| `FLOSS_GATT_MAX_TABLES` | 8 | `floss_gatt.h` | Registered attribute tables. |
| `FLOSS_ATT_PREPARE_MAX` | 512 | `floss_gatt.h` | Prepare Write queue bytes. |
| `FLOSS_HOGP_MAX_REPORTS` | 6 | `floss_hogp.h` | Reports in the HID service. |
| `FLOSS_RFCOMM_MAX_SERVERS` | 2 | `floss_rfcomm.h` | Registered RFCOMM server channels. |
| `FLOSS_RFCOMM_RX_CREDITS` | 7 | `floss_rfcomm.h` | Receive credits kept outstanding per DLC (automatic mode). |
| `FLOSS_RFCOMM_CTRL_QUEUE_LEN` | 6 | `floss_rfcomm.h` | Control frames buffered per session while L2CAP is busy. |
| `FLOSS_SPP_MAX_INSTANCES` | 1 | `floss_rfcomm.h` | Serial port records. |
| `FLOSS_HID_CONNECTABLE_WHILE_CONNECTED` | 0 | `src/hid/floss_hid.c` | Keep page scan on while a host is connected (costs air time, see HID_GAMEPAD.md). |
| `FLOSS_HID_MAX_DESCRIPTOR` | 512 | `src/hid/floss_hid.c` | Largest HID report descriptor. |
| `FLOSS_HID_RECONNECT_RETRIES`, `FLOSS_HID_RECONNECT_BACKOFF_US` | 3, 1 000 000 | `src/hid/floss_hid.c` | Reconnect paging attempts. |
| `FLOSS_TRACE_POLL_MIN_GAP_US` | 500 | `src/floss.c` | Only polls at least this late are traced. |
| `FLOSS_CYW43_FAST_BUS` | 1 | `transport/cyw43/floss_cyw43.c` | CYW43 bulk ring access (section 6). |

Sizing tips: watch `floss_buf_low_watermark()` and `buf_alloc_failures`; a
gamepad-only build can drop `FLOSS_BUF_COUNT` to 8, `FLOSS_ENABLE_LE`/`RFCOMM`
to 0 and `FLOSS_L2CAP_MTU`/`FLOSS_BUF_SIZE` accordingly.

---

## 5. Porting checklist

- [ ] HAL file implements all 9 functions of `floss_hal.h`.
- [ ] `floss_hal_time_us()` is 1 us resolution, monotonic, callable from ISRs.
- [ ] `floss_hal_log_write()` drops instead of blocking.
- [ ] Storage survives a reset (or you accept re-pairing every boot).
- [ ] Critical section works across cores if `floss_hid_set_input()` is called from another core.
- [ ] Transport `headroom <= FLOSS_TRANSPORT_MAX_HEADROOM`.
- [ ] `send()` is finished with `pkt` before returning `FLOSS_OK`; returns `FLOSS_ERR_BUSY` (not blocking) when full.
- [ ] `poll()` never blocks and delivers only complete packets.
- [ ] Run loop calls `floss_poll()` at least every `idle_us` and whenever the transport has data.
- [ ] `FLOSS_GAP_EVT_READY` arrives; `floss_hci_controller()` shows sane `acl_mtu`/`acl_total`.
- [ ] Snoop capture opens in Wireshark (see DEBUGGING.md).
- [ ] `floss_stats_dump()` after a session: no `cmd_timeouts`, no `transport_errors`, `poll_gap` p99 small.

---

## 6. How the Pico W port works

The Pico SDK's `pico_cyw43_arch` libraries initialise the CYW43439 and, with
the radio HCI enabled, also start the SDK's bundled third-party host stack
and its HCI transport. FLOSS wants the bus and firmware loader, not
that stack, so `ports/pico_w` **bypasses `cyw43_arch`** and links the lower
pieces directly:

| SDK piece | Why |
|---|---|
| `pico_cyw43_driver` | `cyw43_driver_init()`: bus, WLAN core, interrupt glue; its shared-bus radio code (`cybt_shared_bus_driver.h`) downloads the radio-core firmware and provides the HCI ring access |
| `cyw43_driver_picow` | the Pico W bus driver underneath |
| `pico_async_context_poll` | the async context the cyw43 driver runs its work in |

Run loop (sketch; the real one in `ports/pico_w/main.c` spins instead of
waiting, because the demo favours latency over power):

```c
static async_context_poll_t g_ctx;

int main(void) {
    async_context_poll_init_with_defaults(&g_ctx);
    cyw43_driver_init(&g_ctx.core);                  /* no cyw43_arch_init() */

    floss_addr_t addr;
    floss_cyw43_default_addr(&addr);
    floss_hci_set_addr_override(&addr);
    floss_init(&(floss_init_t){ .transport = floss_cyw43_transport(),
                                .chipset   = &floss_chipset_cyw43 });
    app_init();
    floss_start();

    for (;;) {
        async_context_poll(&g_ctx.core);             /* service the cyw43 driver */
        uint32_t idle_us = floss_poll();
        app_work();
        async_context_wait_for_work_until(&g_ctx.core,
            make_timeout_time_us(FLOSS_MIN(idle_us, 1000)));
    }
}
```

Notes:

- The transport header on the shared bus is `[len0 len1 len2 type]`, so
  `headroom = 4` and TX stays zero-copy.
- **Fast bus path** (`FLOSS_CYW43_FAST_BUS`, default 1). The SDK's packet
  calls do a full wake/index/data/index/interrupt handshake per call and two
  calls per received packet (~250 us per TX packet, ~600 us per RX packet,
  loop blocked). The fast path drives the same radio rings through the SDK's
  shared-bus primitives: RX reads all ring indices in one transfer, pulls
  everything pending in one bulk read, acknowledges once; TX is one index
  read, data write, index write and interrupt. Measured gain in
  [PERFORMANCE.md](PERFORMANCE.md) (finding 5). Build with
  `FLOSS_CYW43_FAST_BUS=0` to fall back to the SDK calls.
- The chipset descriptor sets `completion_batch = 2` (section 3).
- The radio firmware comes from the cyw43-driver inside the Pico SDK; FLOSS
  does not ship it (see LEGAL.md).
- `floss_hid_set_input()` may be called from core 1 (e.g. a tight input
  sampling loop); the HAL critical section uses a hardware spinlock.
- The exact SDK library list and compile definitions are in
  `ports/pico_w/CMakeLists.txt`.
- Toolchain setup, Windows build notes (prebuilt `pioasm` / `picotool`
  directories) and flashing are in
  [ports/pico_w/README.md](../ports/pico_w/README.md).
