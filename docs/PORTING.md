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
| Data | Small values (< 64 bytes) under 16-bit keys. |
| Keys used | `FLOSS_STORE_KEY_LAST_HOST` (0x0001), `FLOSS_STORE_KEY_BOND_BASE` (0x0100) + index `0..FLOSS_MAX_BONDS-1`. Applications use `>= FLOSS_STORE_KEY_APP_BASE` (0x8000). |
| read | Return bytes read (<= len) or `FLOSS_ERR_NOT_FOUND`. |
| write | Create or replace. Return `FLOSS_OK` or a negative error. Called when a link key is created (pairing): a few times per pairing, never per packet. Flash erase time is acceptable but blocks the radio loop; prefer a write-behind or a wear-levelled sector. |
| delete | Deleting a missing key is not an error. |
| Failure | Errors are logged; the link works but the bond will not survive a reset. |

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

```c
static const uint8_t k_script[] = {
    0x18, 0xFC, 0x04, 0x00, 0x10, 0x0E, 0x00,   /* hypothetical vendor command 0xFC18, 4 param bytes */
};
const floss_chipset_t my_chipset = {
    .name = "mychip", .set_addr_opcode = 0xFC01,
    .init_script = k_script, .init_script_len = sizeof k_script,
};
```

CYW43439 ships as `floss_chipset_cyw43` (set-address opcode 0xFC01).
`floss_cyw43_default_addr()` derives the recommended address (WLAN MAC + 1).

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
| `FLOSS_ASSERT(expr)` | calls `floss_assert_fail()` | Define before including to replace. |

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
(BTstack) and its HCI transport. FLOSS wants the bus and firmware loader, not
that stack, so `ports/pico_w` **bypasses `cyw43_arch`** and links the lower
pieces directly:

| SDK piece | Why |
|---|---|
| `pico_cyw43_driver` | `cyw43_driver_init()`: bus, WLAN core, interrupt glue |
| `cybt_shared_bus` | radio-core firmware download and the HCI read/write calls on the shared bus |
| `pico_async_context_poll` | the async context the cyw43 driver runs its work in |

Run loop (sketch; the real one is in `ports/pico_w`):

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
- The radio firmware comes from the cyw43-driver inside the Pico SDK; FLOSS
  does not ship it (see LEGAL.md).
- `floss_hid_set_input()` may be called from core 1 (e.g. a tight input
  sampling loop); the HAL critical section uses a hardware spinlock.
- The exact SDK library list and compile definitions are in
  `ports/pico_w/CMakeLists.txt`.
