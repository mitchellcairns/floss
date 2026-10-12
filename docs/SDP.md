# SDP client

Asks a peer's SDP server which services it offers and how to reach them.
The typical use: a device that pages a host and opens RFCOMM to an
application-defined service, where the host assigns the RFCOMM server
channel at run time and publishes it in its SDP record. The device looks
the channel up first, then connects.

Compiled when `FLOSS_ENABLE_SDP_CLIENT=1` (the default). Public API:
`include/floss/floss_sdp.h`; implementation: `src/sdp/floss_sdp_client.c`.
The SDP server side (records the stack publishes) is described in the
header and in [ARCHITECTURE.md](ARCHITECTURE.md).

## 1. Model

```
   floss_sdp_query_rfcomm_channel(handle, uuid128, cb, user)
        |  attribute 0x0004 (ProtocolDescriptorList) of records matching uuid128
        v
   floss_sdp_query_ssa(handle, uuid, uuid_len, attr_lo, attr_hi, cb, user)
        |  L2CAP channel to PSM 0x0001 on the existing link (no security)
        |  ServiceSearchAttributeRequest -> Response, continuation rounds
        |  chunks reassembled into FLOSS_SDP_CLIENT_BUF bytes
        v
   cb(handle, status, attribute_lists, len, user)   then the channel closes
```

- One query at a time (stack-wide); a second one returns `FLOSS_ERR_BUSY`.
  The callback may start the next query.
- The callback fires exactly once per accepted query: on success, on an
  SDP Error Response, on timeout, on a malformed answer, or when the
  channel or link goes away.
- `MaximumAttributeByteCount` is sized so a whole response fits our L2CAP
  receive MTU; the server's continuation state is echoed until it is empty.
- No heap and no blocking: the result buffer is static, everything runs in
  the stack context (call from `floss_poll()` callbacks or between polls).

## 2. API

| Function | Notes |
|---|---|
| `floss_sdp_query_ssa(handle, uuid_be, uuid_len, attr_lo, attr_hi, cb, user)` | General query. `uuid_len` 2, 4 or 16 (big-endian, as on the wire); attribute range `attr_lo..attr_hi`. Returns `FLOSS_OK` (callback follows), `FLOSS_ERR_BUSY`, `FLOSS_ERR_INVALID`, or an `floss_l2cap_connect()` error (no callback). |
| `floss_sdp_query_rfcomm_channel(handle, uuid128_be, cb, user)` | Query for attribute 0x0004 of records matching the 128-bit service UUID, then `floss_sdp_rfcomm_channel()`. `cb(handle, status, channel, user)`. |
| `floss_sdp_query_busy()` | True while a query runs. |
| `floss_sdp_iter_init(&it, data, len)` / `floss_sdp_iter_next(&it, &rec, &rec_len)` | Walk the records of a result. `data` may be the AttributeLists sequence or a single attribute list. |
| `floss_sdp_find_attr(data, len, attr_id, &value, &value_len)` | Attribute of the first record that has it; `value` is the whole data element (header included, decode with `floss_de_header()`). |
| `floss_sdp_rfcomm_channel(data, len, &channel)` | RFCOMM server channel from the first record whose ProtocolDescriptorList (or one of its alternatives) contains RFCOMM (UUID 0x0003 in 16, 32 or 128-bit form) followed by a channel 1..30. |

The walker functions are pure and bounds-checked: every element is decoded
with `floss_de_header()`, which rejects any length that runs past the bytes
that remain, so hostile results produce `FLOSS_ERR_PROTOCOL`, never an
out-of-bounds read.

### Status values

| `status` | Meaning |
|---|---|
| `0` | Success (`attribute_lists`/`len` valid during the callback; an empty sequence `35 00` when nothing matched) |
| `> 0` | The peer sent an Error Response; the value is its ErrorCode (`FLOSS_SDP_ERR_INVALID_SYNTAX`, ...) |
| `FLOSS_ERR_TIMEOUT` | No answer for `FLOSS_SDP_CLIENT_TIMEOUT_MS` (10 s) |
| `FLOSS_ERR_IO` | Channel did not open, the peer closed it, or the link dropped |
| `FLOSS_ERR_UNSUPPORTED` | The peer refused PSM 0x0001 |
| `FLOSS_ERR_NOMEM` | Result larger than `FLOSS_SDP_CLIENT_BUF` (512) |
| `FLOSS_ERR_PROTOCOL` | Malformed or unexpected response, or malformed data elements |
| `FLOSS_ERR_NOT_FOUND` | RFCOMM lookup only: the query worked but no record carries an RFCOMM descriptor |

### Configuration

| Macro | Default | Meaning |
|---|---|---|
| `FLOSS_ENABLE_SDP_CLIENT` | 1 | Compile the client. |
| `FLOSS_SDP_CLIENT_BUF` | 512 | Reassembled AttributeLists bytes one query can return. |
| `FLOSS_SDP_CLIENT_TIMEOUT_MS` | 10000 | A query fails after this long without an answer (restarted by every response). |

The query also needs a free L2CAP channel slot (`FLOSS_MAX_L2CAP_CHANNELS`)
while it runs.

## 3. Example: look up the channel, then connect RFCOMM

```c
/* The host's application service, 128-bit UUID in big-endian byte order. */
static const uint8_t k_service[16] = {
    0x6E, 0x40, 0x00, 0x01, 0xB5, 0xA3, 0xF3, 0x93,
    0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E,
};

static floss_rfcomm_dlc_t *s_dlc;
static const floss_rfcomm_ops_t k_ops = { .on_open = app_on_open, .on_data = app_on_data };

static void on_channel(uint16_t handle, int status, uint8_t channel, void *user)
{
    if (status != 0) {
        /* FLOSS_ERR_NOT_FOUND: service not published; > 0: SDP error code; < 0: see table */
        FLOSS_LOGW(APP, "service lookup failed: %d", status);
        return;
    }
    floss_err_t err = floss_rfcomm_connect(handle, channel, &k_ops, NULL, &s_dlc);
    if (err != FLOSS_OK) {
        FLOSS_LOGW(APP, "rfcomm connect: %s", floss_err_str(err));
    }
}

/* Once the ACL link to the host is open (e.g. from a GAP CONNECTED event): */
floss_sdp_query_rfcomm_channel(handle, k_service, on_channel, NULL);
```

Generic queries work the same way; for example, all attributes of every
serial port record, then the service name of the first one:

```c
static void on_records(uint16_t handle, int status, const uint8_t *lists, uint16_t len, void *user)
{
    const uint8_t *name;
    uint16_t name_len;
    if (status == 0 && floss_sdp_find_attr(lists, len, FLOSS_ATTR_SERVICE_NAME, &name, &name_len) == FLOSS_OK) {
        uint8_t type;
        uint32_t text_len;
        uint16_t hdr = floss_de_header(name, name_len, &type, &text_len);
        /* name + hdr .. text_len bytes of text */
    }
}

static const uint8_t k_serial_port[2] = {0x11, 0x01};
floss_sdp_query_ssa(handle, k_serial_port, 2, 0x0000, 0xFFFF, on_records, NULL);
```

## 4. Tests

`tests/test_sdp_client.c` drives the client against the simulator's
scripted SDP server (`floss_sim_sdp_serve()`, `floss_sim_sdp_serve_error()`,
`floss_sim_sdp_serve_silence()` in `transport/sim/floss_sim.h`): RFCOMM
channel found with 16/32/128-bit RFCOMM UUIDs, continuation across many
responses, no matching record, Error Response, malformed data elements,
result overflow, timeout, link loss mid-query, busy and chained queries,
and the walker against truncated and corrupted results.
