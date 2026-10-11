# Low Energy (LE)

Peripheral-role LE stack: advertising, LE links, ATT/GATT server, SMP
pairing/bonding and HID over GATT (HOGP). Compiled when `FLOSS_ENABLE_LE=1`
(the default).

**Status: verified on hardware.** A Pico W running `floss_le_gamepad` pairs
with Windows 11 through FLOSS's SMP (LE Secure Connections, just works,
controller-assisted P-256), reconnects bonded without host action, and
delivers one HID-over-GATT report every connection event (66.7 reports/s at
the 15 ms interval Windows grants). Numbers: [PERFORMANCE.md](PERFORMANCE.md#le-hid-over-gatt).
In-tree unit coverage is limited to the ATT MTU exchange
(`le_att_mtu_exchange` in `tests/test_l2cap.c`).

## 1. Map

```
 APP / HOGP (floss_hogp.h)          reports over GATT notifications
   |  floss_gatt_*                  attribute database, notify / indicate
 GATT DB (floss_gatt.c) + ATT server (floss_att.c)    CID 0x0004
   |                    SMP (floss_smp.c)             CID 0x0006
 LE GAP (floss_le.c): advertising, link table, connection parameters,
   |                  LE signaling (CID 0x0005), fixed-channel routing
 HCI: LE meta events, LE ACL (LE credits, or shared with BR/EDR)
```

| File | Responsibility | Header |
|---|---|---|
| `src/le/floss_le.c` | Advertising, LE link table, connection parameter / PHY / data length policy, LE signaling, routing by CID, hooks from HCI | `floss_le.h` |
| `src/le/floss_att.c` | ATT server: requests, MTU exchange, permission checks, notifications/indications, deferred responses | `floss_gatt.h` |
| `src/le/floss_gatt.c` | Attribute database: registration, lookup, value access, CCCDs (incl. per-bond persistence) | `floss_gatt.h` |
| `src/le/floss_gatt_services.c` | GAP (0x1800), GATT (0x1801), Device Information (0x180A), Battery (0x180F) | `floss_gatt.h` |
| `src/le/floss_hogp.c` | HID service (0x1812), input mailbox, pacing | `floss_hogp.h` |
| `src/le/floss_smp.c` | SMP state machine, LTK requests, bond database | `floss_smp.h` |
| `src/le/floss_smp_crypto.c`, `floss_aes.c` | e, c1, s1, ah, f4, f5, f6, g2; software AES-128 and AES-CMAC | internal |

How LE plugs into HCI and SMP (hooks in `src/floss_internal.h`) is in
[ARCHITECTURE.md section 9](ARCHITECTURE.md#9-le-architecture).

## 2. Bring-up

Everything below is configured after `floss_init()` and before
`floss_start()`; advertising is applied once the controller is ready.

```c
#include "floss/floss_le.h"
#include "floss/floss_gatt.h"
#include "floss/floss_hogp.h"

floss_le_set_name("FLOSS LE Gamepad");
floss_le_set_appearance(FLOSS_LE_APPEARANCE_GAMEPAD);

floss_hogp_config_t cfg;
floss_hogp_config_defaults(&cfg);
cfg.report_map = k_report_map;  cfg.report_map_len = sizeof k_report_map;
cfg.reports = k_reports;        cfg.report_count = FLOSS_ARRAY_SIZE(k_reports);
cfg.report_interval_us = FLOSS_HOGP_INTERVAL_CONN;   /* one report per connection event */
floss_hogp_init(&cfg, &k_callbacks);                 /* HID + DIS + Battery services */

static const uint16_t uuids[] = { FLOSS_GATT_SVC_HID };
floss_le_adv_set_standard(uuids, 1);                 /* flags + appearance + UUIDs + name */
floss_le_adv_start();
```

Then, from any context: `floss_hogp_set_input(id, data, len)`.

## 3. LE GAP (`floss_le.h`)

### Identity and address

| API | Notes |
|---|---|
| `floss_le_set_name()` / `floss_le_name()` | Copied, up to `FLOSS_LE_NAME_MAX` (32) bytes. Used by advertising and the Device Name characteristic. |
| `floss_le_set_appearance()` / `floss_le_appearance()` | `FLOSS_LE_APPEARANCE_GAMEPAD` (0x03C4), `_KEYBOARD`, `_MOUSE`, `_JOYSTICK`, `_HID`. |
| `floss_le_set_static_address()` | Random static address; `NULL` = generate one with `floss_hal_random()` and keep it under `FLOSS_LE_STORE_KEY_STATIC_ADDR`. Used only with `own_addr_type = FLOSS_LE_ADDR_RANDOM`. |
| `floss_le_own_address()` | Current identity address and type. |

### Advertising

| API | Notes |
|---|---|
| `floss_le_ad_t` + `floss_le_ad_init/_add/_flags/_name/_uuid16/_appearance/_tx_power()` | Builds a 31-byte AD list; an add that does not fit returns false and sets `overflow`. `floss_le_ad_name()` falls back to a shortened name. |
| `floss_le_adv_set_data()`, `floss_le_adv_set_scan_rsp()` | Raw payloads (<= `FLOSS_LE_ADV_DATA_MAX`). |
| `floss_le_adv_set_standard(uuids, n)` | Advertising data = flags (general, LE only) + appearance + 16-bit UUIDs + name if it fits; scan response = complete name. |
| `floss_le_adv_params_defaults()` / `floss_le_adv_set_params()` | Connectable undirected (ADV_IND). |
| `floss_le_adv_start()` / `_stop()` / `_active()` | Start is deferred until the controller is ready. |

`floss_le_adv_params_t` defaults: `interval_min` 0x30 / `interval_max` 0x60
(30-60 ms), `own_addr_type` public, `channel_map` 0x07, `filter_policy` 0,
`auto_restart` true (re-advertise when a link drops), `while_connected` false.

### Connection parameters

`floss_le_link_policy_t` (set with `floss_le_set_link_policy()`):

| Field | Default | Effect |
|---|---|---|
| `preferred` | interval 6..12 (7.5-15 ms), latency 0, timeout 300 (3 s) | What we ask the central for; also served as the PPCP characteristic. |
| `update_delay_us` | 1 000 000 | As peripheral, ask this long after connecting (centrals dislike changes during service discovery); 0 = never. |
| `prefer_2m_phy` | true | Request the 2M PHY when both sides support it. |
| `max_data_length` | true | Request 251-byte LE data length when supported. |

The request uses LE Connection Update when the controller supports the
Connection Parameters Request procedure, otherwise the LE signaling
Connection Parameter Update Request on CID 0x0005. `floss_le_request_conn_params()`
asks immediately. The central decides; the outcome arrives as
`FLOSS_LE_EVT_CONN_PARAMS`.

Peripheral latency > 0 saves power but adds up to `latency x interval` to the
first report after idle; keep 0 for controllers.

### Links and events

| API | Notes |
|---|---|
| `floss_le_link(handle)` | `floss_le_link_t` snapshot: peer, role, interval/latency/timeout, `att_mtu`, `security_level`, `encrypted`, `tx_octets`, PHYs. |
| `floss_le_link_index()`, `floss_le_link_handle_at()`, `floss_le_link_count()` | Per-link tables (CCCDs, HOGP) are indexed by `floss_le_link_index()`. |
| `floss_le_disconnect(handle, reason)` | |
| `floss_le_add_listener(fn, user)` | Up to `FLOSS_LE_MAX_LISTENERS` (4). |

Events (`floss_le_evt_type_t`): `CONNECTED`, `CONNECT_FAILED`, `DISCONNECTED`,
`CONN_PARAMS`, `MTU`, `SECURITY`, `DATA_LENGTH`, `PHY`, `ADV_STARTED`,
`ADV_STOPPED`, `INDICATION_DONE` (all prefixed `FLOSS_LE_EVT_`).

## 4. GATT server (`floss_gatt.h`)

### Attribute tables

Services are `static const floss_gatt_attr_t` arrays (they live in flash),
registered once with `floss_gatt_register(table, count)` before
`floss_start()`. Tables are concatenated into one handle space
(handle = base + index); GAP and GATT are registered by the stack first and
start at handle 1.

```c
static uint8_t level = 100;
static floss_gatt_cccd_t level_cccd;
static const floss_gatt_attr_t battery[] = {
    FLOSS_GATT_PRIMARY_SERVICE16(FLOSS_GATT_SVC_BATTERY),
    FLOSS_GATT_CHARACTERISTIC(FLOSS_GATT_PROP_READ | FLOSS_GATT_PROP_NOTIFY),
    FLOSS_GATT_VALUE16(FLOSS_GATT_CHR_BATTERY_LEVEL, FLOSS_GATT_P_READ, &level, 1, NULL),
    FLOSS_GATT_CCCD(level_cccd),
};
uint16_t base = floss_gatt_register(battery, FLOSS_ARRAY_SIZE(battery));  /* value handle = base + 2 */
```

| Macro | Entry |
|---|---|
| `FLOSS_GATT_PRIMARY_SERVICE16(uuid)`, `_PRIMARY_SERVICE128(bytes)`, `_SECONDARY_SERVICE16(uuid)` | Service declaration (value generated from the UUID) |
| `FLOSS_GATT_CHARACTERISTIC(props)` | Declaration; the **next** entry must be its value (value generated from it) |
| `FLOSS_GATT_VALUE16(uuid, perm, ptr, maxlen, lenp)`, `FLOSS_GATT_VALUE128(...)` | Value with storage (`lenp` NULL = fixed length) |
| `FLOSS_GATT_VALUE16_CB(uuid, perm, rd, wr, maxlen, user)` | Value served by `floss_gatt_read_fn` / `floss_gatt_write_fn` |
| `FLOSS_GATT_CCCD(store)`, `FLOSS_GATT_CCCD_P(store, perm)` | CCCD; `store` is a `floss_gatt_cccd_t` (one value per link) |
| `FLOSS_GATT_DESCRIPTOR16(uuid, perm, ptr, len)` | Constant descriptor (e.g. Report Reference) |

Permissions: `FLOSS_GATT_P_READ`, `_WRITE`, `_READ_ENC`, `_WRITE_ENC`,
`_READ_AUTHEN`, `_WRITE_AUTHEN`, shorthands `_RD_ENC`, `_WR_ENC`, `_RW`,
`_RW_ENC`. ENC needs security level >= 2, AUTHEN >= 3. When a client hits a
secured attribute on an unencrypted link, ATT answers with the security
error and, as peripheral, asks SMP for security once
(`floss_smp_request_security()`).

Lookup: `floss_gatt_find(start, end, type16)`, `floss_gatt_attr(handle)`,
`floss_gatt_last_handle()`, `floss_gatt_cccd_get(conn, value_handle)`.
Sizing: `FLOSS_GATT_MAX_TABLES` (8), `FLOSS_ATT_PREPARE_MAX` (512), ATT MTU
offered `FLOSS_ATT_MTU` (185, in `floss_config.h`).

### Server-initiated PDUs

| API | Notes |
|---|---|
| `floss_gatt_notify(conn, handle, data, len)` | Only if the client's CCCD has notify set; truncated to MTU - 3; `FLOSS_ERR_BUSY` when no buffer. |
| `floss_gatt_notify_all(handle, data, len)` | Every subscribed link; returns the count. |
| `floss_gatt_notify_alloc()` + `floss_gatt_notify_send()` | Zero-copy: write the value in place (HOGP uses this). Does not check the CCCD. |
| `floss_gatt_indicate()` / `floss_gatt_indication_pending()` | One outstanding per link; completion as `FLOSS_LE_EVT_INDICATION_DONE`. |

### Standard services

| Service | Registered by | Contents |
|---|---|---|
| GAP 0x1800 | stack (`floss_gatt_services_register_core()`) | Device Name, Appearance, PPCP (from the link policy) |
| GATT 0x1801 | stack | Service Changed + CCCD |
| Device Information 0x180A | `floss_gatt_dis_register(&floss_gatt_dis_t)` (HOGP does it for you) | Manufacturer, Model, Firmware Revision, PnP ID |
| Battery 0x180F | `floss_gatt_battery_register(level)`; update with `floss_gatt_battery_set()` | Battery Level (read + notify) |

### CCCDs persist per bond

A bonded client does **not** rewrite its subscriptions after reconnecting,
so without persistence a rebooted device would never send reports again.
`floss_gatt_cccd_sync(conn)` runs whenever a link's security changes: for a
bonded peer, a link with CCCDs set saves them, a link with none set (fresh
reconnection) restores them. Records live under
`FLOSS_GATT_STORE_KEY_CCCD_BASE + bond index` (up to 13 handle/value pairs
per bond).

## 5. HID over GATT (`floss_hogp.h`)

`floss_hogp_init(cfg, callbacks)` builds the HID service: Report Map (long
read), HID Information, HID Control Point, one Report characteristic per
`floss_hogp_report_t` (with Report Reference and, for inputs, a CCCD), and
Protocol Mode when `boot_device`. Every HID characteristic requires an
encrypted link, which is what makes hosts pair. It also registers Device
Information and (by default) Battery. One HID service per device.

### `floss_hogp_config_t`

| Field | Default | Notes |
|---|---|---|
| `report_map`, `report_map_len` | - | Required; <= 512 bytes. |
| `reports`, `report_count` | - | Up to `FLOSS_HOGP_MAX_REPORTS` (6); `len` excludes the ID byte. |
| `country_code`, `remote_wake`, `boot_device` | 0 / false | HID Information / Protocol Mode. |
| `normally_connectable` | true | HID Information flag. |
| `manufacturer`, `model`, `vendor_id_source`, `vendor_id`, `product_id`, `version` | NULL / 0 | Device Information + PnP ID (`vendor_id_source` 0 = omit PnP ID). |
| `battery_service`, `battery_level` | true, 100 | |
| `report_interval_us` | 0 | 0 = notify on change; > 0 = paced period; **`FLOSS_HOGP_INTERVAL_CONN`** = one report per connection interval, following interval updates (recommended). |
| `send_unchanged` | false | Paced: send even if the data did not change. |
| `max_inflight` | 2 | Un-acknowledged packets per link before reports wait, multiplied by the chipset's `completion_batch`. LE acknowledges a notification one connection event later, so 1 skips every other event. |

### Input path

`floss_hogp_set_input(id, data, len)` copies into the report's seqlock slot
(ISR / other-core safe, one writer per ID). The stack notifies every
subscribed host with the newest contents when the link has room (fewer than
`max_inflight x completion_batch` packets in the controller), on a pacing
tick, or when credits free up. On LE the report ID is not sent; the host maps
characteristics to IDs through the Report Reference descriptor.

Other calls: `floss_hogp_send_input_now()` (bypass the mailbox),
`floss_hogp_connected()` (encrypted **and** subscribed to an input report),
`floss_hogp_handle()`, `floss_hogp_protocol()`, `floss_hogp_suspended()`,
`floss_hogp_set_report_interval()`.

Callbacks (`floss_hogp_callbacks_t`): `on_connected`, `on_disconnected`,
`on_output_report`, `on_get_report`, `on_set_report`, `on_set_protocol`,
`on_control`, `on_report_tick`.

HOGP feeds the same stats as Classic HID (`hid_reports_sent`,
`hid_reports_coalesced`, `hid_send_deferred`, `hid_report_age`,
`hid_send_interval`, `hid_output_reports`).

## 6. SMP (`floss_smp.h`)

Pairing on CID 0x0006 and LTK requests from the controller. P-256 ECDH runs
in the controller (LE Read Local P-256 Public Key / LE Generate DHKey), the
f4/f5/f6/g2 and c1/s1 functions use software AES-CMAC.

| Feature | Support |
|---|---|
| LE legacy pairing | Just Works, Passkey Entry |
| LE Secure Connections | Just Works, Numeric Comparison, Passkey Entry |
| OOB, CSRK signing, cross-transport keys | Not supported |
| Roles | Responder (peripheral) primarily; initiator via `floss_smp_request_security()` |
| Privacy | Peer IRKs stored; resolvable private addresses of bonded peers recognised (`floss_smp_resolve()`) |

### `floss_smp_config_t` (`floss_smp_config_defaults()`, `floss_smp_set_config()`)

| Field | Default |
|---|---|
| `io_capability` | `FLOSS_SMP_IO_NO_INPUT_NO_OUTPUT` |
| `pairable`, `bondable` | true, true |
| `mitm` | false |
| `secure_connections`, `secure_connections_only` | true, false |
| `min_key_size` | 7 |
| `local_keys`, `peer_keys` | `FLOSS_SMP_DIST_ENC` + `FLOSS_SMP_DIST_ID` |

### Events and answers

`floss_smp_add_listener(fn, user)` (up to `FLOSS_SMP_MAX_LISTENERS`, 2):

| Event | Answer |
|---|---|
| `FLOSS_SMP_EVT_PAIRING_STARTED`, `_PAIRING_COMPLETE` (`bonded`, `level`), `_PAIRING_FAILED` (`reason` = `FLOSS_SMP_REASON_*`) | - |
| `FLOSS_SMP_EVT_NUMERIC_COMPARISON` (`passkey`) | `floss_smp_confirm(handle, accept)` |
| `FLOSS_SMP_EVT_PASSKEY_DISPLAY` (`passkey`) | - |
| `FLOSS_SMP_EVT_PASSKEY_INPUT` | `floss_smp_passkey_reply(handle, passkey)` or `FLOSS_SMP_PASSKEY_REJECT` |
| `FLOSS_SMP_EVT_SECURITY_CHANGED`, `_BOND_DELETED` | - |

Security levels (`floss_smp_security_level()`, also `floss_le_link_t.security_level`):
1 none, 2 encrypted unauthenticated, 3 encrypted authenticated (MITM),
4 LE Secure Connections authenticated.

### Bonds

`floss_smp_bond_count()`, `floss_smp_bond_at(i)`, `floss_smp_resolve(type, addr)`,
`floss_smp_bond_delete(type, addr)`, `floss_smp_bond_delete_all()`.
A `floss_smp_bond_t` holds the identity address, LTK (+ EDIV/Rand for
legacy), peer IRK, key size, `FLOSS_SMP_BOND_*` flags and a recency counter.
Up to `FLOSS_MAX_BONDS` LE bonds, separate from the Classic bonds of
`floss_gap_bond_*()`. `floss_smp_local_irk()` returns our IRK.

`floss_hal_random()` must be a real entropy source for LE products (see
[PORTING.md](PORTING.md), section 1).

## 7. Persistent storage keys

| Key | Macro | Contents |
|---|---|---|
| 0x01F0 | `FLOSS_SMP_STORE_KEY_IRK` | Our IRK (generated once) |
| 0x0200 + i | `FLOSS_SMP_STORE_KEY_BOND_BASE` | LE bond i (`0..FLOSS_MAX_BONDS-1`) |
| 0x0280 | `FLOSS_LE_STORE_KEY_STATIC_ADDR` | Generated random static address |
| 0x0300 + i | `FLOSS_GATT_STORE_KEY_CCCD_BASE` | CCCD values of LE bond i |

The full key map, including Classic keys, is in [PORTING.md](PORTING.md#persistent-storage-required-may-be-ram-only).

## 8. Example: `examples/le_gamepad`

`le_gamepad_app_init()` / `le_gamepad_app_task()`, built for the Pico W as
`floss_le_gamepad` (see [ports/pico_w/README.md](../ports/pico_w/README.md)).

- Name "FLOSS LE Gamepad", gamepad appearance, HID UUID advertised.
- Input report ID 1, 13 bytes (16 buttons, hat, X/Y/Z/Rz int16, two 8-bit
  triggers); output report ID 2, 3 bytes (rumble L/R, LED mask), logged.
- Paced with `FLOSS_HOGP_INTERVAL_CONN`; default `max_inflight` (2).
- Synthetic pattern; right stick X (`Z`) carries a millisecond counter so
  `tools/hostprobe` can measure delivery from the host side.
- VID 0x1209 (pid.codes) with placeholder PID 0x0001: request your own.

## 9. Classic or LE?

| | Classic HID | LE HOGP |
|---|---|---|
| Transmit opportunities | Every 1.25 ms poll (800/s ceiling, ~720/s measured) | One per connection interval the host grants (15 ms from Windows: 66.7/s) |
| Best use | Lowest latency | Hosts or power budgets that require LE |

## 10. Debugging

- Log modules `FLOSS_MOD_LE` (LE GAP, ATT, GATT, HOGP) and `FLOSS_MOD_SMP`.
- Snoop captures decode in Wireshark as `btatt`, `btsmp`, `btl2cap`
  (CID 5 signaling) and LE meta events under `bthci_evt`.
- "Reports stop after a device reboot": check the LE log for
  `gatt: restored N CCCD(s)` after encryption; no line means no LE bond for
  that peer.
