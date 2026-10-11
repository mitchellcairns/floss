# Contributing to FLOSS

Thanks for helping. FLOSS is public domain (Unlicense); by submitting a
contribution you dedicate it to the public domain under the same terms and
confirm you have the right to do so.

## 1. Clean-room rule (mandatory)

- Implement from **public specifications** and observed interoperable
  behaviour only. See [docs/LEGAL.md](docs/LEGAL.md).
- **Never copy, translate or paraphrase code from another host stack**, and do
  not read another stack's source while implementing the same feature in FLOSS.
- Cite the specification section in a comment where behaviour is non-obvious
  (e.g. `/* Core spec Vol 3 Part A 4.2 */`).

## 2. Naming

- Never use the radio standard's trademarked brand name or consortium
  trademarks in code, comments, docs, file names or commit messages. Use
  technical terms (HCI, L2CAP, BR/EDR, LE, "Classic").

| Kind | Rule | Example |
|---|---|---|
| Public functions | `floss_<module>_<verb>[_<object>]` | `floss_l2cap_send_buf` |
| Upward hooks | `floss_<receiver>_on_<what>` in `src/floss_internal.h` | `floss_l2cap_on_pdu` |
| Types | `floss_<name>_t` | `floss_conn_t` |
| Macros / constants | `FLOSS_<MODULE>_<NAME>` | `FLOSS_HCI_ERR_PAGE_TIMEOUT` |
| Static (file-local) | no prefix needed, `static` always | `send_config_req` |
| Files | `floss_<module>[_<part>].c/.h` | `floss_smp_crypto.c` |

## 3. Code style

- **C11**, no compiler extensions in core code (`C_EXTENSIONS OFF`). Must build
  warning-free with `-Wall -Wextra -Wshadow -Wstrict-prototypes` (CI uses
  `-Werror`).
- 4-space indentation, no tabs, LF line endings (`.gitattributes` enforces).
  Opening brace on the same line. Lines up to ~110 columns.
- **No heap** (`malloc`/`free`), no recursion, no VLAs. Size everything with
  `floss_config.h` macros.
- Fixed-width integer types. Wire formats via `floss_get_le16()` /
  `floss_put_le16()` etc. **Never cast a struct onto packet bytes.**
- Fallible functions return `floss_err_t` (0 = OK, negative = error).
- Time is `uint32_t` microseconds; compare with `floss_time_diff()`.
- Every file starts with the standard header:
  ```c
  /*
   * FLOSS - Free, Lightweight, Open Short-range Stack
   * SPDX-License-Identifier: Unlicense
   *
   * floss_xxx.c - One line saying what this file does.
   */
  ```
- Public headers document every function with `/** ... */`: what it does,
  context (stack-only or ISR safe), ownership, return values.
- Respect the layer rules in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md):
  public APIs downward, `floss_internal.h` hooks upward.
- Hot paths: add `FLOSS_TRACE` and `FLOSS_STAT_*` where useful, logs only at
  DEBUG/TRACE level.

## 4. Building and testing

```sh
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
```

CMake options: `FLOSS_BUILD_TESTS` (ON), `FLOSS_BUILD_POSIX` (ON),
`FLOSS_WERROR` (ON), `FLOSS_BUILD_FUZZ` (OFF). Tests run against the simulated controller in
`transport/sim`; new protocol features need a test there (happy path plus at
least one malformed-input case).

Parsers of peer or controller input are also covered by a deterministic fuzz
harness (`tests/fuzz/floss_fuzz.c`, built with AddressSanitizer and UBSan
where the toolchain has them, and run by ctest when enabled):

```sh
cmake -S . -B build-fuzz -G Ninja -DFLOSS_BUILD_FUZZ=ON
cmake --build build-fuzz && ctest --test-dir build-fuzz -L fuzz
build-fuzz/tests/floss_fuzz --seed 42 --iters 100000 rfcomm   # one target, longer
```

A failure prints the seed; the same seed and `--iters` replay it exactly
(`--verbose` shows the stack's logs). Hardware changes: also build `ports/pico_w`
and attach a short `floss_stats_dump()` or snoop excerpt to the PR if timing
is involved.

## 5. Commit messages

```
l2cap: answer information request for fixed channels

Explain why, not just what. Reference spec sections and issue numbers.
Wrap at 72 columns.
```

- Subject: `<module>: <imperative summary>`, <= 72 characters, no trailing
  period. Modules: `core`, `hci`, `gap`, `l2cap`, `sdp`, `hid`, `le`, `smp`,
  `rfcomm`, `h4`, `cyw43`, `sim`, `posix`, `pico_w`, `tools`, `docs`, `build`.
- One logical change per commit; tests in the same commit as the code.

## 6. Adding a transport

1. Create `transport/<name>/floss_<name>.h/.c` implementing
   `floss_transport_ops_t` (contract: [docs/PORTING.md](docs/PORTING.md#2-writing-a-transport)).
2. If it is a byte stream, don't write a transport: implement
   `floss_stream_t` and use `transport/h4`.
3. Keep `headroom <= FLOSS_TRANSPORT_MAX_HEADROOM`; honour synchronous
   ownership in `send()`; return `FLOSS_ERR_BUSY` instead of blocking.
4. Add a CMake target (in `cmake/floss.cmake` if it has no vendor SDK
   dependency, otherwise in the port that needs it).
5. Document any third-party license it pulls in, in `docs/LEGAL.md`.

## 7. Adding a port

1. Create `ports/<platform>/` with one HAL file implementing all of
   `floss_hal.h`, the run loop, and a `CMakeLists.txt`.
2. Go through the porting checklist in
   [docs/PORTING.md](docs/PORTING.md#5-porting-checklist).
3. Add build instructions to the README quick start.
