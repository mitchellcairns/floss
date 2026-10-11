# Legal notes

*This document is not legal advice. It describes the project's policies and
the licenses of the code it touches. If you ship a product, consult your own
counsel.*

## 1. Licensing

### FLOSS itself

All code, documentation and tools in this repository are released into the
public domain under the [Unlicense](../LICENSE). Every source file carries
`SPDX-License-Identifier: Unlicense`. Contributions are accepted on the same
terms (see [CONTRIBUTING.md](../CONTRIBUTING.md)).

### Third-party dependencies of ports

FLOSS's core (`include/`, `src/`, `transport/h4`, `transport/sim`,
`ports/posix`) has no third-party dependencies beyond a C11 toolchain and its
C library. Some ports link vendor SDKs that the user obtains separately:

| Component | Used by | License | Distributed by FLOSS? |
|---|---|---|---|
| Raspberry Pi Pico SDK | `ports/pico_w`, `transport/cyw43` | BSD-3-Clause | No (fetched/installed by the user) |
| cyw43-driver (in the Pico SDK) | `transport/cyw43` | Raspberry Pi's license for the driver, which permits use with Raspberry Pi RP-series microcontrollers; other uses need a separate license from the copyright holder. Read the driver's own license file. | No |
| CYW43439 WLAN and radio firmware images | loaded by cyw43-driver at start-up | Supplied by the chip vendor through the cyw43-driver, under the terms stated there | No |

FLOSS does not copy, modify or redistribute any of these. A firmware binary
you build for the Pico W links them in, so that binary is subject to their
terms as well as the Unlicense.

## 2. Naming policy

- The project, its code and its documents do not use the trademarked brand
  names or logos of the radio standard or its consortium.
- Technical terms (HCI, L2CAP, SDP, RFCOMM, ATT, GATT, SMP, BR/EDR, LE, HID)
  are used nominatively, to describe interoperability. Where compatibility
  must be stated, we say FLOSS "interoperates with controllers and hosts
  implementing the Core Specification HCI".
- FLOSS does not claim certification, qualification or endorsement by any
  standards body.
- Integrators who want to use trademarks or logos on their products must
  obtain the rights to do so themselves.

## 3. Clean-room policy

- FLOSS is implemented from **publicly available specifications** (the Core
  Specification and published profile/service specifications, the USB HID
  specification and usage tables, IETF/NIST cryptographic standards) and
  from observing interoperable behaviour (e.g. HCI captures of our own
  devices).
- **No code is taken from other host stacks**, whatever their license. That
  includes copying, translating or closely paraphrasing source.
- Contributors should not consult the source of another host stack while
  implementing the equivalent FLOSS feature.
- Third-party code that is not a host stack (e.g. a public-domain AES
  implementation) may be included only with a compatible license recorded
  next to it, and only after discussion.

## 4. What FLOSS is, and what it is not

FLOSS is a **host-side protocol implementation**. It runs on an application
processor and talks over HCI to a radio controller. The controller (for
example the CYW43439 module on a Pico W) is a separate product that its
manufacturer may have qualified and certified.

Integrators shipping products that include FLOSS are responsible for their
own:

- **regulatory** compliance (e.g. FCC and CE radio-equipment requirements),
  including the use of pre-certified modules and their conditions;
- **trademark** assessment, including any qualification or listing programmes
  required to use particular names or logos;
- **patent** assessment for their product and markets;
- assessment of third-party SDK and firmware licenses (section 1).

FLOSS is provided "as is", without warranty of any kind, as stated in the
Unlicense.
