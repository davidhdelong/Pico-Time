# ESP32-P4 Ethernet NTP — initial build handoff

Built November 18, 2026, application version 0.1.0, using ESP-IDF 6.0.1 and
the official RISC-V toolchain. Target: ESP32-P4-ETH / PoE ETH board family
identified from the supplied Amazon listing, with the manufacturer RMII
schematic used for the PHY and GPIO assignments. Source and wiring are in
[`esp32_p4_ntp/README.md`](../esp32_p4_ntp/README.md).

## Result

Four application and merged factory images built successfully with project
sources compiled using `-Wall -Wextra -Werror`:

| Receiver | Silicon | Application flash-image bytes | Static BSS bytes | Build directory |
|---|---|---:|---:|---|
| L76K | 0.x / 1.x | 1,189,440 | 11,104 | `build-p4-l76k-rev1/` |
| L76K | 3.x | 1,190,576 | 11,104 | `build-p4-l76k-rev3/` |
| UBX | 0.x / 1.x | 1,186,032 | 12,144 | `build-p4-ubx-rev1/` |
| UBX | 3.x | 1,187,152 | 12,144 | `build-p4-ubx-rev3/` |

Static BSS is the sum of ELF BSS sections, not total RAM consumption: SDK
code/data, dynamic driver buffers, task stacks and heap use additional RAM.
Every directory contains `pico_time_p4.bin` for application offset 0x10000
and `pico_time_p4.factory.bin` for merged-image offset 0x0. Factory image sizes
are respectively 1,254,976; 1,256,112; 1,251,568; and 1,252,688 bytes.

Configuration/packaging checks confirm:

- Correct chip ranges: revision code 0–199 for `rev1`, 300–399 for `rev3`.
- DIO / 40 MHz / 4 MB flash layout, with PSRAM unused.
- TLS transport explicitly disabled and no HTTP client/TLS handshake symbols
  linked. SDK baseline crypto initialization remains a platform dependency.
- Factory images contain byte-for-byte copies of their application at 0x10000
  and partition table at 0x8000.
- Build outputs and any local SDK configuration remain excluded from Git.
  New sources contain no existing private Wi-Fi credentials.

Factory SHA-256 values:

```text
L76K rev1 85f00c79994a3d86d6192fbd32a05dcadf9b6e6a0069ba3969a25b409380292c
L76K rev3 4e4917123354220ead0bc958dd1624c81eacd91338a52a8536b36a6a0e5ec256
UBX  rev1 d0366e992557f58575edfcde7d604706402afb4c98ad8fb4f09d1182e9171ab0
UBX  rev3 036c4b99c02133b54736978e52da4e99e02bb01ffe7b1bc78c82de17966a6b90
```

## Validation

`bash esp32_p4_ntp/tools/test.sh` passes strict host builds both normally and
with ASan/UBSan/float-cast-overflow checks. It exercises real shared and new
implementation code for lock acquisition, UTC fractions, NTP reference/originate/
receive/transmit fields, v3/v4 clients, 2036-era timestamp wrap, source loss,
recovery, capture faults, UTC jumps, invalid requests, NMEA checksums/calendar/
coordinates, UBX NAV-PVT validity, and generated UBX configuration/checksums.
Configuration ACK/NACK matching is tested, and the UBX firmware gates UTC on
its receiver's positive ACK.

`python3 esp32_p4_ntp/tests/test_dashboard.py` passes actual embedded-JS
fixtures for healthy, unsynchronized and offline states, timezone display and
single-loop polling. Shell scripts pass syntax checks, Python scripts compile,
and `git diff --check` passes.

No P4 board was attached. Nothing was flashed. Physical PoE, PHY link, DHCP,
GNSS UART/PPS wiring, capture calibration, HTTP/NTP service, failure recovery
and client accuracy remain to be verified on hardware. The connected UBX Pico
and all other deployed firmware were left untouched.

## Timezone and parent firmware policy

Checked the upstream timezone-boundary-builder latest release: **2026f**.
Embedded boundary is 2026f; embedded and local host IANA are **2026e**. Sources
are unchanged, so no regeneration was performed. Both Pico variants and the
P4 keep the existing **2020–2050** range, **301 zones**, **73,617 points**,
**1,374 rings**, and **6,723 transitions**. Geographic/rule tables remain
const in flash/XIP. In the P4 port, lookup occurs only in HTTP presentation,
outside the timing mutex; clock discipline, PPS and NTP timestamps stay UTC.

The required timezone/GPS-validation/UBX host tests passed with
`-std=c11 -Wall -Wextra -Werror`. Both required Pico builds also pass:

| Pico variant | Flash text/data bytes | BSS bytes |
|---|---:|---:|
| L76K 0.5.2 | 1,152,844 | 68,912 |
| UBX 0.5.2 | 1,121,768 | 77,000 |

No timezone download/check, HTTP mirror, HTTPS client or TLS transport was
added to firmware. Source checks/downloads remain host-build activities.

## Scope

This is a GNSS/PPS Ethernet NTP server with DHCP and a local dashboard, not a
complete peripheral port of the Pico appliance. DS3231 RTC/32 kHz holdover,
aging calibration, TM1637 output and PTP/hardware Ethernet packet timestamps
are not implemented. GNSS/PPS expiry produces unsynchronized NTP service.
