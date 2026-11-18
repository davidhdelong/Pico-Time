# ESP32-P4 PoE Ethernet NTP server

Initial ESP-IDF port for the ESP32-P4-ETH / ESP32-P4-POE-ETH board family in
the supplied Amazon listing (ASIN B0FN7JQ2V8). Board wiring follows the
[Waveshare schematic](https://files.waveshare.com/wiki/ESP32-P4-ETH/ESP32-P4-ETH-datasheet.pdf).
The internal RMII MAC uses the onboard IP101 PHY, GPIO31 MDC, GPIO52 MDIO,
GPIO51 reset and external RMII reference clock on GPIO50. The remaining RMII
pins match ESP-IDF's ESP32-P4 default pin assignment. PoE power conversion is
handled by the board/module; Ethernet firmware needs no separate PoE setting.

This project is separate from the Pico firmware and ESP32-C6 comparison
monitor. No existing device needs an update to use the P4 server.

## Features and limits

- Separate L76K and UBX / NEO-F10N builds, one receiver per server.
- Rising PPS captured by GPIO ETM directly into a hardware GPTimer at 10 MHz
  (100 ns ticks). A separate, equally clocked GPTimer supplies ordinary clock
  reads, so software cannot overwrite the PPS capture latch. Its start mapping
  is bracketed; that bound contributes to uncertainty.
- Validated UTC date/time from NMEA RMC or UBX NAV-PVT. A frame must associate
  with the preceding PPS within 900 ms. Four good consecutive intervals qualify
  the reference. Robust interval rejection and alpha-beta frequency tracking
  discipline the clock; the existing UTC continuity/slew logic is reused.
- IPv4 Ethernet DHCP, UDP/123 NTP v3/v4 clients, stratum 1 / `GPS ` when locked.
  Malformed requests are rejected and clients are limited to 16 replies/s per
  address in a bounded 32-entry table. The table is an overload guard, not an
  authentication boundary.
- Local HTTP dashboard and `/api/health` show UTC, timezone, lock status,
  uncertainty, GPS/PPS, Ethernet and error counters. Everything is served
  locally with no browser CDN or firmware download client.
- Embedded timezone rules/boundaries remain presentation-only, in flash/XIP.
  No Wi-Fi credentials, internet time source, HTTPS client or TLS transport is
  used. TLS transport is explicitly disabled in the SDK configuration.
  ESP-IDF's baseline crypto initialization may still link platform crypto
  objects; no TLS handshake/client code is required by this application.

This initial port has **no DS3231 interface, RTC calibration, aging-register
control, RTC-backed holdover, or TM1637 display support**. A brief PPS coast is
allowed within the freshness policy; loss of the qualified reference expires
to LI=3 / stratum 16. It does not claim RTC holdover quality.

NTP RX/TX timestamps are software timestamps around socket handling. This
initial server does not use Ethernet PTP/IEEE1588 hardware packet timestamps
and is not a PTP grandmaster. The 100 ns capture tick is resolution, not a UTC
accuracy certification. The engineering uncertainty allowance is retained;
real packet latency and client accuracy require measurements on the board.

## Wiring

Use GPIO numbers printed on the P4 board, not physical header positions or
the C6 monitor's D0/D1 pins. Camera/audio/display/SD peripherals are unused.

| Receiver signal | P4 board connection |
|---|---|
| GPS TX | GPIO5 (server UART RX) |
| GPS RX | GPIO4 (server UART TX) |
| GPS PPS / TIMEPULSE | GPIO6 |
| GPS ground | GND |
| GPS power | Supply appropriate for the receiver breakout |

Signal levels must be 3.3 V. Share ground, keep PPS wiring short, and do not
connect two receiver outputs together. Board USB/PoE supplies power separately.
Only one host should drive the receiver's RX configuration input at a time.

L76K startup sends its baud command at 9600, switches to 115200, and enables
1 Hz GGA/GSA/RMC output using the Pico project's existing PCAS commands. A
receiver already at 115200 can accept the subsequent configuration there.

UBX startup expects **UART1 at 115200** and sends a RAM-only UBX-CFG-VALSET:
NAV-PVT at 1 Hz, UART1 UBX output, and a positive 1 Hz / 100 ms TIMEPULSE on
the UTC grid. It does not overwrite flash configuration or antenna/user delay
calibration. Older receivers that lack VALSET need separate configuration.
The UBX build requires the receiver's matching configuration ACK before
accepting UTC; a NACK or absent ACK leaves it unqualified and is reported as
`receiver_configured: false`. Incompatible settings or absent UTC messages
leave the server unqualified.
The keys follow [SparkFun's receiver library](https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3/blob/main/src/u-blox_config_keys.h).

## Build

Tested with **ESP-IDF 6.0.1**, using its native CMake/Ninja build and official
ESP32-P4 toolchain. PlatformIO's installed platform lacks a P4 board definition;
the C6 monitor's PlatformIO target is not reused or modified.

Activate your ESP-IDF 6.0.1 environment, then run from the repository root:

```sh
. /path/to/esp-idf/export.sh
bash esp32_p4_ntp/tools/build.sh l76k rev1
bash esp32_p4_ntp/tools/build.sh ubx rev1
bash esp32_p4_ntp/tools/build.sh l76k rev3
bash esp32_p4_ntp/tools/build.sh ubx rev3
```

`rev1` covers **silicon 0.x/1.x**; `rev3` covers **silicon 3.x**. These are
silicon versions, not PCB version labels. ESP-IDF 6 requires distinct binaries
for the two families. Check the chip revision before choosing a flash image.
No P4 was attached during development, so that choice has not been verified
for the user's physical board. [Espressif chip revision configuration](https://github.com/espressif/esp-idf/blob/v6.0.1/components/esp_hw_support/port/esp32p4/Kconfig.hw_support).

Outputs live under `build-p4-<receiver>-<silicon>/`. Defaults use DIO, 40 MHz,
a conservative 4 MB flash layout, no PSRAM, and no credential files. The
timezone database remains in flash. Larger physical flash does not need to
be enabled for this layout.
Each directory contains `pico_time_p4.bin` (application at 0x10000) and
`pico_time_p4.factory.bin` (merged bootloader/partition/application image for
offset 0x0). The factory image is generated from the exact IDF flash manifest.

## Flash and verify

Flashing is a separate action. Identify the **P4's** USB-UART device first;
do not use the connected Pico's serial port. The build output prints the
appropriate esptool command. With the correct build directory and port:

```sh
idf.py -C esp32_p4_ntp -B build-p4-l76k-rev1 -p /dev/ttyUSB_P4 flash monitor
```

Use the actual P4 device path in place of the example. Select the receiver
and silicon directory that matches your hardware. Its DHCP hostname is
`pico-time-p4`; find the lease in the router or read the console's dashboard
URL. No mDNS `.local` name is configured.

After acquisition, verify valid health JSON, `LOCKED`, fix/PPS present,
stratum 1 / `GPS ` NTP replies, timezone versions and expected local zone,
complete dashboard loads with no new HTTP errors, and zero queue/capture
faults. Then test PPS and UART loss/recovery and Ethernet disconnect/reconnect.
Unplugging only PPS must eventually produce unsynchronized NTP responses.
Physical PoE/link/timing behavior and end-to-end NTP accuracy remain unverified.

## Tests

```sh
bash esp32_p4_ntp/tools/test.sh
python3 esp32_p4_ntp/tests/test_dashboard.py
```

Tests compile the real clock, GNSS parsing, NTP packet and shared UTC code
with warnings as errors, both normally and with ASan/UBSan/float-cast checks.
They cover acquisition, fractions, NTP fields and originate echo, v3/v4,
2036-era wrap, loss/recovery, capture faults, UTC jumps, bad requests,
checksum/date/coordinate validation, and a checksum-verified RAM-only UBX
configuration frame. Dashboard tests run its actual embedded JS with healthy,
unsynchronized and offline fixtures.

See [build handoff](../docs/esp32-p4-ntp-initial-build.md) for artifact sizes,
source versions and validation results.
