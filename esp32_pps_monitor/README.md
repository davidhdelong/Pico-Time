# ESP32-C6 PPS alignment monitor

Independent physical comparison of the L76K and UBX appliances' rising PPS
edges, using two MCPWM capture channels on one free-running hardware timer.
This is a separate ESP32 project; it does not replace either Pico firmware.

## Wiring

| Signal | ESP32-C6 input |
| --- | --- |
| L76K PPS | D0 / GPIO0 |
| UBX PPS | D1 / GPIO1 |
| Both appliance grounds | ESP32 GND |

Tap each receiver's existing PPS signal feeding its Pico input (normally
Pico GP16). Do not join the two PPS outputs together. Inputs must be 3.3 V
logic; ESP32 GPIOs are not 5 V tolerant. Use short, similar-length cables;
a 220 ohm to 1 kohm series resistor at each source is reasonable. For an
open-drain PPS output, retain/provide its pull-up to 3.3 V. Internal input
pull-downs keep disconnected inputs quiet. Power the monitor through USB;
only PPS and ground connections are required between devices.

GPIO0/1 are exposed as D0/D1 on the Seeed XIAO ESP32-C6 and avoid native USB
and boot-strapping pins. They are configurable through
`PPS_L76K_GPIO` and `PPS_UBX_GPIO` in `platformio.ini`.

The board's RF switch is enabled with GPIO3 low and selects its onboard
antenna with GPIO14 low. To use an attached external antenna, configure
`PPS_XIAO_ANTENNA_EXTERNAL=1`. These board-internal pins are reserved and
must not be used as PPS inputs.

## Dashboard

The USB console prints the DHCP address; browse `http://ADDRESS/`. The
DHCP hostname is `pps-monitor`. HTTP `/api/health` returns JSON; CSV history
is available at `/api/history.csv`. A POST to `/api/reset` clears only the
measurement window/pair totals, preserving signal qualification.

- Positive offset means **UBX arrives later than L76K**; negative means earlier.
- Latest offset, mean, median, minimum/maximum, RMS offset, population standard
  deviation, peak-to-peak spread and current-segment linear drift (us/s = ppm).
- Last 300 pairs (approximately five uninterrupted minutes at 1 Hz), rolling chart and CSV.
  Chart lines break across interruptions. History rows contain age, offset, and
  segment ID; CSV columns are `uptime_s,ubx_minus_l76k_us,segment`.
- Each channel's period, high-pulse width, freshness, rising edges, estimated
  missing pulses and rejected glitches/bad periods.
- Queue drops and stream invalidations, free heap, uptime and Wi-Fi status.

Both channels require two valid one-second intervals before pairing. Valid
periods are 995–1005 ms. Early glitches do not replace the preceding genuine
edge. Missing pulses reset qualification. Nearest edges are paired within
±400 ms, so correct UTC-second association is assumed rather than established
from PPS alone. Losing either input expires the current reading after 2.5 s;
historical statistics are labelled with the last pair age, and current drift
is suppressed. Drift requires ten pairs from the current continuous segment.
Outage duration and estimated missing pulses update while an input is absent;
recovery commits the missed count without counting it twice. Glitches make an
unmeasured high width unavailable rather than allowing a later falling edge
to corrupt it. Only the first accepted falling edge measures a pulse width.

Hardware timestamps are latched before the ISR runs. Software observation
time is used for freshness, wrap ambiguity protection and chart time only.
The driver reports the real timer resolution (requested 80 MHz, 12.5 ns/tick).
Resolution is not calibrated absolute accuracy: GPIO synchronization, cable
delay, oscillator error and input-path skew still contribute. This monitor
does not measure NTP delivery or prove either receiver's absolute UTC error.

For a useful calibration check, split **one** PPS output to both inputs using
similar cables and measure the fixed skew. Reversing the two paths helps
separate monitor/cable skew from actual receiver alignment. Never connect two
outputs to one input. No simulated samples are mixed into the live readings.

## Build and flash

Pinned PlatformIO `espressif32@6.13.0`, ESP-IDF 5.5.3, board
`seeed_xiao_esp32c6` matches the attached ESP32-C6FH4 with 4 MB embedded flash.

Supply `PPS_WIFI_SSID` and `PPS_WIFI_PASSWORD` through the host environment,
then run from this folder:

```sh
pio run
pio run --target upload
pio device monitor
```

Wi-Fi credentials are generated into a private header under ignored `.pio/`
build output, never tracked source or API JSON. Keep firmware binaries private
because they contain credentials. Wi-Fi power saving is disabled; reconnect
attempts are nonblocking every ten seconds, and scanning selects the strongest
matching access point. Browsers receive a gzip-compressed dashboard when
supported. HTTP has no authentication and is
intended for your trusted LAN. Capture continues independently of HTTP and Wi-Fi.

## Validation

From the parent project directory:

```sh
cc -std=c11 -Wall -Wextra -Werror -Iesp32_pps_monitor/main \
  esp32_pps_monitor/tests/test_monitor.c esp32_pps_monitor/main/pps_monitor.c \
  -lm -o /tmp/test_pps_monitor && /tmp/test_pps_monitor
node esp32_pps_monitor/tests/test_dashboard.js
```

Host tests cover signed offsets, reversed callback arrival order, repeated
32-bit timer wraps, pulse widths/periods, stale inputs, glitches, missing
pulses, requalification, queue-loss invalidation, rolling-window overwrite,
statistics reset, known 1 ppm drift and out-of-window pairing rejection.
Also run the C tests with ASan/UBSan. Browser tests exercise desktop/mobile
layouts and populated, waiting, qualifying, missing-input and offline states.

The original attached ESP32 flash was backed up before deployment to
`../build/esp32-backup/esp32c6-98a316b9aec4-original.bin` (ignored, 4 MB).
Neither Pico appliance needs to be reflashed for this monitor.

## Initial deployment — October 30, 2026

Flashed the attached ESP32-C6FH4 (MAC `98:a3:16:b9:ae:c4`) and verified the
flasher's write hashes. Dashboard: **http://10.254.0.229/** (DHCP address).
Confirmed live hardware capture resolution **80 MHz / 12.5 ns**, valid health
JSON, empty CSV export, statistics reset, and complete dashboard transfer.
Both inputs are correctly absent, with null current offset, zero samples and
zero capture drops: the PPS wires have not yet been connected. Physical
alignment accuracy and channel skew remain to be tested after wiring.

Initial hardware startup exposed the driver's internal-RAM callback-context
requirement; the deployed callbacks now use DRAM-resident channel identifiers
and start successfully with cache-safe ISR configuration.

Measured Wi-Fi signal is weak, approximately **−92 to −96 dBm**. Repeated
health queries worked, but page delivery was intermittent. Compression reduced
the 12,059-byte page to 4,653 bytes; three of four final compressed-page checks
passed, with one timeout. Improve placement or antenna reception before
relying on continuous dashboard updates. Do not interpret this as a PPS
capture error; the current software marks failed updates stale.

Build report: ESP-IDF 5.5.3; application flash usage **929,074 bytes** and
static RAM **41,240 bytes**, as reported by PlatformIO. The private firmware
binary is `.pio/build/esp32c6/firmware.bin`. SHA-256:

```text
748ee25c447c6068650e79527b5f41a367ed034b4e1cae810d0259366110e93c
```

Strict host C tests, ASan/UBSan and Chromium desktop/mobile fixtures passed.
Generated identity/gzip assets were checked byte-for-byte against the tested
HTML. The parent project's timezone, GPS-validation and UBX tests and both
Pico builds also passed without changing either deployed appliance. Parent
timezone sources were checked current at **2026e/2026e**, covering
**2020–2050**, 301 zones, 73,640 points and 6,723 transitions. Parent flash/BSS
sizes remain L76K 1,151,884/68,888 bytes and UBX 1,121,224/76,976 bytes.
The ESP32 monitor itself does not require timezone data.

Hardware capture API reference: [Espressif MCPWM capture documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32c6/api-reference/peripherals/mcpwm.html).

## XIAO pin/antenna correction — firmware 1.0.1

The attached board was identified as a Seeed Studio XIAO ESP32-C6. Its exposed
header inputs are now **D0/GPIO0 for L76K PPS** and **D1/GPIO1 for UBX PPS**;
the generic board's GPIO6/7 mapping is superseded. The PlatformIO target is
`seeed_xiao_esp32c6`. Firmware also activates the XIAO RF switch (GPIO3 low)
and explicitly selects its onboard antenna (GPIO14 low), following
[Seeed's board documentation](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/).
USB and RF-switch pins are protected against PPS assignment at compile time.

Built and flashed version 1.0.1, with flash-write hashes verified. All eight
live health/full-dashboard check cycles passed at **http://10.254.0.229/**.
The API confirms input GPIOs 0/1 and the 80 MHz capture timer. Both inputs
still report no signal, no paired samples and no capture queue drops, so
physical alignment awaits wiring. Wi-Fi reception improved from around
−94 dBm to **−74 to −75 dBm**, with zero disconnects during verification.

Host statistics tests and browser tests passed, including explicit GPIO0/1
label checks. The parent project's required tests and both builds passed;
timezone sources remain current at 2026e with the counts/range above.
PlatformIO reports **930,736 bytes flash** and **41,272 bytes static RAM**.
Current private firmware SHA-256:

```text
3936bc3de0babc9a71ba85b7ae3b517be478e8798f213efbd39c24dced241939
```

## Outage and segmentation fixes — firmware 1.0.2

See [deployment and validation notes](../docs/esp32-monitor-1.0.2-release.md).
