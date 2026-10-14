# UBX 32 kHz deployment — 2026-09-08 UTC

Enabled `RTC_32KHZ_GPIO=22` in the existing `build-ubx-webfix` CMake cache,
preserving UBX protocol, 38400 baud and `DEBUG_LOG=0`. Wiring is DS3231 32kHz
to Pico GP22 (physical pin 29), with a 3.3 V pull-up; SQW remains on GP17.
The generic source default remains disabled; README records the deployment
override and command needed to recreate it.

## Build validation

- GitHub's latest timezone-boundary-builder release and embedded boundary
  version: 2026c. Local IANA and embedded TZDB: 2026c. No regeneration needed.
- Geographic source: `timezones-1970.geojson.zip`; years 2020–2050; 301 zones,
  73,462 points, 1,307 rings, 6,819 transitions.
- Timezone, GPS validation, UBX and RTC holdover host tests passed with
  `-std=c11 -Wall -Wextra -Werror`. RTC tests cover disabled/enabled builds
  and AddressSanitizer/UndefinedBehaviorSanitizer.
- Both required firmware builds passed. UBX text/flash 1,143,676 bytes,
  data 0, BSS 91,628 bytes. L76K text/flash 1,143,052 bytes, data 0,
  BSS 83,472 bytes. L76K feature settings were not changed.
- Timezone call sites remain presentation-only; generated arrays are read-only
  flash/XIP symbols, not a wholesale RAM copy.

## Flash and startup

The sole connected RP2350 was visibly in BOOTSEL (USB 2e8a:000f), mounted at
`/run/media/david/RP2350`, serial `50BABB25DD2BEAFE`. The L76K at 10.254.0.10
remained online while UBX at 10.254.0.11 was offline. Copied only
`build-ubx-webfix/pico_gps_l76k.uf2`; the device rebooted into normal USB mode
and returned at 10.254.0.11 with build `Sep 7 2026 20:13:28`.

UF2 SHA-256:
`5b1baf36853bedaf7f6013540304f10b5174b75291f90d827abeb68eef638cf4`

- At uptime 32 s: GPS fix, PPS, `LOCKED`, 32 kHz enabled.
- By uptime 112 s: RTC SQW qualified.
- At uptime 273 s: 32 kHz valid, four accepted blocks, zero rejections;
  frequency estimate -0.773 ppm (diagnostic estimate, not calibrated accuracy).
- NTP on UDP/123 replied in server mode, stratum 1, refid `GPS `, LI=0,
  matching request origin timestamps and UTC transmit timestamps.
- Dashboard returned HTTP 200 with complete 7,951-byte bodies; health JSON
  reported America/Chicago and timezone/boundary versions 2026c.
- HTTP validation had a caveat: around uptime 233–253 s, one dashboard
  connection reset and one health response was truncated. `http_errors` rose
  from 0 to 1; subsequent requests recovered. This is not an error-free
  startup validation and the cause has not been established.
- Follow-up at uptime 297–358 s: seven consecutive complete health/dashboard
  pairs succeeded. `http_errors` stayed at 1, Wi-Fi reconnects stayed at 0,
  GPS/PPS remained locked, and 32 kHz reached five accepted blocks with zero
  rejections. No reboot was observed.

No serial diagnostics, debug logging, GNSS disconnection, or L76K flash was
performed. Qualification was verified; actual GNSS-loss holdover was not tested.
