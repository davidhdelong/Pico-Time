# Review fixes — Pico 0.5.2 / ESP32 monitor 1.0.3

Implemented the five reviewed issues on November 12, 2026. All three images
are built; none has been flashed or pushed to GitHub for this update.

## Changes

1. Pico high-resolution clock sampling, reanchoring and publication now stay
   inside one interrupt-masked operation. A per-core last-sample floor prevents
   microsecond anchor quantization from reversing sequential clock readings.
   An already-disabled interrupt mask is preserved. Hardware PPS intervals and
   the conservative uncertainty allowance remain unchanged.
2. GGA HDOP/altitude and GSA dilution values use a shared finite-number parser.
   Missing or malformed values, non-finite values, conversion range errors,
   trailing junk and negative dilution values use the existing finite -1
   unavailable sentinel. Valid negative altitudes remain supported. Such
   navigation fields can no longer inject NaN/infinity into health JSON.
3. Coordinates are bounded in floating point before conversion to integer
   degrees, avoiding undefined conversion for oversized finite inputs.
4. Repeated UBX B5 sync candidates replace the saved frame-start timestamp.
   Noise before a valid B5 62 prefix no longer supplies the frame's epoch.
5. ESP32 manual/periodic dashboard refresh cancels the pending polling timer
   before checking busy/hidden state. Reset, visibility changes and recovery
   retain a single scheduled polling loop.

The clock critical section now includes conversion arithmetic and the timer
sample. This can add a short interrupt delay; PIO still captures PPS edges in
hardware. Device verification is required before claiming measured accuracy
or latency improvements. Per-core clock monotonicity does not establish a
calibrated nanosecond phase relationship between the two cores.

## Validation

- `tests/test_hires_clock.py`: actual firmware function with checked interrupt
  mocks, quantization monotonicity, ISR progression, nested interrupt masks,
  both core states and DWT wrap. Normal and ASan/UBSan passed.
- `tests/test_gps_numeric.py`: actual GGA/GSA/coordinate functions with
  checksum-valid malformed sentences, finite JSON, negative altitude and
  coordinate bounds. Normal and ASan/UBSan/float-cast-overflow passed.
- Extended strict GPS validation and UBX tests pass, including repeated sync
  prefixes with distinct timestamps.
- `esp32_pps_monitor/tests/test_polling.js`: repeated manual refreshes,
  hidden/busy states, HTTP failure and recovery retain one timer.
- ESP32 Chromium dashboard tests pass desktop/mobile layouts, offset signs,
  history/gaps, missing/qualifying inputs, outage display, reset and recovery.
- Existing strict monitor statistics tests pass.
- Pico PPS startup/timeline/recovery, 24-hour assembled PIO simulation, NTP
  service/uncertainty, RTC holdover/reporting, HTTP lifecycle, I2C/watchdog and
  dashboard-binding regressions pass, with sanitizers where provided.
- Required timezone, GPS validation and UBX host checks pass using
  `-std=c11 -Wall -Wextra -Werror`. Both required Pico CMake builds pass.
- `git diff --check` passes. No credentials or build artifacts added to Git.

## Build handoff

Upstream boundary latest-release API checked: 2026f. Embedded boundary is
2026f; host and embedded IANA are 2026e. No regeneration was needed. Supported
years remain 2020–2050, with 301 zones, 73,617 points, 1,374 rings and 6,723
transitions in flash/XIP. Timezone call sites remain presentation-only; none
was added to PPS, NTP timestamps, RTC, holdover or discipline.

| Image | Flash bytes | BSS/static RAM bytes | Output |
|---|---:|---:|---|
| L76K 0.5.2 | 1,152,844 | 68,912 | `build-l76k-webfix/pico_gps_l76k.uf2` |
| UBX 0.5.2 | 1,121,768 | 77,000 | `build-ubx-webfix/pico_gps_l76k.uf2` |
| ESP32 1.0.3 | 933,106 | 43,688 | `esp32_pps_monitor/.pio/build/esp32c6/firmware.bin` |

Pico figures are ELF text/data and BSS; ESP32 figures are PlatformIO flash and
static RAM usage. Images contain private deployment credentials and remain
ignored build artifacts.

SHA-256:

```text
L76K  f636fbdf7a22c31f71530f236021b81812a777eeec7c9776556e5d9b0a984f71
UBX   8a2327d0a798e16efc3861b93dfae8115cd4040e47c1aa14105675ca9ef3555b
ESP32 d826defb0c49da4206050df7b07bf3dc48c20ad6e4c1551d48576ba5776daad3
```
