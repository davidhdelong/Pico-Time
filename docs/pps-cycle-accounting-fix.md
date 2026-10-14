# PPS cycle accounting fix — 2026-09-09

## Cause and correction

The PIO down-counter decrements once per two state-machine clocks in its
steady loops, but consecutive rising-edge snapshots also span six clocks
of capture/falling-edge overhead. An X underflow adds another clock.
Converting only the down-counter difference to elapsed time therefore
accumulated a negative timestamp offset. At the deployed 50 MHz counter
rate, the ordinary omitted overhead was 60 ns per second.

`pps_capture_timing.h` converts the complete instruction-clock count and
carries fractional nanoseconds between samples. Initialization now requires
an exactly representable PIO divider, preventing an unsupported requested
frequency from silently introducing a second scale error.

The correction assumes consecutive captures without counter stalls. A
sticky RX FIFO stall, queued-capture backlog, pending capture fault, or
main-loop polling gap over 1.5 seconds restarts the capture producer and
consumer. It invalidates PPS/PLL trust, resets GNSS recovery and RTC frequency
learning, and lets the ordinary acquisition path requalify. Intervals outside
995–1005 ms are rejected rather than accumulated. The raw capture timeline
continues advancing without GNSS calendar messages. RTC trim and persisted
calibration are not changed by this patch.

The existing conservative uncertainty calculation and placement diagnostics
remain intact. `/api/health` additionally exposes `pps_capture_faults`, a
saturating count of capture-stream resets since boot.

## Validation

- `sh tools/test_pps_capture.sh`: actual assembled PIO instruction simulation,
  pulse alignment/width variation, X wrap and blocking FIFO behavior; ordinary
  and ASan/UBSan builds pass. Over 86,401 simulated captures and 1,005 wraps
  (24 simulated hours), old accounting error is −5,194,050 ns and corrected
  accumulated accounting error is zero. This is not a hardware endurance test
  or a claim of zero physical clock uncertainty.
- `python3 tests/test_pps_recovery.py`: extracts the actual firmware recovery
  function; DMA and interrupt configurations pass normal and sanitizer tests
  for FIFO stalls, pending faults, polling gaps, DMA backlog, state invalidation,
  hardware restart ordering and saturation.
- `python3 tests/test_pps_timeline.py`: actual interval reconstruction block
  passes corrected-interval, missing/extra-edge rejection and fresh-anchor
  tests, including sanitizers.
- Required C11 `-Wall -Wextra -Werror` timezone, GPS validation and UBX tests pass.
  Timezone coverage includes geographic selection, DST transitions, non-hour
  offsets, ocean fallback and movement/border cases.
- NTP uncertainty, RTC holdover, RTC reporting, HTTP lifecycle and dashboard
  binding tests pass. Health response bounds are 5,795/6,400 bytes (L76K) and
  6,194/6,400 bytes (UBX).
- Real headless Chrome tests pass desktop/mobile rendering and
  holdover/unqualified/offline/recovery states for both variants; no external
  dashboard requests.
- Both required CMake builds pass. Timezone call-site audit confirms conversion
  is limited to presentation/zone refresh, not PPS, NTP timestamps, RTC sync,
  holdover, leap processing or alpha-beta discipline. Generated data remains
  const flash/XIP data; no runtime timezone download was added.

## Build handoff

Embedded IANA and boundary versions: **2026c / 2026c**. Checked against the
host IANA version and the latest timezone-boundary-builder release, both
2026c. No regeneration was needed. Geographic source remains the 1970 asset;
rules cover **2020–2050**, with **301 zones, 73,462 points, 1,307 rings and
6,819 transitions**.

| Variant | ELF flash text/data bytes | BSS bytes | UF2 |
| --- | ---: | ---: | --- |
| L76K | 1,157,004 | 77,152 | `build-l76k-webfix/pico_gps_l76k.uf2` |
| UBX | 1,156,724 | 85,292 | `build-ubx-webfix/pico_gps_l76k.uf2` |

Both use PPS DMA, a 50 MHz capture counter, `RTC_32KHZ_GPIO=22` and
`DEBUG_LOG=0`.

SHA-256:

```text
L76K 1c8e0885ff69a9253f6163c77d96e2fdbdf63faad1acec08b9467df2ed59d44a
UBX  9d1ff280d6e4ff8d420a4a15b4bfe348dbe04139138305c351989a087ccc87b5
```

These images have **not been flashed** as part of this fix/build/test task.
After an authorized BOOTSEL flash, verify health JSON, GPS/PPS and LOCKED
state, timezone version/zone, UTC NTP replies and stable HTTP error counts.
Then monitor placement offset, observation window, uncertainty and
`pps_capture_faults` over 24 hours. Expect the deterministic ~60 ns/s drift
to disappear; physical timing noise and honest uncertainty remain.
