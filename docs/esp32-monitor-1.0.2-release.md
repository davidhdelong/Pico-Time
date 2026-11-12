# ESP32 monitor 1.0.2 — November 4, 2026

Implemented and flashed the four requested review items on the Seeed XIAO
ESP32-C6, USB identity 98:a3:16:b9:ae:c4, GPIO0 L76K and GPIO1 UBX.

- Only the first falling edge after an accepted rise measures width. A rejected
  rise disarms an unfinished width measurement, retaining an already measured
  width and avoiding false widths from the glitch's falling edge.
- Stale comparisons suppress current drift. Retained statistics are explicitly
  historical and show last-pair age. Stale channel width/period are unavailable.
- A periodic capture-task poll detects outages without requiring a returning
  edge. API/dashboard report outage duration and estimated missing pulses;
  recovery commits the same estimate without double counting.
- Continuous measurement segments identify lost inputs, bad intervals, stream
  invalidations, statistics resets and gaps between accepted pairs. Drift uses
  at least ten samples from the current segment only. Chart lines break at
  segment boundaries and gaps. The chart title correctly describes 300 pairs,
  which can span more than five minutes after interruptions. API history rows
  and CSV now include a segment column.

Strict C host tests and ASan/UBSan passed for signed offsets, timer wraps,
normal width, extra falling edges, glitches before/after a genuine fall,
staleness, live outage counts, recovery without double counting, segment
boundaries, isolation of drift from a step across an outage, rolling history,
qualification and reset. Chromium tests passed desktop/mobile layouts,
historical labels, stale drift suppression, live outage display, line-break
selection, missing/qualifying inputs and network failure/recovery.

Physical fault injection was not performed; glitch/outage behavior was tested
with deterministic host inputs. Live checks after flashing confirmed firmware
1.0.2, both inputs qualified, 21–30 paired samples, no missed pulses, glitches,
queue drops or Wi-Fi disconnects. Offset ranged from -0.200 to -0.175 us and
rolling standard deviation from 0.0080 to 0.0091 us in this short check.
The complete compressed dashboard matched the source exactly (12,842 bytes
decompressed; 4,887 transferred), and CSV included the expected segment IDs.

ESP32 build: PlatformIO flash usage 933,066 bytes, static RAM 43,688 bytes.
Flash writes passed esptool hash verification. Firmware binary SHA-256:
`e8ee7153a5c9d3892c7f5c436ccb2d9ce93099f4b2510122ccbd736baa9a8b9a`.
Dashboard: http://10.254.0.229/ .

## Required parent project build checks

The mandatory upstream timezone check found boundary release 2026f, newer
than embedded 2026e. Downloaded the matching 1970 geographic archive to the
host workspace, updated generator pins, and ran the exact regeneration command
`python3 tools/timezone_generator/generate.py`. Host/embedded IANA remains
2026e; embedded boundary is now 2026f. Range remains 2020–2050, with 301 zones,
73,617 points, 1,374 rings and 6,723 transitions in flash/XIP.

Timezone, GPS validation and UBX host tests passed with warnings as errors;
both Pico variants rebuilt successfully. Timezone lookup call sites remain
presentation-only, outside PPS/NTP timestamping, RTC/holdover and discipline.
No timezone runtime download or TLS functionality was added.

| Pico variant | Flash text/data bytes | BSS bytes |
|---|---:|---:|
| L76K | 1,152,372 | 68,896 |
| UBX | 1,121,560 | 76,984 |

Neither Pico was flashed. The newly rebuilt Pico images contain the boundary
update and are distinct from the previously deployed 0.5.1 binaries. The
ESP32 has no embedded timezone database.
