# PPS hardware epoch — firmware 0.5.1

The first PPS timestamp previously used main-loop processing time. Later
timestamps integrated precise PIO intervals from that delayed initial anchor.
Firmware 0.5.1 instead maps the start of the PIO counter to the local clock
using an interrupt-masked enable bracket and a peripheral-write completion
barrier. It derives the first timestamp from the captured X value, including
initialization clocks and counter underflow. Software arrival time resolves
whole counter wraps only; it does not establish subsecond phase. Later pulses
retain the existing corrected interval timeline and fractional carry.

DMA/IRQ setup now precedes producer startup. Missing capture data or an invalid
start bracket fails closed. PPS loss requests a fresh counter epoch, just as
FIFO stalls, capture backlog, and excessive polling delays do. IRQ and DMA
notifications no longer temporarily overwrite the published timestamp with
their processing time.

This is a hardware-counter measurement with a bounded software epoch mapping,
not a direct latch of the RP2350 system timer. Enable brackets wider than 10 us
or backward clock brackets are rejected. Divider phase, input synchronization,
edge sampling, and local-clock mapping still introduce uncertainty. The
independent GPIO observation brackets and existing 100 us NTP engineering
allowance remain unchanged. No nanosecond UTC accuracy claim is made.

## Validation

- Assembled PIO simulator: first timestamp, delayed consumption, acquisition
  across multiple wraps, fractional carry, odd/even edge timing, FIFO stalls,
  and 24 hours / 86,401 captures with zero accumulated interval arithmetic error.
- Actual firmware timeline harness: 123 ms first-event processing latency
  removed, consecutive intervals, missing/extra edges, fresh acquisition,
  invalid epochs and missing captures.
- Actual enable-bracket harness: interrupt masking, write barrier, midpoint,
  backward and excessive-width rejection.
- Actual recovery harness: DMA and IRQ paths, FIFO stalls, polling gaps,
  backlog and saturated counters.
- All above pass normal and AddressSanitizer/UndefinedBehaviorSanitizer runs
  with `-std=c11 -Wall -Wextra -Werror`.
- Timezone, GPS validation, UBX, NTP uncertainty/service, RTC holdover, and
  I2C/watchdog regressions pass. Both configured firmware builds succeed.
- Timezone call-site audit: lookup/conversion remains in presentation refresh,
  health/display paths; none added to PPS, NTP timestamps, RTC synchronization,
  holdover or the alpha-beta servo.

## Build handoff

Timezone boundary release checked via upstream GitHub latest-release API:
2026e. Embedded boundary/IANA versions and host IANA version are all 2026e;
no regeneration. Years 2020–2050; 301 zones, 73,640 points, 1,381 rings and
6,723 transitions, retained in flash/XIP.

| Variant | Flash text/data bytes | BSS bytes | UF2 |
|---|---:|---:|---|
| L76K | 1,152,724 | 68,896 | `build-l76k-webfix/pico_gps_l76k.uf2` |
| UBX | 1,121,912 | 76,984 | `build-ubx-webfix/pico_gps_l76k.uf2` |

SHA-256:

```text
L76K eabe2cec2e4f689a7535f233547d2684e8f7aa9f480dcf506b1a1ca30d80d172
UBX  364bcb6c2799b44d404d851b280734df0eac6953addb033801504b8214ce64c1
```

The ESP32 measures GNSS PPS edges directly and will not by itself demonstrate
improvement in the Pico's software/NTP clock phase.

## UBX deployment — October 30, 2026

Flashed only the UBX Pico after confirming its mounted RP2350 BOOTSEL volume
and USB serial `50BABB25DD2BEAFE`. UF2 hash matched the handoff above. Network
health at `10.254.0.11` confirms firmware 0.5.1, build Oct 30 2026 13:38:52.

At uptimes 64–84 seconds, GPS/PPS remained LOCKED with 17–18 satellites.
After the 60-sample startup window filled, reported clock uncertainty was
126.905–136.755 us and internal PPS jitter RMS settled from 34 to 16 ns.
Timestamp placement relative to the independent GPIO bracket midpoint was
+1.970 to +10.064 us; observation windows were about 39.7–39.9 us wide.
These midpoint offsets are diagnostics, not calibrated absolute UTC errors.

Three complete dashboard requests succeeded, with zero HTTP errors before
and after. UDP/123 returned a valid UTC NTP reply, stratum 1, reference ID
`GPS ` and matching originate timestamp. America/Chicago and both 2026e
timezone versions were correct. Capture faults, DMA overruns and Wi-Fi
reconnects stayed zero. RTC remained TRACKING with 32 kHz enabled. Debug
logging was not enabled and serial diagnostics were not used.

At this stage L76K had not been flashed. Longer-term phase stability, deliberate PPS-loss
recovery and RTC holdover under the new firmware remain to be checked in
controlled tests; no signal disconnection was performed for this deployment.

## L76K deployment — October 30, 2026

Flashed the L76K image after confirming the RP2350 BOOTSEL volume belonged to
USB serial `44E15817A454A436` and the UF2 SHA-256 matched the handoff above.
The UBX device was not flashed again.

Health at `10.254.0.10` confirms firmware 0.5.1, build Oct 30 2026 13:38:52.
GPS/PPS was LOCKED throughout acquisition sampling. At uptime 67 seconds,
the 60-sample timing window was full, with nine satellites, 32 ns internal PPS
jitter RMS and 153.151 us reported clock uncertainty. Timestamp placement was
+4.104 us relative to a 78.627 us GPIO observation bracket midpoint; this is
not an independently measured UTC error.

Three complete dashboard loads succeeded (13,157 bytes each); HTTP errors
remained zero. UDP/123 replied with stratum 1, `GPS ` reference ID, matching
originate timestamp and expected UTC epoch. Timezone name America/Chicago and
both 2026e source versions were correct. Capture faults, DMA overruns and Wi-Fi
reconnects stayed zero. RTC was TRACKING with 32 kHz enabled. No debug logging
or serial diagnostics were enabled.
