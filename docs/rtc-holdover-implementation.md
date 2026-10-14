# RTC holdover implementation record

## Baseline trace (before changes)

The source is authoritative; this workspace has no Git metadata. The baseline
is the single `main.c` timing implementation and `pps_capture.pio`.

| Concern | Original implementation |
|---|---|
| GNSS capture | `pps_pio_init`, `pps_pio_irq_handler` / `pps_dma_poll`; PIO0 SM0 latches counter on rising PPS; main loop reconstructs `last_pps_time_ns` |
| RTC capture | `rtc_sqw_init`, `rtc_sqw_irq_handler`; GP17 by default, both edges, `hires_time_ns` timestamp, deferred processing |
| Polarity | Main SQW block discovers either edge within `RTC_SQW_ALIGN_MAX_US` (100 ms), remembers `rtc_sqw_second_edge_high` |
| Epoch | `rtc_sqw_epoch_seconds = pps_epoch_seconds + (phase_us < 0)`; advances once per selected edge without GPS |
| PPS freshness | `select_time_at_locked`: 1.5 s; main loss handler clears `pps_time_valid`, `pll_valid`, history; NMEA freshness independently 10 s |
| SQW freshness | 2.5 s in selector and main loss handler; signal loss erases phase mapping |
| Calendar | `rtc_read_time`, `rtc_cache_poll`, `rtc_cache_get_locked`; 1 s polling, 2 s expiry; `g_rtc_cache` |
| Selection | `selected_time_t`, `sntp_quality_t`, `select_time_at_locked`: PPS, qualified SQW, coarse RTC, coarse GPS |
| Fractions | elapsed Pico counter ns multiplied by 1e9 / filtered period, converted to NTP fraction |
| GNSS servo | `ab_servo_update`: `ab_period_ns`, `ab_period_rate_ns_per_sec`, median/MAD history; phase here is period innovation, not a unified UTC phase state |
| RTC period | `rtc_sqw_interval_ns`, `rtc_sqw_pll_valid`: 1/64 exponential update, 995–1005 ms acceptance |
| Aging | `discipline_rtc_from_gps`, `rtc_phase_estimator`, `rtc_drift_estimator`; unbounded regression, six hours/six samples, gated on calendar difference; incorrect 0.1 s/day per LSB |
| RTC writes | `rtc_schedule_set_locked`, `rtc_apply_pending_set_if_due`, `rtc_set_epoch_utc`: main-loop I2C at predicted PPS minus learned bias, not deterministic hardware I2C |
| OSF | `rtc_read_time` rejects status bit 7, but startup `rtc_configure_square_wave_1hz` clears it before initial read |
| NTP | `get_sntp_time_at`, `sntp_recv`, quality helpers: PPS stratum 1/GPS, SQW 2/RTC, coarse 16/LOCL with LI=3; `get_leap_indicator`/`apply_leap_smear` preserve UTC leap policy |
| Uncertainty | `get_rtc_holdover_dispersion_us` stores ns in `age_us`, multiplies incorrect age, caps at 2 s; no holdover quality expiration |
| Status | `health_snapshot_t`, `get_health_snapshot`, `/api/health`, UDP health, dashboard; legacy clock and RTC states differ from active quality |

The original RTC `LOCKED` means six calendar checks, not precision phase lock.
The original alpha-beta filter learns period and period rate, not absolute UTC
phase. Keeping this servo and adding a small output continuity model avoids an
unrelated rewrite of PPS capture, leap processing, and GNSS parsing.

## Hardware reference

DS3231 Rev. 10 datasheet:
https://www.analog.com/media/en/technical-documentation/data-sheets/DS3231.pdf

Nominal aging sensitivity is 0.1 ppm/LSB at 25 C, temperature dependent.
Positive aging adds capacitance and slows the oscillator. A forced temperature
conversion applies the new correction; BSY/CONV must be polled without waiting
in an interrupt. OSF is sticky and must only be acknowledged after setting a
valid calendar. Both SQW and 32 kHz are open-drain outputs.

## Bugs fixed

- Replaced the ns-as-us dispersion calculation with a nanosecond error model.
- Replaced the 0.1 seconds/day aging factor with nominal 0.1 ppm/LSB.
- Preserved OSF before configuration, including a boot latch that requires a
  GNSS calendar set after OSF or a failed initial status read. Output configuration
  writes ones to write-zero-to-clear flag bits, preserving flags asserted between
  read and write. Only a successful GNSS calendar write acknowledges OSF.
- Calendar offset no longer gates frequency calibration; constant SQW phase
  offsets no longer cause repeated calendar resets.
- Qualified holdover now expires on uncertainty/duration, not only signal loss.
- Source transitions share a continuous output clock and cannot step backward
  during ordinary holdover/recovery. Historical NTP reference timestamps no
  longer re-enter the live selector.
- Added adequate stacks: core0 4 KiB, network core 8 KiB in main RAM. Compiler
  stack analysis found the health formatter alone used about 2.4 KiB, exceeding
  the old 2 KiB network-core stack. The HTTP JSON buffer is static on core1;
  serialized callbacks copy its contents into their response allocation.
- Required timezone tests exposed an existing validity bug for fixed-offset
  zones: expiry used the final transition plus a year, rejecting Kathmandu and
  Kolkata after 2020. It now uses the generated 2020–2050 range. No database
  regeneration or timing-path timezone conversion was introduced.

## RTC phase and calendar

`rtc_phase_observe` uses a nine-element rolling median and MAD rejection with a
configurable 2 ms minimum rejection threshold. Qualification requires 32 clean
accepted samples, at most 1 ms peak residual jitter, and 60 stable PPS ticks.
An outlier resets qualification credit; three consecutive bad selected edges
invalidate the mapping. Period discontinuities and stale SQW invalidate it too.
Discovery and maximum absolute phase offset default to 450 ms. The large
absolute limit permits software compensation of a stable offset; it is not an
accuracy specification. Discovery fixes one polarity, rather than switching
between edges as their relative distance changes.

Read-only samples of the existing firmware on the two devices, taken before
flashing anything, showed about -16.3 ms and +18.3 ms phase offsets. Twelve
samples spanned 14 us and 11 us respectively, supporting the 1 ms default jitter
threshold with substantial margin. These are baseline observations, not proof
of the new firmware's hardware performance.

After a seconds-register write, falling is the expected integer-second edge:
the datasheet specifies rising 500 ms after the seconds ACK. Software validates
that falling-edge assumption and retains automatic discovery as a fallback.
Main-loop I2C writes are scheduled at a predicted PPS; writes more than 20 ms
late or lacking a fresh qualified PPS are canceled and retried. There is no
new PIO/I2C driver or claim of deterministic ACK timing. Phase compensation
accounts for the measured physical edge offset.

The calendar is checked at the existing 10-minute/hour intervals. Offsets under
five seconds are tracked; asynchronous one-second discrepancies are not treated
as frequency measurements. Invalid calendars and gross offsets justify a write.

## Frequency calibration and temperature

Accepted phase observations feed a bounded 192-sample regression, decimated to
one sample per 120 seconds (maximum span 6 h 22 min). Positive phase slope means
late edges and therefore **negative oscillator frequency error**. Frequency is
reported with positive meaning fast, in ppm. Regression uncertainty is three
standard errors plus a 0.01 ppm model floor. The hardware margin used during
holdover is additional, not replaced by this statistical floor.

Aging requires six hours, at least six samples, qualified/current PPS and phase,
uncertainty at most 0.05 ppm, frequency error exceeding three times uncertainty,
and six hours since the previous adjustment. The nominal positive sensitivity
magnitude is configurable as `RTC_AGING_PPM_PER_LSB`; a fast oscillator gets a
positive register change. Changes are limited to four LSB and -128..127 bounds.
`RTC_AGING_INVERT` remains an explicit compatibility override.

An adjustment invalidates phase/frequency qualification. A main-loop state
machine waits for BSY/CONV to be clear, requests CONV, and polls both flags every
10 ms without blocking for completion. A two-second failure timeout is exposed
and prevents precision requalification until reboot. Successful completion
restarts estimation. Ordinary GNSS outages preserve learned phase and frequency
for immediate holdover, but start a fresh regression on reacquisition.

Temperature is read for diagnostics at quarter-degree resolution. It does not
drive rapid aging changes or duplicate the DS3231's internal compensation.

## Optional 32.768 kHz

`RTC_32KHZ_GPIO=-1` disables the feature. A user must explicitly choose an
available GPIO, connect the open-drain output with a 3.3 V pull-up, and configure
that GPIO. GP22 in host/build tests is illustrative, not a hardware default.
Common timing/I2C/UART pin collisions are compile-time errors.

PIO1 counts exactly 32768 rising edges per report, independent of duty cycle.
It reports a sequence number and raises about one interrupt per second. The
interrupt timestamps that report using the Pico monotonic counter; no floating
point or I2C runs in the ISR. Sequence gaps and implausible periods invalidate
measurements. This implementation does not claim hardware-latched nanosecond
precision for the report timestamp; its latency budget is part of uncertainty.

Frequency is estimated over 64 reports using:

```
fast_ppm = (cycles * PPS_counter_period_ns /
            (elapsed_Pico_ns * 32768) - 1) * 1e6
```

Four accepted blocks establish validity. A 1/8 filter smooths the frequency
estimate; a 1/16 filter tracks the Pico/RTC report period. Uncertainty includes
a 1.6 ppm endpoint-latency allowance plus measured variation. Estimates outside
100 ppm, or inconsistent changes over 5 ppm, are rejected. Loss of GNSS resets
the in-progress comparison; RTC reconfiguration resets qualification. Signal
absence is detected within 2.5 seconds and falls back to the ordinary SQW path.

While qualified and consistent within 10 ppm of the SQW period, the 32 kHz
period may supply fractional rate interpolation. The high-resolution Pico
counter still supplies fractional timestamps, and SQW still supplies the UTC
epoch. The phase-slope estimator is the aging source; noisier/inconsistent 32 kHz
measurements are not blindly averaged into it.

## Holdover, recovery, and NTP

Only accepted PPS observations that have passed four consecutive recovery
checks update the trusted PPS anchor. That anchor remains usable for 1.5 s,
including a short coast interval after one expected pulse. Rejected samples
cannot replace it. The learned alpha-beta period/rate survives an outage.

At PPS expiry, already-qualified SQW takes over. It advances the saved UTC
second, interpolates using the Pico counter, adds the learned phase offset, and
subtracts accumulated measured RTC frequency drift. Ordinary GNSS loss does
not erase this mapping. Writes, configuration/aging changes, oscillator faults,
SQW loss, gross period/phase inconsistency, and reboot do require requalification.

The outage starts at the last qualified RTC/GNSS observation. Error is:

```
initial_phase_jitter_ns + 2,000,000 ns margin
  + age_ns * (5 ppm hardware margin + learned frequency uncertainty) / 1e6
```

Uncorrected output slew error is also included in the synchronization gate and
NTP dispersion. Defaults are 100 ms maximum error and 24 hours maximum age;
`RTC_HOLDOVER_MAX_DURATION_S=0` disables only the duration limit. With 5 ppm,
the uncertainty gate generally expires after about 5.4 hours, before 24 hours.
Expiry changes quality to coarse/unsynchronized while continuing the best
available RTC-derived estimate.

The shared `utc_continuity_t` holds a UTC/counter anchor and slews toward the
selected measurement at at most 500 ppm. For example, a 10 ms return error takes
at least 20 seconds to remove. Positive and negative errors use the same policy.
There is no ordinary backward step. Errors beyond 100 ms remain unsynchronized
while converging; only initial acquisition before a precision anchor initializes
the clock directly. This deliberately preserves the existing PPS servo rather
than redesigning it into a full multi-sensor servo.

| Timing state | Stratum | Reference | Synchronized |
|---|---:|---|---|
| GNSS_PPS_LOCKED | 1 | GPS | Yes, subject to output error gate |
| GNSS_PPS_COAST (up to 1.5 s) | 1 | GPS | Yes, bounded coast |
| RTC_HOLDOVER | 2 | RTC | Only within age/error policy |
| RTC_COARSE, including expired precision holdover | 16 | LOCL | No, LI=3 |
| GPS_COARSE / acquisition | 16 | LOCL | No, LI=3 |
| UNSYNC / no reference | 16 | LOCL | No, LI=3 |

RTC NTP precision is conservatively -10 rather than the previous -18 claim.
Root dispersion uses the nanosecond error model converted explicitly to us.
The existing UTC leap indicator/smear policy is retained. Reference timestamps
are obtained without changing the live model, and packets are demoted if quality
expires between their receive and transmit timestamp calculations.

## Diagnostics and hardware validation

The existing `/api/health` object and dashboard are extended; existing fields
remain. New fields include timing state, reference ID, filtered phase/jitter,
sample/reject counts, oscillator ppm/uncertainty/source, aging factor/value,
holdover age/error, remaining slew, OSF/initial-read/calendar validity,
temperature, conversion failure, 32 kHz validity/ppm/uncertainty/counts/reason,
Pico report period, SQW period/age, and source changes. `rtc_32k_reject_reason`
is 0=none, 1=sequence/interval discontinuity, 2=implausible/inconsistent frequency.
Validity booleans govern whether numeric values are meaningful.

No firmware was flashed and debug logging was not enabled. After the user
chooses a device and requests flashing, hardware validation should cover:

1. Boot with valid calendar, then OSF set / backup power removed; verify no
   calendar trust before a GNSS set and correct preserved boot diagnostics.
2. Confirm falling-edge relationship after a set and measure phase/jitter under
   network load. Verify 32 kHz pull-up voltage, sequence continuity, absent-input
   behavior, IRQ latency budget, and measured rate against GNSS.
3. Interrupt GNSS while leaving RTC powered; verify immediate qualified holdover,
   growing dispersion, eventual unsynchronized policy, and both signs of recovery
   error without backward client timestamps.
4. Remove SQW; verify expiration and coarse fallback. Reboot without GNSS must
   not inherit precision qualification from the RTC calendar alone.
5. Observe a multi-hour calibration, aging polarity, CONV/BSY completion,
   requalification, and temperature diagnostics on the actual RTC.
6. Verify valid health JSON, dashboard requests and stable http_errors, GNSS
   fix/PPS/relock, timezone version/local zone, and NTP UDP/123 semantics.

Deferred optional work: learning per-device aging sensitivity, temperature
correlation, and 32-kHz-only degraded holdover. The sensitivity is an explicit
parameter ready for a future high-confidence pre/post-adjustment estimator.
32-kHz-only operation needs a separately validated phase/epoch propagation model.
The new output continuity layer provides a common served clock; merging all
internal measurement servos into one estimator is intentionally not part of this
change.

## Test coverage and release checks

`sh tools/test_rtc_holdover.sh` passes with 32 kHz disabled, enabled at the
illustrative GP22, and enabled under AddressSanitizer/UndefinedBehaviorSanitizer.
Each compilation uses C11, `-Wall -Wextra -Werror`. The test calls the same
`timing_select` implementation used by firmware, not a duplicate source policy.

| Required case | Evidence in `tests/test_rtc_holdover.c` |
|---|---|
| ppm/day conversion, aging step/polarity/bounds | `test_units_and_aging` |
| uncertainty units and 1 s / 1 min / 1 h / 24 h growth | `test_uncertainty` |
| phase compensation, jitter qualification, outliers | `test_phase`, `test_source_sequence` |
| phase-slope ppm, bounded rolling window, restart | `test_frequency` |
| 32 kHz ratios, invalid counts, missed reports, no configured GPIO | `test_32k` |
| PPS recovery hysteresis and freshness | `test_recovery_and_semantics` |
| SQW-only holdover entry and continued operation | `test_source_sequence` |
| fast/slow RTC and positive/negative GNSS recovery | `test_continuity`, `test_source_sequence` |
| nondecreasing served timestamps and stale observations | `test_continuity`, `test_source_sequence` |
| OSF validation and flag-preserving output configuration | `test_osf` |
| cold coarse start and signal-loss expiration | `test_source_sequence` |
| stratum/reference/unsynchronized quality transitions | `test_recovery_and_semantics`, `test_source_sequence` |

All required existing host tests pass with warnings as errors:

```
cc -std=c11 -Wall -Wextra -Werror -I. tests/test_timezone.c timezone_lookup.c generated/timezone_data.c -o /tmp/test_timezone && /tmp/test_timezone
cc -std=c11 -Wall -Wextra -Werror -I. tests/test_gps_validation.c -o /tmp/test_gps_validation && /tmp/test_gps_validation
cc -std=c11 -Wall -Wextra -Werror -I. tests/test_ubx.c ubx.c -o /tmp/test_ubx && /tmp/test_ubx
```

Timezone coverage includes geographic selection, DST changes, non-hour rule
offsets, ocean fallback, crossing/recrossing the Apalachicola River timezone
border, and fixed-offset and DST-zone year-range endpoints. The existing host
benchmark also ran: about 4461 ns/polygon lookup and 135 ns/rule lookup on this
host (not Pico performance measurements).

The embedded versions, host `/usr/share/zoneinfo/tzdata.zi`, and latest upstream
boundary release were all `2026c` on the release check. The source asset remains
`timezones-1970.geojson.zip`. Generated files were not rewritten. Data covers
2020–2050, with **301 zones, 73,462 points, 1,307 rings, 6,819 transitions**.
ELF symbols place the const arrays in the `0x100...` flash/XIP region. Calls to
timezone lookup/offset remain in `timezone_refresh` and display presentation;
none occur in PPS capture, RTC synchronization, holdover, NTP timestamp
generation, leap handling, or alpha-beta discipline.

Firmware was also checked with `-Wall -Wextra`. Remaining unused optional-path
and legacy date-format warnings were reproduced against the saved baseline;
new helper code is warning-clean. Hardware observations before flashing are
not substituted for new-firmware validation; see the hardware checklist above.

## Verified firmware artifacts

All four complete firmware builds succeeded. `arm-none-eabi-size` reports:

| Variant | 32 kHz | Text/flash bytes | Data bytes | BSS bytes |
|---|---|---:|---:|---:|
| L76K | Disabled | 1,143,052 | 0 | 83,472 |
| UBX | Disabled | 1,142,436 | 0 | 91,612 |
| L76K | Enabled, illustrative GP22 | 1,144,348 | 0 | 83,488 |
| UBX | Enabled, illustrative GP22 | 1,143,740 | 0 | 91,628 |

The ordinary device update files remain:

- `build-l76k-webfix/pico_gps_l76k.uf2`
- `build-ubx-webfix/pico_gps_l76k.uf2`

Their SHA-256 checksums are respectively:

```
599b0deb814611bffca1e0d2529537edc68a84a69c1b7ba58755cac87acd4cf5
c8bdd18ef4974736d417504fdb66de285fde24a05f6baf16e381504c67b2c200
```

Optional enabled-build artifacts are saved in
`build-rtc32k-check/l76k-verified/` and `build-rtc32k-check/ubx-verified/`.
These are compile-validation images using GP22 and default configuration;
they are not a selection of wiring or deployment configuration for either
existing device. The final `build-rtc32k-check` configuration is UBX enabled.

The network stack symbol is 8192 bytes in main RAM; the core0 linker stack
bounds span 4096 bytes. The SDK's original 2 KiB default core1 stack reservation
is retained for compatibility, but the network core uses the explicit stack.

## Files changed

| File | Change |
|---|---|
| `main.c` | Driver startup/OSF, asynchronous calibration, phase/frequency integration, trusted PPS anchors, source selection, NTP/diagnostics, explicit network stack |
| `rtc_holdover.h` | Configurable policy, units, estimator and continuity types/interfaces |
| `rtc_holdover.c` | Robust phase, bounded slope, aging calculations, uncertainty, source policy and monotonic output |
| `rtc_32k.h` | Optional frequency capture/estimator interface |
| `rtc_32k.c` | Low-rate PIO report ISR, frequency estimate/filter, discontinuity checks, disabled stubs |
| `rtc_32k.pio` | Rising-edge batching and sequence reports |
| `CMakeLists.txt` | New sources/PIO/configuration and stack budgets |
| `tests/test_rtc_holdover.c` | Deterministic numerical and source-sequence regression tests |
| `tools/test_rtc_holdover.sh` | Disabled/enabled/sanitized host test runner |
| `timezone_lookup.c` | Correct fixed-offset zone validity to the generated year range |
| `tests/test_timezone.c` | Required non-hour/border/year-range coverage and data-count report |
| `README.md` | Updated RTC operation, wiring, options, and test instructions |
| `docs/rtc-holdover-implementation.md` | Baseline trace, implementation, verification, deferred options and hardware checklist |

Firmware build directories contain regenerated build metadata, PIO headers,
objects, ELF/UF2 artifacts. There is no Git repository metadata in this
workspace, so no commit or Git diff is available. Original `main.c` and
`CMakeLists.txt` were saved in `/tmp` before firmware edits.
