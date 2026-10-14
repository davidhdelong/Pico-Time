# NTP root-dispersion fix

## Result and deployment status

The excessive PPS-locked dispersion was a **250,000 ppm age-growth term** in
`main.c:get_pps_root_dispersion_us`, not a 16.16 scale mistake or RTC age
leaking into the PPS branch. The replacement calculates source-specific
uncertainty, exposes its components, and conservatively encodes the result.
This is not a small constant substituted for the observed 190 ms.

Both firmware variants have been built and host-tested. **Neither has been
flashed for this task.** The live UBX still reports build `Sep 7 2026 20:13:28`;
there was no BOOTSEL device at the deployment check. A fresh flash request and
BOOTSEL are required for the conditional post-change hardware validation.
The earlier 32 kHz deployment is not a post-fix test.

## Original implementation and exact cause

The original path was:

```
trusted_pps_ns, pps_jitter_rms_us, utc_continuity.residual_ns
 -> get_pps_root_dispersion_us(recv_ns)
 -> ntp_short_from_us(microseconds)
 -> sntp_fill_short_timestamp(resp + 8, raw)
 -> UDP/123 reply -> chrony
```

`sntp_recv` chose this branch only for `SNTP_QUALITY_PPS`: stratum 1, GPS
reference ID, precision -20. Its complete calculation was:

```
age_us = (recv_ns - trusted_pps_ns) / 1000
error_us = 100 + 4 * pps_jitter_rms_us + age_us / 4
           + abs(utc_continuity.residual_ns) / 1000
error_us = min(error_us, 2000000)
raw = floor(error_us * 65536 / 1000000)
```

`age_us / 4` means **250 ms per elapsed second**, not a plausible residual
frequency uncertainty of a disciplined oscillator. The anchor was already
the latest accepted PPS and refreshed correctly. The faulty growth therefore
repeated every second; it did not require hours of holdover or an old NTP
reference timestamp.

The supplied `0.190292` seconds corresponds to raw **12471**, whose exact
wire value is **0.1902923583984375 seconds**. The old integer-microsecond
calculation produces that raw value for internal errors 190293–190307 us.
One reproducible example is:

| Original contribution | Example |
|---|---:|
| Fixed base/safety allowance | 100 us |
| Separate capture contribution | None |
| Four times whole-us RMS | 0 us |
| Separate servo/frequency estimate | None |
| PPS age | 760772 us |
| Erroneous age contribution | 190193 us |
| Remaining output slew | 0 us |
| RTC holdover contribution | **0 us** |
| Internal result | 190293 us |
| Encoded result | 12471 |
| Decoded wire result | 190292.358398 us |

This example reproduces the packet exactly; it is **not a claim that the
unrecorded historical RMS/slew/age inputs are known**. Baseline live UDP tests
independently demonstrated dispersion rising from 48.752 ms at PPS age
194.615 ms to 232.269 ms at age 928.690 ms. See the preserved
[investigation log](ntp-dispersion-investigation.md) for all five samples.

Confirmed additional issues addressed:

- Old fixed-point conversion truncated down and had no general overflow or
  saturation protection. Its bounded original call sites did not overflow;
  the new arbitrary-nanosecond API is hardened for the full input range.
  Scaling and byte order were otherwise correct.
- Timestamp, uncertainty and reference fields could observe different PPS
  updates because they were read under separate lock acquisitions.
- RTC handoff previously carried SQW jitter but not the uncertainty of its
  PPS-derived UTC reference.
- Coarse dispersion used last RTC synchronization/GPS fix times rather than
  the current cached calendar/message sample; a 250 ms base also did not cover
  whole-second truncation. Its four-second cap hid further age growth.
- PIO **interval** quantization alone did not bound the reconstructed PPS
  timestamp's absolute placement in the Pico monotonic clock.

The suspected RTC ns/us age bug was **already fixed before this task**:
`rtc_holdover_error_ns` uses `age_ns * frequency_uncertainty_ppm / 1e6`.
It is covered again by exact ppm/time tests. It did not cause the PPS ramp.

## Architecture and PPS uncertainty model

The existing alpha-beta estimator learns counter period and period-rate,
not a statistically calibrated absolute UTC phase covariance. Its RMS/MAD
and innovation are interval residuals. The PIO/DMA counter is retained, as are
the servo equations, GNSS recovery gate and RTC/32 kHz priority rules.

The counter's first reconstructed PPS timestamp is anchored by application
processing time; subsequent counter deltas propagate that alignment. Capture
and exit instructions also take time without decrementing the PIO counter.
Neither effect is proven small by a 20 ns interval-jitter display. This task
does not rewrite PIO instructions or move timing into GPIO callbacks.

A **reporting-only main-loop observer** samples the existing PPS pin. The last
low observation and first high observation bracket a physical rising edge.
`ntp_placement_bound_ns` computes the maximum distance of the reconstructed
PPS timestamp from the bracket endpoints. Two recent brackets support the
previous trusted pulse during short coast/rejection. Brackets wider than
half a second or inconsistent with the reconstructed timestamp are rejected.
This observes actual alignment error, including startup placement and
accumulated counter-mapping discrepancy; it never changes the served epoch,
PIO/DMA data, period, or servo correction.

For qualified PPS, all terms below are nanoseconds unless stated otherwise:

```
capture = measured placement bound + ceil(1e9 / PPS_CAPTURE_COUNTER_HZ)
residual = max(4 * RMS_ns, 6 * MAD_ns, abs(innovation_ns))
frequency_uncertainty_ppm = max(residual / 1000, oscillator_allowance_ppm)
age = now_ns - trusted_pps_ns
growth = ceil(age * frequency_uncertainty_ppm / 1e6)
clock_uncertainty = capture + residual + growth + 100000 + abs(slew_ns)
```

Correlated residual estimators are combined by a maximum, not summed. One
second of interval error in ns divided by 1000 gives ppm. The oscillator
**frequency offset** and period-rate are not treated as uncertainty.
The residual term bounds phase/reference behavior, while its rate equivalent
covers subsequent extrapolation; this is a conservative engineering envelope,
not a claim that the terms are statistically independent.

The oscillator allowance is 15 ppm after 60 samples, the conventional NTP
allowance, and 5000 ppm while that history fills, matching this firmware's
default +/-5 ms interval acceptance range. The old **100 us** engineering
margin is retained for receiver/counter/model effects; it was not reduced to
influence chrony. Actual measured placement error is **additional**, so that
margin is not misrepresented as a calibrated absolute-error guarantee.
The NTP definitions and conventional tolerance are documented in
[RFC 5905](https://www.rfc-editor.org/rfc/rfc5905.html).

An out-of-order queued timestamp may be older than the continuity model's
current epoch. That readback lag is added to the reported output-mapping
(`dispersion_slew_us`) contribution. Actual continuity behavior is unchanged.

**Important limitation:** if the main loop cannot observe a low/high bracket
(for example, a very narrow PPS pulse or a long processing stall), the
absolute placement estimate is unknown. NTP fails closed to stratum 16 with
saturated dispersion; it does not invent a small bound from interval jitter.
Hardware validation should verify this observer sees the receiver's pulse
width. PPS interval capture itself continues through PIO/DMA.

## RTC holdover and coarse time

At SQW qualification, `rtc_reference_error_ns` records current PPS UTC
uncertainty without the live output slew. On handoff, `timing_select` uses:

```
initial_error_ns = SQW_jitter_ns + carried_PPS_reference_error_ns
holdover_error_ns = initial_error_ns + RTC_HOLDOVER_MARGIN_NS
                  + ceil(holdover_age_ns * residual_frequency_uncertainty_ppm / 1e6)
reported_error_ns = holdover_error_ns + abs(current_output_slew_ns)
```

The saved last-qualified SQW observation anchors outage age. Default RTC
margin remains 2 ms. Frequency uncertainty remains the 5 ppm hardware margin
plus learned regression uncertainty when valid; measured frequency offset is
compensated separately, not added again as uncertainty. The 32 kHz signal
still contributes only eligible RTC interpolation rate; it cannot displace
healthy GNSS PPS. Default synchronization limits remain 100 ms uncertainty
and 24 hours, with SQW freshness required.

PPS reacquisition clears `holdover.active` and uses the PPS model with its
fresh accepted-PPS age. It does not inherit hours-old RTC dispersion, nor
does it reset uncertainty to zero. Coarse time remains stratum 16/LI=3/LOCL.
Its model now allows one second for calendar/message truncation plus age
since the actual cache/message sample and output slew, with saturating
arithmetic rather than the obsolete four-second cap. Expired RTC holdover
error is not discarded if it exceeds the coarse estimate. No trustworthy
sample means unknown/saturated dispersion, not a precision claim.

| State | Packet quality | Uncertainty age/reference |
|---|---|---|
| Acquire before four accepted pulses | 16 / LOCL / LI 3 | Actual coarse sample, or unknown |
| Qualified PPS, initial history | 1 / GPS / existing leap policy | Latest trusted PPS, startup rate allowance |
| PPS tracking | 1 / GPS / existing leap policy | Latest trusted PPS, measured residual envelope |
| Brief coast, up to existing 1.5 s timeout | 1 / GPS | Same last trusted PPS; extrapolation grows |
| Qualified SQW holdover | 2 / RTC | Last GNSS-qualified SQW observation |
| RTC/GNSS coarse or expired holdover | 16 / LOCL / LI 3 | Cache/message age; retained larger expired error |
| No sample or unknown absolute placement | 16 / LOCL / LI 3 | Unknown; saturated wire value |

## Snapshot, packet encoding and reference timestamps

`get_sntp_time_details` selects the timestamp, quality, uncertainty and NTP
reference seconds under one `stats_mutex` acquisition. The NTP reference
remains the latest trusted PPS UTC second for GPS, and last GNSS-qualified
RTC UTC second in holdover. Historical reference formatting does not call
the live source selector. Existing leap-smear handling is preserved.
During an active configured 24-hour smear, the served timescale intentionally
differs from unsmeared UTC; the clock uncertainty estimate is not a claim that
this deliberate smear offset is zero. No active smear was part of the supplied
incident, and this task does not change that existing timescale policy.

`ntp_response_header` is the **actual firmware header writer** tested on the
host. It uses the larger RX/TX uncertainty when the source is unchanged. A
source change, invalid timestamp or unknown uncertainty demotes that one
reply to 16/LOCL/LI=3 and clears a stale reference timestamp. The transitional
four-second minimum is a conservative unsynchronized sentinel, not a PPS
floor, and cannot replace a larger/unknown error.

`ntp_dispersion_from_ns` splits seconds and remainder before multiplying:

```
seconds = ns / 1e9
fraction = ceil((ns % 1e9) * 65536 / 1e9)
raw = saturate_uint32(seconds * 65536 + fraction)
```

It rounds **up** and saturates before overflow. One wire unit is
15.2587890625 us. Zero encodes to zero; 1/10/100 us encode to 1/1/7;
1/10/100 ms encode to 66/656/6554; one second encodes to 65536.
`ntp_write_u32` writes big-endian bytes 8–11. Root delay, bytes 4–7, is signed
16.16 conceptually but exactly zero for the directly attached reference;
there is no upstream network delay to add. Precision stays signed -20 for
PPS and -10 for RTC/coarse: it describes clock resolution, not error bounds.

## Focused units audit

| Values | Units and conversion |
|---|---|
| `trusted_pps_ns`, receive/transmit/placement timestamps | Pico monotonic ns; subtract before conversion |
| `pps_jitter_ns_sumsq` | ns squared; divide by count, sqrt, ceil gives ns RMS |
| `ab_mad_ns`, `ab_innovation_ns`, period state | ns; period-rate is ns per second, not ppm uncertainty |
| `ntp_uncertainty_t` integer members | ns, saturating uint64 arithmetic |
| `frequency_uncertainty_ppm` | ppm; `age_ns * ppm / 1e6` gives ns |
| `rtc_reference_error_ns`, `initial_error_ns`, margin | ns; saturating addition at handoff |
| `rtc_holdover.start_ns`, outage age | ns; never assigned to an `_us` variable |
| Coarse cache `sample_time` | SDK us, checked before multiplication by 1000 |
| API `_us`, `_ms`, `_s` values | ns divided by 1e3, 1e6, 1e9 at display boundary |
| `ntp_root_dispersion_ntp16_16` | unsigned fixed-point seconds; explicit endian writer |
| NTP reference seconds | UTC + NTP epoch offset, unrelated to oscillator-age accounting |

Extreme integer tests cover uint64 saturation, `INT64_MIN` magnitude, maximum
wire values and long-uptime selector timestamps. This does not extend the
firmware's calendar representation beyond its supported date ranges or the
existing NTP era semantics. Other inspected `age_us` variables use SDK
microseconds or an explicit ns/1000 conversion. Timezone functions remain
presentation-only, outside PPS/RTC/holdover/NTP/leap/alpha-beta calculations.

## Wi-Fi timestamps and latency

The device uses CYW43 built-in Wi-Fi and lwIP on core1, not a W5500 NIC.
T2 is an application timestamp in `sntp_recv`, after callback entry, request
validation and rate limiting. Radio/driver/lwIP buffering precedes it.
T3 is obtained later in the same callback, before final header formatting,
payload copy and `udp_sendto`; driver/radio queuing follows it. Neither is a
hardware MAC/PHY timestamp. There is variable software and radio latency;
the observations do not establish a stable fixed bias worth compensating.

One safe improvement moves `pbuf_alloc` **before T3**, eliminating allocator
time after the transmit timestamp. There is no network-stack redesign,
arbitrary fixed correction, or addition of the client's RTT to dispersion.
The baseline chrony response processing times were approximately 127–158 us,
with peer delays around 12.7–15.2 ms (the supplied example was 8.669 ms).
These are measurements of different quantities, not proof of equivalent
internal UTC error. Unmeasured radio/software timestamp-placement variability
remains separate from the internal clock bound exposed here.

## Diagnostics and files

`/api/health` adds:

- `clock_uncertainty_valid`, `clock_uncertainty_us`
- `ntp_root_dispersion_us`, `ntp_root_dispersion_ntp16_16`, `ntp_root_delay_us`
- `dispersion_source`, `dispersion_age_ms`
- `dispersion_base_us`, `dispersion_phase_us`, `dispersion_frequency_us`
- `dispersion_margin_us`, `dispersion_slew_us`
- `dispersion_frequency_uncertainty_ppm`

The existing timing/servo/RTC fields still expose state, stratum/refid,
innovation, period/rate, residual RMS/MAD, rejections, holdover age and RTC
frequency uncertainty. Base means measured PPS placement/quantization in
PPS mode and inherited reference plus SQW error in RTC mode. Current API
values describe its coherent sample, not the last NTP packet; a packet can
round up differently or conservatively demote if its source changes.

The new dashboard section labels base/handoff, interval residual, frequency
growth, margin, output mapping, total and rounded wire value. Invalid/coarse
quality shows **Unqualified**, not a fake precise total.

Modified/added files:

- `main.c`: snapshot integration, uncertainty selection, GPIO reporting
  brackets, SQW reference handoff, NTP callback and health/dashboard.
- `ntp_uncertainty.h/.c`: overflow-safe conversion, placement/model helpers,
  shared packet header writer and source-specific coarse/PPS calculations.
- `rtc_holdover.h/.c`: explicit carried reference-error field and safe sum.
- `CMakeLists.txt`: link new module; retain GCC stack-usage evidence.
- `tests/test_ntp_uncertainty.c`, `tools/test_ntp_uncertainty.sh`: model,
  packet, encoding, bounds, holdover and transition regressions.
- `tests/test_ntp_dashboard.py`: actual embedded JS bindings for both variants
  and health-format capacity check under shipped value ranges.
- `README.md`, this report and the chronological investigation log.

## Tests, builds and remaining hardware validation

Run from the repository root:

```
sh tools/test_ntp_uncertainty.sh
python3 tests/test_ntp_dashboard.py
sh tools/test_rtc_holdover.sh
cc -std=c11 -Wall -Wextra -Werror -I. tests/test_timezone.c timezone_lookup.c generated/timezone_data.c -o /tmp/test_timezone && /tmp/test_timezone
cc -std=c11 -Wall -Wextra -Werror -I. tests/test_gps_validation.c -o /tmp/test_gps_validation && /tmp/test_gps_validation
cc -std=c11 -Wall -Wextra -Werror -I. tests/test_ubx.c ubx.c -o /tmp/test_ubx && /tmp/test_ubx
cmake --build build-l76k-webfix -j2
cmake --build build-ubx-webfix -j2
```

Host tests passed C11/Wall/Wextra/Werror, including ASan/UBSan for timing
models. They cover all requested encoding magnitudes; exact wire units,
rounding, saturation and endian round trips; dynamic PPS/coast/startup/noisy
models; placement brackets, stale/missing/reversed brackets and uint64
extremes; holdover at 0/1/60/3600/21600/86400 seconds with 0.1/1/2/5 ppm;
the six requested source transitions; PPS recovery qualification in existing
RTC tests; and all 25 RX/TX quality pairs in the actual packet header writer,
including LI/stratum/refid/precision/delay/dispersion/reference clearing.

Both dashboard variants pass DOM-binding tests (not a claim of on-device
render testing). Conservative health-format bounds are 5321 bytes L76K and
5720 bytes UBX within the existing 6400-byte static buffer for shipped value
ranges. GCC stack reports show 2480 bytes for the UBX health formatter and
296 for its NTP callback, within the existing 8 KiB network stack; main is
1440 bytes with a separate 4 KiB core0 stack. Library nesting and runtime
headroom still warrant the usual on-device observation.

The explicit firmware Wall/Wextra syntax audit found only the existing
debug-disabled `parse_rmc` unused `phase_us` warning, also present in the
pre-task backup. No new warning was found. No debug logging was enabled.

Timezone sources were checked before builds: embedded IANA/boundary, local
IANA, and upstream boundary release are all **2026c**. No regeneration or
generated-file rewrite was needed. The geographic source remains
`timezones-1970.geojson.zip`, years **2020–2050**, **301 zones, 73,462 points,
1,307 rings, 6,819 transitions**. Arrays remain read-only flash/XIP symbols.
Final ELF sizes and hashes are recorded in the final verification entry below.

Before/after chrony status:

```
Supplied before:                Root dispersion : 0.190292 seconds
Baseline live during diagnosis: Root dispersion : 0.242737 seconds
Later OLD firmware sample:      Root dispersion : 0.011917 seconds
Post-fix device sample:         NOT AVAILABLE — new UF2 not flashed
```

The later old-firmware value is another point in the faulty age/4 sawtooth,
not evidence that the fix is deployed. Do not predict a post-flash number
from interval jitter alone: the new physical placement term may justify
millisecond-scale dispersion. Chrony choosing another source is not failure.

After an authorized UBX flash, check health JSON, pulse brackets/qualification,
GPS/PPS lock, RTC/32 kHz state, timezone/version, HTTP errors, and direct NTP
headers. Then run `chronyc ntpdata 10.254.0.11` and `chronyc sources -v` after
chrony obtains a new sample, comparing the advertised value to the API
breakdown. Do not enable serial/debug merely for this check. Physical
GNSS-loss/recovery and temperature/long-duration calibration remain hardware
tests, not claims inferred from host tests.

## Requirement audit (attachment sections)

| Section | Evidence / disposition |
|---|---|
| 1 | Original complete call/field path above, recorded before edits in investigation log |
| 2 | Exact formula, 12471 wire value, reproducible breakdown, live age/4 ramp; historical inputs explicitly not invented |
| 3 | Units table, explicit ns model, saturating arithmetic and sanitizer tests |
| 4 | Existing RTC age_ns formula inspected; exact ppm regressions; separate PPS branch |
| 5 | Split/ceil/saturate implementation, known magnitudes/endian tests; zero signed root delay |
| 6 | Measured placement plus existing residual envelope, age-scaled rate allowance, retained margin |
| 7 | Latest trusted PPS age in one snapshot; coarse cache age corrected; long-uptime tests |
| 8 | Reference epochs preserved and read atomically; no historical selector reentry |
| 9 | Precision remains -20/-10 independently of dispersion; actual header tests |
| 10 | Carried PPS + SQW error, existing RTC margin, residual-ppm growth and policy tests |
| 11 | State table, selector tests, recovery helper tests and header matrix |
| 12 | Explicit handoff field; recovery clears active holdover; stale-age regression |
| 13 | No RTT term in model; separate Wi-Fi discussion |
| 14 | T2/T3 call-site audit; allocator moved before T3; no fixed Wi-Fi correction |
| 15 | Baseline chrony delay and processing values separated from UTC uncertainty |
| 16 | Health fields and existing servo/RTC diagnostics mapped above |
| 17 | Embedded readable section and actual-JS DOM-binding tests |
| 18 | Encoding tests from zero to maximum, saturation, overflow, round trips and byte order |
| 19 | Healthy age sweep stays microsecond-scale with specified small placement; startup/noisy cases also tested |
| 20 | Required six durations/four ppm rates; policy expiry, exact examples, saturation/invalid inputs |
| 21 | Six selector transitions + recovery gate tests + all quality-pair packet metadata tests |
| 22 | No 100–250 ms PPS floor found; age/4 removed; 100 us retained; coarse base/cap corrected; existing 100 ms holdover gate retained |
| 23 | PPS model has no RTC state inputs; handoff is one-way and recovery clears active RTC |
| 24 | Existing interval RMS/MAD/innovation used without replacing servo or confusing rate estimate with uncertainty |
| 25 | PIO program, DMA capture and IRQ bodies unchanged; observer/model work stays in main/application code |
| 26 | Selector remains PPS-first; new GPIO observations affect reporting/trust only, never pull UTC toward RTC/32 kHz |
| 27 | Before chrony captured; conditional after-hardware check unavailable without authorization/BOOTSEL, explicitly not claimed |
| 28 | Selection is documented as non-gating; correctness tested independently |
| 29 | Units/saturation helpers, no new ISR floating point, no unrelated network/RTC redesign |
| 30 | Required host tests, sanitizers, both builds, warning/stack/capacity audit and regression matrix |
| 31 | This report: cause, models, code/files, diagnostics, tests, before/deployment status and limitations |

## Final verification entry

All listed host commands passed after the last source edit, including normal
and sanitizer NTP runs, both dashboard variants/capacity checks, existing RTC
test variants, timezone, GPS-validation and UBX tests. Both required firmware
builds passed after rechecking that timezone sources remain 2026c.

| Variant | Text/flash bytes | Data bytes | BSS bytes |
|---|---:|---:|---:|
| UBX, GP22 32 kHz enabled | 1,148,204 | 0 | 91,656 |
| L76K, 32 kHz disabled | 1,147,564 | 0 | 83,500 |

Artifacts:

- `build-ubx-webfix/pico_gps_l76k.uf2`, SHA-256
  `bb1cac517dee58232c991fdd6adb2a993654c000e18c1aff7d0950790e8eab23`
- `build-l76k-webfix/pico_gps_l76k.uf2`, SHA-256
  `cbaca56ce82aede0fd33a52f527a1e588425ac84074f2f9ab789e96863e4dcca`

UBX deployment settings remain protocol UBX, UART 38400, GP22 and debug off.
L76K deployment settings and timezone source data were preserved. The
post-fix physical-device/chrony checks remain unperformed, as described above.
