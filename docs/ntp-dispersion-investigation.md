# NTP dispersion investigation — chronological work log

The checkpoint notes below are historical, including their then-open work
items. See [the final implementation report](ntp-dispersion-fix.md) for current
behavior, verification and the explicitly unperformed hardware deployment.

Objective: attachment `2def4d28-47c6-4593-b36d-6c97daf1acd6/pasted-text-1.txt`.
This record is baseline evidence and an implementation work log, not a
completion report. The first calculation/diagnostics implementation is present
but integration and requirement-by-requirement verification remain incomplete.

## Confirmed baseline path

`main.c:sntp_recv` captures `recv_ns` after lwIP callback entry, request
validation and rate limiting. `get_sntp_time_at` invokes the shared clock
selector. The PPS quality branch calls `get_pps_root_dispersion_us(recv_ns)`;
`ntp_short_from_us` multiplies microseconds by 65536 then divides by 1000000
(truncating), and `sntp_fill_short_timestamp` writes the resulting field at
packet bytes 8–11. Root delay is zero at bytes 4–7. PPS precision is explicitly
-20, separately from uncertainty.

The PPS calculation is:

```
age_us = (now_ns - trusted_pps_ns) / 1000
error_us = 100 + 4 * pps_jitter_rms_us + age_us / 4
           + abs(utc_continuity.residual_ns) / 1000
return min(error_us, 2000000)
```

The age term therefore assumes 0.25 seconds of uncertainty per elapsed second
(250,000 ppm). This is the confirmed large-dispersion cause, not an NTP
16.16 scale error and not RTC holdover leaking into the PPS branch.
The age anchor is the latest qualified PPS, refreshed by
`gnss_recovery_observe`, not boot time or last RTC write.

With zero whole-microsecond RMS and slew, an age of about 760.8 ms yields
190.3 ms internal dispersion. The quoted historical packet alone cannot
establish the precise RMS/slew/age breakdown, but it is consistent with this
formula and the live packet evidence below. Do not invent historical inputs.

The suspected RTC ns/us mismatch is already fixed in this worktree:
`rtc_holdover_error_ns` uses `age_ns * frequency_uncertainty_ppm / 1e6`,
then `get_rtc_holdover_dispersion_us` converts ns to us. It is a separate
branch. Existing holdover margin/policy must remain intact and receive
additional regression coverage.

## Live pre-fix evidence, 2026-09-08 UTC

At roughly 10.8 hours uptime, hardware still reports GPS and RTC LOCKED,
32 kHz valid, and long-term RTC frequency valid. `chronyc ntpdata
10.254.0.11` reported stratum 1, LI Normal, refid GPS, precision -20,
root delay zero, root dispersion **0.242737 seconds**, peer delay
15.174773 ms and response time 127.126 us. Client source selection was `^+`;
source selection is not a correctness gate.

Five direct UDP/123 replies demonstrate the erroneous age-dependent ramp:

| Receive fraction / age (ms) | Raw unsigned 16.16 | Decoded dispersion (us) |
|---:|---:|---:|
| 194.614539 | 3195 | 48751.831 |
| 391.990276 | 6428 | 98083.496 |
| 571.335664 | 9367 | 142929.077 |
| 750.142652 | 12296 | 187622.070 |
| 928.689674 | 15222 | 232269.287 |

All five were stratum 1 / LI 0 / GPS. Receive timestamps minus reference
timestamps equal the listed receive fractions. This independently reproduces
the approximately age/4 ramp while PPS stays healthy.

## Work remaining

Complete the focused units/state/reference/servo audit and consult primary NTP
specifications. Implement a defensible measured, source-specific uncertainty
model, overflow-safe conservative 16.16 encoding, API/UI breakdown, conversion,
PPS/holdover/transition/long-uptime tests and both firmware builds under project
timezone policy. Audit packet RX/TX quality consistency and timestamp placement.
Document every explicit attachment requirement and its verification evidence.
Hardware before/after testing requires a newly authorized flash and BOOTSEL;
do not assume the earlier 32 kHz deployment authorizes another flash.

## First implementation checkpoint

Added `ntp_uncertainty.h/.c`, linked from CMake, and
`tests/test_ntp_uncertainty.c`. Replaced PPS age/4 in `main.c` with a dynamic
nanosecond calculation: PIO tick + max(4 RMS, 6 MAD, absolute innovation) +
age times frequency uncertainty + retained 100 us engineering margin + slew.
Frequency uncertainty uses the interval residual envelope converted to ppm,
with a 15 ppm conventional allowance (RFC 5905) once 60 samples exist and a
5000 ppm startup allowance corresponding to current +/-5 ms interval bounds.
The 100 us margin is retained from baseline, not experimentally calibrated.
RMS/MAD/innovation are correlated interval metrics, not independent absolute
UTC measurements; this limitation must be assessed in the final model audit.

Encoding now splits seconds/remainder to avoid overflow, rounds up, and
saturates. Health JSON includes decoded root dispersion, raw 16.16, delay,
clock uncertainty, source, age, base, phase, frequency growth, margin, slew,
and frequency uncertainty. RTC packet dispersion now uses nanoseconds directly.

Primary specification consulted: https://www.rfc-editor.org/rfc/rfc5905.html
(packet/short-format definitions, precision and conventional 15 ppm tolerance).

Validation at this checkpoint: new host tests passed C11/Wall/Wextra/Werror,
also ASan/UBSan; existing timezone, GPS validation, UBX, and three RTC holdover
test variants passed. Both firmware builds passed without reported warnings.
UBX text/data/BSS = 1,146,612/0/91,624 bytes; L76K =
1,145,948/0/83,468. Embedded TZDB/boundary and upstream/local versions all
2026c; no regeneration. Years 2020–2050, 301 zones, 73,462 points, 1,307 rings,
6,819 transitions. No device flashed.

Specific remaining integration gaps to address before release:

- Make timestamp, quality, uncertainty and reference snapshots consistent;
  the current helper's separate lock can encounter a new PPS after the
  request timestamp and currently treats that case as maximum uncertainty.
  RX/TX source changes must not leave stale dispersion/metadata attached.
- Transfer PPS uncertainty into RTC handoff (currently selector initializes
  holdover from SQW jitter alone); preserve policy, compensated frequency,
  and RTC margin without double counting.
- Add human-readable dashboard breakdown, verify JSON capacity/stack usage,
  make endian regression test exercise the firmware's actual writer, and
  add full source-transition/age-source tests, not just helper tests.
- Finish the absolute epoch/capture-path audit: tiny interval jitter does not
  establish absolute PPS-to-UTC uncertainty. Document margins honestly.
- Complete audit of signed conversions, root-delay representation, reference
  semantics, coarse floors/caps, TX placement and configuration variants.
- Repeat full builds/tests after final edits, and write a requirement matrix
  covering all 31 attachment sections; optional hardware validation still
  needs explicit deployment scope/BOOTSEL.

## Snapshot/handoff checkpoint

`get_sntp_time_details` now selects time and obtains uncertainty/reference
metadata under one `stats_mutex` acquisition. Legacy callers retain a wrapper.
NTP RX/TX use independent internally consistent snapshots, take the larger
dispersion when the source is unchanged, and demote a response to unsynchronized
if the source changes during processing (including reacquisition). The wire
writer now calls the exact `ntp_write_u32` helper covered by endian tests.
Health uncertainty comes from the same snapshot as its NTP time and quality.

`timing_observation_t.rtc_reference_error_ns` carries the PPS uncertainty
recorded at SQW qualification into the holdover initial error, in addition to
SQW jitter. Output slew is excluded from that carried reference error and added
once at reporting time. The existing RTC margin and frequency policy remain.
Dashboard has a readable breakdown with base/handoff, interval residual,
frequency growth, margin, slew, total and encoded wire value.

Added selector transition tests for coarse acquisition -> PPS -> coast -> RTC
holdover -> PPS recovery -> RTC expiration -> no source. They verify source,
stratum/refid, inherited reference error, exact growth and removal of stale RTC
age after recovery. They do not yet exercise actual packet header/LI writes or
the firmware's servo recovery statistics; those are remaining coverage gaps.

All required host tests and sanitizer runs passed again. Both builds passed:
UBX text/data/BSS 1,147,236/0/91,632; L76K 1,146,532/0/83,476 bytes. Timezone
versions rechecked: embedded/upstream/local 2026c, unchanged counts/range.
No flash performed. Builds are integration checkpoints, not final release.

Important absolute-timing audit finding still to resolve: main initializes the
first `pps_ns` from application `hires_time_ns()` after DMA polling, then adds
PIO counter deltas on later observations. Thus 20 ns interval quantization
does not bound startup epoch placement latency. Also inspect the PIO's
capture/exit instructions that do not decrement X: their per-period contribution
is not explicitly accounted for in the main-loop delta conversion. Do not
claim the retained 100 us engineering allowance proves absolute UTC accuracy;
measure/bound the actual alignment uncertainty or otherwise address it before
claiming a defensible complete model. Preserve ISR/PIO timing unless a clearly
demonstrated correctness issue requires a narrowly tested change.
