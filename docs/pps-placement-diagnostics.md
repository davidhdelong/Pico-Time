# PPS placement diagnostics — 2026-09-09

Implemented and built. UBX was subsequently flashed on explicit request after
verifying BOOTSEL serial `50BABB25DD2BEAFE` and the matching SHA-256 below.
Live build: `Sep 9 2026 10:30:55`. L76K was not flashed with this release.
This release adds diagnostic evidence; it does not fix or assert a cause for
the overnight uncertainty increase.

Post-flash verification through uptime 318 seconds: GPS/PPS and RTC LOCKED,
17 satellites at the final sample; 32 kHz qualified by uptime 277 seconds with
four accepted measurements and no rejections. Health JSON and the complete
14,105-byte dashboard succeeded after network acquisition; HTTP errors and
Wi-Fi reconnects remained zero. Direct NTP replies were GPS stratum 1, LI 0,
with matching origin timestamps and plausible UTC. Timezone America/Chicago,
IANA/boundaries 2026c. The new placement fields were valid. At the final sample:
window 100.767 us, signed midpoint offset +7.644 us, clock uncertainty 171.896 us.
This five-minute baseline does not establish that the overnight increase is
resolved. No serial diagnostics or debug logging were enabled.

## Interpretation

The active GPS/PPS uncertainty snapshot now carries:

- `pps_placement_valid`: a credible bracket matched the trusted reconstructed
  PPS timestamp using the existing nearest-bracket selection rule.
- `pps_observation_window_us`: time from the last observed low pin level to the
  first observed high level. A wide interval indicates weak edge-observation
  precision, not necessarily clock displacement.
- `pps_timestamp_offset_us`: reconstructed timestamp minus bracket midpoint.
  Positive means later; negative means earlier. The midpoint is only an estimate
  of the edge. Allow roughly half the window on either side when interpreting
  the signed offset (midpoint integer rounding is at most half a nanosecond).

For example, a 20 us window and a -2,450 us offset indicate timestamp displacement
well outside observation ambiguity. A 5,000 us window and near-zero midpoint
offset cannot establish such displacement. A time series after deployment can
distinguish accumulating displacement from changing polling delays.

The dashboard shows these values in **Timing diagnostics → PPS timestamp
placement**, with explicit unavailable states and sign explanation. Invalid
JSON numeric placeholders are zero, gated by `pps_placement_valid=false`.
RTC/coarse sources do not present old PPS evidence as active-source evidence.

## Safety and implementation

The existing placement-bound algorithm is preserved: the maximum distance to
the selected bracket endpoints, excluding implausible half-second associations.
The new detail helper returns that same bound and its matching width/offset.
The old bound helper delegates to it. PIO tick allowance, source-age growth,
margin, slew, conservative wire rounding and invalid-data behavior are unchanged.
The diagnostics travel with `ntp_uncertainty_t`, captured under the existing
stats lock; HTTP does not independently reread a newer edge and mix snapshots.
No PPS/PIO/DMA capture, UTC servo, RTC trim, or holdover policy was changed.

No new persistent buffer, heap allocation, debug log, runtime download, or
network endpoint was added. The 6,400-byte HTTP formatter cap and two-connection
bound remain unchanged. Health-format bounds are 5,762 bytes for L76K and 6,161
for UBX; HTML lengths are 13,457 and 14,105 bytes respectively, served from XIP.
UBX GCC frame reports: health formatter 2,504 bytes; NTP callback 344 bytes.
These individual frames are not a complete worst-case stack-depth measurement.

## Build handoff

| Variant | Flash/text bytes | BSS bytes | Data bytes |
| --- | ---: | ---: | ---: |
| L76K | 1,156,308 | 77,124 | 0 |
| UBX | 1,155,724 | 85,264 | 0 |

Both retain `RTC_32KHZ_GPIO=22` and disabled debug logging. BSS is unchanged from
the web-interface release. UF2 SHA-256:

```text
build-l76k-webfix/pico_gps_l76k.uf2
c93576e52186e2855990357ed5907557d57d89219d0b6fb593d87ed69867b88d

build-ubx-webfix/pico_gps_l76k.uf2
30107e95fc1f918ba3e11e799f370ec19e9bbe72d8bb1068966ce5b100166179
```

Timezone pre-build checks: upstream latest boundary release, embedded boundary,
host IANA, and embedded IANA all **2026c**, so generated files were not rewritten.
The geographic source remains `timezones-1970.geojson.zip`; supported years
**2020–2050**; **301 zones, 73,462 points, 1,307 rings, 6,819 transitions**.
Timezone call sites remain presentation-only, outside PPS capture, NTP timestamp
generation, RTC synchronization, holdover, leap processing and alpha-beta discipline.

Validation: both firmware builds; required timezone/GPS/UBX host tests with
warnings as errors; NTP uncertainty and RTC holdover normal/sanitized tests;
HTTP lifecycle and RTC reporting tests; both dashboard JS/size tests; desktop
and mobile Chrome rendering against local fixtures. Placement tests include
positive/negative/centered offsets, previous-edge association, missing/invalid
brackets, odd widths, and timestamps near UINT64_MAX. Existing uncertainty and
packet-encoding regressions pass. A flash and live time-series collection remain
separate user-authorized deployment steps.
