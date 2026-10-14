# Reviewed logic fixes — deployment handoff

All five findings from the code review are addressed:

1. Every RTC/EEPROM I2C operation now uses a 20 ms transfer deadline. Timeout
   failures trigger bounded bus recovery while holding the I2C mutex: release
   SDA, pulse SCL at most nine times, attempt STOP, and reinitialize the
   controller. GPIO recovery only drives low; high levels are released inputs.
   A permanently low SCL cannot trap recovery in a loop. Failed operations
   return failure rather than reporting successful calendar or trim writes.
2. Watchdog default is 8000 ms. Compile-time validation rejects zero and values
   above 8388 ms, the common supported range for RP2040 and RP2350.
3. Capture-fault recovery clears the trusted PPS timestamp under the statistics
   mutex. Subsequent source selection cannot continue serving GPS stratum 1
   from the revoked reference while PPS acquisition requalifies.
4. Earlier timestamp requests evaluate the latest continuity segment's rate
   without mutating the live clock model. NTP receive instants are no longer
   replaced with the UTC value of a newer update. For historical samples,
   uncertainty includes the maximum difference between opposing allowed slew
   rates, rather than the entire timestamp age.
5. Each leap-file DNS request carries an attempt identifier. Callbacks must
   match an active, pending attempt. Late successes/failures and duplicate
   callbacks cannot resurrect a cancelled fetch or alter a newer request.

## Validation

Passed required C11 `-Wall -Wextra -Werror` timezone, GPS-validation and UBX
host tests, and both required CMake builds.

Timing regression tests cover earlier NTP receive instants under both maximum
slew directions, unchanged live-model state after historical reads, monotonic
forward updates, RTC holdover, uncertainty and ordinary source selection.
The actual capture-recovery function is tested in both DMA and interrupt
configurations, including trusted-reference revocation.

Fault injection against the actual I2C and DNS functions covers timeout
propagation, recovery clock count, permanently stuck SCL, successful retries,
delayed DNS success/failure after cancellation, old callbacks during a newer
request, duplicate callbacks, immediate cached DNS results and normal failure.
Supported/unsupported watchdog settings are compile-tested. Normal and
ASan/UBSan runs pass.

PIO instruction simulation, missing/extra edge rejection, RTC reporting, NTP
uncertainty, HTTP lifecycle, dashboard bindings and real browser tests pass.
Health JSON bounds remain 5795/6400 bytes (L76K), 6194/6400 (UBX).

These are host/build validations. Electrical stuck-bus recovery and final
network behavior require hardware verification after flashing; no claim is
made that these fixes alone resolve the previously observed chrony loss.

## Timezone data and UTC audit

Latest timezone-boundary-builder release was checked at its upstream GitHub
API: **2026e**. The matching `timezones-1970.geojson.zip` was downloaded only
to the host workspace, the generator pin was updated and the exact project
regeneration command was run. IANA rules remain **2026c**, matching the host's
`/usr/share/zoneinfo/tzdata.zi`.

Generated coverage: **2020–2050**, **301 zones**, **73,640 points**,
**1,381 rings**, **6,819 transitions**. ELF symbols confirm arrays remain
read-only flash/XIP data. Timezone calls remain in zone refresh and presentation;
none are in PPS capture, NTP timestamp generation, RTC synchronization,
holdover, leap processing or alpha-beta discipline. No runtime timezone
network client was added.

## Artifacts

| Variant | UF2 | Flash text/data bytes | BSS bytes |
| --- | --- | ---: | ---: |
| L76K | `build-l76k-webfix/pico_gps_l76k.uf2` | 1,161,116 | 77,160 |
| UBX | `build-ubx-webfix/pico_gps_l76k.uf2` | 1,161,044 | 85,300 |

SHA-256:

```text
L76K 9dad1f92c37a20c325d080269bc8f7e50821a2b218ae1b9c2145a738b32a83d5
UBX  69bd3259f0c768e601a5f2b0425d6a187fd4e92922b7dcd1b527ae344b82ad9a
```

Both images retain 32 kHz input on GP22 and disabled debug logging. These
images have not been flashed. On deployment, identify the correct BOOTSEL
device, then verify health JSON, GPS/PPS lock, timezone boundary 2026e and
local zone, UDP NTP UTC/source/stratum, successful dashboard requests and
stable HTTP error counts. Monitor chrony reachability separately from the
appliance's internal lock and capture counters.

## UBX deployment — October 14, 2026

Flashed the UBX image above after confirming BOOTSEL serial
`50BABB25DD2BEAFE`. The running build reports `Oct 14 2026 13:06:27` at
`10.254.0.11`. The L76K was not flashed.

Startup monitoring through uptime 321 seconds verified valid health JSON,
GPS fix/PPS and LOCKED clock, LOCKED RTC, qualified 32 kHz with four accepted
blocks and no rejects, IANA 2026c/boundaries 2026e and America/Chicago.
Every completed dashboard request returned HTTP 200 with the 14,105-byte
page; HTTP errors stayed zero. UDP/123 replies validated packet length,
matching originate timestamp, server mode, normal leap status, GPS reference,
stratum 1 and plausible UTC timestamps. NTP send errors, DMA overruns and
Wi-Fi reconnects remained zero. Debug logging was not enabled.

The first endpoint probe timed out during startup; subsequent checks passed.
One PPS capture recovery was present in the first successful health sample
(uptime 48 seconds) and did not increase through the final sample. The precise
trigger is not recorded by the existing counter. Post-learning uncertainty
samples ranged approximately 135–222 us, with a final value of 216 us.
This verifies startup and service operation, not overnight endurance or
electrical fault injection on physical hardware.
