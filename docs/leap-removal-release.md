# Firmware 0.5.0 — leap subsystem removal

Removed the leap-file DNS/HTTP client, refresh/retry logic, metadata parser,
EEPROM cache/migration code, smear adjustments and scheduled-leap lookup.
Removed their CMake options, health snapshot fields, HTTP/UDP `leap_*` fields,
and the dashboard's Leap Data panel. Existing EEPROM contents are ignored;
this release does not erase or write that storage.

GNSS UTC decoding, the fixed NTP epoch offset, Gregorian leap-year handling,
PPS capture/discipline, RTC aging calibration, 32 kHz rate observation,
holdover and UTC continuity remain. NTP keeps standard LI bits: normal
synchronized GPS/RTC replies use LI=0; unavailable or unqualified time uses
LI=3 and stratum 16. There is no application-generated future leap
announcement or smear. API clients relying on removed `leap_*` fields must
update for 0.5.0.

This change does not promise lower timing uncertainty: the PPS placement and
uncertainty functions are byte-for-byte unchanged. About 8 KiB of static RAM
is released by removal of the fetch buffer and leap state.

## Verification

- Required timezone, GPS validation and UBX host tests passed with C11
  `-Wall -Wextra -Werror`; both firmware CMake builds passed.
- New `python3 tests/test_ntp_service.py` extracts the actual UTC-to-NTP
  conversion and UDP callback. Normal and ASan/UBSan runs verify exact UTC
  seconds/fractions across date boundaries, reference/originate/receive/transmit
  timestamps, GPS stratum 1, RTC stratum 2, unqualified/invalid stratum 16,
  LI=0/3, unavailable PPS placement, malformed requests and allocation failure.
- RTC holdover/continuity, I2C fault recovery/watchdog, NTP uncertainty,
  assembled PIO simulation, capture recovery and PPS timeline tests passed,
  including the existing sanitizer configurations. RTC reporting tests passed.
- HTTP lifecycle and dashboard binding tests passed for both variants.
  Conservative health-format bounds are 5575/6400 bytes for L76K and
  5974/6400 for UBX.
- Real headless browser tests passed desktop/mobile layout and
  holdover/unqualified/offline/recovery states for both variants, with zero
  external dashboard requests.
- Both variants passed SDK syntax/format checks with optional UDP health
  broadcast and CRC enabled. Broadcast CRC implementation was retained.
- Source audit confirms no application leap downloader, cache, smear, DNS
  lookup or outbound TCP connection remains. Linked symbols contain no
  leap/cache/smear functions. PPS timestamp helpers and uncertainty/source
  calculations were compared against the original and preserved exactly.

These checks verify host behavior and compilation. Hardware startup results
for both corrected images are recorded below. Longer-term operation and
network timing still need monitoring.

## Timezone/build handoff

The latest timezone-boundary-builder GitHub release is **2026e**, matching
the existing pinned 1970 boundary archive. Host IANA data advanced from
2026c to **2026e**, so the generator was rerun with its full **2020–2050**
range. Generated data now contains **301 zones, 73,640 points, 1,381 rings,
6,723 transitions**. ELF symbols confirm read-only flash/XIP placement.
Timezone conversion remains in presentation and zone refresh, outside PPS,
NTP timestamp generation, RTC synchronization, holdover and servo discipline.

| Variant | UF2 | Flash text/data bytes | BSS bytes |
| --- | --- | ---: | ---: |
| L76K | `build-l76k-webfix/pico_gps_l76k.uf2` | 1,151,884 | 68,888 |
| UBX | `build-ubx-webfix/pico_gps_l76k.uf2` | 1,121,224 | 76,976 |

Both retain 32 kHz on GP22 and disabled debug logging.

SHA-256:

```text
L76K df5723847514b0774d19e8db9bd9ab744048fec7604bf617325922f10a0dea04
UBX  b5def3f156e024736ff42a6976fbab11297ab66d7f3ea530bdeed0e085c6d90f
```

After flashing, verify health JSON/version, GPS/PPS/RTC lock and 32 kHz
qualification, timezone versions 2026e and correct local zone, valid NTP UTC
replies and LI/stratum, dashboard requests and stable error counters. Monitor
chrony reachability independently from internal appliance clock lock.

## Deployment configuration correction

The first attempted UBX flash used a 0.5.0 image built with placeholder Wi-Fi
defaults, and it did not become reachable. The source defaults had been
sanitized for GitHub, but private settings were absent from the build cache.
Both images above were rebuilt with the saved private Wi-Fi overrides in
ignored build directories. No credential values were put in tracked source.
CMake now rejects a configuration missing either required Wi-Fi setting;
this failure check and successful configurations for both variants passed.
The corrected UBX image was subsequently flashed and checked as described
below. The preceding host test results remain applicable to the
unchanged timing/application code.

## Corrected UBX deployment — October 30, 2026

Confirmed BOOTSEL serial `50BABB25DD2BEAFE`, verified the corrected UBX
checksum above and copied that UF2. The device joined at `10.254.0.11` with
version **0.5.0**, build **Oct 30 2026 10:30:35**. Monitoring continued through
uptime 313 seconds. The first probe ran before the network was reachable;
all ten subsequent health/dashboard/NTP check cycles passed.

GPS fix/PPS and RTC were LOCKED; the final sample used 20 satellites. The
32 kHz input qualified with four accepted blocks and zero rejects. Both
timezone versions were 2026e and local zone America/Chicago. Health JSON had
no `leap_*` fields, and the complete 13,805-byte dashboard returned HTTP 200
without a Leap Data panel.

UDP/123 replies passed length, server-mode, originate-echo, nondecreasing
receive/transmit timestamp, UTC plausibility, LI=0, stratum-1 and GPS reference
checks. Internal uncertainty after learning was approximately 137–165 us;
the final sample was 164.575 us with placement offset +33.577 us.
Capture faults, DMA overruns, HTTP errors, NTP send errors and Wi-Fi
reconnects stayed zero. Debug logging was not enabled.

Network RTT ranged approximately 8–219 ms while server processing time was
approximately 0.24–0.27 ms. Chrony remained on Google and had no fresh UBX
sample at the time checked, so this startup verification does not establish
that the historical chrony latency/reachability problem is resolved.

## Corrected L76K deployment — October 30, 2026

Confirmed BOOTSEL serial `44E15817A454A436`, verified the corrected L76K
checksum above and copied that UF2. The running device at `10.254.0.10`
reports **0.5.0**, build **Oct 30 2026 10:32:29**. The UBX was not flashed
during this operation.

Monitoring continued through uptime 339 seconds. The first HTTP probe timed
out during startup; all eleven subsequent health/dashboard/NTP cycles passed.
GPS fix/PPS and RTC were LOCKED, with 12 satellites in the final sample.
The 32 kHz input qualified with five accepted blocks and zero rejects.
Both timezone versions were 2026e and the local zone was America/Chicago.
Health JSON had no `leap_*` fields, and the complete 13,157-byte dashboard
returned HTTP 200 without a Leap Data panel.

NTP replies passed packet length, server mode, matching originate timestamp,
nondecreasing RX/TX timestamps, UTC plausibility, LI=0, GPS reference and
stratum-1 checks. After initial acquisition, reported uncertainty ranged
approximately 162–281 us; the final sample was 162.053 us with placement
offset +21.367 us. Capture faults, DMA overruns, HTTP errors, NTP send errors
and Wi-Fi reconnects stayed zero. Debug logging was not enabled.

Observed network RTT was approximately 11–85 ms, and server processing time
was approximately 0.24–0.30 ms. These startup checks do not independently
measure absolute UTC accuracy or establish long-term chrony performance.
