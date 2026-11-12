# Project agent instructions

## GitHub push preference

After completing a project update that changes tracked source, tests,
configuration, generated data, or documentation, finish the authorized work
and appropriate validation, summarize the result, and offer to commit and push
the changes to `https://github.com/davidhdelong/Pico-Time.git`.
Ask once per completed update, not after each edit or read-only status check.
If the user already requested a push for that update, complete it without
asking again and report the resulting commit. Otherwise, wait for the user's
acceptance before committing and pushing. Keep credentials, private build
settings, backups, and build artifacts excluded from Git.

## Firmware and timezone-data build policy

This project is an offline Raspberry Pi Pico 2 W time appliance. GNSS, PPS,
the disciplined UTC clock, NTP, RTC, holdover, leap-second handling, and the
alpha-beta servo must remain UTC. Timezone conversion is presentation-only.
Never add a runtime timezone download, update check, HTTP mirror, HTTPS client,
or TLS dependency to the firmware.

Every time a firmware build is requested, check whether the embedded timezone
database is current before compiling:

1. Read the versions in `generated/timezone_data.h` (`TZDB_VERSION` and
   `TZ_BOUNDARY_VERSION`).
2. Query the upstream timezone-boundary-builder releases at:
   `https://api.github.com/repos/evansiroky/timezone-boundary-builder/releases/latest`.
   Use the latest release's `tag_name` as the boundary version. The matching
   `timezones-1970.geojson.zip` asset is the geographic source.
3. Compare that release with the embedded boundary version. Also compare the
   local IANA version from `/usr/share/zoneinfo/tzdata.zi` when available. A
   newer IANA version requires regeneration even if the boundary release did
   not change.
4. If either source is newer, download the matching boundary archive only to
   the host build workspace, update the generator's pinned source/version,
   regenerate the const data, run the timezone tests, and rebuild both the
   L76K and UBX firmware variants. Keep the generated data in flash/XIP; do not
   copy the database wholesale into RAM.
5. If the data is unchanged, build normally without rewriting generated files.
6. Report the source versions, supported year range, generated zone/point/
   transition counts, flash/BSS sizes, and test results in the build handoff.

The exact regeneration command is:

```sh
python3 tools/timezone_generator/generate.py
```

The generator currently uses the 1970 geographic boundary asset and emits
rules for 2020–2050. Do not silently shorten that range or switch to a
current-only boundary asset because it can remove valid geographic zones.

## Device update policy

When regenerated timezone data changes the firmware image, the updated UF2 is
the device update. Use the correct build output:

- L76K device: `build-l76k-webfix/pico_gps_l76k.uf2`, normally `10.254.0.10`
- UBX device: `build-ubx-webfix/pico_gps_l76k.uf2`, normally `10.254.0.11`

Flash only the device/variant in scope and only when the user has requested a
firmware flash. The Pico must be visibly in BOOTSEL mode before copying the
UF2. If it is running normally, report that it needs BOOTSEL instead of
attempting destructive USB or filesystem operations.

After flashing, wait for the Pico to reboot and verify all of the following:

- HTTP `/api/health` returns valid JSON.
- GPS fix and PPS are present; the clock is `LOCKED` when acquisition has
  completed.
- The dashboard reports the expected timezone database version and local zone.
- NTP replies on UDP/123 with the expected UTC source/stratum.
- HTTP dashboard requests complete successfully and `http_errors` does not
  increase.

Debug logging must not be enabled merely to perform this check. If debugging
is already disabled, use the network health endpoint and NTP checks; use the
serial console only when the user asks for serial diagnostics.

## Required validation

Before claiming a build is complete, run the host tests with warnings treated
as errors, including:

```sh
cc -std=c11 -Wall -Wextra -Werror -I. \
  tests/test_timezone.c timezone_lookup.c generated/timezone_data.c \
  -o /tmp/test_timezone && /tmp/test_timezone
cc -std=c11 -Wall -Wextra -Werror -I. \
  tests/test_gps_validation.c -o /tmp/test_gps_validation && /tmp/test_gps_validation
cc -std=c11 -Wall -Wextra -Werror -I. \
  tests/test_ubx.c ubx.c -o /tmp/test_ubx && /tmp/test_ubx
cmake --build build-l76k-webfix -j2
cmake --build build-ubx-webfix -j2
```

The timezone tests must cover geographic selection, UTC offset/DST changes,
non-hour offsets, ocean fallback, and representative movement/border behavior.
Audit timezone call sites before release and confirm none are in PPS capture,
NTP timestamp generation, RTC synchronization, holdover, leap processing, or
alpha-beta discipline.
