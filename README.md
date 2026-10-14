# L76K GPS (Waveshare Pico) reader for Pico 2 W

Reads NMEA sentences from the Waveshare L76K GPS module over UART and prints
parsed GPS stats (fix, satellites, HDOP, altitude, lat/lon, speed, time/date)
to the Pico's USB serial (debug) port. The Pico 2 W connects to WiFi via DHCP
and can drive a TM1637 4-digit display with UTC time. The HTTP dashboard is
enabled by default; disable it with `-DHTTP_SERVER_ENABLE=0`.
WiFi/SNTP runs on core 1 while GPS/RTC handling runs on core 0.
The onboard LED blinks at 1 Hz when GPS PPS is active, is solid during RTC
fallback, and is off when no synchronized time source is available.

## Wiring (default)

- GPS TX -> Pico GP1 (UART0 RX)
- GPS RX -> Pico GP0 (UART0 TX)
- GPS PPS -> Pico GP16 (optional, for PPS discipline)
- RTC SQW/1Hz -> Pico GP17 (optional, for RTC holdover discipline)
- 3V3 -> 3V3, GND -> GND

The Waveshare module defaults to UART0 on GP0/GP1 at 9600 baud. On boot, the
firmware configures it at `GPS_UART_BOOT_BAUD` (9600 by default), then changes
it to `GPS_UART_BAUD` (115200 by default).
The firmware uses the GPS NMEA UTC fields directly by default
(`GPS_NMEA_TIME_OFFSET_SEC=0`) for PPS epoch alignment and RTC discipline.

## Optional: u-blox UBX input

For a u-blox F10N, build the UBX variant with:

```
cmake .. -DPICO_BOARD=pico2_w \
  -DGPS_PROTOCOL_UBX=1 -DGPS_CONFIGURE_MODULE=0 -DGPS_UART_BAUD=9600
```

Configure the receiver to output `UBX-NAV-PVT` and `UBX-TIM-TP` at 1 Hz,
configure the TIMEPULSE output for 1 Hz aligned to UTC, and wire TIMEPULSE to
the configured PPS GPIO. UBX-NAV-PVT supplies validated UTC/navigation data;
UBX-TIM-TP is checksum-validated and decoded for time-pulse metadata; the
physical PPS edge remains the high-precision timing reference. The receiver
must be configured with u-center or its UBX configuration protocol because
the L76K-specific PCAS startup commands are disabled in UBX mode.
The UBX dashboard reports PDOP; NAV-PVT does not provide an HDOP value.

If PPS is wired, it is used to discipline the time between NMEA updates
and keep GPS as the active time source when PPS is present. The RTC is
only adjusted on the normal sync interval (not every PPS tick), so it
slews toward the PPS-disciplined time over time. Some PPS pins are
open-drain; add a pull-up to 3V3 if needed.

PPS is captured by a PIO state machine with a free-running hardware counter;
the captured counter value is transferred through the PIO FIFO, so PPS
intervals are not dependent on variable CPU interrupt latency. The resulting
interval is filtered by the PPS PLL and used for the PPS-disciplined software
timebase and NTP responses.
If RMC time is too far from the PPS edge, it is ignored for epoch updates.
The RMC window tightens dynamically as PPS/RMC alignment stabilizes.
The signed RMC-to-PPS latency is learned during bootstrap, and subsequent
samples are checked against the learned residual so a consistent UART/NMEA
delay is accepted without widening the rejection window. You can
tune the bootstrap and residual limits with `RMC_PPS_BOOTSTRAP_MAX_OFFSET_US`,
`RMC_PPS_MAX_OFFSET_US`, and `RMC_PPS_MIN_OFFSET_US`.
The PPS interval is filtered with a simple PLL-style filter; you can tune
`PPS_PLL_FILTER_SHIFT`, `PPS_PLL_MIN_US`, and `PPS_PLL_MAX_US` at build time.

The DS3231 SQW phase is learned against qualified GNSS PPS, robustly filtered,
and compensated in software during holdover. A constant phase offset does not
trigger an RTC write. Qualification requires at least 60 PPS ticks and 32
accepted RTC phase samples with at most 1 ms jitter by default. The discovery
and absolute offset limit is 450 ms; stability, not proximity alone, determines
trust. Defaults are configurable in `rtc_holdover.h` and through CMake.

A bounded phase-slope regression supplies RTC oscillator frequency in ppm.
Aging calibration needs six hours of stable observations and a sufficiently
precise estimate, independent of calendar offset. The nominal sensitivity is
0.1 ppm/LSB (8.64 ms/day), not 0.1 seconds/day. Positive register changes slow
the oscillator. Manual temperature conversion applies the adjustment, with
nonblocking completion checks. `RTC_AGING_INVERT` remains an explicit override.

Holdover uncertainty grows with age. The default policy stops synchronized NTP
at 100 ms estimated error or 24 hours, while retaining the time estimate.
GNSS recovery requires four accepted consecutive PPS observations; normal
transitions slew at up to 500 ppm without stepping served UTC backward.
The dashboard and `/api/health` expose phase, jitter, frequency, uncertainty,
aging, temperature, startup OSF, source state, and optional 32 kHz diagnostics.
See [implementation and validation details](docs/rtc-holdover-implementation.md).

RMC time acceptance can be tightened with:
`RMC_MIN_FIX_QUALITY`, `RMC_MAX_HDOP`, and `RMC_MIN_SATS`.

## TM1637 UTC display

A TM1637 4-digit display shows synchronized UTC as `HH:MM`; it shows `00:00` until GPS/PPS or trusted RTC-SQW synchronization is available. The default wiring is `CLK -> GP14`, `DIO -> GP15`, `VCC -> 3V3`, and `GND -> GND`. The display is powered at the configured brightness and shows `00:00` until time is synchronized. Change the pins or disable it at build time with `-DTM1637_CLK_PIN=...`, `-DTM1637_DIO_PIN=...`, or `-DTM1637_ENABLE=0`.

## DS3231SN RTC wiring (default)

- RTC SDA -> Pico GP4 (I2C0 SDA)
- RTC SCL -> Pico GP5 (I2C0 SCL)
- RTC SQW/1Hz -> Pico GP17 (optional, recommended for low-jitter holdover)
- 3V3 -> 3V3, GND -> GND

The RTC calendar always contains UTC. Startup preserves the oscillator-stop
flag; a stopped or unreadable RTC needs a valid GNSS calendar set before trust.
Qualified SQW takes over when the last trusted PPS is 1.5 seconds old. Without
qualified SQW, the RTC calendar is a coarse, unsynchronized fallback. RTC SQW
freshness expires after 2.5 seconds; GNSS message freshness separately expires
after 10 seconds. Reboot requires fresh SQW qualification even with a valid RTC.

The optional DS3231 32.768 kHz output can be connected to a user-selected free
GPIO and enabled with `-DRTC_32KHZ_GPIO=<gpio>`. There is no default pin; `-1`
(the default) disables the feature. This open-drain output needs an appropriate
external pull-up to **3.3 V**. Do not connect a module pull-up to 5 V. A PIO1
state machine counts rising edges and interrupts only once per 32768 cycles.
It supplies an additional frequency reference, not 30.5 us timestamp resolution.
SQW remains required for qualified holdover. With an absent 32 kHz signal,
normal SQW-only operation continues.

The deployment builds (`build-ubx-webfix` and `build-l76k-webfix`) enable this input with
`RTC_32KHZ_GPIO=22`: connect DS3231 32kHz to Pico GP22 (physical pin 29),
keeping SQW on GP17. To recreate the setting in an already configured deployment
build, run `cmake -S . -B build-ubx-webfix -DRTC_32KHZ_GPIO=22` or
`cmake -S . -B build-l76k-webfix -DRTC_32KHZ_GPIO=22` before building.

Run the deterministic timing tests with `sh tools/test_rtc_holdover.sh`.

NTP root dispersion uses a source-specific uncertainty model, not elapsed PPS
age divided by four. `/api/health` and the NTP uncertainty dashboard section
show the capture/handoff, interval residual, frequency growth, margin and
output-mapping contributions, along with the rounded wire value. PPS absolute
placement is bounded by reporting-only low/high pin observations; these never
change PIO/DMA timestamps or servo corrections. If a narrow pulse or processing
stall prevents a usable bracket, NTP reports unqualified time rather than
claiming a small error from interval jitter alone.

Run `sh tools/test_ntp_uncertainty.sh` and
`python3 tests/test_ntp_dashboard.py` (the latter requires Node.js) for the
encoding, uncertainty, transition, packet-header and dashboard regressions.
See [the root-dispersion report](docs/ntp-dispersion-fix.md) for the original
cause, model assumptions, diagnostics, verification and deployment status.

## Automatic offline timezone display

The firmware keeps GNSS/PPS, the disciplined clock, NTP, RTC, holdover, and
leap-second processing in UTC. The display and HTTP dashboard may additionally
show local civil time. By default, a valid GNSS position selects the IANA zone
from static generated geographic boundaries (`TIMEZONE_MODE=AUTO`), with three
consecutive candidate results required before changing zones. Lookup is
repeated after approximately 15 km of movement. A no-polygon position safely
falls back to UTC. Compile-time alternatives are `-DTIMEZONE_MODE=\"UTC\"` or
`-DTIMEZONE_MODE=\"MANUAL\" -DTIMEZONE_MANUAL_NAME=\"America/Chicago\"`.

Regenerate the embedded data with:

```
python3 tools/timezone_generator/generate.py
```

The generated database covers 2020–2050 and is entirely offline at runtime.

## Build

```
export PICO_SDK_PATH=/path/to/pico-sdk
mkdir build
cd build
cmake .. -DPICO_BOARD=pico2_w
make -j4
```

Copy `pico_gps_l76k.uf2` to the Pico 2 W.

## Optional: use UART1

If your hardware is wired to UART1 (GP4/GP5), build with:

```
cmake .. -DPICO_BOARD=pico2_w \
  -DGPS_UART_ID=uart1 -DGPS_UART_TX_PIN=4 -DGPS_UART_RX_PIN=5
```

## Optional: change PPS pin

```
cmake .. -DPICO_BOARD=pico2_w -DGPS_PPS_PIN=16
```

## Optional: change GPS NMEA time offset

If your module needs a fixed NMEA-to-UTC correction, build with:

```
cmake .. -DPICO_BOARD=pico2_w -DGPS_NMEA_TIME_OFFSET_SEC=1
```

## Optional: change RTC SQW pin

```
cmake .. -DPICO_BOARD=pico2_w -DRTC_SQW_PIN=17
```

All compile-time settings defined near the top of `main.c` can be overridden
with `-DNAME=value` during CMake configuration. This includes GPS quality
freshness (`GPS_QUALITY_MAX_AGE_MS`, default 3000 ms), RTC SQW thresholds,
PPS filtering, accuracy-mode thresholds, EEPROM caching, and retry limits.

## Optional: use I2C1 for RTC

If your RTC is on I2C1 (GP2/GP3), build with:

```
cmake .. -DPICO_BOARD=pico2_w \
  -DRTC_I2C_ID=i2c1 -DRTC_SDA_PIN=2 -DRTC_SCL_PIN=3
```

## WiFi config

Set your SSID/password at build time:

```
cmake .. -DPICO_BOARD=pico2_w \
  -DWIFI_SSID=\"YourSSID\" -DWIFI_PASSWORD=\"YourPassword\"
```

Set a DHCP hostname (default is `ntp1`):

```
cmake .. -DPICO_BOARD=pico2_w -DWIFI_HOSTNAME=\"ntp1\"
```

Disable WiFi power save for steadier timing (disabled by default):

```
cmake .. -DPICO_BOARD=pico2_w -DWIFI_NO_POWERSAVE=1
```

The firmware connects via DHCP and prints the IP address to the USB serial
port.

A hardware watchdog is enabled by default and can be tuned with
`WATCHDOG_TIMEOUT_MS` (default 8000 ms; accepted range 1–8388 ms, which fits
both RP2040 and RP2350). RTC/EEPROM I2C transfers have a 20000 us deadline
(`RTC_I2C_TIMEOUT_US`) and bounded open-drain bus recovery after a timeout.

The HTTP dashboard and SNTP server have no authentication and are intended for
a trusted local network. Disable the dashboard with `-DHTTP_SERVER_ENABLE=0`
when it is not needed.

## Debug output

Debug prints are disabled by default. Enable them with:

```
cmake .. -DPICO_BOARD=pico2_w -DDEBUG_LOG=1
```

## Host tests

The hardware-independent GPS validation tests can be run with:

```
cc -std=c11 -Wall -Wextra -I. tests/test_gps_validation.c -o /tmp/test_gps_validation
/tmp/test_gps_validation
cc -std=c11 -Wall -Wextra -I. tests/test_ubx.c ubx.c -o /tmp/test_ubx
/tmp/test_ubx
```

## SNTP server

The Pico serves SNTP on UDP port 123. Point clients at the Pico's IP to sync
time from GPS/RTC.

When PPS is available, SNTP replies include sub-second fractional time based
on PPS edge timestamps for higher accuracy.

## Web dashboard

The Pico serves a self-contained live dashboard over HTTP on port 80 by
default. Open `http://<pico-ip>/` in a browser. Disable it with
`-DHTTP_SERVER_ENABLE=0` when it is not needed.
The raw JSON endpoint is available at `http://<pico-ip>/api/health` and
refreshes without browser caching.

The overview prioritizes served UTC, clock uncertainty, GPS reception, local
time, and RTC/holdover readiness. Timing diagnostics and network/timezone/system
details expand on demand. SQW offset is explicitly identified as a
software-compensated offset, not an NTP error. The old rephase counter remains
in the JSON for compatibility; the dashboard instead shows actual successful
RTC calendar writes since boot. Calibration status/history, trim read validity,
and absent/learning/qualified 32 kHz states are reported separately.

Snapshots are fetched every two seconds after the previous request completes.
There is at most one dashboard fetch in flight per tab. Background tabs stop
polling; failed requests mark retained readings stale and back off to 30 seconds.
The displayed UTC is a device snapshot, not a browser-interpolated precision clock.
No external CSS, JavaScript, fonts, images, or services are used.

Timing diagnostics also expose PPS timestamp placement: `pps_placement_valid`,
`pps_observation_window_us`, and `pps_timestamp_offset_us`. The window spans the
observed low-to-high GPIO transition. Signed offset is reconstructed timestamp
minus the window midpoint (positive means later). The midpoint is not an exact
edge measurement. These fields share the uncertainty snapshot and are unavailable
for non-PPS sources or missing/invalid brackets. They do not feed the servo or
reduce the conservative NTP dispersion. See `docs/pps-placement-diagnostics.md`.

HTTP has at most two active application connections and retires clients after
15 seconds without progress. Immutable HTML is transmitted directly from flash
without a full-page heap copy. A health response owns its snapshot until all TCP
bytes are acknowledged; unused formatter capacity is returned with `mem_trim`.
Header parsing is bounded to 4 KiB, stores only a 160-byte request-line buffer,
and handles fragmented headers without generating duplicate responses. Temporary
send-memory shortages and close failures are retried on the TCP poll callback.
Extra connections are rejected rather than consuming unbounded network memory.
UDP health broadcast remains disabled by default.

Host UI/HTTP checks (from the repository root):

```sh
python3 tests/test_ntp_dashboard.py
python3 tests/test_http_server.py
python3 tests/test_rtc_reporting.py
node tests/test_dashboard_browser.js
```

The browser test uses local fixture data, Chrome (`CHROME_BIN` can override its
path), and Node's `ws` package. It checks both variants at desktop/mobile widths
and saves screenshots to a temporary directory. It does not flash a device or
claim to test Wi-Fi behavior. See `docs/web-interface-refresh.md` for the build
and memory handoff.

## Optional health broadcast

The legacy JSON broadcast can be enabled explicitly if needed. Default port: 4040.

```
cmake .. -DPICO_BOARD=pico2_w \
  -DHEALTH_BROADCAST_ENABLE=1 -DHEALTH_BROADCAST_PORT=4040
```

CRC can be toggled with:

```
cmake .. -DPICO_BOARD=pico2_w -DHEALTH_BROADCAST_CRC=1
```

## Leap second file

If the DS3231 module includes the `AT24C32` EEPROM, the firmware caches parsed
leap-second metadata there. On boot it loads the cached data from EEPROM and
only fetches the IANA leap list from the network if the cache is missing,
invalid, or expired. EEPROM is only rewritten when the parsed leap metadata
actually changes. The health JSON includes:
`leap_loaded`, `leap_source`, `leap_valid`, `leap_expire_utc`, `leap_next_utc`,
and `leap_policy`. The policy is `24h-smear`: SNTP timestamps and the UTC
display use a linear 24-hour adjustment before the listed leap. GPS NMEA time
and the RTC remain UTC inputs and are not modified at the hardware boundary.
This keeps the leap behavior explicit: network-facing time and the display
follow the smear, while raw GPS/RTC readings remain available for diagnostics.

PPS jitter stats are reported as:
`pps_jitter_us`, `pps_jitter_min_us`, `pps_jitter_max_us`,
`pps_jitter_rms_us`, plus nanosecond-resolution
`pps_jitter_ns`, `pps_jitter_min_ns`, `pps_jitter_max_ns`, and
`pps_jitter_rms_ns`. The nanosecond values reflect the PPS capture
counter resolution; the UBX build uses a 50 MHz counter (20 ns ticks).

Leap smear status is reported as:
`leap_smear` and `leap_smear_remaining_s`.

Additional health fields:
`uptime_s`, `wifi_link`, `wifi_rssi`, `ip`, `rmc_age_ms`, `rtc_offset_s`,
`rtc_offset_us`, `rtc_sqw`, `rtc_sqw_time`, `rtc_sqw_age_ms`,
`rtc_sqw_phase_us`, `rtc_sqw_rephase_count`, `rmc_pps_offset_us`,
`rmc_pps_window_us`, `accuracy_mode`, `pll_filter_shift`, `clock_state`,
`rtc_status`, `rtc_state`, `rtc_trust`.

`rtc_trust` reports `PRESENT`, `TRACKING`, `TRUSTED_HOLDOVER`, `MISSING`, or
`FAULT`, so RTC availability is distinct from whether its SQW-derived time is
trusted.

When RTC SQW timing is valid, `rtc_offset_s` is derived from the SQW-tracked RTC
second edge instead of a raw RTC register read, and `rtc_offset_us` reflects the
measured SQW phase against GPS PPS. If SQW timing is not valid, the offset falls
back to the coarse RTC register comparison.

When the broadcaster falls back to compact/diagnostic mode, it may also include:
`heap_*`, `lwip_*`, `rtc_offset_us_valid`, `rmc_phase_ok_count`,
`rmc_pps_window_us`, `rtc_track_good`, `rtc_track_bad`,
`health_fmt_drop_count`, `health_alloc_fail_count`, `health_send_err_count`,
`health_rtc_busy_count`, and `health_compact`.

Disable fetching with:

```
cmake .. -DPICO_BOARD=pico2_w -DLEAPFILE_ENABLE=0
```
