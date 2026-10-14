# Web interface and HTTP memory refresh

## Result

Implemented and built for both L76K and UBX. The UBX image was subsequently
flashed on explicit request, after verifying BOOTSEL serial `50BABB25DD2BEAFE`.
Its SHA-256 matches the UBX artifact below; the live build reports
`Sep 8 2026 19:41:47`. The L76K was not flashed with this interface update.

Post-flash UBX checks: valid health JSON and the complete 13,389-byte new
dashboard; GPS/PPS and RTC locked; 32 kHz qualified by the sample at uptime
278 seconds with four accepted measurements and zero rejections; America/Chicago
and both timezone versions 2026c; direct UDP NTP replies with stratum 1, GPS
reference ID, LI 0 and matching origin timestamps. An additional 30 sequential
health/dashboard request pairs at two-second cadence all completed, with
`http_errors` remaining zero. No Wi-Fi reconnects or post-acquisition HTTP
resets were observed in this roughly five-minute startup check. This limited
window does not establish indefinite HTTP stability. No serial diagnostics or
debug logging were enabled.

The overview now shows served UTC, source/stratum, qualified UTC uncertainty,
GPS reception, local time, and RTC/holdover readiness. Timing and system
diagnostics are expandable. Both layouts were rendered and inspected in Chrome
at desktop and mobile widths, including 320 px. There are no external assets.
The browser snapshots stop updating in background tabs, avoid overlapping
requests, and back off from two seconds to thirty seconds after errors. Retained
values are visibly stale on failures, not still presented as live readings.

RTC SQW offset is explicitly software-compensated, separate from NTP UTC error.
The interface distinguishes absent/learning/qualified 32 kHz, frequency-trim
read validity, automatic/fixed trim, calibration status/history, and actual
calendar writes since boot. A calendar write is counted only after the complete
eight-byte I2C transaction succeeds; failed/partial writes are not counted and
the counter saturates. The legacy rephase API field remains for compatibility.
No RTC phase/frequency discipline or aging-adjustment policy was changed.

## HTTP changes and resource bounds

- The page stays in flash/XIP and is queued with no-copy TCP references.
  There is no full HTML allocation or copy per request.
- Each connection owns a 192-byte header and a bounded 160-byte request-line
  buffer. The target ARM connection structure is **388 bytes**.
- JSON is formatted into a connection-owned buffer of at most 6,400 bytes,
  then shrunk with lwIP `mem_trim` before transmission. The old permanent
  6,400-byte scratch buffer and response-concatenation allocation are gone.
- There are at most two active HTTP application connections: an absolute
  application payload/state allocation bound of **13,576 bytes** before
  allocator overhead. This is not a bound on all lwIP memory. TCP PCBs,
  segment/pbuf metadata, other protocols, and allocator overhead are additional.
  The existing 16 KiB lwIP heap and PCB/segment pools were not enlarged.
- Body/header storage remains alive until every queued byte is acknowledged.
  A half-closed request does not discard a pending response. `tcp_close` failure
  retains state and restores callbacks; successful close never accesses the PCB
  again. Temporary send-memory shortages retry from the one-second poll callback.
- The parser waits for the complete header terminator, even across packets,
  caps headers at 4 KiB, and starts only one response per connection. Additional
  input cannot allocate another response. Fifteen idle polls retire a stalled
  connection. Excess active connections are rejected intentionally.

The previous implementation processed the request line again on subsequent
receive callbacks and allocated another response, as well as copying every
page into heap memory. These are demonstrated code defects; they are **not a
proven exclusive cause** of the prior physical-device HTTP failures.

## Build evidence

| Variant | Flash/text before | Flash/text after | BSS before | BSS after |
| --- | ---: | ---: | ---: | ---: |
| L76K, 32 kHz enabled | 1,148,812 | 1,154,884 | 83,516 | 77,124 |
| UBX, 32 kHz enabled | 1,148,204 | 1,154,284 | 91,656 | 85,264 |

All sizes are bytes from `arm-none-eabi-size`; initialized RAM data is zero.
Static RAM falls by **6,392 bytes** in each variant. The richer page costs
roughly 6 KiB more flash, without increasing per-page response payload RAM.
HTML sizes: L76K **12,741 bytes**, UBX **13,389 bytes**.
Conservative health-JSON format bounds: **5,634 / 6,400 bytes** for L76K and
**6,033 / 6,400 bytes** for UBX. These cover shipped field ranges and strings,
not arbitrary future extensions.

GCC stack-use reports: health formatter 2,368 bytes (L76K), 2,488 (UBX);
HTTP receive and send-response callbacks 40 bytes each; poll 16 bytes.
These are individual function frames, not measured worst-case task stack depth.
The existing explicit 8 KiB network-core stack is unchanged.

Build configuration retained: L76K protocol 0 / UART 115200; UBX protocol 1 /
UART 38400; both `RTC_32KHZ_GPIO=22`, `DEBUG_LOG=0`, Pico 2 W, Release.

UF2 outputs and SHA-256:

```text
build-l76k-webfix/pico_gps_l76k.uf2
22e9ed633aa420825fbc0ca450d92b3da21d8ad398a587551647dfa6584e5524

build-ubx-webfix/pico_gps_l76k.uf2
b59a9f106db9132881069a91f34766dc203ebb16091363f52e009f9e085213eb
```

## Timezone and UTC invariants

The GitHub latest-release API was checked before compilation: boundary tag
**2026c**, matching the embedded boundary version. Host IANA tzdata and embedded
IANA data are also **2026c**. No regeneration or download was necessary.
The pinned source remains the
[1970 boundary archive](https://github.com/evansiroky/timezone-boundary-builder/releases/download/2026c/timezones-1970.geojson.zip),
with rules for **2020–2050**: **301 zones, 73,462 points, 1,307 rings, 6,819
transitions**. Generated data remains const in XIP.

Call-site audit: timezone conversion is confined to presentation refresh,
dashboard snapshots, and the display. There are no timezone calls in PPS
capture, NTP timestamp generation, RTC synchronization, holdover, leap processing,
or the alpha-beta discipline. No runtime timezone client/TLS dependency was added.

## Validation

Passed on the final implementation:

- Required C11 `-Wall -Wextra -Werror` timezone, GPS-validation and UBX tests.
  Timezone coverage includes geography, DST/offset transitions, non-hour offsets,
  ocean fallback, movement/border behavior and supported-year bounds.
- NTP uncertainty and RTC holdover tests, including sanitizer runs and optional
  32 kHz configurations.
- `tests/test_http_server.py`: actual firmware HTTP callbacks compiled against
  a fake lwIP with warnings as errors, normally and with AddressSanitizer /
  UndefinedBehaviorSanitizer. Covers byte-fragmented requests, trailing input,
  half-close, partial ACKs, all-bytes-queued-before-ACK, independent near-capacity
  snapshots, connection limit, allocation failure, send/output `ERR_MEM`, tiny
  send windows, close retry, oversized requests/headers, idle timeout and abort
  cleanup. The adversarial two-6,000-byte snapshot case peaks at 13,233 bytes of
  host application allocations (64-bit host state is 416 bytes). This excludes
  real TCP/radio allocations and is not a physical-device RAM measurement.
- `tests/test_rtc_reporting.py`: actual calibration-status expression with
  automatic trim enabled/disabled; six-hour threshold, stale GPS, uncertainty,
  trim interval, conversion states; actual RTC-write counter with complete,
  partial, failed and saturating writes.
- `tests/test_ntp_dashboard.py`: both actual embedded JavaScript variants,
  DOM bindings, qualified/unqualified/holdover states, 32 kHz states, offline /
  HTTP-error / bad-JSON handling, recovery, hidden-tab pause and bounded backoff.
- `tests/test_dashboard_browser.js`: real Chrome rendering of both embedded
  pages against local fixtures; desktop 1280 px, mobile 390 px and 320 px;
  expandable diagnostics, no horizontal overflow or clipped values; holdover,
  unqualified time, offline and recovery; zero external page requests.
- Both deployment firmware builds.

The browser harness prints its screenshot directory. `tools/render_dashboard.py`
can emit either exact embedded page for an independent preview. These tests do
not substitute for the post-flash health/GPS/PPS/timezone/NTP/HTTP checks required
by `AGENTS.md`. Flash only on a new explicit request, with the selected Pico
visibly in BOOTSEL.
