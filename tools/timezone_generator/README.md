# Offline timezone data generator

This generator consumes the [timezone-boundary-builder](https://github.com/evansiroky/timezone-boundary-builder)
`timezones-1970.geojson.zip` release and the host IANA tzdb exposed through
Python `zoneinfo`. The checked build uses release `2026f`, tzdb `2026e`, and
generates rules for 2020–2050. Boundary data is derived from OpenStreetMap and
Natural Earth inputs; see the upstream project for complete licensing and
attribution. IANA tzdb is public-domain data.

Run from the project root:

```sh
python3 tools/timezone_generator/generate.py
```

The command downloads the boundary archive only when absent, then emits
`generated/timezone_data.h` and `generated/timezone_data.c`. The Pico image
contains only those generated const arrays; it does not contain JSON, a TLS
client, an HTTP timezone updater, or any runtime network dependency.

The embedded geographic representation uses simplified polygon rings with
per-ring latitude/longitude bounding boxes. This was selected over a coarse
grid so islands and small territories remain representable without storing an
IANA string in each cell. Lookup is infrequent and cached; a host benchmark on
the generated data measured approximately 994 ns for a Chicago polygon lookup
and 19 ns for a rule lookup on the build host (the polygon lookup is not in a
PPS or NTP path).
