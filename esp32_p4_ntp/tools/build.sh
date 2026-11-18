#!/usr/bin/env bash
set -euo pipefail
receiver=${1:-l76k}
silicon=${2:-rev1}
case "$receiver" in l76k) protocol=0;; ubx) protocol=1;; *) echo 'Receiver must be l76k or ubx' >&2; exit 2;; esac
case "$silicon" in rev1|rev3) ;; *) echo 'Silicon must be rev1 (0.x/1.x) or rev3 (3.x)' >&2; exit 2;; esac
: "${IDF_PATH:?Activate ESP-IDF 6.0.x with its export.sh first}"
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
build_dir="$project_root/build-p4-$receiver-$silicon"
mkdir -p "$build_dir"
idf.py -C "$project_root/esp32_p4_ntp" -B "$build_dir" \
    -DIDF_TARGET=esp32p4 -DP4_GNSS_UBX="$protocol" -DSDKCONFIG="$build_dir/sdkconfig" \
    "-DSDKCONFIG_DEFAULTS=$project_root/esp32_p4_ntp/sdkconfig.defaults;$project_root/esp32_p4_ntp/sdkconfig.$silicon" build
python "$project_root/esp32_p4_ntp/tools/package.py" "$build_dir"
