#!/usr/bin/env bash
set -euo pipefail
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$project_root"
test_dir=$(mktemp -d /tmp/p4-ntp-test.XXXXXX)
trap 'rm -rf "$test_dir"' EXIT
for mode in normal sanitized; do
    flags=()
    if [ "$mode" = sanitized ]; then flags=(-fsanitize=address,undefined,float-cast-overflow -fno-sanitize-recover=all); fi
    cc -std=c11 -Wall -Wextra -Werror -I. -Iesp32_p4_ntp/main "${flags[@]}" \
        esp32_p4_ntp/tests/test_p4.c esp32_p4_ntp/main/p4_clock.c esp32_p4_ntp/main/gnss.c \
        ubx.c rtc_holdover.c ntp_uncertainty.c -lm -o "$test_dir/test"
    "$test_dir/test"
done
