#!/bin/sh
set -eu
test_dir=$(mktemp -d /tmp/pico-pps-tests.XXXXXX)
trap 'rm -f "$test_dir/normal" "$test_dir/sanitized" "$test_dir/pps_capture.pio.h"; rmdir "$test_dir"' EXIT
# Assemble current source independently of the firmware build cache.
pioasm_bin=${PIOASM_BIN:-build-ubx-webfix/pioasm/pioasm}
if [ ! -x "$pioasm_bin" ]; then
    pioasm_bin=build-ubx-webfix/_deps/pioasm-build/pioasm
fi
"$pioasm_bin" -o c-sdk pps_capture.pio "$test_dir/pps_capture.pio.h"
cc -std=c11 -Wall -Wextra -Werror -O2 -I. -I"$test_dir" \
    tests/test_pps_capture.c -o "$test_dir/normal"
"$test_dir/normal"
cc -std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined \
    -I. -I"$test_dir" tests/test_pps_capture.c -o "$test_dir/sanitized"
"$test_dir/sanitized"
