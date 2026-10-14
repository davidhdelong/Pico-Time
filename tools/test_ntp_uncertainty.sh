#!/bin/sh
set -eu
test_dir=$(mktemp -d /tmp/pico-ntp-tests.XXXXXX)
trap 'rm -f "$test_dir/normal" "$test_dir/sanitized"; rmdir "$test_dir"' EXIT
cc -std=c11 -Wall -Wextra -Werror -I. tests/test_ntp_uncertainty.c \
  ntp_uncertainty.c rtc_holdover.c -lm -o "$test_dir/normal"
"$test_dir/normal"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -I. \
  tests/test_ntp_uncertainty.c ntp_uncertainty.c rtc_holdover.c -lm \
  -o "$test_dir/sanitized"
"$test_dir/sanitized"
