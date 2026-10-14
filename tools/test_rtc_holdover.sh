#!/bin/sh
set -eu
test_dir=$(mktemp -d /tmp/pico-rtc-tests.XXXXXX)
trap 'rm -f "$test_dir/disabled" "$test_dir/enabled" "$test_dir/sanitized"; rmdir "$test_dir"' EXIT
cc -std=c11 -Wall -Wextra -Werror -DRTC_32K_HOST_TEST -I. \
  tests/test_rtc_holdover.c rtc_holdover.c rtc_32k.c -lm -o "$test_dir/disabled"
"$test_dir/disabled"
cc -std=c11 -Wall -Wextra -Werror -DRTC_32K_HOST_TEST -DRTC_32KHZ_GPIO=22 -I. \
  tests/test_rtc_holdover.c rtc_holdover.c rtc_32k.c -lm -o "$test_dir/enabled"
"$test_dir/enabled"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined,address \
  -DRTC_32K_HOST_TEST -DRTC_32KHZ_GPIO=22 -I. \
  tests/test_rtc_holdover.c rtc_holdover.c rtc_32k.c -lm -o "$test_dir/sanitized"
"$test_dir/sanitized"
