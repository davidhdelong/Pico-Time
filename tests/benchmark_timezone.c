#include "timezone_lookup.h"
#include <stdio.h>
#include <time.h>

static long long ns_now(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (long long)t.tv_sec * 1000000000LL + t.tv_nsec;
}

int main(void) {
    const int n = 100000;
    volatile timezone_id_t id = 0;
    long long a = ns_now();
    for (int i = 0; i < n; ++i) id = timezone_from_coordinates(300927000, -937199000);
    long long b = ns_now();
    int32_t offset; bool dst;
    long long c = ns_now();
    for (int i = 0; i < n; ++i) timezone_get_offset(id, 1788485373LL + i, &offset, &dst);
    long long d = ns_now();
    printf("polygon_lookup_ns=%lld rule_lookup_ns=%lld zone=%s\n",
           (b - a) / n, (d - c) / n, timezone_name(id));
    return 0;
}
