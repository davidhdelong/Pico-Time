#include "timezone_lookup.h"
#include "generated/timezone_data.h"
#include <assert.h>
#include <stdio.h>

static void zone(double lat, double lon, const char *expected) {
    timezone_id_t id = timezone_from_coordinates((int)(lat * 10000000.0),
                                                  (int)(lon * 10000000.0));
    printf("%.4f %.4f -> %s\n", lat, lon, timezone_name(id));
    assert(__builtin_strcmp(timezone_name(id), expected) == 0);
}

int main(void) {
    zone(30.0927, -93.7199, "America/Chicago");
    zone(40.7128, -74.0060, "America/New_York");
    zone(39.7392, -104.9903, "America/Denver");
    zone(33.4484, -112.0740, "America/Phoenix");
    zone(21.3069, -157.8583, "Pacific/Honolulu");
    zone(61.2181, -149.9003, "America/Anchorage");
    zone(51.5074, -0.1278, "Europe/London");
    zone(48.8566, 2.3522, "Europe/Paris");
    zone(35.6762, 139.6503, "Asia/Tokyo");
    zone(28.6139, 77.2090, "Asia/Kolkata");
    zone(27.7172, 85.3240, "Asia/Kathmandu");
    zone(-33.8688, 151.2093, "Australia/Sydney");
    zone(-34.9285, 138.6007, "Australia/Adelaide");
    zone(-31.9505, 115.8605, "Australia/Perth");
    zone(-41.2866, 174.7756, "Pacific/Auckland");
    zone(47.5615, -52.7126, "America/St_Johns");
    zone(-43.9509, -176.5617, "Pacific/Chatham");
    zone(0.0, 0.0, "UTC");

    timezone_id_t ny;
    assert(timezone_id_from_name("America/New_York", &ny));
    int32_t off; bool dst;
    assert(timezone_get_offset(ny, 1772953200LL, &off, &dst)); // 2026-03-08T07:00Z
    assert(off == -14400 && dst);
    assert(timezone_get_offset(ny, 1762063200LL, &off, &dst)); // 2025-11-02T06:00Z
    assert(off == -18000 && !dst);
    timezone_local_time_t local;
    assert(timezone_utc_to_local(1772953200LL, ny, &local));
    assert(local.hour == 3 && local.min == 0);
    /* Non-hour offsets are checked as rules, not just zone-name selection. */
    timezone_id_t kathmandu, kolkata;
    assert(timezone_id_from_name("Asia/Kathmandu", &kathmandu));
    assert(timezone_id_from_name("Asia/Kolkata", &kolkata));
    assert(timezone_get_offset(kathmandu, 1772953200LL, &off, &dst));
    assert(off == 20700 && !dst);
    assert(timezone_get_offset(kolkata, 1772953200LL, &off, &dst));
    assert(off == 19800 && !dst);
    assert(timezone_get_offset(kathmandu, 1577836800LL, &off, &dst));
    assert(timezone_get_offset(kathmandu, 2556143999LL, &off, &dst));
    assert(!timezone_get_offset(kathmandu, 1577836799LL, &off, &dst));
    assert(!timezone_get_offset(kathmandu, 2556144000LL, &off, &dst));
    assert(!timezone_get_offset(ny, 2556144000LL, &off, &dst));
    /* Cross and recross the Apalachicola River near Blountstown/Bristol.
     * Lookup must not retain the previous geographic selection. */
    zone(30.4435, -85.0452, "America/Chicago");
    zone(30.4325, -84.9771, "America/New_York");
    zone(30.4435, -85.0452, "America/Chicago");
    zone(0.0, 0.0, "UTC");
    puts("timezone tests passed");
    printf("tzdb=%s boundaries=%s years=%d-%d zones=%zu points=%zu rings=%zu transitions=%zu\n",
           TZDB_VERSION, TZ_BOUNDARY_VERSION, TZ_DATA_START_YEAR, TZ_DATA_END_YEAR,
           tz_zone_count, tz_point_count, tz_ring_count, tz_transition_count);
    return 0;
}
