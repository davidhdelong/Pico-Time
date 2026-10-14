#include "timezone_lookup.h"
#include "generated/timezone_data.h"

#include <stddef.h>
#include <string.h>

static bool in_ring(int32_t x, int32_t y, const tz_ring_t *r) {
    if (x < r->min_lon || x > r->max_lon || y < r->min_lat || y > r->max_lat) return false;
    bool inside = false;
    for (uint32_t i = 0, j = r->count - 1; i < r->count; j = i++) {
        const tz_point_t *a = &tz_points[r->first + i];
        const tz_point_t *b = &tz_points[r->first + j];
        if (((a->lat_e5 > y) != (b->lat_e5 > y)) &&
            ((int64_t)(b->lon_e5 - a->lon_e5) * (y - a->lat_e5) /
             (b->lat_e5 - a->lat_e5) + a->lon_e5 < x)) {
            inside = !inside;
        }
    }
    return inside;
}

timezone_id_t timezone_from_coordinates(int32_t latitude_e7, int32_t longitude_e7) {
    if (latitude_e7 < -900000000 || latitude_e7 > 900000000 ||
        longitude_e7 < -1800000000 || longitude_e7 > 1800000000) return TIMEZONE_ID_UTC;
    int32_t lat = latitude_e7 / 100; // generated data uses 1e-5 degrees
    int32_t lon = longitude_e7 / 100;
    for (size_t i = 0; i < tz_ring_count; ++i) {
        const tz_ring_t *r = &tz_rings[i];
        if (r->hole) continue;
        if (in_ring(lon, lat, r)) {
            for (size_t h = 0; h < tz_ring_count; ++h) {
                const tz_ring_t *hole = &tz_rings[h];
                if (hole->zone == r->zone && hole->hole && in_ring(lon, lat, hole))
                    return TIMEZONE_ID_UTC;
            }
            return r->zone;
        }
    }
    return TIMEZONE_ID_UTC;
}

const char *timezone_name(timezone_id_t zone) {
    if (zone == TIMEZONE_ID_UTC) return "UTC";
    for (size_t i = 0; i < tz_zone_count; ++i)
        if (tz_zones[i].id == zone) return tz_zones[i].name;
    return "UTC";
}

bool timezone_id_from_name(const char *name, timezone_id_t *out) {
    if (!name || !out) return false;
    if (strcmp(name, "UTC") == 0) { *out = TIMEZONE_ID_UTC; return true; }
    for (size_t i = 0; i < tz_zone_count; ++i) {
        if (strcmp(name, tz_zones[i].name) == 0) { *out = tz_zones[i].id; return true; }
    }
    return false;
}

static int64_t year_start_epoch(int year) {
    /* Gregorian January 1, in UTC. Validity is the generator's whole year
     * range, not one year beyond a zone's most recent DST transition: fixed
     * offset zones may have only the initial 2020 rule. */
    int previous_year = year - 1;
    int64_t days = 365LL * previous_year + previous_year / 4 -
        previous_year / 100 + previous_year / 400;
    return (days - 719162LL) * 86400LL;
}

bool timezone_get_offset(timezone_id_t zone, int64_t utc_epoch,
                         int32_t *offset_seconds, bool *is_dst) {
    if (!offset_seconds || !is_dst) return false;
    *offset_seconds = 0; *is_dst = false;
    if (zone == TIMEZONE_ID_UTC) return true;
    const tz_zone_t *z = NULL;
    for (size_t i = 0; i < tz_zone_count; ++i) if (tz_zones[i].id == zone) { z = &tz_zones[i]; break; }
    if (!z || !z->count || utc_epoch < year_start_epoch(TZ_DATA_START_YEAR) ||
        utc_epoch >= year_start_epoch(TZ_DATA_END_YEAR + 1) ||
        utc_epoch < tz_transitions[z->first].epoch)
        return false;
    size_t lo = 0, hi = z->count;
    while (lo + 1 < hi) {
        size_t mid = (lo + hi) / 2;
        if ((int64_t)tz_transitions[z->first + mid].epoch <= utc_epoch) lo = mid;
        else hi = mid;
    }
    const tz_transition_t *t = &tz_transitions[z->first + lo];
    *offset_seconds = t->offset; *is_dst = t->dst != 0;
    return true;
}

static void civil_from_days(int64_t z, int *y, unsigned *m, unsigned *d) {
    z += 719468; int era = (z >= 0 ? z : z - 146096) / 146097;
    unsigned doe = (unsigned)(z - era * 146097); unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int yy = (int)yoe + era * 400; unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    unsigned mp = (5 * doy + 2) / 153; *d = doy - (153 * mp + 2) / 5 + 1; *m = mp + (mp < 10 ? 3 : -9); *y = yy + (*m <= 2);
}

bool timezone_utc_to_local(int64_t utc_epoch, timezone_id_t zone, timezone_local_time_t *out) {
    int32_t off; bool dst;
    if (!out || !timezone_get_offset(zone, utc_epoch, &off, &dst)) return false;
    int64_t t = utc_epoch + off; int64_t days = t / 86400; int64_t rem = t % 86400;
    if (rem < 0) { rem += 86400; --days; }
    unsigned m, d; civil_from_days(days, &out->year, &m, &d); out->month = (int)m; out->day = (int)d;
    out->hour = (int)(rem / 3600); out->min = (int)((rem / 60) % 60); out->sec = (int)(rem % 60); return true;
}
