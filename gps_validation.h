#ifndef GPS_VALIDATION_H
#define GPS_VALIDATION_H

#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <float.h>
#include <math.h>
#include <stdlib.h>

/* Missing or malformed navigation data must stay finite when formatted as
 * JSON. -1 is the dashboard's existing unavailable-number sentinel. */
static inline float gps_number_or_unknown(const char *raw, float minimum) {
    if (!raw || !*raw) return -1.0f;
    char *end = NULL;
    errno = 0;
    float value = strtof(raw, &end);
    if (end == raw || *end != '\0' || errno == ERANGE ||
        !isfinite(value) || value < minimum) return -1.0f;
    return value;
}

static inline bool gps_quality_is_fresh(uint64_t now_ns,
                                        uint64_t last_gga_ns,
                                        uint32_t max_age_ms) {
    if (last_gga_ns == 0 || now_ns < last_gga_ns) {
        return false;
    }
    return (now_ns - last_gga_ns) <= ((uint64_t)max_age_ms * 1000000ULL);
}

#endif
