#ifndef GPS_VALIDATION_H
#define GPS_VALIDATION_H

#include <stdbool.h>
#include <stdint.h>

static inline bool gps_quality_is_fresh(uint64_t now_ns,
                                        uint64_t last_gga_ns,
                                        uint32_t max_age_ms) {
    if (last_gga_ns == 0 || now_ns < last_gga_ns) {
        return false;
    }
    return (now_ns - last_gga_ns) <= ((uint64_t)max_age_ms * 1000000ULL);
}

#endif
