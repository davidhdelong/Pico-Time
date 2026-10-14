#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "gps_validation.h"

int main(void) {
    assert(!gps_quality_is_fresh(1000000000ULL, 0, 3000));
    assert(!gps_quality_is_fresh(999999999ULL, 1000000000ULL, 3000));
    assert(gps_quality_is_fresh(1000000000ULL, 1000000000ULL, 3000));
    assert(gps_quality_is_fresh(4000000000ULL, 1000000000ULL, 3000));
    assert(!gps_quality_is_fresh(4000000001ULL, 1000000000ULL, 3000));
    assert(!gps_quality_is_fresh(1000000001ULL, 1000000000ULL, 0));
    return 0;
}
