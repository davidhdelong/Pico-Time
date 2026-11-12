#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "gps_validation.h"

int main(void) {
    assert(gps_number_or_unknown("1.25", 0) == 1.25f);
    assert(gps_number_or_unknown("-42.5", -FLT_MAX) == -42.5f);
    const char *bad[] = {NULL, "", "nan", "NaN", "inf", "-inf", "1e999", "1e-999", "1.2junk", "-0.5"};
    for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); ++i)
        assert(gps_number_or_unknown(bad[i], 0) == -1.0f);
    assert(!gps_quality_is_fresh(1000000000ULL, 0, 3000));
    assert(!gps_quality_is_fresh(999999999ULL, 1000000000ULL, 3000));
    assert(gps_quality_is_fresh(1000000000ULL, 1000000000ULL, 3000));
    assert(gps_quality_is_fresh(4000000000ULL, 1000000000ULL, 3000));
    assert(!gps_quality_is_fresh(4000000001ULL, 1000000000ULL, 3000));
    assert(!gps_quality_is_fresh(1000000001ULL, 1000000000ULL, 0));
    return 0;
}
