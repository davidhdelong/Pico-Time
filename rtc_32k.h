#ifndef RTC_32K_H
#define RTC_32K_H
#include "rtc_holdover.h"
typedef struct {
    bool enabled, valid;
    uint64_t last_ns, base_ns;
    uint32_t sequence, base_sequence, samples, rejects, good_blocks;
    double frequency_error_ppm, uncertainty_ppm;
    double pico_period_ns;
    uint32_t last_reject;
} rtc_32k_t;
void rtc_32k_init(void);
bool rtc_32k_take(uint64_t *timestamp_ns, uint32_t *sequence);
void rtc_32k_observe(rtc_32k_t *f, uint64_t timestamp_ns, uint32_t sequence,
                     bool gnss_locked, uint32_t pps_period_ns);
#endif
