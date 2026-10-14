#ifndef TM1637_H
#define TM1637_H

#include <stdbool.h>
#include <stdint.h>

void tm1637_init(void);
void tm1637_update(uint64_t now_us, uint32_t ntp_seconds, bool time_valid, bool pps_edge, bool pps_phase, int64_t pps_time_us, bool pps_locked, bool rtc_fallback);
uint32_t tm1637_error_count(void);

#endif
