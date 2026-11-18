#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "../../rtc_holdover.h"
#include "../../ntp_uncertainty.h"
#define P4_INTERVALS 17u
typedef struct {
    uint64_t edge_ns, message_ns, trusted_ns, capture_bound_ns;
    int64_t epoch_s, trusted_epoch_s, period_rate, innovation_ns;
    uint32_t period_ns, intervals[P4_INTERVALS], count, next, samples;
    uint64_t rejected, capture_faults;
    bool epoch_known, servo_valid;
    gnss_recovery_t recovery;
    utc_continuity_t continuity;
    rtc_holdover_t holdover;
} p4_clock_t;
typedef struct {
    selected_time_t time;
    ntp_uncertainty_t uncertainty;
    bool locked, pps_present, message_fresh;
} p4_sample_t;
void p4_clock_init(p4_clock_t *c, uint64_t capture_bound_ns);
void p4_clock_fault(p4_clock_t *c);
bool p4_clock_pps(p4_clock_t *c, uint64_t edge_ns);
bool p4_clock_utc(p4_clock_t *c, int64_t epoch_s, uint64_t message_ns);
p4_sample_t p4_clock_sample(p4_clock_t *c, uint64_t now_ns);
bool p4_ntp_reply(p4_clock_t *c, const uint8_t *request, size_t len,
                  uint64_t rx_ns, uint64_t tx_ns, uint8_t reply[48]);
