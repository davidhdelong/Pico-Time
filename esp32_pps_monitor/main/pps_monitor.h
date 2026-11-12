#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define PPS_HISTORY 300u
#define PPS_FRESH_US 2500000ULL
#define PPS_PAIR_MAX_US 400000ULL
typedef struct {
    uint32_t ticks;
    uint64_t observed_us;
    uint8_t input;
    bool rising;
} pps_edge_t;
typedef struct {
    bool seen, have_period, have_width, width_pending, timed_out;
    uint32_t rise_ticks, good_intervals;
    uint64_t rise_us, edges, glitches, missed;
    double period_us, width_us;
    bool pending;
} pps_input_t;
typedef struct { uint64_t observed_us, segment; double offset_us; } pps_pair_t;
typedef struct {
    uint32_t resolution_hz;
    pps_input_t input[2];
    pps_pair_t history[PPS_HISTORY];
    uint32_t count, next;
    uint64_t pairs, unpaired, invalidations, segment;
    bool segment_open;
} pps_monitor_t;
typedef struct {
    bool valid, drift_valid;
    double latest_us, mean_us, median_us, min_us, max_us, stddev_us, rms_us, drift_ppm;
    uint64_t age_us;
    uint32_t drift_samples;
} pps_summary_t;

void pps_monitor_init(pps_monitor_t *m, uint32_t resolution_hz);
void pps_monitor_edge(pps_monitor_t *m, const pps_edge_t *e);
void pps_monitor_invalidate(pps_monitor_t *m);
void pps_monitor_poll(pps_monitor_t *m, uint64_t now_us);
uint64_t pps_input_missed(const pps_input_t *input, uint64_t now_us);
void pps_monitor_clear_stats(pps_monitor_t *m);
bool pps_input_fresh(const pps_input_t *input, uint64_t now_us);
void pps_monitor_summary(const pps_monitor_t *m, uint64_t now_us, pps_summary_t *s);
const pps_pair_t *pps_monitor_pair(const pps_monitor_t *m, uint32_t index);
