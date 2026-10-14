#ifndef NTP_UNCERTAINTY_H
#define NTP_UNCERTAINTY_H
#include <stdint.h>
#include <stdbool.h>
#include "rtc_holdover.h"

typedef struct {
    uint64_t age_ns, capture_ns, phase_ns, growth_ns, margin_ns, slew_ns, total_ns;
    double frequency_uncertainty_ppm;
    /* Reporting only, from the same trusted PPS snapshot as capture_ns. */
    uint64_t placement_window_ns;
    int64_t placement_offset_ns;
    bool placement_valid;
} ntp_uncertainty_t;
/* Reporting-only brackets of a physical low->high transition. Two slots let
 * a rejected/new PPS coexist with the previous trusted PPS during coast. */
typedef struct {
    uint64_t low_ns, high_ns;
} ntp_edge_bracket_t;
typedef struct {
    uint64_t bound_ns, window_ns;
    int64_t offset_ns; /* reconstructed timestamp minus bracket midpoint */
    bool valid;
} ntp_placement_t;
ntp_placement_t ntp_placement_details(uint64_t reconstructed_ns,
    const ntp_edge_bracket_t brackets[2]);
uint64_t ntp_placement_bound_ns(uint64_t reconstructed_ns,
    const ntp_edge_bracket_t brackets[2]);
/* Writes header quality fields and clears stale reference time on a change.
 * T1/T2/T3 are preserved. Delay is zero for this directly attached reference. */
void ntp_response_header(uint8_t packet[48], sntp_quality_t rx_quality,
    sntp_quality_t tx_quality, bool rx_valid, bool tx_valid, uint8_t leap,
    uint64_t rx_uncertainty_ns, uint64_t tx_uncertainty_ns);
ntp_uncertainty_t ntp_coarse_uncertainty(uint64_t age_ns, bool sample_valid, int64_t slew_ns);

uint64_t ntp_saturating_add(uint64_t a, uint64_t b);
uint64_t ntp_abs_ns(int64_t value);
/* Unsigned 16.16 seconds; round UP so quantization cannot understate error. */
uint32_t ntp_dispersion_from_ns(uint64_t dispersion_ns);
uint64_t ntp_dispersion_to_ns(uint32_t ntp16_16);
void ntp_write_u32(uint8_t *out, uint32_t value);
ntp_uncertainty_t ntp_pps_uncertainty(uint64_t age_ns, uint64_t capture_ns,
    uint64_t rms_ns, uint64_t mad_ns, int64_t innovation_ns,
    uint32_t samples, int64_t slew_ns);
#endif
