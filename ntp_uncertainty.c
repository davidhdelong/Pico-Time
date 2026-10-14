#include "ntp_uncertainty.h"
#include <math.h>
#include <string.h>

uint64_t ntp_saturating_add(uint64_t a, uint64_t b) {
    return a > UINT64_MAX - b ? UINT64_MAX : a + b;
}
uint64_t ntp_abs_ns(int64_t v) {
    return v < 0 ? (uint64_t)(-(v + 1)) + 1 : (uint64_t)v;
}
ntp_placement_t ntp_placement_details(uint64_t timestamp_ns, const ntp_edge_bracket_t brackets[2]) {
    ntp_placement_t result = {.bound_ns = UINT64_MAX};
    for (unsigned i = 0; i < 2; ++i) {
        uint64_t low_ns = brackets[i].low_ns, high_ns = brackets[i].high_ns;
        if (!low_ns || high_ns < low_ns || high_ns - low_ns >= 500000000ULL) continue;
        uint64_t a = timestamp_ns >= low_ns ? timestamp_ns - low_ns : low_ns - timestamp_ns;
        uint64_t b = timestamp_ns >= high_ns ? timestamp_ns - high_ns : high_ns - timestamp_ns;
        uint64_t bound_ns = a > b ? a : b;
        /* A nearest-second association is only credible within half a second.
         * Missing/narrow pulses or a stalled main loop fail closed. */
        if (bound_ns < 500000000ULL && bound_ns < result.bound_ns) {
            uint64_t midpoint_ns = low_ns + (high_ns - low_ns) / 2;
            result.bound_ns = bound_ns;
            result.window_ns = high_ns - low_ns;
            /* Accepted candidates are within half a second, so this signed
             * difference fits even when absolute timestamps exceed INT64_MAX. */
            result.offset_ns = timestamp_ns >= midpoint_ns ?
                (int64_t)(timestamp_ns - midpoint_ns) : -(int64_t)(midpoint_ns - timestamp_ns);
            result.valid = true;
        }
    }
    return result;
}
uint64_t ntp_placement_bound_ns(uint64_t timestamp_ns, const ntp_edge_bracket_t brackets[2]) {
    return ntp_placement_details(timestamp_ns, brackets).bound_ns;
}
uint32_t ntp_dispersion_from_ns(uint64_t ns) {
    uint64_t seconds = ns / 1000000000ULL;
    if (seconds >= 65536) return UINT32_MAX;
    /* Split before multiplication: even UINT64_MAX input cannot overflow. */
    uint64_t fraction = ((ns % 1000000000ULL) * 65536ULL + 999999999ULL) / 1000000000ULL;
    uint64_t encoded = seconds * 65536ULL + fraction;
    return encoded > UINT32_MAX ? UINT32_MAX : (uint32_t)encoded;
}
uint64_t ntp_dispersion_to_ns(uint32_t raw) {
    return ((uint64_t)raw * 1000000000ULL + 65535ULL) / 65536ULL;
}
void ntp_write_u32(uint8_t *out, uint32_t v) {
    out[0] = (uint8_t)(v >> 24); out[1] = (uint8_t)(v >> 16);
    out[2] = (uint8_t)(v >> 8); out[3] = (uint8_t)v;
}
void ntp_response_header(uint8_t packet[48], sntp_quality_t rx_quality,
    sntp_quality_t tx_quality, bool rx_valid, bool tx_valid, uint8_t leap,
    uint64_t rx_uncertainty_ns, uint64_t tx_uncertainty_ns) {
    bool changed = !rx_valid || !tx_valid || rx_quality != tx_quality ||
        rx_uncertainty_ns == UINT64_MAX || tx_uncertainty_ns == UINT64_MAX;
    sntp_quality_t quality = changed ? SNTP_QUALITY_NONE : tx_quality;
    uint64_t dispersion_ns = rx_uncertainty_ns > tx_uncertainty_ns ? rx_uncertainty_ns : tx_uncertainty_ns;
    if (changed) {
        if (dispersion_ns < 4000000000ULL) dispersion_ns = 4000000000ULL;
        memset(packet + 16, 0, 8);
    }
    uint8_t stratum = timing_stratum(quality);
    packet[0] = (uint8_t)(((stratum < 16 ? leap & 3u : 3u) << 6) | (4u << 3) | 4u);
    packet[1] = stratum; packet[2] = 6;
    packet[3] = (uint8_t)(quality == SNTP_QUALITY_PPS ? -20 : -10);
    ntp_write_u32(packet + 4, 0); /* signed root delay: exactly zero */
    ntp_write_u32(packet + 8, ntp_dispersion_from_ns(dispersion_ns));
    memcpy(packet + 12, timing_refid(quality), 4);
}
static uint64_t multiply_sat(uint64_t n, unsigned k) {
    return n > UINT64_MAX / k ? UINT64_MAX : n * k;
}
ntp_uncertainty_t ntp_coarse_uncertainty(uint64_t age_ns, bool sample_valid, int64_t slew_ns) {
    /* Whole-second calendar/message truncation can lose nearly a full second.
     * Coarse output is always unsynchronized, regardless of this estimate. */
    ntp_uncertainty_t u = {.age_ns=age_ns, .margin_ns=1000000000ULL,
        .growth_ns=age_ns, .slew_ns=ntp_abs_ns(slew_ns)};
    u.total_ns = sample_valid ? ntp_saturating_add(u.margin_ns,u.growth_ns) : UINT64_MAX;
    u.total_ns = ntp_saturating_add(u.total_ns,u.slew_ns);
    return u;
}
ntp_uncertainty_t ntp_pps_uncertainty(uint64_t age_ns, uint64_t capture_ns,
    uint64_t rms_ns, uint64_t mad_ns, int64_t innovation_ns,
    uint32_t samples, int64_t slew_ns) {
    ntp_uncertainty_t u = { .age_ns = age_ns, .capture_ns = capture_ns,
        /* Preserve the existing 100 us engineering allowance for unmeasured
         * GNSS/counter/model errors. Physical PPS placement is measured
         * separately in capture_ns. This is not a calibrated accuracy claim
         * and is not Wi-Fi RTT. Residuals add dynamically below. */
        .margin_ns = 100000, .slew_ns = ntp_abs_ns(slew_ns) };
    u.phase_ns = multiply_sat(rms_ns, 4);
    uint64_t robust_ns = multiply_sat(mad_ns, 6);
    if (robust_ns > u.phase_ns) u.phase_ns = robust_ns;
    if (ntp_abs_ns(innovation_ns) > u.phase_ns) u.phase_ns = ntp_abs_ns(innovation_ns);
    /* Existing statistics describe one-second INTERVAL residuals, not an
     * independently measured absolute phase. Use an envelope, not a sum of
     * correlated estimators. ns per one-second interval / 1000 = ppm.
     * 15 ppm is the RFC 5905 conventional oscillator allowance; startup uses
     * the existing +/-5 ms interval acceptance envelope until 60 samples.
     * Never substitute the oscillator frequency OFFSET for uncertainty. */
    u.frequency_uncertainty_ppm = (double)u.phase_ns / 1000.0;
    double allowance_ppm = samples >= 60 ? 15.0 : 5000.0;
    if (u.frequency_uncertainty_ppm < allowance_ppm) u.frequency_uncertainty_ppm = allowance_ppm;
    double growth_ns = ceil((double)age_ns * u.frequency_uncertainty_ppm / 1e6);
    u.growth_ns = !isfinite(growth_ns) || growth_ns >= (double)UINT64_MAX ? UINT64_MAX : (uint64_t)growth_ns;
    u.total_ns = ntp_saturating_add(u.capture_ns, u.phase_ns);
    u.total_ns = ntp_saturating_add(u.total_ns, u.growth_ns);
    u.total_ns = ntp_saturating_add(u.total_ns, u.margin_ns);
    u.total_ns = ntp_saturating_add(u.total_ns, u.slew_ns);
    return u;
}
