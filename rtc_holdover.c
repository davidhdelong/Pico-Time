#include "rtc_holdover.h"
#include <limits.h>
#include <math.h>
#include <stdlib.h>

static int64_t median(int64_t *v, unsigned n) {
    for (unsigned i = 1; i < n; ++i) {
        int64_t value = v[i];
        unsigned j = i;
        while (j && v[j - 1] > value) { v[j] = v[j - 1]; --j; }
        v[j] = value;
    }
    return v[n / 2];
}

bool rtc_phase_observe(rtc_phase_filter_t *f, int64_t phase_ns) {
    if (phase_ns < -RTC_PHASE_MAX_OFFSET_NS || phase_ns > RTC_PHASE_MAX_OFFSET_NS) {
        f->qualifying_samples = 0;
        if (f->reject_count < UINT32_MAX) ++f->reject_count;
        return false;
    }
    int64_t copy[RTC_PHASE_WINDOW];
    for (unsigned i = 0; i < f->count; ++i) copy[i] = f->samples_ns[i];
    int64_t center = f->count ? median(copy, f->count) : phase_ns;
    for (unsigned i = 0; i < f->count; ++i) copy[i] = llabs(f->samples_ns[i] - center);
    uint64_t mad_ns = f->count ? (uint64_t)median(copy, f->count) : 0;
    uint64_t threshold_ns = mad_ns * 6u;
    if (threshold_ns < RTC_PHASE_OUTLIER_FLOOR_NS) threshold_ns = RTC_PHASE_OUTLIER_FLOOR_NS;
    if (f->count >= 5 && (uint64_t)llabs(phase_ns - center) > threshold_ns) {
        f->qualifying_samples = 0;
        if (f->reject_count < UINT32_MAX) ++f->reject_count;
        return false;
    }
    f->samples_ns[f->index] = phase_ns;
    f->index = (f->index + 1) % RTC_PHASE_WINDOW;
    if (f->count < RTC_PHASE_WINDOW) ++f->count;
    if (f->sample_count < UINT32_MAX) ++f->sample_count;
    if (f->qualifying_samples < UINT32_MAX) ++f->qualifying_samples;
    for (unsigned i = 0; i < f->count; ++i) copy[i] = f->samples_ns[i];
    f->offset_ns = median(copy, f->count);
    /* Maximum residual within the accepted window is conservative and exposes
     * jitter hidden by a zero MAD on quantized samples. */
    f->jitter_ns = 0;
    for (unsigned i = 0; i < f->count; ++i) {
        uint64_t residual_ns = (uint64_t)llabs(f->samples_ns[i] - f->offset_ns);
        if (residual_ns > f->jitter_ns) f->jitter_ns = residual_ns;
    }
    f->qualified = (f->qualified || f->qualifying_samples >= RTC_PHASE_MIN_SAMPLES) &&
                   f->jitter_ns <= RTC_PHASE_MAX_JITTER_NS;
    return true;
}

bool rtc_frequency_observe(rtc_frequency_filter_t *f, uint64_t now_ns, int64_t phase_ns) {
    if (f->count && (now_ns <= f->last_ns || now_ns - f->last_ns < RTC_SLOPE_SAMPLE_INTERVAL_NS)) return false;
    f->time_ns[f->index] = now_ns;
    f->phase_ns[f->index] = phase_ns;
    f->last_ns = now_ns;
    f->index = (f->index + 1) % RTC_SLOPE_WINDOW;
    if (f->count < RTC_SLOPE_WINDOW) ++f->count;
    if (f->count < 6) return true;
    uint64_t base_ns = f->time_ns[f->count == RTC_SLOPE_WINDOW ? f->index : 0];
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (unsigned i = 0; i < f->count; ++i) {
        double x_s = (double)(f->time_ns[i] - base_ns) / 1e9;
        double y_us = (double)f->phase_ns[i] / 1000.0;
        sx += x_s; sy += y_us; sxx += x_s * x_s; sxy += x_s * y_us;
    }
    double n = f->count, centered_xx = sxx - sx * sx / n;
    if (centered_xx <= 0) return true;
    double slope_us_per_s = (sxy - sx * sy / n) / centered_xx;
    double intercept_us = (sy - slope_us_per_s * sx) / n, rss = 0;
    for (unsigned i = 0; i < f->count; ++i) {
        double x_s = (double)(f->time_ns[i] - base_ns) / 1e9;
        double residual_us = (double)f->phase_ns[i] / 1000.0 - intercept_us - slope_us_per_s * x_s;
        rss += residual_us * residual_us;
    }
    /* A later RTC edge means a slower oscillator: frequency has opposite sign
     * to phase slope. us/s is ppm. 3-sigma regression error + model floor. */
    f->frequency_error_ppm = -slope_us_per_s;
    f->uncertainty_ppm = 3.0 * sqrt(rss / (n - 2) / centered_xx) + 0.01;
    f->duration_ns = now_ns - base_ns;
    f->valid = isfinite(f->uncertainty_ppm) && fabs(f->frequency_error_ppm) <= 100.0;
    return true;
}

double rtc_ppm_to_seconds_per_day(double ppm) { return ppm * 0.0864; }

int rtc_aging_delta(double fast_ppm, double sensitivity_ppm, unsigned max_step, int current) {
    if (!isfinite(fast_ppm) || !isfinite(sensitivity_ppm) || sensitivity_ppm <= 0) return 0;
    /* Positive measured frequency is fast; a positive DS3231 register change
     * slows it. Sensitivity here is the positive magnitude, not signed gain. */
    double change = round(fast_ppm / sensitivity_ppm);
    if (change > max_step) change = max_step;
    if (change < -(double)max_step) change = -(double)max_step;
    if (change > 127 - current) change = 127 - current;
    if (change < -128 - current) change = -128 - current;
    return (int)change;
}

uint64_t rtc_holdover_error_ns(const rtc_holdover_t *h, uint64_t now_ns) {
    if (!isfinite(h->frequency_uncertainty_ppm) || h->frequency_uncertainty_ppm < 0) return UINT64_MAX;
    uint64_t age_ns = h->active && now_ns >= h->start_ns ? now_ns - h->start_ns : 0;
    /* ns * (ppm / 1e6) = ns. Never interpret a counter ns difference as us. */
    double growth_ns = (double)age_ns * h->frequency_uncertainty_ppm / 1e6;
    double total_ns = (double)h->initial_error_ns + RTC_HOLDOVER_MARGIN_NS + growth_ns;
    if (!isfinite(total_ns) || total_ns >= (double)UINT64_MAX || total_ns < 0) return UINT64_MAX;
    return (uint64_t)ceil(total_ns);
}
bool rtc_holdover_within_policy(const rtc_holdover_t *h, uint64_t now_ns) {
    if (!h->active || now_ns < h->start_ns) return false;
    return (!RTC_HOLDOVER_MAX_DURATION_S ||
            (now_ns - h->start_ns) / 1000000000ULL < RTC_HOLDOVER_MAX_DURATION_S) &&
           rtc_holdover_error_ns(h, now_ns) <= RTC_HOLDOVER_MAX_ERROR_NS;
}
int64_t rtc_compensated_elapsed_ns(uint64_t now_ns, uint64_t edge_ns, int64_t phase_ns) {
    return (int64_t)now_ns - (int64_t)edge_ns + phase_ns;
}
bool rtc_32k_frequency(uint64_t elapsed_ns, uint32_t cycles, uint32_t pps_period_ns, double *fast_ppm) {
    if (!elapsed_ns || !cycles || !pps_period_ns || !fast_ppm) return false;
    /* Both intervals use the same Pico counter. PPS supplies the SI scale. */
    double ratio = ((double)cycles * pps_period_ns) / ((double)elapsed_ns * 32768.0);
    *fast_ppm = (ratio - 1.0) * 1e6;
    return isfinite(*fast_ppm) && fabs(*fast_ppm) <= 100.0;
}
bool gnss_recovery_observe(gnss_recovery_t *g, uint64_t edge_ns, bool accepted) {
    bool contiguous = g->last_edge_ns && edge_ns > g->last_edge_ns &&
        edge_ns - g->last_edge_ns >= 995000000ULL && edge_ns - g->last_edge_ns <= 1005000000ULL;
    if (!accepted) { g->good_pulses = 0; g->locked = false; }
    else {
        g->good_pulses = contiguous ? g->good_pulses + (g->good_pulses < GNSS_RECOVERY_GOOD_PULSES) : 1;
        g->locked = g->good_pulses >= GNSS_RECOVERY_GOOD_PULSES;
    }
    g->last_edge_ns = edge_ns;
    return g->locked;
}
int64_t utc_continuity_update(utc_continuity_t *c, uint64_t counter_ns, int64_t target_ns) {
    if (!c->initialized) {
        c->initialized = true; c->counter_ns = counter_ns; c->utc_ns = target_ns;
        c->residual_ns = 0; c->rate = 1.0; return target_ns;
    }
    /* Requests can carry an earlier receive timestamp than the latest status
     * sample. Do not rewind the model for such an observation. */
    if (counter_ns <= c->counter_ns) {
        uint64_t age_ns = c->counter_ns - counter_ns;
        return c->utc_ns - (int64_t)((double)age_ns * c->rate);
    }
    uint64_t elapsed_ns = counter_ns - c->counter_ns;
    int64_t predicted_ns = c->utc_ns + (int64_t)elapsed_ns;
    int64_t error_ns = target_ns - predicted_ns;
    int64_t limit_ns = (int64_t)(elapsed_ns / 1000000ULL * UTC_MAX_SLEW_PPM +
        (elapsed_ns % 1000000ULL) * UTC_MAX_SLEW_PPM / 1000000ULL);
    int64_t correction_ns = error_ns;
    if (correction_ns > limit_ns) correction_ns = limit_ns;
    if (correction_ns < -limit_ns) correction_ns = -limit_ns;
    c->utc_ns = predicted_ns + correction_ns;
    c->rate = 1.0 + (double)correction_ns / (double)elapsed_ns;
    c->counter_ns = counter_ns;
    c->residual_ns = error_ns - correction_ns;
    return c->utc_ns;
}
uint8_t rtc_status_output_bits(uint8_t status, bool enable_32k) {
    /* OSF and alarm flags are write-zero-to-clear. Write ones to preserve even
     * a flag asserted between read and write. */
    return (uint8_t)((status & 0x04u) | 0x83u | (enable_32k ? 0x08u : 0u));
}
bool rtc_calendar_status_valid(uint8_t status) { return (status & 0x80u) == 0; }
bool timing_edge_fresh(uint64_t now_ns, uint64_t edge_ns, uint64_t timeout_ns) {
    return edge_ns && now_ns >= edge_ns && now_ns - edge_ns < timeout_ns;
}
uint8_t timing_stratum(sntp_quality_t quality) {
    return quality == SNTP_QUALITY_PPS ? 1 : quality == SNTP_QUALITY_RTC_SQW ? 2 : 16;
}
const char *timing_refid(sntp_quality_t quality) {
    return quality == SNTP_QUALITY_PPS ? "GPS " : quality == SNTP_QUALITY_RTC_SQW ? "RTC " : "LOCL";
}

selected_time_t timing_select(const timing_observation_t *o, rtc_holdover_t *h, utc_continuity_t *c) {
    selected_time_t out = {0};
    int64_t target_ns = 0;
    if (timing_edge_fresh(o->now_ns, o->pps_ns, o->pps_timeout_ns) && o->pps_period_ns) {
        uint64_t age_ns = o->now_ns - o->pps_ns;
        /* Freshness bounds age to a few seconds; use double to avoid a
         * configurable timeout overflowing an age*1e9 integer product. */
        int64_t elapsed_ns = (int64_t)((double)age_ns * 1e9 / o->pps_period_ns);
        target_ns = o->pps_epoch_s * 1000000000LL + elapsed_ns;
        out.valid = true; out.quality = SNTP_QUALITY_PPS;
        h->active = false;
    } else if (o->rtc_valid && o->rtc_period_ns &&
               timing_edge_fresh(o->now_ns, o->rtc_edge_ns, o->rtc_timeout_ns)) {
        if (!h->active) {
            h->active = true; h->start_ns = o->rtc_qualified_ns;
            h->initial_error_ns = o->rtc_jitter_ns > UINT64_MAX - o->rtc_reference_error_ns ?
                UINT64_MAX : o->rtc_jitter_ns + o->rtc_reference_error_ns;
            h->frequency_uncertainty_ppm = o->rtc_uncertainty_ppm;
        }
        uint64_t age_ns = o->now_ns - o->rtc_edge_ns;
        int64_t elapsed_ns = (int64_t)((double)age_ns * 1e9 / o->rtc_period_ns) + o->rtc_phase_ns;
        uint64_t holdover_age_ns = o->now_ns >= h->start_ns ? o->now_ns - h->start_ns : 0;
        elapsed_ns -= (int64_t)((double)holdover_age_ns * o->rtc_fast_ppm / 1e6);
        target_ns = o->rtc_epoch_s * 1000000000LL + elapsed_ns;
        out.valid = true;
        out.quality = rtc_holdover_within_policy(h, o->now_ns) ? SNTP_QUALITY_RTC_SQW : SNTP_QUALITY_COARSE_RTC;
    } else if (o->coarse_valid) {
        target_ns = o->coarse_epoch_s * 1000000000LL;
        out.valid = true;
        out.quality = o->coarse_rtc ? SNTP_QUALITY_COARSE_RTC : SNTP_QUALITY_COARSE_GPS;
    }
    if (out.valid) {
        if (c->initialized || out.quality == SNTP_QUALITY_PPS || out.quality == SNTP_QUALITY_RTC_SQW) {
            target_ns = utc_continuity_update(c, o->now_ns, target_ns);
            uint64_t residual_ns = (uint64_t)llabs(c->residual_ns);
            uint64_t reference_ns = out.quality == SNTP_QUALITY_RTC_SQW ? rtc_holdover_error_ns(h, o->now_ns) : 0;
            if (residual_ns > RTC_HOLDOVER_MAX_ERROR_NS || reference_ns > RTC_HOLDOVER_MAX_ERROR_NS - residual_ns)
                out.quality = (out.quality == SNTP_QUALITY_PPS || out.quality == SNTP_QUALITY_COARSE_GPS) ?
                    SNTP_QUALITY_COARSE_GPS : SNTP_QUALITY_COARSE_RTC;
        }
        out.unix_sec = target_ns / 1000000000LL;
        int64_t fraction_ns = target_ns % 1000000000LL;
        if (fraction_ns < 0) { --out.unix_sec; fraction_ns += 1000000000LL; }
        out.fraction = (uint32_t)(((uint64_t)fraction_ns << 32) / 1000000000ULL);
    }
    return out;
}
