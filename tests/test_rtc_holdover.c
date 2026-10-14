#include "rtc_holdover.h"
#include "rtc_32k.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SECOND 1000000000ULL
static void near(double a, double b, double tolerance) { assert(fabs(a - b) <= tolerance); }

static void test_units_and_aging(void) {
    near(rtc_ppm_to_seconds_per_day(1), .0864, 1e-12);
    near(rtc_ppm_to_seconds_per_day(.1), .00864, 1e-12);
    assert(rtc_aging_delta(.1, .1, 4, 0) == 1);
    assert(rtc_aging_delta(-.1, .1, 4, 0) == -1);
    assert(rtc_aging_delta(4, .1, 4, 0) == 4);
    assert(rtc_aging_delta(-4, .1, 4, 0) == -4);
    assert(rtc_aging_delta(1, .1, 4, 126) == 1);
    assert(rtc_aging_delta(-1, .1, 4, -127) == -1);
    assert(rtc_aging_delta(1, .1, 4, 127) == 0);
    assert(rtc_aging_delta(-1, .1, 4, -128) == 0);
    assert(rtc_aging_delta(.01, .1, 4, 0) == 0);
    assert(rtc_aging_delta(NAN, .1, 4, 0) == 0);
}
static void test_uncertainty(void) {
    rtc_holdover_t h = {.active = true, .start_ns = SECOND, .initial_error_ns = 100000,
                         .frequency_uncertainty_ppm = 5};
    const uint64_t ages_s[] = {1, 60, 3600, 86400};
    for (unsigned i = 0; i < 4; ++i) {
        uint64_t expected_ns = 100000 + RTC_HOLDOVER_MARGIN_NS + ages_s[i] * 5000;
        assert(rtc_holdover_error_ns(&h, SECOND + ages_s[i] * SECOND) == expected_ns);
    }
    assert(rtc_holdover_within_policy(&h, SECOND + 3600 * SECOND));
    assert(!rtc_holdover_within_policy(&h, SECOND + 86400 * SECOND));
    h.frequency_uncertainty_ppm = 0;
    assert(!rtc_holdover_within_policy(&h, SECOND + 86400 * SECOND));
    h.active = false;
    assert(!rtc_holdover_within_policy(&h, SECOND));
    h.active = true; h.frequency_uncertainty_ppm = INFINITY;
    assert(rtc_holdover_error_ns(&h, SECOND * 2) == UINT64_MAX);
}
static void test_phase(void) {
    rtc_phase_filter_t f = {0};
    for (unsigned i = 0; i < RTC_PHASE_MIN_SAMPLES; ++i)
        assert(rtc_phase_observe(&f, 18000000 + (i % 2 ? 1000 : -1000)));
    assert(f.qualified);
    near((double)f.offset_ns, 18000000, 1000);
    assert(f.jitter_ns <= 2000);
    int64_t old_offset = f.offset_ns;
    assert(!rtc_phase_observe(&f, 100000000));
    assert(f.offset_ns == old_offset && f.reject_count == 1);
    /* A positive phase means the physical edge arrives late: at that edge
     * UTC is already 18 ms into the named second. */
    assert(rtc_compensated_elapsed_ns(SECOND + 18000000, SECOND + 18000000, 18000000) == 18000000);
    assert(rtc_compensated_elapsed_ns(SECOND, SECOND - 16000000, -16000000) == 0);
    memset(&f, 0, sizeof(f));
    for (unsigned i = 0; i < 100; ++i) rtc_phase_observe(&f, i % 2 ? 2000000 : -2000000);
    assert(!f.qualified);
}
static void test_frequency(void) {
    rtc_frequency_filter_t f = {0};
    for (unsigned i = 0; i <= 220; ++i) {
        uint64_t now_ns = SECOND + i * RTC_SLOPE_SAMPLE_INTERVAL_NS;
        /* +360 us/hour phase drift = slow oscillator by 0.1 ppm. */
        int64_t phase_ns = 18000000 + (int64_t)i * 12000;
        assert(rtc_frequency_observe(&f, now_ns, phase_ns));
    }
    assert(f.valid && f.count == RTC_SLOPE_WINDOW);
    near(f.frequency_error_ppm, -.1, 1e-9);
    near(f.uncertainty_ppm, .01, 1e-8);
    assert(f.duration_ns == (RTC_SLOPE_WINDOW - 1) * RTC_SLOPE_SAMPLE_INTERVAL_NS);
    assert(!rtc_frequency_observe(&f, f.last_ns + SECOND, 0));
    /* Reacquisition starts a new estimator, not a regression spanning a jump. */
    memset(&f, 0, sizeof(f));
    for (unsigned i = 0; i < 6; ++i)
        rtc_frequency_observe(&f, SECOND + i * RTC_SLOPE_SAMPLE_INTERVAL_NS, -(int64_t)i * 12000);
    near(f.frequency_error_ppm, .1, 1e-9);
}
static void test_32k(void) {
    double ppm = 0;
    assert(rtc_32k_frequency(64 * SECOND, 64 * 32768, 1000000000, &ppm));
    near(ppm, 0, 1e-9);
    assert(rtc_32k_frequency(64000640000ULL, 64 * 32768, 1000000000, &ppm));
    near(ppm, -10, .001);
    assert(rtc_32k_frequency(64000640000ULL, 64 * 32768, 1000010000, &ppm));
    near(ppm, 0, 1e-9);
    assert(!rtc_32k_frequency(0, 32768, 1000000000, &ppm));
    assert(!rtc_32k_frequency(SECOND / 2, 32768, 1000000000, &ppm));
    rtc_32k_t f = {0};
    for (unsigned i = 0; i < 270; ++i)
        rtc_32k_observe(&f, SECOND + i * SECOND, i, true, 1000000000);
#if RTC_32KHZ_GPIO >= 0
    assert(f.enabled && f.valid && f.samples >= 4);
    near(f.frequency_error_ppm, 0, 1e-9);
    rtc_32k_observe(&f, 271 * SECOND, 273, true, 1000000000);
    assert(!f.valid && f.rejects);
#else
    assert(!f.enabled && !f.valid && !f.samples);
#endif
}
static void test_recovery_and_semantics(void) {
    gnss_recovery_t g = {0};
    for (unsigned i = 1; i < GNSS_RECOVERY_GOOD_PULSES; ++i)
        assert(!gnss_recovery_observe(&g, i * SECOND, true));
    assert(gnss_recovery_observe(&g, GNSS_RECOVERY_GOOD_PULSES * SECOND, true));
    uint64_t last = g.last_edge_ns;
    assert(timing_edge_fresh(last + 1499999999, last, 1500000000));
    assert(!timing_edge_fresh(last + 1500000000, last, 1500000000));
    assert(!gnss_recovery_observe(&g, last + 2 * SECOND, true));
    assert(!gnss_recovery_observe(&g, last + 3 * SECOND, false));
    for (unsigned i = 1; i <= GNSS_RECOVERY_GOOD_PULSES; ++i)
        assert(gnss_recovery_observe(&g, last + (3 + i) * SECOND, true) == (i == GNSS_RECOVERY_GOOD_PULSES));
    assert(timing_edge_fresh(3 * SECOND, SECOND, 2500000000ULL));
    assert(!timing_edge_fresh(3500000000ULL, SECOND, 2500000000ULL));
    for (int quality = 0; quality <= 4; ++quality) {
        assert(timing_stratum((sntp_quality_t)quality) == (quality == 1 ? 1 : quality == 2 ? 2 : 16));
        assert(!strcmp(timing_refid((sntp_quality_t)quality), quality == 1 ? "GPS " : quality == 2 ? "RTC " : "LOCL"));
    }
}
static void test_continuity(void) {
    /* Concurrent NTP RX/TX samples must preserve their receive instant,
     * including when the served clock is slewing at either rate limit. */
    for (int sign = -1; sign <= 1; sign += 2) {
        utc_continuity_t c = {0};
        int64_t epoch_ns = 1788820000LL * (int64_t)SECOND;
        assert(utc_continuity_update(&c, SECOND, epoch_ns) == epoch_ns);
        int64_t correction_ns = sign * (int64_t)UTC_MAX_SLEW_PPM * 1000;
        assert(utc_continuity_update(&c, 2 * SECOND,
            epoch_ns + (int64_t)SECOND + sign * 10000000LL) == epoch_ns + (int64_t)SECOND + correction_ns);
        utc_continuity_t saved = c;
        int64_t rx = utc_continuity_update(&c, 2 * SECOND - 10000000ULL, 0);
        int64_t expected = epoch_ns + 990000000LL + correction_ns * 99 / 100;
        assert(llabs(rx - expected) <= 1);
        assert(memcmp(&saved, &c, sizeof(c)) == 0);
        int64_t tx = utc_continuity_update(&c, 2 * SECOND + 1000000ULL,
            epoch_ns + (int64_t)SECOND + 1000000LL + sign * 10000000LL);
        assert(tx > rx);
    }
    for (int sign = -1; sign <= 1; sign += 2) {
        utc_continuity_t c = {0};
        int64_t epoch_ns = 1788820000LL * (int64_t)SECOND;
        int64_t last = utc_continuity_update(&c, SECOND, epoch_ns);
        /* One hour of fast or slow RTC, then either sign of GNSS correction. */
        for (unsigned t = 1; t <= 4000; ++t) {
            int64_t rtc_error_ns = t <= 3600 ? sign * (int64_t)t * 5000 : 0;
            int64_t value = utc_continuity_update(&c, (t + 1) * SECOND, epoch_ns + t * SECOND + rtc_error_ns);
            assert(value > last);
            last = value;
        }
        assert(llabs(c.residual_ns) < 1000);
        utc_continuity_t saved = c;
        int64_t historical = utc_continuity_update(&c, c.counter_ns - SECOND, epoch_ns);
        assert(llabs(historical - (last - (int64_t)SECOND)) < 1000);
        assert(memcmp(&saved, &c, sizeof(c)) == 0);
    }
}
static void test_osf(void) {
    for (unsigned status = 0; status < 256; ++status) {
        assert(rtc_calendar_status_valid((uint8_t)status) == !(status & 0x80));
        assert(rtc_status_output_bits((uint8_t)status, false) & 0x80);
        assert(rtc_status_output_bits((uint8_t)status, true) & 8);
        assert(!(rtc_status_output_bits((uint8_t)status, false) & 8));
    }
}
static int64_t selected_ns(selected_time_t t) {
    return t.unix_sec * (int64_t)SECOND + (int64_t)(((uint64_t)t.fraction * SECOND) >> 32);
}
static void test_source_sequence(void) {
    for (int sign = -1; sign <= 1; sign += 2) {
        rtc_holdover_t h = {0}; utc_continuity_t c = {0};
        timing_observation_t o = {
            .now_ns = 10 * SECOND, .pps_ns = 10 * SECOND, .pps_epoch_s = 1788800000,
            .pps_period_ns = SECOND, .pps_timeout_ns = 1500000000ULL,
            .rtc_valid = true, .rtc_edge_ns = 10 * SECOND + 18000000,
            .rtc_qualified_ns = 10 * SECOND, .rtc_epoch_s = 1788800000,
            .rtc_phase_ns = 18000000, .rtc_period_ns = SECOND,
            .rtc_timeout_ns = 2500000000ULL, .rtc_uncertainty_ppm = 5,
            .coarse_valid = true, .coarse_rtc = true, .coarse_epoch_s = 1788800000
        };
        selected_time_t t = timing_select(&o, &h, &c);
        assert(t.quality == SNTP_QUALITY_PPS);
        int64_t last = selected_ns(t);
        o.now_ns += 1499999999;
        assert(timing_select(&o, &h, &c).quality == SNTP_QUALITY_PPS);
        /* PPS expires; an already-qualified RTC immediately supplies time.
         * During the simulated outage its physical period is +/-5 ppm. */
        for (unsigned sec = 2; sec < 1000; ++sec) {
            o.now_ns = (10 + sec) * SECOND + 100000000;
            o.rtc_edge_ns = (uint64_t)((int64_t)((10 + sec) * SECOND + 18000000) + sign * (int64_t)sec * 5000);
            o.rtc_epoch_s = 1788800000 + sec;
            o.coarse_epoch_s = o.rtc_epoch_s;
            t = timing_select(&o, &h, &c);
            assert(t.quality == SNTP_QUALITY_RTC_SQW && h.active);
            assert(selected_ns(t) > last); last = selected_ns(t);
        }
        /* Publish PPS observations only after the same recovery helper used
         * by the firmware accepts consecutive good pulses. */
        gnss_recovery_t g = {0};
        for (unsigned sec = 1000; sec < 1060; ++sec) {
            uint64_t edge_ns = (10 + sec) * SECOND;
            bool locked = gnss_recovery_observe(&g, edge_ns, true);
            o.now_ns = edge_ns + 100000000;
            o.rtc_edge_ns += SECOND; ++o.rtc_epoch_s;
            if (locked) { o.pps_ns = edge_ns; o.pps_epoch_s = 1788800000 + sec; }
            t = timing_select(&o, &h, &c);
            assert(t.quality == (locked ? SNTP_QUALITY_PPS : SNTP_QUALITY_RTC_SQW));
            assert(selected_ns(t) > last); last = selected_ns(t);
        }
        assert(llabs(c.residual_ns) <= 1);
        /* Uncertainty expiration changes quality, not availability of time. */
        o.pps_ns = 0;
        o.now_ns += 86400 * SECOND; o.rtc_edge_ns = o.now_ns - 100000000;
        o.rtc_epoch_s += 86400;
        t = timing_select(&o, &h, &c);
        assert(t.valid && t.quality == SNTP_QUALITY_COARSE_RTC);
        assert(selected_ns(t) > last);
        /* RTC signal loss expires its precision reference. */
        o.now_ns += 3 * SECOND;
        assert(timing_select(&o, &h, &c).quality == SNTP_QUALITY_COARSE_RTC);
        o.coarse_rtc = false;
        assert(timing_select(&o, &h, &c).quality == SNTP_QUALITY_COARSE_GPS);
        o.coarse_valid = false;
        t = timing_select(&o, &h, &c);
        assert(!t.valid && timing_stratum(t.quality) == 16);
    }
    /* Cold boot with calendar but no phase mapping is never synchronized. */
    rtc_holdover_t h = {0}; utc_continuity_t c = {0};
    timing_observation_t o = {.now_ns = SECOND, .coarse_valid = true, .coarse_rtc = true,
                               .coarse_epoch_s = 1788800000};
    assert(timing_select(&o, &h, &c).quality == SNTP_QUALITY_COARSE_RTC);
    assert(!c.initialized);
}
int main(void) {
    test_units_and_aging(); test_uncertainty(); test_phase(); test_frequency();
    test_32k(); test_recovery_and_semantics(); test_continuity(); test_osf(); test_source_sequence();
    puts("RTC holdover tests passed");
}
