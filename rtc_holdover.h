#ifndef RTC_HOLDOVER_H
#define RTC_HOLDOVER_H
#include <stdbool.h>
#include <stdint.h>

/* All thresholds are host-testable and compile-time configurable. No GPIO is
 * assigned to optional hardware by default. Positive frequency ppm means fast. */
#ifndef RTC_32KHZ_GPIO
#define RTC_32KHZ_GPIO (-1)
#endif
#ifndef RTC_PHASE_DISCOVERY_NS
#define RTC_PHASE_DISCOVERY_NS 450000000LL
#endif
#ifndef RTC_PHASE_MAX_OFFSET_NS
#define RTC_PHASE_MAX_OFFSET_NS 450000000LL
#endif
#ifndef RTC_PHASE_MAX_JITTER_NS
#define RTC_PHASE_MAX_JITTER_NS 1000000ULL
#endif
#ifndef RTC_PHASE_OUTLIER_FLOOR_NS
#define RTC_PHASE_OUTLIER_FLOOR_NS 2000000ULL
#endif
#ifndef RTC_PHASE_MIN_SAMPLES
#define RTC_PHASE_MIN_SAMPLES 32u
#endif
#ifndef RTC_FREQUENCY_UNCERTAINTY_PPM
#define RTC_FREQUENCY_UNCERTAINTY_PPM 5.0
#endif
#ifndef RTC_HOLDOVER_MAX_ERROR_NS
#define RTC_HOLDOVER_MAX_ERROR_NS 100000000ULL
#endif
#ifndef RTC_HOLDOVER_MAX_DURATION_S
#define RTC_HOLDOVER_MAX_DURATION_S 86400ULL
#endif
#ifndef RTC_HOLDOVER_MARGIN_NS
#define RTC_HOLDOVER_MARGIN_NS 2000000ULL
#endif
#ifndef RTC_AGING_PPM_PER_LSB
#define RTC_AGING_PPM_PER_LSB 0.1
#endif
#ifndef RTC_AGING_MAX_UNCERTAINTY_PPM
#define RTC_AGING_MAX_UNCERTAINTY_PPM 0.05
#endif
#ifndef GNSS_RECOVERY_GOOD_PULSES
#define GNSS_RECOVERY_GOOD_PULSES 4u
#endif
#ifndef UTC_MAX_SLEW_PPM
#define UTC_MAX_SLEW_PPM 500u
#endif
_Static_assert(UTC_MAX_SLEW_PPM > 0 && UTC_MAX_SLEW_PPM < 1000000u, "Slew must preserve positive clock rate");
_Static_assert(RTC_PHASE_DISCOVERY_NS > 0 && RTC_PHASE_DISCOVERY_NS < 500000000LL, "Discovery must identify an unambiguous nearest second");
_Static_assert(RTC_PHASE_MIN_SAMPLES >= 9, "Qualification must fill the robust phase window");

#define RTC_PHASE_WINDOW 9u
#define RTC_SLOPE_WINDOW 192u
#define RTC_SLOPE_SAMPLE_INTERVAL_NS 120000000000ULL
typedef enum {
    SNTP_QUALITY_NONE = 0,
    SNTP_QUALITY_PPS = 1,
    SNTP_QUALITY_RTC_SQW = 2,
    SNTP_QUALITY_COARSE_GPS = 3,
    SNTP_QUALITY_COARSE_RTC = 4
} sntp_quality_t;
typedef struct {
    int64_t samples_ns[RTC_PHASE_WINDOW];
    uint32_t count, index, sample_count, reject_count, qualifying_samples;
    int64_t offset_ns;
    uint64_t jitter_ns;
    bool qualified;
} rtc_phase_filter_t;
typedef struct {
    uint64_t time_ns[RTC_SLOPE_WINDOW];
    int64_t phase_ns[RTC_SLOPE_WINDOW];
    uint32_t count, index;
    uint64_t last_ns, duration_ns;
    double frequency_error_ppm, uncertainty_ppm;
    bool valid;
} rtc_frequency_filter_t;
typedef struct {
    bool active;
    uint64_t start_ns, initial_error_ns;
    double frequency_uncertainty_ppm;
} rtc_holdover_t;
typedef struct {
    bool initialized;
    uint64_t counter_ns;
    int64_t utc_ns, residual_ns;
    double rate; /* Latest served segment rate, for read-only historical samples. */
} utc_continuity_t;
typedef struct {
    uint64_t last_edge_ns;
    uint32_t good_pulses;
    bool locked;
} gnss_recovery_t;
typedef struct {
    bool valid;
    int64_t unix_sec;
    uint32_t fraction;
    sntp_quality_t quality;
} selected_time_t;
typedef struct {
    uint64_t now_ns;
    uint64_t pps_ns, pps_timeout_ns;
    int64_t pps_epoch_s;
    uint32_t pps_period_ns;
    bool rtc_valid;
    uint64_t rtc_edge_ns, rtc_timeout_ns, rtc_qualified_ns;
    int64_t rtc_epoch_s, rtc_phase_ns;
    uint32_t rtc_period_ns;
    uint64_t rtc_jitter_ns;
    uint64_t rtc_reference_error_ns; /* PPS UTC uncertainty at SQW qualification, no output slew. */
    double rtc_fast_ppm, rtc_uncertainty_ppm;
    bool coarse_valid, coarse_rtc;
    int64_t coarse_epoch_s;
} timing_observation_t;
selected_time_t timing_select(const timing_observation_t *o, rtc_holdover_t *h, utc_continuity_t *c);

bool rtc_phase_observe(rtc_phase_filter_t *f, int64_t phase_ns);
bool rtc_frequency_observe(rtc_frequency_filter_t *f, uint64_t now_ns, int64_t phase_ns);
double rtc_ppm_to_seconds_per_day(double ppm);
int rtc_aging_delta(double fast_ppm, double sensitivity_ppm, unsigned max_step, int current);
uint64_t rtc_holdover_error_ns(const rtc_holdover_t *h, uint64_t now_ns);
bool rtc_holdover_within_policy(const rtc_holdover_t *h, uint64_t now_ns);
int64_t rtc_compensated_elapsed_ns(uint64_t now_ns, uint64_t edge_ns, int64_t phase_ns);
bool rtc_32k_frequency(uint64_t elapsed_ns, uint32_t cycles, uint32_t pps_period_ns, double *fast_ppm);
bool gnss_recovery_observe(gnss_recovery_t *g, uint64_t edge_ns, bool accepted);
int64_t utc_continuity_update(utc_continuity_t *c, uint64_t counter_ns, int64_t target_ns);
uint8_t rtc_status_output_bits(uint8_t status, bool enable_32k);
bool rtc_calendar_status_valid(uint8_t status);
bool timing_edge_fresh(uint64_t now_ns, uint64_t edge_ns, uint64_t timeout_ns);
uint8_t timing_stratum(sntp_quality_t quality);
const char *timing_refid(sntp_quality_t quality);
#endif
