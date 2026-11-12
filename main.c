#include <math.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#if PPS_CAPTURE_DMA
#include "hardware/dma.h"
#endif
#include "hardware/structs/m33.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "pico/unique_id.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"
#include "hardware/watchdog.h"

#include "lwip/ip4_addr.h"
#include "lwip/ip.h"
#include "lwip/netif.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/mem.h"
#include "lwip/stats.h"

#include "pps_capture.pio.h"
#include "tm1637.h"
#include "gps_validation.h"
#include "ubx.h"
#include "timezone_lookup.h"
#include "rtc_holdover.h"
#include "rtc_32k.h"
#include "ntp_uncertainty.h"
#include "pps_capture_timing.h"
#include "generated/timezone_data.h"


#ifndef DEBUG_LOG
#define DEBUG_LOG 0
#endif

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "0.5.2"
#endif

#if DEBUG_LOG
#define DBG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DBG_PRINTF(...) \
    do {               \
    } while (0)
#endif

// UART selection and pins for the L76K GPS module.
// Default Waveshare wiring uses UART0 on GP0 (TX) and GP1 (RX).
// GPS TX -> Pico RX (GP1), GPS RX -> Pico TX (GP0).
#ifndef GPS_UART_ID
#define GPS_UART_ID uart0
#endif

#ifndef GPS_UART_TX_PIN
#define GPS_UART_TX_PIN 0
#endif

#ifndef GPS_UART_RX_PIN
#define GPS_UART_RX_PIN 1
#endif

#ifndef GPS_UART_BAUD
#define GPS_UART_BAUD 115200
#endif

#ifndef GPS_UART_BOOT_BAUD
#define GPS_UART_BOOT_BAUD 9600
#endif

#ifndef GPS_CONFIGURE_MODULE
#define GPS_CONFIGURE_MODULE 1
#endif

#ifndef GPS_PROTOCOL_UBX
#define GPS_PROTOCOL_UBX 0
#endif

#ifndef TIMEZONE_MODE
#define TIMEZONE_MODE "AUTO"
#endif
#ifndef TIMEZONE_MANUAL_NAME
#define TIMEZONE_MANUAL_NAME "UTC"
#endif
#ifndef DISPLAY_TIME_MODE
#define DISPLAY_TIME_MODE "LOCAL"
#endif
#ifndef TIMEZONE_MOVE_THRESHOLD_KM
#define TIMEZONE_MOVE_THRESHOLD_KM 15
#endif
#ifndef TIMEZONE_CANDIDATE_COUNT
#define TIMEZONE_CANDIDATE_COUNT 3
#endif

#ifndef GPS_PPS_PIN
#define GPS_PPS_PIN 16
#endif

#ifndef PPS_CAPTURE_COUNTER_HZ
#define PPS_CAPTURE_COUNTER_HZ 10000000u
#endif

#ifndef PPS_CAPTURE_DMA
#define PPS_CAPTURE_DMA 0
#endif

#ifndef PPS_AB_ALPHA_SHIFT
#define PPS_AB_ALPHA_SHIFT 3u
#endif

#ifndef PPS_AB_BETA_SHIFT
#define PPS_AB_BETA_SHIFT 6u
#endif

#ifndef PPS_MEDIAN_WINDOW
#define PPS_MEDIAN_WINDOW 5u
#endif

#ifndef PPS_OUTLIER_FLOOR_NS
#define PPS_OUTLIER_FLOOR_NS 50000LL
#endif

#ifndef TM1637_ENABLE
#define TM1637_ENABLE 1
#endif

#ifndef TM1637_CLK_PIN
#define TM1637_CLK_PIN 14
#endif

#ifndef TM1637_DIO_PIN
#define TM1637_DIO_PIN 15
#endif

#ifndef TM1637_BRIGHTNESS
#define TM1637_BRIGHTNESS 7
#endif

#ifndef RTC_I2C_ID
#define RTC_I2C_ID i2c0
#endif

#ifndef RTC_SDA_PIN
#define RTC_SDA_PIN 4
#endif

#ifndef RTC_SCL_PIN
#define RTC_SCL_PIN 5
#endif

#ifndef RTC_I2C_BAUD
#define RTC_I2C_BAUD 100000
#endif

#define RTC_ADDR 0x68
#define RTC_CONTROL_REG 0x0E
#define RTC_STATUS_REG 0x0F
#define RTC_AGING_REG 0x10

#ifndef RTC_SQW_PIN
#define RTC_SQW_PIN 17
#endif

#if RTC_32KHZ_GPIO >= 0
_Static_assert(RTC_32KHZ_GPIO < 30, "RTC_32KHZ_GPIO must be an exposed Pico GPIO");
_Static_assert(RTC_32KHZ_GPIO != RTC_SQW_PIN && RTC_32KHZ_GPIO != GPS_PPS_PIN &&
               RTC_32KHZ_GPIO != RTC_SDA_PIN && RTC_32KHZ_GPIO != RTC_SCL_PIN &&
               RTC_32KHZ_GPIO != GPS_UART_TX_PIN && RTC_32KHZ_GPIO != GPS_UART_RX_PIN,
               "RTC 32 kHz GPIO conflicts with timing/I2C/UART wiring");
#endif

#ifndef RTC_SQW_PRESENT_TIMEOUT_US
#define RTC_SQW_PRESENT_TIMEOUT_US 2500000ULL
#endif

#ifndef RTC_SQW_MIN_US
#define RTC_SQW_MIN_US 995000
#endif

#ifndef RTC_SQW_MAX_US
#define RTC_SQW_MAX_US 1005000
#endif

#ifndef RTC_SQW_PLL_FILTER_SHIFT
#define RTC_SQW_PLL_FILTER_SHIFT 6
#endif

#ifndef RTC_SQW_ALIGN_MAX_US
#define RTC_SQW_ALIGN_MAX_US 100000
#endif

#ifndef RTC_SQW_HOLDOVER_MAX_PHASE_US
#define RTC_SQW_HOLDOVER_MAX_PHASE_US 50000
#endif

#ifndef RTC_SQW_GOOD_EDGES
#define RTC_SQW_GOOD_EDGES 4
#endif

#ifndef RTC_SQW_PPS_SETTLE_TICKS
#define RTC_SQW_PPS_SETTLE_TICKS 60
#endif

#ifndef RTC_SQW_REPHASE_INTERVAL_MS
#define RTC_SQW_REPHASE_INTERVAL_MS (5 * 60 * 1000)
#endif

#ifndef RTC_SET_BIAS_MAX_US
#define RTC_SET_BIAS_MAX_US 200000
#endif

#ifndef GPS_HOLDOVER_TIMEOUT_MS
#define GPS_HOLDOVER_TIMEOUT_MS 10000
#endif

#ifndef GPS_QUALITY_MAX_AGE_MS
#define GPS_QUALITY_MAX_AGE_MS 3000
#endif

#ifndef GPS_NMEA_TIME_OFFSET_SEC
#define GPS_NMEA_TIME_OFFSET_SEC 0
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "YourSSID"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "YourPassword"
#endif

#ifndef WIFI_AUTH
#define WIFI_AUTH CYW43_AUTH_WPA2_AES_PSK
#endif

#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 30000
#endif

#ifndef WIFI_RETRY_MIN_MS
#define WIFI_RETRY_MIN_MS 30000
#endif

#ifndef WIFI_RETRY_MAX_MS
#define WIFI_RETRY_MAX_MS (10 * 60 * 1000)
#endif

#ifndef SNTP_MAX_REQUESTS_PER_SECOND
#define SNTP_MAX_REQUESTS_PER_SECOND 20
#endif

#ifndef WIFI_HOSTNAME
#define WIFI_HOSTNAME "ntp1"
#endif

#ifndef WIFI_NO_POWERSAVE
#define WIFI_NO_POWERSAVE 1
#endif

#ifndef WATCHDOG_TIMEOUT_MS
#define WATCHDOG_TIMEOUT_MS 8000
#endif
_Static_assert(WATCHDOG_TIMEOUT_MS > 0 && WATCHDOG_TIMEOUT_MS <= 8388,
               "Watchdog timeout must fit both RP2040 and RP2350 hardware");

#ifndef RTC_I2C_TIMEOUT_US
#define RTC_I2C_TIMEOUT_US 20000u
#endif
_Static_assert(RTC_I2C_TIMEOUT_US > 0 && RTC_I2C_TIMEOUT_US <= 100000u,
               "RTC I2C deadline must be bounded");

#ifndef HEALTH_BROADCAST_ENABLE
#define HEALTH_BROADCAST_ENABLE 0
#endif

#ifndef HEALTH_BROADCAST_PORT
#define HEALTH_BROADCAST_PORT 4040
#endif

#ifndef HEALTH_BROADCAST_CRC
#define HEALTH_BROADCAST_CRC 0
#endif

#ifndef HTTP_SERVER_ENABLE
#define HTTP_SERVER_ENABLE 1
#endif

#ifndef HTTP_SERVER_PORT
#define HTTP_SERVER_PORT 80
#endif













#ifndef GPS_LINE_QUEUE_SIZE
#define GPS_LINE_QUEUE_SIZE 8
#endif

#ifndef HIRES_REANCHOR_US
#define HIRES_REANCHOR_US 500000ULL
#endif

#ifndef RTC_CACHE_POLL_MS
#define RTC_CACHE_POLL_MS 1000
#endif

#ifndef RTC_SYNC_INTERVAL_MS
#define RTC_SYNC_INTERVAL_MS (10 * 60 * 1000)
#endif

#ifndef RTC_SYNC_EXTENDED_MS
#define RTC_SYNC_EXTENDED_MS (60 * 60 * 1000)
#endif

#ifndef RTC_SYNC_MAX_OFFSET_SEC
#define RTC_SYNC_MAX_OFFSET_SEC 0
#endif

#ifndef RMC_PPS_MAX_OFFSET_US
#define RMC_PPS_MAX_OFFSET_US 50000
#endif

#ifndef RMC_PPS_MIN_OFFSET_US
#define RMC_PPS_MIN_OFFSET_US (RMC_PPS_MAX_OFFSET_US / 2)
#endif

#ifndef RMC_PPS_MARGIN_US
#define RMC_PPS_MARGIN_US 5000
#endif

#ifndef RMC_PPS_STD_MULT
#define RMC_PPS_STD_MULT 4
#endif

#ifndef RMC_PPS_EMA_SHIFT
#define RMC_PPS_EMA_SHIFT 3
#endif

#ifndef RMC_MIN_FIX_QUALITY
#define RMC_MIN_FIX_QUALITY 1
#endif

#ifndef RMC_MAX_HDOP
#define RMC_MAX_HDOP 2.5f
#endif

#ifndef RMC_MIN_SATS
#define RMC_MIN_SATS 4
#endif

#ifndef PPS_PLL_FILTER_SHIFT
#define PPS_PLL_FILTER_SHIFT 7
#endif

#ifndef PPS_PLL_MIN_US
#define PPS_PLL_MIN_US 995000
#endif

#ifndef PPS_PLL_MAX_US
#define PPS_PLL_MAX_US 1005000
#endif
#ifndef PPS_JITTER_WINDOW
#define PPS_JITTER_WINDOW 60
#endif
#ifndef PPS_PRESENT_TIMEOUT_US
#define PPS_PRESENT_TIMEOUT_US 1500000ULL
#endif
#ifndef RTC_SYNC_FORCE_SET_SEC
#define RTC_SYNC_FORCE_SET_SEC 5
#endif

#ifndef RTC_SYNC_FORCE_SET_PHASE_US
#define RTC_SYNC_FORCE_SET_PHASE_US 50000
#endif
#ifndef ACCURACY_MODE
#define ACCURACY_MODE 1
#endif
#ifndef ACCURACY_PPS_RMS_US
#define ACCURACY_PPS_RMS_US 3000
#endif
#ifndef ACCURACY_PPS_PEAK_US
#define ACCURACY_PPS_PEAK_US 8000
#endif
#ifndef ACCURACY_PPS_MIN_SAMPLES
#define ACCURACY_PPS_MIN_SAMPLES (PPS_JITTER_WINDOW / 2)
#endif
#ifndef ACCURACY_PLL_FILTER_SHIFT
#define ACCURACY_PLL_FILTER_SHIFT 9
#endif
#ifndef ACCURACY_PLL_SHIFT_RMS_T1
#define ACCURACY_PLL_SHIFT_RMS_T1 200
#endif
#ifndef ACCURACY_PLL_SHIFT_RMS_T2
#define ACCURACY_PLL_SHIFT_RMS_T2 500
#endif
#ifndef ACCURACY_PLL_SHIFT_RMS_T3
#define ACCURACY_PLL_SHIFT_RMS_T3 1500
#endif
#ifndef ACCURACY_RMC_MAX_WINDOW_US
#define ACCURACY_RMC_MAX_WINDOW_US 20000
#endif
#ifndef ACCURACY_RMC_RMS_MULT
#define ACCURACY_RMC_RMS_MULT 6
#endif
#ifndef ACCURACY_RMC_RMS_MARGIN_US
#define ACCURACY_RMC_RMS_MARGIN_US 5000
#endif
#ifndef RMC_PPS_MIN_CONSISTENT
#define RMC_PPS_MIN_CONSISTENT 2
#endif
#ifndef RMC_PPS_BOOTSTRAP_SAMPLES
#define RMC_PPS_BOOTSTRAP_SAMPLES 8
#endif
#ifndef RMC_PPS_BOOTSTRAP_MAX_OFFSET_US
#define RMC_PPS_BOOTSTRAP_MAX_OFFSET_US 250000
#endif

#ifndef RTC_AGING_ENABLE
#define RTC_AGING_ENABLE 1
#endif

#ifndef RTC_AGING_MAX_STEP
#define RTC_AGING_MAX_STEP 4
#endif

#ifndef RTC_AGING_INVERT
#define RTC_AGING_INVERT 0
#endif

#ifndef RTC_AGING_WINDOW_SEC
#define RTC_AGING_WINDOW_SEC (6 * 3600)
#endif

#ifndef RTC_AGING_MIN_SAMPLES
#define RTC_AGING_MIN_SAMPLES 6
#endif

typedef struct {
    bool has_fix;
    int fix_quality;
    int satellites;
    float hdop;
    float pdop;
    float vdop;
    float altitude_m;
    float lat_deg;
    float lon_deg;
    bool lat_valid;
    bool lon_valid;
    float speed_knots;
    bool speed_valid;
    bool rmc_valid;
    uint8_t gsv_gps;
    uint8_t gsv_glo;
    uint8_t gsv_gal;
    uint8_t gsv_bds;
    uint8_t gsv_qzss;
    uint8_t ubx_sat_total;
    uint8_t ubx_sat_used;
    uint8_t ubx_sat_gps;
    uint8_t ubx_sat_galileo;
    uint8_t ubx_sat_beidou;
    uint8_t ubx_sat_glonass;
    uint8_t ubx_sat_qzss;
    uint8_t ubx_sat_sbas;
    uint8_t ubx_used_gps;
    uint8_t ubx_used_galileo;
    uint8_t ubx_used_beidou;
    uint8_t ubx_used_glonass;
    uint8_t ubx_used_qzss;
    uint8_t ubx_used_sbas;
    uint8_t gsa_fix_gps;
    uint8_t gsa_fix_glo;
    uint8_t gsa_fix_gal;
    uint8_t gsa_fix_bds;
    uint8_t gsa_fix_gn;
    uint8_t nmea_used_gps;
    uint8_t nmea_used_glo;
    uint8_t nmea_used_gal;
    uint8_t nmea_used_bds;
    uint8_t nmea_used_gn;
    char time_utc[7];  // hhmmss
    char date[7];      // ddmmyy
} gps_stats_t;

typedef struct {
    bool valid;
    bool using_rtc;
    int hour;
    int min;
    int sec;
    int date;
    int month;
    int year;
} time_source_t;

typedef struct {
    timezone_id_t current_id;
    timezone_id_t candidate_id;
    uint8_t candidate_count;
    bool lookup_valid;
    bool source_gps;
    int32_t offset_seconds;
    bool is_dst;
    int32_t last_lat_e7;
    int32_t last_lon_e7;
    uint64_t last_lookup_utc;
    bool rules_out_of_range;
} timezone_state_t;

static timezone_state_t timezone_state = {0};

typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_SOLID = 1,
    LED_MODE_BLINK = 2
} led_mode_t;

typedef enum {
    CLOCK_STATE_UNLOCKED = 0,
    CLOCK_STATE_ACQUIRE = 1,
    CLOCK_STATE_LOCKED = 2,
    CLOCK_STATE_HOLDOVER = 3
} clock_state_t;

typedef enum {
    RTC_STATE_MISSING = 0,
    RTC_STATE_INIT = 1,
    RTC_STATE_TRACKING = 2,
    RTC_STATE_LOCKED = 3,
    RTC_STATE_HOLDOVER = 4,
    RTC_STATE_FAULT = 5
} rtc_state_t;

typedef struct {
    bool valid;
    int64_t base_epoch;
    double sum_x;
    double sum_y;
    double sum_xx;
    double sum_xy;
    int64_t last_epoch;
    uint32_t samples;
} drift_estimator_t;

typedef struct {
    bool valid;
    bool status;
    int hour;
    int min;
    int sec;
    int dow;
    int date;
    int month;
    int year;
    absolute_time_t sample_time;
} rtc_cache_t;

#ifndef RTC_LOCK_GOOD_COUNT
#define RTC_LOCK_GOOD_COUNT 6
#endif

#ifndef RTC_FAULT_BAD_COUNT
#define RTC_FAULT_BAD_COUNT 3
#endif

static bool parse_hhmmss(const char *raw, int *hour, int *min, int *sec);
static uint64_t median_u64(const uint64_t *values, uint32_t count);
static uint64_t ab_mad_ns(void);
static ntp_uncertainty_t get_pps_uncertainty_locked(uint64_t now_ns);
static uint64_t ab_outlier_threshold_ns(void);
static bool parse_ddmmyy(const char *raw, int *date, int *month, int *year);
static bool rmc_within_pps_window(uint64_t rmc_us);
static uint64_t get_last_pps_time_ns(void);
static uint64_t get_last_rtc_sqw_second_time_ns(void);
static uint64_t hires_time_ns(void);
static bool get_sntp_time_at(uint64_t now_ns, uint32_t *seconds, uint32_t *fraction,
                             bool *synced, sntp_quality_t *quality);
static bool drift_estimator_slope_us_per_sec(const drift_estimator_t *est,
                                             double *slope_us_per_sec,
                                             int64_t *elapsed_sec);
static bool rtc_read_reg(uint8_t reg, uint8_t *value);
static void rtc_invalidate_mapping_locked(void);
static void get_ip_string(char *out, size_t out_len);

static mutex_t stats_mutex;
static mutex_t rtc_i2c_mutex;
static uint32_t network_core_stack[2048] __aligned(8);
static absolute_time_t last_gps_fix_time;
static bool have_gps_time = false;
static char last_rtc_set_time[7] = {0};
static char last_rtc_set_date[7] = {0};
static absolute_time_t last_rtc_sync_time;
static bool have_rtc_sync = false;
static int64_t rtc_sync_interval_ms = RTC_SYNC_INTERVAL_MS;
static absolute_time_t last_aging_adjust_time;
static gps_stats_t g_stats = {0};
static time_source_t g_time_snapshot = {0};
static absolute_time_t last_time_snapshot_update;
static led_mode_t g_led_mode = LED_MODE_OFF;
static bool g_led_state = false;
static bool pps_indicator_phase = false;
static int64_t last_pps_us = -1;
static uint64_t last_pps_time_ns = 0;
static volatile bool gps_pps_seen = false;
static volatile bool gps_pps_tick = false;
static volatile uint32_t gps_pps_capture_ticks = 0;
static volatile bool gps_pps_capture_valid = false;
static uint32_t pps_dma_overruns = 0;
static volatile bool pps_capture_fault_pending = false;
static volatile uint32_t pps_capture_faults = 0;
static uint pps_program_offset;
static uint64_t pps_capture_remainder;
static uint64_t pps_capture_last_poll_ns;
static uint64_t pps_capture_start_ns;
static bool pps_capture_start_valid;
#if PPS_CAPTURE_DMA
#define PPS_DMA_RING_BITS 4u
#define PPS_DMA_RING_SIZE (1u << PPS_DMA_RING_BITS)
static volatile uint32_t pps_dma_ring[PPS_DMA_RING_SIZE] __aligned(64);
static int pps_dma_channel = -1;
static uint32_t pps_dma_consumer = 0;
#endif
static int64_t last_rtc_sqw_us = -1;
static uint64_t last_rtc_sqw_time_ns = 0;
static volatile bool rtc_sqw_seen = false;
static volatile bool rtc_sqw_tick = false;
static absolute_time_t last_rmc_update_time;
static uint64_t last_gga_update_ns = 0;
static bool pps_reported = false;
static bool rtc_sqw_reported = false;
static bool pps_time_valid = false;
static int64_t pps_epoch_seconds = 0;
static bool rmc_epoch_valid = false;
static uint8_t rmc_phase_ok_count = 0;
static int64_t rmc_epoch_seconds = 0;
static uint64_t rmc_epoch_time_ns = 0;
static bool pll_valid = false;
static uint32_t pll_interval_ns = 1000000000u;
static uint64_t prev_pps_time_ns = 0;
static uint32_t prev_pps_capture_ticks = 0;
static bool prev_pps_capture_valid = false;
static bool rtc_sqw_pll_valid = false;
static uint32_t rtc_sqw_interval_ns = 1000000000u;
static uint64_t prev_rtc_sqw_time_ns = 0;
static uint64_t last_rtc_sqw_second_time_ns = 0;
static volatile bool rtc_sqw_edge_high = false;
static bool rtc_sqw_second_edge_known = false;
static bool rtc_sqw_second_edge_high = true;
static bool rtc_sqw_time_valid = false;
static uint8_t rtc_sqw_phase_good_count = 0;
static int64_t rtc_sqw_epoch_seconds = 0;
static bool rtc_sqw_phase_valid = false;
static int32_t rtc_sqw_phase_last_us = 0;
static absolute_time_t last_rtc_rephase_time;
static bool rtc_pending_set_valid = false;
static int64_t rtc_pending_set_epoch = 0;
static bool rtc_pending_set_phase = false;
static uint64_t rtc_pending_set_due_ns = 0;
static bool rtc_set_bias_cal_pending = false;
static bool rtc_aging_valid = false;
static bool rtc_osf_at_boot = true;
static bool rtc_osf_boot_read = false;
static bool rtc_calendar_valid = false;
static bool rtc_calendar_needs_set = true;
static bool rtc_conversion_pending = false;
static bool rtc_conversion_started = false;
static uint64_t rtc_conversion_poll_ns = 0;
static uint64_t rtc_conversion_deadline_ns = 0;
static bool rtc_conversion_failed = false;
static bool rtc_temperature_valid = false;
static int16_t rtc_temperature_quarters = 0;
static rtc_phase_filter_t rtc_phase_filter;
static rtc_frequency_filter_t rtc_frequency_filter;
static rtc_holdover_t rtc_holdover;
static rtc_32k_t rtc_32k;
static utc_continuity_t utc_continuity;
static gnss_recovery_t gnss_recovery;
static uint64_t trusted_pps_ns = 0;
static uint64_t rtc_reference_error_ns = 0;
static ntp_edge_bracket_t pps_placement_brackets[2]; /* stats_mutex */
static int64_t trusted_pps_epoch_s = 0;
static uint32_t trusted_pps_period_ns = 1000000000u;
static uint64_t rtc_last_gnss_observation_ns = 0;
static uint64_t rtc_last_qualified_ns = 0;
static int64_t rtc_last_qualified_epoch_s = 0;
static int64_t rtc_holdover_phase_ns = 0;
static double rtc_holdover_fast_ppm = 0;
static bool rtc_holdover_frequency_valid = false;
static double rtc_learned_uncertainty_ppm = RTC_FREQUENCY_UNCERTAINTY_PPM;
static uint32_t rtc_bad_phase_count = 0;
static uint32_t rtc_phase_reject_total = 0;
static int64_t rtc_phase_error_ns = 0;
static const char *rtc_last_reject_reason = "NONE";
static uint32_t timing_source_changes = 0;
static sntp_quality_t timing_previous_quality = SNTP_QUALITY_NONE;
static int8_t rtc_aging_value = 0;
static rtc_state_t g_rtc_state = RTC_STATE_INIT;
static uint32_t rtc_track_good_count = 0;
static uint32_t rtc_track_bad_count = 0;
static uint32_t rtc_sqw_rephase_count = 0;
static uint32_t rtc_calendar_write_count = 0;
static int32_t pps_jitter_buf[PPS_JITTER_WINDOW];
static int64_t pps_jitter_ns_buf[PPS_JITTER_WINDOW];
static uint32_t pps_jitter_count = 0;
static uint32_t pps_jitter_index = 0;
static int32_t pps_jitter_last = 0;
static int64_t pps_jitter_last_ns = 0;
static bool pps_jitter_valid = false;
static int64_t pps_jitter_sumsq = 0;
static int64_t pps_jitter_ns_sumsq = 0;
static uint32_t pps_jitter_rms_us = 0;
static int64_t pll_phase_error_ns = 0;
static int64_t ab_innovation_ns = 0;
static int64_t pps_period_error_last_ns = 0;
static bool accuracy_mode = false;
static uint32_t pps_tick_count = 0;
static uint8_t pll_filter_shift = PPS_PLL_FILTER_SHIFT;
static uint64_t ab_period_ns = 1000000000ULL;
static int64_t ab_period_rate_ns_per_sec = 0;
static bool ab_servo_valid = false;
static uint64_t ab_interval_history[PPS_MEDIAN_WINDOW];
static uint32_t ab_interval_history_count = 0;
static uint32_t ab_interval_history_index = 0;
static uint32_t pps_outliers_rejected = 0;
static int32_t rmc_pps_window_us = RMC_PPS_MAX_OFFSET_US;
static int32_t rmc_pps_avg_us = 0;
static int64_t rmc_pps_var_us2 = 0;
static uint32_t rmc_pps_samples = 0;
static drift_estimator_t rtc_drift_estimator = {0};
static drift_estimator_t rtc_phase_estimator = {0};
#if HEALTH_BROADCAST_ENABLE
static uint32_t health_nonce_counter = 0;
#endif
#if HEALTH_BROADCAST_ENABLE
static uint32_t health_fmt_drop_count = 0;
static uint32_t health_alloc_fail_count = 0;
static uint32_t health_send_err_count = 0;
static uint32_t health_rtc_busy_count = 0;
#endif
static volatile uint32_t gps_checksum_error_count = 0;
static volatile uint32_t gps_rejected_fix_count = 0;
static volatile uint32_t sntp_request_count = 0;
static volatile uint32_t sntp_malformed_count = 0;
static volatile uint32_t sntp_rate_limit_count = 0;
static volatile uint32_t sntp_send_error_count = 0;
static volatile uint32_t http_request_count = 0;
static volatile uint32_t http_error_count = 0;
static volatile uint32_t wifi_reconnect_count = 0;
static volatile uint32_t wifi_reconnect_fail_count = 0;
static uint64_t sntp_rate_window_start_us = 0;
static uint32_t sntp_rate_window_count = 0;
static volatile bool time_snapshot_dirty = false;
static rtc_cache_t g_rtc_cache = {0};
static absolute_time_t last_rtc_cache_poll;
static volatile uint8_t gps_line_head = 0;
static volatile uint8_t gps_line_tail = 0;
static char gps_line_build[128];
static size_t gps_line_build_len = 0;
static uint64_t gps_line_first_byte_ns = 0;
static bool gps_line_overflow = false;
static volatile uint32_t gps_line_drop_count = 0;
static ubx_parser_t ubx_parser;
static volatile uint8_t ubx_frame_head = 0;
static volatile uint8_t ubx_frame_tail = 0;
static ubx_frame_t ubx_frame_queue[GPS_LINE_QUEUE_SIZE];
static volatile uint32_t ubx_frame_drop_count = 0;
static uint32_t hires_cycles_per_sec = 0;
static uint32_t hires_reanchor_cycles = 0;
static bool hires_time_enabled = false;

typedef struct {
    uint64_t anchor_us;
    uint64_t last_ns;
    uint32_t anchor_cycles;
} hires_core_state_t;

static hires_core_state_t hires_clock_state[2];

typedef struct {
    char text[128];
    uint64_t first_byte_ns;
} gps_line_t;

static gps_line_t gps_line_queue[GPS_LINE_QUEUE_SIZE];

static struct udp_pcb *sntp_pcb = NULL;
static bool wifi_driver_ready = false;
#if HTTP_SERVER_ENABLE
static struct tcp_pcb *http_pcb = NULL;
#endif
#if HEALTH_BROADCAST_ENABLE
static struct udp_pcb *health_pcb = NULL;
#endif

#define NTP_PORT 123
#define NTP_DELTA 2208988800UL

static const char *clock_state_str(clock_state_t state) {
    switch (state) {
        case CLOCK_STATE_LOCKED:
            return "LOCKED";
        case CLOCK_STATE_ACQUIRE:
            return "ACQUIRE";
        case CLOCK_STATE_HOLDOVER:
            return "HOLDOVER";
        default:
            return "UNLOCKED";
    }
}

static const char *rtc_state_str(rtc_state_t state) {
    switch (state) {
        case RTC_STATE_MISSING:
            return "MISSING";
        case RTC_STATE_INIT:
            return "INIT";
        case RTC_STATE_TRACKING:
            return "TRACKING";
        case RTC_STATE_LOCKED:
            return "LOCKED";
        case RTC_STATE_HOLDOVER:
            return "HOLDOVER";
        default:
            return "FAULT";
    }
}

static const char *rtc_trust_str(rtc_state_t state, bool status, bool sqw_time_valid) {
    if (!status || state == RTC_STATE_MISSING || state == RTC_STATE_FAULT) {
        return state == RTC_STATE_FAULT ? "FAULT" : "MISSING";
    }
    if (state == RTC_STATE_HOLDOVER && sqw_time_valid) {
        return "TRUSTED_HOLDOVER";
    }
    if (state == RTC_STATE_INIT) {
        return "PRESENT";
    }
    if (state == RTC_STATE_TRACKING || state == RTC_STATE_LOCKED) {
        return "TRACKING";
    }
    return "PRESENT";
}

static const char *sntp_quality_str(sntp_quality_t quality) {
    switch (quality) {
        case SNTP_QUALITY_PPS:
            return "PPS";
        case SNTP_QUALITY_RTC_SQW:
            return "RTC_SQW";
        case SNTP_QUALITY_COARSE_GPS:
            return "GPS_COARSE";
        case SNTP_QUALITY_COARSE_RTC:
            return "RTC_COARSE";
        default:
            return "NONE";
    }
}

static uint8_t sntp_quality_stratum(sntp_quality_t quality) {
    return timing_stratum(quality);
}

static void sntp_quality_refid(sntp_quality_t quality, char out[5]) {
    if (out) memcpy(out, timing_refid(quality), 5);
}

static uint8_t bcd_to_dec(uint8_t bcd) {
    return (uint8_t)(((bcd >> 4) * 10) + (bcd & 0x0F));
}

static uint8_t dec_to_bcd(uint8_t dec) {
    return (uint8_t)(((dec / 10) << 4) | (dec % 10));
}

static int day_of_week(int year, int month, int day) {
    static const int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    if (month < 3) {
        year -= 1;
    }
    int dow = (year + year / 4 - year / 100 + year / 400 + t[month - 1] + day) % 7;
    return dow + 1; // 1=Sunday, 7=Saturday
}

static int64_t days_from_civil(int y, unsigned m, unsigned d) {
    y -= m <= 2;
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned month = (unsigned)((int)m + (m > 2 ? -3 : 9));
    const unsigned doy = (153 * month + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (int64_t)(era * 146097 + (int)doe - 719468);
}

static int64_t utc_to_epoch(int year, int month, int day, int hour, int min, int sec) {
    int64_t days = days_from_civil(year, (unsigned)month, (unsigned)day);
    return days * 86400 + (int64_t)hour * 3600 + (int64_t)min * 60 + sec;
}

static void epoch_to_utc(int64_t unix_sec, int *year, int *month, int *day,
                         int *hour, int *min, int *sec) {
    if (!year || !month || !day || !hour || !min || !sec) {
        return;
    }
    int64_t z = unix_sec / 86400 + 719468;
    int64_t r = unix_sec % 86400;
    if (r < 0) {
        r += 86400;
        z -= 1;
    }
    int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    unsigned doe = (unsigned)(z - era * 146097);
    unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int y = (int)yoe + (int)era * 400;
    unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    unsigned mp = (5 * doy + 2) / 153;
    unsigned d = doy - (153 * mp + 2) / 5 + 1;
    unsigned m = mp + (mp < 10 ? 3 : -9);
    y += (m <= 2);
    *year = y;
    *month = (int)m;
    *day = (int)d;
    *hour = (int)(r / 3600);
    *min = (int)((r % 3600) / 60);
    *sec = (int)(r % 60);
}

static bool gps_fields_to_adjusted_epoch(const char *time_raw, const char *date_raw,
                                         int64_t *epoch) {
    int hour = 0;
    int min = 0;
    int sec = 0;
    int day = 0;
    int month = 0;
    int year = 0;
    if (!epoch) {
        return false;
    }
    if (!parse_hhmmss(time_raw, &hour, &min, &sec) ||
        !parse_ddmmyy(date_raw, &day, &month, &year)) {
        return false;
    }
    *epoch = utc_to_epoch(year, month, day, hour, min, sec) + GPS_NMEA_TIME_OFFSET_SEC;
    return true;
}

static bool gps_stats_to_adjusted_utc(const gps_stats_t *stats,
                                      int *year, int *month, int *day,
                                      int *hour, int *min, int *sec) {
    int64_t epoch = 0;
    if (!stats || !year || !month || !day || !hour || !min || !sec) {
        return false;
    }
    if (!gps_fields_to_adjusted_epoch(stats->time_utc, stats->date, &epoch)) {
        return false;
    }
    epoch_to_utc(epoch, year, month, day, hour, min, sec);
    return true;
}

static void format_local_time(int64_t unix_sec, char *out, size_t out_len) {
    if (!out || out_len == 0 || unix_sec == 0) {
        if (out && out_len) snprintf(out, out_len, "N/A");
        return;
    }
    int y = 0, mo = 0, d = 0, h = 0, mi = 0, s = 0;
    epoch_to_utc(unix_sec, &y, &mo, &d, &h, &mi, &s);
    snprintf(out, out_len, "%04d-%02d-%02dT%02d:%02d:%02d", y, mo, d, h, mi, s);
}

static int days_in_month(int year, int month) {
    static const int days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    int d = days[month - 1];
    if (month == 2) {
        bool leap = ((year % 4) == 0 && (year % 100) != 0) || (year % 400) == 0;
        if (leap) d = 29;
    }
    return d;
}

static bool bcd_value_valid(uint8_t raw, uint8_t max) {
    uint8_t value = bcd_to_dec(raw);
    return ((raw & 0x0Fu) <= 9u) && ((raw >> 4) <= 9u) && value <= max;
}

static void increment_utc_one_second(gps_stats_t *stats) {
    int hour = 0, min = 0, sec = 0;
    int date = 0, month = 0, year = 0;
    if (!parse_hhmmss(stats->time_utc, &hour, &min, &sec)) {
        return;
    }
    if (!parse_ddmmyy(stats->date, &date, &month, &year)) {
        return;
    }
    sec += 1;
    if (sec >= 60) {
        sec = 0;
        min += 1;
    }
    if (min >= 60) {
        min = 0;
        hour += 1;
    }
    if (hour >= 24) {
        hour = 0;
        date += 1;
        int dim = days_in_month(year, month);
        if (date > dim) {
            date = 1;
            month += 1;
            if (month > 12) {
                month = 1;
                year += 1;
            }
        }
    }
    /* Explicit post-rollover bounds also keep format diagnostics meaningful. */
    if (hour < 0 || hour > 23 || min < 0 || min > 59 || sec < 0 || sec > 59) return;
    snprintf(stats->time_utc, sizeof(stats->time_utc), "%02d%02d%02d", hour, min, sec);
    snprintf(stats->date, sizeof(stats->date), "%02d%02d%02d", date, month, year % 100);
}

static void update_led_mode_locked(int64_t now_us) {
    bool pps_locked = pps_time_valid && pll_valid &&
        last_pps_us >= 0 && ((now_us - last_pps_us) < 1500000);
    bool rtc_fallback = g_time_snapshot.valid && g_time_snapshot.using_rtc;

    if (pps_locked) {
        g_led_mode = LED_MODE_BLINK;
    } else if (rtc_fallback) {
        g_led_mode = LED_MODE_SOLID;
        g_led_state = true;
    } else {
        g_led_mode = LED_MODE_OFF;
        g_led_state = false;
    }
}


static void rtc_init(void) {
    i2c_init(RTC_I2C_ID, RTC_I2C_BAUD);
    gpio_set_function(RTC_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(RTC_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(RTC_SDA_PIN);
    gpio_pull_up(RTC_SCL_PIN);
}

/* Caller holds rtc_i2c_mutex. Use open-drain GPIO recovery: never drive high,
 * and bound clock-stretch waits even when the peripheral holds SCL low. */
static void rtc_i2c_recover(void) {
    i2c_deinit(RTC_I2C_ID);
    gpio_init(RTC_SDA_PIN);
    gpio_init(RTC_SCL_PIN);
    gpio_put(RTC_SDA_PIN, false);
    gpio_put(RTC_SCL_PIN, false);
    gpio_set_dir(RTC_SDA_PIN, GPIO_IN);
    gpio_set_dir(RTC_SCL_PIN, GPIO_IN);
    gpio_pull_up(RTC_SDA_PIN);
    gpio_pull_up(RTC_SCL_PIN);
    for (unsigned pulse = 0; pulse < 9; ++pulse) {
        gpio_set_dir(RTC_SCL_PIN, GPIO_OUT);
        sleep_us(5);
        gpio_set_dir(RTC_SCL_PIN, GPIO_IN);
        unsigned wait = 0;
        while (!gpio_get(RTC_SCL_PIN) && wait++ < 100) sleep_us(1);
        if (!gpio_get(RTC_SCL_PIN)) break;
        sleep_us(5);
    }
    /* Attempt STOP; a permanently stuck bus will still fail the next deadline. */
    gpio_set_dir(RTC_SCL_PIN, GPIO_OUT);
    gpio_set_dir(RTC_SDA_PIN, GPIO_OUT);
    sleep_us(5);
    gpio_set_dir(RTC_SCL_PIN, GPIO_IN);
    sleep_us(5);
    gpio_set_dir(RTC_SDA_PIN, GPIO_IN);
    rtc_init();
}

static int rtc_i2c_write(uint8_t address, const uint8_t *data, size_t len, bool nostop) {
    int result = i2c_write_timeout_us(RTC_I2C_ID, address, data, len, nostop, RTC_I2C_TIMEOUT_US);
    if (result == PICO_ERROR_TIMEOUT) rtc_i2c_recover();
    return result;
}

static int rtc_i2c_read(uint8_t address, uint8_t *data, size_t len, bool nostop) {
    int result = i2c_read_timeout_us(RTC_I2C_ID, address, data, len, nostop, RTC_I2C_TIMEOUT_US);
    if (result == PICO_ERROR_TIMEOUT) rtc_i2c_recover();
    return result;
}

static bool rtc_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    mutex_enter_blocking(&rtc_i2c_mutex);
    int wrote = rtc_i2c_write(RTC_ADDR, buf, 2, false);
    mutex_exit(&rtc_i2c_mutex);
    return wrote == 2;
}


static bool rtc_configure_square_wave_1hz(void) {
    uint8_t control = 0;
    uint8_t status = 0;
    if (!rtc_read_reg(RTC_CONTROL_REG, &control)) {
        return false;
    }
    if (!rtc_read_reg(RTC_STATUS_REG, &status)) {
        return false;
    }

    uint8_t next_control = (uint8_t)((control & 0x20u) | 0x40u);
    uint8_t next_status = rtc_status_output_bits(status, RTC_32KHZ_GPIO >= 0);

    if (next_control != control && !rtc_write_reg(RTC_CONTROL_REG, next_control)) {
        return false;
    }
    if (next_status != status && !rtc_write_reg(RTC_STATUS_REG, next_status)) {
        return false;
    }
    return true;
}

static bool rtc_read_reg(uint8_t reg, uint8_t *value) {
    mutex_enter_blocking(&rtc_i2c_mutex);
    if (rtc_i2c_write(RTC_ADDR, &reg, 1, true) != 1) {
        mutex_exit(&rtc_i2c_mutex);
        return false;
    }
    int read = rtc_i2c_read(RTC_ADDR, value, 1, false);
    mutex_exit(&rtc_i2c_mutex);
    return read == 1;
}

static bool rtc_read_aging(int8_t *value) {
    uint8_t raw = 0;
    if (!rtc_read_reg(RTC_AGING_REG, &raw)) {
        return false;
    }
    *value = (int8_t)raw;
    mutex_enter_blocking(&stats_mutex);
    rtc_aging_value = *value;
    rtc_aging_valid = true;
    mutex_exit(&stats_mutex);
    return true;
}

static bool rtc_write_aging(int8_t value) {
    bool ok = rtc_write_reg(RTC_AGING_REG, (uint8_t)value);
    if (ok) {
        mutex_enter_blocking(&stats_mutex);
        rtc_aging_value = value;
        rtc_aging_valid = true;
        mutex_exit(&stats_mutex);
    }
    return ok;
}

static bool rtc_read_time(int *hour, int *min, int *sec,
                          int *dow, int *date, int *month, int *year) {
    if (rtc_calendar_needs_set) return false;
    uint8_t reg = 0x00;
    uint8_t buf[7];

    mutex_enter_blocking(&rtc_i2c_mutex);
    if (rtc_i2c_write(RTC_ADDR, &reg, 1, true) != 1) {
        mutex_exit(&rtc_i2c_mutex);
        return false;
    }
    if (rtc_i2c_read(RTC_ADDR, buf, 7, false) != 7) {
        mutex_exit(&rtc_i2c_mutex);
        return false;
    }
    mutex_exit(&rtc_i2c_mutex);

    uint8_t status = 0;
    if (!rtc_read_reg(RTC_STATUS_REG, &status)) return false;
    if (!rtc_calendar_status_valid(status)) {
        rtc_calendar_needs_set = true;
        return false;
    }
    if (!bcd_value_valid(buf[0] & 0x7F, 59) ||
        !bcd_value_valid(buf[1] & 0x7F, 59) ||
        !bcd_value_valid(buf[2] & 0x3F, 23) ||
        !bcd_value_valid(buf[3] & 0x07, 7) ||
        !bcd_value_valid(buf[4] & 0x3F, 31) ||
        !bcd_value_valid(buf[5] & 0x1F, 12) ||
        !bcd_value_valid(buf[6], 99)) {
        return false;
    }
    *sec = bcd_to_dec(buf[0] & 0x7F);
    *min = bcd_to_dec(buf[1] & 0x7F);
    *hour = bcd_to_dec(buf[2] & 0x3F);
    *dow = bcd_to_dec(buf[3] & 0x07);
    *date = bcd_to_dec(buf[4] & 0x3F);
    *month = bcd_to_dec(buf[5] & 0x1F);
    *year = 2000 + bcd_to_dec(buf[6]);
    if (*date < 1 || *month < 1 || *dow < 1 || *date > days_in_month(*year, *month)) {
        return false;
    }
    return true;
}

static bool rtc_set_time(int hour, int min, int sec,
                         int dow, int date, int month, int year) {
    uint8_t buf[8];
    buf[0] = 0x00;
    buf[1] = dec_to_bcd((uint8_t)sec);
    buf[2] = dec_to_bcd((uint8_t)min);
    buf[3] = dec_to_bcd((uint8_t)hour);
    buf[4] = dec_to_bcd((uint8_t)dow);
    buf[5] = dec_to_bcd((uint8_t)date);
    buf[6] = dec_to_bcd((uint8_t)month);
    buf[7] = dec_to_bcd((uint8_t)(year % 100));

    mutex_enter_blocking(&rtc_i2c_mutex);
    int wrote = rtc_i2c_write(RTC_ADDR, buf, 8, false);
    mutex_exit(&rtc_i2c_mutex);
    if (wrote == 8) {
        mutex_enter_blocking(&stats_mutex);
        if (rtc_calendar_write_count < UINT32_MAX) ++rtc_calendar_write_count;
        mutex_exit(&stats_mutex);
    }
    return wrote == 8;
}

static void gps_uart_apply_baud(uint32_t baud) {
    uart_init(GPS_UART_ID, baud);
    gpio_set_function(GPS_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(GPS_UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(GPS_UART_ID, false, false);
    uart_set_fifo_enabled(GPS_UART_ID, false);
}

static void gps_uart_send_nmea_body(const char *body) {
    uint8_t checksum = 0;
    if (!body || !*body) {
        return;
    }
    uart_putc_raw(GPS_UART_ID, '$');
    for (const char *p = body; *p; ++p) {
        checksum ^= (uint8_t)(*p);
        uart_putc_raw(GPS_UART_ID, (uint8_t)(*p));
    }
    uart_putc_raw(GPS_UART_ID, '*');
    static const char hex[] = "0123456789ABCDEF";
    uart_putc_raw(GPS_UART_ID, hex[(checksum >> 4) & 0x0F]);
    uart_putc_raw(GPS_UART_ID, hex[checksum & 0x0F]);
    uart_putc_raw(GPS_UART_ID, '\r');
    uart_putc_raw(GPS_UART_ID, '\n');
}

#if GPS_PROTOCOL_UBX
static void gps_uart_send_ubx_cfg_msg(uint8_t msg_class, uint8_t msg_id) {
    // CFG-MSG payload: message class/id followed by the output rate for
    // I2C, UART1, UART2, USB, SPI, and reserved output ports.  The F10N's
    // host UART is UART1, so enable the selected message at 1 Hz there.
    static const uint8_t payload[] = {
        0, 0, 0, 0, 0, 0, 0, 0
    };
    // UBX frame = sync(2) + class/id(2) + length(2) + payload(8)
    // + checksum(2) = 16 bytes.  Do not transmit bytes beyond the
    // checksum; they would be interpreted as unrelated UART traffic.
    uint8_t frame[6 + sizeof(payload) + 2];
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    frame[0] = 0xB5;
    frame[1] = 0x62;
    frame[2] = 0x06;
    frame[3] = 0x01;
    frame[4] = sizeof(payload);
    frame[5] = 0;
    frame[6] = msg_class;
    frame[7] = msg_id;
    frame[8] = 0; // I2C
    frame[9] = 1; // UART1
    frame[10] = 0; // UART2
    frame[11] = 0; // USB
    frame[12] = 0; // SPI
    frame[13] = 0; // reserved
    for (size_t i = 2; i < 14; ++i) {
        ck_a = (uint8_t)(ck_a + frame[i]);
        ck_b = (uint8_t)(ck_b + ck_a);
    }
    frame[14] = ck_a;
    frame[15] = ck_b;
    for (size_t i = 0; i < sizeof(frame); ++i) {
        uart_putc_raw(GPS_UART_ID, frame[i]);
    }
    uart_tx_wait_blocking(GPS_UART_ID);
    sleep_ms(100);
}
#endif

static int gps_uart_baud_code(uint32_t baud) {
    switch (baud) {
    case 4800: return 0;
    case 9600: return 1;
    case 19200: return 2;
    case 38400: return 3;
    case 57600: return 4;
    case 115200: return 5;
    default: return -1;
    }
}

static void gps_uart_trim_output_at_current_baud(void) {
    gps_uart_send_nmea_body("PCAS02,1000");
    gps_uart_send_nmea_body("PCAS03,1,0,1,0,1,0,0,0,0,0,,,0,0");
    uart_tx_wait_blocking(GPS_UART_ID);
}

static void gps_uart_configure_module(void) {
#if GPS_PROTOCOL_UBX
    gps_uart_apply_baud(GPS_UART_BAUD);
#if GPS_CONFIGURE_MODULE
    sleep_ms(100);
    gps_uart_send_ubx_cfg_msg(0x01, 0x07); // UBX-NAV-PVT at 1 Hz
    gps_uart_send_ubx_cfg_msg(0x0D, 0x01); // UBX-TIM-TP at 1 Hz
    gps_uart_send_ubx_cfg_msg(0x01, 0x35); // UBX-NAV-SAT at 1 Hz
#endif
#elif GPS_CONFIGURE_MODULE
    gps_uart_apply_baud(GPS_UART_BOOT_BAUD);
    sleep_ms(200);
    gps_uart_trim_output_at_current_baud();
    sleep_ms(50);
    if (GPS_UART_BAUD != GPS_UART_BOOT_BAUD) {
        int baud_code = gps_uart_baud_code(GPS_UART_BAUD);
        if (baud_code >= 0) {
            char baud_cmd[16];
            snprintf(baud_cmd, sizeof(baud_cmd), "PCAS01,%d", baud_code);
            gps_uart_send_nmea_body(baud_cmd);
            uart_tx_wait_blocking(GPS_UART_ID);
            sleep_ms(100);
        }
        gps_uart_apply_baud(GPS_UART_BAUD);
        sleep_ms(100);
    }
    gps_uart_trim_output_at_current_baud();
    sleep_ms(50);
#else
    gps_uart_apply_baud(GPS_UART_BAUD);
#endif
}

static void gps_uart_irq_handler(void) {
    while (uart_is_readable(GPS_UART_ID)) {
        uint8_t byte = (uint8_t)uart_getc(GPS_UART_ID);
#if GPS_PROTOCOL_UBX
        ubx_frame_t frame;
        if (ubx_parser_feed(&ubx_parser, byte, hires_time_ns(), &frame)) {
            uint8_t head = ubx_frame_head;
            uint8_t next = (uint8_t)((head + 1u) % GPS_LINE_QUEUE_SIZE);
            if (next != ubx_frame_tail) {
                ubx_frame_queue[head] = frame;
                ubx_frame_head = next;
            } else {
                __atomic_add_fetch(&ubx_frame_drop_count, 1, __ATOMIC_RELAXED);
            }
        }
        continue;
#else
        char c = (char)byte;
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            if (!gps_line_overflow && gps_line_build_len > 0) {
                uint8_t head = gps_line_head;
                uint8_t next = (uint8_t)((head + 1u) % GPS_LINE_QUEUE_SIZE);
                if (next != gps_line_tail) {
                    gps_line_queue[head].first_byte_ns = gps_line_first_byte_ns;
                    memcpy(gps_line_queue[head].text, gps_line_build, gps_line_build_len);
                    gps_line_queue[head].text[gps_line_build_len] = '\0';
                    gps_line_head = next;
                } else {
                    __atomic_add_fetch(&gps_line_drop_count, 1, __ATOMIC_RELAXED);
                }
            }
            gps_line_build_len = 0;
            gps_line_first_byte_ns = 0;
            gps_line_overflow = false;
            continue;
        }
        if (gps_line_build_len == 0) {
            gps_line_first_byte_ns = hires_time_ns();
        }
        if (gps_line_overflow) {
            continue;
        }
        if (gps_line_build_len < (sizeof(gps_line_build) - 1u)) {
            gps_line_build[gps_line_build_len++] = c;
        } else {
            gps_line_overflow = true;
            gps_line_build_len = 0;
            __atomic_add_fetch(&gps_line_drop_count, 1, __ATOMIC_RELAXED);
        }
#endif
    }
}

static void gps_uart_irq_init(void) {
    int irq_num = (GPS_UART_ID == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler((uint)irq_num, gps_uart_irq_handler);
    irq_set_enabled((uint)irq_num, true);
    uart_set_irq_enables(GPS_UART_ID, true, false);
}

static int split_fields(char *line, char *fields[], int max_fields) {
    int count = 0;
    fields[count++] = line;
    for (char *p = line; *p && count < max_fields; ++p) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        } else if (*p == '*') {
            *p = '\0';
            break;
        }
    }
    return count;
}

static bool nmea_checksum_valid(const char *line) {
    if (!line || line[0] != '$') {
        return false;
    }

    const char *star = strchr(line + 1, '*');
    if (!star || star[1] == '\0' || star[2] == '\0' || star[3] != '\0') {
        return false;
    }

    unsigned int expected = 0;
    if (sscanf(star + 1, "%2x", &expected) != 1 || expected > 0xFFu) {
        return false;
    }

    uint8_t checksum = 0;
    for (const char *p = line + 1; p < star; ++p) {
        checksum ^= (uint8_t)*p;
    }
    return checksum == (uint8_t)expected;
}

static bool parse_deg_min(const char *token, const char *hemi, bool is_lat, float *out_deg) {
    if (!token || !*token || !hemi || !*hemi) {
        return false;
    }

    char *end = NULL;
    float val = strtof(token, &end);
    if (end == token || *end != '\0' || !isfinite(val) || val < 0.0f) {
        return false;
    }
    int max_deg = is_lat ? 90 : 180;
    /* Bound the float before narrowing it; a finite value can still exceed
     * the representable integer range. */
    if (val > (float)max_deg * 100.0f) return false;
    int deg = (int)(val / 100.0f);
    float minutes = val - (float)deg * 100.0f;
    if (deg > max_deg || minutes < 0.0f || minutes >= 60.0f ||
        (deg == max_deg && minutes != 0.0f) ||
        (is_lat && !(hemi[0] == 'N' || hemi[0] == 'S')) ||
        (!is_lat && !(hemi[0] == 'E' || hemi[0] == 'W'))) {
        return false;
    }
    float dec = (float)deg + (minutes / 60.0f);

    if (hemi[0] == 'S' || hemi[0] == 'W') {
        dec = -dec;
    }

    *out_deg = dec;
    return true;
}

static void set_time_field(const char *token, char *dest, size_t dest_len) {
    if (!token || strlen(token) < 6 || dest_len < 7) {
        return;
    }
    memcpy(dest, token, 6);
    dest[6] = '\0';
}

static void set_date_field(const char *token, char *dest, size_t dest_len) {
    if (!token || strlen(token) < 6 || dest_len < 7) {
        return;
    }
    memcpy(dest, token, 6);
    dest[6] = '\0';
}

static void format_time(const char *raw, char *out, size_t out_len) {
    if (!raw || strlen(raw) < 6 || out_len < 9) {
        snprintf(out, out_len, "--:--:--");
        return;
    }
    snprintf(out, out_len, "%c%c:%c%c:%c%c",
             raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
}

static void format_date(const char *raw, char *out, size_t out_len) {
    if (!raw || strlen(raw) < 6 || out_len < 9) {
        snprintf(out, out_len, "--/--/--");
        return;
    }
    snprintf(out, out_len, "%c%c/%c%c/%c%c",
             raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
}

static bool parse_nonnegative_int(const char *raw, int *out) {
    if (!raw || !*raw || !out) return false;
    char *end = NULL;
    long value = strtol(raw, &end, 10);
    if (end == raw || *end != '\0' || value < 0 || value > INT32_MAX) {
        return false;
    }
    *out = (int)value;
    return true;
}

static void rtc_cache_update_locked(bool status,
                                    int hour, int min, int sec,
                                    int dow, int date, int month, int year,
                                    absolute_time_t sample_time) {
    g_rtc_cache.status = status;
    g_rtc_cache.valid = status;
    rtc_calendar_valid = status;
    if (!status) rtc_invalidate_mapping_locked();
    if (status) {
        g_rtc_cache.hour = hour;
        g_rtc_cache.min = min;
        g_rtc_cache.sec = sec;
        g_rtc_cache.dow = dow;
        g_rtc_cache.date = date;
        g_rtc_cache.month = month;
        g_rtc_cache.year = year;
        g_rtc_cache.sample_time = sample_time;
    }
}

static void rtc_cache_update(bool status,
                             int hour, int min, int sec,
                             int dow, int date, int month, int year,
                             absolute_time_t sample_time) {
    mutex_enter_blocking(&stats_mutex);
    rtc_cache_update_locked(status, hour, min, sec, dow, date, month, year, sample_time);
    mutex_exit(&stats_mutex);
}

static bool rtc_cache_get_locked(time_source_t *out) {
    if (!out || !g_rtc_cache.valid || !g_rtc_cache.status) {
        return false;
    }
    int64_t age_ms = absolute_time_diff_us(g_rtc_cache.sample_time, get_absolute_time()) / 1000;
    if (age_ms < 0 || age_ms > (RTC_CACHE_POLL_MS * 2)) {
        return false;
    }
    out->valid = true;
    out->using_rtc = true;
    out->hour = g_rtc_cache.hour;
    out->min = g_rtc_cache.min;
    out->sec = g_rtc_cache.sec;
    out->date = g_rtc_cache.date;
    out->month = g_rtc_cache.month;
    out->year = g_rtc_cache.year;
    return true;
}

static void rtc_cache_poll(void) {
    absolute_time_t now = get_absolute_time();
    if (!is_nil_time(last_rtc_cache_poll)) {
        int64_t age_ms = absolute_time_diff_us(last_rtc_cache_poll, now) / 1000;
        if (age_ms >= 0 && age_ms < RTC_CACHE_POLL_MS) {
            return;
        }
    }
    last_rtc_cache_poll = now;

    int hour = 0, min = 0, sec = 0, dow = 0, date = 0, month = 0, year = 0;
    if (rtc_read_time(&hour, &min, &sec, &dow, &date, &month, &year)) {
        rtc_cache_update(true, hour, min, sec, dow, date, month, year, now);
    } else {
        rtc_cache_update(false, 0, 0, 0, 0, 0, 0, 0, now);
    }
}

static time_source_t choose_time_source(const gps_stats_t *stats) {
    time_source_t out = {0};
    int gh = 0, gm = 0, gs = 0;
    int gd = 0, gmo = 0, gy = 0;

    if (!have_gps_time) {
        if (rtc_cache_get_locked(&out)) {
            return out;
        }

        if (gps_stats_to_adjusted_utc(stats, &gy, &gmo, &gd, &gh, &gm, &gs)) {
            out.valid = true;
            out.using_rtc = false;
            out.hour = gh;
            out.min = gm;
            out.sec = gs;
            out.date = gd;
            out.month = gmo;
            out.year = gy;
        } else if (parse_hhmmss(stats->time_utc, &gh, &gm, &gs)) {
            out.valid = true;
            out.using_rtc = false;
            out.hour = gh;
            out.min = gm;
            out.sec = gs;
        }
        return out;
    }

    // When PPS is not locked, prefer the DS3231 coarse wall-clock time over GPS sentence time.
    if (rtc_cache_get_locked(&out)) {
        return out;
    }

    if (gps_stats_to_adjusted_utc(stats, &gy, &gmo, &gd, &gh, &gm, &gs)) {
        out.valid = true;
        out.using_rtc = false;
        out.hour = gh;
        out.min = gm;
        out.sec = gs;
        out.date = gd;
        out.month = gmo;
        out.year = gy;
    } else if (parse_hhmmss(stats->time_utc, &gh, &gm, &gs)) {
        out.valid = true;
        out.using_rtc = false;
        out.hour = gh;
        out.min = gm;
        out.sec = gs;
        if (parse_ddmmyy(stats->date, &gd, &gmo, &gy)) {
            out.date = gd;
            out.month = gmo;
            out.year = gy;
        }
    }
    return out;
}

// All consumers use this priority: PPS, RTC SQW, then the best coarse cache.
// Caller must hold stats_mutex because choose_time_source() reads shared state.
static void select_time_at_locked(uint64_t now_ns, selected_time_t *out) {
    time_source_t coarse = choose_time_source(&g_stats);
    uint32_t rtc_period_ns = rtc_sqw_pll_valid ? rtc_sqw_interval_ns : 1000000000u;
    /* 32 kHz contributes rate only. SQW still supplies epoch/phase, and the
     * Pico counter supplies fractions. Reject inconsistent rate estimates. */
    if (rtc_32k.valid && timing_edge_fresh(now_ns, rtc_32k.last_ns, 2500000000ULL) &&
        fabs(rtc_32k.pico_period_ns - rtc_period_ns) < 10000.0)
        rtc_period_ns = (uint32_t)rtc_32k.pico_period_ns;
    timing_observation_t observation = {
        .now_ns = now_ns,
        .pps_ns = trusted_pps_ns, .pps_timeout_ns = PPS_PRESENT_TIMEOUT_US * 1000ULL,
        .pps_epoch_s = trusted_pps_epoch_s, .pps_period_ns = trusted_pps_period_ns,
        .rtc_valid = rtc_sqw_time_valid, .rtc_edge_ns = last_rtc_sqw_second_time_ns,
        .rtc_timeout_ns = RTC_SQW_PRESENT_TIMEOUT_US * 1000ULL,
        .rtc_qualified_ns = rtc_last_qualified_ns, .rtc_epoch_s = rtc_sqw_epoch_seconds,
        .rtc_phase_ns = rtc_holdover_phase_ns, .rtc_period_ns = rtc_period_ns,
        .rtc_jitter_ns = rtc_phase_filter.jitter_ns,
        .rtc_reference_error_ns = rtc_reference_error_ns,
        .rtc_fast_ppm = rtc_holdover_fast_ppm, .rtc_uncertainty_ppm = rtc_learned_uncertainty_ppm,
        .coarse_valid = coarse.valid && coarse.year >= 2000,
        .coarse_rtc = coarse.using_rtc
    };
    if (observation.coarse_valid) observation.coarse_epoch_s = utc_to_epoch(coarse.year, coarse.month,
        coarse.date, coarse.hour, coarse.min, coarse.sec);
    *out = timing_select(&observation, &rtc_holdover, &utc_continuity);
    if (out->quality != timing_previous_quality) {
        if (timing_source_changes < UINT32_MAX) ++timing_source_changes;
        timing_previous_quality = out->quality;
    }
}

static void update_time_snapshot_locked(uint64_t now_ns) {
    int64_t now_us = (int64_t)(now_ns / 1000ULL);
    selected_time_t selected;
    select_time_at_locked(now_ns, &selected);
    memset(&g_time_snapshot, 0, sizeof(g_time_snapshot));
    if (selected.valid) {
        g_time_snapshot.valid = true;
        g_time_snapshot.using_rtc = selected.quality == SNTP_QUALITY_RTC_SQW ||
                                     selected.quality == SNTP_QUALITY_COARSE_RTC;
        epoch_to_utc(selected.unix_sec, &g_time_snapshot.year, &g_time_snapshot.month,
                     &g_time_snapshot.date, &g_time_snapshot.hour,
                     &g_time_snapshot.min, &g_time_snapshot.sec);
    }
    if (selected.quality == SNTP_QUALITY_RTC_SQW ||
        selected.quality == SNTP_QUALITY_COARSE_RTC) {
        if (g_rtc_state != RTC_STATE_MISSING && g_rtc_state != RTC_STATE_FAULT) {
            g_rtc_state = RTC_STATE_HOLDOVER;
        }
    } else if (g_rtc_state == RTC_STATE_HOLDOVER) {
        g_rtc_state = have_rtc_sync ? RTC_STATE_TRACKING : RTC_STATE_INIT;
    }
    update_led_mode_locked(now_us);
}

static void update_time_snapshot(void) {
    mutex_enter_blocking(&stats_mutex);
    update_time_snapshot_locked(hires_time_ns());
    mutex_exit(&stats_mutex);
}

static bool get_led_output(void) {
    bool out = false;
    mutex_enter_blocking(&stats_mutex);
    if (g_led_mode == LED_MODE_BLINK || g_led_mode == LED_MODE_SOLID) {
        out = g_led_state;
    }
    mutex_exit(&stats_mutex);
    return out;
}

static void hires_time_init_shared(void) {
#if PICO_RP2350
    hires_cycles_per_sec = clock_get_hz(clk_sys);
    hires_reanchor_cycles = (uint32_t)((hires_cycles_per_sec / 1000000ULL) * HIRES_REANCHOR_US);
    if (hires_reanchor_cycles == 0) {
        hires_reanchor_cycles = hires_cycles_per_sec;
    }
    hires_time_enabled = hires_cycles_per_sec > 0;
#endif
}

static void hires_time_init_core(void) {
#if PICO_RP2350
    if (!hires_time_enabled) {
        return;
    }
    m33_hw->demcr |= M33_DEMCR_TRCENA_BITS;
    m33_hw->dwt_ctrl |= M33_DWT_CTRL_CYCCNTENA_BITS;
    uint core = get_core_num();
    uint32_t save = save_and_disable_interrupts();
    hires_clock_state[core].anchor_us = time_us_64();
    hires_clock_state[core].anchor_cycles = m33_hw->dwt_cyccnt;
    hires_clock_state[core].last_ns = hires_clock_state[core].anchor_us * 1000ULL;
    restore_interrupts(save);
#endif
}

static uint64_t hires_time_ns(void) {
#if PICO_RP2350
    if (hires_time_enabled) {
        uint core = get_core_num();
        uint64_t anchor_us = 0;
        uint32_t anchor_cycles = 0;
        uint32_t cycles = 0;
        uint32_t save = save_and_disable_interrupts();
        anchor_us = hires_clock_state[core].anchor_us;
        anchor_cycles = hires_clock_state[core].anchor_cycles;
        cycles = m33_hw->dwt_cyccnt;
        uint32_t delta_cycles = cycles - anchor_cycles;
        uint64_t result_ns;
        if (delta_cycles >= hires_reanchor_cycles) {
            /* Sampling and publishing the replacement mapping must be one
             * operation: UART/PPS ISRs also call this function. */
            uint64_t now_us = time_us_64();
            cycles = m33_hw->dwt_cyccnt;
            hires_clock_state[core].anchor_us = now_us;
            hires_clock_state[core].anchor_cycles = cycles;
            result_ns = now_us * 1000ULL;
        } else {
            uint64_t delta_ns = ((uint64_t)delta_cycles * 1000000000ULL) / hires_cycles_per_sec;
            result_ns = (anchor_us * 1000ULL) + delta_ns;
        }
        /* Microsecond quantization of a replacement anchor may briefly put
         * it behind the last fine-grained sample. Never publish a reversal. */
        if (result_ns < hires_clock_state[core].last_ns)
            result_ns = hires_clock_state[core].last_ns;
        hires_clock_state[core].last_ns = result_ns;
        restore_interrupts(save);
        return result_ns;
    }
#endif
    return time_us_64() * 1000ULL;
}

static void gps_line_reset(void) {
    uint32_t save = save_and_disable_interrupts();
    gps_line_head = 0;
    gps_line_tail = 0;
    gps_line_build_len = 0;
    gps_line_first_byte_ns = 0;
    gps_line_overflow = false;
#if GPS_PROTOCOL_UBX
    ubx_parser_init(&ubx_parser);
    ubx_frame_head = 0;
    ubx_frame_tail = 0;
#endif
    restore_interrupts(save);
}

static bool gps_line_pop(gps_line_t *out) {
    if (!out) {
        return false;
    }
    uint32_t save = save_and_disable_interrupts();
    if (gps_line_tail == gps_line_head) {
        restore_interrupts(save);
        return false;
    }
    *out = gps_line_queue[gps_line_tail];
    gps_line_tail = (uint8_t)((gps_line_tail + 1u) % GPS_LINE_QUEUE_SIZE);
    restore_interrupts(save);
    return true;
}

#if GPS_PROTOCOL_UBX
static bool ubx_frame_pop(ubx_frame_t *out) {
    if (!out) return false;
    uint32_t save = save_and_disable_interrupts();
    if (ubx_frame_tail == ubx_frame_head) {
        restore_interrupts(save);
        return false;
    }
    *out = ubx_frame_queue[ubx_frame_tail];
    ubx_frame_tail = (uint8_t)((ubx_frame_tail + 1u) % GPS_LINE_QUEUE_SIZE);
    restore_interrupts(save);
    return true;
}
#endif

typedef struct {
    uint32_t gps_checksum_error_count;
    uint32_t gps_rejected_fix_count;
    uint32_t sntp_request_count;
    uint32_t sntp_malformed_count;
    uint32_t sntp_rate_limit_count;
    uint32_t sntp_send_error_count;
    uint32_t http_request_count;
    uint32_t http_error_count;
    uint32_t wifi_reconnect_count;
    uint32_t wifi_reconnect_fail_count;
    int32_t rtc_drift_us_per_day_x10;
    gps_stats_t stats;
    time_source_t ts;
    time_source_t rtc_cache_ts;
    bool pps_present;
    bool pps_time_valid;
    bool pll_ok;
    uint32_t pll_interval;
    int64_t last_pps_us;
    int64_t last_rtc_sqw_us;
    bool pps_jitter_valid;
    int32_t pps_jitter_last;
    int32_t pps_jitter_min;
    int32_t pps_jitter_max;
    uint32_t pps_jitter_rms;
    uint32_t pps_jitter_samples;
    int64_t pps_jitter_last_ns;
    int64_t pps_jitter_min_ns;
    int64_t pps_jitter_max_ns;
    uint64_t pps_jitter_rms_ns;
    int64_t pll_phase_error_ns;
    int64_t ab_innovation_ns;
    int64_t pps_period_error_ns;
    uint32_t pps_dma_overruns;
    bool ab_servo_valid;
    uint64_t ab_period_ns;
    int64_t ab_period_rate_ns_per_sec;
    uint64_t ab_median_ns;
    uint64_t ab_mad_ns;
    uint64_t ab_outlier_threshold_ns;
    uint32_t ab_rejected;
    uint32_t ab_alpha_shift;
    uint32_t ab_beta_shift;
    bool accuracy_mode;
    uint32_t pll_filter_shift;
    uint32_t rmc_phase_ok_count;
    int32_t rmc_pps_offset_us;
    int32_t rmc_pps_window_us;
    uint32_t uptime_s;
    int32_t wifi_rssi;
    bool wifi_link;
    int64_t rmc_age_ms;
    int64_t rtc_offset_s;
    int64_t rtc_offset_us;
    bool rtc_offset_us_valid;
    bool rtc_sqw_present;
    bool rtc_sqw_time_valid;
    int64_t rtc_sqw_age_ms;
    int32_t rtc_sqw_phase_us;
    bool rtc_sqw_phase_valid;
    bool rtc_sqw_pll_ok;
    uint32_t rtc_sqw_interval_ns;
    uint64_t last_rtc_sqw_second_time_ns;
    int64_t rtc_sqw_epoch_seconds;
    int64_t rtc_cache_age_ms;
    uint32_t heap_free_bytes;
    uint32_t heap_used_bytes;
    uint32_t heap_total_bytes;
    uint32_t lwip_mem_used;
    uint32_t lwip_mem_avail;
    uint32_t lwip_mem_max;
    uint32_t lwip_memp_used;
    uint32_t lwip_memp_max;
    char ip[16];
    bool rtc_sync_valid;
    int64_t rtc_sync_age_ms;
    bool rtc_aging_valid;
    int rtc_aging;
    bool rtc_status;
    rtc_state_t rtc_state;
    uint32_t rtc_track_good;
    uint32_t rtc_track_bad;
#if HEALTH_BROADCAST_ENABLE
    uint32_t health_fmt_drop_count;
    uint32_t health_alloc_fail_count;
    uint32_t health_send_err_count;
    uint32_t health_rtc_busy_count;
#endif
    uint32_t rtc_sqw_rephase_count;
    uint32_t gps_line_drop_count;
    uint32_t tm1637_errors;
    bool have_gps_time;
    sntp_quality_t sntp_quality;
    uint8_t sntp_stratum;
    char sntp_refid[5];
    absolute_time_t last_rmc_update_time;
    clock_state_t clock_state;
    char timezone_mode[8];
    char timezone_name[40];
    char timezone_source[16];
    char timezone_offset_text[8];
    int32_t timezone_offset_seconds;
    bool timezone_is_dst;
    bool timezone_rules_out_of_range;
    bool timezone_lookup_valid;
    int64_t timezone_last_lookup_age_s;
    char timezone_candidate[40];
    int32_t timezone_candidate_count;
    int64_t local_unix_sec;
} health_snapshot_t;

static const char *timezone_mode_string(void) {
    return (strcmp(TIMEZONE_MODE, "UTC") == 0) ? "UTC" :
           (strcmp(TIMEZONE_MODE, "MANUAL") == 0) ? "MANUAL" : "AUTO";
}

static void timezone_format_offset(int32_t seconds, char *out, size_t out_len) {
    int32_t value = seconds < 0 ? -seconds : seconds;
    snprintf(out, out_len, "%c%02ld:%02ld", seconds < 0 ? '-' : '+',
             (long)(value / 3600), (long)((value % 3600) / 60));
}

static void timezone_refresh(void) {
    mutex_enter_blocking(&stats_mutex);
    int64_t now_epoch = 0;
    if (g_time_snapshot.valid) now_epoch = utc_to_epoch(g_time_snapshot.year, g_time_snapshot.month,
                                                        g_time_snapshot.date, g_time_snapshot.hour,
                                                        g_time_snapshot.min, g_time_snapshot.sec);
    if (strcmp(TIMEZONE_MODE, "MANUAL") == 0) {
        timezone_id_t id = TIMEZONE_ID_UTC;
        if (timezone_id_from_name(TIMEZONE_MANUAL_NAME, &id)) {
            timezone_state.current_id = id;
            timezone_state.lookup_valid = true;
            timezone_state.source_gps = false;
        }
    } else if (strcmp(TIMEZONE_MODE, "UTC") == 0) {
        timezone_state.current_id = TIMEZONE_ID_UTC;
        timezone_state.lookup_valid = true;
        timezone_state.source_gps = false;
    } else if (g_stats.has_fix && g_stats.lat_valid && g_stats.lon_valid) {
        int32_t lat = (int32_t)lroundf(g_stats.lat_deg * 10000000.0f);
        int32_t lon = (int32_t)lroundf(g_stats.lon_deg * 10000000.0f);
        int32_t dlat = lat - timezone_state.last_lat_e7;
        int32_t dlon = lon - timezone_state.last_lon_e7;
        if (dlon > 1800000000) dlon -= 3600000000LL;
        if (dlon < -1800000000) dlon += 3600000000LL;
        // One degree of latitude is approximately 111 km.  Longitude is
        // deliberately conservative here; this only gates re-lookups.
        int64_t moved2 = (int64_t)dlat * dlat + (int64_t)dlon * dlon;
        int64_t threshold = (int64_t)TIMEZONE_MOVE_THRESHOLD_KM * 9009LL;
        bool need_lookup = !timezone_state.lookup_valid ||
            moved2 >= threshold * threshold;
        if (need_lookup) {
            timezone_id_t candidate = timezone_from_coordinates(lat, lon);
            if (candidate == timezone_state.current_id) {
                timezone_state.candidate_id = candidate;
                timezone_state.candidate_count = 0;
            } else if (candidate == timezone_state.candidate_id) {
                if (timezone_state.candidate_count < 255) timezone_state.candidate_count++;
            } else {
                timezone_state.candidate_id = candidate;
                timezone_state.candidate_count = 1;
            }
            if (!timezone_state.lookup_valid || timezone_state.candidate_count >= TIMEZONE_CANDIDATE_COUNT) {
                timezone_state.current_id = candidate;
                timezone_state.candidate_count = 0;
                timezone_state.lookup_valid = true;
            }
            timezone_state.last_lat_e7 = lat;
            timezone_state.last_lon_e7 = lon;
            timezone_state.last_lookup_utc = g_time_snapshot.valid ? (uint64_t)now_epoch : 0;
            timezone_state.source_gps = true;
        }
    }
    timezone_state.rules_out_of_range = !timezone_get_offset(timezone_state.current_id, now_epoch,
                                                              &timezone_state.offset_seconds,
                                                              &timezone_state.is_dst);
    mutex_exit(&stats_mutex);
}

static void get_health_snapshot(health_snapshot_t *out) {
    if (!out) return;
    absolute_time_t now = get_absolute_time();
    int64_t now_us = to_us_since_boot(now);
    int32_t jitter_buf[PPS_JITTER_WINDOW];
    int64_t jitter_ns_buf[PPS_JITTER_WINDOW];
    uint32_t jitter_count = 0;
    int32_t jitter_last = 0;
    int64_t jitter_last_ns = 0;
    int64_t jitter_ns_sumsq = 0;
    bool jitter_valid = false;
    mutex_enter_blocking(&stats_mutex);
    out->stats = g_stats;
    out->ts = g_time_snapshot;
    out->pps_time_valid = pps_time_valid;
    out->pll_ok = pll_valid;
    out->pll_interval = (pll_interval_ns + 500U) / 1000U;
    out->last_pps_us = last_pps_us;
    out->last_rtc_sqw_us = last_rtc_sqw_us;
    jitter_count = pps_jitter_count;
    if (jitter_count > PPS_JITTER_WINDOW) {
        jitter_count = PPS_JITTER_WINDOW;
    }
    if (jitter_count > 0) {
        memcpy(jitter_buf, pps_jitter_buf, jitter_count * sizeof(jitter_buf[0]));
        memcpy(jitter_ns_buf, pps_jitter_ns_buf, jitter_count * sizeof(jitter_ns_buf[0]));
    }
    jitter_last = pps_jitter_last;
    jitter_last_ns = pps_jitter_last_ns;
    jitter_ns_sumsq = pps_jitter_ns_sumsq;
    jitter_valid = pps_jitter_valid;
    out->rtc_sync_valid = have_rtc_sync;
    out->rtc_sync_age_ms = have_rtc_sync ?
        (absolute_time_diff_us(last_rtc_sync_time, now) / 1000) : -1;
    out->rtc_aging_valid = rtc_aging_valid;
    out->rtc_aging = rtc_aging_value;
    out->pll_phase_error_ns = pll_phase_error_ns;
    out->ab_innovation_ns = ab_innovation_ns;
    out->pps_period_error_ns = pps_period_error_last_ns;
    out->pps_dma_overruns = pps_dma_overruns;
    out->ab_servo_valid = ab_servo_valid;
    out->ab_period_ns = ab_period_ns;
    out->ab_period_rate_ns_per_sec = ab_period_rate_ns_per_sec;
    out->ab_median_ns = ab_interval_history_count > 0
        ? median_u64(ab_interval_history, ab_interval_history_count) : 0;
    out->ab_mad_ns = ab_mad_ns();
    out->ab_outlier_threshold_ns = ab_outlier_threshold_ns();
    out->ab_rejected = pps_outliers_rejected;
    out->ab_alpha_shift = PPS_AB_ALPHA_SHIFT;
    out->ab_beta_shift = PPS_AB_BETA_SHIFT;
    out->accuracy_mode = accuracy_mode;
    out->pll_filter_shift = pll_filter_shift;
    out->rmc_phase_ok_count = rmc_phase_ok_count;
    out->rmc_pps_offset_us = rmc_pps_avg_us;
    out->rmc_pps_window_us = rmc_pps_window_us;
    out->rtc_sqw_time_valid = rtc_sqw_time_valid;
    out->rtc_sqw_phase_us = rtc_sqw_second_edge_known ? rtc_sqw_phase_last_us : 0;
    out->rtc_sqw_phase_valid = rtc_sqw_phase_valid;
    out->rtc_sqw_pll_ok = rtc_sqw_pll_valid;
    out->rtc_sqw_interval_ns = rtc_sqw_pll_valid ? rtc_sqw_interval_ns : 1000000000u;
    out->last_rtc_sqw_second_time_ns = get_last_rtc_sqw_second_time_ns();
    out->rtc_sqw_epoch_seconds = rtc_sqw_epoch_seconds;
    out->have_gps_time = have_gps_time;
    out->last_rmc_update_time = last_rmc_update_time;
    out->rtc_state = g_rtc_state;
    out->rtc_track_good = rtc_track_good_count;
    out->rtc_track_bad = rtc_track_bad_count;
#if HEALTH_BROADCAST_ENABLE
    out->health_fmt_drop_count = __atomic_load_n(&health_fmt_drop_count, __ATOMIC_RELAXED);
    out->health_alloc_fail_count = __atomic_load_n(&health_alloc_fail_count, __ATOMIC_RELAXED);
    out->health_send_err_count = __atomic_load_n(&health_send_err_count, __ATOMIC_RELAXED);
    out->health_rtc_busy_count = __atomic_load_n(&health_rtc_busy_count, __ATOMIC_RELAXED);
#endif
    out->gps_checksum_error_count = __atomic_load_n(&gps_checksum_error_count, __ATOMIC_RELAXED);
    out->gps_rejected_fix_count = __atomic_load_n(&gps_rejected_fix_count, __ATOMIC_RELAXED);
    out->sntp_request_count = __atomic_load_n(&sntp_request_count, __ATOMIC_RELAXED);
    out->sntp_malformed_count = __atomic_load_n(&sntp_malformed_count, __ATOMIC_RELAXED);
    out->sntp_rate_limit_count = __atomic_load_n(&sntp_rate_limit_count, __ATOMIC_RELAXED);
    out->sntp_send_error_count = __atomic_load_n(&sntp_send_error_count, __ATOMIC_RELAXED);
    out->http_request_count = __atomic_load_n(&http_request_count, __ATOMIC_RELAXED);
    out->http_error_count = __atomic_load_n(&http_error_count, __ATOMIC_RELAXED);
    out->wifi_reconnect_count = __atomic_load_n(&wifi_reconnect_count, __ATOMIC_RELAXED);
    out->wifi_reconnect_fail_count = __atomic_load_n(&wifi_reconnect_fail_count, __ATOMIC_RELAXED);
    out->rtc_drift_us_per_day_x10 = 0;
    double drift_us_per_sec = 0.0;
    int64_t drift_elapsed = 0;
    if (drift_estimator_slope_us_per_sec(&rtc_drift_estimator,
                                         &drift_us_per_sec, &drift_elapsed)) {
        double scaled = drift_us_per_sec * 864000.0;
        if (scaled > 2147483647.0) scaled = 2147483647.0;
        if (scaled < -2147483648.0) scaled = -2147483648.0;
        out->rtc_drift_us_per_day_x10 = (int32_t)scaled;
    }
    out->rtc_sqw_rephase_count = rtc_sqw_rephase_count;
    out->gps_line_drop_count = __atomic_load_n(&gps_line_drop_count, __ATOMIC_RELAXED);
    out->tm1637_errors = tm1637_error_count();
    out->rtc_cache_ts.valid = g_rtc_cache.valid;
    out->rtc_cache_ts.using_rtc = true;
    out->rtc_cache_ts.hour = g_rtc_cache.hour;
    out->rtc_cache_ts.min = g_rtc_cache.min;
    out->rtc_cache_ts.sec = g_rtc_cache.sec;
    out->rtc_cache_ts.date = g_rtc_cache.date;
    out->rtc_cache_ts.month = g_rtc_cache.month;
    out->rtc_cache_ts.year = g_rtc_cache.year;
    out->rtc_status = g_rtc_cache.status && g_rtc_cache.valid;
    out->rtc_cache_age_ms = (g_rtc_cache.valid && g_rtc_cache.status) ?
        (absolute_time_diff_us(g_rtc_cache.sample_time, now) / 1000) : -1;
    selected_time_t selected;
    select_time_at_locked(hires_time_ns(), &selected);
    out->sntp_quality = selected.quality;
    out->sntp_stratum = sntp_quality_stratum(out->sntp_quality);
    sntp_quality_refid(out->sntp_quality, out->sntp_refid);
    snprintf(out->timezone_mode, sizeof(out->timezone_mode), "%s", timezone_mode_string());
    snprintf(out->timezone_name, sizeof(out->timezone_name), "%s", timezone_name(timezone_state.current_id));
    snprintf(out->timezone_source, sizeof(out->timezone_source), "%s",
             timezone_state.source_gps ? (timezone_state.current_id ? "GPS_POSITION" : "OCEAN_UTC") :
             (strcmp(TIMEZONE_MODE, "MANUAL") == 0 ? "MANUAL" : "UTC"));
    out->timezone_lookup_valid = timezone_state.lookup_valid;
    out->timezone_offset_seconds = timezone_state.offset_seconds;
    out->timezone_is_dst = timezone_state.is_dst;
    out->timezone_rules_out_of_range = timezone_state.rules_out_of_range;
    timezone_format_offset(out->timezone_offset_seconds, out->timezone_offset_text,
                           sizeof(out->timezone_offset_text));
    out->timezone_last_lookup_age_s = (timezone_state.last_lookup_utc && selected.valid &&
                                      selected.unix_sec >= (int64_t)timezone_state.last_lookup_utc) ?
                                      selected.unix_sec - (int64_t)timezone_state.last_lookup_utc : -1;
    snprintf(out->timezone_candidate, sizeof(out->timezone_candidate), "%s",
             timezone_name(timezone_state.candidate_id));
    out->timezone_candidate_count = timezone_state.candidate_count;
    out->local_unix_sec = selected.valid ? selected.unix_sec + timezone_state.offset_seconds : 0;
    if (out->sntp_quality == SNTP_QUALITY_PPS) {
        out->clock_state = CLOCK_STATE_LOCKED;
    } else if (out->sntp_quality == SNTP_QUALITY_RTC_SQW || out->sntp_quality == SNTP_QUALITY_COARSE_RTC) {
        out->clock_state = CLOCK_STATE_HOLDOVER;
    } else if (have_gps_time) {
        out->clock_state = CLOCK_STATE_ACQUIRE;
    } else {
        out->clock_state = CLOCK_STATE_UNLOCKED;
    }
    mutex_exit(&stats_mutex);

    out->uptime_s = (uint32_t)(now_us / 1000000ULL);

    int64_t rmc_age = absolute_time_diff_us(out->last_rmc_update_time, now) / 1000;
    out->rmc_age_ms = (out->have_gps_time && rmc_age >= 0) ? rmc_age : -1;

    out->wifi_link = false;
    out->wifi_rssi = 0;
    snprintf(out->ip, sizeof(out->ip), "0.0.0.0");
    if (wifi_driver_ready) {
        out->wifi_link = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP;
    }
    get_ip_string(out->ip, sizeof(out->ip));
    int32_t rssi = 0;
    if (wifi_driver_ready && cyw43_wifi_get_rssi(&cyw43_state, &rssi) == 0) {
        out->wifi_rssi = (int32_t)rssi;
    }

    out->rtc_offset_s = -1;
    out->rtc_offset_us = -1;
    out->rtc_offset_us_valid = false;
    out->rtc_sqw_present = false;
    out->rtc_sqw_age_ms = -1;
    uint64_t now_ns = hires_time_ns();
    int64_t ref_epoch = 0;
    bool ref_epoch_valid = false;
    uint32_t ref_sec = 0;
    sntp_quality_t ref_quality = SNTP_QUALITY_NONE;
    if (get_sntp_time_at(now_ns, &ref_sec, NULL, NULL, &ref_quality) &&
        ref_quality != SNTP_QUALITY_RTC_SQW &&
        ref_quality != SNTP_QUALITY_COARSE_RTC) {
        ref_epoch = (int64_t)ref_sec - NTP_DELTA;
        ref_epoch_valid = true;
    } else {
        ref_epoch_valid = gps_fields_to_adjusted_epoch(out->stats.time_utc, out->stats.date, &ref_epoch);
    }

    int64_t rtc_epoch = 0;
    bool rtc_epoch_valid = false;
    if (out->rtc_sqw_time_valid &&
        out->rtc_sqw_phase_valid &&
        out->last_rtc_sqw_second_time_ns > 0 &&
        now_ns >= out->last_rtc_sqw_second_time_ns) {
        uint64_t age_ns = now_ns - out->last_rtc_sqw_second_time_ns;
        uint32_t interval_ns = out->rtc_sqw_pll_ok ? out->rtc_sqw_interval_ns : 1000000000u;
        if (age_ns < (RTC_SQW_PRESENT_TIMEOUT_US * 1000ULL) && interval_ns > 0) {
            uint64_t corrected_ns = (age_ns * 1000000000ULL) / interval_ns;
            rtc_epoch = out->rtc_sqw_epoch_seconds + (int64_t)(corrected_ns / 1000000000ULL);
            rtc_epoch_valid = true;
            out->rtc_offset_us = (int64_t)out->rtc_sqw_phase_us;
            out->rtc_offset_us_valid = true;
        }
    }

    if (!rtc_epoch_valid &&
        out->rtc_status &&
        out->rtc_cache_age_ms >= 0 &&
        out->rtc_cache_age_ms <= (RTC_CACHE_POLL_MS * 2)) {
        rtc_epoch = utc_to_epoch(out->rtc_cache_ts.year, out->rtc_cache_ts.month,
                                 out->rtc_cache_ts.date, out->rtc_cache_ts.hour,
                                 out->rtc_cache_ts.min, out->rtc_cache_ts.sec);
        rtc_epoch_valid = true;
    }

    if (ref_epoch_valid && rtc_epoch_valid) {
        out->rtc_offset_s = ref_epoch - rtc_epoch;
        if (!out->rtc_offset_us_valid) {
            out->rtc_offset_us = out->rtc_offset_s * 1000000LL;
        }
    }

    if (out->last_rtc_sqw_us >= 0) {
        out->rtc_sqw_age_ms = (now_us - out->last_rtc_sqw_us) / 1000;
        out->rtc_sqw_present = (out->rtc_sqw_age_ms >= 0) &&
            ((uint64_t)(now_us - out->last_rtc_sqw_us) < RTC_SQW_PRESENT_TIMEOUT_US);
    }

    out->heap_free_bytes = 0;
    out->heap_used_bytes = 0;
    out->heap_total_bytes = 0;
    out->lwip_mem_used = 0;
    out->lwip_mem_avail = 0;
    out->lwip_mem_max = 0;
    out->lwip_memp_used = 0;
    out->lwip_memp_max = 0;
    extern char end; /* heap start */
    extern char __StackLimit; /* heap limit */
    void *heap_end = sbrk(0);
    if (heap_end && heap_end != (void *)-1) {
        intptr_t total = (intptr_t)&__StackLimit - (intptr_t)&end;
        intptr_t used = (intptr_t)heap_end - (intptr_t)&end;
        intptr_t free = (intptr_t)&__StackLimit - (intptr_t)heap_end;
        if (total < 0) total = 0;
        if (used < 0) used = 0;
        if (free < 0) free = 0;
        out->heap_total_bytes = (uint32_t)total;
        out->heap_used_bytes = (uint32_t)used;
        out->heap_free_bytes = (uint32_t)free;
    }
#if LWIP_STATS && MEM_STATS
    out->lwip_mem_used = (uint32_t)lwip_stats.mem.used;
    out->lwip_mem_avail = (uint32_t)lwip_stats.mem.avail;
    out->lwip_mem_max = (uint32_t)lwip_stats.mem.max;
#endif
#if LWIP_STATS && MEMP_STATS
    uint32_t memp_used = 0;
    uint32_t memp_max = 0;
    for (int i = 0; i < MEMP_MAX; ++i) {
        if (lwip_stats.memp[i]) {
            memp_used += lwip_stats.memp[i]->used;
            memp_max += lwip_stats.memp[i]->max;
        }
    }
    out->lwip_memp_used = memp_used;
    out->lwip_memp_max = memp_max;
#endif
    if (out->last_pps_us >= 0) {
        out->pps_present = ((now_us - out->last_pps_us) < (int64_t)PPS_PRESENT_TIMEOUT_US);
    } else {
        out->pps_present = false;
    }

    out->pps_jitter_valid = jitter_valid;
    out->pps_jitter_last = jitter_last;
    out->pps_jitter_last_ns = jitter_last_ns;
    out->pps_jitter_samples = jitter_count;
    if (jitter_count == 0) {
        out->pps_jitter_min = 0;
        out->pps_jitter_max = 0;
        out->pps_jitter_rms = 0;
        out->pps_jitter_min_ns = 0;
        out->pps_jitter_max_ns = 0;
        out->pps_jitter_rms_ns = 0;
        return;
    }
    int32_t jmin = jitter_buf[0];
    int32_t jmax = jitter_buf[0];
    double sumsq = 0.0;
    for (uint32_t i = 0; i < jitter_count; ++i) {
        int32_t j = jitter_buf[i];
        if (j < jmin) jmin = j;
        if (j > jmax) jmax = j;
        sumsq += (double)j * (double)j;
    }
    out->pps_jitter_min = jmin;
    out->pps_jitter_max = jmax;
    out->pps_jitter_rms = (uint32_t)sqrt(sumsq / (double)jitter_count);
    int64_t jmin_ns = jitter_ns_buf[0];
    int64_t jmax_ns = jitter_ns_buf[0];
    for (uint32_t i = 0; i < jitter_count; ++i) {
        int64_t j_ns = jitter_ns_buf[i];
        if (j_ns < jmin_ns) jmin_ns = j_ns;
        if (j_ns > jmax_ns) jmax_ns = j_ns;
    }
    out->pps_jitter_min_ns = jmin_ns;
    out->pps_jitter_max_ns = jmax_ns;
    out->pps_jitter_rms_ns = (uint64_t)sqrt((double)jitter_ns_sumsq / (double)jitter_count);
}

static void format_rtc_time(int hour, int min, int sec, char *out, size_t out_len) {
    snprintf(out, out_len, "%02d:%02d:%02d", hour, min, sec);
}

static void format_rtc_date(int date, int month, int year, char *out, size_t out_len) {
    snprintf(out, out_len, "%02d/%02d/%02d", date, month, year % 100);
}

static void update_accuracy_mode_locked(void) {
#if ACCURACY_MODE
    if (!pps_time_valid || !pll_valid || !pps_jitter_valid ||
        pps_jitter_count < ACCURACY_PPS_MIN_SAMPLES || pps_jitter_count == 0) {
        accuracy_mode = false;
        pll_filter_shift = PPS_PLL_FILTER_SHIFT;
        pps_jitter_rms_us = 0;
        return;
    }
    double rms = sqrt((double)pps_jitter_sumsq / (double)pps_jitter_count);
    if (rms < 0.0) rms = 0.0;
    pps_jitter_rms_us = (uint32_t)(rms + 0.5);
    bool stable = (rms <= (double)ACCURACY_PPS_RMS_US) &&
                  (llabs((long long)pps_jitter_last) <= (long long)ACCURACY_PPS_PEAK_US);
    accuracy_mode = stable;
    uint8_t accuracy_shift = ACCURACY_PLL_FILTER_SHIFT;
    if (accuracy_shift < 1u) accuracy_shift = 1u;
    if (rms <= ACCURACY_PLL_SHIFT_RMS_T1) {
        pll_filter_shift = accuracy_shift < 31u ? (uint8_t)(accuracy_shift + 1u) : accuracy_shift;
    } else if (rms <= ACCURACY_PLL_SHIFT_RMS_T2) {
        pll_filter_shift = accuracy_shift;
    } else if (rms <= ACCURACY_PLL_SHIFT_RMS_T3) {
        pll_filter_shift = accuracy_shift > 1u ? (uint8_t)(accuracy_shift - 1u) : 1u;
    } else {
        pll_filter_shift = PPS_PLL_FILTER_SHIFT;
    }
#endif
}

// A PPS or GPS-time interruption invalidates the learned sentence phase.
// Keep reacquisition in bootstrap mode so a changed UART delivery delay can
// be learned again instead of being rejected by the stale narrow window.
static void reset_rmc_phase_tracking_locked(void) {
    rmc_epoch_valid = false;
    rmc_phase_ok_count = 0;
    rmc_pps_window_us = RMC_PPS_MAX_OFFSET_US;
    rmc_pps_avg_us = 0;
    rmc_pps_var_us2 = 0;
    rmc_pps_samples = 0;
}

static void rmc_pps_update_stats(uint64_t rmc_ns) {
    uint64_t last_pps = get_last_pps_time_ns();
    if (last_pps == 0) {
        return;
    }
    int64_t phase_ns = (int64_t)rmc_ns - (int64_t)last_pps;
    int64_t phase = phase_ns / 1000LL;
    uint32_t max_offset_us = (rmc_pps_samples < RMC_PPS_BOOTSTRAP_SAMPLES)
        ? RMC_PPS_BOOTSTRAP_MAX_OFFSET_US
        : RMC_PPS_MAX_OFFSET_US;
    if (rmc_pps_samples < RMC_PPS_BOOTSTRAP_SAMPLES) {
        if (llabs((long long)phase) > (int64_t)max_offset_us) return;
    } else if (llabs((long long)phase - (long long)rmc_pps_avg_us) >
               (int64_t)max_offset_us) {
        // After bootstrap, validate the residual from the learned UART delay,
        // not the absolute phase. NMEA delivery can legitimately exceed 50 ms.
        return;
    }
    if (rmc_pps_samples == 0) {
        rmc_pps_avg_us = (int32_t)phase;
        rmc_pps_var_us2 = 0;
        rmc_pps_samples = 1;
    } else {
        int32_t delta = (int32_t)phase - rmc_pps_avg_us;
        rmc_pps_avg_us += (delta >> RMC_PPS_EMA_SHIFT);
        int64_t delta2 = (int64_t)delta * (int64_t)delta;
        rmc_pps_var_us2 += (delta2 - rmc_pps_var_us2) >> RMC_PPS_EMA_SHIFT;
        rmc_pps_samples++;
    }
    int32_t std_us = (int32_t)sqrt((double)rmc_pps_var_us2);
    int32_t window = (RMC_PPS_STD_MULT * std_us) + RMC_PPS_MARGIN_US;
    int32_t minimum_window = RMC_PPS_MIN_OFFSET_US;
#if ACCURACY_MODE
    if (accuracy_mode && minimum_window > ACCURACY_RMC_MAX_WINDOW_US) {
        minimum_window = ACCURACY_RMC_MAX_WINDOW_US;
    }
#endif
    if (window < minimum_window) window = minimum_window;
    if (window > (int32_t)max_offset_us) window = (int32_t)max_offset_us;
#if ACCURACY_MODE
    if (accuracy_mode && rmc_pps_samples >= RMC_PPS_BOOTSTRAP_SAMPLES) {
        uint32_t rms_cap = (pps_jitter_rms_us * ACCURACY_RMC_RMS_MULT) + ACCURACY_RMC_RMS_MARGIN_US;
        if (rms_cap < (uint32_t)RMC_PPS_MIN_OFFSET_US) {
            rms_cap = (uint32_t)RMC_PPS_MIN_OFFSET_US;
        }
        if (rms_cap > (uint32_t)ACCURACY_RMC_MAX_WINDOW_US) {
            rms_cap = (uint32_t)ACCURACY_RMC_MAX_WINDOW_US;
        }
        if (window > (int32_t)rms_cap) {
            window = (int32_t)rms_cap;
        }
    }
#endif
    rmc_pps_window_us = window;
}

static void drift_estimator_reset(drift_estimator_t *est) {
    if (!est) {
        return;
    }
    memset(est, 0, sizeof(*est));
}

static void drift_estimator_update(drift_estimator_t *est, int64_t epoch_sec, double sample_us) {
    if (!est) {
        return;
    }
    if (!est->valid) {
        est->valid = true;
        est->base_epoch = epoch_sec;
    }
    double x = (double)(epoch_sec - est->base_epoch);
    est->sum_x += x;
    est->sum_y += sample_us;
    est->sum_xx += x * x;
    est->sum_xy += x * sample_us;
    est->last_epoch = epoch_sec;
    est->samples++;
}

static bool drift_estimator_slope_us_per_sec(const drift_estimator_t *est, double *slope_us_per_sec,
                                             int64_t *elapsed_sec) {
    if (!est || !est->valid || est->samples < 2) {
        return false;
    }
    double n = (double)est->samples;
    double denom = (n * est->sum_xx) - (est->sum_x * est->sum_x);
    if (denom <= 0.0) {
        return false;
    }
    if (elapsed_sec) {
        *elapsed_sec = est->last_epoch - est->base_epoch;
    }
    if (slope_us_per_sec) {
        *slope_us_per_sec = ((n * est->sum_xy) - (est->sum_x * est->sum_y)) / denom;
    }
    return true;
}

static void rtc_drift_reset(void) {
    drift_estimator_reset(&rtc_drift_estimator);
}

static void rtc_phase_drift_reset(void) {
    drift_estimator_reset(&rtc_phase_estimator);
}

static void rtc_drift_update(int64_t gps_epoch, int64_t rtc_epoch, int64_t abs_delta) {
    if (abs_delta >= RTC_SYNC_FORCE_SET_SEC) {
        return;
    }
    drift_estimator_update(&rtc_drift_estimator, gps_epoch,
                           (double)(gps_epoch - rtc_epoch) * 1000000.0);
}

static void rtc_phase_drift_update(int32_t phase_us, int64_t epoch_sec) {
    drift_estimator_update(&rtc_phase_estimator, epoch_sec, (double)phase_us);
}

static void rtc_schedule_set_locked(int64_t epoch_sec, bool phase_rephase, uint64_t due_ns) {
    rtc_pending_set_valid = true;
    rtc_pending_set_epoch = epoch_sec;
    rtc_pending_set_phase = phase_rephase;
    rtc_pending_set_due_ns = due_ns;
}

static bool rtc_set_epoch_utc(int64_t epoch_sec, bool phase_rephase) {
    int year = 0;
    int month = 0;
    int date = 0;
    int hour = 0;
    int min = 0;
    int sec = 0;
    epoch_to_utc(epoch_sec, &year, &month, &date, &hour, &min, &sec);
    int dow = day_of_week(year, month, date);
    absolute_time_t now = get_absolute_time();

    uint8_t status = 0;
    bool set_ok = rtc_set_time(hour, min, sec, dow, date, month, year);
    /* A verified GNSS-derived calendar set is the only OSF acknowledgement.
     * Preserve alarm flags on the write-zero-to-clear status register. */
    if (set_ok) set_ok = rtc_read_reg(RTC_STATUS_REG, &status) &&
        rtc_write_reg(RTC_STATUS_REG, (uint8_t)((status & 0x0cu) | 3u));
    if (!set_ok) {
        rtc_cache_update(false, 0, 0, 0, 0, 0, 0, 0, now);
        mutex_enter_blocking(&stats_mutex);
        rtc_track_good_count = 0;
        if (rtc_track_bad_count < UINT32_MAX) {
            rtc_track_bad_count++;
        }
        g_rtc_state = (rtc_track_bad_count >= RTC_FAULT_BAD_COUNT)
            ? RTC_STATE_FAULT
            : RTC_STATE_INIT;
        mutex_exit(&stats_mutex);
        return false;
    }

    snprintf(last_rtc_set_time, sizeof(last_rtc_set_time), "%02d%02d%02d", hour, min, sec);
    snprintf(last_rtc_set_date, sizeof(last_rtc_set_date), "%02d%02d%02d",
             date, month, year % 100);
    rtc_calendar_needs_set = false;
    rtc_cache_update(true, hour, min, sec, dow, date, month, year, now);

    mutex_enter_blocking(&stats_mutex);
    last_rtc_sync_time = now;
    have_rtc_sync = true;
    rtc_sync_interval_ms = RTC_SYNC_INTERVAL_MS;
    rtc_track_good_count = 0;
    rtc_track_bad_count = 0;
    rtc_invalidate_mapping_locked();
    /* Datasheet: rising edge follows the seconds ACK by 500 ms; falling is
     * the expected integer-second edge. Validate it, rediscover if needed. */
    rtc_sqw_second_edge_known = true;
    rtc_sqw_second_edge_high = false;
    rtc_sqw_time_valid = false;
    rtc_sqw_phase_valid = false;
    rtc_sqw_phase_good_count = 0;
    rtc_pending_set_valid = false;
    rtc_pending_set_due_ns = 0;
    if (phase_rephase) {
        last_rtc_rephase_time = now;
        rtc_set_bias_cal_pending = true;
        if (rtc_sqw_rephase_count < UINT32_MAX) {
            rtc_sqw_rephase_count++;
        }
    }
    g_rtc_state = RTC_STATE_TRACKING;
    mutex_exit(&stats_mutex);

    mutex_enter_blocking(&stats_mutex);
    rtc_drift_reset();
    rtc_phase_drift_reset();
    mutex_exit(&stats_mutex);
    DBG_PRINTF("RTC sync: set to %02d/%02d/%02d %02d:%02d:%02d UTC%s\n",
               date, month, year % 100, hour, min, sec,
               phase_rephase ? " (PPS rephase)" : "");
    return true;
}

static void rtc_apply_pending_set_if_due(void) {
    bool pending = false;
    bool phase_rephase = false;
    int64_t epoch_sec = 0;
    uint64_t due_ns = 0;
    uint64_t now_ns = hires_time_ns();

    mutex_enter_blocking(&stats_mutex);
    if (rtc_pending_set_valid && (!gnss_recovery.locked ||
        !timing_edge_fresh(now_ns, trusted_pps_ns, PPS_PRESENT_TIMEOUT_US * 1000ULL))) {
        rtc_pending_set_valid = false;
        have_rtc_sync = false;
    }
    if (rtc_pending_set_valid && pps_time_valid && pll_valid) {
        pending = true;
        epoch_sec = rtc_pending_set_epoch;
        phase_rephase = rtc_pending_set_phase;
        due_ns = rtc_pending_set_due_ns;
    }
    mutex_exit(&stats_mutex);

    if (!pending) {
        return;
    }

    if (due_ns > 0 && now_ns < due_ns) {
        return;
    }
    if (due_ns && now_ns - due_ns > 20000000ULL) {
        mutex_enter_blocking(&stats_mutex);
        rtc_pending_set_valid = false;
        have_rtc_sync = false;
        mutex_exit(&stats_mutex);
        return;
    }

    if (!rtc_set_epoch_utc(epoch_sec, phase_rephase)) {
        DBG_PRINTF("RTC sync: FAILED to set time over I2C%s\n",
                   phase_rephase ? " during PPS rephase" : "");
    }
}

static bool parse_hhmmss(const char *raw, int *hour, int *min, int *sec) {
    if (!raw || strlen(raw) < 6) {
        return false;
    }
    if (raw[0] < '0' || raw[0] > '9' || raw[1] < '0' || raw[1] > '9' ||
        raw[2] < '0' || raw[2] > '9' || raw[3] < '0' || raw[3] > '9' ||
        raw[4] < '0' || raw[4] > '9' || raw[5] < '0' || raw[5] > '9') {
        return false;
    }
    *hour = (raw[0] - '0') * 10 + (raw[1] - '0');
    *min = (raw[2] - '0') * 10 + (raw[3] - '0');
    *sec = (raw[4] - '0') * 10 + (raw[5] - '0');
    return *hour <= 23 && *min <= 59 && *sec <= 59;
}

static bool parse_ddmmyy(const char *raw, int *date, int *month, int *year) {
    if (!raw || strlen(raw) < 6) {
        return false;
    }
    if (raw[0] < '0' || raw[0] > '9' || raw[1] < '0' || raw[1] > '9' ||
        raw[2] < '0' || raw[2] > '9' || raw[3] < '0' || raw[3] > '9' ||
        raw[4] < '0' || raw[4] > '9' || raw[5] < '0' || raw[5] > '9') {
        return false;
    }
    *date = (raw[0] - '0') * 10 + (raw[1] - '0');
    *month = (raw[2] - '0') * 10 + (raw[3] - '0');
    *year = 2000 + (raw[4] - '0') * 10 + (raw[5] - '0');
    return *month >= 1 && *month <= 12 && *date >= 1 &&
           *date <= days_in_month(*year, *month);
}

/* Called with stats_mutex held. Ordinary GNSS loss deliberately does not use
 * this path: a learned epoch/phase mapping survives an ordinary outage. */
static void rtc_invalidate_mapping_locked(void) {
    memset(&rtc_32k, 0, sizeof(rtc_32k));
    rtc_sqw_time_valid = false;
    rtc_sqw_phase_valid = false;
    rtc_sqw_phase_good_count = 0;
    rtc_sqw_second_edge_known = false;
    rtc_sqw_pll_valid = false;
    prev_rtc_sqw_time_ns = 0;
    last_rtc_sqw_second_time_ns = 0;
    memset(&rtc_phase_filter, 0, sizeof(rtc_phase_filter));
    memset(&rtc_frequency_filter, 0, sizeof(rtc_frequency_filter));
    rtc_last_gnss_observation_ns = 0;
    rtc_last_qualified_ns = 0;
    rtc_bad_phase_count = 0;
    g_rtc_state = rtc_calendar_valid ? RTC_STATE_TRACKING : RTC_STATE_MISSING;
}

static void rtc_calibration_poll(void) {
    uint64_t now_ns = hires_time_ns();
    if (now_ns < rtc_conversion_poll_ns) return;
    rtc_conversion_poll_ns = now_ns + (rtc_conversion_pending ? 10000000ULL : 1000000000ULL);
    if (rtc_conversion_pending) {
        uint8_t status = 0, control = 0;
        bool read_ok = rtc_read_reg(RTC_STATUS_REG, &status) && rtc_read_reg(RTC_CONTROL_REG, &control);
        if (read_ok && !rtc_conversion_started && !(status & 4u) && !(control & 0x20u)) {
            rtc_conversion_started = rtc_write_reg(RTC_CONTROL_REG, control | 0x20u);
        } else if (read_ok && rtc_conversion_started && !(status & 4u) && !(control & 0x20u)) {
            rtc_conversion_pending = false;
            mutex_enter_blocking(&stats_mutex);
            rtc_invalidate_mapping_locked();
            mutex_exit(&stats_mutex);
        }
        if (now_ns > rtc_conversion_deadline_ns) {
            rtc_conversion_pending = false;
            mutex_enter_blocking(&stats_mutex);
            rtc_conversion_failed = true;
            mutex_exit(&stats_mutex);
        }
        return;
    }
    uint8_t temp_high = 0, temp_low = 0;
    bool temp_ok = rtc_read_reg(0x11, &temp_high) && rtc_read_reg(0x12, &temp_low);
    mutex_enter_blocking(&stats_mutex);
    rtc_temperature_valid = temp_ok;
    if (temp_ok) rtc_temperature_quarters = (int16_t)((int8_t)temp_high * 4 + (temp_low >> 6));
    bool can_adjust = RTC_AGING_ENABLE && !rtc_conversion_failed && gnss_recovery.locked &&
        pps_time_valid && pll_valid && rtc_phase_filter.qualified && rtc_frequency_filter.valid &&
        rtc_frequency_filter.duration_ns >= (uint64_t)RTC_AGING_WINDOW_SEC * 1000000000ULL &&
        rtc_frequency_filter.count >= RTC_AGING_MIN_SAMPLES &&
        rtc_frequency_filter.uncertainty_ppm <= RTC_AGING_MAX_UNCERTAINTY_PPM &&
        now_ns >= rtc_last_gnss_observation_ns && now_ns - rtc_last_gnss_observation_ns < 1500000000ULL &&
        absolute_time_diff_us(last_aging_adjust_time, get_absolute_time()) >= (int64_t)RTC_AGING_WINDOW_SEC * 1000000LL;
    double fast_ppm = rtc_frequency_filter.frequency_error_ppm;
    double uncertainty_ppm = rtc_frequency_filter.uncertainty_ppm;
    mutex_exit(&stats_mutex);
    if (!can_adjust || fabs(fast_ppm) < 3.0 * uncertainty_ppm) return;
    int8_t current = 0;
    if (!rtc_read_aging(&current)) return;
#if RTC_AGING_INVERT
    fast_ppm = -fast_ppm;
#endif
    int delta = rtc_aging_delta(fast_ppm, RTC_AGING_PPM_PER_LSB, RTC_AGING_MAX_STEP, current);
    if (!delta || !rtc_write_aging((int8_t)(current + delta))) return;
    mutex_enter_blocking(&stats_mutex);
    last_aging_adjust_time = get_absolute_time();
    rtc_invalidate_mapping_locked();
    rtc_drift_reset();
    rtc_phase_drift_reset();
    mutex_exit(&stats_mutex);
    rtc_conversion_pending = true;
    rtc_conversion_started = false;
    rtc_conversion_deadline_ns = now_ns + 2000000000ULL;
}

static void discipline_rtc_from_gps(const gps_stats_t *stats) {
    int64_t gps_epoch = 0;
    if (!gps_fields_to_adjusted_epoch(stats->time_utc, stats->date, &gps_epoch)) return;
    absolute_time_t now = get_absolute_time();
    mutex_enter_blocking(&stats_mutex);
    bool due = !have_rtc_sync || absolute_time_diff_us(last_rtc_sync_time, now) / 1000 >= rtc_sync_interval_ms;
    bool locked = pps_time_valid && pll_valid && gnss_recovery.locked;
    mutex_exit(&stats_mutex);
    if (!due || rtc_conversion_pending) return;
    int hour, min, sec, dow, date, month, year;
    bool read_ok = rtc_read_time(&hour, &min, &sec, &dow, &date, &month, &year);
    if (read_ok) {
        rtc_cache_update(true, hour, min, sec, dow, date, month, year, now);
        int64_t rtc_epoch = utc_to_epoch(year, month, date, hour, min, sec);
        int64_t delta_s = llabs(gps_epoch - rtc_epoch);
        mutex_enter_blocking(&stats_mutex);
        rtc_drift_update(gps_epoch, rtc_epoch, delta_s);
        mutex_exit(&stats_mutex);
        /* Asynchronous whole-second reads may disagree by one second. A
         * stable SQW phase offset is handled in software, never by a write. */
        if (delta_s < RTC_SYNC_FORCE_SET_SEC) {
            mutex_enter_blocking(&stats_mutex);
            have_rtc_sync = true;
            last_rtc_sync_time = now;
            rtc_sync_interval_ms = delta_s <= 1 ? RTC_SYNC_EXTENDED_MS : RTC_SYNC_INTERVAL_MS;
            rtc_track_bad_count = 0;
            if (rtc_track_good_count < UINT32_MAX) ++rtc_track_good_count;
            g_rtc_state = rtc_phase_filter.qualified ? RTC_STATE_LOCKED : RTC_STATE_TRACKING;
            mutex_exit(&stats_mutex);
            return;
        }
    }
    if (locked) {
        mutex_enter_blocking(&stats_mutex);
        uint64_t due_ns = last_pps_time_ns + pll_interval_ns;
        int64_t set_epoch = pps_epoch_seconds + 1;
        while (due_ns <= hires_time_ns()) { due_ns += pll_interval_ns; ++set_epoch; }
        rtc_schedule_set_locked(set_epoch, false, due_ns);
        mutex_exit(&stats_mutex);
    } else if (!read_ok) {
        (void)rtc_set_epoch_utc(gps_epoch, false);
    }
}
static bool parse_gga(char *line, gps_stats_t *stats) {
    char *fields[20] = {0};
    int count = split_fields(line, fields, 20);
    if (count < 10) {
        return false;
    }

    int hour = 0, min = 0, sec = 0;
    if (!parse_hhmmss(fields[1], &hour, &min, &sec)) {
        return false;
    }

    set_time_field(fields[1], stats->time_utc, sizeof(stats->time_utc));

    stats->lat_valid = parse_deg_min(fields[2], fields[3], true, &stats->lat_deg);
    stats->lon_valid = parse_deg_min(fields[4], fields[5], false, &stats->lon_deg);

    int fix_quality = 0;
    int satellites = 0;
    if (!parse_nonnegative_int(fields[6], &fix_quality) ||
        !parse_nonnegative_int(fields[7], &satellites) ||
        fix_quality > 8 || satellites > 99) {
        return false;
    }
    stats->fix_quality = fix_quality;
    stats->satellites = satellites;
    stats->hdop = gps_number_or_unknown(fields[8], 0.0f);
    stats->altitude_m = gps_number_or_unknown(fields[9], -FLT_MAX);

    stats->has_fix = stats->fix_quality > 0;
    last_gga_update_ns = hires_time_ns();
    return true;
}

static void update_gsv_count(const char *talker, uint8_t count, gps_stats_t *stats) {
    if (!talker || !stats) return;
    if (talker[0] == 'G' && talker[1] == 'P') {
        stats->gsv_gps = count;
    } else if (talker[0] == 'G' && talker[1] == 'L') {
        stats->gsv_glo = count;
    } else if (talker[0] == 'G' && talker[1] == 'A') {
        stats->gsv_gal = count;
    } else if ((talker[0] == 'G' && talker[1] == 'B') || (talker[0] == 'B' && talker[1] == 'D')) {
        stats->gsv_bds = count;
    } else if (talker[0] == 'G' && talker[1] == 'Q') {
        stats->gsv_qzss = count;
    }
}

static void update_gsa_fix(const char *talker, uint8_t fix_type, gps_stats_t *stats) {
    if (!talker || !stats) return;
    if (talker[0] == 'G' && talker[1] == 'P') {
        stats->gsa_fix_gps = fix_type;
    } else if (talker[0] == 'G' && talker[1] == 'L') {
        stats->gsa_fix_glo = fix_type;
    } else if (talker[0] == 'G' && talker[1] == 'A') {
        stats->gsa_fix_gal = fix_type;
    } else if ((talker[0] == 'G' && talker[1] == 'B') || (talker[0] == 'B' && talker[1] == 'D')) {
        stats->gsa_fix_bds = fix_type;
    } else if (talker[0] == 'G' && talker[1] == 'N') {
        stats->gsa_fix_gn = fix_type;
    }
}

static bool parse_gsv(char *line, gps_stats_t *stats) {
    char *fields[20] = {0};
    int count = split_fields(line, fields, 20);
    if (count < 4) {
        return false;
    }
    const char *talker = line + 1;
    if (!talker || strlen(talker) < 5) {
        return false;
    }
    uint8_t total_in_view = (fields[3] && *fields[3]) ? (uint8_t)atoi(fields[3]) : 0;
    update_gsv_count(talker, total_in_view, stats);
    return true;
}

static bool parse_gsa(char *line, gps_stats_t *stats) {
    char *fields[20] = {0};
    int count = split_fields(line, fields, 20);
    if (count < 3) {
        return false;
    }
    const char *talker = line + 1;
    if (!talker || strlen(talker) < 5) {
        return false;
    }
    uint8_t fix_type = (fields[2] && *fields[2]) ? (uint8_t)atoi(fields[2]) : 0;
    update_gsa_fix(talker, fix_type, stats);
    uint8_t used = 0;
    for (int i = 3; i <= 14 && i < count; ++i) {
        if (fields[i] && *fields[i]) ++used;
    }
    if (talker[0] == 'G' && talker[1] == 'P') {
        stats->nmea_used_gps = used;
    } else if (talker[0] == 'G' && talker[1] == 'L') {
        stats->nmea_used_glo = used;
    } else if (talker[0] == 'G' && talker[1] == 'A') {
        stats->nmea_used_gal = used;
    } else if ((talker[0] == 'G' && talker[1] == 'B') || (talker[0] == 'B' && talker[1] == 'D')) {
        stats->nmea_used_bds = used;
    } else if (talker[0] == 'G' && talker[1] == 'N') {
        stats->nmea_used_gn = used;
    }

    // PDOP/HDOP/VDOP are typically fields 15/16/17 in GSA.
    if (count > 15 && fields[15] && *fields[15]) {
        stats->pdop = gps_number_or_unknown(fields[15], 0.0f);
    }
    if (count > 16 && fields[16] && *fields[16]) {
        stats->hdop = gps_number_or_unknown(fields[16], 0.0f);
    }
    if (count > 17 && fields[17] && *fields[17]) {
        stats->vdop = gps_number_or_unknown(fields[17], 0.0f);
    }
    return true;
}

static bool parse_rmc(char *line, gps_stats_t *stats, uint64_t line_first_byte_ns) {
    char *fields[20] = {0};
    int count = split_fields(line, fields, 20);
    if (count < 10) {
        return false;
    }

    int hour = 0, min = 0, sec = 0;
    int date = 0, month = 0, year = 0;
    if (!parse_hhmmss(fields[1], &hour, &min, &sec) ||
        !parse_ddmmyy(fields[9], &date, &month, &year)) {
        return false;
    }

    set_time_field(fields[1], stats->time_utc, sizeof(stats->time_utc));

    char status = fields[2] && *fields[2] ? fields[2][0] : 'V';
    bool rmc_status_valid = (status == 'A');

    if (fields[7] && *fields[7]) {
        char *end = NULL;
        float speed = strtof(fields[7], &end);
        if (end != fields[7] && *end == '\0' && isfinite(speed) && speed >= 0.0f) {
            stats->speed_knots = speed;
            stats->speed_valid = true;
        } else {
            stats->speed_valid = false;
        }
    }

    set_date_field(fields[9], stats->date, sizeof(stats->date));

    if (fields[3] && *fields[3] && fields[4] && *fields[4]) {
        stats->lat_valid = parse_deg_min(fields[3], fields[4], true, &stats->lat_deg);
    }
    if (fields[5] && *fields[5] && fields[6] && *fields[6]) {
        stats->lon_valid = parse_deg_min(fields[5], fields[6], false, &stats->lon_deg);
    }

    uint64_t now_ns = hires_time_ns();
    bool quality_fresh = gps_quality_is_fresh(now_ns, last_gga_update_ns,
                                              GPS_QUALITY_MAX_AGE_MS);
    bool quality_ok = quality_fresh &&
                      stats->fix_quality >= RMC_MIN_FIX_QUALITY &&
                      stats->satellites >= RMC_MIN_SATS &&
                      stats->hdop >= 0.0f && stats->hdop <= RMC_MAX_HDOP;
    // RMC status A is the receiver's UTC-time validity indication. Do not
    // discard valid UTC/PPS timing just because the position-quality fields
    // temporarily fall outside the navigation display thresholds. Those
    // thresholds are useful for position reporting, but tying them to time
    // acceptance caused the L76K to enter RTC holdover while its PPS output
    // was still present.
    (void)quality_ok;
    stats->rmc_valid = rmc_status_valid;
    bool time_ok = rmc_status_valid;

    if (!time_ok) {
        __atomic_add_fetch(&gps_rejected_fix_count, 1, __ATOMIC_RELAXED);
    }

    if (time_ok) {
        have_gps_time = true;
        last_gps_fix_time = get_absolute_time();
        last_rmc_update_time = get_absolute_time();
        rmc_epoch_time_ns = line_first_byte_ns ? line_first_byte_ns : hires_time_ns();
        int64_t adjusted_epoch = 0;
        if (gps_fields_to_adjusted_epoch(stats->time_utc, stats->date, &adjusted_epoch)) {
            rmc_epoch_seconds = adjusted_epoch;
            rmc_pps_update_stats(rmc_epoch_time_ns);
            if (rmc_within_pps_window(rmc_epoch_time_ns / 1000ULL)) {
                if (rmc_phase_ok_count < 255) {
                    rmc_phase_ok_count++;
                }
                rmc_epoch_valid = (rmc_phase_ok_count >= RMC_PPS_MIN_CONSISTENT);
            } else {
                rmc_epoch_valid = false;
                rmc_phase_ok_count = 0;
                int64_t rmc_us = (int64_t)(rmc_epoch_time_ns / 1000ULL);
                int64_t pps_us = (int64_t)(get_last_pps_time_ns() / 1000ULL);
                int64_t phase_us = rmc_us - pps_us;
                DBG_PRINTF("RMC rejected: PPS phase=%lldus learned=%ldus window=%ldus\n",
                           (long long)phase_us,
                           (long)rmc_pps_avg_us,
                           (long)rmc_pps_window_us);
            }
        }
    }

    return true;
}

#if GPS_PROTOCOL_UBX
// Caller must hold stats_mutex. NAV-PVT supplies validated UTC/navigation data;
// the physical TIMEPULSE edge remains the precise clock reference.
static bool process_ubx_nav_pvt(const ubx_frame_t *frame) {
    ubx_nav_pvt_t pvt;
    if (!ubx_decode_nav_pvt(frame, &pvt)) return false;

    bool calendar_valid = pvt.year >= 2000u && pvt.month >= 1u && pvt.month <= 12u &&
                          pvt.day >= 1u && pvt.day <= (uint8_t)days_in_month(pvt.year, pvt.month) &&
                          pvt.hour <= 23u && pvt.min <= 59u && pvt.sec <= 59u;
    bool time_valid = calendar_valid && (pvt.valid & 0x07u) == 0x07u &&
                      (pvt.flags & 0x01u) != 0;
    g_stats.fix_quality = pvt.fix_type;
    g_stats.has_fix = (pvt.flags & 0x01u) != 0 && pvt.fix_type >= 2;
    g_stats.satellites = pvt.satellites;
    // NAV-PVT reports PDOP only; do not present it as HDOP.
    g_stats.hdop = -1.0f;
    g_stats.pdop = (float)pvt.pdop_100 / 100.0f;
    g_stats.vdop = -1.0f;
    g_stats.lat_deg = (float)pvt.latitude_1e7 / 10000000.0f;
    g_stats.lon_deg = (float)pvt.longitude_1e7 / 10000000.0f;
    g_stats.lat_valid = g_stats.has_fix && pvt.latitude_1e7 >= -900000000 &&
                        pvt.latitude_1e7 <= 900000000;
    g_stats.lon_valid = g_stats.has_fix && pvt.longitude_1e7 >= -1800000000 &&
                        pvt.longitude_1e7 <= 1800000000;
    g_stats.altitude_m = (float)pvt.height_msl_mm / 1000.0f;
    g_stats.speed_knots = (float)pvt.ground_speed_mm_s * 1.943844f / 1000.0f;
    g_stats.speed_valid = g_stats.has_fix;
    g_stats.rmc_valid = time_valid;
    snprintf(g_stats.time_utc, sizeof(g_stats.time_utc), "%02u%02u%02u",
             (unsigned)pvt.hour, (unsigned)pvt.min, (unsigned)pvt.sec);
    snprintf(g_stats.date, sizeof(g_stats.date), "%02u%02u%02u",
             (unsigned)pvt.day, (unsigned)pvt.month, (unsigned)(pvt.year % 100u));

    if (!time_valid) {
        __atomic_add_fetch(&gps_rejected_fix_count, 1, __ATOMIC_RELAXED);
        return true;
    }

    int64_t epoch = utc_to_epoch((int)pvt.year, (int)pvt.month, (int)pvt.day,
                                 (int)pvt.hour, (int)pvt.min, (int)pvt.sec);
    epoch += GPS_NMEA_TIME_OFFSET_SEC;
    have_gps_time = true;
    last_gps_fix_time = get_absolute_time();
    last_rmc_update_time = get_absolute_time();
    rmc_epoch_seconds = epoch;
    rmc_epoch_time_ns = frame->first_byte_ns;
    rmc_pps_update_stats(rmc_epoch_time_ns);
    if (rmc_within_pps_window(rmc_epoch_time_ns / 1000ULL)) {
        if (rmc_phase_ok_count < UINT8_MAX) rmc_phase_ok_count++;
        rmc_epoch_valid = rmc_phase_ok_count >= RMC_PPS_MIN_CONSISTENT;
    } else {
        rmc_epoch_valid = false;
        rmc_phase_ok_count = 0;
    }
    return true;
}

static void process_ubx_frame(const ubx_frame_t *frame) {
    if (!frame) return;
    gps_stats_t stats_copy = {0};
    bool discipline_needed = false;
    mutex_enter_blocking(&stats_mutex);
    if (frame->cls == 0x01u && frame->id == 0x07u) {
        bool parsed = process_ubx_nav_pvt(frame);
        update_time_snapshot_locked(hires_time_ns());
        if (parsed && g_stats.rmc_valid) {
            stats_copy = g_stats;
            discipline_needed = true;
        }
    } else if (frame->cls == 0x0Du && frame->id == 0x01u) {
        ubx_tim_tp_t tim_tp;
        // Decode and validate TIM-TP. The PPS interrupt remains authoritative
        // for the actual edge timestamp and NAV-PVT supplies UTC date/time.
        (void)ubx_decode_tim_tp(frame, &tim_tp);
    } else if (frame->cls == 0x01u && frame->id == 0x35u) {
        ubx_nav_sat_summary_t sat;
        if (ubx_decode_nav_sat(frame, &sat)) {
            g_stats.ubx_sat_total = sat.total;
            g_stats.ubx_sat_used = sat.used;
            g_stats.ubx_sat_gps = sat.gps_total;
            g_stats.ubx_sat_galileo = sat.galileo_total;
            g_stats.ubx_sat_beidou = sat.beidou_total;
            g_stats.ubx_sat_glonass = sat.glonass_total;
            g_stats.ubx_sat_qzss = sat.qzss_total;
            g_stats.ubx_sat_sbas = sat.sbas_total;
            g_stats.ubx_used_gps = sat.gps_used;
            g_stats.ubx_used_galileo = sat.galileo_used;
            g_stats.ubx_used_beidou = sat.beidou_used;
            g_stats.ubx_used_glonass = sat.glonass_used;
            g_stats.ubx_used_qzss = sat.qzss_used;
            g_stats.ubx_used_sbas = sat.sbas_used;
        }
    }
    mutex_exit(&stats_mutex);
    if (discipline_needed) {
        discipline_rtc_from_gps(&stats_copy);
    }
}
#endif

static void print_stats(const gps_stats_t *stats, const time_source_t *ts_in) {
    static bool last_using_rtc = false;
    char time_buf[9];
    char date_buf[9];
    char lat_buf[16];
    char lon_buf[16];
    char hdop_buf[12];
    char alt_buf[12];
    char spd_buf[12];

    time_source_t ts = {0};
    if (ts_in) {
        ts = *ts_in;
    } else {
        ts = choose_time_source(stats);
    }
    bool using_rtc = ts.using_rtc && ts.valid;

    if (ts.valid) {
        format_rtc_time(ts.hour, ts.min, ts.sec, time_buf, sizeof(time_buf));
        format_rtc_date(ts.date, ts.month, ts.year, date_buf, sizeof(date_buf));
    } else {
        format_time(stats->time_utc, time_buf, sizeof(time_buf));
        format_date(stats->date, date_buf, sizeof(date_buf));
    }

    if (stats->lat_valid) {
        snprintf(lat_buf, sizeof(lat_buf), "%.5f", (double)stats->lat_deg);
    } else {
        snprintf(lat_buf, sizeof(lat_buf), "N/A");
    }
    if (stats->lon_valid) {
        snprintf(lon_buf, sizeof(lon_buf), "%.5f", (double)stats->lon_deg);
    } else {
        snprintf(lon_buf, sizeof(lon_buf), "N/A");
    }
    if (stats->hdop >= 0.0f) {
        snprintf(hdop_buf, sizeof(hdop_buf), "%.1f", (double)stats->hdop);
    } else {
        snprintf(hdop_buf, sizeof(hdop_buf), "N/A");
    }
    if (stats->altitude_m >= 0.0f) {
        snprintf(alt_buf, sizeof(alt_buf), "%.1f", (double)stats->altitude_m);
    } else {
        snprintf(alt_buf, sizeof(alt_buf), "N/A");
    }
    if (stats->speed_valid) {
        snprintf(spd_buf, sizeof(spd_buf), "%.1f", (double)stats->speed_knots);
    } else {
        snprintf(spd_buf, sizeof(spd_buf), "N/A");
    }

    if (using_rtc != last_using_rtc) {
        DBG_PRINTF("Time source switched to %s\n", using_rtc ? "RTC holdover" : "GPS");
        last_using_rtc = using_rtc;
    }

    DBG_PRINTF("Fix:%s Q:%d Sats:%d HDOP:%s Alt:%sm Lat:%s Lon:%s Speed:%skt Time:%s Date:%s Src:%s\n",
           stats->has_fix ? "Y" : "N",
           stats->fix_quality,
           stats->satellites,
           hdop_buf,
           alt_buf,
           lat_buf,
           lon_buf,
           spd_buf,
           time_buf,
           date_buf,
           using_rtc ? "RTC" : "GPS");
}

static void get_ip_string(char *out, size_t out_len) {
    if (!out || out_len == 0) {
        return;
    }
    struct netif *n = netif_list;
    if (!n) {
        snprintf(out, out_len, "0.0.0.0");
        return;
    }
    ip4_addr_t ip = *netif_ip4_addr(n);
    ip4addr_ntoa_r(&ip, out, out_len);
}

static void sntp_write_u32(uint8_t *buf, uint32_t value) {
    buf[0] = (uint8_t)(value >> 24);
    buf[1] = (uint8_t)(value >> 16);
    buf[2] = (uint8_t)(value >> 8);
    buf[3] = (uint8_t)(value);
}

static void sntp_fill_timestamp(uint8_t *buf, uint32_t seconds, uint32_t fraction) {
    sntp_write_u32(buf, seconds);
    sntp_write_u32(buf + 4, fraction);
}

#if HEALTH_BROADCAST_ENABLE && HEALTH_BROADCAST_CRC
static uint32_t crc32_ieee(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 1u) {
                crc = (crc >> 1) ^ 0xEDB88320u;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}


#endif

static uint64_t get_last_pps_time_ns(void) {
    uint32_t save = save_and_disable_interrupts();
    uint64_t t = last_pps_time_ns;
    restore_interrupts(save);
    return t;
}

static uint64_t get_last_rtc_sqw_second_time_ns(void) {
    uint32_t save = save_and_disable_interrupts();
    uint64_t t = last_rtc_sqw_second_time_ns;
    restore_interrupts(save);
    return t;
}

static bool rmc_within_pps_window(uint64_t rmc_us) {
    uint64_t last_pps = get_last_pps_time_ns() / 1000ULL;
    if (last_pps == 0) {
        return true;
    }
    int64_t phase = (int64_t)rmc_us - (int64_t)last_pps;
    uint32_t max_offset_us = (rmc_pps_samples < RMC_PPS_BOOTSTRAP_SAMPLES)
        ? RMC_PPS_BOOTSTRAP_MAX_OFFSET_US
        : RMC_PPS_MAX_OFFSET_US;
    int32_t window = rmc_pps_window_us > 0 ? rmc_pps_window_us : RMC_PPS_MAX_OFFSET_US;
    int32_t minimum_window = RMC_PPS_MIN_OFFSET_US;
#if ACCURACY_MODE
    if (accuracy_mode && minimum_window > ACCURACY_RMC_MAX_WINDOW_US) {
        minimum_window = ACCURACY_RMC_MAX_WINDOW_US;
    }
#endif
    if (window < minimum_window) {
        window = minimum_window;
    } else if (window > (int32_t)max_offset_us) {
        window = (int32_t)max_offset_us;
    }
    if (rmc_pps_samples < RMC_PPS_BOOTSTRAP_SAMPLES) {
        return llabs((long long)phase) <= (int64_t)max_offset_us;
    }
    return llabs((long long)phase - (long long)rmc_pps_avg_us) <= (int64_t)window;
}

/* Caller holds stats_mutex, shared with the timestamp/source snapshot. */
static ntp_uncertainty_t get_pps_uncertainty_locked(uint64_t now_ns) {
    uint64_t last_pps_ns = trusted_pps_ns;
    uint32_t samples = pps_jitter_count;
    uint64_t rms_ns = samples && pps_jitter_ns_sumsq > 0 ?
        (uint64_t)ceil(sqrt((double)pps_jitter_ns_sumsq / samples)) : 0;
    uint64_t mad_ns = ab_mad_ns();
    int64_t innovation_ns = ab_innovation_ns;
    int64_t slew_ns = utc_continuity.residual_ns;
    uint64_t age_ns = last_pps_ns && now_ns >= last_pps_ns ? now_ns - last_pps_ns : UINT64_MAX;
    /* Bound actual reconstructed epoch placement against physical pin edges,
     * not just the much smaller PIO interval quantization. */
    uint64_t capture_ns = (1000000000ULL + PPS_CAPTURE_COUNTER_HZ - 1) / PPS_CAPTURE_COUNTER_HZ;
    ntp_placement_t placement = ntp_placement_details(last_pps_ns, pps_placement_brackets);
    capture_ns = ntp_saturating_add(capture_ns, placement.bound_ns);
    ntp_uncertainty_t u = ntp_pps_uncertainty(age_ns, capture_ns, rms_ns, mad_ns,
                               innovation_ns, samples, slew_ns);
    u.placement_valid = placement.valid;
    u.placement_window_ns = placement.window_ns;
    u.placement_offset_ns = placement.offset_ns;
    return u;
}

static ntp_uncertainty_t get_coarse_uncertainty_locked(sntp_quality_t quality, uint64_t now_ns) {
    absolute_time_t sample_time = {0};
    bool valid = false;
    if (quality == SNTP_QUALITY_COARSE_RTC && g_rtc_cache.valid && g_rtc_cache.status) {
        sample_time = g_rtc_cache.sample_time;
        valid = true;
    } else if (quality == SNTP_QUALITY_COARSE_GPS && have_gps_time) {
        sample_time = last_rmc_update_time;
        valid = true;
    }
    uint64_t sample_us = to_us_since_boot(sample_time);
    valid = valid && sample_us <= UINT64_MAX / 1000ULL;
    uint64_t sample_ns = valid ? sample_us * 1000ULL : 0;
    uint64_t age_ns = now_ns >= sample_ns ? now_ns - sample_ns : 0;
    return ntp_coarse_uncertainty(age_ns, valid, utc_continuity.residual_ns);
}

static ntp_uncertainty_t get_ntp_uncertainty_locked(sntp_quality_t quality, uint64_t now_ns) {
    if (quality == SNTP_QUALITY_PPS) return get_pps_uncertainty_locked(now_ns);
    ntp_uncertainty_t u = {0};
    if (quality == SNTP_QUALITY_RTC_SQW) {
        rtc_holdover_t h = rtc_holdover;
        u.slew_ns = ntp_abs_ns(utc_continuity.residual_ns);
        u.age_ns = h.active && now_ns >= h.start_ns ? now_ns - h.start_ns : 0;
        u.capture_ns = h.initial_error_ns;
        u.margin_ns = RTC_HOLDOVER_MARGIN_NS;
        u.frequency_uncertainty_ppm = h.frequency_uncertainty_ppm;
        uint64_t error_ns = rtc_holdover_error_ns(&h, now_ns);
        uint64_t initial_ns = ntp_saturating_add(u.capture_ns, u.margin_ns);
        u.growth_ns = error_ns >= initial_ns ? error_ns - initial_ns : 0;
        u.total_ns = ntp_saturating_add(error_ns, u.slew_ns);
    } else {
        u = get_coarse_uncertainty_locked(quality, now_ns);
        if (quality == SNTP_QUALITY_COARSE_RTC && rtc_holdover.active) {
            uint64_t holdover_ns = ntp_saturating_add(rtc_holdover_error_ns(&rtc_holdover, now_ns), u.slew_ns);
            if (holdover_ns > u.total_ns) {
                /* Preserve expired holdover error instead of hiding it behind
                 * an unrelated coarse-calendar cap. */
                u.capture_ns = holdover_ns - u.total_ns;
                u.total_ns = holdover_ns;
            }
        }
    }
    return u;
}

static bool get_sntp_time_details(uint64_t now_ns, uint32_t *seconds, uint32_t *fraction,
                             bool *synced, sntp_quality_t *quality,
                             ntp_uncertainty_t *uncertainty, uint32_t *reference_seconds) {
    selected_time_t selected;
    mutex_enter_blocking(&stats_mutex);
    select_time_at_locked(now_ns, &selected);
    if (uncertainty) *uncertainty = get_ntp_uncertainty_locked(selected.quality, now_ns);
    if (uncertainty && utc_continuity.initialized && utc_continuity.counter_ns > now_ns) {
        /* Historical samples use the last segment's rate. Allow for an
         * unknown earlier slew direction, bounded by +/-UTC_MAX_SLEW_PPM. */
        uint64_t readback_ns = utc_continuity.counter_ns - now_ns;
        uint64_t history_error_ns = (uint64_t)ceil((double)readback_ns *
            (2.0 * UTC_MAX_SLEW_PPM) / 1000000.0);
        uncertainty->slew_ns = ntp_saturating_add(uncertainty->slew_ns, history_error_ns);
        uncertainty->total_ns = ntp_saturating_add(uncertainty->total_ns, history_error_ns);
    }
    if (selected.quality == SNTP_QUALITY_PPS &&
        ntp_placement_bound_ns(trusted_pps_ns, pps_placement_brackets) == UINT64_MAX) {
        /* A real interval lock is not proof of bounded absolute alignment. */
        selected.quality = SNTP_QUALITY_COARSE_GPS;
    }
    if (reference_seconds) {
        *reference_seconds = selected.quality == SNTP_QUALITY_PPS ?
            (uint32_t)(trusted_pps_epoch_s + NTP_DELTA) :
            selected.quality == SNTP_QUALITY_RTC_SQW ?
            (uint32_t)(rtc_last_qualified_epoch_s + NTP_DELTA) : 0;
    }
    mutex_exit(&stats_mutex);

    if (!selected.valid) {
        if (synced) *synced = false;
        if (quality) *quality = SNTP_QUALITY_NONE;
        return false;
    }

    if (seconds) *seconds = (uint32_t)(selected.unix_sec + NTP_DELTA);
    if (fraction) *fraction = selected.fraction;

    if (synced) *synced = (selected.quality == SNTP_QUALITY_PPS ||
                           selected.quality == SNTP_QUALITY_RTC_SQW);
    if (quality) *quality = selected.quality;
    return true;
}

static bool get_sntp_time_at(uint64_t now_ns, uint32_t *seconds, uint32_t *fraction,
                             bool *synced, sntp_quality_t *quality) {
    return get_sntp_time_details(now_ns, seconds, fraction, synced, quality, NULL, NULL);
}

static void sntp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                      const ip_addr_t *addr, u16_t port) {
    (void)arg;
    if (!p) {
        return;
    }

    if (p->tot_len < 48) {
        __atomic_add_fetch(&sntp_malformed_count, 1, __ATOMIC_RELAXED);
        pbuf_free(p);
        return;
    }
    uint8_t request_header[4];
    pbuf_copy_partial(p, request_header, sizeof(request_header), 0);
    uint8_t request_version = (request_header[0] >> 3) & 7u;
    uint8_t request_mode = request_header[0] & 7u;
    if (request_mode != 3 || (request_version != 3 && request_version != 4)) {
        __atomic_add_fetch(&sntp_malformed_count, 1, __ATOMIC_RELAXED);
        pbuf_free(p);
        return;
    }
    uint64_t rate_now_us = time_us_64();
    if (sntp_rate_window_start_us == 0 ||
        rate_now_us - sntp_rate_window_start_us >= 1000000ULL) {
        sntp_rate_window_start_us = rate_now_us;
        sntp_rate_window_count = 0;
    }
    if (sntp_rate_window_count >= SNTP_MAX_REQUESTS_PER_SECOND) {
        __atomic_add_fetch(&sntp_rate_limit_count, 1, __ATOMIC_RELAXED);
        pbuf_free(p);
        return;
    }
    sntp_rate_window_count++;
    __atomic_add_fetch(&sntp_request_count, 1, __ATOMIC_RELAXED);

    uint64_t recv_ns = hires_time_ns();
    uint8_t resp[48] = {0};
    uint8_t orig_ts[8] = {0};
    pbuf_copy_partial(p, orig_ts, sizeof(orig_ts), 40);
    pbuf_free(p);

    uint32_t ntp_seconds_rx = 0;
    uint32_t ntp_fraction_rx = 0;
    uint32_t ntp_seconds_tx = 0;
    uint32_t ntp_fraction_tx = 0;
    sntp_quality_t rx_quality = SNTP_QUALITY_NONE;
    sntp_quality_t tx_quality = SNTP_QUALITY_NONE;
    ntp_uncertainty_t rx_uncertainty, tx_uncertainty;
    uint32_t reference_seconds = 0;
    bool time_ok_rx = get_sntp_time_details(recv_ns, &ntp_seconds_rx, &ntp_fraction_rx,
        NULL, &rx_quality, &rx_uncertainty, &reference_seconds);

    uint8_t li = time_ok_rx ? 0u : 3u;

    memcpy(resp + 24, orig_ts, sizeof(orig_ts)); // originate timestamp

    if (time_ok_rx) {
        sntp_fill_timestamp(resp + 32, ntp_seconds_rx, ntp_fraction_rx); // receive time
        uint32_t ref_sec = reference_seconds;
        uint32_t ref_frac = 0;
        /* A historical reference timestamp must not mutate the live source
         * selector or continuity model. */
        if (ref_sec) {
            sntp_fill_timestamp(resp + 16, ref_sec, ref_frac);
        }
    }

    /* Allocate before T3: allocator time must not sit after the transmit
     * timestamp. Driver/radio queues still occur after udp_sendto. */
    struct pbuf *q = pbuf_alloc(PBUF_TRANSPORT, sizeof(resp), PBUF_RAM);
    if (!q) {
        __atomic_add_fetch(&sntp_send_error_count, 1, __ATOMIC_RELAXED);
        return;
    }
    uint64_t tx_ns = hires_time_ns();
    bool time_ok_tx = get_sntp_time_details(tx_ns, &ntp_seconds_tx, &ntp_fraction_tx,
        NULL, &tx_quality, &tx_uncertainty, NULL);
    ntp_response_header(resp, rx_quality, tx_quality, time_ok_rx, time_ok_tx,
        li, rx_uncertainty.total_ns, tx_uncertainty.total_ns);
    if (time_ok_tx) {
        sntp_fill_timestamp(resp + 40, ntp_seconds_tx, ntp_fraction_tx); // transmit time
    }

    memcpy(q->payload, resp, sizeof(resp));
    if (udp_sendto(pcb, q, addr, port) != ERR_OK) {
        __atomic_add_fetch(&sntp_send_error_count, 1, __ATOMIC_RELAXED);
    }
    pbuf_free(q);
}

static void sntp_server_init(void) {
    if (sntp_pcb) {
        return;
    }
    sntp_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (!sntp_pcb) {
        DBG_PRINTF("SNTP: pcb alloc failed\n");
        return;
    }
    if (udp_bind(sntp_pcb, IP_ANY_TYPE, NTP_PORT) != ERR_OK) {
        DBG_PRINTF("SNTP: bind failed\n");
        udp_remove(sntp_pcb);
        sntp_pcb = NULL;
        return;
    }
    udp_recv(sntp_pcb, sntp_recv, NULL);
    DBG_PRINTF("SNTP: listening on port %d\n", NTP_PORT);
}


#if HEALTH_BROADCAST_ENABLE
static void health_broadcast_init(void) {
    if (health_pcb) {
        return;
    }
    health_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (!health_pcb) {
        DBG_PRINTF("HEALTH: pcb alloc failed\n");
        return;
    }
    ip_set_option(health_pcb, SOF_BROADCAST);
    if (udp_bind(health_pcb, IP_ANY_TYPE, 0) != ERR_OK) {
        DBG_PRINTF("HEALTH: bind failed\n");
        udp_remove(health_pcb);
        health_pcb = NULL;
        return;
    }
}

static void health_broadcast_send(void) {
    if (!health_pcb) return;

    health_snapshot_t snap;
    get_health_snapshot(&snap);

    uint32_t ntp_sec = 0;
    uint32_t ntp_frac = 0;
    bool synced = false;
    sntp_quality_t active_quality = SNTP_QUALITY_NONE;
    get_sntp_time_at(hires_time_ns(), &ntp_sec, &ntp_frac, &synced, &active_quality);
    uint32_t unix_sec = ntp_sec ? (ntp_sec - NTP_DELTA) : 0;
    char sntp_refid[5];
    sntp_quality_refid(active_quality, sntp_refid);

    uint64_t now_us = to_us_since_boot(get_absolute_time());
    int64_t pps_age_ms = (snap.last_pps_us >= 0)
        ? (int64_t)((now_us - (uint64_t)snap.last_pps_us) / 1000U)
        : -1;
    uint32_t nonce = __atomic_add_fetch(&health_nonce_counter, 1, __ATOMIC_RELAXED);

    char payload[1800];
    int len = snprintf(
        payload, sizeof(payload),
        "{"
        "\"host\":\"%s\","
        "\"unix\":%lu,"
        "\"synced\":%s,"
        "\"src\":\"%s\","
        "\"clock_state\":\"%s\","
        "\"fix\":%s,"
        "\"fixq\":%d,"
        "\"sats\":%d,"
        "\"hdop\":%.2f,"
        "\"pdop\":%.2f,"
        "\"vdop\":%.2f,"
        "\"lat\":%.6f,"
        "\"lon\":%.6f,"
        "\"alt_m\":%.2f,"
        "\"lat_ok\":%s,"
        "\"lon_ok\":%s,"
        "\"nonce\":%lu,"
        "\"uptime_s\":%lu,"
        "\"wifi_link\":%s,"
        "\"wifi_rssi\":%d,"
        "\"ip\":\"%s\","
        "\"gps_line_drops\":%lu,"
        "\"sntp_quality\":\"%s\","
        "\"sntp_stratum\":%u,"
        "\"sntp_refid\":\"%s\","
        "\"rmc_age_ms\":%lld,"
        "\"rmc_pps_offset_us\":%ld,"
        "\"rmc_pps_window_us\":%ld,"
        "\"rtc_offset_s\":%lld,"
        "\"rtc_offset_us\":%lld,"
        "\"rtc_sqw\":%s,"
        "\"rtc_sqw_time\":%s,"
        "\"rtc_sqw_age_ms\":%lld,"
        "\"rtc_sqw_phase_us\":%ld,"
        "\"rtc_sqw_rephase_count\":%lu,"
        "\"pps_jitter_us\":%ld,"
        "\"pps_jitter_min_us\":%ld,"
        "\"pps_jitter_max_us\":%ld,"
        "\"pps_jitter_rms_us\":%lu,"
        "\"pps_jitter_ns\":%lld,"
        "\"pps_jitter_min_ns\":%lld,"
        "\"pps_jitter_max_ns\":%lld,"
        "\"pps_jitter_rms_ns\":%llu,"
        "\"pll_phase_error_ns\":%lld,"
        "\"ab_innovation_ns\":%lld,"
        "\"pps_period_error_ns\":%lld,"
        "\"pps_dma_overruns\":%lu,"
        "\"pps_jitter_samples\":%lu,"
        "\"accuracy_mode\":%s,"
        "\"pll_filter_shift\":%u,"
        "\"discipline_mode\":\"%s\","
        "\"ab_servo_valid\":%s,"
        "\"ab_period_ns\":%llu,"
        "\"ab_rate_ns_per_sec\":%lld,"
        "\"ab_median_ns\":%llu,"
        "\"ab_mad_ns\":%llu,"
        "\"ab_outlier_threshold_ns\":%llu,"
        "\"ab_rejected\":%lu,"
        "\"ab_alpha_shift\":%u,"
        "\"ab_beta_shift\":%u,"
        "\"pps\":%s,"
        "\"pps_time\":%s,"
        "\"pps_age_ms\":%lld,"
        "\"pll\":%s,"
        "\"pll_us\":%u,"
        "\"rtc_sync_age_ms\":%lld,"
        "\"rtc_aging\":%d,"
        "\"rtc_status\":%s,"
        "\"rtc_state\":\"%s\"",
        WIFI_HOSTNAME,
        (unsigned long)unix_sec,
        synced ? "true" : "false",
        snap.ts.using_rtc ? "RTC" : "GPS",
        clock_state_str(snap.clock_state),
        snap.stats.has_fix ? "true" : "false",
        snap.stats.fix_quality,
        snap.stats.satellites,
        (double)snap.stats.hdop,
        (double)snap.stats.pdop,
        (double)snap.stats.vdop,
        (double)snap.stats.lat_deg,
        (double)snap.stats.lon_deg,
        (double)snap.stats.altitude_m,
        snap.stats.lat_valid ? "true" : "false",
        snap.stats.lon_valid ? "true" : "false",
        (unsigned long)nonce,
        (unsigned long)snap.uptime_s,
        snap.wifi_link ? "true" : "false",
        (int)snap.wifi_rssi,
        snap.ip,
        (unsigned long)snap.gps_line_drop_count,
        sntp_quality_str(active_quality),
        (unsigned)sntp_quality_stratum(active_quality),
        sntp_refid,
        (long long)snap.rmc_age_ms,
        (long)snap.rmc_pps_offset_us,
        (long)snap.rmc_pps_window_us,
        (long long)snap.rtc_offset_s,
        (long long)snap.rtc_offset_us,
        snap.rtc_sqw_present ? "true" : "false",
        snap.rtc_sqw_time_valid ? "true" : "false",
        (long long)snap.rtc_sqw_age_ms,
        (long)snap.rtc_sqw_phase_us,
        (unsigned long)snap.rtc_sqw_rephase_count,
        (long)snap.pps_jitter_last,
        (long)snap.pps_jitter_min,
        (long)snap.pps_jitter_max,
        (unsigned long)snap.pps_jitter_rms,
        (long long)snap.pps_jitter_last_ns,
        (long long)snap.pps_jitter_min_ns,
        (long long)snap.pps_jitter_max_ns,
        (unsigned long long)snap.pps_jitter_rms_ns,
        (long long)snap.pll_phase_error_ns,
        (long long)snap.ab_innovation_ns,
        (long long)snap.pps_period_error_ns,
        (unsigned long)snap.pps_dma_overruns,
        (unsigned long)snap.pps_jitter_samples,
        snap.accuracy_mode ? "true" : "false",
        (unsigned)snap.pll_filter_shift,
        snap.pps_present ? (snap.ab_servo_valid ? "TRACKING" : "ACQUIRE")
                         : (snap.pps_time_valid ? "HOLDOVER" : "ACQUIRE"),
        snap.ab_servo_valid ? "true" : "false",
        (unsigned long long)snap.ab_period_ns,
        (long long)snap.ab_period_rate_ns_per_sec,
        (unsigned long long)snap.ab_median_ns,
        (unsigned long long)snap.ab_mad_ns,
        (unsigned long long)snap.ab_outlier_threshold_ns,
        (unsigned long)snap.ab_rejected,
        (unsigned)snap.ab_alpha_shift,
        (unsigned)snap.ab_beta_shift,
        snap.pps_present ? "true" : "false",
        snap.pps_time_valid ? "true" : "false",
        (long long)pps_age_ms,
        snap.pll_ok ? "true" : "false",
        (unsigned)snap.pll_interval,
        (long long)snap.rtc_sync_age_ms,
        snap.rtc_aging_valid ? snap.rtc_aging : 0,
        snap.rtc_status ? "true" : "false",
        rtc_state_str(snap.rtc_state));

    if (len <= 0 || len >= (int)sizeof(payload)) {
        __atomic_add_fetch(&health_fmt_drop_count, 1, __ATOMIC_RELAXED);
        uint32_t fmt_drops = __atomic_load_n(&health_fmt_drop_count, __ATOMIC_RELAXED);
        uint32_t alloc_fails = __atomic_load_n(&health_alloc_fail_count, __ATOMIC_RELAXED);
        uint32_t send_errs = __atomic_load_n(&health_send_err_count, __ATOMIC_RELAXED);
        uint32_t rtc_busy = __atomic_load_n(&health_rtc_busy_count, __ATOMIC_RELAXED);
        len = snprintf(
            payload, sizeof(payload),
            "{"
            "\"host\":\"%s\","
            "\"unix\":%lu,"
            "\"synced\":%s,"
            "\"src\":\"%s\","
            "\"clock_state\":\"%s\","
            "\"pps\":%s,"
            "\"pps_time\":%s,"
            "\"pll\":%s,"
            "\"pll_us\":%u,"
            "\"sntp_quality\":\"%s\","
            "\"sntp_stratum\":%u,"
            "\"sntp_refid\":\"%s\","
            "\"gps_line_drops\":%lu,"
            "\"rmc_age_ms\":%lld,"
            "\"rtc_status\":%s,"
            "\"nonce\":%lu,"
            "\"health_compact\":true,"
            "\"health_fmt_drop_count\":%lu,"
            "\"health_alloc_fail_count\":%lu,"
            "\"health_send_err_count\":%lu,"
            "\"health_rtc_busy_count\":%lu",
            WIFI_HOSTNAME,
            (unsigned long)unix_sec,
            synced ? "true" : "false",
            snap.ts.using_rtc ? "RTC" : "GPS",
            clock_state_str(snap.clock_state),
            snap.pps_present ? "true" : "false",
            snap.pps_time_valid ? "true" : "false",
            snap.pll_ok ? "true" : "false",
            (unsigned)snap.pll_interval,
            sntp_quality_str(active_quality),
            (unsigned)sntp_quality_stratum(active_quality),
            sntp_refid,
            (unsigned long)snap.gps_line_drop_count,
            (long long)snap.rmc_age_ms,
            snap.rtc_status ? "true" : "false",
            (unsigned long)nonce,
            (unsigned long)fmt_drops,
            (unsigned long)alloc_fails,
            (unsigned long)send_errs,
            (unsigned long)rtc_busy);
        if (len <= 0 || len >= (int)sizeof(payload)) {
            return;
        }
    }

    char final_payload[2200];
    int final_len = 0;
#if HEALTH_BROADCAST_CRC
    uint32_t crc = crc32_ieee((const uint8_t *)payload, (size_t)len);
    final_len = snprintf(final_payload, sizeof(final_payload),
                         "%s,\"crc\":\"%08lX\"}", payload, (unsigned long)crc);
#else
    final_len = snprintf(final_payload, sizeof(final_payload), "%s}", payload);
#endif
    if (final_len <= 0 || final_len >= (int)sizeof(final_payload)) {
        __atomic_add_fetch(&health_fmt_drop_count, 1, __ATOMIC_RELAXED);
        return;
    }

    ip_addr_t bcast;
    if (netif_default) {
        ip4_addr_t ip = *netif_ip4_addr(netif_default);
        ip4_addr_t mask = *netif_ip4_netmask(netif_default);
        ip4_addr_t bc;
        ip4_addr_set_u32(&bc, ip4_addr_get_u32(&ip) | ~ip4_addr_get_u32(&mask));
        ip_addr_set_ip4_u32(&bcast, ip4_addr_get_u32(&bc));
    } else {
        IP4_ADDR(ip_2_ip4(&bcast), 255, 255, 255, 255);
    }
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, (u16_t)final_len, PBUF_RAM);
    if (!p) {
        __atomic_add_fetch(&health_alloc_fail_count, 1, __ATOMIC_RELAXED);
        return;
    }
    memcpy(p->payload, final_payload, (size_t)final_len);
    err_t send_err = udp_sendto(health_pcb, p, &bcast, HEALTH_BROADCAST_PORT);
    if (send_err != ERR_OK) {
        __atomic_add_fetch(&health_send_err_count, 1, __ATOMIC_RELAXED);
        DBG_PRINTF("HEALTH: udp_sendto failed err=%d\n", (int)send_err);
    }
    pbuf_free(p);
}
#endif

#if HTTP_SERVER_ENABLE
static const char http_dashboard[] =
    "<!doctype html><html><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<link rel=icon href=\"data:,\">"
    "<title>Pico GPS Clock</title><style>"
    ":root{color-scheme:dark;font:15px/1.5 system-ui,sans-serif;background:#0c141b;color:#e6eef4}*{box-sizing:border-box}body{margin:0}"
    "main{max-width:1120px;margin:auto;padding:32px 24px}h1{font-size:24px;letter-spacing:-.04em;margin:0}"
    "header{display:flex;align-items:center;justify-content:space-between;gap:16px;margin-bottom:24px}.eyebrow{font-size:11px;letter-spacing:.16em;text-transform:uppercase;color:#83b8bc;margin:0 0 6px}"
    ".sub,.note,dt,footer{color:#a3b3c0}.sub{margin:4px 0}.badge{font-size:13px;border:1px solid #344957;border-radius:24px;padding:6px 12px;white-space:nowrap}"
    ".grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:16px}.hero{grid-column:1/-1;background:linear-gradient(115deg,#152c36,#14212c);border-color:#345963}"
    "section{min-width:0;border:1px solid #2c3d49;border-radius:12px;padding:20px;background:#121f29}h2{font-size:13px;letter-spacing:.09em;text-transform:uppercase;color:#9fd6d5;margin:0 0 16px}"
    ".clock{font-size:clamp(24px,5vw,44px);font-weight:600;letter-spacing:-.04em;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}.hero dl{margin-top:18px;grid-template-columns:repeat(4,minmax(0,1fr))}.hero dt{font-size:12px}.hero dd{text-align:left;grid-row:2}"
    "dl{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1.15fr);gap:9px 12px;margin:0}dd{margin:0;text-align:right;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}dt{overflow-wrap:anywhere}"
    ".note{font-size:12px;margin:14px 0 0}.good{color:#86dfc0}.warn{color:#f7cd83}.bad{color:#ffa5a0}details{margin-top:24px}summary{cursor:pointer;padding:12px 0;color:#c8dae5;font-weight:600}summary:focus-visible{outline:2px solid #86dfc0;outline-offset:4px}"
    "footer{margin-top:24px;font-size:12px}body.stale .grid{opacity:.55}@media(max-width:850px){.grid{grid-template-columns:repeat(2,minmax(0,1fr))}}"
    "@media(max-width:560px){main{padding:20px 14px}.grid{grid-template-columns:1fr}header{align-items:flex-start;flex-direction:column}.hero dl{grid-template-columns:1fr 1fr}.hero dd{grid-row:auto;text-align:right}section{padding:18px}.clock{font-size:27px}}"
    "</style></head><body class=stale><main><header><div><p class=eyebrow>Offline time appliance</p><h1>Pico GPS Clock</h1><p class=sub id=status role=status aria-live=polite>Connecting...</p></div><span class=badge id=freshness>Waiting for device</span></header>"
    "<div class=grid><section class=hero><h2>UTC / served time</h2><div class=clock id=utc>-</div><dl>"
    "<dt>Source</dt><dd id=src>-</dd><dt>Clock state</dt><dd id=state>-</dd><dt>NTP service</dt><dd id=sntp>-</dd><dt>UTC uncertainty</dt><dd id=quality_total>-</dd></dl><p class=note>Device snapshot, not a browser precision clock. Network transit delay is separate from UTC uncertainty.</p></section>"
    "<section><h2>GPS reception</h2><dl><dt>Fix</dt><dd id=fix>-</dd><dt>Satellites used</dt><dd id=sats>-</dd><dt>PPS signal</dt><dd id=pps>-</dd>"
#if GPS_PROTOCOL_UBX
    "<dt>PDOP</dt>"
#else
    "<dt>HDOP</dt>"
#endif
    "<dd id=hdop>-</dd><dt>Position</dt><dd id=pos>-</dd></dl></section>"
    "<section><h2>Local time</h2><dl><dt>Local snapshot</dt><dd id=localtime>-</dd><dt>Zone</dt><dd id=tzname>-</dd><dt>UTC offset</dt><dd id=tzoffset>-</dd><dt>Daylight saving</dt><dd id=tzdst>-</dd></dl><p class=note>Presentation only. GPS, RTC and NTP stay in UTC.</p></section>"
    "<section><h2>RTC / holdover</h2><dl><dt>RTC state</dt><dd id=rtc>-</dd><dt>SQW reference</dt><dd id=sqw>-</dd><dt>SQW offset from GPS</dt><dd id=phase>-</dd><dt>32 kHz reference</dt><dd id=rtc_32k>-</dd><dt>Holdover age</dt><dd id=rtc_age>-</dd><dt>Holdover uncertainty</dt><dd id=rtc_error>-</dd></dl><p class=note id=phase_note>Waiting for a qualified phase mapping.</p></section>"
    "</div><details><summary>Timing diagnostics</summary><div class=grid>"
#if GPS_PROTOCOL_UBX
    "<section><h2>UBX Satellites</h2><dl><dt>Visible / used</dt><dd id=ubxsats>-</dd><dt>GPS</dt><dd id=ubxgps>-</dd><dt>Galileo</dt><dd id=ubxgal>-</dd><dt>BeiDou</dt><dd id=ubxbds>-</dd><dt>GLONASS</dt><dd id=ubxglo>-</dd><dt>QZSS</dt><dd id=ubxqzss>-</dd><dt>SBAS</dt><dd id=ubxsbas>-</dd></dl></section>"
#endif
    "<section><h2>PPS capture</h2><dl><dt>Accuracy mode</dt><dd id=accuracy>-</dd><dt>GNSS message age</dt><dd id=rmc>-</dd><dt>Association window</dt><dd id=window>-</dd><dt>Period error</dt><dd id=period_error>-</dd><dt>Jitter RMS</dt><dd id=jitter_ns>-</dd><dt>Jitter samples</dt><dd id=jitter_samples>-</dd></dl></section>"
    "<section><h2>Time Discipline</h2><dl><dt>Servo mode</dt><dd id=disc_mode>-</dd><dt>Servo state</dt><dd id=disc_valid>-</dd><dt>Period</dt><dd id=ab_period>-</dd><dt>Frequency estimate</dt><dd id=ab_rate>-</dd><dt>Servo innovation</dt><dd id=ab_innovation>-</dd><dt>Median interval</dt><dd id=ab_median>-</dd><dt>MAD</dt><dd id=ab_mad>-</dd><dt>Reject threshold</dt><dd id=ab_threshold>-</dd><dt>Rejected samples</dt><dd id=ab_rejected>-</dd><dt>Alpha / Beta</dt><dd id=ab_gains>-</dd></dl></section>"
    "<section><h2>RTC calibration</h2><dl><dt>Calibration</dt><dd id=rtc_calibration>-</dd><dt>Observation history</dt><dd id=rtc_learning>-</dd><dt>Frequency trim</dt><dd id=rtc_aging>-</dd><dt>Residual frequency</dt><dd id=rtc_frequency>-</dd><dt>Phase jitter</dt><dd id=rtc_jitter>-</dd><dt>Temperature</dt><dd id=rtc_temperature>-</dd><dt>Calendar writes / boot</dt><dd id=rtc_writes>-</dd><dt>Oscillator stopped at boot</dt><dd id=rtc_osf>-</dd></dl><p class=note>Trim changes rate; SQW offset is compensated in software. Qualification of 32 kHz is separate from aging calibration.</p></section>"
    "<section><h2>NTP uncertainty</h2><dl><dt>Source</dt><dd id=disp_source>-</dd><dt>Base / handoff</dt><dd id=disp_base>-</dd><dt>Interval residual</dt><dd id=disp_phase>-</dd><dt>Frequency / age</dt><dd id=disp_growth>-</dd><dt>Margin</dt><dd id=disp_margin>-</dd><dt>Output slew</dt><dd id=disp_slew>-</dd><dt>Clock total</dt><dd id=disp_total>-</dd><dt>Wire (rounded up)</dt><dd id=disp_wire>-</dd></dl></section>"
    "<section><h2>PPS timestamp placement</h2><dl><dt>Observation window</dt><dd id=pps_window>-</dd><dt>Timestamp offset</dt><dd id=pps_offset>-</dd></dl><p class=note>Window: time between observed low and high pin levels. Offset: reconstructed timestamp minus window midpoint; positive means later. Midpoint is an estimate, not the exact edge. A narrow window with a large offset suggests timestamp displacement; a wide window limits what we can conclude. Available for the active GPS/PPS source only.</p></section>"
    "</div></details><details><summary>Network, timezone &amp; system</summary><div class=grid>"
    "<section><h2>Network</h2><dl><dt>IP</dt><dd id=ip>-</dd><dt>Wi-Fi</dt><dd id=wifi>-</dd><dt>RSSI</dt><dd id=rssi>-</dd><dt>Reconnects / failed</dt><dd id=wifiretry>-</dd><dt>NTP requests</dt><dd id=ntpreq>-</dd><dt>NTP rate limited</dt><dd id=ntplimit>-</dd><dt>HTTP requests</dt><dd id=httpreq>-</dd><dt>HTTP errors</dt><dd id=httperr>-</dd></dl></section>"
    "<section><h2>Timezone data</h2><dl><dt>Mode</dt><dd id=tzmode>-</dd><dt>Source</dt><dd id=tzsource>-</dd><dt>Candidate</dt><dd id=tzcandidate>-</dd><dt>Rules</dt><dd id=tzrules>-</dd><dt>IANA / years</dt><dd id=tzdata>-</dd><dt>Boundaries</dt><dd id=tzboundary>-</dd></dl></section>"
    "<section><h2>System</h2><dl><dt>Firmware</dt><dd id=version>-</dd><dt>Build</dt><dd id=build>-</dd><dt>Uptime</dt><dd id=uptime>-</dd><dt>GPS checksum errors</dt><dd id=gpserr>-</dd><dt>Rejected fixes</dt><dd id=reject>-</dd></dl></section>"
    "</div></details><footer>Snapshots every 2 seconds while visible. Polling pauses in background tabs and backs off on errors. NTP: UDP port 123. No external assets or services.</footer></main><script>"
    "const $=id=>document.getElementById(id),v=(id,x)=>{const e=$(id);if(e)e.textContent=x==null?'-':x};let tickBusy=false,timer,lastSuccess=0,failures=0;"
    "async function tick(){if(tickBusy||document.hidden)return;tickBusy=true;const ac=new AbortController(),tm=setTimeout(()=>ac.abort(),5000);try{const r=await fetch('/api/health',{cache:'no-store',signal:ac.signal});if(!r.ok)throw Error('HTTP '+r.status);let d=await r.json();if(!Number.isFinite(d.unix)||!Number.isFinite(d.sntp_stratum))throw Error('Invalid health snapshot');if(document.hidden)return;lastSuccess=Date.now();failures=0;document.body.className='';"
    "const locked=d.sntp_stratum===1&&d.clock_uncertainty_valid,holdover=d.sntp_stratum===2&&d.clock_uncertainty_valid;$('status').className='sub '+(locked?'good':'warn');"
    "v('status',locked?'GPS disciplined / UTC synchronized':holdover?'RTC holdover / GPS precision unavailable':'Unqualified time / do not use as a precision reference');v('freshness','Live snapshot');"
    "v('utc',d.unix?new Date(d.unix*1000).toISOString().replace('T',' ').replace('.000Z',' UTC'):'Unavailable');v('src',d.src);v('state',d.clock_state);"
    "v('sntp',d.sntp_quality+' / stratum '+d.sntp_stratum);v('fix',d.fix?'Valid':'No fix');v('sats',d.sats);"
#if GPS_PROTOCOL_UBX
    "v('hdop',d.pdop>=0?d.pdop:'-');"
    "v('ubxsats',d.ubx_sat_total+'/'+d.ubx_sat_used);v('ubxgps',d.ubx_sat_gps+'/'+d.ubx_used_gps);v('ubxgal',d.ubx_sat_galileo+'/'+d.ubx_used_galileo);"
    "v('ubxbds',d.ubx_sat_beidou+'/'+d.ubx_used_beidou);v('ubxglo',d.ubx_sat_glonass+'/'+d.ubx_used_glonass);v('ubxqzss',d.ubx_sat_qzss+'/'+d.ubx_used_qzss);v('ubxsbas',d.ubx_sat_sbas+'/'+d.ubx_used_sbas);"
#else
    "v('hdop',d.hdop>=0?d.hdop:'-');"
#endif
    "v('pos',d.lat_ok&&d.lon_ok?d.lat.toFixed(5)+', '+d.lon.toFixed(5):'-');v('pps',d.pps?'Present':'Absent');"
    "v('tzmode',d.timezone_mode);v('tzname',d.timezone_name);v('localtime',d.local_time);v('tzsource',d.timezone_source);v('tzoffset',d.timezone_offset_text);v('tzdst',d.timezone_is_dst?'Active':'No');v('tzcandidate',d.timezone_candidate+' ('+d.timezone_candidate_count+')');v('tzrules',d.timezone_rules_out_of_range?'Out of range':'OK');v('tzdata',d.timezone_data_version+' '+d.timezone_data_start_year+'-'+d.timezone_data_end_year);"
    "v('tzboundary',d.timezone_boundary_version);v('accuracy',d.accuracy_mode?'Active':'Inactive');v('rmc',d.rmc_age_ms>=0?d.rmc_age_ms+' ms':'-');v('window',d.rmc_pps_window_us+' us');v('period_error',(d.pps_period_error_ns>=0?'+':'')+d.pps_period_error_ns+' ns');v('jitter_ns',d.pps_jitter_rms_ns+' ns');v('jitter_samples',d.pps_jitter_samples);"
    "v('disc_mode',d.discipline_mode);v('disc_valid',d.ab_servo_valid?'Valid':'Not initialized');v('ab_period',d.ab_period_ns+' ns');v('ab_rate',d.ab_rate_ns_per_sec+' ns/s');v('ab_innovation',d.ab_innovation_ns+' ns');v('ab_median',d.ab_median_ns+' ns');v('ab_mad',d.ab_mad_ns+' ns');v('ab_threshold',d.ab_outlier_threshold_ns+' ns');v('ab_rejected',d.ab_rejected);v('ab_gains',(1<<d.ab_alpha_shift)+' / '+(1<<d.ab_beta_shift));"
    "v('rtc',d.rtc_state+(d.rtc_status?'':' / fault'));v('sqw',d.rtc_sqw_time?'Qualified':d.rtc_sqw?'Learning':'Absent');"
    "v('phase',d.rtc_sqw_time?(d.rtc_phase_offset_ns/1e6).toFixed(3)+' ms':'Unqualified');v('phase_note',d.rtc_sqw_time?'SQW offset is compensated in software; it is not the NTP time error.':'Waiting for a qualified phase mapping.');v('ip',d.ip);v('wifi',d.wifi_link?'Connected':'Offline');"
    "v('rtc_jitter',(d.rtc_phase_jitter_ns/1000).toFixed(1)+' us');v('rtc_frequency',d.rtc_frequency_valid?d.rtc_frequency_error_ppm.toFixed(3)+' ppm':'Learning');v('rtc_aging',d.rtc_aging_valid?d.rtc_aging+' / '+(d.rtc_aging_enabled?'automatic':'fixed'):'Unread');v('rtc_calibration',d.rtc_calibration_status);v('rtc_learning',(d.rtc_calibration_observed_s/3600).toFixed(1)+' / '+(d.rtc_calibration_required_s/3600).toFixed(1)+' h');v('rtc_writes',d.rtc_calendar_write_count);v('rtc_temperature',d.rtc_temperature_valid?d.rtc_temperature_c.toFixed(2)+' C':'Unavailable');v('rtc_age',d.rtc_holdover_active?d.rtc_holdover_age_s+' s':'Not active');v('rtc_error',d.rtc_holdover_active?(d.rtc_estimated_error_ns/1e6).toFixed(2)+' ms':'Not in holdover');v('rtc_32k',!d.rtc_32k_enabled?'Disabled':d.rtc_32k_valid?'Qualified / '+d.rtc_32k_frequency_error_ppm.toFixed(3)+' ppm':'Learning / '+d.rtc_32k_sample_count+' accepted');v('rtc_osf',d.rtc_osf_boot_read?(d.rtc_osf_at_boot?'Yes':'No'):'Unread');"
    "if(d.rtc_32k_enabled&&!d.rtc_32k_valid&&(d.rtc_32k_age_ms<0||d.rtc_32k_age_ms>2500))v('rtc_32k','No signal');"
    "v('rssi',d.wifi_rssi+' dBm');v('uptime',Math.floor(d.uptime_s/86400)+'d '+Math.floor(d.uptime_s%86400/3600)+'h '+Math.floor(d.uptime_s%3600/60)+'m');v('version',d.version);v('build',d.build);"
    "v('gpserr',d.gps_checksum_errors);v('reject',d.gps_rejected_fixes);v('ntpreq',d.sntp_requests);"
    "v('ntplimit',d.sntp_rate_limited);v('httpreq',d.http_requests);v('httperr',d.http_errors);"
    "v('wifiretry',d.wifi_reconnects+' / '+d.wifi_reconnect_failures);"
    "const us=x=>Number.isFinite(x)?x.toFixed(1)+' us':'-';v('disp_source',d.dispersion_source);v('disp_base',us(d.dispersion_base_us));v('disp_phase',us(d.dispersion_phase_us));v('disp_growth',us(d.dispersion_frequency_us));v('disp_margin',us(d.dispersion_margin_us));v('disp_slew',us(d.dispersion_slew_us));v('disp_total',d.clock_uncertainty_valid?us(d.clock_uncertainty_us):'Unqualified');v('quality_total',d.clock_uncertainty_valid?us(d.clock_uncertainty_us):'Unqualified');v('disp_wire',us(d.ntp_root_dispersion_us));"
    "v('pps_window',d.pps_placement_valid?us(d.pps_observation_window_us):'Unavailable');v('pps_offset',d.pps_placement_valid?(d.pps_timestamp_offset_us>=0?'+':'')+us(d.pps_timestamp_offset_us):'Unavailable');"
    "}catch(e){failures++;document.body.className='stale';v('status','Device unavailable / displayed values are stale');v('freshness',lastSuccess?'Last update '+Math.floor((Date.now()-lastSuccess)/1000)+' s ago':'No valid snapshot');$('status').className='sub bad'}finally{clearTimeout(tm);tickBusy=false;if(!document.hidden)timer=setTimeout(tick,Math.min(30000,2000*2**Math.min(failures,4)))}}"
    "document.addEventListener('visibilitychange',()=>{clearTimeout(timer);if(document.hidden){document.body.className='stale';v('freshness','Paused / background tab')}else tick()});tick();</script></body></html>";

static int rtc_append_health_json(char *out, size_t out_len, int prefix_len, sntp_quality_t active_quality) {
    if (prefix_len <= 0 || (size_t)prefix_len >= out_len) return -1;
    struct {
        int64_t phase_ns, raw_phase_ns, slew_ns, pps_interval_ns;
        const char *reject_reason;
        uint64_t jitter_ns, age_ns, error_ns, sqw_age_ns;
        uint32_t samples, rejects, changes, period_ns, recovery;
        sntp_quality_t quality;
        uint64_t pps_age_ns;
        double fast_ppm, uncertainty_ppm;
        bool osf_boot, osf_read, calendar, temperature, conversion_failed, frequency_valid;
        bool aging_valid, holdover_active;
        const char *calibration_status;
        uint64_t calibration_observed_s;
        uint32_t calendar_writes;
        int16_t temperature_quarters;
        int aging;
        rtc_32k_t k;
    } d;
    uint64_t now_ns = hires_time_ns();
    mutex_enter_blocking(&stats_mutex);
    d.phase_ns = rtc_phase_filter.offset_ns;
    d.raw_phase_ns = rtc_phase_error_ns;
    d.pps_interval_ns = 1000000000LL + pps_period_error_last_ns;
    d.reject_reason = rtc_last_reject_reason;
    d.jitter_ns = rtc_phase_filter.jitter_ns;
    d.samples = rtc_phase_filter.sample_count;
    d.rejects = rtc_phase_reject_total;
    d.age_ns = rtc_holdover.active && now_ns >= rtc_holdover.start_ns ? now_ns - rtc_holdover.start_ns : 0;
    d.error_ns = rtc_holdover_error_ns(&rtc_holdover, now_ns);
    d.slew_ns = utc_continuity.residual_ns;
    d.fast_ppm = rtc_holdover.active ? rtc_holdover_fast_ppm : rtc_frequency_filter.frequency_error_ppm;
    d.uncertainty_ppm = rtc_holdover.active ? rtc_holdover.frequency_uncertainty_ppm : rtc_frequency_filter.uncertainty_ppm;
    d.frequency_valid = rtc_holdover.active ? rtc_holdover_frequency_valid : rtc_frequency_filter.valid;
    d.osf_boot = rtc_osf_at_boot; d.osf_read = rtc_osf_boot_read;
    d.calendar = rtc_calendar_valid; d.temperature = rtc_temperature_valid;
    d.temperature_quarters = rtc_temperature_quarters;
    d.conversion_failed = rtc_conversion_failed;
    d.aging = rtc_aging_value;
    d.aging_valid = rtc_aging_valid;
    d.holdover_active = rtc_holdover.active;
    d.calendar_writes = rtc_calendar_write_count;
    d.calibration_observed_s = rtc_frequency_filter.duration_ns / 1000000000ULL;
    d.calibration_status = !RTC_AGING_ENABLE ? "Disabled" :
        rtc_conversion_failed ? "Paused / conversion fault" :
        rtc_conversion_pending ? "Applying trim" :
        !gnss_recovery.locked || !pps_time_valid || !pll_valid ||
        now_ns < rtc_last_gnss_observation_ns || now_ns - rtc_last_gnss_observation_ns >= 1500000000ULL ? "Paused / no GPS reference" :
        !rtc_phase_filter.qualified || !rtc_frequency_filter.valid ||
        d.calibration_observed_s < RTC_AGING_WINDOW_SEC ||
        rtc_frequency_filter.count < RTC_AGING_MIN_SAMPLES ? "Learning" :
        rtc_frequency_filter.uncertainty_ppm > RTC_AGING_MAX_UNCERTAINTY_PPM ? "Learning / uncertainty" :
        absolute_time_diff_us(last_aging_adjust_time, get_absolute_time()) < (int64_t)RTC_AGING_WINDOW_SEC * 1000000LL ? "Waiting / trim interval" :
        "Qualified";
    d.k = rtc_32k;
    d.changes = timing_source_changes;
    d.period_ns = rtc_sqw_interval_ns;
    d.recovery = gnss_recovery.good_pulses;
    d.quality = active_quality;
    d.pps_age_ns = now_ns >= trusted_pps_ns ? now_ns - trusted_pps_ns : UINT64_MAX;
    d.sqw_age_ns = last_rtc_sqw_second_time_ns && now_ns >= last_rtc_sqw_second_time_ns ? now_ns - last_rtc_sqw_second_time_ns : UINT64_MAX;
    mutex_exit(&stats_mutex);
    /* Extend the existing object; no high-rate logs or additional service. */
    size_t offset = (size_t)prefix_len - 1;
    const char *timing_state = d.quality == SNTP_QUALITY_PPS ?
        (d.pps_age_ns > 1100000000ULL ? "GNSS_PPS_COAST" : "GNSS_PPS_LOCKED") :
        d.quality == SNTP_QUALITY_RTC_SQW ? "RTC_HOLDOVER" :
        d.quality == SNTP_QUALITY_COARSE_RTC ? "RTC_COARSE" :
        d.quality == SNTP_QUALITY_COARSE_GPS ? "GPS_COARSE" : "UNSYNC";
    int added = snprintf(out + offset, out_len - offset,
        ",\"timing_state\":\"%s\",\"sntp_refid\":\"%.4s\",\"rtc_phase_offset_ns\":%lld,\"rtc_phase_jitter_ns\":%llu,\"rtc_phase_sample_count\":%lu,"
        "\"rtc_phase_reject_count\":%lu,\"rtc_holdover_age_s\":%llu,\"rtc_estimated_error_ns\":%llu,"
        "\"utc_slew_remaining_ns\":%lld,\"rtc_frequency_error_ppm\":%.4f,\"rtc_frequency_uncertainty_ppm\":%.4f,"
        "\"rtc_frequency_valid\":%s,\"rtc_frequency_source\":\"SQW_PHASE\",\"rtc_aging\":%d,\"rtc_aging_ppm_per_lsb\":%.3f,"
        "\"rtc_osf_at_boot\":%s,\"rtc_osf_boot_read\":%s,\"rtc_calendar_valid\":%s,"
        "\"rtc_temperature_valid\":%s,\"rtc_temperature_c\":%.2f,\"rtc_conversion_failed\":%s,"
        "\"rtc_32k_enabled\":%s,\"rtc_32k_valid\":%s,\"rtc_32k_frequency_error_ppm\":%.3f,"
        "\"rtc_32k_frequency_uncertainty_ppm\":%.3f,\"rtc_32k_sample_count\":%lu,\"rtc_32k_reject_count\":%lu,"
        "\"rtc_32k_reject_reason\":%lu,\"rtc_32k_pico_period_ns\":%.0f,\"rtc_sqw_interval_ns\":%lu,"
        "\"rtc_sqw_age_ms\":%lld,\"timing_source_changes\":%lu,\"gnss_recovery_good_pulses\":%lu,"
        "\"rtc_phase_error_ns\":%lld,\"pps_interval_ns\":%lld,\"rtc_reject_reason\":\"%s\",\"rtc_pico_frequency_ratio\":%.9f,"
        "\"rtc_aging_valid\":%s,\"rtc_aging_enabled\":%s,\"rtc_holdover_active\":%s,"
        "\"rtc_calendar_write_count\":%lu,\"rtc_calibration_status\":\"%s\","
        "\"rtc_calibration_observed_s\":%llu,\"rtc_calibration_required_s\":%lu,\"rtc_32k_age_ms\":%lld}",
        timing_state, timing_refid(d.quality), (long long)d.phase_ns, (unsigned long long)d.jitter_ns, (unsigned long)d.samples,
        (unsigned long)d.rejects, (unsigned long long)(d.age_ns / 1000000000ULL), (unsigned long long)d.error_ns,
        (long long)d.slew_ns, d.fast_ppm, d.uncertainty_ppm, d.frequency_valid ? "true" : "false",
        d.aging, (double)RTC_AGING_PPM_PER_LSB, d.osf_boot ? "true" : "false", d.osf_read ? "true" : "false",
        d.calendar ? "true" : "false", d.temperature ? "true" : "false", d.temperature_quarters / 4.0,
        d.conversion_failed ? "true" : "false", RTC_32KHZ_GPIO >= 0 ? "true" : "false", d.k.valid ? "true" : "false",
        d.k.frequency_error_ppm, d.k.uncertainty_ppm, (unsigned long)d.k.samples, (unsigned long)d.k.rejects,
        (unsigned long)d.k.last_reject, d.k.pico_period_ns, (unsigned long)d.period_ns,
        d.sqw_age_ns == UINT64_MAX ? -1LL : (long long)(d.sqw_age_ns / 1000000ULL),
        (unsigned long)d.changes, (unsigned long)d.recovery, (long long)d.raw_phase_ns,
        (long long)d.pps_interval_ns, d.reject_reason, d.k.pico_period_ns / 1e9,
        d.aging_valid ? "true" : "false", RTC_AGING_ENABLE ? "true" : "false",
        d.holdover_active ? "true" : "false", (unsigned long)d.calendar_writes,
        d.calibration_status, (unsigned long long)d.calibration_observed_s,
        (unsigned long)RTC_AGING_WINDOW_SEC,
        d.k.last_ns && now_ns >= d.k.last_ns ? (long long)((now_ns - d.k.last_ns) / 1000000ULL) : -1LL);
    if (added < 0 || (size_t)added >= out_len - offset) return -1;
    return (int)offset + added;
}

static int http_build_health_json(char *out, size_t out_len) {
    health_snapshot_t snap;
    get_health_snapshot(&snap);
    uint32_t ntp_sec = 0;
    uint32_t ntp_frac = 0;
    bool synced = false;
    sntp_quality_t quality = SNTP_QUALITY_NONE;
    ntp_uncertainty_t u;
    get_sntp_time_details(hires_time_ns(), &ntp_sec, &ntp_frac, &synced, &quality, &u, NULL);
    uint32_t unix_sec = ntp_sec >= NTP_DELTA ? ntp_sec - NTP_DELTA : 0;
    char local[24];
    format_local_time(snap.local_unix_sec, local, sizeof(local));
    uint64_t now_us = to_us_since_boot(get_absolute_time());
    int64_t pps_age_ms = snap.last_pps_us >= 0
        ? (int64_t)((now_us - (uint64_t)snap.last_pps_us) / 1000U) : -1;
    int prefix_len = snprintf(out, out_len,
        "{\"version\":\"%s\",\"build\":\"%s %s\",\"unix\":%lu,\"synced\":%s,\"src\":\"%s\",\"clock_state\":\"%s\","
#if GPS_PROTOCOL_UBX
        "\"fix\":%s,\"fixq\":%d,\"sats\":%d,\"ubx_sat_total\":%u,\"ubx_sat_used\":%u,"
        "\"ubx_sat_gps\":%u,\"ubx_used_gps\":%u,\"ubx_sat_galileo\":%u,\"ubx_used_galileo\":%u,"
        "\"ubx_sat_beidou\":%u,\"ubx_used_beidou\":%u,\"ubx_sat_glonass\":%u,\"ubx_used_glonass\":%u,"
        "\"ubx_sat_qzss\":%u,\"ubx_used_qzss\":%u,\"ubx_sat_sbas\":%u,\"ubx_used_sbas\":%u,"
#else
        "\"fix\":%s,\"fixq\":%d,\"sats\":%d,"
#endif
        "\"hdop\":%.2f,\"pdop\":%.2f,\"vdop\":%.2f,"
        "\"lat\":%.6f,\"lon\":%.6f,\"lat_ok\":%s,\"lon_ok\":%s,"
        "\"timezone_mode\":\"%s\",\"timezone_name\":\"%s\",\"timezone_source\":\"%s\","
        "\"timezone_offset_seconds\":%ld,\"timezone_offset_text\":\"%s\",\"timezone_is_dst\":%s,\"timezone_lookup_valid\":%s,\"timezone_rules_out_of_range\":%s,"
        "\"timezone_last_lookup_age_s\":%lld,\"timezone_candidate\":\"%s\",\"timezone_candidate_count\":%ld,"
        "\"timezone_data_version\":\"%s\",\"timezone_boundary_version\":\"%s\","
        "\"timezone_data_start_year\":%d,\"timezone_data_end_year\":%d,\"local_time\":\"%s\","
        "\"sntp_quality\":\"%s\",\"sntp_stratum\":%u,\"pps\":%s,\"pps_age_ms\":%lld,\"rmc_age_ms\":%lld,"
        "\"pps_jitter_us\":%ld,\"pps_jitter_min_us\":%ld,\"pps_jitter_max_us\":%ld,"
        "\"pps_jitter_rms_us\":%lu,"
        "\"pps_jitter_ns\":%lld,\"pps_jitter_min_ns\":%lld,\"pps_jitter_max_ns\":%lld,"
        "\"pps_jitter_rms_ns\":%llu,\"pll_phase_error_ns\":%lld,\"ab_innovation_ns\":%lld,\"pps_period_error_ns\":%lld,"
        "\"pps_dma_overruns\":%lu,\"pps_jitter_samples\":%lu,\"accuracy_mode\":%s,"
        "\"pll_filter_shift\":%u,\"rmc_pps_offset_us\":%ld,\"rmc_pps_window_us\":%ld,"
        "\"discipline_mode\":\"%s\",\"ab_servo_valid\":%s,\"ab_period_ns\":%llu,"
        "\"ab_rate_ns_per_sec\":%lld,\"ab_median_ns\":%llu,\"ab_mad_ns\":%llu,"
        "\"ab_outlier_threshold_ns\":%llu,\"ab_rejected\":%lu,\"ab_alpha_shift\":%u,\"ab_beta_shift\":%u,"
        "\"rtc_state\":\"%s\",\"rtc_trust\":\"%s\",\"rtc_status\":%s,\"rtc_sqw\":%s,\"rtc_sqw_time\":%s,"
        "\"rtc_sqw_phase_us\":%ld,\"rtc_sqw_rephase_count\":%lu,\"wifi_link\":%s,\"wifi_rssi\":%d,\"ip\":\"%s\",\"uptime_s\":%lu,"
        "\"gps_checksum_errors\":%lu,\"gps_rejected_fixes\":%lu,"
        "\"sntp_requests\":%lu,\"sntp_malformed\":%lu,\"sntp_rate_limited\":%lu,"
        "\"sntp_send_errors\":%lu,\"http_requests\":%lu,\"http_errors\":%lu,"
        "\"wifi_reconnects\":%lu,\"wifi_reconnect_failures\":%lu,"
        "\"tm1637_errors\":%lu,\"rtc_drift_us_per_day_x10\":%ld}",
        FIRMWARE_VERSION, __DATE__, __TIME__, (unsigned long)unix_sec, synced ? "true" : "false",
        snap.ts.using_rtc ? "RTC" : "GPS", clock_state_str(snap.clock_state),
        snap.stats.has_fix ? "true" : "false", snap.stats.fix_quality, snap.stats.satellites,
#if GPS_PROTOCOL_UBX
        (unsigned)snap.stats.ubx_sat_total, (unsigned)snap.stats.ubx_sat_used,
        (unsigned)snap.stats.ubx_sat_gps, (unsigned)snap.stats.ubx_used_gps,
        (unsigned)snap.stats.ubx_sat_galileo, (unsigned)snap.stats.ubx_used_galileo,
        (unsigned)snap.stats.ubx_sat_beidou, (unsigned)snap.stats.ubx_used_beidou,
        (unsigned)snap.stats.ubx_sat_glonass, (unsigned)snap.stats.ubx_used_glonass,
        (unsigned)snap.stats.ubx_sat_qzss, (unsigned)snap.stats.ubx_used_qzss,
        (unsigned)snap.stats.ubx_sat_sbas, (unsigned)snap.stats.ubx_used_sbas,
#endif
        (double)snap.stats.hdop, (double)snap.stats.pdop, (double)snap.stats.vdop,
        (double)snap.stats.lat_deg, (double)snap.stats.lon_deg,
        snap.stats.lat_valid ? "true" : "false", snap.stats.lon_valid ? "true" : "false",
        snap.timezone_mode, snap.timezone_name, snap.timezone_source,
        (long)snap.timezone_offset_seconds, snap.timezone_offset_text, snap.timezone_is_dst ? "true" : "false",
        snap.timezone_lookup_valid ? "true" : "false", snap.timezone_rules_out_of_range ? "true" : "false",
        (long long)snap.timezone_last_lookup_age_s,
        snap.timezone_candidate, (long)snap.timezone_candidate_count,
        TZDB_VERSION, TZ_BOUNDARY_VERSION, TZ_DATA_START_YEAR, TZ_DATA_END_YEAR, local,
        sntp_quality_str(quality), (unsigned)sntp_quality_stratum(quality),
        snap.pps_present ? "true" : "false", (long long)pps_age_ms, (long long)snap.rmc_age_ms,
        (long)snap.pps_jitter_last, (long)snap.pps_jitter_min, (long)snap.pps_jitter_max,
        (unsigned long)snap.pps_jitter_rms,
        (long long)snap.pps_jitter_last_ns, (long long)snap.pps_jitter_min_ns,
        (long long)snap.pps_jitter_max_ns, (unsigned long long)snap.pps_jitter_rms_ns,
        (long long)snap.pll_phase_error_ns, (long long)snap.ab_innovation_ns,
        (long long)snap.pps_period_error_ns, (unsigned long)snap.pps_dma_overruns,
        (unsigned long)snap.pps_jitter_samples,
        snap.accuracy_mode ? "true" : "false", (unsigned)snap.pll_filter_shift,
        (long)snap.rmc_pps_offset_us, (long)snap.rmc_pps_window_us,
        snap.pps_present ? (snap.ab_servo_valid ? "TRACKING" : "ACQUIRE")
                         : (snap.pps_time_valid ? "HOLDOVER" : "ACQUIRE"),
        snap.ab_servo_valid ? "true" : "false",
        (unsigned long long)snap.ab_period_ns,
        (long long)snap.ab_period_rate_ns_per_sec,
        (unsigned long long)snap.ab_median_ns,
        (unsigned long long)snap.ab_mad_ns,
        (unsigned long long)snap.ab_outlier_threshold_ns,
        (unsigned long)snap.ab_rejected,
        (unsigned)snap.ab_alpha_shift,
        (unsigned)snap.ab_beta_shift, rtc_state_str(snap.rtc_state),
        rtc_trust_str(snap.rtc_state, snap.rtc_status,
                      snap.rtc_sqw_time_valid && quality == SNTP_QUALITY_RTC_SQW),
        snap.rtc_status ? "true" : "false", snap.rtc_sqw_present ? "true" : "false",
        snap.rtc_sqw_time_valid ? "true" : "false",
        (long)snap.rtc_sqw_phase_us, (unsigned long)snap.rtc_sqw_rephase_count, snap.wifi_link ? "true" : "false",
        (int)snap.wifi_rssi, snap.ip, (unsigned long)snap.uptime_s,
        (unsigned long)snap.gps_checksum_error_count,
        (unsigned long)snap.gps_rejected_fix_count,
        (unsigned long)snap.sntp_request_count,
        (unsigned long)snap.sntp_malformed_count,
        (unsigned long)snap.sntp_rate_limit_count,
        (unsigned long)snap.sntp_send_error_count,
        (unsigned long)snap.http_request_count,
        (unsigned long)snap.http_error_count,
        (unsigned long)snap.wifi_reconnect_count,
        (unsigned long)snap.wifi_reconnect_fail_count,
        (unsigned long)snap.tm1637_errors,
        (long)snap.rtc_drift_us_per_day_x10);
    int length = rtc_append_health_json(out, out_len, prefix_len, quality);
    if (length <= 0 || (size_t)length >= out_len) return -1;
    uint32_t raw = ntp_dispersion_from_ns(u.total_ns);
    size_t offset = (size_t)length - 1;
    int added = snprintf(out + offset, out_len - offset,
        ",\"clock_uncertainty_valid\":%s,\"ntp_root_dispersion_us\":%.3f,\"ntp_root_dispersion_ntp16_16\":%lu,"
        "\"ntp_root_delay_us\":0,\"clock_uncertainty_us\":%.3f,"
        "\"dispersion_source\":\"%.4s\",\"dispersion_age_ms\":%.3f,"
        "\"dispersion_base_us\":%.3f,\"dispersion_phase_us\":%.3f,"
        "\"dispersion_frequency_us\":%.3f,\"dispersion_margin_us\":%.3f,"
        "\"dispersion_slew_us\":%.3f,\"dispersion_frequency_uncertainty_ppm\":%.6f,"
        "\"pps_placement_valid\":%s,\"pps_observation_window_us\":%.3f,\"pps_timestamp_offset_us\":%.3f,\"pps_capture_faults\":%lu}",
        u.total_ns != UINT64_MAX && (quality == SNTP_QUALITY_PPS || quality == SNTP_QUALITY_RTC_SQW) ? "true" : "false",
        ntp_dispersion_to_ns(raw) / 1000.0, (unsigned long)raw, u.total_ns / 1000.0,
        timing_refid(quality), u.age_ns / 1e6, u.capture_ns / 1000.0, u.phase_ns / 1000.0,
        u.growth_ns / 1000.0, u.margin_ns / 1000.0, u.slew_ns / 1000.0,
        u.frequency_uncertainty_ppm, u.placement_valid ? "true" : "false",
        u.placement_window_ns / 1000.0, u.placement_offset_ns / 1000.0,
        (unsigned long)__atomic_load_n(&pps_capture_faults, __ATOMIC_RELAXED));
    if (added < 0 || (size_t)added >= out_len - offset) return -1;
    return (int)offset + added;
}

typedef struct {
    char request[160];
    uint16_t len;
    uint16_t request_bytes;
    uint32_t header_tail;
    char header[192];
    size_t header_len;
    const char *response;
    char *owned_body;
    size_t response_len;
    size_t response_offset;
    size_t acknowledged;
    uint8_t idle_polls;
} http_conn_state_t;

/* At most two snapshots (12.8 KiB), with no full-page heap copies. The
 * dashboard is immutable XIP data. TCP references remain valid until ACK. */
#define HTTP_MAX_CONNECTIONS 2u
#define HTTP_HEALTH_CAPACITY 6400u
static unsigned http_active_connections;
static err_t http_recv(void *, struct tcp_pcb *, struct pbuf *, err_t);
static err_t http_sent(void *, struct tcp_pcb *, u16_t);
static err_t http_poll(void *, struct tcp_pcb *);
static void http_err(void *, err_t);

static void http_free_state(http_conn_state_t *state) {
    if (!state) return;
    if (state->owned_body) mem_free(state->owned_body);
    mem_free(state);
    --http_active_connections;
}

static err_t http_close_connection(struct tcp_pcb *tpcb, http_conn_state_t *state) {
    /* tcp_close may free the PCB immediately. Detach before closing, and
     * restore callbacks only on failure, when lwIP guarantees it is live. */
    tcp_arg(tpcb, NULL);
    tcp_recv(tpcb, NULL);
    tcp_err(tpcb, NULL);
    tcp_sent(tpcb, NULL);
    tcp_poll(tpcb, NULL, 0);
    err_t err = tcp_close(tpcb);
    if (err != ERR_OK) {
        tcp_arg(tpcb, state);
        tcp_recv(tpcb, http_recv);
        tcp_err(tpcb, http_err);
        tcp_sent(tpcb, http_sent);
        tcp_poll(tpcb, http_poll, 2);
        return err;
    }
    http_free_state(state);
    return ERR_OK;
}

static void http_err(void *arg, err_t err) {
    (void)err;
    http_free_state((http_conn_state_t *)arg);
}

static err_t http_send_pending(struct tcp_pcb *tpcb, http_conn_state_t *state) {
    if (!state || !state->response || state->response_offset >= state->response_len) {
        return ERR_OK;
    }

    bool header = state->response_offset < state->header_len;
    size_t remaining = header ? state->header_len - state->response_offset :
        state->response_len - state->response_offset;
    const char *data = header ? state->header + state->response_offset :
        state->response + state->response_offset - state->header_len;
    u16_t available = tcp_sndbuf(tpcb);
    if (available == 0) {
        return ERR_MEM;
    }
    u16_t chunk = (u16_t)(remaining > TCP_MSS ? TCP_MSS : remaining);
    if (chunk > available) chunk = available;
    if (chunk == 0) return ERR_MEM;
    /* Header and body storage outlive every queued TCP reference. */
    err_t err = tcp_write(tpcb, data, chunk, 0);
    if (err != ERR_OK) {
        return err;
    }
    state->response_offset += chunk;
    return tcp_output(tpcb);
}

static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    http_conn_state_t *state = (http_conn_state_t *)arg;
    if (!state) return ERR_OK;
    state->acknowledged += len;
    state->idle_polls = 0;
    if (state && state->response_offset < state->response_len) {
        err_t err = http_send_pending(tpcb, state);
        if (err != ERR_OK && err != ERR_MEM) {
            __atomic_add_fetch(&http_error_count, 1, __ATOMIC_RELAXED);
            tcp_abort(tpcb);
            return ERR_ABRT;
        }
        return ERR_OK;
    }
    // tcp_sent() may run after an earlier segment is acknowledged while
    // later response data is still queued or unacknowledged.  Closing here
    // unconditionally can truncate large HTML/JSON responses.
    if (state->acknowledged < state->response_len) {
        return ERR_OK;
    }
    err_t err = http_close_connection(tpcb, state);
    return err == ERR_MEM ? ERR_OK : err;
}

static err_t http_poll(void *arg, struct tcp_pcb *tpcb) {
    http_conn_state_t *state = (http_conn_state_t *)arg;
    if (!state) return ERR_OK;
    if (++state->idle_polls >= 15) {
        __atomic_add_fetch(&http_error_count, 1, __ATOMIC_RELAXED);
        tcp_abort(tpcb);
        return ERR_ABRT;
    }
    if (state->response && state->acknowledged == state->response_len) {
        err_t err = http_close_connection(tpcb, state);
        return err == ERR_MEM ? ERR_OK : err;
    }
    err_t err = http_send_pending(tpcb, state);
    if (err == ERR_OK) err = tcp_output(tpcb);
    if (err != ERR_OK && err != ERR_MEM) {
        __atomic_add_fetch(&http_error_count, 1, __ATOMIC_RELAXED);
        tcp_abort(tpcb);
        return ERR_ABRT;
    }
    return ERR_OK;
}

static err_t http_send_response(struct tcp_pcb *tpcb, const char *type,
                                int status, const char *body, size_t body_len) {
    http_conn_state_t *state = (http_conn_state_t *)tpcb->callback_arg;
    if (!state || state->response) return ERR_OK;
    const char *reason = status == 200 ? "OK" : (status == 404 ? "Not Found" : "Bad Request");
    int header_len = snprintf(state->header, sizeof(state->header),
                              "HTTP/1.1 %d %s\r\nContent-Type: %s\r\n"
                              "Content-Length: %lu\r\nCache-Control: no-store\r\n"
                              "Connection: close\r\n\r\n",
                              status, reason, type, (unsigned long)body_len);
    if (header_len <= 0 || header_len >= (int)sizeof(state->header)) {
        __atomic_add_fetch(&http_error_count, 1, __ATOMIC_RELAXED);
        tcp_abort(tpcb);
        return ERR_ABRT;
    }
    size_t response_len = (size_t)header_len + body_len;
    if (!state || response_len == 0) {
        __atomic_add_fetch(&http_error_count, 1, __ATOMIC_RELAXED);
        tcp_abort(tpcb);
        return ERR_ABRT;
    }
    state->response = body;
    state->header_len = (size_t)header_len;
    state->response_len = response_len;
    state->response_offset = 0;
    tcp_sent(tpcb, http_sent);
    err_t send_status = http_send_pending(tpcb, state);
    if (send_status != ERR_OK && send_status != ERR_MEM) {
        __atomic_add_fetch(&http_error_count, 1, __ATOMIC_RELAXED);
        tcp_abort(tpcb);
        return ERR_ABRT;
    }
    return ERR_OK;
}

static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    http_conn_state_t *state = (http_conn_state_t *)arg;
    if (err != ERR_OK) {
        __atomic_add_fetch(&http_error_count, 1, __ATOMIC_RELAXED);
        if (p) pbuf_free(p);
        tcp_abort(tpcb);
        return ERR_ABRT;
    }
    if (!p) {
        /* A client may half-close its request while still reading our reply. */
        if (!state || !state->response) {
            err_t close_err = http_close_connection(tpcb, state);
            if (close_err != ERR_OK) { tcp_abort(tpcb); return ERR_ABRT; }
        }
        return ERR_OK;
    }
    if (!state) {
        pbuf_free(p);
        tcp_abort(tpcb);
        return ERR_ABRT;
    }
    if (state->response) {
        /* Consume trailing headers/pipelined input, never start a second
         * response on this Connection: close socket. */
        tcp_recved(tpcb, p->tot_len);
        pbuf_free(p);
        return ERR_OK;
    }
    /* Keep only the request line, but wait for the complete header block.
     * A browser's headers can span several pbufs and exceed 160 bytes. */
    bool headers_done = false;
    for (u16_t i = 0; i < p->tot_len && state->request_bytes < 4096; ++i) {
        state->header_tail = (state->header_tail << 8) | pbuf_get_at(p, i);
        ++state->request_bytes;
        if (state->header_tail == 0x0d0a0d0au) { headers_done = true; break; }
    }
    size_t room = sizeof(state->request) - 1u - state->len;
    size_t copy_len = p->tot_len < room ? p->tot_len : room;
    if (copy_len > 0) {
        pbuf_copy_partial(p, state->request + state->len, (u16_t)copy_len, 0);
        state->len = (uint16_t)(state->len + copy_len);
        state->request[state->len] = '\0';
    }
    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    if (state->request_bytes >= 4096 || strstr(state->request, "\r\n") == NULL) {
        if (state->request_bytes >= 4096 || state->len >= sizeof(state->request) - 1u) {
            static const char bad_request[] = "Request too large\n";
            __atomic_add_fetch(&http_request_count, 1, __ATOMIC_RELAXED);
            return http_send_response(tpcb, "text/plain", 400, bad_request, sizeof(bad_request) - 1);
        }
        return ERR_OK;
    }
    if (!headers_done) return ERR_OK;
    __atomic_add_fetch(&http_request_count, 1, __ATOMIC_RELAXED);
    if (strncmp(state->request, "GET /api/health ", 16) == 0) {
        state->owned_body = mem_malloc(HTTP_HEALTH_CAPACITY);
        if (!state->owned_body) {
            __atomic_add_fetch(&http_error_count, 1, __ATOMIC_RELAXED);
            tcp_abort(tpcb);
            return ERR_ABRT;
        }
        int len = http_build_health_json(state->owned_body, HTTP_HEALTH_CAPACITY);
        if (len < 0 || len >= (int)HTTP_HEALTH_CAPACITY) {
            __atomic_add_fetch(&http_error_count, 1, __ATOMIC_RELAXED);
            tcp_abort(tpcb);
            return ERR_ABRT;
        }
        /* Release unused formatter capacity before retaining the snapshot
         * for transmission. The shipped lwIP heap shrinks this in place. */
        char *trimmed = mem_trim(state->owned_body, (size_t)len + 1u);
        if (trimmed) state->owned_body = trimmed;
        return http_send_response(tpcb, "application/json", 200, state->owned_body, (size_t)len);
    }
    if (strncmp(state->request, "GET / ", 6) == 0 ||
        strncmp(state->request, "GET /index.html ", 16) == 0) {
        return http_send_response(tpcb, "text/html; charset=utf-8", 200,
                                  http_dashboard, sizeof(http_dashboard) - 1);
    }
    static const char not_found[] = "Not found\n";
    return http_send_response(tpcb, "text/plain", 404, not_found, sizeof(not_found) - 1);
}

static err_t http_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    (void)arg;
    if (err != ERR_OK || !newpcb) return err;
    if (http_active_connections >= HTTP_MAX_CONNECTIONS) {
        tcp_abort(newpcb);
        return ERR_ABRT;
    }
    http_conn_state_t *state = (http_conn_state_t *)mem_calloc(1, sizeof(*state));
    if (!state) {
        tcp_abort(newpcb);
        return ERR_ABRT;
    }
    ++http_active_connections;
    tcp_arg(newpcb, state);
    tcp_recv(newpcb, http_recv);
    tcp_err(newpcb, http_err);
    tcp_poll(newpcb, http_poll, 2);
    return ERR_OK;
}
static void http_server_init(void) {
    if (http_pcb) return;
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (!pcb) {
        DBG_PRINTF("HTTP: pcb alloc failed\n");
        return;
    }
    if (tcp_bind(pcb, IP_ANY_TYPE, HTTP_SERVER_PORT) != ERR_OK) {
        DBG_PRINTF("HTTP: bind failed\n");
        tcp_abort(pcb);
        return;
    }
    http_pcb = tcp_listen(pcb);
    if (!http_pcb) {
        DBG_PRINTF("HTTP: listen failed\n");
        tcp_abort(pcb);
        return;
    }
    tcp_accept(http_pcb, http_accept);
    DBG_PRINTF("HTTP: dashboard listening on port %d\n", HTTP_SERVER_PORT);
}
#endif

static void pps_pio_irq_handler(void) {
    pio_interrupt_clear(pio0, 0);
#if !PPS_CAPTURE_DMA
    unsigned captures = 0;
    while (!pio_sm_is_rx_fifo_empty(pio0, 0)) {
        gps_pps_capture_ticks = pio_sm_get(pio0, 0);
        gps_pps_capture_valid = true;
        ++captures;
    }
    if (captures > 1 || gps_pps_tick) pps_capture_fault_pending = true;
#endif
    gps_pps_seen = true;
    gps_pps_tick = true;
}

#if PPS_CAPTURE_DMA
static void pps_dma_poll(void) {
    if (pps_dma_channel < 0) return;
    uintptr_t base = (uintptr_t)pps_dma_ring;
    uintptr_t write_addr = dma_hw->ch[pps_dma_channel].write_addr;
    uint32_t producer = (uint32_t)((write_addr - base) >> 2) & (PPS_DMA_RING_SIZE - 1u);
    uint32_t consumer = pps_dma_consumer & (PPS_DMA_RING_SIZE - 1u);
    if (producer == consumer) return;
    if (((producer - consumer) & (PPS_DMA_RING_SIZE - 1u)) > 1u) {
        pps_capture_fault_pending = true;
        return;
    }

    gps_pps_capture_ticks = pps_dma_ring[consumer];
    gps_pps_capture_valid = true;
    pps_dma_consumer = consumer + 1u;
    gps_pps_seen = true;
    gps_pps_tick = true;
}
#endif

static uint64_t median_u64(const uint64_t *values, uint32_t count) {
    uint64_t sorted[PPS_MEDIAN_WINDOW];
    if (count == 0) return 0;
    if (count > PPS_MEDIAN_WINDOW) count = PPS_MEDIAN_WINDOW;
    memcpy(sorted, values, count * sizeof(sorted[0]));
    for (uint32_t i = 1; i < count; ++i) {
        uint64_t value = sorted[i];
        uint32_t j = i;
        while (j > 0 && sorted[j - 1] > value) {
            sorted[j] = sorted[j - 1];
            --j;
        }
        sorted[j] = value;
    }
    return sorted[count / 2u];
}

static uint64_t ab_mad_ns(void) {
    if (ab_interval_history_count < 3u) return 0;
    uint64_t center = median_u64(ab_interval_history, ab_interval_history_count);
    uint64_t deviations[PPS_MEDIAN_WINDOW];
    for (uint32_t i = 0; i < ab_interval_history_count; ++i) {
        uint64_t sample = ab_interval_history[i];
        deviations[i] = sample > center ? sample - center : center - sample;
    }
    return median_u64(deviations, ab_interval_history_count);
}

static uint64_t ab_outlier_threshold_ns(void) {
    if (ab_interval_history_count < 3u) return UINT64_MAX;
    uint64_t mad = ab_mad_ns();
    uint64_t threshold = mad > (UINT64_MAX / 6u) ? UINT64_MAX : mad * 6u;
    if (threshold < (uint64_t)PPS_OUTLIER_FLOOR_NS) {
        threshold = (uint64_t)PPS_OUTLIER_FLOOR_NS;
    }
    return threshold;
}

static void ab_history_add(uint64_t interval_ns) {
    ab_interval_history[ab_interval_history_index] = interval_ns;
    ab_interval_history_index = (ab_interval_history_index + 1u) % PPS_MEDIAN_WINDOW;
    if (ab_interval_history_count < PPS_MEDIAN_WINDOW) {
        ab_interval_history_count++;
    }
}

static bool ab_servo_update(uint64_t measured_ns, int64_t *innovation_out) {
    uint64_t threshold = ab_outlier_threshold_ns();
    if (ab_interval_history_count >= 3u) {
        uint64_t center = median_u64(ab_interval_history, ab_interval_history_count);
        uint64_t residual = measured_ns > center ? measured_ns - center : center - measured_ns;
        if (residual > threshold) {
            if (pps_outliers_rejected < UINT32_MAX) pps_outliers_rejected++;
            if (innovation_out) {
                int64_t prediction = (int64_t)ab_period_ns +
                                     ab_period_rate_ns_per_sec;
                *innovation_out = (int64_t)measured_ns - prediction;
            }
            return false;
        }
    }

    ab_history_add(measured_ns);
    if (!ab_servo_valid) {
        ab_period_ns = measured_ns;
        ab_period_rate_ns_per_sec = 0;
        ab_servo_valid = true;
        pll_interval_ns = (uint32_t)measured_ns;
        if (innovation_out) *innovation_out = 0;
        return true;
    }

    int64_t prediction = (int64_t)ab_period_ns + ab_period_rate_ns_per_sec;
    int64_t innovation = (int64_t)measured_ns - prediction;
    int64_t phase_correction = innovation >> PPS_AB_ALPHA_SHIFT;
    int64_t rate_correction = innovation >> PPS_AB_BETA_SHIFT;
    // Alpha-beta state update: apply alpha to the predicted period, not the
    // previous period.  This incorporates the learned period-rate estimate.
    ab_period_ns = (uint64_t)(prediction + phase_correction);
    ab_period_rate_ns_per_sec += rate_correction;
    if (ab_period_rate_ns_per_sec > 200000LL) ab_period_rate_ns_per_sec = 200000LL;
    if (ab_period_rate_ns_per_sec < -200000LL) ab_period_rate_ns_per_sec = -200000LL;
    if (ab_period_ns < (uint64_t)(PPS_PLL_MIN_US * 1000ULL)) {
        ab_period_ns = (uint64_t)(PPS_PLL_MIN_US * 1000ULL);
    } else if (ab_period_ns > (uint64_t)(PPS_PLL_MAX_US * 1000ULL)) {
        ab_period_ns = (uint64_t)(PPS_PLL_MAX_US * 1000ULL);
    }
    pll_interval_ns = (uint32_t)ab_period_ns;
    if (innovation_out) *innovation_out = innovation;
    return true;
}

static void rtc_sqw_irq_handler(uint gpio, uint32_t events) {
    (void)events;
    if (gpio != RTC_SQW_PIN) {
        return;
    }
    last_rtc_sqw_time_ns = hires_time_ns();
    if (events & GPIO_IRQ_EDGE_RISE) {
        rtc_sqw_edge_high = true;
    } else if (events & GPIO_IRQ_EDGE_FALL) {
        rtc_sqw_edge_high = false;
    }
    rtc_sqw_seen = true;
    rtc_sqw_tick = true;
}

/* Establish the hardware counter epoch while interrupts cannot delay the
 * enable write. Never use FIFO/DMA processing time as the PPS phase anchor.
 * The remaining bracket width, divider phase and input synchronization are
 * covered by the existing conservative uncertainty allowance. */
static void pps_capture_start(void) {
    pio_sm_clkdiv_restart(pio0, 0);
    uint32_t save = save_and_disable_interrupts();
    uint64_t before_ns = hires_time_ns();
    pio_sm_set_enabled(pio0, 0, true);
    __dsb(); /* Complete the peripheral write before closing the bracket. */
    uint64_t after_ns = hires_time_ns();
    restore_interrupts(save);
    pps_capture_start_valid = after_ns >= before_ns && after_ns - before_ns <= 10000u;
    pps_capture_start_ns = before_ns + (after_ns >= before_ns ? (after_ns - before_ns) / 2u : 0);
}

static void pps_pio_init(void) {
    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &pps_capture_program);
    pps_program_offset = offset;
    pio_gpio_init(pio, GPS_PPS_PIN);

    pio_sm_config c = pps_capture_program_get_default_config(offset);
    sm_config_set_in_pins(&c, GPS_PPS_PIN);
    sm_config_set_jmp_pin(&c, GPS_PPS_PIN);
    uint32_t divider256 = 0;
    if (!pps_capture_divider(clock_get_hz(clk_sys), PPS_CAPTURE_COUNTER_HZ, &divider256)) {
        panic("PPS counter frequency requires an exact PIO divider");
    }
    sm_config_set_clkdiv_int_frac(&c, (uint16_t)(divider256 >> 8), (uint8_t)divider256);
    pio_sm_set_consecutive_pindirs(pio, sm, GPS_PPS_PIN, 1, false);
    pio_sm_init(pio, sm, offset, &c);

#if PPS_CAPTURE_DMA
    pps_dma_channel = dma_claim_unused_channel(true);
    dma_channel_config dma_cfg = dma_channel_get_default_config((uint)pps_dma_channel);
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_cfg, false);
    channel_config_set_write_increment(&dma_cfg, true);
    channel_config_set_ring(&dma_cfg, true, PPS_DMA_RING_BITS + 2u);
    channel_config_set_dreq(&dma_cfg, pio_get_dreq(pio, sm, false));
    pps_dma_consumer = 0;
    dma_channel_configure((uint)pps_dma_channel, &dma_cfg,
                          pps_dma_ring, &pio->rxf[sm],
                          dma_encode_endless_transfer_count(), true);
#else
    irq_set_exclusive_handler(PIO0_IRQ_0, pps_pio_irq_handler);
    irq_set_enabled(PIO0_IRQ_0, true);
    pio_set_irq0_source_enabled(pio, pis_interrupt0, true);
#endif
    /* DMA/IRQ consumer must be ready before the producer starts. */
    pps_capture_start();
}

/* Main-core only. A stalled counter/backlogged capture stream cannot be
 * repaired by adding the ordinary per-edge overhead. Restart both producer
 * and consumer, invalidate PPS trust, and let normal GNSS recovery requalify. */
static void pps_capture_service(void) {
    uint64_t now_ns = hires_time_ns();
    bool fault = pps_capture_fault_pending ||
        (pio0->fdebug & (1u << PIO_FDEBUG_RXSTALL_LSB)) ||
        (pps_capture_last_poll_ns && now_ns > pps_capture_last_poll_ns &&
         now_ns - pps_capture_last_poll_ns > 1500000000ULL);
    pps_capture_last_poll_ns = now_ns;
    bool backlog = false;
#if PPS_CAPTURE_DMA
    if (pps_dma_channel >= 0) {
        uintptr_t address = dma_hw->ch[pps_dma_channel].write_addr;
        uint32_t producer = (uint32_t)((address - (uintptr_t)pps_dma_ring) >> 2) & (PPS_DMA_RING_SIZE - 1u);
        if (((producer - pps_dma_consumer) & (PPS_DMA_RING_SIZE - 1u)) > 1u) {
            fault = true;
            backlog = true;
        }
    }
#endif
    if (!fault) return;
#if !PPS_CAPTURE_DMA
    irq_set_enabled(PIO0_IRQ_0, false);
#endif
    pio_sm_set_enabled(pio0, 0, false);
#if PPS_CAPTURE_DMA
    if (pps_dma_channel >= 0) dma_channel_abort((uint)pps_dma_channel);
#endif
    pio_sm_clear_fifos(pio0, 0);
    pio_sm_restart(pio0, 0);
    pio_sm_clkdiv_restart(pio0, 0);
    pio_sm_exec(pio0, 0, pio_encode_jmp(pps_program_offset));
    pio_interrupt_clear(pio0, 0);
    pio0->fdebug = 1u << PIO_FDEBUG_RXSTALL_LSB; /* write-one-to-clear */
    gps_pps_capture_valid = false;
    gps_pps_seen = false;
    gps_pps_tick = false;
    pps_capture_fault_pending = false;
    pps_capture_remainder = 0;
    mutex_enter_blocking(&stats_mutex);
    if (backlog && pps_dma_overruns < UINT32_MAX) ++pps_dma_overruns;
    prev_pps_capture_valid = false;
    prev_pps_time_ns = 0;
    pps_time_valid = false;
    pll_valid = false;
    accuracy_mode = false;
    trusted_pps_ns = 0;
    gnss_recovery.locked = false;
    gnss_recovery.good_pulses = 0;
    memset(&rtc_frequency_filter, 0, sizeof(rtc_frequency_filter));
    if (pps_capture_faults < UINT32_MAX) __atomic_add_fetch(&pps_capture_faults, 1, __ATOMIC_RELAXED);
    mutex_exit(&stats_mutex);
#if PPS_CAPTURE_DMA
    if (pps_dma_channel >= 0) {
        pps_dma_consumer = 0;
        dma_channel_set_write_addr((uint)pps_dma_channel, pps_dma_ring, false);
        dma_channel_set_trans_count((uint)pps_dma_channel, dma_encode_endless_transfer_count(), true);
    }
#else
    irq_set_enabled(PIO0_IRQ_0, true);
#endif
    pps_capture_start();
}

static void rtc_sqw_init(void) {
    gpio_init(RTC_SQW_PIN);
    gpio_set_dir(RTC_SQW_PIN, GPIO_IN);
    gpio_pull_up(RTC_SQW_PIN);
    gpio_set_irq_enabled_with_callback(RTC_SQW_PIN,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &rtc_sqw_irq_handler);
}

static bool wifi_init_and_connect(void) {
    if (cyw43_arch_init()) {
        DBG_PRINTF("WiFi init failed\n");
        return false;
    }
    wifi_driver_ready = true;

    cyw43_arch_enable_sta_mode();
#if WIFI_NO_POWERSAVE
    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);
#endif
    DBG_PRINTF("WiFi: connecting to %s\n", WIFI_SSID);
    int err = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASSWORD, WIFI_AUTH, WIFI_CONNECT_TIMEOUT_MS);
    if (err) {
        DBG_PRINTF("WiFi: connection failed (%d)\n", err);
        return true;
    }

    char ip_buf[16];
    get_ip_string(ip_buf, sizeof(ip_buf));
    DBG_PRINTF("WiFi: connected, IP=%s\n", ip_buf);
    sntp_server_init();
#if HTTP_SERVER_ENABLE
    http_server_init();
#endif
#if HEALTH_BROADCAST_ENABLE
    health_broadcast_init();
#endif
    return true;
}

static void core1_entry(void) {
    hires_time_init_core();
    if (!wifi_init_and_connect()) {
        DBG_PRINTF("WiFi disabled after initialization failure\n");
        while (true) {
            sleep_ms(1000);
        }
    }
#if HEALTH_BROADCAST_ENABLE
    uint64_t last_health_us = 0;
#endif
    uint64_t last_wifi_retry_us = time_us_64();
    uint32_t wifi_retry_ms = WIFI_RETRY_MIN_MS;
    while (true) {
        cyw43_arch_poll();
        uint64_t wifi_now_us = time_us_64();
        int wifi_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        if (wifi_status != CYW43_LINK_UP &&
            (wifi_now_us - last_wifi_retry_us) >= ((uint64_t)wifi_retry_ms * 1000ULL)) {
            last_wifi_retry_us = wifi_now_us;
            __atomic_add_fetch(&wifi_reconnect_count, 1, __ATOMIC_RELAXED);
            DBG_PRINTF("WiFi: link unavailable, reconnecting\n");
            if (cyw43_arch_wifi_connect_timeout_ms(
                    WIFI_SSID, WIFI_PASSWORD, WIFI_AUTH, WIFI_CONNECT_TIMEOUT_MS) == 0) {
                DBG_PRINTF("WiFi: reconnected\n");
                sntp_server_init();
#if HTTP_SERVER_ENABLE
                http_server_init();
#endif
#if HEALTH_BROADCAST_ENABLE
                health_broadcast_init();
#endif
                wifi_retry_ms = WIFI_RETRY_MIN_MS;
            } else {
                __atomic_add_fetch(&wifi_reconnect_fail_count, 1, __ATOMIC_RELAXED);
                if (wifi_retry_ms < WIFI_RETRY_MAX_MS / 2) {
                    wifi_retry_ms *= 2;
                } else {
                    wifi_retry_ms = WIFI_RETRY_MAX_MS;
                }
                DBG_PRINTF("WiFi: reconnect failed\n");
            }
        }
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, get_led_output() ? 1 : 0);
#if HEALTH_BROADCAST_ENABLE
        uint64_t health_now_us = time_us_64();
        if ((health_now_us - last_health_us) >= 1000000ULL) {
            last_health_us = health_now_us;
            cyw43_arch_lwip_begin();
            health_broadcast_send();
            cyw43_arch_lwip_end();
        }
#endif
        sleep_ms(1);
    }
}

int main(void) {
    stdio_init_all();
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
    mutex_init(&rtc_i2c_mutex);
    mutex_init(&stats_mutex);
    hires_time_init_shared();
    hires_time_init_core();

    // Give USB serial time to enumerate on host.
    sleep_ms(2000);

    gps_uart_configure_module();
    gps_line_reset();
    gps_uart_irq_init();
    rtc_init();
    uint8_t boot_status = 0;
    rtc_osf_boot_read = rtc_read_reg(RTC_STATUS_REG, &boot_status);
    rtc_osf_at_boot = !rtc_osf_boot_read || !rtc_calendar_status_valid(boot_status);
    rtc_calendar_needs_set = rtc_osf_at_boot;
    rtc_cache_poll();
    if (rtc_configure_square_wave_1hz()) {
        DBG_PRINTF("RTC SQW: 1Hz enabled on GP%d\n", RTC_SQW_PIN);
    } else {
        DBG_PRINTF("RTC SQW: failed to enable 1Hz output\n");
    }
    rtc_sqw_init();
    rtc_32k_init();
    int8_t boot_aging = 0;
    (void)rtc_read_aging(&boot_aging);
    gpio_init(GPS_PPS_PIN);
    gpio_set_dir(GPS_PPS_PIN, GPIO_IN);
    gpio_pull_down(GPS_PPS_PIN);
    pps_pio_init();
tm1637_init();

    rtc_cache_poll();
    update_time_snapshot();
    last_time_snapshot_update = get_absolute_time();

    multicore_launch_core1_with_stack(core1_entry, network_core_stack, sizeof(network_core_stack));

    DBG_PRINTF("L76K GPS reader started. Baud=%d, TX=GP%d, RX=GP%d\n",
           GPS_UART_BAUD, GPS_UART_TX_PIN, GPS_UART_RX_PIN);
    DBG_PRINTF("Waiting for GPS stats...\n");

    /* Independent, main-loop-only observation for uncertainty reporting.
     * Never feeds PPS epoch, period, PIO, DMA or servo correction. */
    uint64_t placement_last_low_ns = 0;
    bool placement_was_high = true;
    while (true) {
        watchdog_update();
        pps_capture_service();
        uint64_t placement_before_ns = hires_time_ns();
        bool placement_high = gpio_get(GPS_PPS_PIN);
        if (!placement_high) {
            placement_last_low_ns = placement_before_ns;
        } else if (!placement_was_high && placement_last_low_ns) {
            uint64_t placement_after_ns = hires_time_ns();
            mutex_enter_blocking(&stats_mutex);
            pps_placement_brackets[1] = pps_placement_brackets[0];
            pps_placement_brackets[0] = (ntp_edge_bracket_t){placement_last_low_ns, placement_after_ns};
            mutex_exit(&stats_mutex);
        }
        placement_was_high = placement_high;
#if PPS_CAPTURE_DMA
        pps_dma_poll();
#endif
        bool pps_display_edge = false;
        int64_t pps_display_time_us = -1;
        if (gps_pps_tick) {
            gps_pps_tick = false;
            if (gps_pps_seen) {
                gps_pps_seen = false;
                pps_display_edge = true;
                if (!pps_reported) {
                    DBG_PRINTF("GPS PPS detected on GP%d\n", GPS_PPS_PIN);
                    pps_reported = true;
                }
                absolute_time_t now = get_absolute_time();
                int64_t now_us = to_us_since_boot(now);
                pps_display_time_us = now_us;
                uint64_t now_ns = hires_time_ns();
                mutex_enter_blocking(&stats_mutex);
                last_pps_us = now_us;
                uint32_t capture_ticks = 0;
                bool capture_valid = false;
                uint32_t irq_save = save_and_disable_interrupts();
                capture_ticks = gps_pps_capture_ticks;
                capture_valid = gps_pps_capture_valid;
                gps_pps_capture_valid = false;
                restore_interrupts(irq_save);
                uint64_t pps_ns = now_ns;
                if (capture_valid && prev_pps_capture_valid &&
                    prev_pps_time_ns > 0) {
                    uint64_t delta_ns = pps_capture_elapsed_ns(prev_pps_capture_ticks,
                        capture_ticks, PPS_CAPTURE_COUNTER_HZ, &pps_capture_remainder);
                    if (delta_ns < PPS_PLL_MIN_US * 1000ULL || delta_ns > PPS_PLL_MAX_US * 1000ULL) {
                        /* Missing/extra edges break the consecutive-pulse
                         * assumption. Do not integrate an ambiguous interval. */
                        pps_capture_fault_pending = true;
                        mutex_exit(&stats_mutex);
                        continue;
                    }
                    pps_ns = prev_pps_time_ns + delta_ns;
                } else if (!capture_valid || !pps_capture_start_valid ||
                           !pps_capture_first_ns(capture_ticks, PPS_CAPTURE_COUNTER_HZ,
                               pps_capture_start_ns, now_ns, &pps_ns, &pps_capture_remainder)) {
                    pps_capture_fault_pending = true;
                    mutex_exit(&stats_mutex);
                    continue;
                }
                last_pps_time_ns = pps_ns;
                if (capture_valid) {
                    prev_pps_capture_ticks = capture_ticks;
                    prev_pps_capture_valid = true;
                }
                bool accepted_pps = false;
                if (have_gps_time) {
                    int64_t age_us = absolute_time_diff_us(last_rmc_update_time, now);
                    if (age_us < 0 || age_us > 500000) {
                        increment_utc_one_second(&g_stats);
                    }
                    if (prev_pps_time_ns > 0 && pps_ns > prev_pps_time_ns) {
                        uint64_t measured = pps_ns - prev_pps_time_ns;
                        if (measured >= (PPS_PLL_MIN_US * 1000ULL) &&
                            measured <= (PPS_PLL_MAX_US * 1000ULL)) {
                            // Keep the constant error from the ideal 1-second
                            // interval separate from true short-term jitter.
                            // The latter is measured around the learned median
                            // interval, so a stable receiver frequency error
                            // is not incorrectly reported as jitter.
                            int64_t period_error_ns = (int64_t)measured - 1000000000LL;
                            uint64_t jitter_reference_ns = ab_interval_history_count > 0
                                ? median_u64(ab_interval_history, ab_interval_history_count)
                                : 1000000000ULL;
                            int64_t jitter_ns = (int64_t)measured -
                                (int64_t)jitter_reference_ns;
                            pps_period_error_last_ns = period_error_ns;
                            int32_t jitter = (int32_t)(jitter_ns / 1000LL);
                            pps_jitter_last = jitter;
                            pps_jitter_last_ns = jitter_ns;
                            pps_jitter_valid = true;
                            if (pps_jitter_count == PPS_JITTER_WINDOW) {
                                int32_t old = pps_jitter_buf[pps_jitter_index];
                                int64_t old_ns = pps_jitter_ns_buf[pps_jitter_index];
                                pps_jitter_sumsq -= (int64_t)old * (int64_t)old;
                                pps_jitter_ns_sumsq -= old_ns * old_ns;
                            }
                            pps_jitter_buf[pps_jitter_index] = jitter;
                            pps_jitter_ns_buf[pps_jitter_index] = jitter_ns;
                            pps_jitter_sumsq += (int64_t)jitter * (int64_t)jitter;
                            pps_jitter_ns_sumsq += jitter_ns * jitter_ns;
                            pps_jitter_index = (pps_jitter_index + 1) % PPS_JITTER_WINDOW;
                            if (pps_jitter_count < PPS_JITTER_WINDOW) {
                                pps_jitter_count++;
                            }
#if GPS_PROTOCOL_UBX
                            DBG_PRINTF("PPS: interval=%llu ns jitter=%lld ns\n",
                                       (unsigned long long)measured,
                                       (long long)jitter_ns);
#endif
                            // The adaptive alpha-beta servo maintains both
                            // the PPS period and its slow frequency drift.
                            // Median/MAD rejection prevents one bad sample
                            // from changing either state variable.
                            pll_phase_error_ns = 0;
                            ab_innovation_ns = 0;
                            if (ab_servo_update(measured, &ab_innovation_ns)) {
                                pll_valid = true;
                                accepted_pps = true;
                            }
                        }
                    }
                    if (rmc_epoch_valid) {
                        if (now_ns > rmc_epoch_time_ns &&
                            (now_ns - rmc_epoch_time_ns) < 1500000000ULL) {
                            pps_epoch_seconds = rmc_epoch_seconds + 1;
                        } else {
                            pps_epoch_seconds = rmc_epoch_seconds;
                        }
                        pps_time_valid = true;
                        rmc_epoch_valid = false;
                    } else if (pps_time_valid) {
                        pps_epoch_seconds += 1;
                    }
                    pps_tick_count++;
                    if ((pps_tick_count & 0x03u) == 0u) {
                        update_accuracy_mode_locked();
                    }
                    last_gps_fix_time = now;
                    time_snapshot_dirty = true;
                    if (g_led_mode == LED_MODE_BLINK) {
                        // Use the UTC second associated with this PPS edge
                        // as the common indicator phase.  This keeps the
                        // onboard LED and TM1637 colon aligned, and removes
                        // arbitrary phase differences after boot.
                        pps_indicator_phase = (pps_epoch_seconds & 1LL) != 0;
                        g_led_state = pps_indicator_phase;
                    }
                } else {
                    pps_time_valid = false;
                    accuracy_mode = false;
                    pll_filter_shift = PPS_PLL_FILTER_SHIFT;
                    rmc_phase_ok_count = 0;
                    pps_tick_count = 0;
                    update_led_mode_locked(now_us);
                }
                /* The raw capture timeline advances even while GNSS calendar
                 * messages are unavailable; it is not a GPS-fix-dependent clock. */
                prev_pps_time_ns = pps_ns;
                if (gnss_recovery_observe(&gnss_recovery, pps_ns, accepted_pps && pps_time_valid)) {
                    trusted_pps_ns = pps_ns;
                    trusted_pps_epoch_s = pps_epoch_seconds;
                    trusted_pps_period_ns = pll_interval_ns;
                }
                mutex_exit(&stats_mutex);
                last_time_snapshot_update = now;
                rtc_apply_pending_set_if_due();
            }
        }

        if (rtc_sqw_tick) {
            rtc_sqw_tick = false;
            if (rtc_sqw_seen) {
                uint64_t sqw_ns = 0;
                bool edge_high = false;
                uint32_t save = save_and_disable_interrupts();
                sqw_ns = last_rtc_sqw_time_ns;
                edge_high = rtc_sqw_edge_high;
                rtc_sqw_seen = false;
                restore_interrupts(save);
                if (!rtc_sqw_reported) {
                    DBG_PRINTF("RTC SQW detected on GP%d\n", RTC_SQW_PIN);
                    rtc_sqw_reported = true;
                }
                mutex_enter_blocking(&stats_mutex);
                last_rtc_sqw_us = to_us_since_boot(get_absolute_time());
                uint64_t gps_pps_ns = get_last_pps_time_ns();
                bool second_edge = false;
                bool pps_settled = pps_time_valid && pll_valid && gnss_recovery.locked &&
                    (pps_tick_count >= RTC_SQW_PPS_SETTLE_TICKS) &&
                    sqw_ns >= gps_pps_ns && sqw_ns - gps_pps_ns < 1500000000ULL;
                if (pps_settled && rtc_calendar_valid && !rtc_conversion_pending && !rtc_conversion_failed) {
                    int64_t phase_ns = (int64_t)sqw_ns - (int64_t)gps_pps_ns;
                    if (phase_ns > 500000000LL) phase_ns -= 1000000000LL;
                    bool selected_edge = rtc_sqw_second_edge_known && edge_high == rtc_sqw_second_edge_high;
                    bool discover = !rtc_sqw_second_edge_known && llabs(phase_ns) <= RTC_PHASE_DISCOVERY_NS;
                    if (selected_edge || discover) {
                        /* Scale counter nanoseconds to GNSS nanoseconds before
                         * learning phase/frequency; retain one polarity. */
                        phase_ns = (int64_t)((double)phase_ns * 1e9 / pll_interval_ns);
                        rtc_phase_error_ns = phase_ns;
                        rtc_sqw_phase_last_us = (int32_t)(phase_ns / 1000);
                        if (rtc_phase_observe(&rtc_phase_filter, phase_ns)) {
                            rtc_bad_phase_count = 0;
                            rtc_sqw_second_edge_known = true;
                            rtc_sqw_second_edge_high = edge_high;
                            rtc_sqw_phase_valid = true;
                            second_edge = true;
                            rtc_sqw_epoch_seconds = pps_epoch_seconds + (phase_ns < 0 ? 1 : 0);
                            if (rtc_last_gnss_observation_ns &&
                                sqw_ns - rtc_last_gnss_observation_ns > 2500000000ULL) {
                                memset(&rtc_frequency_filter, 0, sizeof(rtc_frequency_filter));
                            }
                            rtc_last_gnss_observation_ns = sqw_ns;
                            rtc_frequency_observe(&rtc_frequency_filter, sqw_ns, phase_ns);
                            rtc_phase_drift_update((int32_t)(phase_ns / 1000), rtc_sqw_epoch_seconds);
                            rtc_sqw_time_valid = rtc_phase_filter.qualified;
                            if (rtc_sqw_time_valid) {
                                ntp_uncertainty_t pps_error = get_pps_uncertainty_locked(sqw_ns);
                                /* SQW maps to the PPS UTC target, not the currently
                                 * slewing served clock. Its live slew is added once
                                 * when reporting holdover dispersion. */
                                rtc_reference_error_ns = pps_error.total_ns - pps_error.slew_ns;
                                g_rtc_state = RTC_STATE_LOCKED;
                                rtc_last_qualified_ns = sqw_ns;
                                rtc_last_qualified_epoch_s = rtc_sqw_epoch_seconds;
                                rtc_holdover_phase_ns = rtc_phase_filter.offset_ns;
                                rtc_holdover_fast_ppm = rtc_frequency_filter.valid ? rtc_frequency_filter.frequency_error_ppm : 0;
                                rtc_holdover_frequency_valid = rtc_frequency_filter.valid;
                                rtc_learned_uncertainty_ppm = RTC_FREQUENCY_UNCERTAINTY_PPM +
                                    (rtc_frequency_filter.valid ? rtc_frequency_filter.uncertainty_ppm : 0);
                            }
                        } else {
                            rtc_last_reject_reason = llabs(phase_ns) > RTC_PHASE_MAX_OFFSET_NS ? "PHASE_RANGE" : "PHASE_OUTLIER";
                            if (rtc_phase_reject_total < UINT32_MAX) ++rtc_phase_reject_total;
                            if (++rtc_bad_phase_count >= 3) rtc_invalidate_mapping_locked();
                        }
                    }
                } else if (rtc_sqw_time_valid && rtc_sqw_second_edge_known &&
                           edge_high == rtc_sqw_second_edge_high) {
                    second_edge = true;
                    rtc_sqw_epoch_seconds += 1;
                }

                if (second_edge) {
                    if (prev_rtc_sqw_time_ns > 0 && sqw_ns > prev_rtc_sqw_time_ns) {
                        uint64_t measured = sqw_ns - prev_rtc_sqw_time_ns;
                        if (measured >= (RTC_SQW_MIN_US * 1000ULL) &&
                            measured <= (RTC_SQW_MAX_US * 1000ULL)) {
                            if (!rtc_sqw_pll_valid) {
                                rtc_sqw_interval_ns = (uint32_t)measured;
                                rtc_sqw_pll_valid = true;
                            } else {
                                int64_t diff = (int64_t)measured - (int64_t)rtc_sqw_interval_ns;
                                if (diff != 0) {
                                    rtc_sqw_interval_ns = (uint32_t)((int64_t)rtc_sqw_interval_ns +
                                                                     (diff >> RTC_SQW_PLL_FILTER_SHIFT));
                                }
                            }
                        } else {
                            rtc_last_reject_reason = "SQW_PERIOD";
                            rtc_invalidate_mapping_locked();
                            second_edge = false;
                        }
                    }
                    if (second_edge) {
                        prev_rtc_sqw_time_ns = sqw_ns;
                        last_rtc_sqw_second_time_ns = sqw_ns;
                    }
                }
                time_snapshot_dirty = true;
                mutex_exit(&stats_mutex);
            }
        }

        uint64_t rtc_32k_ns = 0;
        uint32_t rtc_32k_sequence = 0;
        if (rtc_32k_take(&rtc_32k_ns, &rtc_32k_sequence)) {
            mutex_enter_blocking(&stats_mutex);
            if (!rtc_conversion_pending && rtc_calendar_valid) {
                rtc_32k_observe(&rtc_32k, rtc_32k_ns, rtc_32k_sequence,
                    gnss_recovery.locked && pps_time_valid && pll_valid, pll_interval_ns);
            }
            mutex_exit(&stats_mutex);
        }
        mutex_enter_blocking(&stats_mutex);
        if (rtc_32k.last_ns && hires_time_ns() - rtc_32k.last_ns > 2500000000ULL) rtc_32k.valid = false;
        mutex_exit(&stats_mutex);
        rtc_cache_poll();
        rtc_calibration_poll();
        rtc_apply_pending_set_if_due();

        int64_t age_us = absolute_time_diff_us(last_time_snapshot_update, get_absolute_time());
        if (age_us > 1000000) {
            update_time_snapshot();
            last_time_snapshot_update = get_absolute_time();
        }

        mutex_enter_blocking(&stats_mutex);
        if (have_gps_time) {
            int64_t rmc_stale_ms =
                absolute_time_diff_us(last_rmc_update_time, get_absolute_time()) / 1000;
            if (rmc_stale_ms < 0 || rmc_stale_ms > GPS_HOLDOVER_TIMEOUT_MS) {
                have_gps_time = false;
                reset_rmc_phase_tracking_locked();
            }
        }
        mutex_exit(&stats_mutex);

    if (pps_reported) {
        absolute_time_t now = get_absolute_time();
        int64_t last_pps_local = -1;
        mutex_enter_blocking(&stats_mutex);
        last_pps_local = last_pps_us;
        mutex_exit(&stats_mutex);
        if (last_pps_local >= 0 &&
            (to_us_since_boot(now) - last_pps_local) > (int64_t)PPS_PRESENT_TIMEOUT_US) {
            DBG_PRINTF("GPS PPS lost\n");
            pps_reported = false;
            /* The next first-capture mapping requires a freshly initialized
             * X counter, not a counter with previous pulse overhead. */
            pps_capture_fault_pending = true;
            mutex_enter_blocking(&stats_mutex);            pps_time_valid = false;
            pll_valid = false;
            accuracy_mode = false;
            pll_filter_shift = PPS_PLL_FILTER_SHIFT;
            pps_jitter_valid = false;
            pps_jitter_count = 0;
            pps_jitter_index = 0;
            pps_jitter_sumsq = 0;
            pps_jitter_last_ns = 0;
            pps_jitter_ns_sumsq = 0;
            pps_jitter_rms_us = 0;
            pll_phase_error_ns = 0;
            ab_innovation_ns = 0;
            pps_period_error_last_ns = 0;
            // Preserve the last learned oscillator relationship.  The PPS
            // and NTP quality flags above are invalidated, but retaining the
            // period/rate lets reacquisition start from the calibrated local
            // oscillator instead of assuming exactly 1 second.
            ab_interval_history_count = 0;
            ab_interval_history_index = 0;
            pps_outliers_rejected = 0;
            reset_rmc_phase_tracking_locked();
            pps_tick_count = 0;
            prev_pps_capture_valid = false;
            prev_pps_time_ns = 0;
            gnss_recovery.locked = false;
            gnss_recovery.good_pulses = 0;
            memset(&rtc_frequency_filter, 0, sizeof(rtc_frequency_filter));
            mutex_exit(&stats_mutex);
        }
    }

    if (rtc_sqw_reported) {
        absolute_time_t now = get_absolute_time();
        int64_t last_sqw_local = -1;
        mutex_enter_blocking(&stats_mutex);
        last_sqw_local = last_rtc_sqw_us;
        mutex_exit(&stats_mutex);
        if (last_sqw_local >= 0 &&
            (to_us_since_boot(now) - last_sqw_local) > (int64_t)RTC_SQW_PRESENT_TIMEOUT_US) {
            DBG_PRINTF("RTC SQW lost\n");
            rtc_sqw_reported = false;
            mutex_enter_blocking(&stats_mutex);
            rtc_sqw_time_valid = false;
            rtc_sqw_phase_valid = false;
            rtc_sqw_phase_good_count = 0;
            rtc_sqw_pll_valid = false;
            prev_rtc_sqw_time_ns = 0;
            last_rtc_sqw_second_time_ns = 0;
            rtc_sqw_second_edge_known = false;
            rtc_last_reject_reason = "SQW_TIMEOUT";
            rtc_invalidate_mapping_locked();
            mutex_exit(&stats_mutex);
        }
    }

        uint32_t display_ntp = 0;
        uint32_t display_fraction = 0;
        bool display_time_valid = false;
        sntp_quality_t display_quality = SNTP_QUALITY_NONE;
        display_time_valid = get_sntp_time_at(hires_time_ns(), &display_ntp, &display_fraction, NULL, &display_quality);
        bool display_pps_locked = display_quality == SNTP_QUALITY_PPS;
        bool display_rtc_fallback = display_quality == SNTP_QUALITY_RTC_SQW ||
            display_quality == SNTP_QUALITY_COARSE_RTC;
        if (display_time_valid && strcmp(DISPLAY_TIME_MODE, "UTC") != 0) {
            int32_t display_offset = 0;
            bool display_dst = false;
            mutex_enter_blocking(&stats_mutex);
            timezone_id_t display_zone = timezone_state.current_id;
            mutex_exit(&stats_mutex);
            int64_t display_unix = (int64_t)display_ntp - NTP_DELTA;
            if (timezone_get_offset(display_zone, display_unix, &display_offset, &display_dst)) {
                display_ntp = (uint32_t)(display_unix + display_offset + NTP_DELTA);
            }
            (void)display_dst;
        }
        tm1637_update(time_us_64(), display_ntp, display_time_valid, pps_display_edge,
                      pps_indicator_phase,
                      pps_display_time_us, display_pps_locked, display_rtc_fallback);

    if (time_snapshot_dirty) {
        time_snapshot_dirty = false;
        update_time_snapshot();
        last_time_snapshot_update = get_absolute_time();
    }

        // Presentation-only work.  The lookup is cached and only repeats
        // after a substantial position change; it is never part of PPS/NTP
        // timestamp generation.
        timezone_refresh();

 #if GPS_PROTOCOL_UBX
        ubx_frame_t ubx_frame;
        if (ubx_frame_pop(&ubx_frame)) {
            process_ubx_frame(&ubx_frame);
        } else {
            tight_loop_contents();
        }
 #else
        gps_line_t gps_line;
        if (gps_line_pop(&gps_line)) {
            char *line = gps_line.text;
            if (!nmea_checksum_valid(line)) {
                __atomic_add_fetch(&gps_checksum_error_count, 1, __ATOMIC_RELAXED);
                continue;
            }
            if (strncmp(line, "$GPGGA", 6) == 0 || strncmp(line, "$GNGGA", 6) == 0) {
                bool parsed = false;
                gps_stats_t stats_copy;
                time_source_t ts_copy;
                mutex_enter_blocking(&stats_mutex);
                parsed = parse_gga(line, &g_stats);
                if (parsed) {
                    update_time_snapshot_locked(hires_time_ns());
                    stats_copy = g_stats;
                    ts_copy = g_time_snapshot;
                }
                mutex_exit(&stats_mutex);
                if (parsed) {
                    print_stats(&stats_copy, &ts_copy);
                }
            } else if (strncmp(line, "$GPGSA", 6) == 0 || strncmp(line, "$GNGSA", 6) == 0 ||
                       strncmp(line, "$GLGSA", 6) == 0 || strncmp(line, "$GAGSA", 6) == 0 ||
                       strncmp(line, "$GBGSA", 6) == 0 || strncmp(line, "$BDGSA", 6) == 0) {
                mutex_enter_blocking(&stats_mutex);
                parse_gsa(line, &g_stats);
                mutex_exit(&stats_mutex);
            } else if (strncmp(line, "$GPGSV", 6) == 0 || strncmp(line, "$GNGSV", 6) == 0 ||
                       strncmp(line, "$GLGSV", 6) == 0 || strncmp(line, "$GAGSV", 6) == 0 ||
                       strncmp(line, "$GBGSV", 6) == 0 || strncmp(line, "$BDGSV", 6) == 0) {
                mutex_enter_blocking(&stats_mutex);
                parse_gsv(line, &g_stats);
                mutex_exit(&stats_mutex);
            } else if (strncmp(line, "$GPRMC", 6) == 0 || strncmp(line, "$GNRMC", 6) == 0) {
                bool parsed = false;
                bool rmc_valid = false;
                gps_stats_t stats_copy;
                mutex_enter_blocking(&stats_mutex);
                parsed = parse_rmc(line, &g_stats, gps_line.first_byte_ns);
                if (parsed) {
                    update_time_snapshot_locked(hires_time_ns());
                    stats_copy = g_stats;
                    rmc_valid = g_stats.rmc_valid;
                }
                mutex_exit(&stats_mutex);
                if (parsed && rmc_valid) {
                    discipline_rtc_from_gps(&stats_copy);
                }
            }
        } else {
            tight_loop_contents();
        }
 #endif
    }
}
