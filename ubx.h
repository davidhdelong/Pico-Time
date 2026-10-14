#ifndef UBX_H
#define UBX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UBX_MAX_PAYLOAD 1024

typedef struct {
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    uint8_t payload[UBX_MAX_PAYLOAD];
    uint64_t first_byte_ns;
} ubx_frame_t;

typedef struct {
    uint8_t state;
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    uint16_t index;
    uint8_t payload[UBX_MAX_PAYLOAD];
    uint8_t ck_a;
    uint8_t ck_b;
    uint8_t rx_ck_a;
    uint8_t rx_ck_b;
    uint64_t first_byte_ns;
} ubx_parser_t;

typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t time_accuracy_ns;
    int32_t nano;
    uint8_t fix_type;
    uint8_t flags;
    uint8_t satellites;
    int32_t longitude_1e7;
    int32_t latitude_1e7;
    int32_t height_msl_mm;
    int32_t ground_speed_mm_s;
    uint16_t pdop_100;
} ubx_nav_pvt_t;

typedef struct {
    uint32_t tow_ms;
    uint32_t tow_sub_ms;
    int32_t q_err;
    uint16_t week;
    uint8_t flags;
    uint8_t ref_info;
} ubx_tim_tp_t;

typedef struct {
    uint8_t total;
    uint8_t used;
    uint8_t gps_total;
    uint8_t gps_used;
    uint8_t sbas_total;
    uint8_t sbas_used;
    uint8_t galileo_total;
    uint8_t galileo_used;
    uint8_t beidou_total;
    uint8_t beidou_used;
    uint8_t qzss_total;
    uint8_t qzss_used;
    uint8_t glonass_total;
    uint8_t glonass_used;
} ubx_nav_sat_summary_t;

void ubx_parser_init(ubx_parser_t *parser);
bool ubx_parser_feed(ubx_parser_t *parser, uint8_t byte, uint64_t byte_time_ns,
                     ubx_frame_t *out);
bool ubx_decode_nav_pvt(const ubx_frame_t *frame, ubx_nav_pvt_t *out);
bool ubx_decode_tim_tp(const ubx_frame_t *frame, ubx_tim_tp_t *out);
bool ubx_decode_nav_sat(const ubx_frame_t *frame, ubx_nav_sat_summary_t *out);

#endif
