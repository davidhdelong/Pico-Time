#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "../../ubx.h"
typedef struct {
    bool time_valid, fix, position_valid, satellites_valid;
    int64_t epoch_s;
    double latitude, longitude;
    unsigned satellites;
} p4_gnss_t;
bool p4_gnss_nmea(char *line, p4_gnss_t *out);
bool p4_gnss_ubx(const ubx_frame_t *frame, p4_gnss_t *out);
size_t p4_gnss_ubx_config(uint8_t *out, size_t capacity);
bool p4_gnss_config_ack(const ubx_frame_t *frame, bool *accepted);
