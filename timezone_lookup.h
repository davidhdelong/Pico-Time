#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef uint16_t timezone_id_t;

#define TIMEZONE_ID_UTC ((timezone_id_t)0)

typedef struct {
    int year, month, day, hour, min, sec;
} timezone_local_time_t;

timezone_id_t timezone_from_coordinates(int32_t latitude_e7, int32_t longitude_e7);
const char *timezone_name(timezone_id_t zone);
bool timezone_id_from_name(const char *name, timezone_id_t *out);
bool timezone_get_offset(timezone_id_t zone, int64_t utc_epoch,
                         int32_t *offset_seconds, bool *is_dst);
bool timezone_utc_to_local(int64_t utc_epoch, timezone_id_t zone,
                           timezone_local_time_t *out);
