#include "ubx.h"

#include <string.h>

enum {
    UBX_SYNC1 = 0,
    UBX_SYNC2,
    UBX_CLASS,
    UBX_ID,
    UBX_LEN_LO,
    UBX_LEN_HI,
    UBX_PAYLOAD,
    UBX_CK_A,
    UBX_CK_B
};

static uint16_t get_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t get_i32(const uint8_t *p) {
    return (int32_t)get_u32(p);
}

static void checksum_add(ubx_parser_t *parser, uint8_t byte) {
    parser->ck_a = (uint8_t)(parser->ck_a + byte);
    parser->ck_b = (uint8_t)(parser->ck_b + parser->ck_a);
}

void ubx_parser_init(ubx_parser_t *parser) {
    if (parser) {
        memset(parser, 0, sizeof(*parser));
        parser->state = UBX_SYNC1;
    }
}

bool ubx_parser_feed(ubx_parser_t *parser, uint8_t byte, uint64_t byte_time_ns,
                     ubx_frame_t *out) {
    if (!parser || !out) return false;

    switch (parser->state) {
    case UBX_SYNC1:
        if (byte == 0xB5u) {
            parser->first_byte_ns = byte_time_ns;
            parser->state = UBX_SYNC2;
        }
        break;
    case UBX_SYNC2:
        if (byte == 0x62u) {
            parser->state = UBX_CLASS;
            parser->ck_a = 0;
            parser->ck_b = 0;
        } else if (byte == 0xB5u) {
            /* This byte replaces the preceding possible sync prefix. */
            parser->first_byte_ns = byte_time_ns;
        } else {
            parser->state = UBX_SYNC1;
        }
        break;
    case UBX_CLASS:
        parser->cls = byte;
        checksum_add(parser, byte);
        parser->state = UBX_ID;
        break;
    case UBX_ID:
        parser->id = byte;
        checksum_add(parser, byte);
        parser->state = UBX_LEN_LO;
        break;
    case UBX_LEN_LO:
        parser->len = byte;
        checksum_add(parser, byte);
        parser->state = UBX_LEN_HI;
        break;
    case UBX_LEN_HI:
        parser->len |= (uint16_t)byte << 8;
        checksum_add(parser, byte);
        if (parser->len > UBX_MAX_PAYLOAD) {
            parser->state = UBX_SYNC1;
        } else if (parser->len == 0) {
            parser->state = UBX_CK_A;
        } else {
            parser->index = 0;
            parser->state = UBX_PAYLOAD;
        }
        break;
    case UBX_PAYLOAD:
        parser->payload[parser->index++] = byte;
        checksum_add(parser, byte);
        if (parser->index >= parser->len) parser->state = UBX_CK_A;
        break;
    case UBX_CK_A:
        parser->rx_ck_a = byte;
        parser->state = UBX_CK_B;
        break;
    case UBX_CK_B:
        parser->rx_ck_b = byte;
        parser->state = UBX_SYNC1;
        if (parser->rx_ck_a == parser->ck_a && parser->rx_ck_b == parser->ck_b) {
            out->cls = parser->cls;
            out->id = parser->id;
            out->len = parser->len;
            out->first_byte_ns = parser->first_byte_ns;
            if (out->len > 0) memcpy(out->payload, parser->payload, out->len);
            return true;
        }
        break;
    default:
        parser->state = UBX_SYNC1;
        break;
    }
    return false;
}

bool ubx_decode_nav_pvt(const ubx_frame_t *frame, ubx_nav_pvt_t *out) {
    if (!frame || !out || frame->cls != 0x01u || frame->id != 0x07u || frame->len < 92u) {
        return false;
    }
    const uint8_t *p = frame->payload;
    out->year = get_u16(p + 4);
    out->month = p[6]; out->day = p[7]; out->hour = p[8];
    out->min = p[9]; out->sec = p[10]; out->valid = p[11];
    out->time_accuracy_ns = get_u32(p + 12);
    out->nano = get_i32(p + 16);
    out->fix_type = p[20]; out->flags = p[21]; out->satellites = p[23];
    out->longitude_1e7 = get_i32(p + 24);
    out->latitude_1e7 = get_i32(p + 28);
    out->height_msl_mm = get_i32(p + 36);
    out->ground_speed_mm_s = get_i32(p + 60);
    out->pdop_100 = get_u16(p + 76);
    return true;
}

bool ubx_decode_tim_tp(const ubx_frame_t *frame, ubx_tim_tp_t *out) {
    if (!frame || !out || frame->cls != 0x0Du || frame->id != 0x01u || frame->len < 16u) {
        return false;
    }
    const uint8_t *p = frame->payload;
    out->tow_ms = get_u32(p);
    out->tow_sub_ms = get_u32(p + 4);
    out->q_err = get_i32(p + 8);
    out->week = get_u16(p + 12);
    out->flags = p[14];
    out->ref_info = p[15];
    return true;
}

bool ubx_decode_nav_sat(const ubx_frame_t *frame, ubx_nav_sat_summary_t *out) {
    if (!frame || !out || frame->cls != 0x01u || frame->id != 0x35u || frame->len < 8u) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    uint8_t count = frame->payload[5];
    if ((size_t)8u + (size_t)count * 12u > frame->len) return false;
    out->total = count;
    for (uint8_t i = 0; i < count; ++i) {
        const uint8_t *sv = frame->payload + 8u + (size_t)i * 12u;
        uint8_t gnss_id = sv[0];
        bool used = (get_u32(sv + 8) & (1u << 3)) != 0;
        uint8_t *total = NULL;
        uint8_t *used_count = NULL;
        switch (gnss_id) {
        case 0: total = &out->gps_total; used_count = &out->gps_used; break;
        case 1: total = &out->sbas_total; used_count = &out->sbas_used; break;
        case 2: total = &out->galileo_total; used_count = &out->galileo_used; break;
        case 3: total = &out->beidou_total; used_count = &out->beidou_used; break;
        case 5: total = &out->qzss_total; used_count = &out->qzss_used; break;
        case 6: total = &out->glonass_total; used_count = &out->glonass_used; break;
        default: break;
        }
        if (used) {
            if (out->used < UINT8_MAX) ++out->used;
            if (used_count && *used_count < UINT8_MAX) ++*used_count;
        }
        if (total && *total < UINT8_MAX) ++*total;
    }
    return true;
}
