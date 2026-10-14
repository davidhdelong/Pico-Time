#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "ubx.h"

static size_t make_frame(uint8_t cls, uint8_t id, const uint8_t *payload,
                         uint16_t len, uint8_t *out) {
    uint8_t a = 0, b = 0;
    size_t n = 0;
    out[n++] = 0xB5; out[n++] = 0x62;
    out[n++] = cls; out[n++] = id;
    out[n++] = (uint8_t)len; out[n++] = (uint8_t)(len >> 8);
    for (uint16_t i = 0; i < len; ++i) {
        out[n++] = payload[i];
    }
    for (size_t i = 2; i < n; ++i) {
        a = (uint8_t)(a + out[i]);
        b = (uint8_t)(b + a);
    }
    out[n++] = a; out[n++] = b;
    return n;
}

static void put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

int main(void) {
    ubx_parser_t parser;
    ubx_frame_t frame;
    uint8_t payload[92] = {0};
    uint8_t wire[128];
    ubx_nav_pvt_t pvt;
    ubx_tim_tp_t tim;
    ubx_nav_sat_summary_t sat;

    put_u16(payload + 4, 2026);
    payload[6] = 8; payload[7] = 30; payload[8] = 12;
    payload[9] = 34; payload[10] = 56; payload[11] = 7;
    put_u32(payload + 12, 1234);
    payload[20] = 3; payload[21] = 1; payload[23] = 15;
    put_u32(payload + 24, (uint32_t)(-937938460));
    put_u32(payload + 28, (uint32_t)300809900);
    put_u32(payload + 36, 12500);
    put_u32(payload + 60, 2500);
    put_u16(payload + 76, 80);

    ubx_parser_init(&parser);
    size_t len = make_frame(0x01, 0x07, payload, sizeof(payload), wire);
    bool complete = false;
    for (size_t i = 0; i < len; ++i) {
        if (ubx_parser_feed(&parser, wire[i], 1000 + i, &frame)) complete = true;
    }
    assert(complete);
    assert(ubx_decode_nav_pvt(&frame, &pvt));
    assert(pvt.year == 2026 && pvt.month == 8 && pvt.sec == 56);
    assert(pvt.fix_type == 3 && pvt.satellites == 15);
    assert(pvt.latitude_1e7 == 300809900);

    uint8_t tim_payload[16] = {0};
    put_u32(tim_payload, 123456);
    put_u32(tim_payload + 4, 42);
    put_u32(tim_payload + 8, (uint32_t)-17);
    put_u16(tim_payload + 12, 2345);
    tim_payload[14] = 1; tim_payload[15] = 2;
    len = make_frame(0x0D, 0x01, tim_payload, sizeof(tim_payload), wire);
    ubx_parser_init(&parser);
    complete = false;
    for (size_t i = 0; i < len; ++i) {
        if (ubx_parser_feed(&parser, wire[i], 2000 + i, &frame)) complete = true;
    }
    assert(complete && ubx_decode_tim_tp(&frame, &tim));
    assert(tim.tow_ms == 123456 && tim.tow_sub_ms == 42);
    assert(tim.q_err == -17 && tim.week == 2345);

    uint8_t sat_payload[32] = {0};
    sat_payload[5] = 2;
    sat_payload[8] = 0; sat_payload[9] = 7; sat_payload[10] = 42; sat_payload[16] = 1 << 3;
    sat_payload[20] = 2; sat_payload[21] = 11; sat_payload[22] = 35;
    len = make_frame(0x01, 0x35, sat_payload, sizeof(sat_payload), wire);
    ubx_parser_init(&parser);
    complete = false;
    for (size_t i = 0; i < len; ++i) {
        if (ubx_parser_feed(&parser, wire[i], 2500 + i, &frame)) complete = true;
    }
    assert(complete && ubx_decode_nav_sat(&frame, &sat));
    assert(sat.total == 2 && sat.used == 1);
    assert(sat.gps_total == 1 && sat.gps_used == 1 && sat.galileo_total == 1);

    wire[10] ^= 1;
    ubx_parser_init(&parser);
    complete = false;
    for (size_t i = 0; i < len; ++i) {
        if (ubx_parser_feed(&parser, wire[i], 3000 + i, &frame)) complete = true;
    }
    assert(!complete);

    // A declared payload larger than the parser buffer must be rejected and
    // must not produce a frame.
    uint8_t oversized[] = {0xB5, 0x62, 0x01, 0x07, 0x01, 0x04};
    ubx_parser_init(&parser);
    complete = false;
    for (size_t i = 0; i < sizeof(oversized); ++i) {
        if (ubx_parser_feed(&parser, oversized[i], 4000 + i, &frame)) complete = true;
    }
    assert(!complete);

    // A truncated NAV-SAT payload must be rejected before any entry is read.
    uint8_t short_sat[8] = {0};
    short_sat[5] = 1;
    len = make_frame(0x01, 0x35, short_sat, sizeof(short_sat), wire);
    ubx_parser_init(&parser);
    complete = false;
    for (size_t i = 0; i < len; ++i) {
        if (ubx_parser_feed(&parser, wire[i], 5000 + i, &frame)) complete = true;
    }
    assert(complete && !ubx_decode_nav_sat(&frame, &sat));
    return 0;
}
