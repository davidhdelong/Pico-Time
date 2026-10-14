#include "tm1637.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdio.h>

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
#ifndef DEBUG_LOG
#define DEBUG_LOG 1
#endif
#if DEBUG_LOG
#define DBG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DBG_PRINTF(...) do { } while (0)
#endif

#if TM1637_ENABLE
static const uint8_t segments[10] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};
static uint32_t errors;
static uint64_t last_update_us;
static bool pps_colon_on = false;

static void dio_low(void) { gpio_put(TM1637_DIO_PIN, 0); gpio_set_dir(TM1637_DIO_PIN, GPIO_OUT); }
static void dio_release(void) { gpio_set_dir(TM1637_DIO_PIN, GPIO_IN); gpio_pull_up(TM1637_DIO_PIN); }
static bool write_byte(uint8_t value) {
    for (int bit = 0; bit < 8; ++bit) {
        gpio_put(TM1637_CLK_PIN, 0);
        if ((value >> bit) & 1u) dio_release(); else dio_low();
        sleep_us(5); gpio_put(TM1637_CLK_PIN, 1); sleep_us(5);
    }
    gpio_put(TM1637_CLK_PIN, 0); dio_release(); sleep_us(5);
    gpio_put(TM1637_CLK_PIN, 1); sleep_us(5);
    bool ack = !gpio_get(TM1637_DIO_PIN);
    gpio_put(TM1637_CLK_PIN, 0);
    return ack;
}
static void start(void) { dio_release(); gpio_put(TM1637_CLK_PIN, 1); sleep_us(5); dio_low(); sleep_us(5); gpio_put(TM1637_CLK_PIN, 0); }
static void stop(void) { dio_low(); gpio_put(TM1637_CLK_PIN, 1); sleep_us(5); dio_release(); sleep_us(5); }
static bool write_time(int hour, int min, bool colon_on) {
    uint8_t digits[4] = {segments[(hour / 10) % 10], (uint8_t)(segments[hour % 10] | (colon_on ? 0x80 : 0x00)), segments[(min / 10) % 10], segments[min % 10]};
    bool ok = true;
    start(); ok &= write_byte(0x40); stop();
    start(); ok &= write_byte(0xc0); for (int i = 0; i < 4; ++i) ok &= write_byte(digits[i]); stop();
    start(); ok &= write_byte((uint8_t)(0x88 | (TM1637_BRIGHTNESS & 7))); stop();
    if (!ok) __atomic_add_fetch(&errors, 1, __ATOMIC_RELAXED);
    return ok;
}
void tm1637_init(void) {
    gpio_init(TM1637_CLK_PIN); gpio_set_dir(TM1637_CLK_PIN, GPIO_OUT); gpio_put(TM1637_CLK_PIN, 1);
    gpio_init(TM1637_DIO_PIN); dio_release(); (void)write_time(0, 0, false);
    DBG_PRINTF("TM1637: UTC display on CLK=GP%d DIO=GP%d\n", TM1637_CLK_PIN, TM1637_DIO_PIN);
}
void tm1637_update(uint64_t now_us, uint32_t ntp_seconds, bool time_valid, bool pps_edge, bool pps_phase, int64_t pps_time_us, bool pps_locked, bool rtc_fallback) {
    (void)pps_time_us;
    if (pps_edge && pps_locked) pps_colon_on = pps_phase;
    if (!pps_edge && now_us - last_update_us < 100000ULL) return;
    last_update_us = now_us;
    if (time_valid) {
        uint32_t utc = ntp_seconds - 2208988800UL;
        bool colon_on = pps_locked ? pps_colon_on : rtc_fallback;
        (void)write_time((int)((utc % 86400U) / 3600U), (int)((utc % 3600U) / 60U), colon_on);
    } else {
        (void)write_time(0, 0, false);
    }
}
uint32_t tm1637_error_count(void) { return __atomic_load_n(&errors, __ATOMIC_RELAXED); }
#else
void tm1637_init(void) { }
void tm1637_update(uint64_t now_us, uint32_t ntp_seconds, bool time_valid, bool pps_edge, bool pps_phase, int64_t pps_time_us, bool pps_locked, bool rtc_fallback) { (void)now_us; (void)ntp_seconds; (void)time_valid; (void)pps_edge; (void)pps_phase; (void)pps_time_us; (void)pps_locked; (void)rtc_fallback; }
uint32_t tm1637_error_count(void) { return 0; }
#endif
