"""Host-test the actual reporting expression and counted RTC write function."""
import pathlib
import re
import subprocess
import tempfile

source = pathlib.Path("main.c").read_text()
status = source.split("    d.calibration_observed_s =", 1)[1].split("    d.k =", 1)[0]
status = "    d.calibration_observed_s =" + status
write = source.split("static bool rtc_set_time(", 1)[1].split("static void gps_uart_apply_baud", 1)[0]
write = "static bool rtc_set_time(" + write
defaults = "\n".join(re.findall(r"#ifndef RTC_AGING_(?:ENABLE|WINDOW_SEC|MIN_SAMPLES)\n.*?\n#endif", source))
harness = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "rtc_holdover.h"
/* DEFAULTS */
static bool rtc_conversion_failed, rtc_conversion_pending, pps_time_valid=true, pll_valid=true;
static gnss_recovery_t gnss_recovery = {.locked=true};
static rtc_phase_filter_t rtc_phase_filter = {.qualified=true};
static rtc_frequency_filter_t rtc_frequency_filter = {.valid=true, .count=6, .uncertainty_ppm=.01};
static uint64_t rtc_last_gnss_observation_ns, now_ns=30000000000000ULL;
static int64_t last_aging_adjust_time;
static int64_t get_absolute_time(void) { return (int64_t)(now_ns/1000); }
static int64_t absolute_time_diff_us(int64_t a,int64_t b) { return b-a; }
static const char *status(void) {
    struct { uint64_t calibration_observed_s; const char *calibration_status; } d;
    /* STATUS */
    return d.calibration_status;
}
#define RTC_I2C_ID 0
#define RTC_ADDR 0x68
static int rtc_i2c_mutex, stats_mutex, write_result=8;
static uint32_t rtc_calendar_write_count;
static uint8_t dec_to_bcd(uint8_t n) { return (uint8_t)((n/10)*16+n%10); }
static void mutex_enter_blocking(int *m) { assert(*m==0);*m=1; }
static void mutex_exit(int *m) { assert(*m==1);*m=0; }
static int rtc_i2c_write(int addr,const uint8_t *buf,int n,bool nostop) {
    (void)addr;(void)nostop;assert(rtc_i2c_mutex==1&&stats_mutex==0);
    assert(n==8&&buf[0]==0&&buf[1]==0x30&&buf[2]==0x25&&buf[3]==0x17);
    return write_result;
}
/* WRITE */
int main(void) {
    rtc_last_gnss_observation_ns=now_ns;
    rtc_frequency_filter.duration_ns=(uint64_t)RTC_AGING_WINDOW_SEC*1000000000ULL;
#if RTC_AGING_ENABLE
    assert(!strcmp(status(),"Qualified"));
    --rtc_frequency_filter.duration_ns; assert(!strcmp(status(),"Learning"));
    ++rtc_frequency_filter.duration_ns;
    rtc_frequency_filter.uncertainty_ppm=.06;assert(!strcmp(status(),"Learning / uncertainty"));
    rtc_frequency_filter.uncertainty_ppm=.01;
    last_aging_adjust_time=get_absolute_time();assert(!strcmp(status(),"Waiting / trim interval"));
    last_aging_adjust_time=0;
    rtc_last_gnss_observation_ns=now_ns-1500000000ULL;assert(!strcmp(status(),"Paused / no GPS reference"));
    rtc_last_gnss_observation_ns=now_ns;
    gnss_recovery.locked=false;assert(!strcmp(status(),"Paused / no GPS reference"));gnss_recovery.locked=true;
    rtc_phase_filter.qualified=false;assert(!strcmp(status(),"Learning"));rtc_phase_filter.qualified=true;
    rtc_conversion_pending=true;assert(!strcmp(status(),"Applying trim"));rtc_conversion_pending=false;
    rtc_conversion_failed=true;assert(!strcmp(status(),"Paused / conversion fault"));rtc_conversion_failed=false;
#else
    assert(!strcmp(status(),"Disabled"));
#endif
    assert(rtc_set_time(17,25,30,3,8,9,2026));assert(rtc_calendar_write_count==1);
    write_result=7;assert(!rtc_set_time(17,25,30,3,8,9,2026));assert(rtc_calendar_write_count==1);
    write_result=-1;assert(!rtc_set_time(17,25,30,3,8,9,2026));assert(rtc_calendar_write_count==1);
    rtc_calendar_write_count=UINT32_MAX;write_result=8;
    assert(rtc_set_time(17,25,30,3,8,9,2026));assert(rtc_calendar_write_count==UINT32_MAX);
    puts("RTC calibration reporting and calendar-write counter passed");
}
'''.replace("/* DEFAULTS */", defaults).replace("/* STATUS */", status).replace("/* WRITE */", write)
with tempfile.TemporaryDirectory(prefix="pico-rtc-report-") as folder:
    for enabled in (0, 1):
        binary = str(pathlib.Path(folder) / "test")
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I.",
                        f"-DRTC_AGING_ENABLE={enabled}", "-x", "c", "-", "-o", binary],
                       input=harness, text=True, check=True)
        subprocess.run([binary], check=True)
