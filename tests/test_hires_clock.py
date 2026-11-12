"""Exercise the shipped high-resolution clock with interrupt/clock mocks."""
import pathlib
import subprocess
import tempfile

source = pathlib.Path("main.c").read_text()
start = "static uint64_t hires_time_ns(void) {"
function = start + source.split(start, 1)[1].split("static void gps_line_reset", 1)[0]
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
typedef unsigned uint;
#define PICO_RP2350 1
static bool hires_time_enabled=true, masked;
static uint core;
static uint32_t hires_cycles_per_sec=150000000, hires_reanchor_cycles=75000000;
static struct {uint64_t anchor_us,last_ns;uint32_t anchor_cycles;} hires_clock_state[2];
static struct {uint32_t dwt_cyccnt;} device,*m33_hw=&device;
static uint64_t timer_us;
static uint get_core_num(void) {return core;}
static uint32_t save_and_disable_interrupts(void) {uint32_t old=masked;masked=true;return old;}
static void restore_interrupts(uint32_t saved) {assert(masked);masked=saved;}
static uint64_t time_us_64(void) {
    /* Old reanchor code reached this point with interrupts enabled: a UART
     * ISR could resample and publish an anchor before the caller overwrote it. */
    assert(masked);
    return timer_us;
}
/* FUNCTION */
int main(void) {
    for(core=0;core<2;++core) {
        hires_clock_state[core].anchor_us=1000000;
        device.dwt_cyccnt=74999999;
        uint64_t previous=hires_time_ns();
        assert(previous==1499999993ULL&&!masked);
        device.dwt_cyccnt=75000000;timer_us=1499999;
        assert(hires_time_ns()==previous&&!masked); /* coarse quantization */
        device.dwt_cyccnt+=150;assert(hires_time_ns()>previous);
        device.dwt_cyccnt+=75000000;timer_us=2000000;
        assert(hires_time_ns()==2000000000ULL&&!masked);
        /* An ISR runs after publication; its counter progression must not
         * disappear from subsequent clock readings. */
        device.dwt_cyccnt+=1500;
        assert(hires_time_ns()==2000010000ULL);
        masked=true;device.dwt_cyccnt+=75000000;timer_us=2500010;
        assert(hires_time_ns()==2500010000ULL&&masked);masked=false;
        /* A normal DWT wrap remains a positive modular delta. */
        hires_clock_state[core].anchor_cycles=UINT32_MAX-74;
        device.dwt_cyccnt=75;
        assert(hires_time_ns()==2500011000ULL);
    }
    puts("Hires clock: atomic reanchor, quantization monotonicity, ISR progression, both cores and DWT wrap passed");
}
'''.replace("/* FUNCTION */", function)
with tempfile.TemporaryDirectory(prefix="hires-regression-") as folder:
    binary = str(pathlib.Path(folder)/"test")
    for flags in ([], ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]):
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", *flags,
                        "-x", "c", "-", "-o", binary], input=harness, text=True, check=True)
        subprocess.run([binary], check=True)
