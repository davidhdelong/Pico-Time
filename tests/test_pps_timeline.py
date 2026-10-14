"""Exercise the firmware's actual capture interval acceptance block."""
import pathlib
import subprocess
import tempfile

source = pathlib.Path("main.c").read_text()
block = "                uint64_t pps_ns = now_ns;" + source.split(
    "                uint64_t pps_ns = now_ns;", 1)[1].split("                last_pps_time_ns = pps_ns;", 1)[0]
# The raw timeline commit must remain outside the calendar-availability branch.
assert source.index("prev_pps_time_ns = pps_ns;", source.index(block)) > source.index(
    "/* The raw capture timeline advances", source.index(block))
harness = r'''
#include <assert.h>
#include <stdio.h>
#include "pps_capture_timing.h"
#define PPS_CAPTURE_COUNTER_HZ 50000000u
#define PPS_PLL_MIN_US 995000ULL
#define PPS_PLL_MAX_US 1005000ULL
static bool prev_pps_capture_valid, pps_capture_fault_pending;
static uint64_t prev_pps_time_ns, pps_capture_remainder;
static uint32_t prev_pps_capture_ticks;
static int stats_mutex;
static void mutex_exit(int *m) { assert(*m==1);*m=0; }
static bool step(uint32_t capture_ticks, bool capture_valid, uint64_t now_ns) {
    for (unsigned i=0;i<1;++i) {
        stats_mutex=1;
        /* BLOCK */
        prev_pps_time_ns=pps_ns;
        prev_pps_capture_ticks=capture_ticks;
        prev_pps_capture_valid=capture_valid;
        mutex_exit(&stats_mutex);
        return true;
    }
    return false;
}
int main(void) {
    uint32_t raw=3000000000u;
    pps_capture_remainder=17;
    assert(step(raw,true,10000000000ULL));assert(!pps_capture_remainder);
    for(unsigned i=1;i<=60;++i) {
        raw-=49999997u;
        assert(step(raw,true,999999999999ULL));
        assert(prev_pps_time_ns==10000000000ULL+i*1000000000ULL);
    }
    uint64_t before=prev_pps_time_ns;
    assert(!step(raw-99999997u,true,0)); /* missing edge */
    assert(pps_capture_fault_pending&&prev_pps_time_ns==before&&!stats_mutex);
    pps_capture_fault_pending=false;
    assert(!step(raw-100u,true,0)); /* extra/glitch edge */
    assert(pps_capture_fault_pending&&prev_pps_time_ns==before&&!stats_mutex);
    prev_pps_capture_valid=false;pps_capture_remainder=7;
    assert(step(123,true,90000000000ULL));
    assert(prev_pps_time_ns==90000000000ULL&&!pps_capture_remainder);
    puts("Actual PPS timeline: corrected intervals, missing/extra edge rejection and fresh anchor passed");
}
'''.replace("/* BLOCK */", block)
with tempfile.TemporaryDirectory(prefix="pps-timeline-") as folder:
    binary = str(pathlib.Path(folder) / "test")
    for flags in ([], ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]):
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I.",
                        *flags, "-x", "c", "-", "-o", binary], input=harness, text=True, check=True)
        subprocess.run([binary], check=True)
