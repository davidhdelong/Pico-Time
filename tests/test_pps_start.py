"""Check the shipped counter-enable bracket and failure conditions."""
import pathlib
import subprocess
import tempfile

source = pathlib.Path("main.c").read_text()
function = "static void pps_capture_start(void) {" + source.split(
    "static void pps_capture_start(void) {", 1)[1].split("\nstatic void pps_pio_init", 1)[0]
harness = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
typedef unsigned uint;
static int stage;
static uint64_t pps_capture_start_ns, before, after;
static bool pps_capture_start_valid;
static void *pio0=(void*)1;
static void pio_sm_clkdiv_restart(void *p,uint sm) {
    assert(p==pio0&&!sm&&stage==0);stage=1;
}
static uint32_t save_and_disable_interrupts(void) {
    assert(stage==1);stage=2;return 7;
}
static uint64_t hires_time_ns(void) {
    if(stage==2){stage=3;return before;}
    assert(stage==5);stage=6;return after;
}
static void pio_sm_set_enabled(void *p,uint sm,bool en) {
    assert(p==pio0&&!sm&&en&&stage==3);stage=4;
}
static void __dsb(void) {assert(stage==4);stage=5;}
static void restore_interrupts(uint32_t save) {
    assert(save==7&&stage==6);stage=0;
}
/* FUNCTION */
int main(void) {
    before=100000;after=100080;pps_capture_start();
    assert(pps_capture_start_valid&&pps_capture_start_ns==100040);
    before=200000;after=199999;pps_capture_start();
    assert(!pps_capture_start_valid);
    before=300000;after=310001;pps_capture_start();
    assert(!pps_capture_start_valid);
    return 0;
}
'''.replace("/* FUNCTION */", function)
with tempfile.TemporaryDirectory(prefix="pps-start-") as folder:
    binary = str(pathlib.Path(folder) / "test")
    for flags in ([], ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]):
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        *flags, "-x", "c", "-", "-o", binary],
                       input=harness, text=True, check=True)
        subprocess.run([binary], check=True)
print("PPS counter start: interrupt masking, write completion and invalid brackets passed")
