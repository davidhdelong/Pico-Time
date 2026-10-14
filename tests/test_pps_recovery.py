"""Compile the actual capture recovery function against checked hardware mocks."""
import pathlib
import subprocess
import tempfile

source = pathlib.Path("main.c").read_text()
function = "static void pps_capture_service(void) {" + source.split(
    "static void pps_capture_service(void) {", 1)[1].split("static void rtc_sqw_init", 1)[0]
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
typedef unsigned int uint;
#define PIO_FDEBUG_RXSTALL_LSB 0
#define PIO0_IRQ_0 0
#define PPS_DMA_RING_SIZE 16
static struct { uint32_t fdebug; } device, *pio0=&device;
static bool pps_capture_fault_pending, gps_pps_capture_valid, gps_pps_seen, gps_pps_tick;
static bool prev_pps_capture_valid, pps_time_valid, pll_valid, accuracy_mode;
static uint64_t now, pps_capture_last_poll_ns, pps_capture_remainder, prev_pps_time_ns, trusted_pps_ns;
static uint32_t pps_capture_faults, pps_dma_overruns;
static uint pps_program_offset=7;
static struct { bool locked; uint good_pulses; } gnss_recovery;
static uint64_t rtc_frequency_filter[8];
static int stats_mutex, stage, resets;
static uint64_t hires_time_ns(void) { return now; }
static void mutex_enter_blocking(int *m) { assert(!*m); *m=1; }
static void mutex_exit(int *m) { assert(*m); *m=0; }
static void pio_sm_set_enabled(void *p, uint s, bool enabled) {
    assert(p==pio0 && s==0);
    if (!enabled) { assert(stage==0); stage=1; }
    else { assert(stage==6); assert(pio0->fdebug==1); pio0->fdebug=0; stage=0; ++resets; }
}
static void pio_sm_clear_fifos(void *p,uint s) { (void)p;(void)s;assert(stage==1);stage=2; }
static void pio_sm_restart(void *p,uint s) { (void)p;(void)s;assert(stage==2);stage=3; }
static void pio_sm_clkdiv_restart(void *p,uint s) { (void)p;(void)s;assert(stage==3);stage=4; }
static uint pio_encode_jmp(uint offset) { assert(offset==7);return offset; }
static void pio_sm_exec(void *p,uint s,uint op) { (void)p;(void)s;assert(op==7&&stage==4);stage=5; }
static void pio_interrupt_clear(void *p,uint s) { (void)p;(void)s;assert(stage==5);stage=6; }
#if PPS_CAPTURE_DMA
static struct { struct { uintptr_t write_addr; } ch[1]; } hw, *dma_hw=&hw;
static int pps_dma_channel=0;
static uint32_t pps_dma_ring[16], pps_dma_consumer;
static void dma_channel_abort(uint c) { assert(c==0&&stage==1); }
static void dma_channel_set_write_addr(uint c,void *p,bool trigger) {
    assert(c==0&&p==pps_dma_ring&&!trigger&&stage==6); hw.ch[0].write_addr=(uintptr_t)p;
}
static uint dma_encode_endless_transfer_count(void) { return 123; }
static void dma_channel_set_trans_count(uint c,uint n,bool trigger) {
    assert(c==0&&n==123&&trigger&&stage==6);
    assert(!pps_time_valid&&!gnss_recovery.locked&&!gps_pps_tick);
}
#else
static void irq_set_enabled(uint irq,bool enabled) { assert(irq==0);assert(stage==(enabled?6:0)); }
#endif
/* FUNCTION */
static void trusted(void) {
    trusted_pps_ns=now;
    gps_pps_capture_valid=gps_pps_seen=gps_pps_tick=true;
    prev_pps_capture_valid=pps_time_valid=pll_valid=accuracy_mode=true;
    gnss_recovery.locked=true;gnss_recovery.good_pulses=4;
    pps_capture_remainder=9;prev_pps_time_ns=123;
    memset(rtc_frequency_filter,1,sizeof(rtc_frequency_filter));
}
static void recovered(void) {
    assert(!trusted_pps_ns);
    assert(!gps_pps_capture_valid&&!gps_pps_seen&&!gps_pps_tick);
    assert(!prev_pps_capture_valid&&!pps_time_valid&&!pll_valid&&!accuracy_mode);
    assert(!gnss_recovery.locked&&!gnss_recovery.good_pulses);
    assert(!pps_capture_remainder&&!prev_pps_time_ns&&!pps_capture_fault_pending);
    for(uint i=0;i<8;++i) assert(!rtc_frequency_filter[i]);
    assert(!stage&&!stats_mutex);
}
int main(void) {
#if PPS_CAPTURE_DMA
    hw.ch[0].write_addr=(uintptr_t)pps_dma_ring;
#endif
    now=10000000000ULL;trusted();pps_capture_service();assert(!resets&&pps_time_valid);
    now-=10;pps_capture_service();assert(!resets); /* small clock reanchor */
    pio0->fdebug=1;pps_capture_service();recovered();assert(resets==1&&pps_capture_faults==1);
    trusted();pps_capture_fault_pending=true;pps_capture_service();recovered();assert(resets==2);
    trusted();now+=16000000000ULL;pps_capture_service();recovered();assert(resets==3);
#if PPS_CAPTURE_DMA
    trusted();hw.ch[0].write_addr=(uintptr_t)(pps_dma_ring+1);
    pps_capture_service();assert(resets==3&&pps_time_valid);
    hw.ch[0].write_addr=(uintptr_t)(pps_dma_ring+2);
    pps_capture_service();recovered();assert(resets==4&&pps_dma_overruns==1);
    pps_dma_overruns=UINT32_MAX;hw.ch[0].write_addr=(uintptr_t)(pps_dma_ring+3);
#endif
    pps_capture_faults=UINT32_MAX;pps_capture_fault_pending=true;
    pps_capture_service();recovered();assert(pps_capture_faults==UINT32_MAX);
#if PPS_CAPTURE_DMA
    assert(pps_dma_overruns==UINT32_MAX);
#endif
    puts("Actual PPS recovery: FIFO stall, pending fault, poll gap, backlog and saturation passed");
}
'''.replace("/* FUNCTION */", function)
with tempfile.TemporaryDirectory(prefix="pps-recovery-") as folder:
    for dma in (0, 1):
        for sanitizer in (False, True):
            binary = str(pathlib.Path(folder) / "test")
            flags = ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"] if sanitizer else []
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            f"-DPPS_CAPTURE_DMA={dma}", *flags, "-x", "c", "-", "-o", binary],
                           input=harness, text=True, check=True)
            subprocess.run([binary], check=True)
