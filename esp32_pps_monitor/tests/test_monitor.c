#include "pps_monitor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static uint32_t ticks(uint64_t ns) { return (uint32_t)(ns * 80 / 1000); }
static void edge(pps_monitor_t *m, unsigned channel, uint64_t ns, bool rise) {
    pps_edge_t e = {.ticks=ticks(ns),.observed_us=ns/1000,.input=channel,.rising=rise};
    pps_monitor_edge(m,&e);
}
static void train(pps_monitor_t *m, int64_t offset_ns, bool reverse, unsigned seconds) {
    for (unsigned s=1;s<=seconds;++s) {
        uint64_t a=(uint64_t)s*1000000000ULL;
        uint64_t b=(uint64_t)((int64_t)a+offset_ns);
        if(reverse) {edge(m,1,b,true);edge(m,0,a,true);}
        else {edge(m,0,a,true);edge(m,1,b,true);}
        edge(m,0,a+100000000ULL,false);edge(m,1,b+100000000ULL,false);
    }
}
static void near(double a,double b,double tol) { assert(fabs(a-b)<tol); }
int main(void) {
    pps_monitor_t m;pps_summary_t s;
    for(int sign=-1;sign<=1;sign+=2) for(unsigned reverse=0;reverse<2;++reverse) {
        pps_monitor_init(&m,80000000);train(&m,sign*1250,reverse,125);
        pps_monitor_summary(&m,125001000,&s);
        assert(s.valid&&m.count==123&&s.drift_valid);
        near(s.latest_us,sign*1.25,.00001);near(s.mean_us,sign*1.25,.00001);
        near(s.stddev_us,0,.00001);near(s.rms_us,1.25,.00001);near(s.drift_ppm,0,.00001);
        near(m.input[0].period_us,1000000,.00001);near(m.input[0].width_us,100000,.00001);
        pps_monitor_summary(&m,130000000,&s);assert(!s.valid);
    }
    puts("Both offset signs, reversed ISR order, multiple counter wraps, period/width and stale samples passed");
    pps_monitor_init(&m,80000000);train(&m,0,false,4);
    uint64_t before=m.pairs;
    edge(&m,0,4100000000ULL,true);assert(m.input[0].glitches==1&&m.pairs==before);
    edge(&m,0,5000000000ULL,true);edge(&m,1,5000000000ULL,true);
    assert(m.pairs==before+1);
    edge(&m,0,7000000000ULL,true);edge(&m,1,7000000000ULL,true);
    assert(m.input[0].missed==1&&m.input[1].missed==1);
    pps_monitor_summary(&m,7000000,&s);assert(!s.valid);
    edge(&m,0,8000000000ULL,true);edge(&m,1,8000000000ULL,true);
    edge(&m,0,9000000000ULL,true);edge(&m,1,9000000000ULL,true);
    pps_monitor_summary(&m,9000000,&s);assert(s.valid);
    pps_monitor_invalidate(&m);pps_monitor_summary(&m,9000000,&s);assert(!s.valid);
    puts("Glitch rejection, missed pulses, two-interval requalification and queue-drop invalidation passed");
    pps_monitor_init(&m,80000000);
    for(unsigned j=1;j<=400;++j) {
        uint64_t a=(uint64_t)j*1000000000ULL;
        edge(&m,0,a,true);edge(&m,1,a+j*1000ULL,true);
    }
    pps_monitor_summary(&m,400001000,&s);
    assert(s.valid&&m.count==PPS_HISTORY&&m.pairs==398);
    near(s.drift_ppm,1,.0001);near(s.latest_us,400,.0001);
    near(s.median_us,250.5,.0001);near(s.mean_us,250.5,.0001);
    near(s.min_us,101,.0001);near(s.max_us,400,.0001);
    assert(pps_monitor_pair(&m,0)->offset_us==101);
    assert(!pps_monitor_pair(&m,PPS_HISTORY));
    pps_monitor_clear_stats(&m);pps_monitor_summary(&m,400001000,&s);
    assert(!s.valid&&!m.count&&!m.pairs&&m.input[0].good_intervals==2);
    puts("Rolling window overwrite, statistics, 1 ppm drift and statistics reset passed");
    pps_monitor_init(&m,80000000);train(&m,450000000,false,5);
    assert(!m.count&&m.unpaired>0);
    puts("Out-of-gate edge pairing rejected");
    /* A rejected extra pulse must not overwrite a valid high width. */
    pps_monitor_init(&m,80000000);train(&m,0,false,15);
    near(m.input[0].width_us,100000,.00001);
    edge(&m,0,15200000000ULL,true);edge(&m,0,15201000000ULL,false);
    near(m.input[0].width_us,100000,.00001);
    edge(&m,1,15300000000ULL,false); /* repeated falling edge */
    near(m.input[1].width_us,100000,.00001);
    uint64_t old_segment=m.segment;
    pps_monitor_poll(&m,45000000);
    pps_monitor_summary(&m,45000000,&s);
    assert(!s.valid&&!s.drift_valid&&m.count==13);
    assert(m.segment==old_segment+1);
    assert(!m.input[0].have_width&&!m.input[0].have_period);
    assert(pps_input_missed(&m.input[0],45000000)==29);
    pps_monitor_poll(&m,45999999);assert(m.segment==old_segment+1);
    assert(pps_input_missed(&m.input[0],45999999)==30);
    for(unsigned j=46;j<=60;++j) {
        edge(&m,0,(uint64_t)j*1000000000ULL,true);
        edge(&m,1,(uint64_t)j*1000000000ULL+100000,true);
    }
    assert(m.input[0].missed==30);
    assert(pps_input_missed(&m.input[0],60000200)==30); /* no double count */
    pps_monitor_summary(&m,60000200,&s);
    assert(s.valid&&s.drift_valid&&s.drift_samples==13);
    near(s.drift_ppm,0,.000001); /* offset step across outage is not drift */
    assert(pps_monitor_pair(&m,12)->segment!=pps_monitor_pair(&m,13)->segment);
    puts("Glitch-safe width, live outage counts, segment boundaries and isolated recovery drift passed");
    /* A glitch before the genuine falling edge makes width ambiguous. */
    pps_monitor_init(&m,80000000);
    edge(&m,0,1000000000ULL,true);edge(&m,0,1010000000ULL,true);
    edge(&m,0,1011000000ULL,false);edge(&m,0,1100000000ULL,false);
    assert(!m.input[0].have_width);

}
