#include "ntp_uncertainty.h"
#include "rtc_holdover.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void encoding(void) {
    const uint64_t values[] = {0, 1000, 10000, 100000, 1000000, 10000000,
        100000000, 190292000, 1000000000, 10000000000ULL,
        65535000000000ULL, 65535999984741ULL};
    for (unsigned i = 0; i < sizeof(values)/sizeof(values[0]); ++i) {
        uint32_t raw = ntp_dispersion_from_ns(values[i]);
        uint64_t decoded = ntp_dispersion_to_ns(raw);
        assert(decoded >= values[i]);
        assert(decoded - values[i] <= 15259);
        uint8_t wire[4]; ntp_write_u32(wire, raw);
        assert((((uint32_t)wire[0] << 24) | ((uint32_t)wire[1] << 16) |
            ((uint32_t)wire[2] << 8) | wire[3]) == raw);
    }
    assert(ntp_dispersion_from_ns(1000000000) == 65536);
    assert(ntp_dispersion_from_ns(0) == 0);
    assert(ntp_dispersion_from_ns(1000) == 1);
    assert(ntp_dispersion_from_ns(10000) == 1);
    assert(ntp_dispersion_from_ns(100000) == 7);
    assert(ntp_dispersion_from_ns(1000000) == 66);
    assert(ntp_dispersion_from_ns(10000000) == 656);
    assert(ntp_dispersion_from_ns(100000000) == 6554);
    assert(ntp_dispersion_from_ns(190292000) == 12471);
    assert(ntp_dispersion_from_ns(1) == 1);
    assert(ntp_dispersion_from_ns(65536000000000ULL) == UINT32_MAX);
    assert(ntp_dispersion_from_ns(UINT64_MAX) == UINT32_MAX);
    assert(ntp_abs_ns(INT64_MIN) == (1ULL << 63));
    assert(ntp_saturating_add(UINT64_MAX, 1) == UINT64_MAX);
}
static void placement(void) {
    ntp_edge_bracket_t b[2]={{1000000000000ULL,1000000020000ULL},{999000000000ULL,999000020000ULL}};
    assert(ntp_placement_bound_ns(1000000010000ULL,b)==10000);
    ntp_placement_t p = ntp_placement_details(1000000010000ULL,b);
    assert(p.valid && p.window_ns == 20000 && p.offset_ns == 0 && p.bound_ns == 10000);
    p = ntp_placement_details(1000000030000ULL,b);
    assert(p.valid && p.offset_ns == 20000 && p.bound_ns == 30000);
    p = ntp_placement_details(999999990000ULL,b);
    assert(p.valid && p.offset_ns == -20000);
    p = ntp_placement_details(999000005000ULL,b);
    assert(p.valid && p.offset_ns == -5000 && p.window_ns == 20000);
    assert(ntp_placement_bound_ns(1000000030000ULL,b)==30000);
    assert(ntp_placement_bound_ns(999000005000ULL,b)==15000); /* previous trusted PPS coast */
    assert(ntp_placement_bound_ns(999999990000ULL,b)==30000);
    assert(ntp_placement_bound_ns(0,b)==UINT64_MAX);
    b[0]=(ntp_edge_bracket_t){0}; b[1]=(ntp_edge_bracket_t){0};
    assert(ntp_placement_bound_ns(1000000010000ULL,b)==UINT64_MAX);
    p = ntp_placement_details(1000000010000ULL,b);
    assert(!p.valid && p.window_ns == 0 && p.offset_ns == 0);
    b[0]=(ntp_edge_bracket_t){100,99};
    assert(ntp_placement_bound_ns(100,b)==UINT64_MAX);
    b[0]=(ntp_edge_bracket_t){100,600000100};
    assert(ntp_placement_bound_ns(100,b)==UINT64_MAX);
    b[0]=(ntp_edge_bracket_t){UINT64_MAX-10000,UINT64_MAX};
    assert(ntp_placement_bound_ns(UINT64_MAX-5000,b)==5000);
    p = ntp_placement_details(UINT64_MAX-5000,b);
    assert(p.valid && p.offset_ns == 0 && p.window_ns == 10000);
    b[0]=(ntp_edge_bracket_t){1000000000ULL,1000000005ULL};
    p = ntp_placement_details(1000000002ULL,b);
    assert(p.valid && p.window_ns == 5 && p.offset_ns == 0 && p.bound_ns == 3);
}
static uint32_t read32(const uint8_t *p) {
    return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];
}
static void packets(void) {
    for (unsigned rx=0;rx<=4;++rx) for(unsigned tx=0;tx<=4;++tx) {
        uint8_t p[48]; memset(p,0x5a,sizeof(p));
        ntp_response_header(p,(sntp_quality_t)rx,(sntp_quality_t)tx,true,true,0,100000,200000);
        sntp_quality_t q=rx==tx?(sntp_quality_t)tx:SNTP_QUALITY_NONE;
        assert(p[1]==timing_stratum(q));
        assert((p[0]>>6)==(timing_stratum(q)<16?0:3));
        assert((p[0]&0x3f)==0x24 && p[2]==6);
        assert((int8_t)p[3]==(q==SNTP_QUALITY_PPS?-20:-10));
        assert(read32(p+4)==0);
        assert(memcmp(p+12,timing_refid(q),4)==0);
        assert(read32(p+8)==ntp_dispersion_from_ns(rx==tx?200000:4000000000ULL));
        assert(read32(p+24)==0x5a5a5a5a && read32(p+40)==0x5a5a5a5a);
        assert(read32(p+16)==(rx==tx?0x5a5a5a5a:0));
    }
    for(unsigned leap=0;leap<4;++leap) {
        uint8_t p[48]={0};
        ntp_response_header(p,SNTP_QUALITY_PPS,SNTP_QUALITY_PPS,true,true,(uint8_t)leap,100000,100000);
        assert(p[0]>>6==leap);
        ntp_response_header(p,SNTP_QUALITY_PPS,SNTP_QUALITY_PPS,true,false,0,UINT64_MAX,0);
        assert(p[1]==16 && p[0]>>6==3 && read32(p+8)==UINT32_MAX);
        ntp_response_header(p,SNTP_QUALITY_PPS,SNTP_QUALITY_PPS,true,true,0,UINT64_MAX,0);
        assert(p[1]==16 && p[0]>>6==3 && read32(p+8)==UINT32_MAX);
    }
}
static void pps(void) {
    for (uint64_t age = 0; age <= 1500000000ULL; age += 100000000) {
        ntp_uncertainty_t u = ntp_pps_uncertainty(age, 20, 20, 10, -30, 60, 0);
        assert(u.phase_ns == 80);
        assert(u.frequency_uncertainty_ppm == 15);
        assert(u.growth_ns == age * 15 / 1000000);
        assert(u.total_ns == 100100 + u.growth_ns);
        assert(u.total_ns < 200000); /* Not the original 100–200 ms ramp. */
    }
    ntp_uncertainty_t early = ntp_pps_uncertainty(1000000000,20,20,10,30,4,0);
    assert(early.total_ns > 5000000);
    ntp_uncertainty_t noisy = ntp_pps_uncertainty(1000000000,20,20000,10,30,60,1000);
    assert(noisy.frequency_uncertainty_ppm == 80);
    assert(noisy.total_ns == 261020);
    ntp_uncertainty_t enormous = ntp_pps_uncertainty(UINT64_MAX,20,UINT64_MAX,0,0,60,INT64_MIN);
    assert(enormous.total_ns == UINT64_MAX);
}
static void holdover(void) {
    const uint64_t seconds[] = {0,1,60,3600,21600,86400};
    const double ppm[] = {0.1,1,2,5};
    for (unsigned j=0;j<4;++j) {
        rtc_holdover_t h = {.active=true,.start_ns=1000000000000000000ULL,
            .initial_error_ns=100000,.frequency_uncertainty_ppm=ppm[j]};
        uint64_t previous=0;
        for (unsigned i=0;i<6;++i) {
            uint64_t v=rtc_holdover_error_ns(&h,h.start_ns+seconds[i]*1000000000ULL);
            uint64_t expected=100000+RTC_HOLDOVER_MARGIN_NS+(uint64_t)(seconds[i]*1000*ppm[j]);
            assert(v>=previous && v>=expected && v-expected<=1);
            previous=v;
        }
        assert(!rtc_holdover_within_policy(&h,h.start_ns+86400000000000ULL));
    }
    rtc_holdover_t h={.active=true,.start_ns=1,.frequency_uncertainty_ppm=-1};
    assert(rtc_holdover_error_ns(&h,UINT64_MAX)==UINT64_MAX);
    h.frequency_uncertainty_ppm=1; h.initial_error_ns=UINT64_MAX;
    assert(rtc_holdover_error_ns(&h,UINT64_MAX)==UINT64_MAX);
    h.initial_error_ns=0;
    assert(rtc_holdover_error_ns(&h,h.start_ns+3600000000000ULL)==RTC_HOLDOVER_MARGIN_NS+3600000);
    h.frequency_uncertainty_ppm=0.1;
    assert(rtc_holdover_error_ns(&h,h.start_ns+3600000000000ULL)==RTC_HOLDOVER_MARGIN_NS+360000);
}
static void coarse(void) {
    ntp_uncertainty_t u=ntp_coarse_uncertainty(1000000000,true,100);
    assert(u.total_ns==2000000100ULL && u.age_ns==1000000000);
    u=ntp_coarse_uncertainty(24000000000000ULL,true,0);
    assert(u.total_ns==24001000000000ULL); /* no obsolete four-second cap */
    u=ntp_coarse_uncertainty(UINT64_MAX,true,INT64_MIN);
    assert(u.total_ns==UINT64_MAX);
    u=ntp_coarse_uncertainty(0,false,0);
    assert(u.total_ns==UINT64_MAX);
}
static void transitions(void) {
    rtc_holdover_t h = {0};
    utc_continuity_t c = {0};
    timing_observation_t o = {.now_ns=1000000000000ULL,
        .pps_timeout_ns=1500000000ULL,.pps_period_ns=1000000000,
        .pps_epoch_s=1788830000,.coarse_epoch_s=1788830000,
        .coarse_valid=true,.rtc_timeout_ns=2500000000ULL,
        .rtc_period_ns=1000000000,.rtc_jitter_ns=20000,
        .rtc_reference_error_ns=115000,.rtc_uncertainty_ppm=1};
    selected_time_t s = timing_select(&o,&h,&c);
    assert(s.quality==SNTP_QUALITY_COARSE_GPS && timing_stratum(s.quality)==16);
    assert(strcmp(timing_refid(s.quality),"LOCL")==0);
    o.pps_ns=o.now_ns;
    s=timing_select(&o,&h,&c);
    assert(s.quality==SNTP_QUALITY_PPS && timing_stratum(s.quality)==1);
    assert(strcmp(timing_refid(s.quality),"GPS ")==0 && !h.active);
    o.now_ns+=1200000000ULL;
    s=timing_select(&o,&h,&c);
    assert(s.quality==SNTP_QUALITY_PPS); /* short coast uses original anchor */
    ntp_uncertainty_t u=ntp_pps_uncertainty(o.now_ns-o.pps_ns,20,20,10,30,60,0);
    assert(u.age_ns==1200000000ULL && u.total_ns<200000);
    o.now_ns=o.pps_ns+2000000000ULL;
    o.rtc_valid=true; o.rtc_edge_ns=o.now_ns; o.rtc_qualified_ns=o.pps_ns;
    o.rtc_epoch_s=o.pps_epoch_s+2;
    s=timing_select(&o,&h,&c);
    assert(s.quality==SNTP_QUALITY_RTC_SQW && timing_stratum(s.quality)==2);
    assert(strcmp(timing_refid(s.quality),"RTC ")==0);
    assert(h.initial_error_ns==135000 && h.start_ns==o.rtc_qualified_ns);
    assert(rtc_holdover_error_ns(&h,o.now_ns)==2137000);
    o.now_ns+=3600000000000ULL; o.rtc_edge_ns=o.now_ns; o.rtc_epoch_s+=3600;
    s=timing_select(&o,&h,&c);
    assert(s.quality==SNTP_QUALITY_RTC_SQW);
    assert(rtc_holdover_error_ns(&h,o.now_ns)==5737000);
    o.pps_ns=o.now_ns; o.pps_epoch_s=o.rtc_epoch_s;
    s=timing_select(&o,&h,&c);
    assert(s.quality==SNTP_QUALITY_PPS && !h.active);
    u=ntp_pps_uncertainty(o.now_ns-o.pps_ns,20,20,10,30,60,0);
    assert(u.total_ns==100100 && u.age_ns==0); /* no old RTC hours */
    o.now_ns+=2000000000ULL; o.rtc_edge_ns=o.now_ns; o.rtc_epoch_s+=2;
    o.rtc_qualified_ns=o.pps_ns;
    s=timing_select(&o,&h,&c); assert(s.quality==SNTP_QUALITY_RTC_SQW);
    o.now_ns+=86400000000000ULL; o.rtc_edge_ns=o.now_ns; o.rtc_epoch_s+=86400;
    s=timing_select(&o,&h,&c);
    assert(s.quality==SNTP_QUALITY_COARSE_RTC && timing_stratum(s.quality)==16);
    o.rtc_valid=false; o.coarse_valid=false;
    s=timing_select(&o,&h,&c);
    assert(!s.valid && s.quality==SNTP_QUALITY_NONE && timing_stratum(s.quality)==16);
    c=(utc_continuity_t){0};
    o.now_ns=UINT64_MAX-1000000000ULL; o.pps_ns=o.now_ns-100000000ULL;
    s=timing_select(&o,&h,&c);
    assert(s.valid && s.quality==SNTP_QUALITY_PPS && !h.active);
    u=ntp_pps_uncertainty(o.now_ns-o.pps_ns,20,20,10,30,60,0);
    assert(u.total_ns==101600);
}
int main(void) { encoding(); placement(); packets(); pps(); holdover(); coarse(); transitions(); puts("NTP uncertainty tests passed"); }
