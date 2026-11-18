#include "p4_clock.h"
#include "gnss.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const int64_t base = 1795000000LL;
static void train(p4_clock_t *c, unsigned first, unsigned last, int64_t utc_base) {
    for (unsigned s=first;s<=last;++s) {
        uint64_t ns=(uint64_t)s*1000000000ULL;
        p4_clock_pps(c,ns);
        assert(p4_clock_utc(c,utc_base+s,ns+100000000ULL));
    }
}
static uint32_t u32(const uint8_t *p) {
    return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3];
}
static void nmea(const char *body,char *out,size_t size) {
    unsigned ck=0;for(const char *p=body;*p;++p)ck^=(unsigned char)*p;
    snprintf(out,size,"$%s*%02X",body,ck);
}
int main(void) {
    p4_clock_t c;p4_clock_init(&c,2000);
    uint8_t request[513]={0x23},reply[48];request[40]=0x42;
    assert(p4_ntp_reply(&c,request,48,100,200,reply));
    assert(reply[1]==16&&(reply[0]>>6)==3&&u32(reply+16)==0);
    train(&c,1,80,base);
    p4_sample_t s=p4_clock_sample(&c,80500000000ULL);
    assert(s.locked&&s.time.unix_sec==base+80&&s.time.fraction==0x80000000u);
    assert(s.uncertainty.total_ns>=102000&&s.uncertainty.frequency_uncertainty_ppm>=15);
    assert(p4_ntp_reply(&c,request,48,80510000000ULL,80511000000ULL,reply));
    assert(reply[1]==1&&(reply[0]&7)==4&&(reply[0]>>6)==0);
    assert(!memcmp(reply+12,"GPS ",4)&&!memcmp(reply+24,request+40,8));
    assert(u32(reply+32)==(uint32_t)(base+80+2208988800LL));
    assert(u32(reply+40)==u32(reply+32)&&u32(reply+44)>u32(reply+36));
    uint64_t edge=c.edge_ns;
    assert(!p4_clock_pps(&c,edge+10000000));assert(c.edge_ns==edge&&c.rejected==1);
    s=p4_clock_sample(&c,84000000000ULL);assert(!s.locked);
    assert(p4_ntp_reply(&c,request,48,84000000000ULL,84001000000ULL,reply));
    assert(reply[1]==16&&(reply[0]>>6)==3&&u32(reply+16)==0);
    p4_clock_fault(&c);train(&c,90,98,base);
    s=p4_clock_sample(&c,98100000000ULL);assert(s.locked);
    p4_clock_pps(&c,100000000000ULL);assert(!p4_clock_sample(&c,100100000000ULL).locked);
    train(&c,101,108,base);assert(p4_clock_sample(&c,108200000000ULL).locked);
    assert(p4_clock_utc(&c,base+108+86400,108300000000ULL));
    assert(!p4_clock_sample(&c,108400000000ULL).locked);
    assert(!p4_clock_utc(&c,base,108950000000ULL)); /* ambiguous UART phase */
    assert(!p4_ntp_reply(&c,request,47,1,2,reply));
    assert(!p4_ntp_reply(&c,request,513,1,2,reply));
    request[0]=0x24;assert(!p4_ntp_reply(&c,request,48,1,2,reply));
    request[0]=0x13;assert(!p4_ntp_reply(&c,request,48,1,2,reply));
    request[0]=0x1b;assert(p4_ntp_reply(&c,request,48,1,2,reply)); /* v3 client */
    p4_clock_init(&c,1000);train(&c,1,70,2200000000LL); /* NTP era rollover */
    assert(p4_ntp_reply(&c,request,48,70100000000ULL,70101000000ULL,reply));
    assert(reply[1]==1&&u32(reply+40)==(uint32_t)(2200000070LL+2208988800LL));
    p4_gnss_t msg;char line[256];
    nmea("GPRMC,000000.00,A,3000.0,N,09300.0,W,0,0,010100,,,A",line,sizeof(line));
    assert(p4_gnss_nmea(line,&msg)&&msg.time_valid&&msg.epoch_s==946684800);
    assert(msg.position_valid&&msg.latitude==30&&msg.longitude==-93);
    nmea("GPRMC,000000,V,,,,,,,010100,,,N",line,sizeof(line));
    assert(p4_gnss_nmea(line,&msg)&&!msg.time_valid);
    nmea("GPRMC,000000,A,1e30,N,0,E,0,0,310226,,,A",line,sizeof(line));
    assert(p4_gnss_nmea(line,&msg)&&!msg.position_valid&&!msg.time_valid);
    nmea("GPGGA,000000,3000,N,09300,W,1,09,1.0,10,M,0,M,,",line,sizeof(line));
    assert(p4_gnss_nmea(line,&msg)&&msg.satellites==9&&msg.position_valid&&!msg.time_valid);
    line[strlen(line)-1]='z';assert(!p4_gnss_nmea(line,&msg));
    ubx_frame_t frame={.cls=1,.id=7,.len=92};
    frame.payload[4]=0xe8;frame.payload[5]=7;frame.payload[6]=2;frame.payload[7]=29;
    frame.payload[11]=7;frame.payload[20]=3;frame.payload[21]=1;frame.payload[23]=12;
    assert(p4_gnss_ubx(&frame,&msg)&&msg.time_valid&&msg.epoch_s==1709164800);
    frame.payload[11]=0;assert(p4_gnss_ubx(&frame,&msg)&&!msg.time_valid);
    uint8_t cfg[160];size_t n=p4_gnss_ubx_config(cfg,sizeof(cfg));assert(n>0);
    assert(!p4_gnss_ubx_config(cfg,8));
    ubx_parser_t parser;ubx_parser_init(&parser);bool decoded=false;
    for(size_t i=0;i<n;++i)decoded=ubx_parser_feed(&parser,cfg[i],i,&frame);
    assert(decoded&&frame.cls==6&&frame.id==0x8a&&frame.payload[1]==1);
    assert(frame.payload[frame.len-1]==0); /* final setting: UTC time grid */
    frame=(ubx_frame_t){.cls=5,.id=1,.len=2,.payload={6,0x8a}};
    bool accepted=false;assert(p4_gnss_config_ack(&frame,&accepted)&&accepted);
    frame.id=0;assert(p4_gnss_config_ack(&frame,&accepted)&&!accepted);
    frame.payload[1]=1;assert(!p4_gnss_config_ack(&frame,&accepted));
    puts("P4 NTP: GNSS/PPS lock, UTC fractions, packet fields, v3/v4, era rollover, loss/recovery, faults and GNSS/config validation passed");
}
