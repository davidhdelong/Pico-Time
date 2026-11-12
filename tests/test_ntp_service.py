"""Host-test the actual UTC conversion and UDP NTP callback, without radio I/O."""
import pathlib
import subprocess
import tempfile

source = pathlib.Path("main.c").read_text()
def extract(start, end):
    return start + source.split(start, 1)[1].split(end, 1)[0]

conversion = extract("static bool get_sntp_time_details(uint64_t", "static bool get_sntp_time_at")
callback = extract("static void sntp_recv(void", "static void sntp_server_init")
helpers = extract("static void sntp_write_u32", "#if HEALTH_BROADCAST_ENABLE && HEALTH_BROADCAST_CRC")
for removed in ("leap_", "smear", "dns_gethostbyname", "tcp_connect", "RTC_EEPROM", "LEAPFILE"):
    assert removed not in source, removed

harness = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ntp_uncertainty.h"
#define NTP_DELTA 2208988800UL
#define SNTP_MAX_REQUESTS_PER_SECOND 20
#define PBUF_TRANSPORT 0
#define PBUF_RAM 0
#define ERR_OK 0
typedef uint16_t u16_t;
typedef int ip_addr_t;
struct udp_pcb { int unused; };
struct pbuf { uint16_t tot_len; void *payload; };
static int stats_mutex, allocations, replies;
static bool allocation_fail;
static uint32_t sntp_malformed_count,sntp_rate_limit_count,sntp_request_count,sntp_send_error_count;
static uint32_t sntp_rate_window_count;
static uint64_t sntp_rate_window_start_us, now=1000000000ULL;
static uint64_t trusted_pps_ns=1000000000ULL;
static int64_t trusted_pps_epoch_s=1793372709,rtc_last_qualified_epoch_s=1793372708;
static utc_continuity_t utc_continuity;
static ntp_edge_bracket_t pps_placement_brackets[2]={{999990000,1000010000},{0,0}};
static selected_time_t selected;
static uint8_t wire[48],request_bytes[48];
static struct pbuf request={48,request_bytes};
static void mutex_enter_blocking(int *m) { assert(!*m);*m=1; }
static void mutex_exit(int *m) { assert(*m);*m=0; }
static void select_time_at_locked(uint64_t ns,selected_time_t *out) {
    assert(stats_mutex);*out=selected;
    if(ns>1000000000ULL)out->fraction+=0x1000;
}
static ntp_uncertainty_t get_ntp_uncertainty_locked(sntp_quality_t q,uint64_t ns) {
    (void)q;(void)ns;assert(stats_mutex);return (ntp_uncertainty_t){.total_ns=100000};
}
/* CONVERSION */
static uint64_t time_us_64(void) { return now/1000; }
static uint64_t hires_time_ns(void) { uint64_t old=now;now+=1000;return old; }
static void pbuf_copy_partial(struct pbuf *p,void *dst,size_t len,size_t offset) {
    assert(offset+len<=p->tot_len);memcpy(dst,(uint8_t *)p->payload+offset,len);
}
static void pbuf_free(struct pbuf *p) {
    if(p==&request)return;
    free(p->payload);free(p);--allocations;
}
static struct pbuf *pbuf_alloc(int layer,size_t len,int type) {
    (void)layer;(void)type;if(allocation_fail)return NULL;
    struct pbuf *p=malloc(sizeof(*p));assert(p);p->payload=malloc(len);assert(p->payload);
    p->tot_len=(uint16_t)len;++allocations;return p;
}
static int udp_sendto(struct udp_pcb *pcb,struct pbuf *p,const ip_addr_t *addr,u16_t port) {
    (void)pcb;(void)addr;assert(port==123&&p->tot_len==48);memcpy(wire,p->payload,48);++replies;return 0;
}
/* HELPERS */
/* CALLBACK */
static uint32_t read32(unsigned i) {
    return (uint32_t)wire[i]<<24|(uint32_t)wire[i+1]<<16|(uint32_t)wire[i+2]<<8|wire[i+3];
}
static void send_request(void) {
    now=1000000000ULL;sntp_rate_window_count=0;
    sntp_recv(NULL,NULL,&request,NULL,123);assert(!allocations&&!stats_mutex);
}
int main(void) {
    /* Preserve every UTC fraction bit across dates and second boundaries. */
    const int64_t epochs[]={1483228799,1483228800,1798761599,1798761600,1810771199,1810771200};
    const uint32_t fractions[]={0,1,0x80000000u,0xffffffffu};
    for(unsigned i=0;i<sizeof(epochs)/sizeof(*epochs);++i)for(unsigned j=0;j<4;++j) {
        selected=(selected_time_t){.valid=true,.unix_sec=epochs[i],.fraction=fractions[j],.quality=SNTP_QUALITY_PPS};
        uint32_t sec=0,frac=0,ref=0;bool synced=false;sntp_quality_t quality=SNTP_QUALITY_NONE;
        assert(get_sntp_time_details(1000000000ULL,&sec,&frac,&synced,&quality,NULL,&ref));
        assert(sec==(uint32_t)(epochs[i]+NTP_DELTA)&&frac==fractions[j]&&synced);
        assert(quality==SNTP_QUALITY_PPS&&ref==(uint32_t)(trusted_pps_epoch_s+NTP_DELTA));
    }
    selected=(selected_time_t){.valid=true,.unix_sec=1793372709,.fraction=0x12345678,.quality=SNTP_QUALITY_PPS};
    request_bytes[0]=0x23;for(unsigned i=40;i<48;++i)request_bytes[i]=(uint8_t)i;
    send_request();assert(replies==1&&wire[0]==0x24&&wire[1]==1&&!memcmp(wire+12,"GPS ",4));
    assert(!memcmp(wire+24,request_bytes+40,8));
    assert(read32(16)==(uint32_t)(trusted_pps_epoch_s+NTP_DELTA)&&read32(20)==0);
    assert(read32(32)==(uint32_t)(selected.unix_sec+NTP_DELTA)&&read32(36)==selected.fraction);
    assert(read32(40)==read32(32)&&read32(44)==selected.fraction+0x1000);
    selected.quality=SNTP_QUALITY_RTC_SQW;send_request();assert(wire[1]==2&&wire[0]>>6==0&&!memcmp(wire+12,"RTC ",4));
    selected.quality=SNTP_QUALITY_COARSE_GPS;send_request();assert(wire[1]==16&&wire[0]>>6==3);
    selected.valid=false;send_request();assert(wire[1]==16&&wire[0]>>6==3&&read32(32)==0&&read32(40)==0);
    selected.valid=true;selected.quality=SNTP_QUALITY_PPS;
    memset(pps_placement_brackets,0,sizeof(pps_placement_brackets));send_request();assert(wire[1]==16&&wire[0]>>6==3);
    int before=replies;request_bytes[0]=0x24;send_request();assert(replies==before&&sntp_malformed_count==1);
    request_bytes[0]=0x23;allocation_fail=true;send_request();assert(replies==before&&sntp_send_error_count==1);
    puts("NTP service: exact UTC/fractions, reference/originate/RX/TX timestamps, GPS/RTC/coarse/invalid quality and failures passed");
}
'''.replace("/* CONVERSION */", conversion).replace("/* HELPERS */", helpers).replace("/* CALLBACK */", callback)

with tempfile.TemporaryDirectory(prefix="pico-ntp-service-") as folder:
    binary=str(pathlib.Path(folder)/"test")
    for flags in ([],["-fsanitize=address,undefined","-fno-omit-frame-pointer"]):
        subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror","-I.",*flags,
                        "-x","c","-","ntp_uncertainty.c","rtc_holdover.c","-lm","-o",binary],
                       input=harness,text=True,check=True)
        subprocess.run([binary],check=True)
