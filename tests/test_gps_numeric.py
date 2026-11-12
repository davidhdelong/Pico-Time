"""Fault-inject the shipped NMEA parsers and coordinate conversion."""
import json
import pathlib
import subprocess
import tempfile

source = pathlib.Path("main.c").read_text()
def section(start, end):
    return start + source.split(start, 1)[1].split(end, 1)[0]

functions = "\n".join([
    section("static int split_fields(", "static bool nmea_checksum_valid"),
    section("static bool nmea_checksum_valid(", "static bool parse_deg_min"),
    section("static bool parse_deg_min(", "static void set_time_field"),
    section("static void set_time_field(", "static void set_date_field"),
    section("static bool parse_nonnegative_int(", "static void rtc_cache_update_locked"),
    section("static bool parse_hhmmss(const char *raw, int *hour, int *min, int *sec) {", "static bool parse_ddmmyy"),
    section("static bool parse_gga(", "static void update_gsv_count"),
    section("static bool parse_gsa(", "static bool parse_rmc"),
])
harness = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "gps_validation.h"
typedef struct {
    char time_utc[16]; bool lat_valid,lon_valid,has_fix;
    float lat_deg,lon_deg,hdop,pdop,vdop,altitude_m;
    int fix_quality,satellites;
    uint8_t nmea_used_gps,nmea_used_glo,nmea_used_gal,nmea_used_bds,nmea_used_gn;
} gps_stats_t;
static uint64_t last_gga_update_ns;
static uint64_t hires_time_ns(void) {return 100;}
static void update_gsa_fix(const char *t,uint8_t f,gps_stats_t *s) {(void)t;(void)f;(void)s;}
/* FUNCTIONS */
static void checked_line(char *line,size_t capacity,const char *body) {
    unsigned ck=0;for(const char *p=body;*p;++p)ck^=(unsigned char)*p;
    snprintf(line,capacity,"$%s*%02X",body,ck);assert(nmea_checksum_valid(line));
}
int main(void) {
    const char *bad[]={"nan","inf","-inf","1e999","1e-999","1.2oops"};
    for(unsigned j=0;j<sizeof(bad)/sizeof(bad[0]);++j) {
        char body[256],line[280];gps_stats_t stats={0};
        snprintf(body,sizeof(body),"GPGGA,123456,3000.0,N,09300.0,W,1,08,%s,%s,M,0,M,,",bad[j],bad[j]);
        checked_line(line,sizeof(line),body);assert(parse_gga(line,&stats));
        assert(stats.hdop==-1&&stats.altitude_m==-1);
        snprintf(body,sizeof(body),"GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,%s,%s,%s",bad[j],bad[j],bad[j]);
        checked_line(line,sizeof(line),body);assert(parse_gsa(line,&stats));
        assert(stats.pdop==-1&&stats.hdop==-1&&stats.vdop==-1);
        printf("{\"hdop\":%.2f,\"pdop\":%.2f,\"vdop\":%.2f}\n",(double)stats.hdop,(double)stats.pdop,(double)stats.vdop);
    }
    char line[280];gps_stats_t stats={0};
    checked_line(line,sizeof(line),"GPGGA,123456,3000.0,N,09300.0,W,1,08,1.25,-42.5,M,0,M,,");
    assert(parse_gga(line,&stats)&&stats.hdop==1.25f&&stats.altitude_m==-42.5f);
    float out=17;
    const char *invalid[]={"1e30","1e999","nan","inf","9000.1","8960.0","-1","junk"};
    for(unsigned j=0;j<sizeof(invalid)/sizeof(invalid[0]);++j) {
        assert(!parse_deg_min(invalid[j],"N",true,&out));assert(out==17);
    }
    assert(parse_deg_min("9000","N",true,&out)&&out==90);
    assert(parse_deg_min("18000","W",false,&out)&&out==-180);
    assert(!parse_deg_min("18000.1","W",false,&out));
}
'''.replace("/* FUNCTIONS */", functions)
with tempfile.TemporaryDirectory(prefix="gps-numeric-") as folder:
    binary = str(pathlib.Path(folder)/"test")
    for flags in ([], ["-fsanitize=address,undefined,float-cast-overflow", "-fno-sanitize-recover=all"]):
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I.",
                        *flags, "-x", "c", "-", "-o", binary], input=harness, text=True, check=True)
        result = subprocess.run([binary], capture_output=True, text=True, check=True)
        for line in result.stdout.splitlines():
            assert json.loads(line) == {"hdop": -1, "pdop": -1, "vdop": -1}
print("NMEA numeric faults: checksum-valid GGA/GSA, finite JSON, valid negative altitude and coordinate bounds passed")
