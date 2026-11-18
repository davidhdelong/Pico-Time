#include "gnss.h"
#include "../../gps_validation.h"
#include <string.h>
#include <ctype.h>

static bool calendar(int y, int m, int d, int h, int min, int sec, int64_t *epoch) {
    static const unsigned days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (y < 2000 || y > 2099 || m < 1 || m > 12 || d < 1 ||
        d > (int)(days[m - 1] + (m == 2 && y % 4 == 0)) ||
        h < 0 || h > 23 || min < 0 || min > 59 || sec < 0 || sec > 59) return false;
    int64_t n = 10957;
    for (int yr = 2000; yr < y; ++yr) n += 365 + (yr % 4 == 0);
    for (int mo = 1; mo < m; ++mo) n += days[mo - 1] + (mo == 2 && y % 4 == 0);
    *epoch = (n + d - 1) * 86400 + h * 3600 + min * 60 + sec;
    return true;
}
static int hex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}
static int two(const char *p) { return isdigit((unsigned char)p[0]) && isdigit((unsigned char)p[1]) ? (p[0]-'0')*10+p[1]-'0' : -1; }
static bool coordinate(const char *raw, const char *hemi, bool lat, double *out) {
    if (!raw || !*raw || !hemi || strlen(hemi) != 1) return false;
    char *end; double v = strtod(raw, &end), limit = lat ? 90 : 180;
    if (*end || !isfinite(v) || v < 0 || v > limit * 100) return false;
    int deg = (int)(v / 100); double minutes = v - deg * 100;
    if (minutes >= 60 || (lat ? (*hemi != 'N' && *hemi != 'S') : (*hemi != 'E' && *hemi != 'W'))) return false;
    *out = (deg + minutes / 60) * ((*hemi == 'S' || *hemi == 'W') ? -1 : 1);
    return true;
}
bool p4_gnss_nmea(char *line, p4_gnss_t *o) {
    memset(o, 0, sizeof(*o));
    if (!line || line[0] != '$') return false;
    char *star = strchr(line, '*');
    if (!star || strlen(star) != 3 || hex(star[1]) < 0 || hex(star[2]) < 0) return false;
    unsigned sum = 0; for (char *p = line + 1; p < star; ++p) sum ^= (unsigned char)*p;
    if (sum != (unsigned)(hex(star[1]) * 16 + hex(star[2]))) return false;
    *star = 0; char *fields[24] = {line}; unsigned n = 1;
    for (char *p = line; *p && n < 24; ++p) if (*p == ',') { *p = 0; fields[n++] = p + 1; }
    if (strlen(fields[0]) != 6) return false;
    if (!strcmp(fields[0] + 3, "RMC") && n >= 10) {
        o->fix = !strcmp(fields[2], "A");
        if (strlen(fields[1]) < 6 || strlen(fields[9]) != 6) return false;
        int yy = two(fields[9] + 4);
        o->time_valid = o->fix && yy >= 0 && calendar(2000 + yy, two(fields[9] + 2), two(fields[9]),
            two(fields[1]), two(fields[1] + 2), two(fields[1] + 4), &o->epoch_s);
        o->position_valid = o->fix && coordinate(fields[3], fields[4], true, &o->latitude) &&
            coordinate(fields[5], fields[6], false, &o->longitude);
        return true;
    }
    if (!strcmp(fields[0] + 3, "GGA") && n >= 10) {
        float fix = gps_number_or_unknown(fields[6], 0), sats = gps_number_or_unknown(fields[7], 0);
        o->fix = fix >= 1 && fix <= 8;
        o->satellites = sats >= 0 && sats <= 99 ? (unsigned)sats : 0;
        o->satellites_valid = true;
        o->position_valid = o->fix && coordinate(fields[2], fields[3], true, &o->latitude) &&
            coordinate(fields[4], fields[5], false, &o->longitude);
        return true;
    }
    return false;
}
bool p4_gnss_ubx(const ubx_frame_t *f, p4_gnss_t *o) {
    ubx_nav_pvt_t p;
    memset(o, 0, sizeof(*o));
    if (!ubx_decode_nav_pvt(f, &p)) return false;
    o->fix = (p.flags & 1u) && p.fix_type >= 2; o->satellites = p.satellites; o->satellites_valid = true;
    o->time_valid = o->fix && (p.valid & 7u) == 7u && calendar(p.year,p.month,p.day,p.hour,p.min,p.sec,&o->epoch_s);
    o->latitude = p.latitude_1e7 / 1e7; o->longitude = p.longitude_1e7 / 1e7;
    o->position_valid = o->fix && p.latitude_1e7 >= -900000000 && p.latitude_1e7 <= 900000000 &&
        p.longitude_1e7 >= -1800000000 && p.longitude_1e7 <= 1800000000;
    return true;
}
size_t p4_gnss_ubx_config(uint8_t *out, size_t capacity) {
    /* UBX-CFG-VALSET, RAM layer only. F10 configuration keys are also listed
     * in SparkFun's manufacturer library u-blox_config_keys.h. */
    static const struct {uint32_t key, value; unsigned size;} settings[] = {
        {0x10740001,1,1}, /* UART1 UBX output */
        {0x20910007,1,1}, /* NAV-PVT every navigation epoch */
        {0x30210001,1000,2}, {0x30210002,1,2}, /* 1 Hz navigation */
        {0x20050023,0,1}, {0x20050030,1,1}, /* period and length units */
        {0x40050002,1000000,4}, {0x40050003,1000000,4},
        {0x40050004,100000,4}, {0x40050005,100000,4},
        {0x10050007,1,1}, {0x10050008,1,1}, {0x10050009,1,1},
        {0x1005000a,1,1}, {0x1005000b,1,1}, {0x2005000c,0,1} /* UTC */
    };
    size_t payload = 4;
    for (unsigned i = 0; i < sizeof(settings)/sizeof(settings[0]); ++i) payload += 4 + settings[i].size;
    if (!out || capacity < payload + 8) return 0;
    out[0]=0xb5;out[1]=0x62;out[2]=0x06;out[3]=0x8a;
    out[4]=(uint8_t)payload;out[5]=(uint8_t)(payload>>8);
    out[6]=0;out[7]=1;out[8]=0;out[9]=0;
    size_t pos=10;
    for (unsigned i=0;i<sizeof(settings)/sizeof(settings[0]);++i) {
        for (unsigned j=0;j<4;++j) out[pos++]=(uint8_t)(settings[i].key>>(j*8));
        for (unsigned j=0;j<settings[i].size;++j) out[pos++]=(uint8_t)(settings[i].value>>(j*8));
    }
    uint8_t a=0,b=0;for (size_t i=2;i<pos;++i) {a+=out[i];b+=a;}
    out[pos++]=a;out[pos++]=b;return pos;
}
bool p4_gnss_config_ack(const ubx_frame_t *f, bool *accepted) {
    if (f->cls != 5 || (f->id != 0 && f->id != 1) || f->len != 2 ||
        f->payload[0] != 6 || f->payload[1] != 0x8a) return false;
    *accepted = f->id == 1;
    return true;
}
