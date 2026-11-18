#include "p4_clock.h"
#include <math.h>
#include <string.h>

static uint32_t median(const uint32_t *v, uint32_t n) {
    uint32_t a[P4_INTERVALS];
    memcpy(a, v, n * sizeof(*a));
    for (uint32_t i = 1; i < n; ++i) {
        uint32_t x = a[i], j = i;
        while (j && a[j - 1] > x) { a[j] = a[j - 1]; --j; }
        a[j] = x;
    }
    return a[n / 2];
}
static uint32_t mad(const p4_clock_t *c, uint32_t center) {
    uint32_t a[P4_INTERVALS];
    for (uint32_t i = 0; i < c->count; ++i)
        a[i] = c->intervals[i] > center ? c->intervals[i] - center : center - c->intervals[i];
    return median(a, c->count);
}
void p4_clock_init(p4_clock_t *c, uint64_t bound) {
    memset(c, 0, sizeof(*c)); c->capture_bound_ns = bound; c->period_ns = 1000000000u;
}
void p4_clock_fault(p4_clock_t *c) {
    ++c->capture_faults;
    c->edge_ns = c->trusted_ns = 0; c->epoch_known = false;
    c->servo_valid = false; c->count = c->next = c->samples = 0;
    memset(&c->recovery, 0, sizeof(c->recovery));
}
bool p4_clock_utc(p4_clock_t *c, int64_t epoch, uint64_t message_ns) {
    /* A sentence describes the preceding rising PPS. Fail closed outside
     * that association window, including task/queue backlog. */
    if (!c->edge_ns || message_ns < c->edge_ns || message_ns - c->edge_ns >= 900000000ULL)
        return false;
    if (c->epoch_known && epoch != c->epoch_s) {
        c->trusted_ns = 0;
        memset(&c->recovery, 0, sizeof(c->recovery));
    }
    c->epoch_s = epoch; c->epoch_known = true; c->message_ns = message_ns;
    return true;
}
bool p4_clock_pps(p4_clock_t *c, uint64_t ns) {
    if (c->edge_ns && (ns <= c->edge_ns || ns - c->edge_ns < 900000000ULL)) {
        ++c->rejected; return false; /* preserve genuine-edge baseline */
    }
    uint64_t delta = c->edge_ns ? ns - c->edge_ns : 0;
    bool good = delta >= 995000000ULL && delta <= 1005000000ULL;
    if (good && c->count >= 3) {
        uint32_t center = median(c->intervals, c->count);
        uint64_t threshold = (uint64_t)mad(c, center) * 6u;
        if (threshold < 50000) threshold = 50000;
        uint64_t residual = delta > center ? delta - center : center - delta;
        if (residual > threshold) { good = false; ++c->rejected; }
    }
    if (c->epoch_known && delta >= 995000000ULL && delta <= 1005000000ULL) ++c->epoch_s;
    else c->epoch_known = false;
    if (good) {
        c->intervals[c->next] = (uint32_t)delta;
        c->next = (c->next + 1) % P4_INTERVALS;
        if (c->count < P4_INTERVALS) ++c->count;
        if (c->samples < UINT32_MAX) ++c->samples;
        if (!c->servo_valid) { c->period_ns = (uint32_t)delta; c->period_rate = 0; c->servo_valid = true; }
        else {
            int64_t prediction = (int64_t)c->period_ns + c->period_rate;
            c->innovation_ns = (int64_t)delta - prediction;
            c->period_rate += c->innovation_ns / 64;
            if (c->period_rate > 200000) c->period_rate = 200000;
            if (c->period_rate < -200000) c->period_rate = -200000;
            prediction += c->innovation_ns / 8;
            if (prediction < 995000000) prediction = 995000000;
            if (prediction > 1005000000) prediction = 1005000000;
            c->period_ns = (uint32_t)prediction;
        }
    } else if (c->edge_ns) {
        c->count = c->next = c->samples = 0; c->servo_valid = false;
    }
    c->edge_ns = ns;
    bool dated = c->epoch_known && ns >= c->message_ns && ns - c->message_ns < 1500000000ULL;
    if (gnss_recovery_observe(&c->recovery, ns, good && dated)) {
        c->trusted_ns = ns; c->trusted_epoch_s = c->epoch_s;
    } else if (!good) c->trusted_ns = 0;
    return good;
}
p4_sample_t p4_clock_sample(p4_clock_t *c, uint64_t now) {
    p4_sample_t s = {0};
    s.pps_present = timing_edge_fresh(now, c->edge_ns, 2500000000ULL);
    s.message_fresh = timing_edge_fresh(now, c->message_ns, 3000000000ULL);
    timing_observation_t o = {.now_ns = now, .pps_ns = s.message_fresh ? c->trusted_ns : 0,
        .pps_timeout_ns = 2500000000ULL, .pps_epoch_s = c->trusted_epoch_s, .pps_period_ns = c->period_ns};
    s.time = timing_select(&o, &c->holdover, &c->continuity);
    s.locked = s.time.valid && s.time.quality == SNTP_QUALITY_PPS;
    if (s.locked) {
        uint32_t center = c->count ? median(c->intervals, c->count) : c->period_ns;
        double square = 0;
        for (uint32_t i = 0; i < c->count; ++i) {
            double e = (double)c->intervals[i] - center; square += e * e;
        }
        s.uncertainty = ntp_pps_uncertainty(now - c->trusted_ns, c->capture_bound_ns,
            c->count ? (uint64_t)sqrt(square / c->count) : 0,
            c->count ? mad(c, center) : 0, c->innovation_ns, c->samples, c->continuity.residual_ns);
    } else s.uncertainty.total_ns = UINT64_MAX;
    return s;
}
static void stamp(uint8_t *p, selected_time_t s) {
    if (!s.valid) { memset(p, 0, 8); return; }
    ntp_write_u32(p, (uint32_t)(s.unix_sec + 2208988800LL));
    ntp_write_u32(p + 4, s.fraction);
}
bool p4_ntp_reply(p4_clock_t *c, const uint8_t *req, size_t len,
                  uint64_t rx, uint64_t tx, uint8_t out[48]) {
    if (!req || !out || len < 48 || len > 512 || (req[0] & 7u) != 3 ||
        ((req[0] >> 3) & 7u) < 3 || ((req[0] >> 3) & 7u) > 4 || tx < rx) return false;
    p4_sample_t a = p4_clock_sample(c, rx), b = p4_clock_sample(c, tx);
    memset(out, 0, 48); memcpy(out + 24, req + 40, 8);
    stamp(out + 32, a.time); stamp(out + 40, b.time);
    if (b.locked) ntp_write_u32(out + 16, (uint32_t)(c->trusted_epoch_s + 2208988800LL));
    ntp_response_header(out, a.time.quality, b.time.quality, a.locked, b.locked, 0,
                        a.uncertainty.total_ns, b.uncertainty.total_ns);
    return true;
}
