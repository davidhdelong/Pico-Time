#include "pps_monitor.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

void pps_monitor_init(pps_monitor_t *m, uint32_t resolution_hz) {
    memset(m, 0, sizeof(*m));
    m->resolution_hz = resolution_hz;
    m->segment = 1;
}
bool pps_input_fresh(const pps_input_t *i, uint64_t now_us) {
    return i->seen && now_us >= i->rise_us && now_us - i->rise_us < PPS_FRESH_US;
}
static void break_segment(pps_monitor_t *m) {
    if (m->segment_open) ++m->segment;
    m->segment_open = false;
    m->input[0].pending = m->input[1].pending = false;
}
static uint64_t missing_intervals(uint64_t elapsed) {
    uint64_t intervals = elapsed / 1000000ULL + (elapsed % 1000000ULL >= 500000ULL);
    return intervals > 0 ? intervals - 1 : 0;
}
uint64_t pps_input_missed(const pps_input_t *i, uint64_t now_us) {
    uint64_t extra = i->seen && now_us >= i->rise_us ? missing_intervals(now_us - i->rise_us) : 0;
    return i->missed > UINT64_MAX - extra ? UINT64_MAX : i->missed + extra;
}
void pps_monitor_poll(pps_monitor_t *m, uint64_t now_us) {
    for (unsigned j = 0; j < 2; ++j) {
        pps_input_t *i = &m->input[j];
        if (i->seen && !i->timed_out && !pps_input_fresh(i, now_us) && now_us >= i->rise_us) {
            break_segment(m);
            i->timed_out = true;
            i->good_intervals = 0;
            i->have_period = i->have_width = i->width_pending = false;
        }
    }
}
void pps_monitor_invalidate(pps_monitor_t *m) {
    break_segment(m);
    memset(m->input, 0, sizeof(m->input));
    ++m->invalidations;
}
void pps_monitor_clear_stats(pps_monitor_t *m) {
    break_segment(m);
    m->count = m->next = 0;
    m->pairs = m->unpaired = 0;
}
const pps_pair_t *pps_monitor_pair(const pps_monitor_t *m, uint32_t index) {
    if (index >= m->count) return NULL;
    uint32_t oldest = m->count == PPS_HISTORY ? m->next : 0;
    return &m->history[(oldest + index) % PPS_HISTORY];
}
static void pair_pending(pps_monitor_t *m) {
    pps_input_t *a = &m->input[0], *b = &m->input[1];
    if (!a->pending || !b->pending) return;
    uint64_t observed_gap = a->rise_us > b->rise_us ? a->rise_us - b->rise_us : b->rise_us - a->rise_us;
    int64_t ticks = (uint32_t)(b->rise_ticks - a->rise_ticks);
    if (ticks > INT32_MAX) ticks -= 4294967296LL;
    double us = (double)ticks * 1000000.0 / m->resolution_hz;
    if (observed_gap > PPS_PAIR_MAX_US || fabs(us) > PPS_PAIR_MAX_US) {
        if (a->rise_us < b->rise_us) a->pending = false;
        else b->pending = false;
        ++m->unpaired;
        return;
    }
    const pps_pair_t *previous = m->count ? pps_monitor_pair(m, m->count - 1) : NULL;
    uint64_t observed = a->rise_us > b->rise_us ? a->rise_us : b->rise_us;
    if (previous && observed >= previous->observed_us && observed - previous->observed_us >= 1500000ULL)
        break_segment(m);
    m->history[m->next] = (pps_pair_t){.observed_us = a->rise_us > b->rise_us ? a->rise_us : b->rise_us, .offset_us = us, .segment = m->segment};
    m->segment_open = true;
    m->next = (m->next + 1) % PPS_HISTORY;
    if (m->count < PPS_HISTORY) ++m->count;
    ++m->pairs;
    a->pending = b->pending = false;
}
void pps_monitor_edge(pps_monitor_t *m, const pps_edge_t *e) {
    if (e->input > 1 || !m->resolution_hz) return;
    pps_monitor_poll(m, e->observed_us);
    pps_input_t *i = &m->input[e->input];
    if (!e->rising) {
        if (i->seen && i->width_pending && e->observed_us >= i->rise_us && e->observed_us - i->rise_us < 1000000) {
            double width = (double)(uint32_t)(e->ticks - i->rise_ticks) * 1000000.0 / m->resolution_hz;
            if (width < 1000000) { i->width_us = width; i->have_width = true; i->width_pending = false; }
        }
        return;
    }
    ++i->edges;
    if (i->seen) {
        if (e->observed_us <= i->rise_us) { ++i->glitches; i->width_pending = false; return; }
        uint64_t elapsed = e->observed_us - i->rise_us;
        double period = (double)(uint32_t)(e->ticks - i->rise_ticks) * 1000000.0 / m->resolution_hz;
        /* An early glitch must not replace the preceding genuine edge. */
        if (elapsed < 900000 || (elapsed < 1500000 && period < 900000)) {
            ++i->glitches; i->width_pending = false; return;
        }
        bool valid = elapsed < 1500000 && period >= 995000 && period <= 1005000;
        if (valid) {
            i->period_us = period; i->have_period = true;
            if (i->good_intervals < 2) ++i->good_intervals;
        } else {
            if (elapsed >= 1500000) i->missed += missing_intervals(elapsed);
            else ++i->glitches;
            i->good_intervals = 0; i->have_period = false;
            break_segment(m);
        }
    }
    if (i->pending) ++m->unpaired;
    i->rise_ticks = e->ticks; i->rise_us = e->observed_us;
    i->seen = true; i->have_width = false; i->width_pending = true; i->timed_out = false;
    i->pending = i->good_intervals >= 2;
    pair_pending(m);
}
static int compare_double(const void *a, const void *b) {
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}
void pps_monitor_summary(const pps_monitor_t *m, uint64_t now_us, pps_summary_t *s) {
    memset(s, 0, sizeof(*s));
    if (!m->count) return;
    const pps_pair_t *last = pps_monitor_pair(m, m->count - 1);
    s->age_us = now_us >= last->observed_us ? now_us - last->observed_us : UINT64_MAX;
    s->valid = s->age_us < PPS_FRESH_US && pps_input_fresh(&m->input[0], now_us) &&
        pps_input_fresh(&m->input[1], now_us) && m->input[0].good_intervals >= 2 && m->input[1].good_intervals >= 2;
    s->latest_us = last->offset_us;
    double values[PPS_HISTORY], mean = 0, m2 = 0, squares = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
    uint64_t first_us = 0;
    s->min_us = s->max_us = last->offset_us;
    for (uint32_t j = 0; j < m->count; ++j) {
        const pps_pair_t *p = pps_monitor_pair(m, j);
        double value = p->offset_us, delta = value - mean;
        values[j] = value; mean += delta / (j + 1); m2 += delta * (value - mean);
        squares += value * value;
        if (value < s->min_us) s->min_us = value;
        if (value > s->max_us) s->max_us = value;
        if (p->segment == m->segment) {
            if (!s->drift_samples) first_us = p->observed_us;
            double x = (double)(p->observed_us - first_us) / 1000000.0;
            ++s->drift_samples; sx += x; sy += value; sxx += x * x; sxy += x * value;
        }
    }
    s->mean_us = mean; s->stddev_us = sqrt(fmax(0, m2 / m->count));
    s->rms_us = sqrt(squares / m->count);
    qsort(values, m->count, sizeof(values[0]), compare_double);
    s->median_us = m->count % 2 ? values[m->count / 2] : (values[m->count / 2 - 1] + values[m->count / 2]) / 2;
    double denominator = s->drift_samples ? sxx - sx * sx / s->drift_samples : 0;
    if (s->valid && s->drift_samples >= 10 && denominator > 0) {
        s->drift_valid = true;
        s->drift_ppm = (sxy - sx * sy / s->drift_samples) / denominator; /* microseconds/second = ppm */
    }
}
