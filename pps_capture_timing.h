#ifndef PPS_CAPTURE_TIMING_H
#define PPS_CAPTURE_TIMING_H
#include <stdbool.h>
#include <stdint.h>

/* pps_capture.pio: one decrement per two clocks in steady low/high loops.
 * Between consecutive rising-edge MOV snapshots there are six additional
 * clocks (four on capture, two on falling), and one more on X underflow.
 * Only use for consecutive captures with no RX FIFO stall. */
static inline uint64_t pps_capture_elapsed_ns(uint32_t previous, uint32_t current,
                                             uint32_t counter_hz, uint64_t *remainder) {
    uint64_t clocks = (uint64_t)(uint32_t)(previous - current) * 2u + 6u;
    if (current > previous) ++clocks;
    uint64_t hz = (uint64_t)counter_hz * 2u;
    if (!hz) return UINT64_MAX;
    /* Even UINT32_MAX decrements * 2 * 1e9 fits uint64_t. Carry division
     * remainder across intervals rather than accumulating rounding bias. */
    uint64_t numerator = clocks * 1000000000ULL + *remainder;
    *remainder = numerator % hz;
    return numerator / hz;
}

/* Require an exactly representable 16.8 PIO divider. Never silently convert
 * counter ticks using a requested frequency that hardware cannot produce. */
static inline bool pps_capture_divider(uint32_t system_hz, uint32_t counter_hz,
                                       uint32_t *divider256) {
    uint64_t hz = (uint64_t)counter_hz * 2u;
    uint64_t scaled = (uint64_t)system_hz * 256u;
    if (!hz || scaled % hz) return false;
    uint64_t divider = scaled / hz;
    if (divider < 256u || divider > 0xffffffu) return false;
    *divider256 = (uint32_t)divider;
    return true;
}
#endif
