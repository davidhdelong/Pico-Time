#ifndef PPS_CAPTURE_TIMING_H
#define PPS_CAPTURE_TIMING_H
#include <stdbool.h>
#include <stdint.h>

/* Map the FIRST MOV ISR,X snapshot after a fresh PIO restart to the local
 * clock. Cycle zero is the start of MOV X,~NULL; the first snapshot is at
 * 2 + 2*decrements clocks. Each X underflow costs one extra clock.
 * Software arrival time resolves whole counter wraps ONLY, never phase.
 * Caller must service captures within 1.5 s and discard FIFO stalls/backlog.
 * The enable bracket includes the start phase of the PIO divider. GPIO input
 * synchronization and edge sampling add a few clocks of bounded latency;
 * this is not a calibrated UTC phase measurement. */
static inline bool pps_capture_first_ns(uint32_t current, uint32_t counter_hz,
                                       uint64_t start_ns, uint64_t observed_ns,
                                       uint64_t *timestamp_ns, uint64_t *remainder) {
    uint64_t hz = (uint64_t)counter_hz * 2u;
    if (!hz || observed_ns < start_ns) return false;
    uint64_t age = observed_ns - start_ns;
    if (age / 1000000000ULL > UINT64_MAX / hz) return false;
    uint64_t estimated = (age / 1000000000ULL) * hz;
    uint64_t fraction = (age % 1000000000ULL) * hz / 1000000000ULL;
    if (estimated > UINT64_MAX - fraction) return false;
    estimated += fraction;
    uint64_t clocks = 2u + (uint64_t)(UINT32_MAX - current) * 2u;
    const uint64_t wrap_clocks = 8589934593ULL;
    if (estimated > clocks) {
        uint64_t difference = estimated - clocks;
        uint64_t wraps = difference / wrap_clocks;
        if (difference % wrap_clocks > wrap_clocks / 2u) ++wraps;
        if (wraps > (UINT64_MAX - clocks) / wrap_clocks) return false;
        clocks += wraps * wrap_clocks;
    }
    uint64_t seconds = clocks / hz;
    if (seconds > UINT64_MAX / 1000000000ULL) return false;
    uint64_t numerator = (clocks % hz) * 1000000000ULL;
    uint64_t elapsed = seconds * 1000000000ULL;
    if (elapsed > UINT64_MAX - numerator / hz) return false;
    elapsed += numerator / hz;
    if (start_ns > UINT64_MAX - elapsed) return false;
    uint64_t timestamp = start_ns + elapsed;
    if (timestamp > observed_ns ? timestamp - observed_ns > 10000u :
                                 observed_ns - timestamp > 1500000000ULL) return false;
    *timestamp_ns = timestamp;
    *remainder = numerator % hz;
    return true;
}

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
