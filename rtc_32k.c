#include "rtc_32k.h"
#include <math.h>

void rtc_32k_observe(rtc_32k_t *f, uint64_t timestamp_ns, uint32_t sequence,
                     bool gnss_locked, uint32_t pps_period_ns) {
    f->enabled = RTC_32KHZ_GPIO >= 0;
    if (!f->enabled) return;
    uint64_t interval_ns = timestamp_ns - f->last_ns;
    bool continuous = f->last_ns && timestamp_ns > f->last_ns &&
        sequence - f->sequence == 1 && interval_ns >= 995000000ULL && interval_ns <= 1005000000ULL;
    if (!continuous) {
        f->valid = false; f->base_ns = 0; f->good_blocks = 0;
        if (f->last_ns) { if (f->rejects < UINT32_MAX) ++f->rejects; f->last_reject = 1; }
    } else {
        f->pico_period_ns = f->pico_period_ns > 0 ?
            f->pico_period_ns + ((double)interval_ns - f->pico_period_ns) / 16.0 : (double)interval_ns;
    }
    f->last_ns = timestamp_ns; f->sequence = sequence;
    if (!gnss_locked) { f->base_ns = 0; return; }
    if (!f->base_ns) { f->base_ns = timestamp_ns; f->base_sequence = sequence; return; }
    uint32_t reports = sequence - f->base_sequence;
    if (reports < 64) return;
    double fast_ppm = 0;
    if (reports > 128 || !rtc_32k_frequency(timestamp_ns - f->base_ns, reports * 32768u, pps_period_ns, &fast_ppm) ||
        (f->valid && fabs(fast_ppm - f->frequency_error_ppm) > 5.0)) {
        if (f->rejects < UINT32_MAX) ++f->rejects;
        f->last_reject = 2; f->valid = false; f->good_blocks = 0;
    } else {
        double residual = f->good_blocks ? fabs(fast_ppm - f->frequency_error_ppm) : 1.0;
        f->frequency_error_ppm = f->good_blocks ? f->frequency_error_ppm + (fast_ppm - f->frequency_error_ppm) / 8.0 : fast_ppm;
        /* 100 us endpoint latency budget over 64 seconds, plus variation.
         * This deliberately does not claim PIO-counter timestamp precision. */
        f->uncertainty_ppm = 1.6 + residual;
        if (f->samples < UINT32_MAX) ++f->samples;
        if (f->good_blocks < 4) ++f->good_blocks;
        f->valid = f->good_blocks >= 4; f->last_reject = 0;
    }
    f->base_ns = timestamp_ns; f->base_sequence = sequence;
}

#ifndef RTC_32K_HOST_TEST
#if RTC_32KHZ_GPIO >= 0
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico/time.h"
#include "rtc_32k.pio.h"
static int rtc_32k_sm;
static volatile uint64_t captured_ns;
static volatile uint32_t captured_sequence;
static volatile bool captured;
static void rtc_32k_irq(void) {
    /* Low-rate timestamp uses the same monotonic microsecond epoch underlying
     * hires_time_ns. ISR quantization/latency is included in uncertainty. */
    uint64_t now_ns = time_us_64() * 1000ULL;
    pio_interrupt_clear(pio1, 0);
    while (!pio_sm_is_rx_fifo_empty(pio1, (uint)rtc_32k_sm)) {
        captured_sequence = ~pio_sm_get(pio1, (uint)rtc_32k_sm);
        captured_ns = now_ns;
        captured = true;
    }
}
void rtc_32k_init(void) {
    rtc_32k_sm = pio_claim_unused_sm(pio1, true);
    uint offset = pio_add_program(pio1, &rtc_32k_program);
    pio_gpio_init(pio1, RTC_32KHZ_GPIO);
    gpio_pull_up(RTC_32KHZ_GPIO);
    pio_sm_config config = rtc_32k_program_get_default_config(offset);
    sm_config_set_in_pins(&config, RTC_32KHZ_GPIO);
    pio_sm_set_consecutive_pindirs(pio1, (uint)rtc_32k_sm, RTC_32KHZ_GPIO, 1, false);
    pio_sm_init(pio1, (uint)rtc_32k_sm, offset, &config);
    pio_sm_put_blocking(pio1, (uint)rtc_32k_sm, 32767u);
    irq_set_exclusive_handler(PIO1_IRQ_0, rtc_32k_irq);
    pio_set_irq0_source_enabled(pio1, pis_interrupt0, true);
    irq_set_enabled(PIO1_IRQ_0, true);
    pio_sm_set_enabled(pio1, (uint)rtc_32k_sm, true);
}
bool rtc_32k_take(uint64_t *timestamp_ns, uint32_t *sequence) {
    uint32_t saved = save_and_disable_interrupts();
    bool ready = captured;
    if (ready) { *timestamp_ns = captured_ns; *sequence = captured_sequence; captured = false; }
    restore_interrupts(saved);
    return ready;
}
#else
void rtc_32k_init(void) {}
bool rtc_32k_take(uint64_t *timestamp_ns, uint32_t *sequence) { (void)timestamp_ns; (void)sequence; return false; }
#endif
#endif
