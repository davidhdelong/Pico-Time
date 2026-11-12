/* Execute the assembled, shipped PIO instructions with deterministic pins.
 * Stable two-instruction loops are fast-forwarded; all capture, transition,
 * wrap, PUSH and IRQ instructions execute individually. */
#include <assert.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "pps_capture_timing.h"
#define PICO_NO_HARDWARE 1
#include "pps_capture.pio.h"

typedef struct { uint32_t ticks; uint64_t cycle; } capture_t;
typedef struct {
    unsigned pc, count;
    uint32_t x, isr;
    uint64_t cycle, capture_cycle;
    bool rxstall;
    capture_t fifo[4];
} machine_t;
static const uint32_t hz = 50000000;
static void step(machine_t *s, bool pin) {
    assert(s->pc < sizeof(pps_capture_program_instructions)/sizeof(uint16_t));
    uint16_t op = pps_capture_program_instructions[s->pc];
    unsigned next = s->pc == pps_capture_wrap ? pps_capture_wrap_target : s->pc+1;
    assert((op & 0x1f00u) == 0); /* no delay/side-set in this program */
    if ((op >> 13) == 0) {
        unsigned condition = (op >> 5) & 7;
        bool jump;
        switch (condition) {
            case 0: jump=true; break;
            case 2: jump=s->x!=0; --s->x; break;
            case 6: jump=pin; break;
            default: assert(false); jump=false;
        }
        if (jump) next=op&31;
    } else if (op == 0xa02b) s->x=UINT32_MAX; /* MOV X, ~NULL */
    else if (op == 0xa0c1) {s->isr=s->x; s->capture_cycle=s->cycle;} /* MOV ISR, X */
    else if (op == 0x8020) { /* PUSH BLOCK, four-word RX FIFO */
        if (s->count == 4) {s->rxstall=true;next=s->pc;}
        else {s->fifo[s->count++]=(capture_t){s->isr,s->capture_cycle};s->isr=0;}
    } else if (op != 0xc000) assert(false); /* IRQ NOWAIT 0: never stalls */
    s->pc=next; ++s->cycle;
}
typedef struct {
    capture_t first, previous;
    uint64_t remainder, reconstructed_ns, uncorrected_ns, count, wraps;
} stream_t;
static void consume(machine_t *s, stream_t *out) {
    if (!s->count) return;
    capture_t sample=s->fifo[0];
    --s->count;memmove(s->fifo,s->fifo+1,s->count*sizeof(capture_t));
    if (!out->count) out->first=sample;
    else {
        uint64_t elapsed=pps_capture_elapsed_ns(out->previous.ticks,sample.ticks,hz,&out->remainder);
        assert(elapsed==(sample.cycle-out->previous.cycle)*10);
        out->reconstructed_ns+=elapsed;
        out->uncorrected_ns+=(uint64_t)(uint32_t)(out->previous.ticks-sample.ticks)*20;
        if (sample.ticks>out->previous.ticks) ++out->wraps;
    }
    ++out->count;out->previous=sample;
}
static void run(machine_t *s, uint64_t until, uint64_t period, uint64_t rise,
                uint64_t width, stream_t *out) {
    assert(rise+width<period && width>=10);
    while(s->cycle<until) {
        uint64_t phase=s->cycle%period;
        bool pin=phase>=rise && phase<rise+width;
        uint64_t next=s->cycle-phase+(phase<rise?rise:phase<rise+width?rise+width:period+rise);
        /* These pairs are asserted against assembled code, not comments. */
        if ((s->pc==1&&!pin)||(s->pc==7&&pin)) {
            assert(pps_capture_program_instructions[1]==0x00c4);
            assert(pps_capture_program_instructions[2]==0x0041);
            assert(pps_capture_program_instructions[7]==0x00c9);
            assert(pps_capture_program_instructions[9]==0x0047);
            uint64_t loops=(next-s->cycle)/2;
            if(loops>(until-s->cycle)/2)loops=(until-s->cycle)/2;
            if(loops>s->x)loops=s->x; /* execute underflow individually */
            if(loops){s->x-=(uint32_t)loops;s->cycle+=2*loops;continue;}
        }
        if(s->pc==5&&s->count==4&&!out){s->rxstall=true;s->cycle=until;break;}
        step(s,pin);
        if(out)consume(s,out);
    }
}
static void simulate_day(void) {
    machine_t s={0};stream_t stream={0};
    run(&s,86401ULL*100000000,100000000,1000,10000000,&stream);
    uint64_t elapsed=(stream.previous.cycle-stream.first.cycle)*10;
    assert(stream.count==86401 && stream.wraps>1000);
    assert(stream.reconstructed_ns==elapsed);
    assert(elapsed-stream.uncorrected_ns==86400ULL*60+stream.wraps*10);
    printf("24h PIO simulation: captures=%"PRIu64", wraps=%"PRIu64
           ", old error=-%"PRIu64" ns, corrected accumulated error=0 ns\n",
           stream.count,stream.wraps,elapsed-stream.uncorrected_ns);
}
static void transitions_and_wrap(void) {
    /* Force wrap through each part of the low/high/edge path, and vary
     * pulse alignment and width, including odd numbers of PIO clocks. */
    for(uint32_t seed=0;seed<160;++seed) {
        for(uint64_t width=10;width<40;++width) {
            machine_t s={0};stream_t stream={0};step(&s,false);s.x=seed;
            run(&s,4000,101,7,width,&stream);
            assert(stream.count>30 && !s.rxstall);
        }
    }
    puts("Counter wrap and odd/even pulse widths/alignment passed");
}
static void fifo_stall(void) {
    machine_t s={0};run(&s,7000,1000,10,100,NULL);
    assert(s.rxstall&&s.count==4&&s.pc==5);
    uint32_t stopped=s.x;run(&s,17000,1000,10,100,NULL);assert(s.x==stopped);
    /* A stalled timebase MUST be discarded, not overhead-corrected. Model
     * the hardware restart performed by pps_capture_service. */
    uint64_t now=s.cycle;memset(&s,0,sizeof(s));s.cycle=now;
    stream_t stream={0};run(&s,now+10000,1000,10,100,&stream);
    assert(!s.rxstall&&stream.count>=9);
    puts("RX FIFO stall stops counter; reset starts a clean capture stream");
}
static void first_capture_mapping(void) {
    /* Execute real assembled instructions, including multiple counter wraps
     * before any PPS exists. Arrival delay must not change the mapped edge. */
    const uint64_t rises[] = {7, 99999999, 8589934600ULL, 50000000000ULL};
    for (unsigned j = 0; j < sizeof(rises)/sizeof(rises[0]); ++j) {
        machine_t s = {0}; stream_t stream = {0};
        uint64_t rise = rises[j];
        run(&s, rise + 20, rise + 1000, rise, 10, &stream);
        assert(stream.count == 1);
        for (uint64_t delay = 1000; delay <= 1000000000ULL; delay *= 1000) {
            uint64_t stamp = 0, rem = 0, start = 123456789ULL;
            uint64_t expected = start + stream.first.cycle * 10;
            assert(pps_capture_first_ns(stream.first.ticks, hz, start,
                                       expected + delay, &stamp, &rem));
            assert(stamp == expected && rem == 0);
        }
    }
    uint64_t stamp = 42, rem = 17;
    assert(!pps_capture_first_ns(0, 0, 1, 2, &stamp, &rem));
    assert(!pps_capture_first_ns(0, hz, 10, 9, &stamp, &rem));
    assert(!pps_capture_first_ns(UINT32_MAX, hz, 10, 2000000010ULL, &stamp, &rem));
    assert(stamp == 42 && rem == 17); /* rejection leaves output untouched */
    assert(pps_capture_first_ns(UINT32_MAX-1u, 30000000, 0, 1000, &stamp, &rem));
    assert(stamp == 66 && rem == 40000000);
    stamp += pps_capture_elapsed_ns(UINT32_MAX-1u, UINT32_MAX-11u, 30000000, &rem);
    assert(stamp == 500 && rem == 0); /* first-edge fraction carries forward */
    puts("First PPS hardware epoch: real PIO, delayed service, long acquisition and wrap mapping passed");
}
static void arithmetic(void) {
    uint32_t div=0;
    assert(pps_capture_divider(150000000,50000000,&div)&&div==384);
    assert(pps_capture_divider(150000000,10000000,&div)&&div==1920);
    assert(!pps_capture_divider(150000000,49000000,&div));
    assert(!pps_capture_divider(150000000,100000000,&div));
    assert(!pps_capture_divider(150000000,0,&div));
    uint64_t rem=0;
    assert(pps_capture_elapsed_ns(100,90,10000000,&rem)==1300);
    assert(pps_capture_elapsed_ns(2,UINT32_MAX-2,50000000,&rem)==170);
    /* Non-integral nanosecond periods carry fractions, rather than losing
     * the fractional part on every pulse. */
    rem=0;uint64_t sum=0;
    for(unsigned i=0;i<1000;++i)sum+=pps_capture_elapsed_ns(100,90,30000000,&rem);
    assert(sum==26000ULL*1000000000/60000000);
    rem=0;assert(pps_capture_elapsed_ns(UINT32_MAX,0,1,&rem)>0);
    assert(pps_capture_elapsed_ns(100,90,0,&rem)==UINT64_MAX);
}
int main(void){arithmetic();first_capture_mapping();transitions_and_wrap();fifo_stall();simulate_day();}
