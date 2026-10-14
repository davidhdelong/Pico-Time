"""Fault injection against the actual I2C and leap-fetch functions in main.c."""
import pathlib
import subprocess
import tempfile

source = pathlib.Path("main.c").read_text()
def section(start, end):
    return start + source.split(start, 1)[1].split(end, 1)[0]

i2c = section("static void rtc_i2c_recover(void)", "static bool rtc_write_reg")
dns = section("static void leap_dns_cb(", "#endif")
abort = section("static void leap_fetch_abort(void)", "static bool leap_parse_file")
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
typedef unsigned uint;
#define RTC_I2C_ID 0
#define RTC_SDA_PIN 1
#define RTC_SCL_PIN 2
#define GPIO_IN 0
#define GPIO_OUT 1
#define RTC_I2C_TIMEOUT_US 20000u
#define PICO_ERROR_TIMEOUT (-1)
#define DBG_PRINTF(...) ((void)0)
#define ERR_OK 0
#define ERR_INPROGRESS (-5)
#define IPADDR_TYPE_V4 0
#define LEAPFILE_HOST "test.invalid"
typedef int err_t;
typedef int ip_addr_t;
struct tcp_pcb { int id; } pcbs[8];
static struct {
    struct tcp_pcb *pcb;
    bool in_progress, dns_pending;
    uintptr_t attempt;
    ip_addr_t addr;
    uint64_t start_us, last_attempt_us;
    size_t len;
    bool truncated;
} g_leap_fetch;
static uint64_t now;
static int result, recoveries, elapsed_us, clock_lows, creates, closes, connects;
static bool stuck_scl;
static void i2c_deinit(int id) { assert(id==RTC_I2C_ID);++recoveries; }
static void rtc_init(void) {}
static void gpio_init(uint pin) { assert(pin==1||pin==2); }
static void gpio_put(uint pin,bool value) { assert((pin==1||pin==2)&&!value); }
static void gpio_set_dir(uint pin,bool output) { if(pin==2&&output)++clock_lows; }
static void gpio_pull_up(uint pin) { assert(pin==1||pin==2); }
static bool gpio_get(uint pin) { assert(pin==2);return !stuck_scl; }
static void sleep_us(uint us) { elapsed_us+=(int)us; }
static int i2c_write_timeout_us(int id,uint8_t address,const uint8_t *p,size_t n,bool nostop,uint timeout) {
    (void)address;(void)p;(void)n;(void)nostop;assert(id==0&&timeout==20000);return result;
}
static int i2c_read_timeout_us(int id,uint8_t address,uint8_t *p,size_t n,bool nostop,uint timeout) {
    (void)address;(void)p;(void)n;(void)nostop;assert(id==0&&timeout==20000);return result;
}
/* I2C */
static void tcp_arg(struct tcp_pcb *p,void *arg) { assert(p);(void)arg; }
static void tcp_recv(struct tcp_pcb *p,void *cb) { assert(p);(void)cb; }
static void tcp_err(struct tcp_pcb *p,void *cb) { assert(p);(void)cb; }
static int tcp_close(struct tcp_pcb *p) { assert(p);++closes;return 0; }
/* ABORT */
static void *leap_tcp_recv, *leap_tcp_err, *leap_tcp_connected;
static struct tcp_pcb *tcp_new_ip_type(int type) { assert(type==0);return &pcbs[creates++]; }
static int tcp_connect(struct tcp_pcb *p,const ip_addr_t *addr,int port,void *cb) {
    assert(p&&addr&&port==80);(void)cb;++connects;return 0;
}
static uint64_t time_us_64(void) { return now; }
static int dns_result=ERR_INPROGRESS;
static void (*pending_cb)(const char *,const ip_addr_t *,void *);
static void *pending_arg;
static int dns_gethostbyname(const char *name,ip_addr_t *out,
                            void (*cb)(const char *,const ip_addr_t *,void *),void *arg) {
    assert(!strcmp(name,LEAPFILE_HOST));*out=7;pending_cb=cb;pending_arg=arg;return dns_result;
}
/* DNS */
int main(void) {
    uint8_t data=0;result=1;
    assert(rtc_i2c_write(0x68,&data,1,false)==1&&!recoveries);
    result=PICO_ERROR_TIMEOUT;
    assert(rtc_i2c_write(0x68,&data,1,true)==PICO_ERROR_TIMEOUT);
    assert(recoveries==1&&clock_lows==10&&elapsed_us<=200);
    clock_lows=elapsed_us=0;stuck_scl=true;
    assert(rtc_i2c_read(0x68,&data,1,false)==PICO_ERROR_TIMEOUT);
    assert(recoveries==2&&clock_lows==2&&elapsed_us<200);
    result=1;assert(rtc_i2c_read(0x68,&data,1,false)==1&&recoveries==2);
    result=-2;assert(rtc_i2c_write(0x68,&data,1,false)==-2&&recoveries==2);
    puts("I2C: deadlines, timeout failure propagation, nine recovery clocks, stuck SCL and retry passed");

    ip_addr_t address=7;
    leap_fetch_start();void *old=pending_arg;
    assert(g_leap_fetch.in_progress&&g_leap_fetch.dns_pending);
    now=10000001;leap_fetch_poll();assert(!g_leap_fetch.in_progress);
    pending_cb(LEAPFILE_HOST,&address,old);assert(!creates&&!connects);
    leap_fetch_start();void *current=pending_arg;assert(current!=old);
    pending_cb(LEAPFILE_HOST,NULL,old);assert(g_leap_fetch.in_progress);
    pending_cb(LEAPFILE_HOST,&address,old);assert(!creates);
    pending_cb(LEAPFILE_HOST,&address,current);assert(creates==1&&connects==1&&!g_leap_fetch.dns_pending);
    pending_cb(LEAPFILE_HOST,&address,current);assert(creates==1);
    now+=10000001;leap_fetch_poll();assert(closes==1&&!g_leap_fetch.pcb);
    dns_result=ERR_OK;leap_fetch_start();assert(creates==2&&connects==2);
    leap_fetch_abort();dns_result=-2;leap_fetch_start();assert(!g_leap_fetch.in_progress&&creates==2);
    dns_result=ERR_INPROGRESS;leap_fetch_start();pending_cb(LEAPFILE_HOST,NULL,pending_arg);
    assert(!g_leap_fetch.in_progress);
    puts("DNS: timeout, stale success/failure, duplicate callback, cached result and failure passed");
}
'''.replace("/* I2C */", i2c).replace("/* DNS */", dns).replace("/* ABORT */", abort)

watchdog = section("#ifndef WATCHDOG_TIMEOUT_MS", "#ifndef RTC_I2C_TIMEOUT_US")
with tempfile.TemporaryDirectory(prefix="pico-review-fixes-") as folder:
    binary = str(pathlib.Path(folder) / "test")
    for flags in ([], ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]):
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", *flags,
                        "-x", "c", "-", "-o", binary], input=harness, text=True, check=True)
        subprocess.run([binary], check=True)
    for timeout in (8000, 0, 30000):
        result = subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                                 f"-DWATCHDOG_TIMEOUT_MS={timeout}", "-x", "c", "-", "-o", binary],
                                input=watchdog+"\nint main(void){return 0;}\n", text=True, capture_output=True)
        assert (result.returncode == 0) == (timeout == 8000), result.stderr
    print("Watchdog: supported default accepted; zero and 30-second overrides rejected")
