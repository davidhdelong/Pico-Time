"""Fault injection against actual bounded I2C functions and watchdog settings."""
import pathlib
import subprocess
import tempfile

source = pathlib.Path("main.c").read_text()
def section(start, end):
    return start + source.split(start, 1)[1].split(end, 1)[0]

i2c = section("static void rtc_i2c_recover(void)", "static bool rtc_write_reg")
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
static int result, recoveries, elapsed_us, clock_lows;
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


}
'''.replace("/* I2C */", i2c)

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
