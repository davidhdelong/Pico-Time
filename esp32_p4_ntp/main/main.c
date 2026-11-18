#include <assert.h>
#include <math.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/socket.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gpio_etm.h"
#include "driver/gptimer.h"
#include "driver/gptimer_etm.h"
#include "driver/uart.h"
#include "esp_etm.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "lwip/inet.h"
#include "board.h"
#include "gnss.h"
#include "p4_clock.h"
#include "../../timezone_lookup.h"
#include "../../generated/timezone_data.h"

static const char *TAG = "pico-time-p4";
static gptimer_handle_t edge_timer, read_timer;
static uint64_t read_origin_ns, origin_bound_ns;
static QueueHandle_t pps_queue, uart_queue;
static SemaphoreHandle_t state_mutex;
static p4_clock_t clock_state;
static p4_gnss_t gnss_state;
static bool link_up, ip_ready;
static bool receiver_configured = !P4_GNSS_UBX;
static char ip_address[16] = "0.0.0.0";
static uint64_t position_ns;
static uint32_t queue_drops, ntp_requests, ntp_bad, ntp_limited, ntp_errors, http_requests, http_errors, uart_errors;
static const char *dashboard =
    "<!doctype html><html lang=en><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>Pico Time · Ethernet</title><style>body{background:#101820;color:#eef5fa;font:16px system-ui;margin:3em auto;padding:0 1em;max-width:55em}"
    "section{background:#1a2934;padding:1.5em;border-radius:1em}dt{color:#a5bbca}dd{margin:0 0 1em;font-size:1.2em}a{color:#8bd5ff}</style>"
    "<h1>Pico Time · ESP32-P4 Ethernet</h1><p id=status>Connecting</p><section><dl>"
    "<dt>UTC snapshot</dt><dd id=utc>—</dd><dt>Local time</dt><dd id=local>—</dd>"
    "<dt>Clock uncertainty</dt><dd id=error>—</dd><dt>GPS / PPS</dt><dd id=gps>—</dd>"
    "<dt>Timezone data</dt><dd id=tz>—</dd><dt>Ethernet</dt><dd id=eth>—</dd><dt>Timing counters</dt><dd id=counters>—</dd></dl></section>"
    "<p>UTC timing uses GNSS and hardware-captured PPS. Local time is presentation only."
    " Ethernet packet timestamps are software timestamps; this server does not advertise PTP or hardware NTP packet timestamping.</p>"
    "<a href=/api/health>Health JSON</a><script>const put=(id,v)=>document.getElementById(id).textContent=v;"
    "async function poll(){if(document.hidden){setTimeout(poll,2000);return}const ac=new AbortController(),t=setTimeout(()=>ac.abort(),4000);"
    "try{let r=await fetch('/api/health',{cache:'no-store',signal:ac.signal});if(!r.ok)throw Error(r.status);let d=await r.json();"
    "put('status',d.clock_state+' · '+d.gnss+' · firmware '+d.version);put('utc',d.utc||'Unqualified');put('local',d.local_time||'Unavailable');"
    "put('error',d.clock_uncertainty_us===null?'Unqualified':d.clock_uncertainty_us.toFixed(1)+' µs');"
    "put('gps',(d.fix?'Fix':'No fix')+' · '+d.sats+' satellites · PPS '+(d.pps?'present':'absent'));"
    "put('tz',d.timezone_name+' · rules '+d.timezone_data_version+' / boundaries '+d.timezone_boundary_version);put('eth',d.ethernet_link?d.ip:'Link down');put('counters',d.ntp_requests+' NTP requests · '+d.pps_capture_faults+' capture faults · '+d.queue_drops+' queue drops');"
    "}catch(e){put('status','Device unavailable · displayed values are historical')}finally{clearTimeout(t);setTimeout(poll,2000)}}poll()</script></html>";

static uint64_t now_ns(void) {
    uint64_t ticks = 0;
    ESP_ERROR_CHECK(gptimer_get_raw_count(read_timer, &ticks));
    return read_origin_ns + ticks * P4_TICK_NS;
}
static void IRAM_ATTR on_pps(void *arg) {
    (void)arg;
    uint64_t ticks = 0;
    if (gptimer_get_captured_count(edge_timer, &ticks) != ESP_OK ||
        xQueueSendFromISR(pps_queue, &ticks, NULL) != pdTRUE)
        __atomic_add_fetch(&queue_drops, 1, __ATOMIC_RELAXED);
}
static void start_timers(void) {
    /* Software raw reads trigger a capture. Give them a separate timer so
     * HTTP/NTP cannot overwrite the PPS capture register before its ISR reads
     * it. Both timers use the same clock source and 10 MHz divider. */
    gptimer_config_t cfg = {.clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP, .resolution_hz = P4_TIMER_HZ};
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &edge_timer));
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &read_timer));
    uint32_t edge_hz, read_hz;
    ESP_ERROR_CHECK(gptimer_get_resolution(edge_timer, &edge_hz));
    ESP_ERROR_CHECK(gptimer_get_resolution(read_timer, &read_hz));
    assert(edge_hz == P4_TIMER_HZ && read_hz == P4_TIMER_HZ);
    ESP_ERROR_CHECK(gptimer_enable(edge_timer)); ESP_ERROR_CHECK(gptimer_enable(read_timer));
    ESP_ERROR_CHECK(gptimer_start(edge_timer));
    uint64_t before, after;
    ESP_ERROR_CHECK(gptimer_get_raw_count(edge_timer, &before));
    ESP_ERROR_CHECK(gptimer_start(read_timer));
    ESP_ERROR_CHECK(gptimer_get_raw_count(edge_timer, &after));
    read_origin_ns = (before + (after - before) / 2u) * P4_TICK_NS;
    origin_bound_ns = (after - before) * P4_TICK_NS + 2u * P4_TICK_NS;
    p4_clock_init(&clock_state, origin_bound_ns);
    pps_queue = xQueueCreate(16, sizeof(uint64_t)); assert(pps_queue);
    gpio_config_t io = {.pin_bit_mask = 1ULL << P4_GPS_PPS, .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, .intr_type = GPIO_INTR_POSEDGE};
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_etm_event_config_t ge = {.edge = GPIO_ETM_EVENT_EDGE_POS};
    esp_etm_event_handle_t event;
    ESP_ERROR_CHECK(gpio_new_etm_event(&ge, &event));
    ESP_ERROR_CHECK(gpio_etm_event_bind_gpio(event, P4_GPS_PPS));
    gptimer_etm_task_config_t tc = {.task_type = GPTIMER_ETM_TASK_CAPTURE};
    esp_etm_task_handle_t task;
    ESP_ERROR_CHECK(gptimer_new_etm_task(edge_timer, &tc, &task));
    esp_etm_channel_config_t cc = {0}; esp_etm_channel_handle_t channel;
    ESP_ERROR_CHECK(esp_etm_new_channel(&cc, &channel));
    ESP_ERROR_CHECK(esp_etm_channel_connect(channel, event, task));
    ESP_ERROR_CHECK(esp_etm_channel_enable(channel));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(P4_GPS_PPS, on_pps, NULL));
    ESP_LOGI(TAG, "PPS GPIO%d; capture resolution %u ns; clock-map bound %llu ns", P4_GPS_PPS,
             P4_TICK_NS, (unsigned long long)origin_bound_ns);
}
static void timing_task(void *arg) {
    (void)arg; uint32_t previous_drops = 0;
    for (;;) {
        uint64_t ticks;
        if (xQueueReceive(pps_queue, &ticks, pdMS_TO_TICKS(100)) != pdTRUE) continue;
        uint64_t now = now_ns(), ns = ticks * P4_TICK_NS;
        uint32_t drops = (unsigned)__atomic_load_n(&queue_drops, __ATOMIC_RELAXED);
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        if (drops != previous_drops || now < ns || now - ns > 200000000ULL) {
            p4_clock_fault(&clock_state); previous_drops = drops;
            /* Retire the whole untrusted stream, not just its last entry. */
            while (xQueueReceive(pps_queue, &ticks, 0) == pdTRUE) {}
        } else p4_clock_pps(&clock_state, ns);
        xSemaphoreGive(state_mutex);
    }
}
static void ethernet_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    (void)arg;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (base == ETH_EVENT) {
        if (id == ETHERNET_EVENT_CONNECTED) link_up = true;
        if (id == ETHERNET_EVENT_DISCONNECTED || id == ETHERNET_EVENT_STOP) {
            link_up = ip_ready = false; strcpy(ip_address, "0.0.0.0");
        }
    } else if (base == IP_EVENT && id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *e = data;
        snprintf(ip_address, sizeof(ip_address), IPSTR, IP2STR(&e->ip_info.ip));
        ip_ready = true; ESP_LOGI(TAG, "NTP UDP/123; dashboard http://%s/", ip_address);
    } else if (base == IP_EVENT && id == IP_EVENT_ETH_LOST_IP) ip_ready = false;
    xSemaphoreGive(state_mutex);
}
static void start_ethernet(void) {
    ESP_ERROR_CHECK(esp_netif_init()); ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, ethernet_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, ethernet_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_LOST_IP, ethernet_event, NULL));
    eth_mac_config_t mc = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t ec = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    ec.smi_gpio.mdc_num = P4_ETH_MDC; ec.smi_gpio.mdio_num = P4_ETH_MDIO;
    eth_phy_config_t pc = ETH_PHY_DEFAULT_CONFIG();
    pc.phy_addr = -1; pc.reset_gpio_num = P4_ETH_PHY_RESET;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&ec, &mc);
    /* IDF 6's IEEE 802.3 generic PHY driver supports the onboard IP101. */
    esp_eth_phy_t *phy = esp_eth_phy_new_generic(&pc); assert(mac && phy);
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy); esp_eth_handle_t eth;
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth));
    esp_netif_config_t nc = ESP_NETIF_DEFAULT_ETH(); esp_netif_t *netif = esp_netif_new(&nc); assert(netif);
    ESP_ERROR_CHECK(esp_netif_set_hostname(netif, "pico-time-p4"));
    ESP_ERROR_CHECK(esp_netif_attach(netif, esp_eth_new_netif_glue(eth)));
    ESP_ERROR_CHECK(esp_eth_start(eth));
}
static void accept_gnss(p4_gnss_t msg, uint64_t first_ns) {
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    /* RMC has no satellite count; preserve the most recent GGA count. */
    if (msg.satellites_valid) gnss_state.satellites = msg.satellites;
    gnss_state.fix = msg.fix;
    if (msg.position_valid) {
        gnss_state.latitude = msg.latitude; gnss_state.longitude = msg.longitude;
        gnss_state.position_valid = true; position_ns = first_ns;
    }
    if (msg.time_valid && receiver_configured) (void)p4_clock_utc(&clock_state, msg.epoch_s, first_ns);
    xSemaphoreGive(state_mutex);
}
#if !P4_GNSS_UBX
static void send_nmea(const char *body) {
    unsigned ck = 0; for (const char *p = body; *p; ++p) ck ^= (unsigned char)*p;
    char frame[128]; int n = snprintf(frame, sizeof(frame), "$%s*%02X\r\n", body, ck);
    assert(n > 0 && n < (int)sizeof(frame));
    uart_write_bytes(UART_NUM_1, frame, n); ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(200)));
}
#endif
static void gnss_task(void *arg) {
    (void)arg;
    static ubx_parser_t parser; static ubx_frame_t frame;
    ubx_parser_init(&parser);
    char line[192]; size_t used = 0; uint64_t first_ns = 0;
    for (;;) {
        uart_event_t event;
        while (xQueueReceive(uart_queue, &event, 0) == pdTRUE) {
            if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL || event.type == UART_FRAME_ERR || event.type == UART_PARITY_ERR) {
                uart_flush_input(UART_NUM_1); used = 0; ubx_parser_init(&parser);
                __atomic_add_fetch(&uart_errors, 1, __ATOMIC_RELAXED);
                xSemaphoreTake(state_mutex, portMAX_DELAY); p4_clock_fault(&clock_state); xSemaphoreGive(state_mutex);
            }
        }
        uint8_t byte;
        if (uart_read_bytes(UART_NUM_1, &byte, 1, pdMS_TO_TICKS(10)) != 1) continue;
        uint64_t received = now_ns();
        if (P4_GNSS_UBX) {
            if (ubx_parser_feed(&parser, byte, received, &frame)) {
                bool accepted;
                if (p4_gnss_config_ack(&frame, &accepted)) {
                    xSemaphoreTake(state_mutex, portMAX_DELAY);
                    receiver_configured = accepted;
                    if (!accepted) p4_clock_fault(&clock_state);
                    xSemaphoreGive(state_mutex);
                    ESP_LOGI(TAG, "Receiver UTC/PPS configuration %s", accepted ? "accepted" : "REJECTED");
                }
                p4_gnss_t msg; if (p4_gnss_ubx(&frame, &msg)) accept_gnss(msg, frame.first_byte_ns);
            }
        } else {
            if (byte == '$') { used = 0; first_ns = received; }
            if (byte == '\r') continue;
            if (byte == '\n') {
                if (used) { line[used] = 0; p4_gnss_t msg; if (p4_gnss_nmea(line, &msg)) accept_gnss(msg, first_ns); }
                used = 0; first_ns = 0;
            } else if (first_ns && used < sizeof(line) - 1) line[used++] = (char)byte;
            else { used = 0; first_ns = 0; }
        }
    }
}
static void start_gnss(void) {
    uart_config_t cfg = {.baud_rate = P4_GNSS_UBX ? 115200 : 9600,
        .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, P4_GPS_TX, P4_GPS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 4096, 0, 32, &uart_queue, 0));
#if !P4_GNSS_UBX
    vTaskDelay(pdMS_TO_TICKS(200)); send_nmea("PCAS01,5");
    vTaskDelay(pdMS_TO_TICKS(150)); ESP_ERROR_CHECK(uart_set_baudrate(UART_NUM_1, 115200));
    send_nmea("PCAS02,1000"); send_nmea("PCAS03,1,0,1,0,1,0,0,0,0,0,,,0,0");
#else
    uint8_t config_frame[160]; size_t n = p4_gnss_ubx_config(config_frame, sizeof(config_frame)); assert(n);
    vTaskDelay(pdMS_TO_TICKS(200));
    uart_write_bytes(UART_NUM_1, config_frame, n);
    ESP_ERROR_CHECK(uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(200)));
#endif
    assert(xTaskCreate(gnss_task, "gnss", 8192, NULL, 12, NULL) == pdPASS);
}
typedef struct { uint32_t address; uint64_t window_ns; unsigned count; bool used; } rate_entry_t;
static bool rate_ok(rate_entry_t *entries, uint32_t address, uint64_t now) {
    unsigned slot = 0;
    for (unsigned i = 0; i < 32; ++i) {
        if (entries[i].used && entries[i].address == address) { slot = i; goto found; }
        if (!entries[i].used || entries[i].window_ns < entries[slot].window_ns) slot = i;
    }
    entries[slot] = (rate_entry_t){.address = address, .window_ns = now, .used = true};
found:
    if (now - entries[slot].window_ns >= 1000000000ULL) { entries[slot].window_ns = now; entries[slot].count = 0; }
    return ++entries[slot].count <= 16;
}
static void ntp_task(void *arg) {
    (void)arg; static rate_entry_t rates[32];
    for (;;) {
        int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        struct sockaddr_in local = {.sin_family = AF_INET, .sin_port = htons(123), .sin_addr.s_addr = htonl(INADDR_ANY)};
        if (fd < 0 || bind(fd, (struct sockaddr *)&local, sizeof(local)) != 0) {
            if (fd >= 0) close(fd);
            __atomic_add_fetch(&ntp_errors, 1, __ATOMIC_RELAXED); vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }
        for (;;) {
            uint8_t request[513], reply[48]; struct sockaddr_in peer; socklen_t size = sizeof(peer);
            int n = recvfrom(fd, request, sizeof(request), 0, (struct sockaddr *)&peer, &size);
            uint64_t rx = now_ns(); if (n < 0) break;
            __atomic_add_fetch(&ntp_requests, 1, __ATOMIC_RELAXED);
            if (!rate_ok(rates, peer.sin_addr.s_addr, rx)) { __atomic_add_fetch(&ntp_limited, 1, __ATOMIC_RELAXED); continue; }
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            bool good = p4_ntp_reply(&clock_state, request, (size_t)n, rx, now_ns(), reply);
            xSemaphoreGive(state_mutex);
            if (!good) { __atomic_add_fetch(&ntp_bad, 1, __ATOMIC_RELAXED); continue; }
            if (sendto(fd, reply, sizeof(reply), 0, (struct sockaddr *)&peer, size) != sizeof(reply))
                __atomic_add_fetch(&ntp_errors, 1, __ATOMIC_RELAXED);
        }
        close(fd); __atomic_add_fetch(&ntp_errors, 1, __ATOMIC_RELAXED);
    }
}
static esp_err_t page_handler(httpd_req_t *req) {
    __atomic_add_fetch(&http_requests, 1, __ATOMIC_RELAXED);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    esp_err_t result = httpd_resp_sendstr(req, dashboard);
    if (result != ESP_OK) __atomic_add_fetch(&http_errors, 1, __ATOMIC_RELAXED);
    return result;
}
static void format_time(int64_t epoch, char *out, size_t n) {
    time_t t = (time_t)epoch; struct tm value;
    if (gmtime_r(&t, &value)) strftime(out, n, "%Y-%m-%dT%H:%M:%S", &value);
    else if (n) out[0] = 0;
}
static esp_err_t health_handler(httpd_req_t *req) {
    __atomic_add_fetch(&http_requests, 1, __ATOMIC_RELAXED);
    uint64_t now = now_ns(); p4_sample_t sample; p4_gnss_t gps;
    uint64_t faults, rejects, position_age; uint32_t period, samples; bool link, ready, configured;
    char ip[16];
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    sample = p4_clock_sample(&clock_state, now); gps = gnss_state;
    faults = clock_state.capture_faults; rejects = clock_state.rejected;
    period = clock_state.period_ns; samples = clock_state.samples;
    position_age = now >= position_ns ? now - position_ns : UINT64_MAX;
    link = link_up; ready = ip_ready; memcpy(ip, ip_address, sizeof(ip));
    configured = receiver_configured;
    xSemaphoreGive(state_mutex);
    /* All geographic/rule lookup is presentation-only and outside the timing
     * mutex. The const tables stay in flash; no update client exists. */
    timezone_id_t zone = TIMEZONE_ID_UTC; int32_t offset = 0; bool dst = false;
    bool located = gps.position_valid && position_age < 10000000000ULL;
    if (located) zone = timezone_from_coordinates((int32_t)llround(gps.latitude * 1e7), (int32_t)llround(gps.longitude * 1e7));
    bool rules = sample.time.valid && timezone_get_offset(zone, sample.time.unix_sec, &offset, &dst);
    char utc[32] = "", local[32] = "", uncertainty[40], unix_s[40];
    if (sample.time.valid) {
        format_time(sample.time.unix_sec, utc, sizeof(utc));
        if (rules) format_time(sample.time.unix_sec + offset, local, sizeof(local));
        snprintf(unix_s, sizeof(unix_s), "%lld", (long long)sample.time.unix_sec);
    } else strcpy(unix_s, "null");
    if (sample.locked) snprintf(uncertainty, sizeof(uncertainty), "%.3f", sample.uncertainty.total_ns / 1000.0);
    else strcpy(uncertainty, "null");
    char body[2304]; int n = snprintf(body, sizeof(body),
        "{\"version\":\"0.1.0\",\"device\":\"ESP32-P4 Ethernet NTP server\",\"gnss\":\"%s\",\"receiver_configured\":%s,"
        "\"unix\":%s,\"utc\":%s%s%s,\"local_time\":%s%s%s,\"clock_state\":\"%s\",\"synced\":%s,"
        "\"sntp_stratum\":%u,\"sntp_refid\":\"%s\",\"clock_uncertainty_us\":%s,"
        "\"pps\":%s,\"fix\":%s,\"sats\":%u,\"pps_period_ns\":%u,\"pps_samples\":%u,"
        "\"pps_capture_faults\":%llu,\"pps_rejected\":%llu,\"queue_drops\":%u,\"uart_errors\":%u,"
        "\"ethernet_link\":%s,\"ip_ready\":%s,\"ip\":\"%s\",\"uptime_s\":%llu,"
        "\"ntp_requests\":%u,\"ntp_malformed\":%u,\"ntp_rate_limited\":%u,\"ntp_send_errors\":%u,\"http_requests\":%u,\"http_errors\":%u,"
        "\"timezone_name\":\"%s\",\"timezone_lookup_valid\":%s,\"timezone_data_version\":\"%s\","
        "\"timezone_boundary_version\":\"%s\",\"timezone_data_start_year\":%d,\"timezone_data_end_year\":%d,"
        "\"timezone_offset_seconds\":%ld,\"timezone_is_dst\":%s,\"timezone_rules_out_of_range\":%s,"
        "\"pps_gpio\":%d,\"gps_rx_gpio\":%d,\"gps_tx_gpio\":%d,\"capture_tick_ns\":%u,\"capture_map_bound_ns\":%llu}",
        P4_GNSS_UBX ? "UBX" : "L76K", configured ? "true" : "false", unix_s,
        sample.time.valid ? "\"" : "", sample.time.valid ? utc : "null", sample.time.valid ? "\"" : "",
        rules ? "\"" : "", rules ? local : "null", rules ? "\"" : "",
        sample.locked ? "LOCKED" : sample.pps_present ? "ACQUIRING" : "UNSYNCED", sample.locked ? "true" : "false",
        sample.locked ? 1u : 16u, sample.locked ? "GPS " : "LOCL", uncertainty,
        sample.pps_present ? "true" : "false", gps.fix && sample.message_fresh ? "true" : "false", gps.satellites, (unsigned)period, (unsigned)samples,
        (unsigned long long)faults, (unsigned long long)rejects, (unsigned)__atomic_load_n(&queue_drops, __ATOMIC_RELAXED),
        (unsigned)__atomic_load_n(&uart_errors, __ATOMIC_RELAXED), link ? "true" : "false", ready ? "true" : "false", ip,
        (unsigned long long)(now / 1000000000ULL), (unsigned)__atomic_load_n(&ntp_requests, __ATOMIC_RELAXED),
        (unsigned)__atomic_load_n(&ntp_bad, __ATOMIC_RELAXED), (unsigned)__atomic_load_n(&ntp_limited, __ATOMIC_RELAXED),
        (unsigned)__atomic_load_n(&ntp_errors, __ATOMIC_RELAXED), (unsigned)__atomic_load_n(&http_requests, __ATOMIC_RELAXED),
        (unsigned)__atomic_load_n(&http_errors, __ATOMIC_RELAXED),
        timezone_name(zone), located ? "true" : "false", TZDB_VERSION, TZ_BOUNDARY_VERSION,
        TZ_DATA_START_YEAR, TZ_DATA_END_YEAR, (long)offset, dst ? "true" : "false",
        sample.time.valid && !rules ? "true" : "false", P4_GPS_PPS, P4_GPS_RX, P4_GPS_TX,
        P4_TICK_NS, (unsigned long long)origin_bound_ns);
    if (n < 0 || n >= (int)sizeof(body)) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Health response too large");
    httpd_resp_set_type(req, "application/json"); httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t result = httpd_resp_send(req, body, n);
    if (result != ESP_OK) __atomic_add_fetch(&http_errors, 1, __ATOMIC_RELAXED);
    return result;
}
void app_main(void) {
    state_mutex = xSemaphoreCreateMutex(); assert(state_mutex);
    start_timers();
    assert(xTaskCreate(timing_task, "pps", 4096, NULL, 18, NULL) == pdPASS);
    start_gnss(); start_ethernet();
    assert(xTaskCreate(ntp_task, "ntp-server", 6144, NULL, 10, NULL) == pdPASS);
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG(); cfg.stack_size = 8192; cfg.max_open_sockets = 4;
    cfg.lru_purge_enable = true; cfg.recv_wait_timeout = cfg.send_wait_timeout = 3;
    httpd_handle_t http; ESP_ERROR_CHECK(httpd_start(&http, &cfg));
    httpd_uri_t home = {.uri = "/", .method = HTTP_GET, .handler = page_handler};
    httpd_uri_t health = {.uri = "/api/health", .method = HTTP_GET, .handler = health_handler};
    ESP_ERROR_CHECK(httpd_register_uri_handler(http, &home)); ESP_ERROR_CHECK(httpd_register_uri_handler(http, &health));
    ESP_LOGI(TAG, "GNSS %s: RX GPIO%d TX GPIO%d PPS GPIO%d; Ethernet only", P4_GNSS_UBX ? "UBX" : "L76K", P4_GPS_RX, P4_GPS_TX, P4_GPS_PPS);
}
