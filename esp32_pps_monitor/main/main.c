#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/mcpwm_cap.h"
#include "esp_attr.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "private_wifi.h"
#include "dashboard_asset.h"
#include "pps_monitor.h"

#ifndef PPS_L76K_GPIO
#define PPS_L76K_GPIO 0
#endif
#ifndef PPS_UBX_GPIO
#define PPS_UBX_GPIO 1
#endif
#ifndef PPS_XIAO_ANTENNA_EXTERNAL
#define PPS_XIAO_ANTENNA_EXTERNAL 0
#endif
_Static_assert(PPS_L76K_GPIO != PPS_UBX_GPIO, "PPS inputs must use distinct pins");
_Static_assert(PPS_L76K_GPIO != 12 && PPS_L76K_GPIO != 13 && PPS_UBX_GPIO != 12 && PPS_UBX_GPIO != 13,
               "Keep native USB pins free");
_Static_assert(PPS_L76K_GPIO != 3 && PPS_L76K_GPIO != 14 && PPS_UBX_GPIO != 3 && PPS_UBX_GPIO != 14,
               "Keep XIAO RF-switch pins free");
static const char *TAG = "pps-monitor";
static const int pins[2] = {PPS_L76K_GPIO, PPS_UBX_GPIO};
static const char *names[2] = {"L76K", "UBX"};
static DRAM_ATTR uint8_t capture_ids[2] = {0, 1};
static QueueHandle_t capture_queue;
static SemaphoreHandle_t state_mutex;
static pps_monitor_t monitor;
static uint32_t queue_drops;
static bool wifi_connected;
static uint32_t wifi_disconnects;
static char ip_address[16] = "0.0.0.0";

static bool IRAM_ATTR on_capture(mcpwm_cap_channel_handle_t channel,
                                const mcpwm_capture_event_data_t *data, void *context) {
    (void)channel;
    pps_edge_t edge = {.ticks = data->cap_value, .observed_us = esp_timer_get_time(),
                      .input = *(const uint8_t *)context, .rising = data->cap_edge == MCPWM_CAP_EDGE_POS};
    BaseType_t woke = pdFALSE;
    if (xQueueSendFromISR(capture_queue, &edge, &woke) != pdTRUE)
        __atomic_add_fetch(&queue_drops, 1, __ATOMIC_RELAXED);
    return woke == pdTRUE;
}
static void capture_task(void *context) {
    (void)context;
    uint32_t last_drops = 0;
    for (;;) {
        pps_edge_t edge;
        bool have = xQueueReceive(capture_queue, &edge, pdMS_TO_TICKS(100)) == pdTRUE;
        uint32_t drops = __atomic_load_n(&queue_drops, __ATOMIC_RELAXED);
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        if (drops != last_drops) { pps_monitor_invalidate(&monitor); last_drops = drops; }
        if (have) {
            uint64_t now = esp_timer_get_time();
            if (now >= edge.observed_us && now - edge.observed_us < PPS_FRESH_US)
                pps_monitor_edge(&monitor, &edge);
            else pps_monitor_invalidate(&monitor);
        }
        if (!have) pps_monitor_poll(&monitor, esp_timer_get_time());
        xSemaphoreGive(state_mutex);
    }
}
static void start_capture(void) {
    capture_queue = xQueueCreate(64, sizeof(pps_edge_t));
    assert(capture_queue);
    mcpwm_cap_timer_handle_t timer;
    mcpwm_capture_timer_config_t config = {.group_id = 0, .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
                                          .resolution_hz = 80000000};
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&config, &timer));
    uint32_t resolution;
    ESP_ERROR_CHECK(mcpwm_capture_timer_get_resolution(timer, &resolution));
    pps_monitor_init(&monitor, resolution);
    for (unsigned i = 0; i < 2; ++i) {
        mcpwm_cap_channel_handle_t channel;
        mcpwm_capture_channel_config_t c = {.gpio_num = pins[i], .intr_priority = 3, .prescale = 1,
            .flags = {.pos_edge = true, .neg_edge = true, .pull_down = true}};
        ESP_ERROR_CHECK(mcpwm_new_capture_channel(timer, &c, &channel));
        mcpwm_capture_event_callbacks_t cb = {.on_cap = on_capture};
        ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(channel, &cb, &capture_ids[i]));
        ESP_ERROR_CHECK(mcpwm_capture_channel_enable(channel));
    }
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(timer));
    assert(xTaskCreate(capture_task, "pps-capture", 4096, NULL, 15, NULL) == pdPASS);
    ESP_LOGI(TAG, "Hardware capture: %"PRIu32" Hz; L76K GPIO%d, UBX GPIO%d", resolution, pins[0], pins[1]);
}
static void wifi_event(void *context, esp_event_base_t base, int32_t id, void *event) {
    (void)context;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false; ++wifi_disconnects;
        strcpy(ip_address, "0.0.0.0");
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = event;
        snprintf(ip_address, sizeof(ip_address), IPSTR, IP2STR(&e->ip_info.ip));
        wifi_connected = true;
        ESP_LOGI(TAG, "Dashboard http://%s/", ip_address);
    }
    xSemaphoreGive(state_mutex);
}
static void wifi_task(void *context) {
    (void)context;
    for (;;) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        bool connected = wifi_connected;
        xSemaphoreGive(state_mutex);
        if (!connected) esp_wifi_connect();
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
static void start_wifi(void) {
    /* Seeed XIAO ESP32-C6: GPIO3 low powers the RF switch. GPIO14 low
     * selects the onboard antenna; high selects the external connector. */
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_3, 0));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_3, GPIO_MODE_OUTPUT));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_14, PPS_XIAO_ANTENNA_EXTERNAL ? 1 : 0));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    assert(netif);
    ESP_ERROR_CHECK(esp_netif_set_hostname(netif, "pps-monitor"));
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL));
    wifi_config_t wifi = {0};
    _Static_assert(sizeof(PPS_WIFI_SSID) <= sizeof(wifi.sta.ssid) + 1, "SSID too long");
    _Static_assert(sizeof(PPS_WIFI_PASSWORD) <= sizeof(wifi.sta.password) + 1, "Password too long");
    memcpy(wifi.sta.ssid, PPS_WIFI_SSID, sizeof(PPS_WIFI_SSID) - 1);
    memcpy(wifi.sta.password, PPS_WIFI_PASSWORD, sizeof(PPS_WIFI_PASSWORD) - 1);
    wifi.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi.sta.pmf_cfg.capable = true;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    assert(xTaskCreate(wifi_task, "wifi-retry", 3072, NULL, 3, NULL) == pdPASS);
}
static void number(cJSON *o, const char *key, double value, bool valid) {
    if (valid && isfinite(value)) cJSON_AddNumberToObject(o, key, value);
    else cJSON_AddNullToObject(o, key);
}
static pps_monitor_t *copy_monitor(void) {
    pps_monitor_t *copy = malloc(sizeof(*copy));
    if (copy) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        *copy = monitor;
        xSemaphoreGive(state_mutex);
    }
    return copy;
}
static esp_err_t health_handler(httpd_req_t *req) {
    pps_monitor_t *copy = copy_monitor();
    if (!copy) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory unavailable");
    uint64_t now = esp_timer_get_time();
    pps_summary_t summary;
    pps_monitor_summary(copy, now, &summary);
    cJSON *root = cJSON_CreateObject();
    if (!root) { free(copy); return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory unavailable"); }
    cJSON_AddStringToObject(root, "version", "1.0.2");
    cJSON_AddStringToObject(root, "device", "ESP32-C6 PPS alignment monitor");
    cJSON_AddNumberToObject(root, "uptime_s", now / 1000000.0);
    cJSON_AddBoolToObject(root, "valid", summary.valid);
    cJSON_AddNumberToObject(root, "resolution_hz", copy->resolution_hz);
    cJSON_AddNumberToObject(root, "tick_ns", 1000000000.0 / copy->resolution_hz);
    cJSON_AddNumberToObject(root, "window_capacity", PPS_HISTORY);
    cJSON_AddNumberToObject(root, "samples", copy->count);
    cJSON_AddNumberToObject(root, "segment", copy->segment);
    cJSON_AddNumberToObject(root, "drift_samples", summary.drift_samples);
    cJSON_AddBoolToObject(root, "statistics_historical", !summary.valid && copy->count > 0);
    cJSON_AddNumberToObject(root, "total_pairs", copy->pairs);
    cJSON_AddNumberToObject(root, "unpaired", copy->unpaired);
    cJSON_AddNumberToObject(root, "invalidations", copy->invalidations);
    cJSON_AddNumberToObject(root, "queue_drops", __atomic_load_n(&queue_drops, __ATOMIC_RELAXED));
    number(root, "offset_us", summary.latest_us, summary.valid);
    number(root, "mean_us", summary.mean_us, copy->count > 0);
    number(root, "median_us", summary.median_us, copy->count > 0);
    number(root, "min_us", summary.min_us, copy->count > 0);
    number(root, "max_us", summary.max_us, copy->count > 0);
    number(root, "stddev_us", summary.stddev_us, copy->count > 0);
    number(root, "rms_us", summary.rms_us, copy->count > 0);
    number(root, "peak_to_peak_us", summary.max_us - summary.min_us, copy->count > 0);
    number(root, "drift_ppm", summary.drift_ppm, summary.drift_valid);
    number(root, "pair_age_ms", summary.age_us / 1000.0, copy->count > 0);
    cJSON_AddNumberToObject(root, "free_heap", esp_get_free_heap_size());
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    cJSON_AddBoolToObject(root, "wifi_connected", wifi_connected);
    cJSON_AddNumberToObject(root, "wifi_disconnects", wifi_disconnects);
    cJSON_AddStringToObject(root, "ip", ip_address);
    bool connected = wifi_connected;
    xSemaphoreGive(state_mutex);
    wifi_ap_record_t ap;
    bool have_rssi = esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
    number(root, "wifi_rssi", have_rssi ? ap.rssi : 0, connected && have_rssi);
    cJSON *channels = cJSON_AddArrayToObject(root, "channels");
    for (unsigned j = 0; j < 2; ++j) {
        const pps_input_t *i = &copy->input[j];
        cJSON *ch = cJSON_CreateObject();
        cJSON_AddItemToArray(channels, ch);
        cJSON_AddStringToObject(ch, "name", names[j]);
        cJSON_AddNumberToObject(ch, "gpio", pins[j]);
        cJSON_AddBoolToObject(ch, "present", pps_input_fresh(i, now));
        cJSON_AddBoolToObject(ch, "qualified", pps_input_fresh(i, now) && i->good_intervals >= 2);
        cJSON_AddNumberToObject(ch, "edges", i->edges);
        cJSON_AddNumberToObject(ch, "glitches", i->glitches);
        cJSON_AddNumberToObject(ch, "missed", pps_input_missed(i, now));
        number(ch, "outage_ms", now >= i->rise_us + 1000000ULL ? (double)(now - i->rise_us - 1000000ULL) / 1000.0 : 0,
               i->seen && !pps_input_fresh(i, now));
        number(ch, "age_ms", (double)(now - i->rise_us) / 1000.0, i->seen && now >= i->rise_us);
        number(ch, "period_us", i->period_us, i->have_period && pps_input_fresh(i, now));
        number(ch, "width_us", i->width_us, i->have_width && pps_input_fresh(i, now));
    }
    cJSON *history = cJSON_AddArrayToObject(root, "history");
    for (uint32_t j = 0; j < copy->count; ++j) {
        const pps_pair_t *p = pps_monitor_pair(copy, j);
        cJSON *row = cJSON_CreateArray();
        cJSON_AddItemToArray(row, cJSON_CreateNumber((double)(now - p->observed_us) / 1000000.0));
        cJSON_AddItemToArray(row, cJSON_CreateNumber(p->offset_us));
        cJSON_AddItemToArray(row, cJSON_CreateNumber(p->segment));
        cJSON_AddItemToArray(history, row);
    }
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root); free(copy);
    if (!json) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory unavailable");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t result = httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    free(json);
    return result;
}
static esp_err_t page_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Vary", "Accept-Encoding");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    char encoding[96];
    if (httpd_req_get_hdr_value_str(req, "Accept-Encoding", encoding, sizeof(encoding)) == ESP_OK &&
        strstr(encoding, "gzip") && !strstr(encoding, "gzip;q=0")) {
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)monitor_dashboard_gzip, sizeof(monitor_dashboard_gzip));
    }
    return httpd_resp_send(req, monitor_dashboard, HTTPD_RESP_USE_STRLEN);
}
static esp_err_t reset_handler(httpd_req_t *req) {
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    pps_monitor_clear_stats(&monitor);
    xSemaphoreGive(state_mutex);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}
static esp_err_t csv_handler(httpd_req_t *req) {
    pps_monitor_t *copy = copy_monitor();
    if (!copy) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory unavailable");
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=pps-alignment.csv");
    esp_err_t result = httpd_resp_sendstr_chunk(req, "uptime_s,ubx_minus_l76k_us,segment\n");
    for (uint32_t j = 0; j < copy->count && result == ESP_OK; ++j) {
        const pps_pair_t *p = pps_monitor_pair(copy, j);
        char line[96];
        int n = snprintf(line, sizeof(line), "%.6f,%.6f,%"PRIu64"\n", p->observed_us / 1000000.0, p->offset_us, p->segment);
        result = httpd_resp_send_chunk(req, line, n);
    }
    free(copy);
    if (result == ESP_OK) result = httpd_resp_send_chunk(req, NULL, 0);
    return result;
}
static void start_http(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192; config.max_open_sockets = 4; config.lru_purge_enable = true;
    config.recv_wait_timeout = 3; config.send_wait_timeout = 3;
    httpd_handle_t server;
    ESP_ERROR_CHECK(httpd_start(&server, &config));
    httpd_uri_t routes[] = {
        {.uri = "/", .method = HTTP_GET, .handler = page_handler},
        {.uri = "/api/health", .method = HTTP_GET, .handler = health_handler},
        {.uri = "/api/reset", .method = HTTP_POST, .handler = reset_handler},
        {.uri = "/api/history.csv", .method = HTTP_GET, .handler = csv_handler},
    };
    for (unsigned j = 0; j < sizeof(routes) / sizeof(routes[0]); ++j)
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &routes[j]));
}
void app_main(void) {
    state_mutex = xSemaphoreCreateMutex(); assert(state_mutex);
    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); nvs = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs);
    start_capture(); start_wifi(); start_http();
    ESP_LOGI(TAG, "Ready: positive offset means UBX arrives after L76K. Inputs must be 3.3 V.");
}
