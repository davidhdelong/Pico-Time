idf_component_register(
    SRCS "main.c" "pps_monitor.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_mcpwm esp_wifi esp_event esp_netif esp_http_server esp_timer nvs_flash json
)
