set(ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
idf_component_register(SRCS "main.c" "p4_clock.c" "gnss.c"
    "${ROOT}/ubx.c" "${ROOT}/rtc_holdover.c" "${ROOT}/ntp_uncertainty.c"
    "${ROOT}/timezone_lookup.c" "${ROOT}/generated/timezone_data.c"
    INCLUDE_DIRS "."
    REQUIRES esp_eth esp_netif esp_event esp_http_server esp_driver_gpio
             esp_driver_gptimer esp_driver_uart esp_timer esp_system esp_hw_support)
if(NOT DEFINED P4_GNSS_UBX)
    set(P4_GNSS_UBX 0)
endif()
if(NOT P4_GNSS_UBX MATCHES "^[01]$")
    message(FATAL_ERROR "P4_GNSS_UBX must be 0 (L76K) or 1 (UBX)")
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE P4_GNSS_UBX=${P4_GNSS_UBX})
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
