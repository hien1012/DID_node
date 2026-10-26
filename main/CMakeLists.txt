idf_component_register(
    SRCS
        "app_main.c"
        "rec_session.c"
        "cam_record.c"
        "cam_stream.c"
        "camera_registers.c"
        "mic_record.c"
        "node_storage.c"
        "node_upload.c"
        "node_health.c"
        "wifi_manager.c"
        "time_sync.c"
        "device_identity.c"
        "status_led.c"
        "net_bench.c"
        "timing_trace.c"
        "la_probe.c"
        "sysview_uart_guard.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        lwip
        esp_event
        esp_http_client
        esp_netif
        esp_timer
        esp_wifi
        esp32-camera
        fatfs
        json
        nvs_flash
        sdmmc
        spi_flash
)

# SystemView over UART (sdkconfig.sysview_uart) only: route SEGGER_RTT_ReadNoLock
# through the guard in sysview_uart_guard.c (IDF 5.1.2 calls the flash-resident
# UART driver from ISRs, which panics while the cache is disabled).
if(CONFIG_APPTRACE_SV_DEST_UART)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=SEGGER_RTT_ReadNoLock")
endif()

# cam_record.c calls the bundled camera HAL pause/resume API.
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${PROJECT_DIR}/components/esp32-camera/driver/private_include")
