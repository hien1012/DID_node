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
)

# cam_record.c calls the bundled camera HAL pause/resume API.
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${PROJECT_DIR}/components/esp32-camera/driver/private_include")
