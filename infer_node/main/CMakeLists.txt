# Wi-Fi, clock, SD mount, pins and app_config.h are shared with did_node: compiled
# from ../../main as they are, so app_config.h stays the only place for the
# Wi-Fi / server settings. Everything specific to inference lives here.
idf_component_register(
    SRCS
        "infer_app.c"
        "infer_health.c"
        "mic_infer.c"
        "cam_infer.c"
        "sample_store.c"
        "sample_upload.c"
        "result_report.c"
        "task_usage.c"
        "audio/audio_model.cc"
        "audio/logmel_features_int8.c"
        "audio/model_data_int8.cc"
        "vision/cam_model.cc"
        "vision/fd_mix.c"
        "vision/stage2.c"
        "vision/model.cc"
        "../../main/camera_registers.c"
        "../../main/device_identity.c"
        "../../main/node_storage.c"
        "../../main/status_led.c"
        "../../main/time_sync.c"
        "../../main/wifi_manager.c"
    INCLUDE_DIRS "." "audio" "vision" "../../main"
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
        tflite-micro
)

# stage2.h: history is only for drawing tracks on the PC.
target_compile_definitions(${COMPONENT_LIB} PRIVATE STAGE2_DISABLE_HISTORY)

# cam_infer.c calls the bundled camera HAL pause/resume API.
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../components/esp32-camera/driver/private_include")
