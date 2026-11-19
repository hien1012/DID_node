# Wi-Fi, clock, SD mount, pins and app_config.h are shared with did_node: compiled
# from ../../main as they are, so app_config.h stays the only place for the
# Wi-Fi / server settings. Everything specific to inference lives here.
idf_component_register(
    SRCS
        "infer_app.c"
        "infer_health.c"
        "mic_infer.c"
        "cam_infer.c"
        "debug_dump.c"
        "debug_replay.c"
        "debug_tensors.cc"
        "sample_store.c"
        "sample_upload.c"
        "result_report.c"
        "task_usage.c"
        "audio/audio_model.cc"
        "audio/audio_debug.c"
        "audio/logmel_features_int8.c"
        "audio/model_data_int8.cc"
        "vision/cam_debug.c"
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

# Audio debug dump (audio/audio_debug.c). Use a separate build directory so the
# normal build stays untouched:  idf.py -B build_debug -DINFER_AUDIO_DEBUG_DUMP=1 build flash
if(INFER_AUDIO_DEBUG_DUMP)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE INFER_AUDIO_DEBUG_DUMP=1)
endif()

# Camera debug dump (vision/cam_debug.c), same idea:
#   idf.py -B build_cam_debug -DINFER_CAM_DEBUG_DUMP=1 -DESP_NN_KERNEL_SET=all build flash
if(INFER_CAM_DEBUG_DUMP)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE INFER_CAM_DEBUG_DUMP=1)
endif()

# SD replay (debug_replay.c): same inputs from /sdcard/debug/in, once per kernel set:
#   idf.py -B build_replay_nn  -DINFER_DEBUG_REPLAY=1 -DESP_NN_KERNEL_SET=all  build flash
#   idf.py -B build_replay_ref -DINFER_DEBUG_REPLAY=1 -DESP_NN_KERNEL_SET=none build flash
if(INFER_DEBUG_REPLAY)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE INFER_DEBUG_REPLAY=1)
endif()

# Microphone high-pass filter (infer_config.h INFER_MIC_HPF_ENABLE); both 0 and 1 are
# passed through so an explicit -DINFER_MIC_HPF_ENABLE=0 overrides the header default:
#   idf.py -B build_hpf -DINFER_MIC_HPF_ENABLE=1 build flash
if(DEFINED INFER_MIC_HPF_ENABLE)
    if(INFER_MIC_HPF_ENABLE)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE INFER_MIC_HPF_ENABLE=1)
    else()
        target_compile_definitions(${COMPONENT_LIB} PRIVATE INFER_MIC_HPF_ENABLE=0)
    endif()
endif()

# The dump meta records which kernels this build uses (the ESP_NN define itself is
# private to the tflite-micro component).
if(DEFINED ESP_NN_KERNEL_SET)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE "INFER_ESP_NN_KERNEL_SET=\"${ESP_NN_KERNEL_SET}\"")
endif()

# cam_infer.c calls the bundled camera HAL pause/resume API.
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../components/esp32-camera/driver/private_include")
