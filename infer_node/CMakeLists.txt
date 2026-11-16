# ESP32 (TTGO T8) inference firmware, ported from DID_NODE/infer_node (ESP32-S3).
# One board runs either the camera or the microphone pipeline (main/infer_config.h);
# results go to DID_NODE/server_mix.py right away, low-confidence samples are stored
# on SD and uploaded in batches. Independent of ../main (the recording firmware);
# Wi-Fi / server settings and a few shared drivers are compiled from there.
cmake_minimum_required(VERSION 3.16)

# ../components: locally patched esp32-camera (pause/resume), same copy as ../main.
# components/: tflite-micro + esp-nn (from DID_NODE/infer_node/components).
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")

# Which TFLM kernels use ESP-NN; see components/tflite-micro/CMakeLists.txt.
#   auto (default) = all, for both roles:
#     mic: TTGO_I2S_Record_Test used ESP-NN; on the board CNN 1433 ms -> 278 ms, outputs OK.
#     cam: FD_did_cam_v4 once got constant (uniform) softmax output with ESP-NN on the ESP32.
#          If the cam CNN probabilities look stuck, fall back to -DESP_NN_KERNEL_SET=none,
#          or bisect with dw / conv / fc / arith / pool / softmax.
#   Several groups are comma-separated, e.g. all except fc: dw,conv,arith,pool,softmax
#   Override (sticks in the CMake cache until changed back with -DESP_NN_KERNEL_SET=auto):
#     idf.py -B build -DESP_NN_KERNEL_SET=none build
set(ESP_NN_KERNEL_SET "auto" CACHE STRING "ESP-NN kernels: auto/none/all or comma-separated dw/conv/fc/arith/pool/softmax")
if(ESP_NN_KERNEL_SET STREQUAL "auto")
    set(ESP_NN_KERNEL_SET "all")
endif()
message(STATUS "ESP-NN kernel set: ${ESP_NN_KERNEL_SET}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(did_infer_node_esp32)
