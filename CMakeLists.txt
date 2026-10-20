# ESP32 (TTGO T8) firmware: audio or camera node (branch esp32-legacy)
#   idf.py -B build -D FIELD_FIRMWARE=audio build
#   idf.py -B build -D FIELD_FIRMWARE=camera build
cmake_minimum_required(VERSION 3.16)

set(FIELD_FIRMWARE "audio" CACHE STRING "Firmware variant: audio or camera")
# 兩種韌體各用自己的 sdkconfig（camera 需要 PSRAM 與較大的 app 分割）。
# 共用一個 sdkconfig 時，先 build 的那種會留下完整設定，另一種的 defaults 就不會套用。
set(SDKCONFIG "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.${FIELD_FIRMWARE}")
if(FIELD_FIRMWARE STREQUAL "camera")
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.camera.defaults")
endif()

# Locally patched esp32-camera in components/.
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/components"
    $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(esp_http_client)
