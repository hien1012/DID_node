# DID node inference firmware: microphone and camera each capture and run their
# own model; results go to server_mix.py right away and low-confidence samples
# are stored on SD and uploaded in batches. Independent of ../main (did_node),
# which keeps recording full sessions; shared drivers are compiled from there.
cmake_minimum_required(VERSION 3.16)

# Locally patched esp32-camera (pause/resume), the same copy did_node uses.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")

# Which TFLM kernels use ESP-NN; see components/tflite-micro/CMakeLists.txt.
#   idf.py -B build -DESP_NN_KERNEL_SET=all build
set(ESP_NN_KERNEL_SET "none" CACHE STRING "ESP-NN kernels: none/all/dw/conv/fc/arith/pool/softmax")

# App version = git describe, like ESP-IDF's default, but ASCII only: a tag with non-ASCII
# characters (e.g. Chinese) ends up in build/project_description.json, which idf.py reads
# with the Windows code page (cp950) and fails ("Cannot load ... project_description.json").
execute_process(
    COMMAND git describe --tags --always --dirty
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    OUTPUT_VARIABLE INFER_GIT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE INFER_GIT_RESULT)
if(INFER_GIT_RESULT EQUAL 0 AND INFER_GIT_VERSION)
    string(REGEX REPLACE "[^ -~]+" "_" PROJECT_VER "${INFER_GIT_VERSION}")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(did_infer_node)
