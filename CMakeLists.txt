# ESP32-S3 field node: camera + microphone recorded together, uploaded after
# every session. The old ESP32 audio/camera firmware is on branch esp32-legacy.
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(did_node)
