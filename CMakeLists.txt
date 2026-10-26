# ESP32-S3 field node: camera + microphone recorded together, uploaded after
# every session. The old ESP32 audio/camera firmware is on branch esp32-legacy.
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(did_node)

# LA task probes (main/la_task_trace.h): force-include into the freertos C sources
# so traceTASK_SWITCHED_IN/OUT() drive the idle pins and the bound task pins.
# The header checks APP_LA_PROBE_ENABLE itself and is empty when it is 0 or
# SystemView is on. Joined "-include<path>" form keeps the path (it contains a
# space) one argument; C only, the port's .S files must not see it.
idf_component_get_property(freertos_lib freertos COMPONENT_LIB)
target_compile_options(${freertos_lib} PRIVATE
    "$<$<COMPILE_LANGUAGE:C>:-include${CMAKE_CURRENT_LIST_DIR}/main/la_task_trace.h>")

# SystemView over UART (sdkconfig.sysview_uart): IDF 5.1.2 SEGGER_RTT_esp.c still
# uses the SystemView 2.x names USER_START/STOP, renamed MARK_START/STOP (same
# IDs 15/16) in the bundled SystemView 3.x header. Map them without patching IDF.
if(CONFIG_APPTRACE_SV_DEST_UART)
    idf_component_get_property(app_trace_lib app_trace COMPONENT_LIB)
    target_compile_definitions(${app_trace_lib} PRIVATE
        SYSVIEW_EVTID_USER_START=SYSVIEW_EVTID_MARK_START
        SYSVIEW_EVTID_USER_STOP=SYSVIEW_EVTID_MARK_STOP)
endif()
