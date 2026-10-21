# ESP32-S3 field node: camera + microphone recorded together, uploaded after
# every session. The old ESP32 audio/camera firmware is on branch esp32-legacy.
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(did_node)

# SystemView over UART (sdkconfig.sysview_uart): IDF 5.1.2 SEGGER_RTT_esp.c still
# uses the SystemView 2.x names USER_START/STOP, renamed MARK_START/STOP (same
# IDs 15/16) in the bundled SystemView 3.x header. Map them without patching IDF.
if(CONFIG_APPTRACE_SV_DEST_UART)
    idf_component_get_property(app_trace_lib app_trace COMPONENT_LIB)
    target_compile_definitions(${app_trace_lib} PRIVATE
        SYSVIEW_EVTID_USER_START=SYSVIEW_EVTID_MARK_START
        SYSVIEW_EVTID_USER_STOP=SYSVIEW_EVTID_MARK_STOP)
endif()
