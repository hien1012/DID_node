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
        "la_isr_trace.S"
        "isr_profile.c"
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

# LA ISR probes (la_isr_trace.S): wrap the FreeRTOS interrupt entry/exit so ISR
# time drops the idle / task pins. Same condition as la_task_trace.h:
# APP_LA_PROBE_ENABLE 1 in app_config.h (read here; editing the file re-runs
# CMake, and la_probe.c fails the build if the two sides disagree) and no SystemView.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/app_config.h")
file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/app_config.h" la_probe_on
    REGEX "^#define[ \t]+APP_LA_PROBE_ENABLE[ \t]+1")
if(la_probe_on AND NOT CONFIG_APPTRACE_SV_ENABLE)
    target_link_libraries(${COMPONENT_LIB} INTERFACE
        "-Wl,--wrap=_frxt_int_enter" "-Wl,--wrap=_frxt_int_exit")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE LA_ISR_WRAP=1)
endif()

# C ISR profiler (isr_profile.c): wrap esp_intr_alloc_intrstatus and
# xt_set_interrupt_handler when APP_ISR_PROFILE_ENABLE is 1 in app_config.h.
file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/app_config.h" isr_profile_on
    REGEX "^#define[ \t]+APP_ISR_PROFILE_ENABLE[ \t]+1")
if(isr_profile_on)
    target_link_libraries(${COMPONENT_LIB} INTERFACE
        "-Wl,--wrap=esp_intr_alloc"
        "-Wl,--wrap=esp_intr_alloc_intrstatus"
        "-Wl,--wrap=xt_set_interrupt_handler")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ISR_PROFILE_WRAP=1)
endif()

# cam_record.c calls the bundled camera HAL pause/resume API.
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${PROJECT_DIR}/components/esp32-camera/driver/private_include")
