# Select one firmware at configure time:
#   idf.py -D FIELD_FIRMWARE=audio build
#   idf.py -D FIELD_FIRMWARE=camera build
set(FIELD_FIRMWARE "audio" CACHE STRING "Firmware variant: audio or camera")
set_property(CACHE FIELD_FIRMWARE PROPERTY STRINGS audio camera)
string(TOLOWER "${FIELD_FIRMWARE}" FIELD_FIRMWARE)

set(common_sources
    "device_identity.c"
    "wifi_manager.c"
    "time_sync.c"
)

set(common_requires
    driver
    esp_event
    esp_http_client
    esp_netif
    esp_timer
    esp_wifi
    fatfs
    json
    nvs_flash
    sdmmc
)

if(FIELD_FIRMWARE STREQUAL "audio")
    set(firmware_sources
        "app_main.c"
        "audio_record.c"
        "storage.c"
        "upload.c"
        "health.c"
    )
    set(firmware_requires "")
elseif(FIELD_FIRMWARE STREQUAL "camera")
    set(firmware_sources
        "cam_app_main.c"
        "cam_record.c"
        "camera_registers.c"
        "cam_storage.c"
        "cam_upload.c"
        "cam_health.c"
    )
    set(firmware_requires
        esp32-camera
    )
else()
    message(FATAL_ERROR
        "Invalid FIELD_FIRMWARE='${FIELD_FIRMWARE}'. Use 'audio' or 'camera'.")
endif()

message(STATUS "Building FIELD_FIRMWARE=${FIELD_FIRMWARE}")

idf_component_register(
    SRCS ${firmware_sources} ${common_sources}
    INCLUDE_DIRS "."
    REQUIRES ${common_requires}
    PRIV_REQUIRES ${firmware_requires}
)
