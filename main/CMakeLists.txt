# Embed the server root certificate into the final binary
#
# (If this was a component, we would set COMPONENT_EMBED_TXTFILES here.)
set(requires "")
idf_build_get_property(target IDF_TARGET)

if(${target} STREQUAL "linux")
    list(APPEND requires esp_stubs esp-tls esp_http_client protocol_examples_common nvs_flash)
endif()
idf_component_register(SRCS "cam_app_main.c" "cam_record.c" "device_identity.c" "app_main.c"
                            "audio_record.c"
                            "storage.c"
                            "wifi_manager.c"
                            "time_sync.c"
                            "upload.c"
                            "health.c"
                    INCLUDE_DIRS "."
                    REQUIRES ${requires} driver esp_event esp_http_client esp_netif
                            esp_timer esp_wifi fatfs json nvs_flash sdmmc)
