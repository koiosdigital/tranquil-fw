file(GLOB_RECURSE NESTED_SRC
       **.cpp
)


list(APPEND INCLUDE_DIRS ".")
list(APPEND INCLUDE_DIRS "api")
list(APPEND INCLUDE_DIRS "api/dispatcher")
list(APPEND INCLUDE_DIRS "api/handlers")
list(APPEND INCLUDE_DIRS "api/state")
list(APPEND INCLUDE_DIRS "usb_pd")
list(APPEND INCLUDE_DIRS "storage")
list(APPEND INCLUDE_DIRS "storage/jobs")
list(APPEND INCLUDE_DIRS "storage/jobs/thumbnail")
list(APPEND INCLUDE_DIRS "drm")
list(APPEND INCLUDE_DIRS "sockets")

message(STATUS ${NESTED_SRC})

idf_component_register(
    SRCS ${NESTED_SRC}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    PRIV_REQUIRES lwip esp_wifi heap bootloader_support kd_common wifi_provisioning mbedtls esp_http_server kd_pixdriver stusb4500 sand_table tranquil-app vfs json esp_driver_sdmmc fatfs esp32-idf-sqlite3 esp_websocket_client
)

idf_build_set_property(COMPILE_OPTIONS "-Wno-missing-field-initializers" APPEND)

idf_build_set_property(COMPILE_OPTIONS "-DDEVICE_NAME_PREFIX=\"TRANQUIL\"" APPEND)