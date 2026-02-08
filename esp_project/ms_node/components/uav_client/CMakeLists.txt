idf_component_register(
    SRCS "uav_client.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_http_client mbedtls json esp_event log
)
