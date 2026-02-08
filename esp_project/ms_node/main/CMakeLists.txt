    SRCS 
        "ms_node.c" 
        "compression_bench.c" 
        "ble_beacon.c" 
        "ble_gatt_service.c"
        "state_machine.c"
        "election.c"
        "metrics.c"
        "neighbor_manager.c"
        "esp_now_manager.c"
        "auth.c"
        "led_manager.c"
        "ble_manager.c"
        "persistence.c"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash esp_wifi esp_adc driver esp_timer mbedtls
    PRIV_REQUIRES sensors logger esp_timer pme battery spiffs compression bt nvs_flash esp_wifi driver mbedtls rf_receiver uav_client
