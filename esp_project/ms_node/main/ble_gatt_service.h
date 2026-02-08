#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize BLE GATT service for data retrieval
esp_err_t ble_gatt_service_init(void);

// Start/stop GATT service advertising
esp_err_t ble_gatt_service_start(void);
esp_err_t ble_gatt_service_stop(void);

#ifdef __cplusplus
}
#endif
