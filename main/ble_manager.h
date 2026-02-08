#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include "metrics.h"
#include <stdbool.h>
#include <stdint.h>

// BLE Advertisement packet structure
// BLE Score Advertisement Packet (optimized for 31-byte BLE limit)
// Reduced to 17 bytes to fit in manufacturer data (22 byte AD structure - 4
// byte overhead = 18 bytes, but getting 17) Structure:
// [Length(1)][Type(1)][CompanyID(2)][OurData(17)] = 21 bytes total
typedef struct __attribute__((packed)) {
  uint16_t company_id;   // 2 bytes - Company Identifier (0x02E5 for Espressif)
  uint32_t node_id;      // 4 bytes - Node identifier
  float score;           // 4 bytes - Composite score
  uint16_t battery;      // 2 bytes - Battery level (0-10000 = 0.00-100.00%)
  uint16_t trust;        // 2 bytes - Trust score (0-10000 = 0.00-1.00)
  uint16_t link_quality; // 2 bytes - Link quality (0-10000 = 0.00-1.00)
  uint8_t wifi_mac[2]; // 2 bytes - WiFi MAC address (last 2 bytes only, prefix
                       // can be derived)
  bool is_ch;          // 1 byte - Is Cluster Head
  uint8_t seq_num;     // 1 byte - Sequence number for PER calculation
  uint8_t hmac[1];     // 1 byte - Truncated HMAC
} ble_score_packet_t;  // Total: 20 bytes (2+4+4+2+2+2+2+1+1+1)

/**
 * @brief Initialize BLE manager
 */
void ble_manager_init(void);

/**
 * @brief Start BLE extended advertising
 */
void ble_manager_start_advertising(void);

/**
 * @brief Stop BLE advertising
 */
void ble_manager_stop_advertising(void);

/**
 * @brief Start BLE scanning
 */
void ble_manager_start_scanning(void);

/**
 * @brief Stop BLE scanning
 */
void ble_manager_stop_scanning(void);

/**
 * @brief Update advertisement with current metrics
 */
void ble_manager_update_advertisement(void);

/**
 * @brief Check if BLE is initialized
 */
bool ble_manager_is_ready(void);

#endif // BLE_MANAGER_H
