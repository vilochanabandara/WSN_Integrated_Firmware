#include "ble_manager.h"
#include "auth.h"
#include "config.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_nimble_hci.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "metrics.h"
#include "neighbor_manager.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include <string.h>

static const char *TAG = "BLE";

static bool ble_ready = false;
static bool advertising = false;
static bool scanning = false;
static uint8_t g_seq_num = 0; // Global sequence number for PER calculation

// Forward declarations
extern uint32_t g_node_id;
extern uint8_t g_cluster_key[CLUSTER_KEY_SIZE];
extern bool g_is_ch;

// BLE Extended Advertising Callbacks
static int ble_gap_event(struct ble_gap_event *event, void *arg);

// BLE host task callback
static void ble_host_task(void *param) {
  ESP_LOGI(TAG, "BLE host task started");
  nimble_port_run(); // This function will not return
}

// BLE host sync callback - called when host stack is ready
static void ble_on_sync(void) {
  ESP_LOGI(TAG, "BLE host synchronized");
  ble_ready = true;
}

// BLE host reset callback - called when host stack resets
static void ble_on_reset(int reason) {
  ESP_LOGI(TAG, "BLE host reset, reason: %d", reason);
  ble_ready = false;
}

void ble_manager_init(void) {
  ESP_LOGI(TAG, "Initializing BLE");
  ESP_LOGI(TAG, "sizeof(ble_score_packet_t) = %d", sizeof(ble_score_packet_t));

  // Register callbacks - MUST be set BEFORE nimble_port_init()
  ble_hs_cfg.sync_cb = ble_on_sync;
  ble_hs_cfg.reset_cb = ble_on_reset;

  // Initialize NimBLE port (this initializes both controller and host stack)
  esp_err_t ret = nimble_port_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize NimBLE port: %s", esp_err_to_name(ret));
    ESP_LOGE(TAG, "Error code: 0x%x", ret);
    return;
  }

  ESP_LOGI(TAG, "NimBLE port initialized");

  // Initialize GAP
  ble_svc_gap_init();

  // Set device name
  char device_name[32];
  snprintf(device_name, sizeof(device_name), "%s%lu", BLE_DEVICE_NAME_PREFIX,
           g_node_id);
  ble_svc_gap_device_name_set(device_name);

  ESP_LOGI(TAG, "Starting BLE host task");

  // Start BLE host task
  nimble_port_freertos_init(ble_host_task);

  ESP_LOGI(TAG, "BLE initialization complete (waiting for sync)");
}

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
  struct ble_gap_conn_desc desc;
  int rc;

  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    ESP_LOGI(TAG, "Connection %s; status=%d",
             event->connect.status == 0 ? "established" : "failed",
             event->connect.status);

    if (event->connect.status == 0) {
      rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
      assert(rc == 0);
    }
    return 0;

  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGI(TAG, "Disconnect: reason=%d", event->disconnect.reason);
    return 0;

  case BLE_GAP_EVENT_CONN_UPDATE:
    ESP_LOGI(TAG, "Connection updated");
    return 0;

  case BLE_GAP_EVENT_ADV_COMPLETE:
    ESP_LOGI(TAG, "Advertising complete");
    advertising = false;
    return 0;

  case BLE_GAP_EVENT_DISC_COMPLETE:
    ESP_LOGI(TAG, "Scan complete");
    scanning = false;
    return 0;

  case BLE_GAP_EVENT_DISC: {
    // Process discovered device
    struct ble_gap_disc_desc *disc = &event->disc;

    ESP_LOGI(TAG, "Discovery event received: data_len=%d, rssi=%d",
             disc->length_data, disc->rssi);

    // Parse advertising data to find manufacturer data (type 0xFF)
    // AD structure format: [Length][Type][Data...]
    const uint8_t *data = disc->data;
    size_t data_len = disc->length_data;
    size_t offset = 0;
    bool found_mfg_data = false;

    while (offset < data_len) {
      if (offset + 1 >= data_len)
        break; // Need at least length byte

      uint8_t ad_len = data[offset];
      if (ad_len == 0 || offset + ad_len > data_len)
        break; // Invalid length (ad_len includes length byte itself)

      uint8_t ad_type = data[offset + 1];

      // Check if this is manufacturer data (type 0xFF)
      if (ad_type == 0xFF &&
          ad_len >= 3) { // At least 2 bytes company ID + 1 byte data
        found_mfg_data = true;

        // BLE AD structure: [Length][Type][Data...]
        // ad_len is the length byte value, which includes: Type (1) + Data (N)
        // Data = Company ID (2) + Our Payload
        // Our Payload is now part of the struct (starting after company_id)
        // But the struct *includes* company_id at the beginning.
        // So if we cast the data pointer starting at Company ID to the struct,
        // it matches. Offset points to Length byte. Manufacturer Data Value
        // starts at: offset + 1 (Length) + 1 (Type) = offset + 2
        size_t mfg_data_offset = offset + 2;
        size_t mfg_data_len = ad_len - 1; // Length - Type byte = Data Length

        ESP_LOGI(
            TAG,
            "Mfg data: ad_len=%d, data_portion=%d, our_data_len=%d, offset=%d",
            ad_len, ad_len - 2, mfg_data_len, mfg_data_offset);

        // Safety check: make sure we have enough data
        // Note: mfg_data_len is size_t (unsigned), so can't be < 0
        if (mfg_data_len == 0 || mfg_data_offset + mfg_data_len > data_len) {
          ESP_LOGW(TAG, "Invalid mfg_data_len: %d (ad_len=%d, data_len=%d)",
                   mfg_data_len, ad_len, data_len);
          offset += ad_len; // Move to next AD structure (ad_len already
                            // includes length byte)
          continue;
        }

        ESP_LOGI(TAG,
                 "Found manufacturer data: ad_len=%d, mfg_data_offset=%d, "
                 "mfg_data_len=%d",
                 ad_len, mfg_data_offset, mfg_data_len);

        // Print raw bytes
        char hex_str[128] = {0};
        for (int i = 0; i < mfg_data_len && i < 40; i++) {
          sprintf(hex_str + i * 3, "%02x ", data[mfg_data_offset + i]);
        }
        ESP_LOGI(TAG, "Mfg Data Hex: %s", hex_str);

        // Check if we have enough data for our packet (20 bytes)
        // Also verify we don't read past buffer
        ESP_LOGW(TAG,
                 "Mfg data check: mfg_data_len=%d, needed=%d, offset=%d, "
                 "total_len=%d",
                 mfg_data_len, sizeof(ble_score_packet_t), mfg_data_offset,
                 data_len);
        if (mfg_data_len >= sizeof(ble_score_packet_t) &&
            mfg_data_offset + sizeof(ble_score_packet_t) <= data_len) {
          ble_score_packet_t *pkt =
              (ble_score_packet_t *)&data[mfg_data_offset];

          // Debug: Print raw bytes of received packet (last byte should be
          // HMAC)
          if (mfg_data_len >= sizeof(ble_score_packet_t)) {
            ESP_LOGD(TAG,
                     "Packet last 2 bytes (last should be HMAC): %02x %02x",
                     data[mfg_data_offset + sizeof(ble_score_packet_t) - 2],
                     data[mfg_data_offset + sizeof(ble_score_packet_t) - 1]);
          } else {
            ESP_LOGW(TAG, "Packet truncated! Have %d bytes, need %d",
                     mfg_data_len, sizeof(ble_score_packet_t));
          }

          // Skip our own packets (Double check)
          if (pkt->node_id == g_node_id || pkt->node_id == 0) {
            // ESP_LOGD(TAG, "Skipping own/invalid packet from node %lu",
            // pkt->node_id);
            offset += ad_len; // Move to next AD structure
            continue;
          }

          // Quick validation: check if node_id looks reasonable (not 0, not all
          // 0xFF)
          if (pkt->node_id == 0 || pkt->node_id == 0xFFFFFFFF) {
            ESP_LOGD(TAG, "Invalid node_id in packet, skipping");
            offset += ad_len; // Move to next AD structure
            continue;
          }

          ESP_LOGI(TAG,
                   "Packet found: node_id=%lu (0x%08lx), our_id=%lu (0x%08lx)",
                   pkt->node_id, pkt->node_id, g_node_id, g_node_id);
          ESP_LOGW(
              TAG,
              "Received packet HMAC: %02x, mfg_data_len=%d, packet_size=%d",
              pkt->hmac[0], mfg_data_len, sizeof(ble_score_packet_t));

          // Verify HMAC (using 1-byte truncated HMAC)
          uint8_t computed_hmac[32]; // Full SHA256
          // Verify HMAC
          // Message is all fields except HMAC (last 1 byte) and company_id
          // (first 2 bytes)
          size_t message_len = sizeof(ble_score_packet_t) - 1 - 2;
          uint8_t message[sizeof(ble_score_packet_t) - 1 - 2];

          // Safe copy - copy all fields except HMAC
          if (message_len <= sizeof(message)) {
            // Copy from the start of node_id, skipping company_id
            memcpy(message, &pkt->node_id, message_len);

            if (auth_generate_hmac(message, message_len, g_cluster_key,
                                   computed_hmac)) {
              // Compare first byte of HMAC (constant-time comparison, reduced
              // to 1 byte)
              int hmac_diff = computed_hmac[0] ^ pkt->hmac[0];

              // Debug: Log HMAC values for troubleshooting
              ESP_LOGW(TAG,
                       "HMAC check: computed=%02x, received=%02x, diff=%d, "
                       "node_id=%lu, seq_num=%d",
                       computed_hmac[0], pkt->hmac[0], hmac_diff, pkt->node_id,
                       pkt->seq_num);

              if (hmac_diff == 0) {
                // Valid packet - update neighbor
                metrics_record_hmac_success(true);
                // Convert uint16_t back to float (0-10000 -> 0.0-1.0)
                float battery_f =
                    (float)pkt->battery / 10000.0f; // 0-10000 -> 0.00-1.00
                float trust_f =
                    (float)pkt->trust / 10000.0f; // 0-10000 -> 0.00-1.00
                float link_quality_f =
                    (float)pkt->link_quality / 10000.0f; // 0-10000 -> 0.00-1.00

                ESP_LOGI(TAG,
                         "Discovered neighbor: node_id=%lu, score=%.2f, "
                         "rssi=%d, seq=%d",
                         pkt->node_id, pkt->score, disc->rssi, pkt->seq_num);

                // Update our own RSSI metric (average of neighbors)
                metrics_update_rssi((float)disc->rssi);

                // Update our Trust Metric (using Neighbor's Trust as
                // Reputation)
                metrics_update_trust(trust_f);

                // Reconstruct full MAC address (ESP32 prefix + last 2 bytes we
                // received)
                uint8_t full_mac_recon[6];
                full_mac_recon[0] = 0x10; // ESP32 common prefix
                full_mac_recon[1] = 0x20;
                full_mac_recon[2] = 0xBA;
                full_mac_recon[3] = 0x00; // Unknown middle byte
                memcpy(&full_mac_recon[4], pkt->wifi_mac,
                       2); // Use the 2 bytes we have

                // MODIFIED: Pass seq_num to neighbor manager for PER
                // calculation
                neighbor_manager_update(pkt->node_id, full_mac_recon,
                                        disc->rssi, pkt->score, battery_f, 0,
                                        trust_f, link_quality_f, pkt->is_ch,
                                        pkt->seq_num);
                break; // Found and processed, no need to continue
              } else {
                ESP_LOGW(TAG, "HMAC verification failed for node %lu",
                         pkt->node_id);
                metrics_record_hmac_success(false);
              }
            } else {
              ESP_LOGW(TAG, "HMAC generation failed");
            }
          }
        } else {
          ESP_LOGD(TAG, "Insufficient data for packet: have=%d, need=%d",
                   mfg_data_len, sizeof(ble_score_packet_t));
        }
      }

      // Move to next AD structure (ad_len already includes the length byte
      // itself)
      offset += ad_len;
    }

    // Log if we received a discovery event but didn't find our manufacturer
    // data
    if (!found_mfg_data && data_len > 0) {
      ESP_LOGI(TAG, "Discovery event: data_len=%d, rssi=%d (no mfg data)",
               data_len, disc->rssi);
      // Log first few bytes for debugging
      if (data_len > 0) {
        char hex_str[64] = {0};
        int hex_len = data_len < 20 ? data_len : 20;
        for (int i = 0; i < hex_len; i++) {
          sprintf(hex_str + i * 3, "%02x ", data[i]);
        }
        ESP_LOGI(TAG, "First %d bytes: %s", hex_len, hex_str);
      }
    }
  }
    return 0;

  default:
    return 0;
  }
}

void ble_manager_start_advertising(void) {
  if (!ble_ready || advertising) {
    return;
  }

  // Use regular BLE advertising (extended advertising API changed in v6.1)
  // Set advertising data first
  ble_manager_update_advertisement();

  // Configure advertising parameters
  struct ble_gap_adv_params adv_params;
  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  adv_params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN;
  adv_params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MAX;

  // Start advertising
  int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                             &adv_params, ble_gap_event, NULL);
  if (rc != 0) {
    ESP_LOGE(TAG, "Failed to start advertising: %d", rc);
    return;
  }

  advertising = true;
  ESP_LOGI(TAG, "BLE advertising started");
}

// Forward declaration
extern bool g_is_ch;

void ble_manager_stop_advertising(void) {
  if (!advertising) {
    return;
  }

  int rc = ble_gap_adv_stop();
  if (rc != 0) {
    ESP_LOGE(TAG, "Failed to stop advertising: %d", rc);
  }
  advertising = false;
  ESP_LOGI(TAG, "BLE advertising stopped");
}

void ble_manager_start_scanning(void) {
  if (!ble_ready || scanning) {
    return;
  }

  // Configure scan parameters
  struct ble_gap_disc_params disc_params;
  memset(&disc_params, 0, sizeof(disc_params));
  disc_params.itvl = BLE_SCAN_INTERVAL_MS * 1000 / 625; // Units of 0.625ms
  disc_params.window = BLE_SCAN_WINDOW_MS * 1000 / 625;
  disc_params.filter_policy = BLE_HCI_SCAN_FILT_NO_WL;
  disc_params.passive = 0; // Active scan
  disc_params.limited = 0;

  // Start discovery
  int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params,
                        ble_gap_event, NULL);
  if (rc != 0) {
    ESP_LOGE(TAG, "Failed to start scanning: %d", rc);
    return;
  }

  scanning = true;
  ESP_LOGI(TAG, "BLE scanning started");
}

void ble_manager_stop_scanning(void) {
  if (!scanning) {
    return;
  }

  int rc = ble_gap_disc_cancel();
  if (rc != 0) {
    ESP_LOGE(TAG, "Failed to stop scanning: %d", rc);
  }
  scanning = false;
  ESP_LOGI(TAG, "BLE scanning stopped");
}

// Static buffer for advertisement packet to ensure it persists
static ble_score_packet_t adv_packet;

void ble_manager_update_advertisement(void) {
  if (!ble_ready) {
    return;
  }

  // Get current metrics
  node_metrics_t metrics = metrics_get_current();

  // Build score packet with scaled values (use static buffer)
  ble_score_packet_t *pkt = &adv_packet;
  memset(pkt, 0, sizeof(ble_score_packet_t));
  pkt->node_id = g_node_id;
  pkt->score = metrics.composite_score;
  // Scale float values to uint16_t (0.0-100.0 -> 0-10000, 0.0-1.0 -> 0-10000)
  pkt->battery = (uint16_t)(metrics.battery * 10000.0f); // 0.00-1.00 -> 0-10000
  pkt->trust = (uint16_t)(metrics.trust * 10000.0f);     // 0.00-1.00 -> 0-10000
  pkt->link_quality =
      (uint16_t)(metrics.link_quality * 10000.0f); // 0.00-1.00 -> 0-10000
  pkt->is_ch = g_is_ch;

  // Increment sequence number
  pkt->seq_num = g_seq_num++;

  // Get Wi-Fi MAC address (read from EFUSE to avoid Wi-Fi driver dependency)
  // Store only last 2 bytes (first 4 bytes are typically ESP32 prefix:
  // 10:20:BA:XX)
  uint8_t wifi_mac[6];
  esp_read_mac(wifi_mac, ESP_MAC_WIFI_STA);
  memcpy(pkt->wifi_mac, &wifi_mac[4], 2); // Copy last 2 bytes

  // Generate HMAC (1 byte - reduced to fit BLE 20-byte limit)
  // Message is all fields except HMAC (last 1 byte) and company_id (first 2
  // bytes)
  size_t message_len = sizeof(ble_score_packet_t) - 1 -
                       2; // Exclude 1-byte HMAC and 2-byte company_id
  uint8_t message[sizeof(ble_score_packet_t) - 1 - 2];
  memcpy(message, &pkt->node_id, message_len); // Copy from node_id onwards

  uint8_t temp_hmac[32]; // SHA256 produces 32 bytes
  auth_generate_hmac(message, message_len, g_cluster_key, temp_hmac);
  memcpy(pkt->hmac, temp_hmac,
         1); // Copy only 1 byte for truncated HMAC (reduced to fit 20 bytes)

  // Debug: Verify HMAC was set correctly
  ESP_LOGI(
      TAG, "Advertising packet: node_id=%lu, seq=%d, HMAC=%02x, packet_size=%d",
      pkt->node_id, pkt->seq_num, pkt->hmac[0], sizeof(ble_score_packet_t));

  // Set advertising data using fields
  struct ble_hs_adv_fields fields;
  memset(&fields, 0, sizeof(fields));

  // Explicitly disable all optional fields to maximize space for manufacturer
  // data
  fields.name = NULL;
  fields.name_len = 0;
  fields.name_is_complete = 0;
  fields.flags = 0;
  fields.tx_pwr_lvl_is_present = 0; // Don't include TX power

  // Use manufacturer data to carry our packet
  // NimBLE does NOT automatically add a Company ID, so we must add it manually
  // But now our struct INCLUDES the Company ID at the beginning.
  // So we just set it in the struct and pass the struct directly.

  pkt->company_id = 0x02E5; // Espressif Company ID

  fields.mfg_data = (uint8_t *)pkt;
  fields.mfg_data_len = sizeof(*pkt);

  // Debug: Print packet bytes before transmission (last byte should be HMAC)
  ESP_LOGI(TAG, "Transmitting packet bytes (last 2): %02x %02x",
           ((uint8_t *)pkt)[sizeof(ble_score_packet_t) - 2],
           ((uint8_t *)pkt)[sizeof(ble_score_packet_t) - 1]);

  int rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0) {
    ESP_LOGE(TAG, "Failed to set advertising data: %d", rc);
    return;
  }

  ESP_LOGD(TAG, "Advertisement updated successfully");
}

bool ble_manager_is_ready(void) { return ble_ready; }
