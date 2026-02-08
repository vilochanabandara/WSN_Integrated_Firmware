#include "esp_now_manager.h"
#include "config.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "metrics.h"
#include "neighbor_manager.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

static const char *TAG = "ESP_NOW";

static void esp_now_send_cb(const void *arg, esp_now_send_status_t status) {
  // Update Self Link Quality (PER)
  // 1.0 for success, 0.0 for failure
  // metrics_update_per((status == ESP_NOW_SEND_SUCCESS) ? 1.0f : 0.0f); //
  // REMOVED: Using BLE PER

  // Handle opaque argument (points to struct with MAC as first member)
  const uint8_t **ptr_to_mac = (const uint8_t **)arg;
  const uint8_t *mac_addr = *ptr_to_mac;

  if (mac_addr) {
    neighbor_entry_t *n = neighbor_manager_get_by_mac(mac_addr);
    if (n) {
      neighbor_manager_update_trust(n->node_id, status == ESP_NOW_SEND_SUCCESS);
    }
  }

  if (status != ESP_NOW_SEND_SUCCESS) {
    ESP_LOGW(TAG, "ESP-NOW send failed, status: %d", status);
  }
}

static void esp_now_recv_cb(const esp_now_recv_info_t *info,
                            const uint8_t *data, int len) {
  if (len == sizeof(sensor_payload_t)) {
    // It's a sensor packet!
    const sensor_payload_t *payload = (const sensor_payload_t *)data;
    ESP_LOGI(TAG,
             "RX Sensor Data from node_%lu: Temp=%.1fC, Hum=%.1f%%, Gas=%d, "
             "Audio=%.3f",
             payload->node_id, payload->temp_c, payload->hum_pct, payload->aqi,
             payload->audio_rms);

    // Update neighbor trust
    neighbor_entry_t *n = neighbor_manager_get_by_mac(info->src_addr);
    if (n) {
      neighbor_manager_update_trust(n->node_id, true);
    }
    return;
  }

  if (len < sizeof(uint32_t)) {
    ESP_LOGW(TAG, "Received invalid data length: %d", len);
    return;
  }

  // Update Self Trust (Reputation/HSR)
  // Receiving data is good!
  // metrics_update_trust(1.0f, 1.0f, 0.5f); // REMOVED: Using BLE Trust

  // Update Neighbor Trust
  neighbor_entry_t *n = neighbor_manager_get_by_mac(info->src_addr);
  if (n) {
    neighbor_manager_update_trust(n->node_id, true);
  }

  // In a real application, we would parse the message here
  // For now, just log it
  ESP_LOGI(TAG, "Received %d bytes from " MACSTR, len, MAC2STR(info->src_addr));
}

esp_err_t esp_now_manager_init(void) {
  ESP_LOGI(TAG, "Initializing ESP-NOW...");

  // Initialize Wi-Fi
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());

  // Set channel (must match other nodes)
  ESP_ERROR_CHECK(esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

  // Enable power save for coexistence
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));

  // Initialize ESP-NOW
  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_send_cb((esp_now_send_cb_t)esp_now_send_cb));
  ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));

  // Set PMK (Primary Master Key)
  ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)ESP_NOW_PMK));

  ESP_LOGI(TAG, "ESP-NOW initialized on channel %d", ESP_NOW_CHANNEL);
  return ESP_OK;
}

esp_err_t esp_now_manager_register_peer(const uint8_t *peer_addr,
                                        bool encrypt) {
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  memcpy(peer.peer_addr, peer_addr, ESP_NOW_ETH_ALEN);
  peer.channel = ESP_NOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = encrypt;

  if (encrypt) {
    memcpy(peer.lmk, ESP_NOW_LMK, ESP_NOW_KEY_LEN);
  }

  if (esp_now_is_peer_exist(peer_addr)) {
    return ESP_OK;
  }

  return esp_now_add_peer(&peer);
}

esp_err_t esp_now_manager_send_data(const uint8_t *peer_addr,
                                    const uint8_t *data, size_t len) {
  return esp_now_send(peer_addr, data, len);
}
