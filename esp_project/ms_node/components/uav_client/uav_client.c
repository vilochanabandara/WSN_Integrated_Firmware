#include "uav_client.h"
#include "cJSON.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "mbedtls/md.h"
#include <string.h>

static const char *TAG = "UAV_CLIENT";

// --- HMAC Helper ---
static void generate_token(const char *node_id, const char *metadata,
                           char *output_hex) {
  char payload[128];
  snprintf(payload, sizeof(payload), "%s|%s", node_id, metadata);

  const char *key = UAV_SECRET_KEY;
  unsigned char output[32];

  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, (const unsigned char *)key, strlen(key));
  mbedtls_md_hmac_update(&ctx, (const unsigned char *)payload, strlen(payload));
  mbedtls_md_hmac_finish(&ctx, output);
  mbedtls_md_free(&ctx);

  for (int i = 0; i < 32; i++) {
    sprintf(output_hex + (i * 2), "%02x", output[i]);
  }
  output_hex[64] = 0;
}

// --- Wi-Fi Helper ---
// Note: This assumes the main application manages the Wi-Fi stack init.
// We just use esp_wifi_connect().

static esp_err_t wifi_join(const char *ssid, const char *pass) {
  wifi_config_t wifi_config = {0};
  strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
  strncpy((char *)wifi_config.sta.password, pass,
          sizeof(wifi_config.sta.password));

  ESP_LOGI(TAG, "Connecting to %s...", ssid);
  ESP_ERROR_CHECK(esp_wifi_disconnect()); // Ensure cleanly disconnected first
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_connect());

  // Wait for connection (Simplified: assumes default event loop handles IP)
  // In a robust app, use event groups. For this port, we delay.
  int retries = 0;
  while (retries < 20) {
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
      ESP_LOGI(TAG, "Connected to AP");
      return ESP_OK;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    retries++;
  }
  ESP_LOGE(TAG, "Failed to connect to AP");
  return ESP_FAIL;
}

esp_err_t uav_client_run_onboarding(void) {
  ESP_LOGI(TAG, "Starting UAV Onboarding Sequence");

  // 1. Connect to Wi-Fi
  if (wifi_join(UAV_WIFI_SSID, UAV_WIFI_PASS) != ESP_OK) {
    return ESP_FAIL;
  }

  // 2. Prepare Data
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0],
           mac[1], mac[2], mac[3], mac[4], mac[5]);

  const char *node_id = "wsn_node";              // Dynamic?
  const char *metadata = "lat=7.123;lon=80.456"; // Placeholder

  char token[65];
  generate_token(node_id, metadata, token);

  // 3. POST /onboard
  esp_http_client_config_t config = {
      .url = UAV_SERVER_URL_ONBOARD,
      .method = HTTP_METHOD_POST,
      .timeout_ms = 5000,
  };
  esp_http_client_handle_t client = esp_http_client_init(&config);
  esp_http_client_set_header(client, "Content-Type", "application/json");

  char post_data[256];
  snprintf(post_data, sizeof(post_data),
           "{\"node_id\":\"%s\",\"mac\":\"%s\",\"token\":\"%s\",\"metadata\":"
           "\"%s\"}",
           node_id, mac_str, token, metadata);

  esp_http_client_set_post_field(client, post_data, strlen(post_data));

  esp_err_t err = esp_http_client_perform(client);
  char session_id[64] = {0};
  bool success = false;

  if (err == ESP_OK) {
    int status_code = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "Onboard Status: %d", status_code);
    if (status_code == 200) {
      int content_len = esp_http_client_get_content_length(client);
      if (content_len > 0) {
        char *buffer = malloc(content_len + 1);
        if (buffer) {
          esp_http_client_read_response(client, buffer, content_len);
          buffer[content_len] = 0;
          ESP_LOGI(TAG, "Response: %s", buffer);

          cJSON *json = cJSON_Parse(buffer);
          if (json) {
            cJSON *sid = cJSON_GetObjectItem(json, "session_id");
            if (cJSON_IsString(sid) && (sid->valuestring != NULL)) {
              strncpy(session_id, sid->valuestring, sizeof(session_id) - 1);
              ESP_LOGI(TAG, "Session ID: %s", session_id);
              success = true;
            }
            cJSON_Delete(json);
          }
          free(buffer);
        }
      }
    }
  } else {
    ESP_LOGE(TAG, "Onboard POST failed: %s", esp_err_to_name(err));
  }
  esp_http_client_cleanup(client);

  // 4. POST /ack
  if (success) {
    esp_http_client_config_t ack_config = {
        .url = UAV_SERVER_URL_ACK,
        .method = HTTP_METHOD_POST,
    };
    client = esp_http_client_init(&ack_config);
    esp_http_client_set_header(client, "Content-Type", "application/json");

    snprintf(post_data, sizeof(post_data), "{\"session_id\":\"%s\"}",
             session_id);
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    err = esp_http_client_perform(client);
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "ACK Sent. Status: %d",
               esp_http_client_get_status_code(client));
    }
    esp_http_client_cleanup(client);
  }

  return success ? ESP_OK : ESP_FAIL;
}
