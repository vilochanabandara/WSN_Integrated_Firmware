#include "auth.h"
#include "ble_manager.h"
#include "config.h"
#include "election.h"
#include "esp_bt.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now_manager.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_manager.h" // Added this include
#include "metrics.h"
#include "neighbor_manager.h"
#include "nvs_flash.h"
#include "persistence.h"
#include "state_machine.h"
#include <stdio.h>

static const char *TAG = "MAIN";

// Global cluster key (MUST be changed in production!)
// In production, derive from network key or use secure key exchange
uint8_t g_cluster_key[CLUSTER_KEY_SIZE] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
    0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
    0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20};

// State machine task
void state_machine_task(void *pvParameters) {
  ESP_LOGI(TAG, "State machine task started");

  while (1) {
    state_machine_run();
    vTaskDelay(pdMS_TO_TICKS(100)); // Run every 100ms
  }
}

// Metrics update task
void metrics_task(void *pvParameters) {
  ESP_LOGI(TAG, "Metrics task started");

  while (1) {
    metrics_update();
    uint32_t ch_id = neighbor_manager_get_current_ch();
    size_t cluster_size = neighbor_manager_get_count();
    ESP_LOGI(TAG, "STATUS: State=%s, Role=%s, CH=%lu, Size=%zu",
             state_machine_get_state_name(), g_is_ch ? "CH" : "NODE", ch_id,
             cluster_size);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
  }
}

// Console task to read commands from stdin
void console_task(void *pvParameters) {
  char line[128];
  int pos = 0;

  ESP_LOGI(
      TAG,
      "Console task started. Type 'SET_WEIGHTS b u t l' to change weights.");

  while (1) {
    int c = fgetc(stdin);
    if (c == EOF) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    if (c == '\n' || c == '\r') {
      line[pos] = 0; // Null terminate
      if (pos > 0) {
        // Parse command
        if (strncmp(line, "SET_WEIGHTS", 11) == 0) {
          float b, u, t, l;
          if (sscanf(line + 11, "%f %f %f %f", &b, &u, &t, &l) == 4) {
            metrics_set_weights(b, u, t, l);
            printf("OK\n");
          } else {
            printf("ERROR: Invalid format\n");
          }
        } else if (strcmp(line, "GET_WEIGHTS") == 0) {
          // We don't have getters exposed but we can verify via logs
          printf("OK\n");
        }
      }
      pos = 0;
    } else {
      if (pos < sizeof(line) - 1) {
        line[pos++] = (char)c;
      }
    }
  }
}

void init_task(void *pvParameters) {
  ESP_LOGI(TAG, "Init task started");

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Initialize subsystems
  auth_init();
  metrics_init();
  neighbor_manager_init();
  election_init();
  persistence_init(); // Initialize persistence before other systems
  ble_manager_init();
  esp_now_manager_init(); // Initialize ESP-NOW for member-to-CH communication

  // Initialize state machine (this will set initial state)
  state_machine_init();

  ESP_LOGI(TAG, "Creating tasks...");
  xTaskCreate(state_machine_task, "state_machine",
              STATE_MACHINE_TASK_STACK_SIZE, NULL, STATE_MACHINE_TASK_PRIORITY,
              NULL);

  xTaskCreate(metrics_task, "metrics", METRICS_TASK_STACK_SIZE, NULL,
              METRICS_TASK_PRIORITY, NULL);

  // High priority for console to ensure responsiveness
  xTaskCreate(console_task, "console", 4096, NULL, 5, NULL);

  ESP_LOGI(TAG, "System initialized. Node ID: %lu", g_node_id);
  ESP_LOGI(TAG, "Current state: %s", state_machine_get_state_name());

  // Delete this task
  vTaskDelete(NULL);
}

void app_main(void) {
  // Very early log to verify device is running
  printf("=== ESP32 Starting ===\n");
  ESP_LOGI(TAG, "Cluster Creation System Starting...");
  printf("=== After ESP_LOGI ===\n");

  // Initialize LED Manager (Moved to earlier)
  led_manager_init();

  // Create init task with larger stack
  xTaskCreate(init_task, "init_task", 8192, NULL, 5, NULL);
  ESP_LOGI(TAG, "Init task created");
}
