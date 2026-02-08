#include "cJSON.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_sleep.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <esp_rom_sys.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TAG "WSN_NODE"

// ------------ CONFIGURATION -----------------
// MAC Addresses
static const uint8_t MAC_CH[6] = {0x10, 0x20, 0xBA, 0x4D, 0xEB, 0x1C};

// Dummy Battery Levels & Link Quality
#define BAT_MS1 45 // MS1 has 45%
#define LQ_MS1 90  // MS1 Link Quality 90%
#define BAT_MS2 80 // MS2 has 80%
#define LQ_MS2 60  // MS2 Link Quality 60%

typedef enum { ROLE_UNKNOWN, ROLE_CH, ROLE_MS } node_role_t;

typedef struct {
  uint8_t mac[6];
  int fixed_battery_level;
  int fixed_link_quality;
  char name[10];
} node_config_t;

// Known Nodes Table
static const node_config_t NODES[] = {
    {{0x10, 0x20, 0xBA, 0x4D, 0xEB, 0x1C}, 100, 100, "CH"}, // CH
    {{0x10, 0x20, 0xBA, 0x4C, 0x59, 0x8C}, BAT_MS1, LQ_MS1, "MS1"},
    {{0x30, 0xED, 0xA0, 0xBB, 0x4C, 0x58}, BAT_MS2, LQ_MS2, "MS2"}};
#define NUM_NODES (sizeof(NODES) / sizeof(node_config_t))

#define WIFI_CHANNEL 1
#define JSON_PATH "/spiffs/data.json"
#define SLOT_US 10000000       // 10 seconds per slot
#define START_DELAY_US 5000000 // 5 seconds initial delay
#define DATA_SIZE_BYTES 4096   // 4KB dummy data

// ------------ DATA STRUCTURES -----------------
typedef struct __attribute__((packed)) {
  int64_t epoch_us;
  uint8_t slot_index;
  int assigned_duration_sec;
} schedule_msg_t;

typedef struct {
  uint8_t mac[6];
  int battery_level;
  int link_quality;
  int priority_score;
} runtime_node_info_t;

// ------------ GLOBALS -----------------
static uint8_t my_mac[6];
static node_role_t my_role = ROLE_UNKNOWN;
static int my_dummy_battery = 0;
static const char *my_name = "UNK";

static volatile bool schedule_received = false;
static schedule_msg_t current_schedule;

// ------------ UTILS -----------------
int get_battery_level() { return my_dummy_battery; }

char *generate_compressed_data(int *len) {
  *len = 100 + (esp_random() % 100); // Random size between 100 and 199 bytes
  uint8_t *data = malloc(*len);
  if (data) {
    esp_fill_random(data, *len); // Generate random "compressed" data
  }
  return (char *)data;
}

void init_spiffs() {
  if (esp_spiffs_mounted(NULL))
    return;

  ESP_LOGI(TAG, "Initializing SPIFFS...");
  esp_vfs_spiffs_conf_t conf = {.base_path = "/spiffs",
                                .partition_label = NULL,
                                .max_files = 5,
                                .format_if_mount_failed = true};
  esp_err_t ret = esp_vfs_spiffs_register(&conf);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "SPIFFS mounted.");
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
  } else {
    ESP_LOGE(TAG, "SPIFFS mount failed (%s)", esp_err_to_name(ret));
  }
}

// Convert hex string to byte array
uint8_t *hex_str_to_bytes(const char *hex, int *out_len) {
  size_t len = strlen(hex);
  if (len % 2 != 0)
    return NULL;
  *out_len = len / 2;
  uint8_t *bytes = malloc(*out_len);
  if (!bytes)
    return NULL;

  for (size_t i = 0; i < *out_len; i++) {
    sscanf(hex + 2 * i, "%2hhx", &bytes[i]);
  }
  return bytes;
}

void save_json_to_spiffs(const char *ms_mac, const uint8_t *data, int len) {
  init_spiffs();

  // Check available space logic
  size_t total = 0, used = 0;
  if (esp_spiffs_info(NULL, &total, &used) == ESP_OK) {
    if (total - used < 5000) {
      ESP_LOGE(TAG, "Error: Storage Full (Free: %d bytes). Cannot save data.",
               total - used);
      return;
    }
  }

  cJSON *entry = cJSON_CreateObject();
  cJSON_AddStringToObject(entry, "ms_mac", ms_mac);

  char *hex_str = malloc(len * 2 + 1);
  if (hex_str) {
    for (int i = 0; i < len; i++) {
      sprintf(hex_str + (i * 2), "%02X", data[i]);
    }
    hex_str[len * 2] = '\0';
    cJSON_AddStringToObject(entry, "compressed_data", hex_str);
    free(hex_str);
  }

  // Read existing file
  FILE *f = fopen(JSON_PATH, "r");
  cJSON *root = NULL;

  if (f) {
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size > (50 * 1024)) {
      ESP_LOGW(TAG, "JSON file too large! clearing.");
      fclose(f);
      remove(JSON_PATH);
      root = cJSON_CreateArray();
    } else {
      char *buffer = malloc(size + 1);
      if (buffer) {
        fread(buffer, 1, size, f);
        buffer[size] = 0;
        root = cJSON_Parse(buffer);
        free(buffer);
      }
      fclose(f);
    }
  }

  if (!root)
    root = cJSON_CreateArray();
  cJSON_AddItemToArray(root, entry);

  char *str = cJSON_PrintUnformatted(root);
  if (str) {
    f = fopen(JSON_PATH, "w");
    if (f) {
      fwrite(str, 1, strlen(str), f);
      fclose(f);
      ESP_LOGI(TAG, "Data saved to JSON.");
    }
    free(str);
  }
  cJSON_Delete(root);
}

// Function to send stored data to new CH
void send_stored_data() {
  init_spiffs();
  FILE *f = fopen(JSON_PATH, "r");
  if (!f) {
    ESP_LOGI(TAG, "No stored data to send.");
    return;
  }

  fseek(f, 0, SEEK_END);
  long size = ftell(f);
  fseek(f, 0, SEEK_SET);

  char *buffer = malloc(size + 1);
  if (!buffer) {
    fclose(f);
    return;
  }

  fread(buffer, 1, size, f);
  buffer[size] = 0;
  fclose(f);

  cJSON *root = cJSON_Parse(buffer);
  free(buffer);

  if (root && cJSON_IsArray(root)) {
    ESP_LOGI(TAG, "Found %d stored records. Sending to CH...",
             cJSON_GetArraySize(root));
    cJSON *entry = NULL;
    cJSON_ArrayForEach(entry, root) {
      cJSON *data_item = cJSON_GetObjectItem(entry, "compressed_data");
      if (cJSON_IsString(data_item)) {
        int len = 0;
        uint8_t *data = hex_str_to_bytes(data_item->valuestring, &len);
        if (data) {
          esp_now_send(MAC_CH, data, len);
          ESP_LOGI(TAG, "Forwarded stored packet (%d bytes)", len);
          free(data);
          vTaskDelay(pdMS_TO_TICKS(100)); // Small delay between packets
        }
      }
    }
    // Clear file after successful send (Simulated success)
    remove(JSON_PATH);
    ESP_LOGI(TAG, "Stored data forwarded and cleared.");
  }
  cJSON_Delete(root);
}

// ------------ ESP-NOW CALLBACKS -----------------
void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (my_role == ROLE_MS) {
    if (len == sizeof(schedule_msg_t)) {
      memcpy(&current_schedule, data, sizeof(schedule_msg_t));
      schedule_received = true;
      ESP_LOGI(TAG, "Received Schedule: Epoch %lld, Slot %d",
               current_schedule.epoch_us, current_schedule.slot_index);
    }
  } else if (my_role == ROLE_CH) {
    char mac_str[18];
    sprintf(mac_str, "%02X:%02X:%02X:%02X:%02X:%02X", info->src_addr[0],
            info->src_addr[1], info->src_addr[2], info->src_addr[3],
            info->src_addr[4], info->src_addr[5]);

    ESP_LOGI(TAG, "Received %d bytes from %s", len, mac_str);
    save_json_to_spiffs(mac_str, data, len);
  }
}

void send_cb(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  // Callback logic if needed
}

// ------------ SETUP -----------------
void setup_wifi_espnow() {
  nvs_flash_init();
  esp_netif_init();
  esp_event_loop_create_default();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_start();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_ps(WIFI_PS_NONE);

  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_cb));
  ESP_ERROR_CHECK(esp_now_register_send_cb(send_cb));
}

void add_peer(const uint8_t *mac) {
  if (mac == NULL)
    return;
  esp_now_peer_info_t peer = {0};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = WIFI_CHANNEL;
  peer.ifidx = ESP_IF_WIFI_STA;
  if (!esp_now_is_peer_exist(mac)) {
    esp_now_add_peer(&peer);
  }
}

// ------------ ROLES -----------------
void identify_role() {
  esp_read_mac(my_mac, ESP_MAC_WIFI_STA);
  ESP_LOGI(TAG, "My MAC: %02X:%02X:%02X:%02X:%02X:%02X", my_mac[0], my_mac[1],
           my_mac[2], my_mac[3], my_mac[4], my_mac[5]);

  for (int i = 0; i < NUM_NODES; i++) {
    if (memcmp(my_mac, NODES[i].mac, 6) == 0) {
      my_dummy_battery = NODES[i].fixed_battery_level;
      my_name = NODES[i].name;
      if (strcmp(my_name, "CH") == 0) {
        my_role = ROLE_CH;
      } else {
        my_role = ROLE_MS;
      }
      ESP_LOGI(TAG, "Identified as %s (Role: %s, Bat: %d%%)", my_name,
               my_role == ROLE_CH ? "CH" : "MS", my_dummy_battery);
      return;
    }
  }
  ESP_LOGW(TAG, "Unknown MAC! Defaulting to MS role.");
  my_role = ROLE_MS;
  my_dummy_battery = 0;
}

int compare_priority(const void *a, const void *b) {
  const runtime_node_info_t *nA = (const runtime_node_info_t *)a;
  const runtime_node_info_t *nB = (const runtime_node_info_t *)b;
  // Descending order: Higher priority score goes first
  return nB->priority_score - nA->priority_score;
}

void run_ch() {
  init_spiffs();

  // Add all known MS nodes as peers
  for (int i = 0; i < NUM_NODES; i++) {
    if (strcmp(NODES[i].name, "CH") != 0) {
      add_peer(NODES[i].mac);
    }
  }

  // Active Runtime Node List
  runtime_node_info_t active_nodes[NUM_NODES - 1];
  int ms_count = 0;

  for (int i = 0; i < NUM_NODES; i++) {
    if (strcmp(NODES[i].name, "CH") != 0) {
      memcpy(active_nodes[ms_count].mac, NODES[i].mac, 6);
      active_nodes[ms_count].battery_level = NODES[i].fixed_battery_level;
      active_nodes[ms_count].link_quality = NODES[i].fixed_link_quality;

      // Calculate Priority Score
      // Formula: P = LinkQuality + (100 - Battery)
      // Higher Link Quality -> Higher Priority
      // Lower Battery -> Higher Priority (100 - Bat)
      active_nodes[ms_count].priority_score =
          active_nodes[ms_count].link_quality +
          (100 - active_nodes[ms_count].battery_level);

      ms_count++;
    }
  }

  while (1) {
    // 1. Sort by Priority (Descending)
    qsort(active_nodes, ms_count, sizeof(runtime_node_info_t),
          compare_priority);

    // 2. Define Epoch Start
    int64_t now = esp_timer_get_time();
    int64_t epoch_start = now + START_DELAY_US;

    // 3. Send Schedule to each MS
    for (int i = 0; i < ms_count; i++) {
      schedule_msg_t msg;
      msg.epoch_us = epoch_start;
      msg.slot_index = i;
      msg.assigned_duration_sec = SLOT_US / 1000000;

      esp_now_send(active_nodes[i].mac, (uint8_t *)&msg, sizeof(msg));
      ESP_LOGI(TAG, "Scheduled MS (Battery %d%%, LinkQuality %d%%, Score %d) for Slot %d",
               active_nodes[i].battery_level, active_nodes[i].link_quality,
               active_nodes[i].priority_score, i);
    }

    // 4. CH Waits for the entire cycle
    int cycle_duration_ms =
        (START_DELAY_US / 1000) + (ms_count * (SLOT_US / 1000)) + 2000;
    ESP_LOGI(TAG, "Cycle started. Waiting %d ms...", cycle_duration_ms);
    vTaskDelay(cycle_duration_ms / portTICK_PERIOD_MS);
  }
}

void run_ms() {
  add_peer(MAC_CH);

  while (1) {
    // 0. Reset schedule flag
    schedule_received = false;

    // 1. Wait for Schedule
    ESP_LOGI(TAG, "Waiting for schedule from CH...");
    while (!schedule_received) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    // 2. Calculate Wake/Tx Time
    int64_t now = esp_timer_get_time();
    int64_t my_slot_start =
        current_schedule.epoch_us + (current_schedule.slot_index * SLOT_US);
    int64_t wait_time_us = my_slot_start - now;

    if (wait_time_us > 2000000) {
      ESP_LOGI(TAG, "Wait %lld ms. Entering Light Sleep...",
               wait_time_us / 1000);
      vTaskDelay(wait_time_us / 1000 / portTICK_PERIOD_MS);
    } else if (wait_time_us > 0) {
      vTaskDelay(wait_time_us / 1000 / portTICK_PERIOD_MS);
    }

    // 3. Send Data
    ESP_LOGI(TAG, "Slot %d Started!", current_schedule.slot_index);

    // 3a. First, send any stored data (Handover logic)
    send_stored_data();

    // 3b. Send current compressed data
    int data_len = 0;
    char *compressed_data = generate_compressed_data(&data_len);

    if (compressed_data) {
      if (data_len < 250) {
        esp_now_send(MAC_CH, (uint8_t *)compressed_data, data_len);
        ESP_LOGI(TAG, "Sent %d bytes of compressed data", data_len);
      } else {
        esp_now_send(MAC_CH, (uint8_t *)compressed_data, 249);
      }
      free(compressed_data);
    } else {
      ESP_LOGE(TAG, "Failed to generate compressed data!");
    }

    // 4. Sleep Logic
    int total_slots = 2; // Hardcoded known total
    int64_t cycle_end = current_schedule.epoch_us + (total_slots * SLOT_US);
    int64_t sleep_duration = (cycle_end - esp_timer_get_time()) + 5000000;

    if (sleep_duration > 0) {
      ESP_LOGI(TAG, "Data sent. Deep Sleeping for %lld ms...",
               sleep_duration / 1000);
      esp_deep_sleep(sleep_duration);
    } else {
      ESP_LOGI(TAG, "Cycle over? Restarting.");
      esp_restart();
    }
  }
}

void app_main(void) {
  setup_wifi_espnow();
  identify_role();

  if (my_role == ROLE_CH) {
    run_ch();
  } else {
    run_ms();
  }
}
