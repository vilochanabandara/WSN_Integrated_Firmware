#include "neighbor_manager.h"
#include "config.h"
#include "esp_log.h"
#include "esp_now_manager.h"
#include "esp_timer.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static const char *TAG = "NEIGHBOR";

static neighbor_entry_t neighbor_table[MAX_NEIGHBORS];
static size_t neighbor_count = 0;

void neighbor_manager_init(void) {
  memset(neighbor_table, 0, sizeof(neighbor_table));
  neighbor_count = 0;
  ESP_LOGI(TAG, "Neighbor manager initialized");
}

void neighbor_manager_update(uint32_t node_id, const uint8_t *mac_addr,
                             int8_t rssi, float score, float battery,
                             uint64_t uptime, float trust, float link_quality,
                             bool is_ch, uint8_t seq_num) {
  uint64_t now_ms = esp_timer_get_time() / 1000;

  // Find existing entry
  for (size_t i = 0; i < neighbor_count; i++) {
    if (neighbor_table[i].node_id == node_id) {
      // Update existing
      neighbor_entry_t *entry = &neighbor_table[i];

      // Calculate missed packets
      // Example: last=5, curr=7 -> missed=1 (packet 6)
      // Example: last=255, curr=1 -> missed=1 (packet 0)
      int diff = (int)seq_num - (int)entry->last_seq_num;
      if (diff < 0)
        diff += 256; // Handle wrap-around

      int missed = 0;
      if (diff > 1) {
        missed = diff - 1;
      }

      // Sanity check: if diff is huge (e.g. node rebooted), ignore
      if (missed > 20)
        missed = 0;

      // Update PER metrics (1 received, N missed)
      metrics_record_ble_reception(1, missed);

      entry->last_seq_num = seq_num;

      // Update MAC if changed (unlikely but possible if device replaced)
      if (mac_addr) {
        memcpy(entry->mac_addr, mac_addr, 6);
        // Register as ESP-NOW peer
        esp_now_manager_register_peer(mac_addr, false);
      }

      // Update RSSI EWMA
      if (entry->rssi_ewma == 0.0f) {
        entry->rssi_ewma = (float)rssi;
      } else {
        entry->rssi_ewma = RSSI_EWMA_ALPHA * (float)rssi +
                           (1.0f - RSSI_EWMA_ALPHA) * entry->rssi_ewma;
      }

      entry->last_rssi = rssi;
      entry->score = score;
      entry->battery = battery;
      entry->uptime_seconds = uptime;
      entry->trust = trust;
      entry->link_quality = link_quality;
      entry->last_seen_ms = now_ms;
      entry->is_ch = is_ch;
      if (is_ch) {
        entry->ch_announce_timestamp = now_ms;
      }
      entry->verified = true;

      return;
    }
  }

  // Add new neighbor
  if (neighbor_count < MAX_NEIGHBORS) {
    neighbor_entry_t *entry = &neighbor_table[neighbor_count];
    entry->node_id = node_id;
    if (mac_addr) {
      memcpy(entry->mac_addr, mac_addr, 6);
      // Register as ESP-NOW peer
      esp_now_manager_register_peer(mac_addr, false);
    }
    entry->rssi_ewma = (float)rssi;
    entry->last_rssi = rssi;
    entry->score = score;
    entry->battery = battery;
    entry->uptime_seconds = uptime;
    entry->trust = trust;
    entry->link_quality = link_quality;
    entry->last_seen_ms = now_ms;
    entry->is_ch = is_ch;
    entry->ch_announce_timestamp = is_ch ? now_ms : 0;
    entry->verified = true;
    entry->last_seq_num = seq_num; // Initialize sequence number
    neighbor_count++;

    ESP_LOGI(TAG, "Added neighbor: node_id=%lu, RSSI=%d, Seq=%d", node_id, rssi,
             seq_num);
  } else {
    ESP_LOGW(TAG, "Neighbor table full, cannot add node %lu", node_id);
  }
}

neighbor_entry_t *neighbor_manager_get(uint32_t node_id) {
  for (size_t i = 0; i < neighbor_count; i++) {
    if (neighbor_table[i].node_id == node_id) {
      return &neighbor_table[i];
    }
  }
  return NULL;
}

neighbor_entry_t *neighbor_manager_get_by_mac(const uint8_t *mac_addr) {
  for (size_t i = 0; i < neighbor_count; i++) {
    if (memcmp(neighbor_table[i].mac_addr, mac_addr, 6) == 0) {
      return &neighbor_table[i];
    }
  }
  return NULL;
}

size_t neighbor_manager_get_all(neighbor_entry_t *neighbors, size_t max_count) {
  if (neighbors == NULL || max_count == 0) {
    return 0;
  }

  size_t count = (neighbor_count < max_count) ? neighbor_count : max_count;
  if (count > 0) {
    memcpy(neighbors, neighbor_table, count * sizeof(neighbor_entry_t));
  }
  return count;
}

void neighbor_manager_cleanup_stale(void) {
  uint64_t now_ms = esp_timer_get_time() / 1000;
  size_t write_idx = 0;

  for (size_t i = 0; i < neighbor_count; i++) {
    if (now_ms - neighbor_table[i].last_seen_ms < NEIGHBOR_TIMEOUT_MS) {
      // Keep this neighbor
      if (write_idx != i) {
        neighbor_table[write_idx] = neighbor_table[i];
      }
      write_idx++;
    } else {
      ESP_LOGI(TAG, "Removed stale neighbor: node_id=%lu",
               neighbor_table[i].node_id);
    }
  }

  neighbor_count = write_idx;
}

bool neighbor_manager_is_in_cluster(const neighbor_entry_t *neighbor) {
  return neighbor->rssi_ewma >= CLUSTER_RADIUS_RSSI_THRESHOLD;
}

uint32_t neighbor_manager_get_current_ch(void) {
  uint64_t now_ms = esp_timer_get_time() / 1000;

  for (size_t i = 0; i < neighbor_count; i++) {
    if (neighbor_table[i].is_ch && neighbor_table[i].verified) {
      // Check if CH announcement is recent
      if (now_ms - neighbor_table[i].ch_announce_timestamp <
          CH_BEACON_TIMEOUT_MS) {
        return neighbor_table[i].node_id;
      }
    }
  }

  return 0; // No valid CH
}

bool neighbor_manager_get_ch_mac(uint8_t *mac_out) {
  uint64_t now_ms = esp_timer_get_time() / 1000;

  for (size_t i = 0; i < neighbor_count; i++) {
    if (neighbor_table[i].is_ch && neighbor_table[i].verified) {
      if (now_ms - neighbor_table[i].ch_announce_timestamp <
          CH_BEACON_TIMEOUT_MS) {
        if (mac_out) {
          memcpy(mac_out, neighbor_table[i].mac_addr, 6);
        }
        return true;
      }
    }
  }
  return false;
}

void neighbor_manager_update_trust(uint32_t node_id, bool success) {
  neighbor_entry_t *entry = neighbor_manager_get(node_id);
  if (entry) {
    // Update trust metrics using EWMA
    // Alpha = 0.1 (slowly adapt)
    float target = success ? 1.0f : 0.0f;
    entry->trust = 0.9f * entry->trust + 0.1f * target;

    // Mark as verified if trust is high enough
    if (entry->trust > 0.3f) {
      entry->verified = true;
    }

    ESP_LOGI(TAG,
             "[TEST] Trust Updated for Node %lu: New Score = %.2f (Success=%d)",
             node_id, entry->trust, success);
  }
}

size_t neighbor_manager_get_count(void) { return neighbor_count; }
