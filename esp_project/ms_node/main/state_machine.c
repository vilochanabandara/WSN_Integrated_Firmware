#include "state_machine.h"
#include "ble_manager.h"
#include "config.h"
#include "election.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now_manager.h"
#include "esp_timer.h"
#include "led_manager.h"
#include "metrics.h"
#include "neighbor_manager.h"
#include "rf_receiver.h"
#include "storage_manager.h"
#include "uav_client.h"
#include <stdlib.h> // For qsort
#include <string.h>

static int compare_priority(const void *a, const void *b) {
  const neighbor_entry_t *na = (const neighbor_entry_t *)a;
  const neighbor_entry_t *nb = (const neighbor_entry_t *)b;

  // Githmi's Formula: P = LinkQuality + (100 - Battery)
  // Higher P goes first.
  // Note: metrics are 0.0-1.0, so we scale by 100.
  float score_a =
      (na->link_quality * 100.0f) + (100.0f - (na->battery * 100.0f));
  float score_b =
      (nb->link_quality * 100.0f) + (100.0f - (nb->battery * 100.0f));

  if (score_b > score_a)
    return 1;
  if (score_b < score_a)
    return -1;
  return 0;
}

uint32_t state_machine_get_sleep_time_ms(void) {
  // Only smart sleep in MEMBER state with a valid schedule
  if (g_current_state == STATE_MEMBER) {
    schedule_msg_t sched = esp_now_get_current_schedule();
    int64_t now_us = esp_timer_get_time();

    if (sched.magic == ESP_NOW_MAGIC_SCHEDULE &&
        sched.epoch_us > (now_us - (SLOT_DURATION_SEC *
                                    10000000LL))) { /* Valid recent schedule */

      int64_t my_slot_start =
          sched.epoch_us +
          (sched.slot_index * sched.slot_duration_sec * 1000000LL);
      int64_t time_to_slot = my_slot_start - now_us;

      if (time_to_slot > 0) {
        // We are early. Sleep until slot.
        return (uint32_t)(time_to_slot / 1000);
      } else {
        // We are IN the slot or LATE.
        // If we are strictly in the slot, we shouldn't sleep long.
        // If we missed it, sleep default.
        int64_t time_in_slot = -time_to_slot;
        int64_t slot_len_us = sched.slot_duration_sec * 1000000LL;

        if (time_in_slot < slot_len_us) {
          // Currently IN slot. Don't sleep! Run loop immediately.
          return 100;
        }
      }
    }
  }
  return 5000; // Default sleep
}

static const char *TAG = "STATE";

node_state_t g_current_state = STATE_INIT;
bool g_is_ch = false;
uint32_t g_node_id = 0;
uint64_t g_mac_addr = 0;

// Forward declarations
extern uint8_t g_cluster_key[CLUSTER_KEY_SIZE];

static uint64_t state_entry_time = 0;

const char *state_machine_get_state_name(void) {
  switch (g_current_state) {
  case STATE_INIT:
    return "INIT";
  case STATE_DISCOVER:
    return "DISCOVER";
  case STATE_CANDIDATE:
    return "CANDIDATE";
  case STATE_CH:
    return "CH";
  case STATE_MEMBER:
    return "MEMBER";
  case STATE_UAV_ONBOARDING:
    return "UAV_ONBOARDING";
  case STATE_SLEEP:
    return "SLEEP";
  default:
    return "UNKNOWN";
  }
}

static void transition_to_state(node_state_t new_state) {
  if (g_current_state == new_state) {
    return;
  }

  ESP_LOGI(TAG, "State transition: %s -> %s", state_machine_get_state_name(),
           (new_state == STATE_INIT             ? "INIT"
            : new_state == STATE_DISCOVER       ? "DISCOVER"
            : new_state == STATE_CANDIDATE      ? "CANDIDATE"
            : new_state == STATE_CH             ? "CH"
            : new_state == STATE_MEMBER         ? "MEMBER"
            : new_state == STATE_UAV_ONBOARDING ? "UAV_ONBOARDING"
                                                : "SLEEP"));

  // Ensure global flag matches state
  if (new_state == STATE_CH) {
    g_is_ch = true;
  } else {
    g_is_ch = false;
  }

  g_current_state = new_state;
  led_manager_set_state(new_state);
  state_entry_time = esp_timer_get_time() / 1000;
}

void state_machine_init(void) {
  // Get MAC address for node ID
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  g_mac_addr = ((uint64_t)mac[0] << 40) | ((uint64_t)mac[1] << 32) |
               ((uint64_t)mac[2] << 24) | ((uint64_t)mac[3] << 16) |
               ((uint64_t)mac[4] << 8) | mac[5];

  // Use lower 32 bits as node ID
  g_node_id = (uint32_t)(g_mac_addr & 0xFFFFFFFF);

  ESP_LOGI(TAG, "State machine initialized: node_id=%lu, MAC=%llx", g_node_id,
           g_mac_addr);

  transition_to_state(STATE_INIT);
}

void state_machine_force_uav_test(void) {
  ESP_LOGI(TAG, "Forcing UAV Test Mode (Manual Trigger)");
  transition_to_state(STATE_UAV_ONBOARDING);
}

void state_machine_run(void) {
  uint64_t now_ms = esp_timer_get_time() / 1000;

  switch (g_current_state) {
  case STATE_INIT:
    // Boot & self-init
    ESP_LOGI(TAG, "Boot & self-init");

    // Transition to DISCOVER after a short delay (bypass BLE ready check)
    if (now_ms - state_entry_time > 2000) { // Wait 2 seconds for initialization
      transition_to_state(STATE_DISCOVER);
    }
    break;

  case STATE_DISCOVER:
    // Neighbor discovery phase
    if (now_ms - state_entry_time < 5000) { // 5 second discovery
      if (!ble_manager_is_ready()) {
        break;
      }

      // Start both advertising and scanning so nodes can discover each other
      ble_manager_start_advertising();
      ble_manager_start_scanning();

      // Update metrics and advertisement periodically
      static uint64_t last_update = 0;
      if (now_ms - last_update >= 1000) { // Update every second
        // metrics_update(); // Handled by metrics_task
        ble_manager_update_advertisement();
        last_update = now_ms;

        // Check if there's already an existing CH (check continuously, not just
        // at end)
        uint32_t existing_ch = neighbor_manager_get_current_ch();
        if (existing_ch != 0 && (now_ms - state_entry_time) >= 2000) {
          // Found CH after at least 2 seconds of discovery (give time to find
          // neighbors)
          ESP_LOGI(TAG,
                   "DISCOVER: Found existing CH node_%lu after %llu ms, "
                   "joining as MEMBER",
                   existing_ch,
                   (unsigned long long)(now_ms - state_entry_time));
          // Keep BLE running for beaconing as MEMBER (don't stop)
          // Just stop scanning to save power
          ble_manager_stop_scanning();
          g_is_ch = false;
          transition_to_state(STATE_MEMBER);
          break; // Exit DISCOVER immediately
        }
      }
    } else {
      // Discovery complete (5 seconds elapsed) - check if there's already a CH
      uint32_t existing_ch = neighbor_manager_get_current_ch();
      if (existing_ch != 0) {
        ESP_LOGI(TAG,
                 "DISCOVER: Found existing CH node_%lu at end of window, "
                 "joining as MEMBER",
                 existing_ch);
        // Keep BLE running for beaconing as MEMBER (don't stop)
        // Just stop scanning to save power
        ble_manager_stop_scanning();
        g_is_ch = false;
        transition_to_state(STATE_MEMBER);
      } else {
        // No CH found, move to candidate for election
        // Keep both advertising and scanning for election
        transition_to_state(STATE_CANDIDATE);
        election_reset_window();
      }
    }
    break;

  case STATE_CANDIDATE:
    // Candidate phase - collect scores and run election
    {
      // Start advertising our score
      ble_manager_start_advertising();
      ble_manager_start_scanning();

      // Update metrics and advertisement periodically
      static uint64_t last_candidate_update = 0;
      if (now_ms - last_candidate_update >= 1000) {
        // metrics_update(); // Handled by metrics_task
        ble_manager_update_advertisement();
        last_candidate_update = now_ms;
      }

      // Cleanup stale neighbors
      neighbor_manager_cleanup_stale();

      // Check if election window has expired
      uint64_t window_start = election_get_window_start();
      if (window_start == 0) {
        election_reset_window();
        window_start = election_get_window_start();
      }

      if (now_ms - window_start >= ELECTION_WINDOW_MS) {
        // Run election
        uint32_t winner = election_run();

        if (winner == g_node_id) {
          // We won!
          g_is_ch = true;
          transition_to_state(STATE_CH);
        } else if (winner != 0) {
          // Someone else won
          g_is_ch = false;
          transition_to_state(STATE_MEMBER);
        } else {
          // No valid winner, restart discovery
          ESP_LOGW(TAG, "No valid election winner, restarting discovery");
          transition_to_state(STATE_DISCOVER);
        }
      }
    }
    break;

  case STATE_CH:
    // Cluster Head duties
    {
      // Update metrics
      // metrics_update(); // Handled by metrics_task

      // Update CH announcement
      ble_manager_update_advertisement();

      // Check if re-election is needed
      if (election_check_reelection_needed()) {
        // Check if there's already a valid CH we're yielding to
        uint32_t other_ch = neighbor_manager_get_current_ch();
        if (other_ch != 0) {
          ESP_LOGI(TAG, "Yielding to existing CH %lu, becoming MEMBER",
                   other_ch);
          g_is_ch = false;
          transition_to_state(STATE_MEMBER);
        } else {
          ESP_LOGI(TAG, "Re-election triggered, returning to candidate");
          g_is_ch = false;
          transition_to_state(STATE_CANDIDATE);
          election_reset_window();
        }
        break;
      }

      // CH duties: maintain member list, etc.
      neighbor_manager_cleanup_stale();

      // Check cluster size
      neighbor_entry_t neighbors[MAX_NEIGHBORS];
      size_t count = neighbor_manager_get_all(neighbors, MAX_NEIGHBORS);
      if (count > MAX_CLUSTER_SIZE) {
        ESP_LOGW(TAG, "Cluster size exceeded (%zu), triggering split", count);
        // In production, implement cluster split logic
      }

      // UAV Trigger Check
      if (rf_receiver_check_trigger()) {
        ESP_LOGI(TAG, "UAV Trigger detected! Transitioning to UAV ONBOARDING");
        transition_to_state(STATE_UAV_ONBOARDING);
      }

      // ---------------------------------------------------------
      // TIME SLICING SCHEDULER (Novelty)
      // ---------------------------------------------------------
      static uint64_t last_schedule_broadcast = 0;
      // Cycle: 5s buffer + (N * 1s slots). Min 10s.
      if (now_ms - last_schedule_broadcast >= 10000) {
        neighbor_entry_t neighbors[MAX_NEIGHBORS];
        size_t count = neighbor_manager_get_all(neighbors, MAX_NEIGHBORS);

        if (count > 0) {
          // Sort by Priority (Githmi-style: P = Link + (100-Bat))
          qsort(neighbors, count, sizeof(neighbor_entry_t), compare_priority);

          int64_t epoch_us = esp_timer_get_time() + 5000000; // Start in 5s

          for (size_t i = 0; i < count; i++) {
            schedule_msg_t sched;
            sched.epoch_us = epoch_us;
            sched.slot_index = i;
            sched.slot_duration_sec = 1; // 1 second per node
            sched.magic = ESP_NOW_MAGIC_SCHEDULE;

            // Broadcast to each (using Unicast for reliability)
            esp_now_manager_send_data(neighbors[i].mac_addr, (uint8_t *)&sched,
                                      sizeof(sched));
            ESP_LOGI(TAG, "SCHED: Assigned Slot %d to Node %lu (Score %.2f)",
                     (int)i, neighbors[i].node_id, neighbors[i].score);
          }
          last_schedule_broadcast = now_ms;
        }
      }
    }
    break;

  case STATE_MEMBER:
    // Member duties
    {
      // Update metrics
      // metrics_update(); // Handled by metrics_task

      // Ensure BLE advertising is active (for broadcasting our metrics)
      static bool member_ble_started = false;
      if (!member_ble_started) {
        if (ble_manager_is_ready()) {
          ble_manager_start_advertising();
          member_ble_started = true;
          ESP_LOGI(TAG, "MEMBER: BLE advertising started");
        } else {
          ESP_LOGW(TAG, "MEMBER: Waiting for BLE to be ready");
          break; // Wait for BLE to be ready
        }
      }

      // Check if CH is still valid
      uint32_t current_ch = neighbor_manager_get_current_ch();
      if (current_ch == 0) {
        ESP_LOGW(
            TAG,
            "CH lost (current_ch=0), returning to candidate to find new CH");
        member_ble_started = false; // Reset flag
        transition_to_state(STATE_CANDIDATE);
        election_reset_window();
        break;
      }

      // Update advertisement (as member) - throttled to avoid crashes
      static uint64_t last_adv_update = 0;
      uint64_t now_ms = esp_timer_get_time() / 1000;
      if (now_ms - last_adv_update >=
          1000) { // Update every 1 second, not every 100ms
        if (ble_manager_is_ready()) {
          ble_manager_update_advertisement();
          last_adv_update = now_ms;
        }
      }

      // Member duties: duty-cycle radio, send keep-alive, etc.
      neighbor_manager_cleanup_stale();

      // Check if re-election is needed
      if (election_check_reelection_needed()) {
        ESP_LOGI(TAG, "Re-election needed, returning to candidate");
        member_ble_started = false; // Reset flag
        transition_to_state(STATE_CANDIDATE);
        election_reset_window();
      }

      // Send data (Time Sliced or Fallback)
      static uint64_t last_data_send = 0;
      schedule_msg_t sched = esp_now_get_current_schedule();
      int64_t now_us = esp_timer_get_time();
      bool can_send = false;

      // Check Time Slicing
      if (sched.magic == ESP_NOW_MAGIC_SCHEDULE &&
          sched.epoch_us > (now_us - 60000000)) {
        // We have a recent schedule
        int64_t my_start =
            sched.epoch_us +
            (sched.slot_index * sched.slot_duration_sec * 1000000LL);
        int64_t my_end = my_start + (sched.slot_duration_sec * 1000000LL);

        if (now_us >= my_start && now_us < my_end) {
          // We are in our slot!
          // Only send once per slot (simple logic: check if we just sent)
          if ((now_us - (last_data_send * 1000)) >
              2000000) { // Enforce >2s gap (one per cycle concept)
            can_send = true;
            ESP_LOGI(TAG, "TIME SLICING: In Slot %d (window match), sending...",
                     sched.slot_index);
          }
        }
      } else {
        // Fallback: Default 1s interval (Legacy/No-Schedule mode)
        if ((now_ms - last_data_send) >= 1000) {
          can_send = true;
        }
      }

      if (current_ch != 0 && can_send) {
        uint8_t ch_mac[6];
        if (neighbor_manager_get_ch_mac(ch_mac)) {
          sensor_payload_t payload;
          metrics_get_sensor_data(&payload);

          // Only send if we have valid data (timestamp != 0)
          if (payload.timestamp_ms != 0) {
            esp_err_t ret = esp_now_manager_send_data(
                ch_mac, (uint8_t *)&payload, sizeof(payload));
            if (ret == ESP_OK) {
              last_data_send = now_ms;
              ESP_LOGI(TAG, "Sent sensor data to CH (Node %lu)",
                       payload.node_id);
            } else {
              ESP_LOGW(TAG, "Failed to send data to CH: %s",
                       esp_err_to_name(ret));
            }
          }
        } else {
          ESP_LOGW(TAG, "CH MAC not found, cannot send data");
        }
      }

      // -------------------------------------------------------------
      // Time-Bounded Burst (Novelty: "Sprint" during slot)
      // -------------------------------------------------------------
      int64_t now_us = esp_timer_get_time();
      schedule_msg_t sched = esp_now_get_current_schedule();
      bool in_slot = false;
      int64_t slot_end_us = 0;

      if (sched.magic == ESP_NOW_MAGIC_SCHEDULE &&
          sched.epoch_us > (now_us - (SLOT_DURATION_SEC * 10000000LL))) {
        int64_t my_start =
            sched.epoch_us +
            (sched.slot_index * sched.slot_duration_sec * 1000000LL);
        int64_t my_end = my_start + (sched.slot_duration_sec * 1000000LL);
        slot_end_us = my_end;

        if (now_us >= my_start && now_us < my_end) {
          in_slot = true;
        }
      }

      // If we are in our slot, burst send stored data!
      if (in_slot && current_ch != 0) {
        uint8_t ch_mac[6];
        if (neighbor_manager_get_ch_mac(ch_mac)) {
          char history_line[256];
          int packets_sent = 0;

          // Keep sending as long as we have >1s remaining in slot
          while ((esp_timer_get_time() < (slot_end_us - 1000000LL))) {
            if (storage_manager_pop_line(history_line, sizeof(history_line)) ==
                ESP_OK) {
              esp_err_t ret = esp_now_manager_send_data(
                  ch_mac, (uint8_t *)history_line, strlen(history_line));
              if (ret == ESP_OK) {
                packets_sent++;
                // Small delay to prevent radio buffer overflow
                // We use a busy-wait delay or very short vTaskDelay?
                // Since we are in task context, short vTaskDelay is safer.
                vTaskDelay(pdMS_TO_TICKS(50));
              } else {
                ESP_LOGW(TAG, "Burst send failed: %s", esp_err_to_name(ret));
                break; // Radio busy? Stop bursting.
              }
            } else {
              break; // No more data
            }
          }
          if (packets_sent > 0) {
            ESP_LOGI(TAG, "BURST: Sent %d stored packets during Slot %d",
                     packets_sent, sched.slot_index);
          }
        }
      }

      // Fallback: Lazy single line sync if no schedule
      static uint64_t last_history_sync = 0;
      if (!in_slot && current_ch != 0 &&
          (esp_timer_get_time() / 1000 - last_history_sync) >= 5000) {
        // ... existing fallback code ...
      }
      last_history_sync = esp_timer_get_time() / 1000;
    }
    break;

  case STATE_UAV_ONBOARDING:
    // UAV Interaction Phase
    {
      ESP_LOGI(TAG, "Starting UAV Onboarding Sequence...");

      // Stop BLE temporarily to avoid interference
      ble_manager_stop_scanning();

      // Execute onboarding (Blocking for now)
      esp_err_t ret = uav_client_run_onboarding();

      if (ret == ESP_OK) {
        ESP_LOGI(TAG, "UAV Onboarding SUCCESS");
      } else {
        ESP_LOGE(TAG, "UAV Onboarding FAILED: %s", esp_err_to_name(ret));
      }

      // Return to CH state
      ESP_LOGI(TAG, "Returning to CH state");
      transition_to_state(STATE_CH);

      // Re-enable BLE advertising
      ble_manager_start_advertising();
    }
    break;

  case STATE_SLEEP:
    // Sleep state (for future implementation)
    break;
  }
}
