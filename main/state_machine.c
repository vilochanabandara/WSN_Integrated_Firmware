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
#include <string.h>

static const char *TAG = "STATE";

node_state_t g_current_state = STATE_INIT;
bool g_is_ch = false;
uint32_t g_node_id = 0;
uint64_t g_mac_addr = 0;

// Forward declarations
extern uint8_t g_cluster_key[CLUSTER_KEY_SIZE];

static uint64_t state_entry_time = 0;
static uint64_t last_ch_beacon_time = 0;

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
           (new_state == STATE_INIT        ? "INIT"
            : new_state == STATE_DISCOVER  ? "DISCOVER"
            : new_state == STATE_CANDIDATE ? "CANDIDATE"
            : new_state == STATE_CH        ? "CH"
            : new_state == STATE_MEMBER    ? "MEMBER"
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
      }
    } else {
      // Discovery complete, move to candidate
      ble_manager_stop_advertising();
      ble_manager_stop_scanning();
      transition_to_state(STATE_CANDIDATE);
      election_reset_window();
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
    }
    break;

  case STATE_MEMBER:
    // Member duties
    {
      // Update metrics
      // metrics_update(); // Handled by metrics_task

      // Check if CH is still valid
      uint32_t current_ch = neighbor_manager_get_current_ch();
      if (current_ch == 0) {
        ESP_LOGI(TAG, "CH lost, returning to candidate");
        transition_to_state(STATE_CANDIDATE);
        election_reset_window();
        break;
      }

      // Update advertisement (as member)
      ble_manager_update_advertisement();

      // Member duties: duty-cycle radio, send keep-alive, etc.
      neighbor_manager_cleanup_stale();

      // Check if re-election is needed
      if (election_check_reelection_needed()) {
        ESP_LOGI(TAG, "Re-election needed, returning to candidate");
        transition_to_state(STATE_CANDIDATE);
        election_reset_window();
      }

      // Send dummy sensor data to CH (throttled to avoid flooding)
      static uint64_t last_data_send = 0;
      uint64_t now_ms = esp_timer_get_time() / 1000;

      if (current_ch != 0 && (now_ms - last_data_send) >=
                                 1000) { // Send every 1 second (TEST MODE)
        uint8_t ch_mac[6];
        if (neighbor_manager_get_ch_mac(ch_mac)) {
          char *dummy_data = "Sensor Data";
          esp_err_t ret = esp_now_manager_send_data(
              ch_mac, (uint8_t *)dummy_data, strlen(dummy_data));
          if (ret == ESP_OK) {
            last_data_send = now_ms;
          } else {
            ESP_LOGW(TAG, "Failed to send data to CH: %s",
                     esp_err_to_name(ret));
          }
        } else {
          ESP_LOGW(TAG, "CH MAC not found, cannot send data");
        }
      }
    }
    break;

  case STATE_SLEEP:
    // Sleep state (for future implementation)
    break;
  }
}
