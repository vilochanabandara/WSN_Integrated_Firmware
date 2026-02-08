#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>

#include "aht21_sensor.h"
#include "bme280_sensor.h"
#include "config.h"
#include "ens160_sensor.h"
#include "gy271_sensor.h"
#include "i2c_bus.h"
#include "ina219_sensor.h"
#include "inmp441_sensor.h"
#include "sensor_config.h"
#include "sensors.h"

#include "auth.h"
#include "battery.h"
#include "ble_manager.h"
#include "election.h"
#include "esp_now_manager.h"
#include "led_manager.h"
#include "logger.h"
#include "metrics.h"
#include "neighbor_manager.h"
#include "nvs_flash.h"
#include "persistence.h"
#include "pme.h"
#include "rf_receiver.h"
#include "state_machine.h"
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "main";

// Set to 0 to disable the one-shot forced log/flush used for SPIFFS
// verification.
#ifndef LOGGER_FORCE_FLUSH_TEST
#define LOGGER_FORCE_FLUSH_TEST 0
#endif

void compression_bench_run_once(void);

static sensor_config_t s_sensor_config = {0};

// Per-sensor last read timestamps for interval control
static uint64_t s_last_env_read_ms = 0;
static uint64_t s_last_gas_read_ms = 0;
static uint64_t s_last_mag_read_ms = 0;
static uint64_t s_last_power_read_ms = 0;
static uint64_t s_last_audio_read_ms = 0;

// Change-detection thresholds to suppress redundant logs
static const float THRESH_TEMP_C = 0.1f;
static const float THRESH_HUM_PCT = 0.5f;
static const float THRESH_PRESS_HPA = 0.5f;
static const float THRESH_VBUS_V = 0.005f;    // 5 mV
static const float THRESH_CURRENT_MA = 5.0f;  // 5 mA
static const float THRESH_AUDIO_RMS = 0.001f; // arbitrary small delta
static const float THRESH_MAG_UT = 0.05f;
static const float THRESH_SHUNT_MV = 1.0f;

typedef struct {
  bool have;
  float bme_t, bme_h, bme_p;
  float aht_t, aht_h;
  uint16_t aqi, tvoc, eco2;
  float mag_x, mag_y, mag_z;
  float bus_v, shunt_mv, current_ma;
  unsigned audio_samples;
  float audio_rms, audio_peak;
} last_log_t;

static last_log_t s_last_log = {.have = false};

static bool changed_f(float prev, float curr, float thresh) {
  return fabsf(curr - prev) >= thresh;
}

static uint32_t sample_period_ms_for_mode(pme_mode_t mode) {
  // Main loop rate - runs fast for responsiveness
  // Individual sensors check their own intervals
  switch (mode) {
  case PME_MODE_NORMAL:
    return 2000; // Fast loop (2 seconds) for responsive sampling
  case PME_MODE_POWER_SAVE:
    return 5000; // Moderate loop (5 seconds) to save power
  case PME_MODE_CRITICAL:
    return 2000; // Won't be used (deep sleep)
  default:
    return 2000;
  }
}

static void log_wakeup_reason(void) {
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  switch (cause) {
  case ESP_SLEEP_WAKEUP_TIMER:
    ESP_LOGI(TAG, "wakeup cause: timer");
    break;
  case ESP_SLEEP_WAKEUP_UNDEFINED:
    ESP_LOGI(TAG, "wakeup cause: power-on or reset");
    break;
  default:
    ESP_LOGI(TAG, "wakeup cause: %d", cause);
    break;
  }
}

static void compression_bench_task(void *arg) {
  // Let the system settle before running the bench to avoid early log
  // contention.
  vTaskDelay(pdMS_TO_TICKS(500));
  compression_bench_run_once();
  vTaskDelete(NULL);
}

// Apply one key=value to sensor config (serial console, same keys as BLE).
static esp_err_t apply_config_key_value(const char *key_value) {
  char buf[128];
  size_t len = strlen(key_value);
  if (len >= sizeof(buf))
    return ESP_ERR_INVALID_ARG;
  memcpy(buf, key_value, len + 1);
  char *eq = strchr(buf, '=');
  if (!eq)
    return ESP_ERR_INVALID_ARG;
  *eq = '\0';
  const char *key = buf;
  const char *value = eq + 1;
  sensor_config_t cfg;
  sensor_config_get(&cfg);
  if (strcmp(key, "audio_interval_ms") == 0) {
    cfg.audio_interval_ms = (uint32_t)atoi(value);
  } else if (strcmp(key, "env_sensor_interval_ms") == 0) {
    cfg.env_sensor_interval_ms = (uint32_t)atoi(value);
  } else if (strcmp(key, "gas_sensor_interval_ms") == 0) {
    cfg.gas_sensor_interval_ms = (uint32_t)atoi(value);
  } else if (strcmp(key, "mag_sensor_interval_ms") == 0) {
    cfg.mag_sensor_interval_ms = (uint32_t)atoi(value);
  } else if (strcmp(key, "power_sensor_interval_ms") == 0) {
    cfg.power_sensor_interval_ms = (uint32_t)atoi(value);
  } else if (strcmp(key, "inmp441_enabled") == 0) {
    cfg.inmp441_enabled = (atoi(value) != 0);
  } else if (strcmp(key, "bme280_enabled") == 0) {
    cfg.bme280_enabled = (atoi(value) != 0);
  } else if (strcmp(key, "ens160_enabled") == 0) {
    cfg.ens160_enabled = (atoi(value) != 0);
  } else if (strcmp(key, "gy271_enabled") == 0) {
    cfg.gy271_enabled = (atoi(value) != 0);
  } else if (strcmp(key, "audio_sample_rate") == 0) {
    cfg.audio_sample_rate = (uint32_t)atoi(value);
  } else if (strcmp(key, "audio_duration_ms") == 0) {
    cfg.audio_duration_ms = (uint32_t)atoi(value);
  } else if (strcmp(key, "beacon_interval_ms") == 0) {
    cfg.beacon_interval_ms = (uint32_t)atoi(value);
  } else if (strcmp(key, "beacon_offset_ms") == 0) {
    cfg.beacon_offset_ms = (uint32_t)atoi(value);
  } else {
    ESP_LOGW(TAG, "Unknown config key: %s", key);
    return ESP_ERR_NOT_FOUND;
  }
  sensor_config_update(&cfg);
  sensor_config_save(&cfg);
  ESP_LOGI(TAG, "Config updated: %s=%s", key, value);
  return ESP_OK;
}

// Print cluster report for host script (CLUSTER command).
static void cluster_report_print(void) {
  node_metrics_t m = metrics_get_current();
  uint32_t ch_id = neighbor_manager_get_current_ch();
  size_t member_count = neighbor_manager_get_member_count();
  uint64_t mac = g_mac_addr;

  printf("CLUSTER_REPORT_START\n");
  printf("NODE_ID=%" PRIu32 "\n", g_node_id);
  printf("MAC=%02x:%02x:%02x:%02x:%02x:%02x\n",
         (unsigned)((mac >> 40) & 0xff), (unsigned)((mac >> 32) & 0xff),
         (unsigned)((mac >> 24) & 0xff), (unsigned)((mac >> 16) & 0xff),
         (unsigned)((mac >> 8) & 0xff), (unsigned)(mac & 0xff));
  printf("ROLE=%s\n", state_machine_get_state_name());
  printf("IS_CH=%d\n", g_is_ch ? 1 : 0);
  printf("STELLAR_SCORE=%.4f\n", m.stellar_score);
  printf("COMPOSITE_SCORE=%.4f\n", m.composite_score);
  printf("BATTERY=%.2f\n", m.battery);
  printf("TRUST=%.2f\n", m.trust);
  printf("LINK_QUALITY=%.2f\n", m.link_quality);
  printf("CURRENT_CH=%" PRIu32 "\n", ch_id);
  printf("MEMBER_COUNT=%zu\n", member_count);

  neighbor_entry_t neighbors[MAX_NEIGHBORS];
  size_t n = neighbor_manager_get_all(neighbors, MAX_NEIGHBORS);
  for (size_t i = 0; i < n; i++) {
    if (!neighbors[i].is_ch && neighbors[i].verified &&
        neighbors[i].rssi_ewma >= CLUSTER_RADIUS_RSSI_THRESHOLD) {
      printf("MEMBER_ID=%" PRIu32 "\n", neighbors[i].node_id);
      printf("MEMBER_MAC=%02x:%02x:%02x:%02x:%02x:%02x\n",
             neighbors[i].mac_addr[0], neighbors[i].mac_addr[1],
             neighbors[i].mac_addr[2], neighbors[i].mac_addr[3],
             neighbors[i].mac_addr[4], neighbors[i].mac_addr[5]);
      printf("MEMBER_SCORE=%.4f\n", neighbors[i].score);
    }
  }
  printf("CLUSTER_REPORT_END\n");
}

// Serial console task: "CONFIG key=value" and "CLUSTER" for report.
static void console_config_task(void *pvParameters) {
  char line[128];
  int pos = 0;
  ESP_LOGI(TAG, "Serial: CONFIG key=value or CLUSTER for report");
  for (;;) {
    int c = fgetc(stdin);
    if (c == EOF || c == '\r') {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    if (c == '\n') {
      line[pos] = '\0';
      if (pos > 0) {
        if (strncmp(line, "CONFIG ", 7) == 0) {
          esp_err_t err = apply_config_key_value(line + 7);
          if (err == ESP_OK) {
            printf("OK config applied\n");
          } else {
            printf("ERR config %s\n", esp_err_to_name(err));
          }
        } else if (strcmp(line, "CLUSTER") == 0) {
          cluster_report_print();
        }
      }
      pos = 0;
      continue;
    }
    if (pos < (int)sizeof(line) - 1)
      line[pos++] = (char)c;
  }
}

static void logger_force_sample_flush(void) {
#if LOGGER_FORCE_FLUSH_TEST
  ESP_LOGW(TAG, "FORCE FLUSH TEST: writing sample lines");
  for (int i = 0; i < 4; ++i) {
    char buf[160];
    int n = snprintf(buf, sizeof(buf),
                     "{\"ts_ms\":%llu,\"force_sample\":true,\"seq\":%d,"
                     "\"note\":\"mslg smoke test\"}",
                     (unsigned long long)(esp_timer_get_time() / 1000ULL), i);
    if (n > 0 && n < (int)sizeof(buf)) {
      (void)logger_append_line(buf);
    }
  }
  esp_err_t fr = logger_flush();
  ESP_LOGW(TAG, "FORCE FLUSH TEST: flush %s", (fr == ESP_OK) ? "ok" : "failed");
#endif
}

// ========== STELLAR CLUSTER TASKS (from original clusterCreation) ==========

// State machine task - runs every 100ms for responsive state transitions
static void state_machine_task(void *pvParameters) {
  ESP_LOGI(TAG, "State machine task started");

  while (1) {
    state_machine_run();
    vTaskDelay(pdMS_TO_TICKS(100)); // Run every 100ms
  }
}

// Metrics update task - updates STELLAR metrics every second
static void metrics_task(void *pvParameters) {
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

// ===========================================================================

void app_main(void) {
  // Initialize NVS
  esp_err_t nvs_ret = nvs_flash_init();
  if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvs_ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(nvs_ret);

  // Initialize Managers
  // Use ble_manager directly (NimBLE) instead of legacy ble_beacon (Bluedroid)

  // Initialize STELLAR subsystems (CRITICAL - must be before
  // state_machine_init)
  auth_init();
  metrics_init();
  neighbor_manager_init();
  election_init();
  persistence_init(); // Initialize persistence before other systems

  ble_manager_init();
  led_manager_init();
  ESP_ERROR_CHECK(logger_init());

  // Initialize ESP-NOW
  esp_now_manager_init();

  // Initialize RF Receiver
  rf_receiver_init();

  // Initialize State Machine (MUST be after all subsystems)
  state_machine_init();
  vTaskDelay(pdMS_TO_TICKS(50));

  // Load sensor configuration from NVS
  ESP_ERROR_CHECK(sensor_config_load(&s_sensor_config));
  ESP_LOGI(TAG,
           "Sensor config: audio_interval=%" PRIu32 "ms, env_interval=%" PRIu32
           "ms",
           s_sensor_config.audio_interval_ms,
           s_sensor_config.env_sensor_interval_ms);

  // Display node ID
  char node_id[18];
  if (logger_get_node_id(node_id, sizeof(node_id)) == ESP_OK) {
    ESP_LOGI(TAG, "Node ID: %s", node_id);
  }

  // Display storage status
  size_t total = 0, used = 0;
  if (logger_get_storage_usage(&used, &total) == ESP_OK) {
    ESP_LOGI(TAG, "Storage: %u/%u bytes (%.1f%% used)", (unsigned)used,
             (unsigned)total, (total > 0) ? (100.0f * used / total) : 0.0f);
  }

  // Emit a small batch of lines and flush once so SPIFFS dump shows an MSLG
  // block.
  logger_force_sample_flush();
  vTaskDelay(pdMS_TO_TICKS(20));

  log_wakeup_reason();

  // ---- Battery + PME before cluster tasks (so metrics_task sees valid battery) ----
  battery_cfg_t bcfg = {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_3, // ADC1 CH3 = GPIO4 (ESP32-S3)
      .atten = ADC_ATTEN_DB_2_5,
      .r1_ohm = 220000,
      .r2_ohm = 100000,
      .samples = 32,
  };
  ESP_ERROR_CHECK(battery_init(&bcfg));

  pme_config_t cfg = {
      .th = {.normal_min_pct = 60, .power_save_min_pct = 10},
      .fake_start_pct = 100,
      .fake_drop_per_tick = 1,
      .fake_tick_ms = 1000,
  };
  ESP_ERROR_CHECK(pme_init(&cfg));

  // ========== STELLAR CLUSTER TASKS (after battery/PME so metrics_task gets valid battery) ==========
  ESP_LOGI(TAG, "Creating STELLAR cluster tasks...");
  xTaskCreate(state_machine_task, "state_machine", 8192, NULL, 5, NULL);
  xTaskCreate(metrics_task, "metrics", 4096, NULL, 4, NULL);
  (void)xTaskCreate(console_config_task, "console_cfg", 4096, NULL,
                    tskIDLE_PRIORITY + 1, NULL);

  esp_err_t ret;

  // Initialize I2C bus BEFORE sensor initialization
  vTaskDelay(pdMS_TO_TICKS(30));
  ret = ms_i2c_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
#if !ENABLE_MOCK_SENSORS
    ESP_ERROR_CHECK(ret); // Fatal error - can't proceed without I2C
#else
    ESP_LOGW(TAG, "Proceeding in MOCK SENSOR mode without I2C");
#endif
  }
  vTaskDelay(pdMS_TO_TICKS(20));

  // Sensor init with retry logic (3 attempts)
  const int MAX_RETRIES = 3;
  const int RETRY_DELAY_MS = 500;

  for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
    ret = bme280_init();
    if (ret == ESP_OK)
      break;
    ESP_LOGW(TAG, "BME280 init attempt %d/%d failed: %s", attempt, MAX_RETRIES,
             esp_err_to_name(ret));
    if (attempt < MAX_RETRIES)
      vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
  }
  if (ret != ESP_OK)
    ESP_LOGW(TAG, "BME280 init skipped after %d retries", MAX_RETRIES);

  for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
    ret = aht21_init();
    if (ret == ESP_OK)
      break;
    ESP_LOGW(TAG, "AHT21 init attempt %d/%d failed: %s", attempt, MAX_RETRIES,
             esp_err_to_name(ret));
    if (attempt < MAX_RETRIES)
      vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
  }
  if (ret != ESP_OK)
    ESP_LOGW(TAG, "AHT21 init skipped after %d retries", MAX_RETRIES);

  for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
    ret = ens160_init();
    if (ret == ESP_OK)
      break;
    ESP_LOGW(TAG, "ENS160 init attempt %d/%d failed: %s", attempt, MAX_RETRIES,
             esp_err_to_name(ret));
    if (attempt < MAX_RETRIES)
      vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
  }
  if (ret != ESP_OK)
    ESP_LOGW(TAG, "ENS160 init skipped after %d retries", MAX_RETRIES);

  for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
    ret = gy271_init();
    if (ret == ESP_OK)
      break;
    ESP_LOGW(TAG, "GY-271 init attempt %d/%d failed: %s", attempt, MAX_RETRIES,
             esp_err_to_name(ret));
    if (attempt < MAX_RETRIES)
      vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
  }
  if (ret != ESP_OK)
    ESP_LOGW(TAG, "GY-271 init skipped after %d retries", MAX_RETRIES);

  for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
    ret = ina219_init_basic();
    if (ret == ESP_OK)
      break;
    ESP_LOGW(TAG, "INA219 init attempt %d/%d failed: %s", attempt, MAX_RETRIES,
             esp_err_to_name(ret));
    if (attempt < MAX_RETRIES)
      vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
  }
  if (ret != ESP_OK)
    ESP_LOGW(TAG, "INA219 init skipped after %d retries", MAX_RETRIES);

  // INMP441 I2S microphone (default config: GPIO5/6/7, 16kHz)
  inmp441_config_t inmp_cfg = {.ws_pin = 5,
                               .sck_pin = 6,
                               .sd_pin = 7,
                               .sample_rate = 16000,
                               .bits_per_sample = 16,
                               .buffer_samples = 512};
  for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
    ret = inmp441_init(&inmp_cfg);
    if (ret == ESP_OK)
      break;
    ESP_LOGW(TAG, "INMP441 init attempt %d/%d failed: %s", attempt, MAX_RETRIES,
             esp_err_to_name(ret));
    if (attempt < MAX_RETRIES)
      vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
  }
  if (ret != ESP_OK)
    ESP_LOGW(TAG, "INMP441 init skipped after %d retries", MAX_RETRIES);

  // Run sanity check AFTER all sensors initialized to detect presence
  sensors_raw_sanity_check();
  vTaskDelay(pdMS_TO_TICKS(200));

  // Dump log file to UART on boot (commented out - triggers watchdog on large
  // files) vTaskDelay(pdMS_TO_TICKS(2000)); // Let system settle before dump
  // logger_dump_to_uart();
  // vTaskDelay(pdMS_TO_TICKS(1000)); // Give time for UART output

  static int s_config_reload_count = 0;
  static bool s_first_loop = true;
  while (1) {

    if (s_first_loop) {
      ESP_LOGI(TAG, "Main loop running (state machine + metrics active)");
      s_first_loop = false;
    }

    // Reload config periodically so BLE/serial updates take effect
    if (++s_config_reload_count >= 15) {
      s_config_reload_count = 0;
      (void)sensor_config_load(&s_sensor_config);
    }

    // ---- Battery read (real) ----
    uint32_t vadc_mv = 0, vbat_mv = 0;
    uint8_t batt_pct = 0;
    bool use_mock_battery = false;

#if ENABLE_MOCK_SENSORS
    use_mock_battery = true;
#endif

    // If not forced mock, try real read. If read fails or voltage is critically
    // low (floating pin), fallback to mock.
    if (!use_mock_battery &&
        battery_read(&vadc_mv, &vbat_mv, &batt_pct) == ESP_OK &&
        vbat_mv > 2000) {
      ESP_LOGI(TAG, "BAT vadc=%lumV vbat=%lumV pct=%u%%",
               (unsigned long)vadc_mv, (unsigned long)vbat_mv, batt_pct);

      // Feed PME with real percentage
      pme_set_batt_pct(batt_pct);
    } else {
      // Mock Mode or Fallback
#if ENABLE_MOCK_SENSORS
      // Simulate battery drain
      static uint8_t sim_batt = 100;
      static int calls = 0;
      if (calls++ % 10 == 0 && sim_batt > 10)
        sim_batt--;
      batt_pct = sim_batt;
      vbat_mv = 3300 + (sim_batt * 9); // Approximate 3.3V - 4.2V mapped
      ESP_LOGW(TAG, "[MOCK] Battery: %u%% (Simulated)", batt_pct);
      pme_set_batt_pct(batt_pct);
#else
      // Fallback for USB power (no battery detected but node is running)
      // Assume 100% to prevent re-election loop due to "dead battery"
      batt_pct = 100;
      vbat_mv = 5000; // USB voltage approx
      ESP_LOGW(TAG, "Battery not detected (USB Power?), assuming 100%%");
      pme_set_batt_pct(batt_pct);
#endif
    }

    // PME mode is always derived from the latest stored % (real if available)
    pme_mode_t mode = pme_get_mode();
    ESP_LOGI(TAG, "PME batt=%u%% mode=%s", pme_get_batt_pct(),
             pme_mode_to_str(mode));

    // Check storage and warn if nearing full
    if (logger_storage_critical()) {
      ESP_LOGW(TAG,
               "Storage CRITICAL (>95%%), will clear old data on next write");
    } else if (logger_storage_warning()) {
      ESP_LOGW(TAG, "Storage WARNING (>90%%)");
    }

    // NOTE: metrics_update() and state_machine_run() are now handled by
    // dedicated tasks (metrics_task runs every 1s, state_machine_task runs
    // every 100ms) This allows the main loop to focus on sensor sampling
    // without blocking STELLAR operations

    // ---- Per-sensor interval timing (mode-dependent) ----
    uint64_t now_ms = esp_timer_get_time() / 1000ULL;

    // Calculate intervals based on PME mode
    uint32_t env_interval_ms, gas_interval_ms, mag_interval_ms,
        power_interval_ms, audio_interval_ms;

    switch (mode) {
    case PME_MODE_NORMAL:
      // Slower sampling in Normal mode (targeting week-long retention)
      env_interval_ms = 60000;    // 60 seconds
      gas_interval_ms = 180000;   // 180 seconds (3 minutes, 3x env)
      mag_interval_ms = 60000;    // 60 seconds
      power_interval_ms = 60000;  // 60 seconds
      audio_interval_ms = 600000; // 600 seconds (10 minutes)
      break;

    case PME_MODE_POWER_SAVE:
      // Further slowed sampling in PowerSave mode
      env_interval_ms = 300000;   // 300 seconds (5 minutes)
      gas_interval_ms = 600000;   // 600 seconds (10 minutes, 2x env)
      mag_interval_ms = 300000;   // 300 seconds
      power_interval_ms = 120000; // 120 seconds (still track battery)
      audio_interval_ms = 900000; // 900 seconds (15 minutes)
      break;

    case PME_MODE_CRITICAL:
    default:
      // Minimal sampling in Critical mode
      env_interval_ms = 7200000; // 7200 seconds (2 hours)
      gas_interval_ms = 7200000; // 2 hours
      mag_interval_ms = 7200000; // 2 hours
      power_interval_ms = 60000; // 60 seconds (1 minute, still monitor battery)
      audio_interval_ms = 7200000; // 2 hours
      break;
    }

    bool time_for_env = (now_ms - s_last_env_read_ms) >= env_interval_ms;
    bool time_for_gas = (now_ms - s_last_gas_read_ms) >= gas_interval_ms;
    bool time_for_mag = (now_ms - s_last_mag_read_ms) >= mag_interval_ms;
    bool time_for_power = (now_ms - s_last_power_read_ms) >= power_interval_ms;
    bool time_for_audio = (now_ms - s_last_audio_read_ms) >= audio_interval_ms;

    // ---- Sensor reads (interval and mode-aware) ----
    bme280_reading_t bme = {0};
    aht21_reading_t aht = {0};
    uint8_t aht_raw[AHT21_RAW_LEN] = {0};
    ens160_reading_t ens = {0};
    gy271_reading_t mag = {0};
    ina219_basic_t ina = {0};
    inmp441_reading_t audio = {0};

    bool do_full = (mode == PME_MODE_NORMAL);
    bool do_light =
        (mode != PME_MODE_CRITICAL); // Passive keeps light sensors + INA
    bool do_audio = (mode == PME_MODE_NORMAL); // Allow mic in Normal mode only

    bool ok_bme = false;
    bool ok_aht = false;
    bool ok_ens = false;
    bool ok_mag = false;
    bool ok_ina = false;
    bool ok_audio = false;

    // Environmental sensors (BME280, AHT21) - interval + mode gated
    if (do_full && time_for_env && s_sensor_config.bme280_enabled) {
      ok_bme = (bme280_read(&bme) == ESP_OK);
#if ENABLE_MOCK_SENSORS
      if (!ok_bme) {
        float t_offset = 25.0f + 5.0f * sinf(now_ms / 10000.0f);
        bme.temperature_c = t_offset;
        bme.humidity_pct = 50.0f + 10.0f * cosf(now_ms / 10000.0f);
        bme.pressure_hpa = 1013.0f + 5.0f * sinf(now_ms / 20000.0f);
        ok_bme = true;
        ESP_LOGW(TAG, "[MOCK] BME280 Data Generated");
      }
#endif
      if (ok_bme)
        s_last_env_read_ms = now_ms;
    }

    if (do_light && time_for_env && s_sensor_config.aht21_enabled) {
      ok_aht = (aht21_read_with_raw(&aht, aht_raw) == ESP_OK);
#if ENABLE_MOCK_SENSORS
      if (!ok_aht) {
        aht.temperature_c = 25.0f + 5.0f * sinf(now_ms / 10000.0f);
        aht.humidity_pct = 50.0f + 10.0f * cosf(now_ms / 10000.0f);
        ok_aht = true;
      }
#endif
      if (ok_aht)
        s_last_env_read_ms = now_ms;
    }

    // Gas sensor (ENS160) - interval + mode gated
    if (do_light && time_for_gas && s_sensor_config.ens160_enabled) {
      ok_ens = (ens160_read_iaq(&ens) == ESP_OK);
#if ENABLE_MOCK_SENSORS
      if (!ok_ens) {
        ens.aqi_uba = 1 + (now_ms / 1000) % 5;
        ens.tvoc_ppb = 10 + (now_ms / 1000) % 50;
        ens.eco2_ppm = 400 + (now_ms / 1000) % 100;
        ens.status = 0;
        ok_ens = true;
      }
#endif
      if (ok_ens)
        s_last_gas_read_ms = now_ms;
    }

    // Power monitor (INA219) - interval + mode gated
    if (do_light && time_for_power && s_sensor_config.ina219_enabled) {
      ok_ina = (ina219_read_basic(&ina) == ESP_OK);
#if ENABLE_MOCK_SENSORS
      if (!ok_ina) {
        ina.bus_voltage_v = 4.0f;
        ina.shunt_voltage_mv = 10.0f + 5.0f * sinf(now_ms / 5000.0f);
        ina.current_ma = ina.shunt_voltage_mv / 0.1f; // assume 0.1 ohm
        ok_ina = true;
      }
#endif
      if (ok_ina)
        s_last_power_read_ms = now_ms;
    }

    // Magnetometer (GY-271) - interval + mode gated
    if (do_full && time_for_mag && s_sensor_config.gy271_enabled) {
      ok_mag = (gy271_read(&mag) == ESP_OK);
#if ENABLE_MOCK_SENSORS
      if (!ok_mag) {
        mag.x_uT = 30.0f * cosf(now_ms / 5000.0f);
        mag.y_uT = 30.0f * sinf(now_ms / 5000.0f);
        mag.z_uT = 40.0f;
        ok_mag = true;
      }
#endif
      if (ok_mag)
        s_last_mag_read_ms = now_ms;
    }

    // Microphone (INMP441) - interval + mode gated
    if (do_audio && time_for_audio && s_sensor_config.inmp441_enabled) {
      ok_audio = (inmp441_read(&audio) == ESP_OK && audio.valid);
#if ENABLE_MOCK_SENSORS
      if (!ok_audio) {
        audio.count = 512;
        audio.rms_amplitude = 0.05f + 0.02f * sinf(now_ms / 1000.0f);
        audio.peak_amplitude = audio.rms_amplitude * 1.414f;
        audio.timestamp_ms = now_ms;
        audio.valid = true;
        ok_audio = true;
      }
#endif
      if (ok_audio)
        s_last_audio_read_ms = now_ms;
    }

    if (ok_aht) {
      (void)ens160_set_env(aht.temperature_c, aht.humidity_pct);
    }

    if (ok_bme)
      ESP_LOGI(TAG, "BME280 T=%.2f C | H=%.2f %% | P=%.2f hPa",
               bme.temperature_c, bme.humidity_pct, bme.pressure_hpa);
    if (ok_aht)
      ESP_LOGI(TAG, "AHT21 T=%.2f C | H=%.2f %%", aht.temperature_c,
               aht.humidity_pct);
    if (ok_ens)
      ESP_LOGI(TAG,
               "ENS160 status: 0x%02X | AQI=%u | TVOC=%u ppb | eCO2=%u ppm",
               ens.status, ens.aqi_uba, ens.tvoc_ppb, ens.eco2_ppm);
    if (ok_mag)
      ESP_LOGI(TAG, "GY-271 status: 0x%02X | uT: X=%.2f Y=%.2f Z=%.2f",
               mag.status, mag.x_uT, mag.y_uT, mag.z_uT);
    if (ok_ina)
      ESP_LOGI(TAG, "INA219 bus=%.3f V | shunt=%.3f mV | i=%.1f mA",
               ina.bus_voltage_v, ina.shunt_voltage_mv, ina.current_ma);
    if (ok_audio)
      ESP_LOGI(TAG, "INMP441 samples=%u | rms=%.4f | peak=%.4f | ts=%lu ms",
               (unsigned)audio.count, audio.rms_amplitude, audio.peak_amplitude,
               (unsigned long)audio.timestamp_ms);

    // ---- JSON log line ----
    bool any_ok = ok_bme || ok_aht || ok_ens || ok_mag || ok_ina || ok_audio;
    if (any_ok) {
      bool changed = !s_last_log.have;

      if (ok_bme) {
        changed |=
            changed_f(s_last_log.bme_t, bme.temperature_c, THRESH_TEMP_C);
        changed |=
            changed_f(s_last_log.bme_h, bme.humidity_pct, THRESH_HUM_PCT);
        changed |=
            changed_f(s_last_log.bme_p, bme.pressure_hpa, THRESH_PRESS_HPA);
      }
      if (ok_aht) {
        changed |=
            changed_f(s_last_log.aht_t, aht.temperature_c, THRESH_TEMP_C);
        changed |=
            changed_f(s_last_log.aht_h, aht.humidity_pct, THRESH_HUM_PCT);
      }
      if (ok_ens) {
        changed |= (s_last_log.aqi != ens.aqi_uba) ||
                   (s_last_log.tvoc != ens.tvoc_ppb) ||
                   (s_last_log.eco2 != ens.eco2_ppm);
      }
      if (ok_mag) {
        changed |= changed_f(s_last_log.mag_x, mag.x_uT, THRESH_MAG_UT) ||
                   changed_f(s_last_log.mag_y, mag.y_uT, THRESH_MAG_UT) ||
                   changed_f(s_last_log.mag_z, mag.z_uT, THRESH_MAG_UT);
      }
      if (ok_ina) {
        changed |=
            changed_f(s_last_log.bus_v, ina.bus_voltage_v, THRESH_VBUS_V) ||
            changed_f(s_last_log.shunt_mv, ina.shunt_voltage_mv,
                      THRESH_SHUNT_MV) ||
            changed_f(s_last_log.current_ma, ina.current_ma, THRESH_CURRENT_MA);
      }
      if (ok_audio) {
        changed |= (s_last_log.audio_samples != audio.count) ||
                   changed_f(s_last_log.audio_rms, audio.rms_amplitude,
                             THRESH_AUDIO_RMS) ||
                   changed_f(s_last_log.audio_peak, audio.peak_amplitude,
                             THRESH_AUDIO_RMS);
      }

      if (changed) {
        char line[400];
        int n = snprintf(
            line, sizeof(line),
            "{\"ts_ms\":%llu,"
            "\"env\":{\"bme_t\":%.2f,\"bme_h\":%.2f,\"bme_p\":%.2f,"
            "\"aht_t\":%.2f,\"aht_h\":%.2f},"
            "\"gas\":{\"aqi\":%u,\"tvoc\":%u,\"eco2\":%u},"
            "\"mag\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
            "\"power\":{\"bus_v\":%.3f,\"shunt_mv\":%.3f,\"i_ma\":%.2f},"
            "\"audio\":{\"samples\":%u,\"rms\":%.4f,\"peak\":%.4f}}",
            (unsigned long long)(esp_timer_get_time() / 1000ULL),
            ok_bme ? bme.temperature_c : 0.0f, ok_bme ? bme.humidity_pct : 0.0f,
            ok_bme ? bme.pressure_hpa : 0.0f, ok_aht ? aht.temperature_c : 0.0f,
            ok_aht ? aht.humidity_pct : 0.0f, ok_ens ? ens.aqi_uba : 0,
            ok_ens ? ens.tvoc_ppb : 0, ok_ens ? ens.eco2_ppm : 0,
            ok_mag ? mag.x_uT : 0.0f, ok_mag ? mag.y_uT : 0.0f,
            ok_mag ? mag.z_uT : 0.0f, ok_ina ? ina.bus_voltage_v : 0.0f,
            ok_ina ? ina.shunt_voltage_mv : 0.0f,
            ok_ina ? ina.current_ma : 0.0f,
            ok_audio ? (unsigned)audio.count : 0,
            ok_audio ? audio.rms_amplitude : 0.0f,
            ok_audio ? audio.peak_amplitude : 0.0f);

        if (n > 0 && n < (int)sizeof(line)) {
          if (logger_append_line(line) == ESP_OK) {
            s_last_log.have = true;
            if (ok_bme) {
              s_last_log.bme_t = bme.temperature_c;
              s_last_log.bme_h = bme.humidity_pct;
              s_last_log.bme_p = bme.pressure_hpa;
            }
            if (ok_aht) {
              s_last_log.aht_t = aht.temperature_c;
              s_last_log.aht_h = aht.humidity_pct;
            }
            if (ok_ens) {
              s_last_log.aqi = ens.aqi_uba;
              s_last_log.tvoc = ens.tvoc_ppb;
              s_last_log.eco2 = ens.eco2_ppm;
            }
            if (ok_mag) {
              s_last_log.mag_x = mag.x_uT;
              s_last_log.mag_y = mag.y_uT;
              s_last_log.mag_z = mag.z_uT;
            }
            if (ok_ina) {
              s_last_log.bus_v = ina.bus_voltage_v;
              s_last_log.shunt_mv = ina.shunt_voltage_mv;
              s_last_log.current_ma = ina.current_ma;
            }
            if (ok_audio) {
              s_last_log.audio_samples = audio.count;
              s_last_log.audio_rms = audio.rms_amplitude;
              s_last_log.audio_peak = audio.peak_amplitude;
            }
          }
        } else {
          ESP_LOGW(TAG, "Log line truncated, skipped");
        }
      }
    }

    // ---- Storage monitoring ----
    if (logger_storage_warning()) {
      size_t used = 0, total = 0;
      (void)logger_get_storage_usage(&used, &total);
      ESP_LOGW(TAG, "Storage warning: %u/%u bytes (%.1f%% full)",
               (unsigned)used, (unsigned)total,
               (total > 0) ? (100.0f * used / total) : 0.0f);
    }

    // ---- Deep sleep decision ----
    if (mode == PME_MODE_CRITICAL) {
      uint32_t sleep_ms = 1800000; // 30 minutes - wake to recheck battery
      ESP_LOGW(TAG,
               "PME critical: entering deep sleep for %" PRIu32
               " ms (will recheck battery)",
               sleep_ms);

      // Ensure buffered logs are written before power-down.
      (void)logger_flush();

      ESP_ERROR_CHECK(
          esp_sleep_enable_timer_wakeup((uint64_t)sleep_ms * 1000ULL));
      esp_deep_sleep_start();
    }

    uint32_t period_ms = sample_period_ms_for_mode(mode);
    vTaskDelay(pdMS_TO_TICKS(period_ms));
  }
}