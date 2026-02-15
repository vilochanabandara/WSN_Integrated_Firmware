#pragma once

// =============================================================================
// Algorithm robustness: single CH, hysteresis, score smoothing, tie-break by
// node_id, election stagger. Sensors/battery: real when hw connected, dummy
// when not (so cluster still runs and reports SENSORS_REAL/BATTERY_REAL).
// =============================================================================

// Cluster Config
#define CLUSTER_KEY_SIZE 32
#define MAX_NEIGHBORS 10
#define MAX_CLUSTER_SIZE 5
#define ELECTION_WINDOW_MS 10000
#define ELECTION_STAGGER_MS                                                    \
  3000 // Stagger by node_id so lowest runs first → single CH
#define SLOT_DURATION_SEC 10 // User Requested: 10s per node (Githmi Style)
#define STATE_MACHINE_TASK_STACK_SIZE 4096
#define STATE_MACHINE_TASK_PRIORITY 5
#define METRICS_TASK_STACK_SIZE 4096
#define METRICS_TASK_PRIORITY 4

// Weights
#define WEIGHT_BATTERY 0.3f
#define WEIGHT_UPTIME 0.2f
#define WEIGHT_TRUST 0.3f
#define WEIGHT_LINK_QUALITY 0.2f
#define HSR_WEIGHT 0.4f
#define REPUTATION_WEIGHT 0.3f
#define PDR_WEIGHT 0.3f
#define RSSI_EWMA_ALPHA 0.2f
#define NEIGHBOR_TIMEOUT_MS 20000
#define CLUSTER_RADIUS_RSSI_THRESHOLD -85.0f
#define CH_BEACON_TIMEOUT_MS                                                   \
  10000 // 10 seconds - Faster failure detection (was 60s)
#define CH_MEMBER_HYSTERESIS_MS                                                \
  8000 // Require CH missing this long before leaving MEMBER (reduces flicker)
#define CH_MEMBER_MISSING_CONSECUTIVE                                          \
  15 // Consecutive runs with CH missing before starting hysteresis (stability)

// STELLAR
// When 1: allow simulated battery drain in main loop when no battery hw; when
// 0: use dummy 100% only
#define ENABLE_MOCK_SENSORS 1
#define USE_STELLAR_ALGORITHM 1
// Smooth STELLAR score to avoid re-election from brief dips (0.1 = slow, 0.3 =
// fast)
#define STELLAR_SCORE_EWMA_ALPHA 0.15f
#define ENTROPY_GAMMA 1.0f
#define EWMA_VARIANCE_ALPHA 0.1f
#define MIN_WEIGHT_VALUE 0.05f
#define LYAPUNOV_BETA 0.1f
#define LYAPUNOV_ETA 0.01f
#define LYAPUNOV_LAMBDA 0.01f
#define CONVERGENCE_THRESHOLD 0.001f
#define UTILITY_LAMBDA_B 2.0f
#define UTILITY_LAMBDA_U 1.0f
#define UTILITY_GAMMA_L 2.0f
#define UPTIME_MAX_DAYS 7.0f
#define PARETO_DELTA 0.1f
#define CENTRALITY_EPSILON 0.5f
#define DISAGREE_BATTERY 0.1f
#define DISAGREE_UPTIME 0.1f
#define DISAGREE_TRUST 0.1f
#define DISAGREE_LINKQ 0.1f
#define PDR_EWMA_ALPHA 0.1f
#define TRUST_FLOOR 0.2f
#define BATTERY_LOW_THRESHOLD 0.2f
#define LINK_QUALITY_FLOOR 0.2f

// ESP-NOW
#define ESP_NOW_CHANNEL 1
#define ESP_NOW_PMK "pmk1234567890123"
#define ESP_NOW_LMK "lmk1234567890123"

// Persistence
#define SPIFFS_BASE_PATH "/spiffs"

// BLE Configuration
#define BLE_DEVICE_NAME_PREFIX "MSN-"
#define BLE_SCAN_INTERVAL_MS 100 // Scan interval
#define BLE_SCAN_WINDOW_MS 50 // Scan window (50% duty cycle to allow CPU idle)
