#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>

// Cluster Configuration
#define CLUSTER_KEY_SIZE 32
#define CLUSTER_RADIUS_RSSI_THRESHOLD -85 // dBm
#define MAX_NEIGHBORS 20
#define ELECTION_WINDOW_MS 5000
#define CH_BEACON_INTERVAL_MS 1000 // Faster beacons (1s)
#define CH_BEACON_TIMEOUT_MS 15000 // Increased to 15s for stability

// Demo Mode for simulated metrics (set to 1 to enable)
#define DEMO_MODE 1

// Metrics Weights (must sum to 1.0)
#define WEIGHT_BATTERY 0.25f
#define WEIGHT_UPTIME 0.25f
#define WEIGHT_TRUST 0.30f
#define WEIGHT_LINK_QUALITY 0.20f

// Thresholds
#define BATTERY_LOW_THRESHOLD 0.15f
#define TRUST_FLOOR 0.30f
#define LINK_QUALITY_FLOOR 0.40f
#define MAX_CLUSTER_SIZE 10

// BLE Configuration
#define BLE_ADV_INTERVAL_MS 500 // Faster advertising (500ms)
#define BLE_SCAN_INTERVAL_MS 2000
#define BLE_SCAN_WINDOW_MS 500
#define BLE_DEVICE_NAME_PREFIX "CH_NODE_"

// ESP-NOW Configuration
#define ESP_NOW_CHANNEL 1
#define ESP_NOW_PMK "pmk1234567890123"
#define ESP_NOW_LMK "lmk1234567890123"

// Neighbor Discovery
#define NEIGHBOR_TIMEOUT_MS 30000 // Increased to 30s for stability
#define RSSI_EWMA_ALPHA 0.3f
#define PDR_EWMA_ALPHA 0.2f

// Trust Computation
#define HSR_WEIGHT 0.4f
#define PDR_WEIGHT 0.4f
#define REPUTATION_WEIGHT 0.2f

// Persistence
#define RTC_UPTIME_ADDR 0
#define SPIFFS_BASE_PATH "/spiffs"

// FreeRTOS Task Configuration
#define STATE_MACHINE_TASK_PRIORITY 3
#define STATE_MACHINE_TASK_STACK_SIZE                                          \
  8192 // Increased from 4096 to prevent stack overflow
#define METRICS_TASK_PRIORITY 5
#define METRICS_TASK_STACK_SIZE 4096
#define TASK_PRIORITY_BLE 6
#define TASK_STACK_SIZE_BLE 8192
#define TASK_PRIORITY_ELECTION 4
#define TASK_STACK_SIZE_ELECTION 4096

// HMAC Configuration
#define HMAC_LENGTH 16 // Truncated HMAC-SHA256

// Node ID (should be unique per node - in production, use MAC address)
extern uint32_t g_node_id;

// Cluster Key (should be pre-shared or derived from network key)
extern uint8_t g_cluster_key[CLUSTER_KEY_SIZE];

// ============================================
// STELLAR Algorithm Parameters
// Stochastic Trust-Enhanced Lyapunov-stable
// Leader Allocation and Ranking
// ============================================

// Lyapunov Stability Parameters
#define LYAPUNOV_ETA 0.05f   // Learning rate η for gradient descent
#define LYAPUNOV_BETA 0.02f  // Mean-reversion rate β
#define LYAPUNOV_LAMBDA 0.1f // Regularization parameter λ

// Entropy Confidence Parameters
#define ENTROPY_GAMMA 2.0f       // Confidence scaling γ (softmax temperature)
#define EWMA_VARIANCE_ALPHA 0.2f // Variance estimation EWMA coefficient

// Utility Function Shape Parameters
#define UTILITY_LAMBDA_B 3.0f  // Battery utility curvature (concave exp)
#define UTILITY_LAMBDA_U 2.0f  // Uptime utility saturation rate (tanh)
#define UTILITY_GAMMA_L 0.7f   // Link quality power utility exponent
#define UPTIME_MAX_DAYS 365.0f // Normalization constant for uptime

// Pareto Optimization Parameters
#define PARETO_DELTA 0.15f      // Dominance bonus coefficient δ
#define CENTRALITY_EPSILON 0.1f // Centrality penalty coefficient ε

// Nash Bargaining Disagreement Points (minimum acceptable values)
#define DISAGREE_BATTERY 0.15f // Minimum battery for Nash solution
#define DISAGREE_TRUST 0.30f   // Minimum trust for Nash solution
#define DISAGREE_LINKQ 0.40f   // Minimum link quality for Nash solution
#define DISAGREE_UPTIME 0.0f   // Minimum uptime for Nash solution

// Convergence Parameters
#define WEIGHT_UPDATE_INTERVAL_MS 5000 // Weight adaptation interval
#define CONVERGENCE_THRESHOLD 0.01f    // Weight convergence ε threshold
#define MIN_WEIGHT_VALUE 0.05f         // Minimum weight (prevent zero)

// STELLAR Algorithm Enable Flag (set to 0 to use legacy algorithm)
#define USE_STELLAR_ALGORITHM 1

#endif // CONFIG_H
