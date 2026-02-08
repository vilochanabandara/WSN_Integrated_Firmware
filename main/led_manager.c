#include "led_manager.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

// GPIO for WS2812 RGB LED on ESP32-S3-DevKitC-1
#define LED_GPIO 48

static const char *TAG = "LED_MANAGER";

static TaskHandle_t led_task_handle = NULL;
static led_strip_handle_t led_strip;
static volatile node_state_t current_led_state = STATE_INIT;

static void led_task(void *pvParameters) {
  while (1) {
    switch (current_led_state) {
    case STATE_CH:
      // CH = SOLID BLUE
      // Standard RGB Mapping verified.
      led_strip_set_pixel(led_strip, 0, 0, 0, 50); // Blue
      led_strip_refresh(led_strip);
      vTaskDelay(pdMS_TO_TICKS(1000));
      break;

    case STATE_MEMBER:
      // MEMBER = BLINKING GREEN
      // Standard RGB Mapping verified.
      led_strip_set_pixel(led_strip, 0, 0, 50, 0); // Green
      led_strip_refresh(led_strip);
      vTaskDelay(pdMS_TO_TICKS(200));

      led_strip_clear(led_strip);
      led_strip_refresh(led_strip);
      vTaskDelay(pdMS_TO_TICKS(1800));
      break;

    case STATE_INIT:
    case STATE_DISCOVER:
    case STATE_CANDIDATE:
      // Fast Blink WHITE (All channels equal, order irrelevant)
      led_strip_set_pixel(led_strip, 0, 20, 20, 20);
      led_strip_refresh(led_strip);
      vTaskDelay(pdMS_TO_TICKS(100));

      led_strip_clear(led_strip);
      led_strip_refresh(led_strip);
      vTaskDelay(pdMS_TO_TICKS(100));
      break;

    case STATE_SLEEP:
    default:
      // OFF
      led_strip_clear(led_strip);
      led_strip_refresh(led_strip);
      vTaskDelay(pdMS_TO_TICKS(1000));
      break;
    }
  }
}

void led_manager_init(void) {
  ESP_LOGI(TAG, "Initializing LED Manager on GPIO %d (High Contrast Mode)",
           LED_GPIO);

  // RGB LED configuration
  led_strip_config_t strip_config = {
      .strip_gpio_num = LED_GPIO,
      .max_leds = 1,
      .led_model = LED_MODEL_WS2812,
      .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
      .flags.invert_out = false,
  };

  led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 10 * 1000 * 1000, // 10MHz
      .flags.with_dma = false,
  };

  ESP_ERROR_CHECK(
      led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
  led_strip_clear(led_strip);

  // Create task for LED control
  xTaskCreate(led_task, "led_task", 4096, NULL, 1, &led_task_handle);
}

void led_manager_set_state(node_state_t state) {
  if (current_led_state != state) {
    ESP_LOGI(TAG, "LED State changed: %d -> %d", current_led_state, state);
    current_led_state = state;
  }
}
