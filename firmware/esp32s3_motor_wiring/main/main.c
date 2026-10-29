#include <inttypes.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "fwr_motor_wiring";
static const gpio_num_t MOTOR_INPUTS[] = {
    GPIO_NUM_4,  // D157B AIN1
    GPIO_NUM_5,  // D157B AIN2
    GPIO_NUM_6,  // D157B BIN1
    GPIO_NUM_7,  // D157B BIN2
};

void app_main(void)
{
    // This first wiring check never sends a motor movement command.
    for (size_t i = 0; i < sizeof(MOTOR_INPUTS) / sizeof(MOTOR_INPUTS[0]); ++i) {
        ESP_ERROR_CHECK(gpio_reset_pin(MOTOR_INPUTS[i]));
        ESP_ERROR_CHECK(gpio_set_level(MOTOR_INPUTS[i], 0));
        ESP_ERROR_CHECK(gpio_set_direction(MOTOR_INPUTS[i], GPIO_MODE_OUTPUT));
    }
    ESP_LOGI(TAG, "GPIO4/5/6/7 configured as LOW outputs; no motor PWM");
    ESP_LOGI(TAG, "ESP-IDF %s; motor power must remain OFF during wiring check", esp_get_idf_version());

    uint32_t heartbeat = 0;
    while (true) {
        // Reassert coast command; useful if another task is added accidentally.
        for (size_t i = 0; i < sizeof(MOTOR_INPUTS) / sizeof(MOTOR_INPUTS[0]); ++i) {
            ESP_ERROR_CHECK(gpio_set_level(MOTOR_INPUTS[i], 0));
        }
        ESP_LOGI(TAG, "heartbeat=%" PRIu32 " AIN1=0 AIN2=0 BIN1=0 BIN2=0", heartbeat++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
