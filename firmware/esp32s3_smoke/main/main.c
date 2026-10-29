#include <inttypes.h>
#include <stdbool.h>

#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "fwr_smoke";

void app_main(void)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    ESP_LOGI(TAG, "ESP32-S3 serial smoke test; cores=%d, IDF=%s",
             chip.cores, esp_get_idf_version());
    ESP_LOGI(TAG, "No motor GPIO is configured by this firmware");

    uint32_t count = 0;
    while (true) {
        ESP_LOGI(TAG, "heartbeat=%" PRIu32, count++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
