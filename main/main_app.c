#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "app_sdcard.h"
#include "app_axis6.h"
#include "app_gps.h"
#include "app_gui.h"
#include "app_vibration.h"
#include "app_power.h"
#include "ml_window.h"

static const char *TAG = "app_main";

void app_main(void)
{
    if (app_gui_start() != ESP_OK) {
        ESP_LOGE(TAG, "GUI init failed, continuing to start other subsystems");
    }

    app_sdcard_start();
    vTaskDelay(pdMS_TO_TICKS(200));

    ml_window_init();

    app_vibration_init();

    app_axis6_start();
    app_gps_start();

    app_power_start();

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
