#include "app_power.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_vibration.h"
#include "app_gui.h"
#include "sdkconfig.h"

#define KEY_GPIO            ((gpio_num_t)CONFIG_JOFTMODE_POWER_KEY_GPIO)
#define SCAN_INTERVAL_MS    20
#define LONG_PRESS_MS       2000

static TaskHandle_t s_power_task = NULL;
static bool s_power_on = true;  // cached software state for the pseudo power domain

static void apply_power_state(bool on)
{
    s_power_on = on;
    if (on) {
        app_vibration_stop();
        app_vibration_pulse_ms(120);  // short pulse to indicate wake
        app_gui_screen_on();
    } else {
        app_vibration_stop();
        app_vibration_pulse_ms(80);   // shorter pulse to indicate suspend
        app_gui_screen_off();
    }
}

static void key_task(void *arg)
{
    bool pressed_prev = false;
    uint32_t pressed_ms = 0;
    bool long_triggered = false;  // ensure we toggle only once per press

    for (;;) {
        int level = gpio_get_level(KEY_GPIO);
        bool pressed = (level == 0);  // button is active-low

        if (pressed) {
            if (pressed_prev) {
                pressed_ms += SCAN_INTERVAL_MS;
            } else {
                pressed_ms = SCAN_INTERVAL_MS;
            }

            if (!long_triggered && pressed_ms >= LONG_PRESS_MS) {
                apply_power_state(!s_power_on);
                long_triggered = true;
            }
        } else {
            pressed_ms = 0;
            long_triggered = false;
        }

        pressed_prev = pressed;
        vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL_MS));
    }
}

esp_err_t app_power_start(void)
{
    if (s_power_task) {
        return ESP_OK;
    }

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << KEY_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,   // keep input high when no external pull-up is present
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io);
    if (ret != ESP_OK) {
        return ret;
    }

    apply_power_state(true);  // ensure GUI/power domain starts in ON state

    BaseType_t ok = xTaskCreate(key_task, "power_key", 2048, NULL, 5, &s_power_task);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}
