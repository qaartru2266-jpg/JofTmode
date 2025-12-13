#include "app_vibration.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define VIB_GPIO          GPIO_NUM_3
#define VIB_ACTIVE_LEVEL  1  // 高电平导通；如发现反相可改为 0

static TaskHandle_t vib_task = NULL;
static vib_pattern_t cur_pat = {200, 800};  // 默认模式：0.2s on / 0.8s off

static void vib_on(void)  { gpio_set_level(VIB_GPIO, VIB_ACTIVE_LEVEL); }
static void vib_off(void) { gpio_set_level(VIB_GPIO, !VIB_ACTIVE_LEVEL); }

static bool vib_task_running(void) { return vib_task != NULL; }

static void vib_task_fn(void *arg)
{
    for (;;) {
        vib_on();
        vTaskDelay(pdMS_TO_TICKS(cur_pat.on_ms));
        vib_off();
        vTaskDelay(pdMS_TO_TICKS(cur_pat.off_ms));
    }
}

esp_err_t app_vibration_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << VIB_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io);
    if (ret != ESP_OK) return ret;
    vib_off();  // 默认关闭
    return ESP_OK;
}

esp_err_t app_vibration_start(vib_pattern_t pat)
{
    cur_pat = pat;
    if (vib_task) return ESP_OK;  // 已在运行

    BaseType_t ok = xTaskCreate(vib_task_fn, "vib_task", 2048, NULL, 5, &vib_task);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

void app_vibration_stop(void)
{
    if (vib_task) {
        vTaskDelete(vib_task);
        vib_task = NULL;
    }
    vib_off();
}

void app_vibration_pulse_ms(uint32_t on_ms)
{
    // 仅在未运行周期任务时提供一次性脉冲
    if (vib_task_running()) return;
    vib_on();
    vTaskDelay(pdMS_TO_TICKS(on_ms));
    vib_off();
}
