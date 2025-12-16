#include "app_gui.h"

#include "amoled_port.h"
#include "esp_check.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_qspi_amoled.h"
#include "esp_log.h"

static const char *TAG = "app_gui";

static bool s_gui_started;
static bool s_screen_on = true;

static void app_gui_apply_screen_state(bool on)
{
    esp_lcd_panel_handle_t panel = amoled_app_get_panel();
    if (!panel) {
        ESP_LOGW(TAG, "Panel handle is not ready");
        return;
    }

    if (s_screen_on == on) {
        return;
    }

    esp_err_t err = esp_lcd_panel_disp_on_off(panel, on);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set panel state (%s)", esp_err_to_name(err));
        return;
    }

    uint32_t brightness = on ? 0xFF : 0x00;
    err = panel_qspi_amoled_set_brightness(panel, brightness);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to update brightness (%s)", esp_err_to_name(err));
    }

    s_screen_on = on;
}

esp_err_t app_gui_start(void)
{
    if (s_gui_started) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(amoled_app_lcd_init(), TAG, "LCD init failed");
    ESP_RETURN_ON_ERROR(amoled_app_touch_init(), TAG, "Touch init failed");
    ESP_RETURN_ON_ERROR(amoled_app_lvgl_init(), TAG, "LVGL init failed");
    amoled_app_main_display();

    s_gui_started = true;
    s_screen_on = true;
    ESP_LOGI(TAG, "GUI initialized using AMOLED reference port");
    return ESP_OK;
}

void app_gui_screen_on(void)
{
    if (!s_gui_started) {
        ESP_LOGW(TAG, "Screen on requested before GUI init");
    }
    app_gui_apply_screen_state(true);
}

void app_gui_screen_off(void)
{
    if (!s_gui_started) {
        ESP_LOGW(TAG, "Screen off requested before GUI init");
    }
    app_gui_apply_screen_state(false);
}

bool app_gui_screen_is_on(void)
{
    return s_screen_on;
}
