#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_ops.h"

esp_err_t amoled_app_lcd_init(void);
esp_err_t amoled_app_touch_init(void);
esp_err_t amoled_app_lvgl_init(void);
void amoled_app_main_display(void);

esp_lcd_panel_handle_t amoled_app_get_panel(void);
