#pragma once
#include <stdbool.h>
#include "esp_err.h"

esp_err_t app_gui_start(void);

// 屏幕显隐控制（用于长按逻辑）
void app_gui_screen_on(void);
void app_gui_screen_off(void);
bool app_gui_screen_is_on(void);
