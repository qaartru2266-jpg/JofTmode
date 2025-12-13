#pragma once

#include "esp_err.h"
#include <stdint.h>

typedef struct {
    uint32_t on_ms;   // 持续震动时长
    uint32_t off_ms;  // 间歇时间
} vib_pattern_t;

esp_err_t app_vibration_init(void);                // 配置 GPIO
esp_err_t app_vibration_start(vib_pattern_t pat);  // 启动或更新间歇模式
void app_vibration_stop(void);                     // 停止并拉低输出
void app_vibration_pulse_ms(uint32_t on_ms);       // 简单脉冲，便于提示
