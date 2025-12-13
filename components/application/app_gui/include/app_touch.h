// components/application/app_gui/include/app_touch.h
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化触控
void app_touch_init(void);

// 读取触控坐标，返回 true 表示有按下
bool app_touch_read(int32_t *x, int32_t *y);

#ifdef __cplusplus
}
#endif