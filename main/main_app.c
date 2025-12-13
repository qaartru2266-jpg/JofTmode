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

void app_main(void)
{
    // 1) 先把SD准备好（同步完成：挂载/建CSV+表头）
    app_sdcard_start();                 // 必须在IMU前面
    vTaskDelay(pdMS_TO_TICKS(200));

    // 2) 初始化ML窗口
    ml_window_init();

    // 2.5) 初始化震动模块，交由 app_power 控制
    app_vibration_init();

    // 3) 再启动IMU/GPS/屏幕
    app_axis6_start();
    app_gps_start();
    app_gui_start();

    // 4) 启动长按电源任务（控制屏幕+震动）
    app_power_start();

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}