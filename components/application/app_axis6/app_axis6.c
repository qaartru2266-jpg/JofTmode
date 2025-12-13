#include "stdio.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "axis6_interface.h"
#include "app_axis6.h"
#include "app_sdcard.h"

static const char* TAG = "axis6";
static int warmup = 5;   // 跳过前5帧（约200ms），按需调，目前测试下来7帧是最好的

t_sQMI8658 qmi8658_info;

static void axis6_task(void* arg)
{
    i2c_master_init();
    qmi8658_init();

    TickType_t last_wake = xTaskGetTickCount();
    int hb = 0;

    ESP_LOGW(TAG, "axis6 task started");

    while (1) {
        // 阻塞读取IMU
        qmi8658_Read_AccAndGry(&qmi8658_info);

         if (warmup > 0) {
            warmup--;
        } else {
            app_sdcard_append_csv_row(NULL);   // 只在暖机后开始落盘
        }   

        // 直接写一行（若SD未就绪，内部return）
        app_sdcard_append_csv_row(NULL);

        if (++hb >= 25) {
            hb = 0;
            UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGW(TAG, "tick acc=(%d,%d,%d) gyr=(%d,%d,%d) stack_free=%u words",
                qmi8658_info.acc_x, qmi8658_info.acc_y, qmi8658_info.acc_z,
                qmi8658_info.gyr_x, qmi8658_info.gyr_y, qmi8658_info.gyr_z,
                (unsigned)free_words);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(40)); // 25Hz
    }
}

void app_axis6_start(void)
{
    xTaskCreate(axis6_task, "axis6", 8192, NULL, 10, NULL);
}
