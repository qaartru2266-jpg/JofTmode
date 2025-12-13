#ifndef __APP_AXIS6_H__
#define __APP_AXIS6_H__

#include "axis6_interface.h"  // 提供 t_sQMI8658 类型

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyr_x;
    int16_t gyr_y;
    int16_t gyr_z;
} App_Axis6_Info_S;

// 全局 IMU 原始数据（在 .c 里定义，这里声明给其它模块用）
extern t_sQMI8658 qmi8658_info;

// 提供一个便捷函数拿 IMU 原始值（如果别处需要）
App_Axis6_Info_S App_Get_Axis6_Info(void);

// 启动/停止 IMU 任务
void app_axis6_start(void);
void app_axis6_stop(void);

#define App_Axis6_Task_Start app_axis6_start

#ifdef __cplusplus
}
#endif

#endif // __APP_AXIS6_H__
