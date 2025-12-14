#ifndef APP_AXIS6_H
#define APP_AXIS6_H

#include "axis6_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

extern t_sQMI8658 qmi8658_info;

void app_axis6_start(void);

#define App_Axis6_Task_Start app_axis6_start

#ifdef __cplusplus
}
#endif

#endif /* APP_AXIS6_H */
