#ifndef APP_SDCARD_H
#define APP_SDCARD_H

#include "ml_window.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_sdcard_start(void);
bool app_sdcard_is_ready(void);
bool app_ml_get_latest(ml_result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_SDCARD_H */
