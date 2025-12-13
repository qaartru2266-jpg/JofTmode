#pragma once
#ifdef __cplusplus
extern "C" {
#endif

// 同步初始化：挂载SD、创建CSV并写表头（只需调用一次）
void app_sdcard_start(void);

// 由IMU任务每40ms调用，直接写一行
void app_sdcard_append_csv_row(const char* unused);

#include "ml_window.h"    // 需要 ml_result_t

// 非消耗型：取最近一次 ML 结果（若有）
bool app_ml_get_latest(ml_result_t *out);

// 查询CSV是否已就绪（打开且表头已写）
bool app_sdcard_is_ready(void);

#ifdef __cplusplus
}
#endif
