// components/ml/include/ml_window.h
#pragma once
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int   pred;     // 0=walk, 1=ebike
    float p_walk;
    float p_ebike;
} ml_result_t;

bool ml_window_init(void);

void ml_window_push_sample_raw(int ax, int ay, int az,
                               int gx, int gy, int gz,
                               bool has_gps,
                               float speed_mps,
                               float course_deg_now);

bool ml_get_latest_result(ml_result_t* out);

#ifdef __cplusplus
}
#endif
