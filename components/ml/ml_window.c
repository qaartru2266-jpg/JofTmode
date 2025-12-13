// components/ml/ml_window.c  —— 纯 C，实现外部 API（供 C 代码调用）
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "esp_timer.h"

#include "ml_window.h"

// 与模型一致
#define K_T 75   // 时间长度
#define K_C 8    // 通道数：acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,speed_mps,turn_rate_deg_s

// 由 ml_runner.cc 暴露的 C 接口（注意：ml_runner.cc 里已用了 extern "C"）
bool ml_init(void);
bool ml_infer(const float window_75x8[K_T][K_C],
              int* out_pred, float* out_p_walk, float* out_p_ebike);

// ------------------- 窗口缓冲（环形） -------------------
static float s_ring[K_T][K_C];
static int   s_wr = 0;         // 写指针
static int   s_count = 0;      // 累计已写帧数（<= K_T）

// GPS 衍生量计算（转向角速度）
static bool   s_have_prev_gps = false;
static float  s_prev_course = 0.0f;
static int64_t s_prev_gps_time_us = 0;
static float  s_last_speed = 0.0f;

// 最近一次推理结果
static volatile bool        s_has_result = false;
static volatile ml_result_t s_last_res = {0};

// 角差归一化到 [-180, 180]
static inline float wrap_deg(float d)
{
    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// 取最近 75 帧，按时间从旧到新，拷贝到 out_win
static void snapshot_window(float out_win[K_T][K_C])
{
    // 若不足 75 帧，也照样补“已有的”帧；ml_infer 仍然可以跑（模型已量化），
    // 但建议尽量保证 s_count>=K_T 再信任结果。
    int n = (s_count < K_T) ? s_count : K_T;
    int start = (s_wr - n + K_T) % K_T;

    // 先清零（防止不足 75 帧时使用到垃圾数据）
    for (int t = 0; t < K_T; ++t) {
        for (int c = 0; c < K_C; ++c) {
            out_win[t][c] = 0.0f;
        }
    }

    for (int i = 0; i < n; ++i) {
        int src = (start + i) % K_T;
        for (int c = 0; c < K_C; ++c) {
            out_win[i][c] = s_ring[src][c];
        }
    }
}

bool ml_window_init(void)
{
    memset((void*)s_ring, 0, sizeof(s_ring));
    s_wr = 0;
    s_count = 0;
    s_have_prev_gps = false;
    s_prev_course = 0.0f;
    s_prev_gps_time_us = 0;
    s_last_speed = 0.0f;
    s_has_result = false;
    s_last_res.pred = 0;
    s_last_res.p_walk = 0.0f;
    s_last_res.p_ebike = 0.0f;

    return ml_init();  // 调用 TFLM 初始化（来自 ml_runner.cc）
}

void ml_window_push_sample_raw(int ax, int ay, int az,
                               int gx, int gy, int gz,
                               bool has_gps,
                               float speed_mps,
                               float course_deg_now)
{
    // 1) 计算 turn_rate_deg_s （由 course 导数得到）
    float turn_rate = 0.0f;
    int64_t now_us = esp_timer_get_time();

    if (has_gps) {
        // 更新“最后速度”
        s_last_speed = speed_mps;

        if (s_have_prev_gps) {
            float dcourse = wrap_deg(course_deg_now - s_prev_course);
            float dt_s = (float)(now_us - s_prev_gps_time_us) / 1000000.0f;
            if (dt_s > 0.0f) {
                turn_rate = dcourse / dt_s;  // deg/s
            }
        }
        s_have_prev_gps = true;
        s_prev_course = course_deg_now;
        s_prev_gps_time_us = now_us;
    } else {
        // 没有 GPS，就复用最后速度，转向角速度置 0
        speed_mps = s_last_speed;
        turn_rate = 0.0f;
    }

    // 2) 组 1 帧（顺序必须与训练一致）
    float frame[K_C];
    frame[0] = (float)ax;
    frame[1] = (float)ay;
    frame[2] = (float)az;
    frame[3] = (float)gx;
    frame[4] = (float)gy;
    frame[5] = (float)gz;
    frame[6] = speed_mps;
    frame[7] = turn_rate;

    // 3) 写入环形缓冲
    for (int c = 0; c < K_C; ++c) s_ring[s_wr][c] = frame[c];
    s_wr = (s_wr + 1) % K_T;
    if (s_count < K_T) s_count++;

    // 4) 满 75 帧就做一次推理（你现在是 25 Hz，每秒会推 25 次，窗口滑动一步推一次）
    if (s_count >= K_T) {
        float win[K_T][K_C];
        snapshot_window(win);

        int pred = 0;
        float pw = 0.0f, pe = 0.0f;
        if (ml_infer(win, &pred, &pw, &pe)) {
            ml_result_t r = { pred, pw, pe };
            s_last_res = r;
            s_has_result = true;
        }
    }
}

bool ml_get_latest_result(ml_result_t* out)
{
    if (!s_has_result || !out) return false;
    *out = s_last_res;
    return true;
}
