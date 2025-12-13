// components/ml/ml_runner.cc —— 直接整文件替换

#include "esp_log.h"
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <inttypes.h>  // 修正 ESP_LOG* 的 PRI 宏

// 用可变解析器 + 手动注册需要的算子
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

// 由 tools/bin2cc.py 生成（model_data.cc）
extern const unsigned char g_model[];   // 只声明数组，不声明长度，避免链接问题

namespace {

constexpr const char* TAG = "ml_runner";

// ===== 输入形状（deploy_params.json）=====
constexpr int kT = 75;   // 时间长度（帧数）
constexpr int kC = 8;    // 通道数

// ===== 训练时的标准化参数（deploy_params.json）=====
// 顺序：acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, speed_mps, turn_rate_deg_s
static const float kMu[kC] = {
    1593.7335205078125f, 2013.6827392578125f,  832.3717041015625f,
    -737.0647583007812f,  296.4726257324219f,   44.95747756958008f,
       1.471951961517334f,  -0.6813814043998718f
};
static const float kSigma[kC] = {
    1080.1966552734375f,  2071.291259765625f,   7815.89013671875f,
    1498.8814697265625f,  1205.115234375f,      1906.49658203125f,
       2.258960008621216f,   44.726905822753906f
};

// 标签映射：{"walk":0, "ebike":1}
enum : int { LABEL_WALK = 0, LABEL_EBIKE = 1 };

// —— TFLM 对象 —— //
static tflite::MicroMutableOpResolver<8> s_resolver;  // 8 个足够
static const tflite::Model*      s_model        = nullptr;
static tflite::MicroInterpreter* s_interpreter  = nullptr;
static bool                      s_ready        = false;   // 分配成功后才置 true

// Arena：先给 160KB（不够可以再加）
alignas(16) static uint8_t s_tensor_arena[160 * 1024];

// 预计算：q = round(x * sA + sB)
static float sA[kC];
static float sB[kC];

// 输出量化参数（把 int8 还原为 float 概率）
static float s_out_scale = 0.0f;
static int   s_out_zero  = 0;

} // namespace

extern "C" bool ml_init(void)
/**
 * 初始化 TFLM，并根据输入量化参数 + 训练时 μ/σ，预计算每个通道的量化系数 sA/sB。
 */
{
    s_ready = false;

    s_model = tflite::GetModel(g_model);
    if (!s_model) {
        ESP_LOGE(TAG, "GetModel failed");
        return false;
    }
    if (s_model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model schema %" PRIu32 " != %d",
                 (uint32_t)s_model->version(), TFLITE_SCHEMA_VERSION);
        return false;
    }

    // 手动注册模型会用到的算子
    s_resolver.AddConv2D();         // Conv1D 常由 Conv2D(kx1) 表达
    s_resolver.AddFullyConnected(); // Dense
    s_resolver.AddReshape();        // 形状调整
    s_resolver.AddMean();           // GlobalAveragePooling1D -> ReduceMean
    s_resolver.AddSoftmax();        // 输出层
    s_resolver.AddExpandDims();     // 

    static tflite::MicroInterpreter static_interpreter(
        s_model, s_resolver, s_tensor_arena, sizeof(s_tensor_arena), /*error_reporter=*/nullptr);
    s_interpreter = &static_interpreter;

    if (s_interpreter->AllocateTensors() != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors failed (arena=%u bytes)", (unsigned)sizeof(s_tensor_arena));
        return false;
    }

    TfLiteTensor* in  = s_interpreter->input(0);
    TfLiteTensor* out = s_interpreter->output(0);
    if (!in || !out) {
        ESP_LOGE(TAG, "input/output tensor is null");
        return false;
    }

    // 读取输入/输出张量的量化参数
    const float in_scale      = in->params.scale;
    const int   in_zero_point = in->params.zero_point;
    s_out_scale               = out->params.scale;
    s_out_zero                = out->params.zero_point;

    // 预计算：q = round( x*(1/(σ*in_scale)) + (in_zero - μ/(σ*in_scale)) )
    for (int ch = 0; ch < kC; ++ch) {
        float inv = 1.0f / (kSigma[ch] * in_scale);
        sA[ch] = inv;
        sB[ch] = in_zero_point - kMu[ch] * inv;
    }

    s_ready = true;
    ESP_LOGI(TAG, "TFLM ready. in_scale=%.7f in_zp=%d  out_scale=%.7f out_zp=%d",
             in_scale, in_zero_point, s_out_scale, s_out_zero);
    return true;
}

extern "C" bool ml_infer(const float window_75x8[kT][kC],
                         int* out_pred, float* out_p_walk, float* out_p_ebike)
/**
 * 喂入一个窗口（75x8，单位与训练一致），同步推理并返回结果。
 * - out_pred: 0=walk, 1=ebike
 * - out_p_walk/out_p_ebike: 反量化“概率”（若输出层是 softmax，则两者近似相加为 1）
 */
{
    // 双重保护
    if (!s_ready || !s_interpreter) return false;
    TfLiteTensor* in  = s_interpreter->input(0);
    TfLiteTensor* out = s_interpreter->output(0);
    if (!in || !out) return false;

    // 1) 量化填充输入张量（int8）
    int8_t* dst = in->data.int8;
    for (int t = 0; t < kT; ++t) {
        for (int ch = 0; ch < kC; ++ch) {
            float x = window_75x8[t][ch];
            int   q = lroundf(x * sA[ch] + sB[ch]);
            if (q < -128) q = -128;
            if (q > 127)  q = 127;
            *dst++ = (int8_t)q;
        }
    }

    // 2) 推理
    if (s_interpreter->Invoke() != kTfLiteOk) {
        ESP_LOGE(TAG, "Invoke failed");
        return false;
    }

    // 3) 读取输出
    const int8_t* o = out->data.int8;   // 2 维：0=walk, 1=ebike
    int i_walk  = o[LABEL_WALK];
    int i_ebike = o[LABEL_EBIKE];

    if (out_pred) {
        *out_pred = (i_ebike > i_walk) ? LABEL_EBIKE : LABEL_WALK;
    }
    if (out_p_walk)  *out_p_walk  = (i_walk  - s_out_zero) * s_out_scale;
    if (out_p_ebike) *out_p_ebike = (i_ebike - s_out_zero) * s_out_scale;

    return true;
}
