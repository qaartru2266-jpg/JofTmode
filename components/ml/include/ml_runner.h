#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


bool ml_init(void);  
bool ml_infer(const float window_75x8[75][8],
              int* out_pred, float* out_p_walk, float* out_p_ebike);
              

#ifdef __cplusplus
}
#endif
