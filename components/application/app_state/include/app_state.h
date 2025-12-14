#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "app_gps.h"

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
    int64_t timestamp_us;
} app_state_imu_sample_t;

void app_state_init(void);

void app_state_set_imu_sample(const app_state_imu_sample_t *sample);
bool app_state_get_latest_imu(app_state_imu_sample_t *out_sample);

void app_state_set_gps_data(const GNSS_Data *data);
bool app_state_get_latest_gps(GNSS_Data *out_data);

#ifdef __cplusplus
}
#endif

#endif /* APP_STATE_H */
