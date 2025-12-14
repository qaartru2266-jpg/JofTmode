#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "app_state.h"

static SemaphoreHandle_t s_state_lock;
static StaticSemaphore_t s_state_lock_buffer;
static app_state_imu_sample_t s_latest_imu;
static bool s_have_imu = false;
static GNSS_Data s_latest_gps;
static bool s_have_gps = false;

void app_state_init(void)
{
    if (s_state_lock == NULL) {
        s_state_lock = xSemaphoreCreateMutexStatic(&s_state_lock_buffer);
    }
}

void app_state_set_imu_sample(const app_state_imu_sample_t *sample)
{
    if (!sample) {
        return;
    }
    if (s_state_lock == NULL) {
        app_state_init();
    }

    if (s_state_lock) {
        xSemaphoreTake(s_state_lock, portMAX_DELAY);
        s_latest_imu = *sample;
        s_have_imu = true;
        xSemaphoreGive(s_state_lock);
    }
}

bool app_state_get_latest_imu(app_state_imu_sample_t *out_sample)
{
    if (!out_sample) {
        return false;
    }
    if (s_state_lock == NULL) {
        app_state_init();
    }

    bool have_sample = false;
    if (s_state_lock) {
        xSemaphoreTake(s_state_lock, portMAX_DELAY);
        if (s_have_imu) {
            *out_sample = s_latest_imu;
            have_sample = true;
        }
        xSemaphoreGive(s_state_lock);
    }
    return have_sample;
}

void app_state_set_gps_data(const GNSS_Data *data)
{
    if (!data) {
        return;
    }
    if (s_state_lock == NULL) {
        app_state_init();
    }
    if (s_state_lock) {
        xSemaphoreTake(s_state_lock, portMAX_DELAY);
        s_latest_gps = *data;
        s_have_gps = true;
        xSemaphoreGive(s_state_lock);
    }
}

bool app_state_get_latest_gps(GNSS_Data *out_data)
{
    if (!out_data) {
        return false;
    }
    if (s_state_lock == NULL) {
        app_state_init();
    }
    bool have_data = false;
    if (s_state_lock) {
        xSemaphoreTake(s_state_lock, portMAX_DELAY);
        if (s_have_gps) {
            *out_data = s_latest_gps;
            have_data = true;
        }
        xSemaphoreGive(s_state_lock);
    }
    return have_data;
}
