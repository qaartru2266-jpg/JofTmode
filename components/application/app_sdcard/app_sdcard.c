#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "app_state.h"
#include "app_sdcard.h"
#include "ml_window.h"

#define MOUNT_POINT         "/sdcard"
#define SDCARD_SPI_HOST     SPI3_HOST
#define SDCARD_PIN_MOSI     GPIO_NUM_17
#define SDCARD_PIN_MISO     GPIO_NUM_15
#define SDCARD_PIN_SCLK     GPIO_NUM_16
#define SDCARD_PIN_CS       GPIO_NUM_18
#define SDCARD_BOOT_KHZ     400
#define FLUSH_EVERY_LINES   25
#define FSYNC_EVERY_FLUSH   1
#define LOGGER_INTERVAL_MS  40

static const char *TAG = "app_sdcard";

static bool s_bus_ok = false;
static bool s_mounted = false;
static sdmmc_card_t *s_card = NULL;
static FILE *s_csv = NULL;
static char s_csv_path[64] = {0};
static bool s_ready = false;

static TaskHandle_t s_logger_task = NULL;
static int64_t s_last_logged_imu_ts = 0;

static uint32_t s_lines_since_flush = 0;
static uint32_t s_flush_since_sync = 0;

static ml_result_t s_last_ml;
static volatile bool s_last_ml_valid = false;

static bool s_have_last_gps_snapshot = false;
static double s_last_lat = 0.0;
static double s_last_lon = 0.0;
static float s_last_spd = 0.0f;
static float s_last_course = 0.0f;
static char s_last_date[12] = {0};
static char s_last_time[16] = {0};

static sdmmc_host_t s_host = SDSPI_HOST_DEFAULT();
static spi_bus_config_t s_buscfg = {
    .mosi_io_num = SDCARD_PIN_MOSI,
    .miso_io_num = SDCARD_PIN_MISO,
    .sclk_io_num = SDCARD_PIN_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 8192,
};
static sdspi_device_config_t s_slotcfg;
static esp_vfs_fat_sdmmc_mount_config_t s_mountcfg = {
    .format_if_mount_failed = false,
    .max_files = 5,
    .allocation_unit_size = 0
};

static inline void csv_sync_now(void)
{
    if (!s_csv) {
        return;
    }
    fflush(s_csv);
    (void)fsync(fileno(s_csv));
}

static esp_err_t sdcard_init_mount_once(void)
{
    if (!s_bus_ok) {
        s_host.slot = SDCARD_SPI_HOST;
        s_host.max_freq_khz = SDCARD_BOOT_KHZ;

        gpio_config_t io = {
            .pin_bit_mask = (1ULL << SDCARD_PIN_MOSI) | (1ULL << SDCARD_PIN_MISO) | (1ULL << SDCARD_PIN_CS),
            .mode = GPIO_MODE_INPUT_OUTPUT,
            .pull_up_en = 1,
            .pull_down_en = 0,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io);
        gpio_set_level(SDCARD_PIN_CS, 1);

        esp_err_t err = spi_bus_initialize(s_host.slot, &s_buscfg, SPI_DMA_CH_AUTO);
        if (err == ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "SPI bus already initialized, reuse current bus");
        } else if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
            return err;
        }
        s_bus_ok = true;
    }

    if (!s_mounted) {
        s_slotcfg = (sdspi_device_config_t)SDSPI_DEVICE_CONFIG_DEFAULT();
        s_slotcfg.gpio_cs = SDCARD_PIN_CS;
        s_slotcfg.host_id = s_host.slot;

        esp_err_t e = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &s_host, &s_slotcfg, &s_mountcfg, &s_card);
        if (e == ESP_OK || e == ESP_ERR_INVALID_STATE) {
            s_mounted = true;
            ESP_LOGW(TAG, "SD mounted at %s", MOUNT_POINT);
        } else {
            ESP_LOGE(TAG, "mount failed: %s", esp_err_to_name(e));
            return e;
        }
    }
    return ESP_OK;
}

static void make_unique_csv_path(char *out, size_t outsz)
{
    for (int i = 1; i <= 9999; ++i) {
        snprintf(out, outsz, MOUNT_POINT "/log_%04d.csv", i);
        FILE *f = fopen(out, "r");
        if (!f) {
            return;
        }
        fclose(f);
    }
    snprintf(out, outsz, MOUNT_POINT "/log_overflow.csv");
}

static esp_err_t csv_open_create_header(void)
{
    if (!s_mounted) {
        ESP_LOGE(TAG, "SD not mounted");
        return ESP_FAIL;
    }
    make_unique_csv_path(s_csv_path, sizeof(s_csv_path));
    ESP_LOGW(TAG, "Create CSV: %s", s_csv_path);

    s_csv = fopen(s_csv_path, "w");
    if (!s_csv) {
        int e = errno;
        ESP_LOGE(TAG, "fopen failed: errno=%d (%s)", e, strerror(e));
        return ESP_FAIL;
    }

    setvbuf(s_csv, NULL, _IONBF, 0);

    const char *header =
        "date,timestamp,timestamp_ms,latitude,longitude,speed_mps,course_deg,"
        "acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,"
        "ml_pred,ml_p_walk,ml_p_ebike\r\n";

    if (fprintf(s_csv, "%s", header) <= 0) {
        int e = errno;
        ESP_LOGE(TAG, "write header failed: errno=%d (%s)", e, strerror(e));
        fclose(s_csv);
        s_csv = NULL;
        return ESP_FAIL;
    }

    csv_sync_now();
    s_lines_since_flush = 0;
    s_flush_since_sync = 0;
    s_ready = true;
    ESP_LOGW(TAG, "CSV header OK & SYNCED");
    return ESP_OK;
}

static void copy_token(char *dst, size_t dst_sz, const char *src)
{
    if (!dst || dst_sz == 0) {
        return;
    }
    if (!src) {
        dst[0] = '\0';
        return;
    }
    strncpy(dst, src, dst_sz - 1);
    dst[dst_sz - 1] = '\0';
}

static void update_cached_gps(const GNSS_Data *data)
{
    if (!data || data->is_valid == 0) {
        return;
    }
    s_last_lat = data->latitude;
    s_last_lon = data->longitude;
    s_last_spd = data->speed;
    s_last_course = data->course;
    copy_token(s_last_date, sizeof(s_last_date), data->date);
    copy_token(s_last_time, sizeof(s_last_time), data->timestamp);
    s_have_last_gps_snapshot = true;
}

static void append_csv_row(const app_state_imu_sample_t *imu_sample, bool use_gps)
{
    if (!(s_ready && s_csv) || !imu_sample) {
        return;
    }

    long long ts_ms = imu_sample->timestamp_us / 1000;

    const char *date_str = "";
    const char *time_str = "";
    double lat = 0.0;
    double lon = 0.0;
    float spd = 0.0f;
    float crs = 0.0f;

    if (use_gps && s_have_last_gps_snapshot) {
        date_str = s_last_date;
        time_str = s_last_time;
        lat = s_last_lat;
        lon = s_last_lon;
        spd = s_last_spd;
        crs = s_last_course;
    }

    if (use_gps && s_have_last_gps_snapshot) {
        fprintf(s_csv, "%s,%s,%lld,%.6lf,%.6lf,%.6f,%.6f,",
                date_str, time_str, ts_ms, lat, lon, spd, crs);
    } else {
        fprintf(s_csv, "%s,%s,%lld,,,,,", date_str, time_str, ts_ms);
    }

    fprintf(s_csv, "%d,%d,%d,%d,%d,%d",
            imu_sample->acc_x, imu_sample->acc_y, imu_sample->acc_z,
            imu_sample->gyr_x, imu_sample->gyr_y, imu_sample->gyr_z);

    ml_result_t r;
    bool have_ml = ml_get_latest_result(&r);
    if (have_ml) {
        const char *label = (r.pred == 0) ? "walk" : "ebike";
        fprintf(s_csv, ",%s,%.3f,%.3f\r\n", label, r.p_walk, r.p_ebike);
        s_last_ml = r;
        s_last_ml_valid = true;
    } else {
        fprintf(s_csv, ",,,\r\n");
    }

    if (++s_lines_since_flush >= FLUSH_EVERY_LINES) {
        s_lines_since_flush = 0;
        fflush(s_csv);
        if (++s_flush_since_sync >= FSYNC_EVERY_FLUSH) {
            s_flush_since_sync = 0;
            (void)fsync(fileno(s_csv));
        }
    }
}

static void logger_step(void)
{
    if (!(s_ready && s_csv)) {
        return;
    }

    app_state_imu_sample_t imu;
    if (!app_state_get_latest_imu(&imu)) {
        return;
    }
    if (imu.timestamp_us == s_last_logged_imu_ts) {
        return;
    }
    s_last_logged_imu_ts = imu.timestamp_us;

    GNSS_Data gps_snapshot = {0};
    bool have_gps = app_state_get_latest_gps(&gps_snapshot);
    if (have_gps) {
        update_cached_gps(&gps_snapshot);
    }

    float speed = have_gps ? gps_snapshot.speed : s_last_spd;
    float course = have_gps ? gps_snapshot.course : s_last_course;
    bool gps_valid = have_gps ? (gps_snapshot.is_valid == 1) : s_have_last_gps_snapshot;

    ml_window_push_sample_raw(
        imu.acc_x, imu.acc_y, imu.acc_z,
        imu.gyr_x, imu.gyr_y, imu.gyr_z,
        gps_valid, speed, course
    );

    append_csv_row(&imu, gps_valid);
}

static void sdcard_logger_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        logger_step();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOGGER_INTERVAL_MS));
    }
}

void app_sdcard_start(void)
{
    static bool done = false;
    if (done) {
        return;
    }
    done = true;

    ESP_LOGW(TAG, "init...");
    if (sdcard_init_mount_once() != ESP_OK) {
        return;
    }
    if (csv_open_create_header() != ESP_OK) {
        return;
    }

    if (s_logger_task == NULL) {
        xTaskCreate(sdcard_logger_task, "sd_logger", 4096, NULL, 8, &s_logger_task);
    }
}

bool app_sdcard_is_ready(void)
{
    return s_ready && (s_csv != NULL);
}

bool app_ml_get_latest(ml_result_t *out)
{
    if (!out || !s_last_ml_valid) {
        return false;
    }
    *out = s_last_ml;
    return true;
}
