#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <sys/unistd.h>   // fsync(), fileno()

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"

#include "app_gps.h"
#include "app_axis6.h"
#include "ml_window.h"
#include "app_sdcard.h"

// ===== 硬件与策略 =====
#define MOUNT_POINT         "/sdcard"
#define SDCARD_SPI_HOST     SPI2_HOST
#define SDCARD_PIN_MOSI     GPIO_NUM_17
#define SDCARD_PIN_MISO     GPIO_NUM_15
#define SDCARD_PIN_SCLK     GPIO_NUM_16
#define SDCARD_PIN_CS       GPIO_NUM_18
#define SDCARD_BOOT_KHZ     400       // 先 400kHz，稳定再升

//  刷盘/同步策略：调试期先 aggressive，确认PC可见
#define FLUSH_EVERY_LINES   25        // 每 25 行 fflush（≈1秒一次）
#define FSYNC_EVERY_FLUSH   1         // 每次 fflush 后都 fsync；确认OK后可改成 4 或 10 降低写放大

static const char* TAG = "app_sdcard";

// 状态
static bool          s_bus_ok   = false;
static bool          s_mounted  = false;
static sdmmc_card_t* s_card     = NULL;

static FILE*  s_csv = NULL;
static char   s_csv_path[64] = {0};
static bool   s_ready = false;

// 最近一次 ML 结果快照（给 GUI 读，不消耗）
static ml_result_t s_last_ml;
static volatile bool s_last_ml_valid = false;

// 最近一次有效GPS（用于沿用）
static bool   s_have_last_gps = false;
static double s_last_lat=0, s_last_lon=0;
static float  s_last_spd=0, s_last_course=0;
static char   s_last_date[12] = {0};   // "DDMMYY"
static char   s_last_time[16] = {0};   // "HHMMSS.sss"

// 写行计数（用于定期刷盘/同步）
static uint32_t s_lines_since_flush = 0;
static uint32_t s_flush_since_sync  = 0;

// SPI host/bus
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

// ---------- 小工具：立刻把目录项和数据刷到卡上 ----------
static inline void csv_sync_now(void)
{
    if (!s_csv) return;
    fflush(s_csv);                   // 刷 stdio 缓冲
    (void)fsync(fileno(s_csv));      // 刷到 FATFS/卡（关键：更新文件大小）
}

static esp_err_t sdcard_init_mount_once(void)
{
    if (!s_bus_ok) {
        s_host.slot = SDCARD_SPI_HOST;
        s_host.max_freq_khz = SDCARD_BOOT_KHZ;

        // 片选拉高 + 上拉，抗抖
        gpio_config_t io = {
            .pin_bit_mask = (1ULL<<SDCARD_PIN_MOSI) | (1ULL<<SDCARD_PIN_MISO) | (1ULL<<SDCARD_PIN_CS),
            .mode         = GPIO_MODE_INPUT_OUTPUT,
            .pull_up_en   = 1,
            .pull_down_en = 0,
            .intr_type    = GPIO_INTR_DISABLE
        };
        gpio_config(&io);
        gpio_set_level(SDCARD_PIN_CS, 1);

        esp_err_t err = spi_bus_initialize(s_host.slot, &s_buscfg, SPI_DMA_CH_AUTO);
        if (err == ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "SPI bus already inited, reuse");
        } else if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
            return err;
        }
        s_bus_ok = true;
        ESP_LOGW(TAG, "SPI ready @ %dkHz (MOSI=%d MISO=%d SCLK=%d CS=%d)",
                 s_host.max_freq_khz, SDCARD_PIN_MOSI, SDCARD_PIN_MISO, SDCARD_PIN_SCLK, SDCARD_PIN_CS);
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

static void make_unique_csv_path(char* out, size_t outsz)
{
    for (int i = 1; i <= 9999; ++i) {
        snprintf(out, outsz, MOUNT_POINT "/log_%04d.csv", i);
        FILE* f = fopen(out, "r");
        if (!f) return;  // 不存在
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

    // 无缓冲 + 立即同步表头，避免“空壳文件”
    setvbuf(s_csv, NULL, _IONBF, 0);

    const char* header =
        "date,timestamp,timestamp_ms,latitude,longitude,speed_mps,course_deg,"
        "acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,"
        "ml_pred,ml_p_walk,ml_p_ebike\r\n";

    int nw = fprintf(s_csv, "%s", header);
    if (nw <= 0) {
        int e = errno;
        ESP_LOGE(TAG, "write header failed: errno=%d (%s)", e, strerror(e));
        fclose(s_csv); s_csv = NULL;
        return ESP_FAIL;
    }

    csv_sync_now();                  // ★★ 关键：同步目录项，PC 立刻能看到表头
    s_lines_since_flush = 0;
    s_flush_since_sync  = 0;

    s_ready = true;
    ESP_LOGW(TAG, "CSV header OK & SYNCED");
    return ESP_OK;
}

// ===== 对外接口 =====

void app_sdcard_start(void)
{
    static bool done = false;
    if (done) return;
    done = true;

    ESP_LOGW(TAG, "init...");
    if (sdcard_init_mount_once() != ESP_OK) return;
    (void)csv_open_create_header();  // 失败会打印错误并保持 s_ready=false
}

bool app_sdcard_is_ready(void)
{
    return s_ready && (s_csv != NULL);
}

void app_sdcard_append_csv_row(const char* unused)
{
    if (!(s_ready && s_csv)) return;

    // 时间戳（毫秒）
    long long ts_ms = (long long)(esp_timer_get_time() / 1000);

    // IMU
    int ax = qmi8658_info.acc_x;
    int ay = qmi8658_info.acc_y;
    int az = qmi8658_info.acc_z;
    int gx = qmi8658_info.gyr_x;
    int gy = qmi8658_info.gyr_y;
    int gz = qmi8658_info.gyr_z;

    // GPS：沿用最近有效
    const char* date_str = "";
    const char* time_str = "";
    double lat = 0.0, lon = 0.0;
    float  spd = 0.0f, crs = 0.0f;
    bool   use_gps = false;

    extern GNSS_Data current_data; // 在 app_gps.c
    if (current_data.is_valid == 1) {
        s_have_last_gps = true;
        s_last_lat    = current_data.latitude;
        s_last_lon    = current_data.longitude;
        s_last_spd    = current_data.speed;
        s_last_course = current_data.course;

        strncpy(s_last_date, current_data.date, sizeof(s_last_date)-1);
        s_last_date[sizeof(s_last_date)-1] = '\0';
        strncpy(s_last_time, current_data.timestamp, sizeof(s_last_time)-1);
        s_last_time[sizeof(s_last_time)-1] = '\0';

        date_str = s_last_date; time_str = s_last_time;
        lat = s_last_lat; lon = s_last_lon; spd = s_last_spd; crs = s_last_course;
        use_gps = true;
    } else if (s_have_last_gps) {
        date_str = s_last_date; time_str = s_last_time;
        lat = s_last_lat; lon = s_last_lon; spd = s_last_spd; crs = s_last_course;
        use_gps = true;
    }

    // 推到 ML 窗口（75 帧一判）
    ml_window_push_sample_raw(ax, ay, az, gx, gy, gz, use_gps, spd, crs);

    // 取最新 ML 结果（可能尚未产生）
    ml_result_t r;
    bool have_ml = ml_get_latest_result(&r);

    // 写一行
    if (use_gps) {
        fprintf(s_csv, "%s,%s,%lld,%.6lf,%.6lf,%.6f,%.6f,",
                date_str, time_str, ts_ms, lat, lon, spd, crs);
    } else {
        fprintf(s_csv, "%s,%s,%lld,,,,,", date_str, time_str, ts_ms);
    }
    fprintf(s_csv, "%d,%d,%d,%d,%d,%d",
            ax, ay, az, gx, gy, gz);

    if (have_ml) {
        const char* label = (r.pred == 0) ? "walk" : "ebike";  // 按你的训练定义：0=walk, 1=ebike
        fprintf(s_csv, ",%s,%.3f,%.3f\r\n", label, r.p_walk, r.p_ebike);
    } else {
        fprintf(s_csv, ",,,\r\n");
    }


    // ---- 定期刷盘/同步，保证PC看到数据增长 ----
    if (++s_lines_since_flush >= FLUSH_EVERY_LINES) {
        s_lines_since_flush = 0;
        fflush(s_csv);
        if (++s_flush_since_sync >= FSYNC_EVERY_FLUSH) {
            s_flush_since_sync = 0;
            (void)fsync(fileno(s_csv));
        }
    }

    if (have_ml) {
    s_last_ml = r;
    s_last_ml_valid = true;
}

}

bool app_ml_get_latest(ml_result_t *out)
{
    if (!out) return false;
    if (!s_last_ml_valid) return false;
    *out = s_last_ml;   // 非消耗型拷贝
    return true;
}
