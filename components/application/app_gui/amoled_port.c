#include "amoled_port.h"

#include <inttypes.h>
#include <sys/lock.h>
#include <sys/param.h>

#include "amoled_config.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_chsc6417.h"
#include "esp_lcd_qspi_amoled.h"
#include "wallpaper_image.h"
#include "lv_demos.h"

static const char *TAG = "app_gui";

static esp_lcd_panel_handle_t s_panel_handle;
static esp_lcd_panel_io_handle_t s_panel_io;
static bool s_lcd_initialized;

#if AMOLED_USE_TOUCH
static esp_lcd_touch_handle_t s_touch_handle;
static bool s_touch_initialized;
#endif

static lv_display_t *s_lvgl_display;
static bool s_lvgl_initialized;
static TaskHandle_t s_lvgl_task;
static esp_timer_handle_t s_lvgl_tick_timer;
static _lock_t s_lvgl_api_lock;
static lv_obj_t *s_wallpaper_image;
static lv_obj_t *s_clock_label;
static lv_timer_t *s_clock_timer;

static const qspi_amoled_lcd_init_cmd_t s_lcd_init_cmds[] = {
    {0xFE, (uint8_t []){0x00}, 1, 0},
    {0xF0, (uint8_t []){0x50}, 1, 0},
    {0xB1, (uint8_t []){0x78,0x70}, 2, 0},
    {0xC4, (uint8_t []){0x80}, 1, 0},
    {0x35, (uint8_t []){0x00}, 1, 0},
    {0x36, (uint8_t []){0x00}, 1, 0},
#if LV_COLOR_DEPTH == 16
    {0x3A, (uint8_t []){0x55}, 1, 0},
#elif LV_COLOR_DEPTH == 24
    {0x3A, (uint8_t []){0x77}, 1, 0},
#endif
    {0x53, (uint8_t []){0x20}, 1, 0},
    {0x51, (uint8_t []){0xFF}, 1, 0},
    {0x63, (uint8_t []){0xFF}, 1, 0},
    {0x64, (uint8_t []){0x10}, 1, 0},
    {0x67, (uint8_t []){0x01}, 1, 0},
    {0x68, (uint8_t []){0x31}, 1, 0},
    {0x2A, (uint8_t []){0x00,0x00,0x01,0xdf}, 4, 0},
    {0x2B, (uint8_t []){0x00,0x00,0x01,0xdf}, 4, 0},
    {0x11, (uint8_t []){0x00}, 0, 120},
    {0x29, (uint8_t []){0x00}, 0, 120},
};

static bool amoled_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

static void amoled_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    const size_t pixels = (size_t)(offsetx2 + 1 - offsetx1) * (size_t)(offsety2 + 1 - offsety1);

#if LV_COLOR_DEPTH == 16
    lv_draw_sw_rgb565_swap(px_map, pixels);
#elif LV_COLOR_DEPTH == 24
    for (size_t i = 0; i < pixels; ++i) {
        uint8_t *px = px_map + i * 3;
        uint8_t tmp = px[0];
        px[0] = px[2];
        px[2] = tmp;
    }
#endif

    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
}

static void amoled_lvgl_rounder_cb(lv_event_t *e)
{
    lv_area_t *area = lv_event_get_param(e);
    uint16_t x1 = area->x1;
    uint16_t x2 = area->x2;
    uint16_t y1 = area->y1;
    uint16_t y2 = area->y2;

    area->x1 = (x1 >> 1) << 1;
    area->y1 = (y1 >> 1) << 1;
    area->x2 = ((x2 >> 1) << 1) + 1;
    area->y2 = ((y2 >> 1) << 1) + 1;
}

#if AMOLED_USE_TOUCH
static void amoled_lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)lv_indev_get_user_data(indev);
    if (!tp) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    esp_lcd_touch_read_data(tp);
    esp_lcd_touch_point_data_t points[CONFIG_ESP_LCD_TOUCH_MAX_POINTS] = {0};
    uint8_t point_count = 0;
    esp_err_t ret = esp_lcd_touch_get_data(tp, points, &point_count, 1);
    if (ret == ESP_OK && point_count > 0) {
        data->point.x = points[0].x;
        data->point.y = points[0].y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
#endif

static void amoled_increase_lvgl_tick(void *arg)
{
    lv_tick_inc(AMOLED_LVGL_TICK_PERIOD_MS);
}

static void amoled_wallpaper_update_clock(lv_timer_t *timer)
{
    if (!timer) {
        return;
    }
    lv_obj_t *label = (lv_obj_t *)lv_timer_get_user_data(timer);
    if (!label) {
        return;
    }
    uint64_t us = esp_timer_get_time();
    uint32_t seconds = (uint32_t)(us / 1000000ULL);
    uint32_t hours = (seconds / 3600U) % 24U;
    uint32_t minutes = (seconds / 60U) % 60U;
    uint32_t secs = seconds % 60U;
    lv_label_set_text_fmt(label, "%02" PRIu32 ":%02" PRIu32 ":%02" PRIu32, hours, minutes, secs);
}

static void amoled_create_wallpaper_ui(void)
{
    if (!s_lvgl_display) {
        return;
    }

    lv_obj_t *screen = lv_screen_active();

    if (!s_wallpaper_image) {
        s_wallpaper_image = lv_image_create(screen);
        lv_image_set_src(s_wallpaper_image, &wallpaper1);
        lv_obj_align(s_wallpaper_image, LV_ALIGN_CENTER, 0, 0);
    }

    if (!s_clock_label) {
        s_clock_label = lv_label_create(screen);
        lv_obj_set_style_text_color(s_clock_label, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_text_font(s_clock_label, LV_FONT_DEFAULT, LV_PART_MAIN);
        lv_label_set_text(s_clock_label, "00:00:00");
        lv_obj_align(s_clock_label, LV_ALIGN_BOTTOM_MID, 0, -20);
    }

    if (!s_clock_timer) {
        s_clock_timer = lv_timer_create(amoled_wallpaper_update_clock, 1000, s_clock_label);
        amoled_wallpaper_update_clock(s_clock_timer);
    }
}

static void amoled_lvgl_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    while (true) {
        _lock_acquire(&s_lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&s_lvgl_api_lock);

        time_till_next_ms = MAX(time_till_next_ms, AMOLED_LVGL_TASK_MIN_DELAY_MS);
        time_till_next_ms = MIN(time_till_next_ms, AMOLED_LVGL_TASK_MAX_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(time_till_next_ms));
    }
}

static void *allocate_lvgl_buffer(size_t size, const char *tag_suffix)
{
    const uint32_t caps_to_try[] = {
        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA,
        MALLOC_CAP_SPIRAM,
        MALLOC_CAP_8BIT | MALLOC_CAP_DMA,
        MALLOC_CAP_DEFAULT,
    };

    for (size_t i = 0; i < sizeof(caps_to_try) / sizeof(caps_to_try[0]); ++i) {
        void *buf = heap_caps_malloc(size, caps_to_try[i]);
        if (buf) {
            ESP_LOGI(TAG, "%s buffer allocated (%u bytes, caps=0x%08" PRIX32 ")", tag_suffix, (unsigned)size, (uint32_t)caps_to_try[i]);
            return buf;
        }
    }

    ESP_LOGE(TAG, "%s buffer allocation failed (%u bytes)", tag_suffix, (unsigned)size);
    return NULL;
}

static esp_err_t amoled_allocate_lvgl_buffers(void **buf1, void **buf2, size_t *buf_sz, size_t *buf_lines)
{
    size_t lines = AMOLED_LVGL_BUF_HEIGHT;
    if (lines < AMOLED_LVGL_MIN_BUF_HEIGHT) {
        lines = AMOLED_LVGL_MIN_BUF_HEIGHT;
    }

    while (lines >= AMOLED_LVGL_MIN_BUF_HEIGHT) {
        size_t draw_buffer_sz = AMOLED_LCD_H_RES * lines * sizeof(lv_color_t);
        void *primary = allocate_lvgl_buffer(draw_buffer_sz, "LVGL buf1");
        if (!primary) {
            lines /= 2;
            continue;
        }

        void *secondary = allocate_lvgl_buffer(draw_buffer_sz, "LVGL buf2");
        if (!secondary) {
            if (lines / 2 >= AMOLED_LVGL_MIN_BUF_HEIGHT) {
                ESP_LOGW(TAG, "Second LVGL buffer alloc failed for %u lines, retrying with half height", (unsigned)lines);
                heap_caps_free(primary);
                lines /= 2;
                continue;
            }

            ESP_LOGW(TAG, "Second LVGL buffer alloc failed, falling back to single buffer (%u lines)", (unsigned)lines);
        }

        *buf1 = primary;
        *buf2 = secondary;
        *buf_sz = draw_buffer_sz;
        *buf_lines = lines;
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to allocate LVGL draw buffer even at minimum height (%u lines)", (unsigned)AMOLED_LVGL_MIN_BUF_HEIGHT);
    return ESP_ERR_NO_MEM;
}

#if AMOLED_USE_TOUCH && AMOLED_ENABLE_I2C_SCAN
static void amoled_touch_scan_bus(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 0x7F; ++addr) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(AMOLED_TOUCH_HOST, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found I2C device at address 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "I2C scan completed.");
}
#endif

esp_err_t amoled_app_lcd_init(void)
{
    if (s_lcd_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initialize QSPI bus");
    const spi_bus_config_t buscfg = QSPI_AMOLED_PANEL_BUS_QSPI_CONFIG(
        AMOLED_PIN_LCD_PCLK,
        AMOLED_PIN_LCD_DATA0,
        AMOLED_PIN_LCD_DATA1,
        AMOLED_PIN_LCD_DATA2,
        AMOLED_PIN_LCD_DATA3,
        AMOLED_LCD_H_RES * AMOLED_LCD_V_RES * AMOLED_LCD_BIT_PER_PIXEL / 8
    );

    esp_err_t ret = spi_bus_initialize(AMOLED_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "QSPI bus already initialized, reusing it");
        ret = ESP_OK;
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to initialize QSPI bus");

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = AMOLED_PIN_LCD_CS,
        .dc_gpio_num = -1,
        .spi_mode = AMOLED_QSPI_MODE,
        .pclk_hz = AMOLED_QSPI_MAX_PCLK,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .lcd_cmd_bits = 32,
        .lcd_param_bits = 8,
        .flags = {
            .quad_mode = true,
        },
    };

    qspi_amoled_vendor_config_t vendor_config = {
        .init_cmds = s_lcd_init_cmds,
        .init_cmds_size = sizeof(s_lcd_init_cmds) / sizeof(s_lcd_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };

    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)AMOLED_LCD_HOST, &io_config, &s_panel_io),
                        TAG, "Failed to install panel IO");

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = AMOLED_PIN_LCD_RST,
        .rgb_ele_order = AMOLED_LCD_RGB_ORDER,
        .bits_per_pixel = AMOLED_LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };

    ESP_LOGI(TAG, "Install QSPI AMOLED panel driver");
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_qspi_amoled(s_panel_io, &panel_config, &s_panel_handle),
                        TAG, "Failed to create panel handle");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel_handle), TAG, "panel reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel_handle), TAG, "panel init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(s_panel_handle, AMOLED_LCD_X_GAP, AMOLED_LCD_Y_GAP),
                        TAG, "set gap failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel_handle, true), TAG, "display on failed");
    ESP_RETURN_ON_ERROR(panel_qspi_amoled_set_brightness(s_panel_handle, 0xFF),
                        TAG, "set brightness failed");

    s_lcd_initialized = true;
    return ESP_OK;
}

esp_err_t amoled_app_touch_init(void)
{
#if AMOLED_USE_TOUCH
    if (s_touch_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initialize touch I2C bus");
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = AMOLED_PIN_TOUCH_SDA,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = AMOLED_PIN_TOUCH_SCL,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = AMOLED_TOUCH_I2C_SPEED_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_param_config(AMOLED_TOUCH_HOST, &i2c_conf), TAG, "i2c param config failed");
    ESP_RETURN_ON_ERROR(i2c_set_timeout(AMOLED_TOUCH_HOST, 0x02), TAG, "i2c set timeout failed");
    esp_err_t ret = i2c_driver_install(AMOLED_TOUCH_HOST, i2c_conf.mode, 0, 0, 0);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "I2C bus already initialized, reusing it");
        ret = ESP_OK;
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "i2c driver install failed");

#if AMOLED_ENABLE_I2C_SCAN
    amoled_touch_scan_bus();
#endif

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CHSC6417_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)AMOLED_TOUCH_HOST, &tp_io_config, &tp_io_handle),
                        TAG, "Failed to create touch panel IO");

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = AMOLED_LCD_H_RES,
        .y_max = AMOLED_LCD_V_RES,
        .rst_gpio_num = AMOLED_PIN_TOUCH_RST,
        .int_gpio_num = AMOLED_PIN_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = AMOLED_TOUCH_MIRROR_X,
            .mirror_y = AMOLED_TOUCH_MIRROR_Y,
        },
    };

    ESP_LOGI(TAG, "Initialize CHSC6417 touch controller");
    ESP_RETURN_ON_ERROR(esp_lcd_touch_new_i2c_chsc6417(tp_io_handle, &tp_cfg, &s_touch_handle),
                        TAG, "Failed to initialize touch controller");

    s_touch_initialized = true;
#endif
    return ESP_OK;
}

esp_err_t amoled_app_lvgl_init(void)
{
    if (s_lvgl_initialized) {
        return ESP_OK;
    }

    ESP_RETURN_ON_FALSE(s_panel_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "LCD not initialized");

    ESP_LOGI(TAG, "Initialize LVGL");
    lv_init();

    size_t draw_buffer_sz = 0;
    size_t buffer_lines = 0;
    void *buf1 = NULL;
    void *buf2 = NULL;
    ESP_RETURN_ON_ERROR(amoled_allocate_lvgl_buffers(&buf1, &buf2, &draw_buffer_sz, &buffer_lines),
                        TAG, "Failed to allocate LVGL draw buffers");

    s_lvgl_display = lv_display_create(AMOLED_LCD_H_RES, AMOLED_LCD_V_RES);
    lv_display_set_buffers(s_lvgl_display, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_user_data(s_lvgl_display, s_panel_handle);
    lv_display_set_color_format(s_lvgl_display, AMOLED_LV_COLOR_FORMAT);
    lv_display_set_flush_cb(s_lvgl_display, amoled_lvgl_flush_cb);
    lv_display_add_event_cb(s_lvgl_display, amoled_lvgl_rounder_cb, LV_EVENT_INVALIDATE_AREA, NULL);

    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &amoled_increase_lvgl_tick,
        .name = "lvgl_tick",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&lvgl_tick_timer_args, &s_lvgl_tick_timer),
                        TAG, "Failed to create LVGL tick timer");
    esp_err_t timer_ret = esp_timer_start_periodic(s_lvgl_tick_timer, AMOLED_LVGL_TICK_PERIOD_MS * 1000);
    if (timer_ret != ESP_OK) {
        esp_timer_delete(s_lvgl_tick_timer);
        s_lvgl_tick_timer = NULL;
        ESP_RETURN_ON_ERROR(timer_ret, TAG, "Failed to start LVGL tick timer");
    }

    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = amoled_notify_lvgl_flush_ready,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_register_event_callbacks(s_panel_io, &cbs, s_lvgl_display),
                        TAG, "Failed to register panel IO callbacks");

#if AMOLED_USE_TOUCH
    if (s_touch_handle) {
        lv_indev_t *indev = lv_indev_create();
        lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_display(indev, s_lvgl_display);
        lv_indev_set_read_cb(indev, amoled_lvgl_touch_cb);
        lv_indev_set_user_data(indev, s_touch_handle);
    }
#endif

    BaseType_t ok = xTaskCreate(amoled_lvgl_task, "LVGL", AMOLED_LVGL_TASK_STACK_SIZE, NULL,
                                AMOLED_LVGL_TASK_PRIORITY, &s_lvgl_task);
    if (ok != pdPASS) {
        if (s_lvgl_tick_timer) {
            esp_timer_stop(s_lvgl_tick_timer);
            esp_timer_delete(s_lvgl_tick_timer);
            s_lvgl_tick_timer = NULL;
        }
        ESP_LOGE(TAG, "Failed to create LVGL task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LVGL buffers ready: %u lines (%s buffering)",
             (unsigned)buffer_lines, buf2 ? "double" : "single");
    s_lvgl_initialized = true;
    return ESP_OK;
}

void amoled_app_main_display(void)
{
    if (!s_lvgl_initialized) {
        ESP_LOGW(TAG, "LVGL not initialized, skip demo");
        return;
    }

    _lock_acquire(&s_lvgl_api_lock);
#if LV_USE_DEMO_WIDGETS
    lv_demo_widgets();
#else
    amoled_create_wallpaper_ui();
#endif
    _lock_release(&s_lvgl_api_lock);
}

esp_lcd_panel_handle_t amoled_app_get_panel(void)
{
    return s_panel_handle;
}
