// main/app_gui.c
#include "app_gui.h"
#include "display_hal.h"


#include "lvgl.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "ml_window.h"    // ml_get_latest_result()
#include "app_sdcard.h"   

#include <string.h>
#include <stdio.h>
#include <time.h>

#include "app_touch.h"


static const char *TAG = "app_gui";
static display_hal_t s_hal;
static esp_lcd_panel_handle_t s_panel_handle = NULL;
static bool s_screen_on = true;
static lv_indev_t *s_touch_indev = NULL; 

static lv_obj_t *s_main_screen = NULL;
static lv_obj_t *s_clock_screen = NULL;
static lv_obj_t *s_clock_label = NULL;
static lv_timer_t *s_clock_timer = NULL;
static time_t s_clock_epoch = 0;

extern const lv_image_dsc_t wallpaper1;

/* ---------- LVGL tick（v9 要求“返回毫秒”） ---------- */
static uint32_t lv_tick_cb(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ---------- 刷新回调：把 px_map 区域刷到面板 ---------- */
static void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    if (s_hal.trans_done) (void)xSemaphoreTake(s_hal.trans_done, 0);

    esp_err_t e = esp_lcd_panel_draw_bitmap(
        s_hal.panel,
        area->x1, area->y1,
        area->x2 + 1, area->y2 + 1,     //  +1（右下角开区间）
        px_map                          //  RGB888，3B/像素，试过rgb5xx会乱码
    );

    if (e == ESP_OK && s_hal.trans_done) {
        (void)xSemaphoreTake(s_hal.trans_done, portMAX_DELAY);
    } else if (e != ESP_OK) {
        ESP_LOGE(TAG, "draw_bitmap failed: %d", (int)e);
    }

    lv_display_flush_ready(disp);
}

#if 0
#define GUI_UPDATE_MS  200   // 每 200ms 刷一次文案

/* 若工程里没实现 app_ml_get_latest，这个弱符号会生效，返回 false。一旦在 app_sdcard.c 里提供了真正的实现，这里会被自动覆盖。 */
__attribute__((weak)) bool app_ml_get_latest(ml_result_t *out) {
    (void)out; return false;
}

/* 若 lv_conf.h 启用了 20 号字，这里声明它（文件作用域，避免函数内告警） */
#if LV_FONT_MONTSERRAT_20
LV_FONT_DECLARE(lv_font_montserrat_20);
#endif

/* ---------- 只“取值并拼文本”，不操作 UI ---------- */
static void build_text_from_ml(char *out, size_t out_sz)
{
    ml_result_t r;
    bool ok = app_ml_get_latest(&r);     // 优先快照（非消耗型）
    if (!ok) ok = ml_get_latest_result(&r); // 回退

    if (ok) {
        const char *mode = (r.pred == 0) ? "walk" : "ebike";
        snprintf(out, out_sz, "%s", mode);   // ← 只显示 walk/ebike
    } else {
        snprintf(out, out_sz, "Waiting for model");
    }
}
#endif

/* ---------- 触控回调函数 ---------- */
static void touch_read_cb(lv_indev_t * indev, lv_indev_data_t * data)
{
    LV_UNUSED(indev);

    int32_t x = 0, y = 0;
    bool pressed = app_touch_read(&x, &y); // 调用 C++ 那边的方法

    if(pressed) {
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = x;
        data->point.y = y;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void update_clock_label(void)
{
    if (!s_clock_label) {
        return;
    }

    struct tm current_time;
    if (localtime_r(&s_clock_epoch, &current_time) == NULL) {
        return;
    }

    char buf[32];
    strftime(buf, sizeof(buf), "%Y/%m/%d %H:%M:%S", &current_time);
    lv_label_set_text(s_clock_label, buf);
}

static void clock_timer_cb(lv_timer_t *timer)
{
    LV_UNUSED(timer);

    s_clock_epoch += 1;
    update_clock_label();
}

static void gesture_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_GESTURE) {
        return;
    }

    lv_obj_t *target = lv_event_get_target(e);
    lv_dir_t dir = LV_DIR_NONE;
    lv_indev_t *active = lv_indev_active();
    if (active) {
        dir = lv_indev_get_gesture_dir(active);
    }

    if (target == s_main_screen && dir == LV_DIR_LEFT) {
        lv_screen_load_anim(s_clock_screen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
    } else if (target == s_clock_screen && dir == LV_DIR_RIGHT) {
        lv_screen_load_anim(s_main_screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
    }
}


/* ---------- GUI 任务（唯一地方调用 lv_label_set_text） ---------- */
static void gui_task(void *arg)
{
    ESP_LOGI(TAG, "GUI task start");

    // 1) 初始化底层显示
    esp_err_t err = display_hal_init(&s_hal);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "display_hal_init failed: %d", (int)err);
        vTaskDelete(NULL);
        return;
    }
    s_panel_handle = s_hal.panel;
    s_screen_on = true;

    // 2) 初始化 LVGL & tick
    lv_init();
    lv_tick_set_cb(lv_tick_cb);

    // 3) 创建 display —— 保持 RGB888（3 字节/像素）
    lv_display_t *disp = lv_display_create(s_hal.hor_res, s_hal.ver_res);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB888);

    // 4) 配置行缓冲（格式也用 RGB888）
    const uint32_t line_cnt = 24;   // 24 行缓冲，带宽与内存的折中
    lv_draw_buf_t *dbuf1 = lv_draw_buf_create(
        s_hal.hor_res, line_cnt, LV_COLOR_FORMAT_RGB888, 0
    );
    lv_display_set_draw_buffers(disp, dbuf1, NULL);

    // 5) 刷新回调
    lv_display_set_flush_cb(disp, flush_cb);

// ... 原有的 lv_display_set_flush_cb(disp, flush_cb); 之后 ...

    // --- 新增：初始化硬件触控 ---
    app_touch_init();

    // --- 新增：注册 LVGL 输入设备 (LVGL v9 写法) ---
    s_touch_indev = lv_indev_create();                // 创建输入设备
    lv_indev_set_type(s_touch_indev, LV_INDEV_TYPE_POINTER);  // 设置类型为指针(触摸)
    lv_indev_set_read_cb(s_touch_indev, touch_read_cb);       // 设置回调函数
    lv_indev_set_display(s_touch_indev, disp);                // 绑定到当前屏幕

    // ... 原有的 lv_obj_set_style_bg_color ...


    // 6) 黑底 + 白字；彻底“非斜体”，并保证真正居中
    s_main_screen = lv_screen_active();
    lv_obj_set_style_bg_color(s_main_screen, lv_color_black(), 0);

#if 0
    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

    // 让 Label 宽度 = 整屏宽，“居中”有参照
    lv_obj_set_width(label, s_hal.hor_res);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);  // 多行时正常换行（此处单行）

    // 字体：启用了 20 号字就用20；否则退回默认字体（默认是正体）
    #if LV_FONT_MONTSERRAT_20
        lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
    #else
        lv_obj_set_style_text_font(label, LV_FONT_DEFAULT, 0);
    #endif

    // 明确禁用任何几何变换（防止倾斜/旋转）
    lv_obj_set_style_transform_angle(label, 0, 0);
    lv_obj_set_style_transform_skew_x(label, 0, 0);
    lv_obj_set_style_transform_skew_y(label, 0, 0);
    lv_obj_set_style_text_decor(label, LV_TEXT_DECOR_NONE, 0);

    // 放中间
    lv_obj_center(label);

    // 首帧文案
    lv_label_set_text(label, "Waiting for model");
#endif

    // 用整屏图片替换文字显示
    lv_obj_t *img = lv_image_create(s_main_screen);
    lv_image_set_src(img, &wallpaper1);
    lv_obj_set_size(img, s_hal.hor_res, s_hal.ver_res);
    lv_obj_center(img);
    lv_obj_add_flag(img, LV_OBJ_FLAG_GESTURE_BUBBLE);

    // Screen 2: clock screen with black background
    s_clock_screen = lv_obj_create(NULL);
    lv_obj_remove_style_all(s_clock_screen);
    lv_obj_set_style_bg_color(s_clock_screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_clock_screen, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_clock_screen, LV_OBJ_FLAG_SCROLLABLE);

    s_clock_label = lv_label_create(s_clock_screen);
    lv_obj_set_style_text_color(s_clock_label, lv_color_white(), 0);
    lv_obj_set_style_text_align(s_clock_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(s_clock_label);
    lv_obj_add_flag(s_clock_label, LV_OBJ_FLAG_GESTURE_BUBBLE);

    struct tm initial_tm = {
        .tm_year = 2025 - 1900,
        .tm_mon = 12 - 1,
        .tm_mday = 9,
        .tm_hour = 10,
        .tm_min = 11,
        .tm_sec = 0,
    };
    s_clock_epoch = mktime(&initial_tm);
    update_clock_label();

    s_clock_timer = lv_timer_create(clock_timer_cb, 1000, NULL);
    if (s_clock_timer) {
        lv_timer_set_repeat_count(s_clock_timer, -1);
    }

    lv_obj_add_event_cb(s_main_screen, gesture_event_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_add_event_cb(s_clock_screen, gesture_event_cb, LV_EVENT_GESTURE, NULL);

    // 7) 启动时主动全刷
    lv_obj_invalidate(s_main_screen);
    lv_refr_now(disp);

#if 0
    // 8) 主循环：每 GUI_UPDATE_MS 构文案；仅变化时才 set_text
    uint32_t last_update_ms = 0;
    char     last_txt[32] = {0};
#endif

    ESP_LOGW(TAG, "LVGL fmt=RGB888; ensure panel is 3B/px (18/24bpp) to avoid slant.");

    while (1) {
        lv_timer_handler();

#if 0
        uint32_t now = lv_tick_cb();
        if (now - last_update_ms >= GUI_UPDATE_MS) {
            last_update_ms = now;

            char txt[32];
            build_text_from_ml(txt, sizeof(txt));

            if (strcmp(last_txt, txt) != 0) {
                lv_label_set_text(label, txt);       // ← 唯一修改 UI 文本的地方
                strncpy(last_txt, txt, sizeof(last_txt) - 1);
                last_txt[sizeof(last_txt) - 1] = '\0';
            }
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t app_gui_start(void)
{
    static bool started = false;
    if (started) return ESP_OK;
    started = true;

    BaseType_t ok = xTaskCreate(gui_task, "gui", 8192, NULL, 5, NULL);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

void app_gui_screen_on(void)
{
    s_screen_on = true;
    if (s_panel_handle) {
        esp_lcd_panel_disp_on_off(s_panel_handle, true);
    }
}

void app_gui_screen_off(void)
{
    s_screen_on = false;
    if (s_panel_handle) {
        esp_lcd_panel_disp_on_off(s_panel_handle, false);
    }
}

bool app_gui_screen_is_on(void)
{
    return s_screen_on;
}
