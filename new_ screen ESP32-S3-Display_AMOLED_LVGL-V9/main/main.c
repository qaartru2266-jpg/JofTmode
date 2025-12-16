#include "lvgl_init.h"

static void app_main_display(void)
{
    /* Show LVGL objects */
    _lock_acquire(&lvgl_api_lock);

    // lv_demo_music();
    // lv_demo_benchmark();
    lv_demo_widgets();

    /* Task unlock */
    _lock_release(&lvgl_api_lock);
}

void app_main(void)
{
    /* LCD HW initialization */
    ESP_ERROR_CHECK(app_lcd_init());

    /* Encoder initialization */
    ESP_ERROR_CHECK(app_touch_init());

    /* LVGL initialization */
    ESP_ERROR_CHECK(app_lvgl_init());

    /* Show LVGL objects */
    app_main_display();
}