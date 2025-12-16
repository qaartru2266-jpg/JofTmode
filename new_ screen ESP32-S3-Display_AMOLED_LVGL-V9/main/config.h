#pragma once

#include <stdio.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "lv_demos.h"
#include "esp_lcd_qspi_amoled.h"
// 引入你唯一的触摸驱动
#include "esp_lcd_touch_chsc6417.h"

static const char *TAG = "AM160_Project";

// ================= 硬件配置 =================
#define LCD_HOST    SPI2_HOST
#define TOUCH_HOST  I2C_NUM_0

// 屏幕颜色深度配置
#if LV_COLOR_DEPTH == 16
    #define LCD_BIT_PER_PIXEL       (16)
    #define LV_COLOR_FORMAT         LV_COLOR_FORMAT_RGB565
    #define LCD_RGB_ELEMENT_ORDER   LCD_RGB_ELEMENT_ORDER_RGB
#elif LV_COLOR_DEPTH == 24
    #define LCD_BIT_PER_PIXEL       (24)
    #define LV_COLOR_FORMAT         LV_COLOR_FORMAT_RGB888
    #define LCD_RGB_ELEMENT_ORDER   LCD_RGB_ELEMENT_ORDER_BGR
#endif

#define EXAMPLE_USE_TOUCH           1 

// ================= 引脚定义 (针对 AM160Q480480LK) =================
#define EXAMPLE_PIN_NUM_LCD_CS      (GPIO_NUM_14)
#define EXAMPLE_PIN_NUM_LCD_PCLK    (GPIO_NUM_7) 
#define EXAMPLE_PIN_NUM_LCD_DATA0   (GPIO_NUM_10)
#define EXAMPLE_PIN_NUM_LCD_DATA1   (GPIO_NUM_11)
#define EXAMPLE_PIN_NUM_LCD_DATA2   (GPIO_NUM_12)
#define EXAMPLE_PIN_NUM_LCD_DATA3   (GPIO_NUM_13)
#define EXAMPLE_PIN_NUM_LCD_RST     (GPIO_NUM_47)

// 触摸引脚
#define EXAMPLE_PIN_NUM_TOUCH_SCL   (GPIO_NUM_42)
#define EXAMPLE_PIN_NUM_TOUCH_SDA   (GPIO_NUM_41)
#define EXAMPLE_PIN_NUM_TOUCH_RST   (GPIO_NUM_40)
#define EXAMPLE_PIN_NUM_TOUCH_INT   (GPIO_NUM_39)

// ================= 屏幕参数 =================
#define EXAMPLE_LCD_H_RES           480
#define EXAMPLE_LCD_V_RES           480
#define EXAMPLE_LCD_X_GAP           0
#define EXAMPLE_LCD_Y_GAP           0
#define EXAMPLE_TOUCH_MIRROR_X      false
#define EXAMPLE_TOUCH_MIRROR_Y      false

#define AMOLED_QSPI_MODE            0
#define AMOLED_QSPI_MAX_PCLK        40 * 1000 * 1000

// LVGL 任务配置
#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024 * 2)
#define EXAMPLE_LVGL_TASK_PRIORITY     2
#define EXAMPLE_LVGL_BUF_HEIGHT        (EXAMPLE_LCD_V_RES / 10)

// 全局变量声明
static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
#if EXAMPLE_USE_TOUCH
    extern esp_lcd_touch_handle_t tp; // 声明触摸句柄
#endif

// ================= 屏幕初始化指令 (CH13613) =================
static const qspi_amoled_lcd_init_cmd_t lcd_init_cmds[] = {
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