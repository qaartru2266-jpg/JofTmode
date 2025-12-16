#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_qspi_amoled.h"
#include "lvgl.h"

// Hardware resources used by the AMOLED reference design
#define AMOLED_LCD_HOST          SPI2_HOST
#define AMOLED_TOUCH_HOST        I2C_NUM_0

#define AMOLED_USE_TOUCH         1

#if LV_COLOR_DEPTH == 16
#define AMOLED_LCD_BIT_PER_PIXEL    (16)
#define AMOLED_LV_COLOR_FORMAT      LV_COLOR_FORMAT_RGB565
#define AMOLED_LCD_RGB_ORDER        LCD_RGB_ELEMENT_ORDER_BGR
#elif LV_COLOR_DEPTH == 24
#define AMOLED_LCD_BIT_PER_PIXEL    (24)
#define AMOLED_LV_COLOR_FORMAT      LV_COLOR_FORMAT_RGB888
#define AMOLED_LCD_RGB_ORDER        LCD_RGB_ELEMENT_ORDER_BGR
#else
#error "Unsupported LV_COLOR_DEPTH for AMOLED panel"
#endif

// Pinout (AM160Q480480LK)
#define AMOLED_PIN_LCD_CS        GPIO_NUM_14
#define AMOLED_PIN_LCD_PCLK      GPIO_NUM_7
#define AMOLED_PIN_LCD_DATA0     GPIO_NUM_10
#define AMOLED_PIN_LCD_DATA1     GPIO_NUM_11
#define AMOLED_PIN_LCD_DATA2     GPIO_NUM_12
#define AMOLED_PIN_LCD_DATA3     GPIO_NUM_13
#define AMOLED_PIN_LCD_RST       GPIO_NUM_47

#define AMOLED_PIN_TOUCH_SCL     GPIO_NUM_42
#define AMOLED_PIN_TOUCH_SDA     GPIO_NUM_41
#define AMOLED_PIN_TOUCH_RST     GPIO_NUM_40
#define AMOLED_PIN_TOUCH_INT     GPIO_NUM_39

// Panel geometry
#define AMOLED_LCD_H_RES         480
#define AMOLED_LCD_V_RES         480
#define AMOLED_LCD_X_GAP         0
#define AMOLED_LCD_Y_GAP         0
#define AMOLED_TOUCH_MIRROR_X    false
#define AMOLED_TOUCH_MIRROR_Y    false

#define AMOLED_QSPI_MODE         0
#define AMOLED_QSPI_MAX_PCLK     (40 * 1000 * 1000)

// LVGL scheduling
#define AMOLED_LVGL_TICK_PERIOD_MS     2
#define AMOLED_LVGL_TASK_MAX_DELAY_MS  500
#define AMOLED_LVGL_TASK_MIN_DELAY_MS  1
#define AMOLED_LVGL_TASK_STACK_SIZE    (4 * 1024 * 2)
#define AMOLED_LVGL_TASK_PRIORITY      2
#define AMOLED_LVGL_BUF_HEIGHT         (AMOLED_LCD_V_RES / 10)
#define AMOLED_LVGL_MIN_BUF_HEIGHT     12

// Optional helpers
#define AMOLED_ENABLE_I2C_SCAN         0
#define AMOLED_TOUCH_I2C_SPEED_HZ      (200 * 1000)
