#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "freertos/semphr.h"


typedef struct {
    esp_lcd_panel_handle_t     panel;      // 由 esp_lcd_new_panel_sh8601() 产生
    esp_lcd_panel_io_handle_t  io;         // IO 句柄（new_panel_io_spi 后得到）
    SemaphoreHandle_t          trans_done; // DMA 传输完成信号量

    uint16_t                   hor_res;
    uint16_t                   ver_res;
} display_hal_t;

// 初始化 SH8601（QSPI），点亮屏并返回 panel 句柄与分辨率
esp_err_t display_hal_init(display_hal_t *out);

