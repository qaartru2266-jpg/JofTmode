idf_component_register(
    SRCS
        "display_hal.c"          # 只保留你真实存在的 .c；不要写 sh8601_vendor.c
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_lcd                   # 你的 HAL 若用到了 esp_lcd APIs
        esp_lcd_sh8601            # 依赖你自家的 sh8601 组件
)
