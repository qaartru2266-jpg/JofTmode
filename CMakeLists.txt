cmake_minimum_required(VERSION 3.16)

# 告诉 IDF 额外去二级目录找组件：
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/components/drivers/esp_lcd_qspi_amoled"
    "${CMAKE_CURRENT_LIST_DIR}/components/drivers/esp_lcd_touch_chsc6417"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(JofTmode)
