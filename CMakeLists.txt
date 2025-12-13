cmake_minimum_required(VERSION 3.16)

# 告诉 IDF 额外去二级目录找组件：
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/components/drivers/display_hal"
    "${CMAKE_CURRENT_LIST_DIR}/components/drivers/esp_lcd_sh8601"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(JofTmode)

