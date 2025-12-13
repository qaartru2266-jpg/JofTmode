idf_component_register(
    SRCS
        "ml_runner.cc"
        "ml_window.c"
        "model/model_data.cc"
    INCLUDE_DIRS
        "."
        "include"
    REQUIRES              # 公开依赖（让本组件能找到这些头文件/库）
        esp-tflite-micro
        esp-nn
    PRIV_REQUIRES         # 私有依赖（只编译期/链接期用，不往外暴露）
        esp_timer
)
