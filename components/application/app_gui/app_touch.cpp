// components/application/app_gui/app_touch.cpp
#include "app_touch.h"
#include "SensorLib.h"
#include "TouchDrvCST92xx.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "AppTouch";

// --- 引脚定义 (参考你的例程 example_qspi_with_ram.cpp) ---
#define TOUCH_SDA  21
#define TOUCH_SCL  14
#define TOUCH_RST  40
#define TOUCH_INT  11

// 使用 I2C 端口 1，避免和其他传感器(通常在端口0)冲突
#define TOUCH_I2C_PORT  I2C_NUM_1 

// 定义触控对象
TouchDrvCST92xx touch;
int16_t touch_x[5], touch_y[5]; // 缓存坐标

// 初始化 I2C (如果你的项目中其他地方已经初始化了这两个脚的I2C，这里需要调整)
static void touch_i2c_init(void) {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)TOUCH_SDA;
    conf.scl_io_num = (gpio_num_t)TOUCH_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000; // 100kHz
    
    // 检查是否已经安装，如果没安装才安装
    if (i2c_driver_install(TOUCH_I2C_PORT, conf.mode, 0, 0, 0) == ESP_OK) {
        i2c_param_config(TOUCH_I2C_PORT, &conf);
        ESP_LOGI(TAG, "Touch I2C initialized");
    } else {
        ESP_LOGW(TAG, "Touch I2C might be already initialized, skipping init");
    }
}

// 供 C 语言调用的初始化函数
void app_touch_init(void) {
    touch_i2c_init();

    touch.setPins(TOUCH_RST, TOUCH_INT);
    
    // 这里的地址 0x5A 是 CST92xx 的常见地址
    bool has_begun = touch.begin(TOUCH_I2C_PORT, 0x5A, TOUCH_SDA, TOUCH_SCL);
    if (!has_begun) {
        ESP_LOGE(TAG, "Failed to find CST92xx - check wiring!");
        return;
    }

    touch.reset();
    
    // 设置分辨率 (参考例程)
    touch.setMaxCoordinates(466, 466); 
    
    // 设置镜像/翻转 (参考例程)
    touch.setMirrorXY(true, true);

    ESP_LOGI(TAG, "Touch initialized successfully");
}

// 供 C 语言调用的读取函数
bool app_touch_read(int32_t *x, int32_t *y) {
    // 读取 1 个点
    uint8_t touched = touch.getPoint(touch_x, touch_y, 1);
    
    if (touched > 0) {
        *x = touch_x[0];
        *y = touch_y[0];
        
        // 【关键修改】改成 ESP_LOGE 以强制打印，并加上 (int) 强转防止报错
        ESP_LOGE(TAG, "Touch: X=%d, Y=%d", (int)*x, (int)*y); 
        
        return true;
    }
    return false;
}