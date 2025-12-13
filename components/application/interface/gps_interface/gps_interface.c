#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#include "gps_interface.h"


#define TXD_PIN (GPIO_NUM_8)
#define RXD_PIN (GPIO_NUM_9)

void gps_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    // uart_driver_install(UART_NUM_1, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0);
    uart_driver_install(UART_NUM_1, GPS_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

unsigned int GpsSendData(const char* logName, const char* data, const int len)
{
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}


unsigned int GpsReadData(unsigned char *r_data)
{
    // unsigned short len;
    // uart_get_buffered_data_len(UART_NUM_1, &len);  
    // return uart_read_bytes(UART_NUM_1, r_data, len, 2 / portTICK_PERIOD_MS);
    return uart_read_bytes(UART_NUM_1, r_data, GPS_BUF_SIZE, 0);
}

