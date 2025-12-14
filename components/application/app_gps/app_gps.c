#include <string.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_gps.h"
#include "app_gps_parser.h"
#include "app_state.h"
#include "gps_interface.h"

static const char *TAG = "gps";

static unsigned char s_read_buf[GPS_BUF_SIZE];
static char s_line_buf[GPS_BUF_SIZE];
static size_t s_line_len = 0;
static gps_parser_t s_parser;

static void dispatch_sentence(const char *line)
{
    GNSS_Data parsed = {0};
    if (gps_parser_handle_sentence(&s_parser, line, &parsed)) {
        app_state_set_gps_data(&parsed);
        ESP_LOGD(TAG, "GPS updated: lat=%.5f lon=%.5f speed=%.2f",
                 parsed.latitude, parsed.longitude, parsed.speed);
    }
}

static void process_buffer(const unsigned char *data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        char c = (char)data[i];

        if (c == '$') {
            s_line_len = 0;
        }

        if ((c == '\r' || c == '\n')) {
            if (s_line_len > 0) {
                s_line_buf[s_line_len] = '\0';
                dispatch_sentence(s_line_buf);
                s_line_len = 0;
            }
            continue;
        }

        if (s_line_len == 0 && c != '$') {
            continue;
        }

        if (s_line_len < GPS_BUF_SIZE - 1) {
            s_line_buf[s_line_len++] = c;
        } else {
            s_line_len = 0;
        }
    }
}

static void app_gps_task(void *arg)
{
    gps_parser_init(&s_parser);
    gps_init();

    while (1) {
        memset(s_read_buf, 0, sizeof(s_read_buf));
        unsigned int len = GpsReadData(s_read_buf);
        if (len > 0) {
            process_buffer(s_read_buf, len);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_gps_start(void)
{
    xTaskCreate(app_gps_task, "app_gps", 10240, NULL, 10, NULL);
}
