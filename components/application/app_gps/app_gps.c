#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gps_interface.h"
#include "app_gps.h"

const static char *TAG = "gps";
static unsigned char read_buf[GPS_BUF_SIZE];


GNSS_Data current_data = {0};



// 度分格式转换为十进制度
double ddm_to_degrees(double ddm) {
    if (ddm < 0) ddm = -ddm;
    int degrees = (int)(ddm / 100);
    double minutes = ddm - (degrees * 100.0);
    return (ddm < 0 ? -1 : 1) * (degrees + (minutes / 60.0));
}

// 解析系统标识
SatelliteSystem parse_system_id(const char* sentence) {
    char prefix[3] = {0};
    strncpy(prefix, sentence + 1, 2);
    
    if (strcmp(prefix, "GP") == 0) return SYS_GPS;
    if (strcmp(prefix, "GL") == 0) return SYS_GLONASS;
    if (strcmp(prefix, "BD") == 0) return SYS_BEIDOU;
    if (strcmp(prefix, "GA") == 0) return SYS_GALILEO;
    if (strcmp(prefix, "GN") == 0) return SYS_GNSS;
    return SYS_UNKNOWN;
}

// 校验和验证
int validate_checksum(const char* sentence) {
    const char* asterisk = strchr(sentence, '*');
    if (!asterisk) return 0;
    
    unsigned char calculated = 0;
    for (const char* p = sentence + 1; p < asterisk; p++) {
        calculated ^= *p;
    }
    
    unsigned char received = (unsigned char)strtol(asterisk + 1, NULL, 16);
    return calculated == received;
}

// 解析定位模式
PositionMode parse_position_mode(char mode_char) {
    switch (mode_char) {
        case 'A': return MODE_AUTONOMOUS;
        case 'D': return MODE_DIFFERENTIAL;
        case 'V': return MODE_INVALID;
        case 'R': return MODE_VALIDATED;  // 差分验证
        default:  return MODE_UNKNOWN;
    }
}

// 解析天线状态
void parse_antenna_status(GNSS_Data* data, const char* sentence) {
    char* antenna_pos = strstr(sentence, "ANTENNA ");
    if (!antenna_pos) return;
    
    antenna_pos += 8;
    if (strncmp(antenna_pos, "OPEN", 4) == 0) {
        data->antenna_status = ANTENNA_OPEN;
        data->is_valid = 0;  // 天线开路时标记为无效
    }
    else if (strncmp(antenna_pos, "SHORT", 5) == 0) {
        data->antenna_status = ANTENNA_SHORT;
        data->is_valid = 0;  // 天线短路时标记为无效
    }
    else if (strncmp(antenna_pos, "OK", 2) == 0) {
        data->antenna_status = ANTENNA_OK;
    }
}

// 解析GGA语句
void parse_GGA(GNSS_Data* data, const char* sentence) {
    // if (!validate_checksum(sentence)) return;
    
    char copy[128];
    strncpy(copy, sentence, sizeof(copy)-1);
    char* token = strtok(copy, ",");
    int field_count = 0;
    double lat_ddm = 0.0, lon_ddm = 0.0;
    
    data->system = parse_system_id(sentence);
    
    while (token != NULL) {
        switch (field_count) {
            case 1:  // UTC时间
                if (strlen(token) > 0) 
                    strncpy(data->timestamp, token, sizeof(data->timestamp)-1);
                break;
            case 2:  // 纬度 (度分格式)
                if (token[0]) lat_ddm = atof(token);
                break;
            case 3:  // 纬度方向 (N/S)
                if (token[0] && token[0] == 'S') lat_ddm = -lat_ddm;
                break;
            case 4:  // 经度 (度分格式)
                if (token[0]) lon_ddm = atof(token);
                break;
            case 5:  // 经度方向 (E/W)
                if (token[0] && token[0] == 'W') lon_ddm = -lon_ddm;
                break;
            case 6:  // 定位质量指示
                // 0=无效，1=GPS定位，2=差分定位
                break;
            case 7:  // 卫星数量
                data->satellite_count = token[0] ? atoi(token) : 0;
                break;
            case 8:  // HDOP水平精度因子
                data->hdop = token[0] ? atof(token) : 99.9;
                break;
            case 9:  // 海拔高度
                if (token[0]) data->altitude = atof(token);
                break;
            case 11: // 大地水准面分离
                if (token[0]) data->geoid_separation = atof(token);
                break;
        }
        
        if (field_count == 5) {
            data->latitude = ddm_to_degrees(lat_ddm);
            data->longitude = ddm_to_degrees(lon_ddm);
        }
        
        token = strtok(NULL, ",");
        field_count++;
    }
}

// 解析RMC语句
void parse_RMC(GNSS_Data* data, const char* sentence) {
    // if (!validate_checksum(sentence)) return;
    
    char copy[128];
    strncpy(copy, sentence, sizeof(copy)-1);
    char* token = strtok(copy, ",");
    int field_count = 0;
    double lat_ddm = 0.0, lon_ddm = 0.0;
    char status_char = 0;
    char mode_char = 0;
    
    data->system = parse_system_id(sentence);
    
    while (token != NULL) {
        switch (field_count) {
            case 1:  // UTC时间
                if (strlen(token) > 0) 
                    strncpy(data->timestamp, token, sizeof(data->timestamp)-1);
                break;
            case 2:  // 状态 (A=有效，V=无效)
                status_char = token[0];
                data->is_valid = (status_char == 'A') ? 1 : 0;
                break;
            case 3:  // 纬度 (度分格式)
                if (token[0]) lat_ddm = atof(token);
                break;
            case 4:  // 纬度方向
                if (token[0] && token[0] == 'S') lat_ddm = -lat_ddm;
                break;
            case 5:  // 经度
                if (token[0]) lon_ddm = atof(token);
                break;
            case 6:  // 经度方向
                if (token[0] && token[0] == 'W') lon_ddm = -lon_ddm;
                break;
            case 7:  // 地面速度 (节)
                if (token[0] && data->is_valid)
                    data->speed = atof(token) * 0.5144; // 节转为m/s
                break;
            case 8:  // 航向 (度)
                if (token[0] && data->is_valid)
                    data->course = atof(token);
                break;
            case 9:  // UTC日期
                if (token[0]) strncpy(data->date, token, sizeof(data->date)-1);
                break;
            case 12: // 定位模式
                if (token[0]) mode_char = token[0];
                break;
        }
        
        if (field_count == 6) {
            data->latitude = ddm_to_degrees(lat_ddm);
            data->longitude = ddm_to_degrees(lon_ddm);
        }
        
        token = strtok(NULL, ",");
        field_count++;
    }
    
    data->position_mode = parse_position_mode(mode_char);
    
    // 如果模式无效，覆盖有效状态
    if (data->position_mode == MODE_INVALID) {
        data->is_valid = 0;
    }
}

// 解析VTG语句
void parse_VTG(GNSS_Data* data, const char* sentence) {
    if (!validate_checksum(sentence)) return;
    
    char copy[128];
    strncpy(copy, sentence, sizeof(copy)-1);
    char* token = strtok(copy, ",");
    int field_count = 0;
    
    data->system = parse_system_id(sentence);
    
    while (token != NULL && data->is_valid) {
        switch (field_count) {
            case 1:  // 真北方向航向 (度)
                if (token[0]) data->course = atof(token);
                break;
            case 3:  // 磁北方向航向 (度) - 跳过
                break;
            case 5:  // 地面速度 (节)
                if (token[0]) data->speed = atof(token) * 0.5144; // 节转为m/s
                break;
            case 7:  // 地面速度 (km/h)
                if (token[0]) data->speed = atof(token) / 3.6; // km/h转为m/s
                break;
        }
        token = strtok(NULL, ",");
        field_count++;
    }
}

// 解析GSA语句
void parse_GSA(GNSS_Data* data, const char* sentence) {
    if (!validate_checksum(sentence)) return;
    
    char copy[128];
    strncpy(copy, sentence, sizeof(copy)-1);
    char* token = strtok(copy, ",");
    int field_count = 0;
    int sat_count = 0;
    
    while (token != NULL) {
        switch (field_count) {
            case 2: // 卫星PRN列表开始
                for (int i = 0; i < 12; i++) {
                    token = strtok(NULL, ",");
                    field_count++;
                    if (token == NULL || *token == '\0') break;
                    if (atoi(token) > 0) sat_count++;
                }
                break;
            case 15: // PDOP位置精度因子
                break;
            case 16: // HDOP水平精度因子
                if (token[0]) data->hdop = atof(token);
                break;
            case 17: // VDOP垂直精度因子
                break;
        }
        token = strtok(NULL, ",");
        field_count++;
    }
    
    if (sat_count > data->satellite_count) {
        data->satellite_count = sat_count;
    }
}

// 解析GSV语句
void parse_GSV(GNSS_Data* data, const char* sentence) {
    if (!validate_checksum(sentence)) return;
    
    char copy[128];
    strncpy(copy, sentence, sizeof(copy)-1);
    char* token = strtok(copy, ",");
    int field_count = 0;
    int total_msgs = 0, msg_num = 0, total_sats = 0;
    
    SatelliteSystem sys = parse_system_id(sentence);
    if (sys == SYS_GPS || sys == SYS_BEIDOU) {
        // 只统计GPS和北斗卫星
        while (token != NULL) {
            switch (field_count) {
                case 1: // 总消息数
                    total_msgs = token[0] ? atoi(token) : 0;
                    break;
                case 2: // 当前消息序号
                    msg_num = token[0] ? atoi(token) : 0;
                    break;
                case 3: // 总卫星数
                    total_sats = token[0] ? atoi(token) : 0;
                    if (total_sats > data->satellite_total) {
                        data->satellite_total = total_sats;
                    }
                    break;
            }
            token = strtok(NULL, ",");
            field_count++;
        }
    }
}

// 解析ZDA语句
void parse_ZDA(GNSS_Data* data, const char* sentence) {
    if (!validate_checksum(sentence)) return;
    
    char copy[128];
    strncpy(copy, sentence, sizeof(copy)-1);
    char* token = strtok(copy, ",");
    int field_count = 0;
    
    while (token != NULL) {
        switch (field_count) {
            case 1:  // UTC时间
                if (strlen(token) > 0) 
                    strncpy(data->timestamp, token, sizeof(data->timestamp)-1);
                break;
            case 2:  // 日
                if (strlen(token) == 2) {
                    strncpy(data->date, token, 2);
                }
                break;
            case 3:  // 月
                if (strlen(token) == 2 && data->date[0] != '\0') {
                    strncpy(data->date + 2, token, 2);
                }
                break;
            case 4:  // 年
                if (strlen(token) == 4 && data->date[0] != '\0') {
                    strncpy(data->date + 4, token + 2, 2); // 取后两位
                }
                break;
        }
        token = strtok(NULL, ",");
        field_count++;
    }
}

// 主解析函数
void parse_nmea_sentence(const char* sentence, GNSS_Data* data) {
    // 检查最小有效长度和起始字符
    if (strlen(sentence) < 7 || sentence[0] != '$') return;
    
    // 执行校验和验证
    if (!validate_checksum(sentence)) {
        ESP_LOGI(TAG, "校验失败: %s\n", sentence);
        return;
    }
    
    // 根据语句类型调用不同解析器
    if (strstr(sentence, "$GNGGA") || strstr(sentence, "$GPGGA")) {
        parse_GGA(data, sentence);
    }
    else if (strstr(sentence, "$GNRMC") || strstr(sentence, "$GPRMC")) {
        parse_RMC(data, sentence);
    }
    else if (strstr(sentence, "$GNVTG") || strstr(sentence, "$GPVTG")) {
        parse_VTG(data, sentence);
    }
    else if (strstr(sentence, "$GNGSA") || strstr(sentence, "$GPGSA") ||
             strstr(sentence, "$BDGSA")) {
        parse_GSA(data, sentence);
    }
    else if (strstr(sentence, "$GPGSV") || strstr(sentence, "$BDGSV") ||
             strstr(sentence, "$GLGSV")) {
        parse_GSV(data, sentence);
    }
    else if (strstr(sentence, "$GNZDA") || strstr(sentence, "$GPZDA")) {
        parse_ZDA(data, sentence);
    }
    else if (strstr(sentence, "$GPTXT")) {
        parse_antenna_status(data, sentence);
    }
    else if (strstr(sentence, "$GNGLL")) {
        // 可选实现GNGLL解析
    }
}

// 获取卫星系统名称
const char* get_system_name(SatelliteSystem sys) {
    switch (sys) {
        case SYS_GPS: return "GPS";
        case SYS_GLONASS: return "GLONASS";
        case SYS_BEIDOU: return "BeiDou";
        case SYS_GALILEO: return "Galileo";
        case SYS_GNSS: return "Multi-GNSS";
        default: return "Unknown";
    }
}

// 获取定位模式名称
const char* get_mode_name(PositionMode mode) {
    switch (mode) {
        case MODE_AUTONOMOUS: return "Autonomous";
        case MODE_DIFFERENTIAL: return "DGPS/DGNSS";
        case MODE_VALIDATED: return "Validated RTK";
        case MODE_INVALID: return "Invalid";
        default: return "Unknown";
    }
}

// 获取天线状态名称
const char* get_antenna_status(AntennaStatus status) {
    switch (status) {
        case ANTENNA_OK: return "OK";
        case ANTENNA_SHORT: return "SHORT";
        case ANTENNA_OPEN: return "OPEN";
        default: return "Unknown";
    }
}

// 打印GNSS数据
void print_gnss_data(const GNSS_Data* data) {
    ESP_LOGI(TAG, "\n===== GNSS定位数据 [%s] =====\n", data->timestamp);
    ESP_LOGI(TAG, "卫星系统: %s\n", get_system_name(data->system));
    ESP_LOGI(TAG, "位置: %.6f°N, %.6f°E\n", data->latitude, data->longitude);
    ESP_LOGI(TAG, "高度: %.1f米 (Geoid: %.1f米)\n", data->altitude, data->geoid_separation);
    ESP_LOGI(TAG, "速度: %.2f m/s | 方向: %.1f°\n", data->speed, data->course);
    ESP_LOGI(TAG, "卫星: 使用%d颗 / 可见%d颗 | HDOP: %.1f\n", 
           data->satellite_count, data->satellite_total, data->hdop);
    ESP_LOGI(TAG, "日期: %.*s-%.*s-20%.*s\n", 
           2, data->date, 2, data->date + 2, 2, data->date + 4);
    ESP_LOGI(TAG, "定位模式: %s | 天线状态: %s\n", 
           get_mode_name(data->position_mode), 
           get_antenna_status(data->antenna_status));
    ESP_LOGI(TAG, "数据质量: %s\n", data->is_valid ? "有效" : "无效");
    ESP_LOGI(TAG, "===================================\n");
}

// // 示例：处理实际数据流
// int main() {
//     // 实际NMEA数据流
//     const char* nmea_data[] = {
//         "$GNGLL,2242.19882,N,11357.74954,E,135345.000,A,A*44",
//         "$GNGSA,A,3,16,26,28,,,,,,,,,,2.3,1.2,2.0,1 * 38",
//         "$GNGSA,A,3,08,09,13,19,20,36,38,,,,,,2.3,1.2,2.0,4 * 33",
//         "$GPGSV,2,1,08,10,05,170,,16,52,235,30,26,74,319,26,27,15,186,13,0 * 6F",
//         "$GPGSV,2,2,08,28,39,044,10,29,19,058,,31,49,004,,32,43,126,,0 * 6A",
//         "$BDGSV,2,1,08,08,35,205,18,09,48,324,27,13,40,218,28,19,45,310,28,0 * 7C",
//         "$BDGSV,2,2,08,20,70,206,31,36,23,264,30,37,,,20,38,27,192,18,0 * 43",
//         "$GNRMC,135345.000,A,2242.19882,N,11357.74954,E,0.74,238.78,140725,,,A,V*09",
//         "$GNVTG,238.78,T,,M,0.74,N,1.38,K,A*2C",
//         "$GNZDA,135345.000,14,07,2025,00,00 * 4A",
//         "$GPTXT,01,01,01,ANTENNA OPEN*25",
//         "$GNGGA,135346.000,2242.19881,N,11357.74964,E,1,10,1.2,121.8,M,-3.7,M,,*53"
//     };
    
//     GNSS_Data data = {0};
//     data.is_valid = 0; // 初始为无效
//     data.position_mode = MODE_UNKNOWN;
//     data.antenna_status = ANTENNA_UNKNOWN;
//     data.hdop = 99.9;
    
//     int data_count = sizeof(nmea_data) / sizeof(nmea_data[0]);
    
//     for (int i = 0; i < data_count; i++) {
//         parse_nmea_sentence(nmea_data[i], &data);
//     }
    
//     // 最终打印解析结果
//     print_gnss_data(&data);
    
//     // 特别处理天线开路情况
//     if (data.antenna_status == ANTENNA_OPEN) {
//         // printf("\n警告：检测到天线开路！\n");
//         // printf("请进行以下检查：\n");
//         // printf("1. 确保GPS天线正确连接\n");
//         // printf("2. 检查天线端口是否有损坏\n");
//         // printf("3. 尝试重启GNSS模块\n");
//     }
    
//     return 0;
// }

GNSS_Data GetGpsInfo(void)
{
    return current_data;
}


void Deal_Data(unsigned char *data, unsigned int len)
{
    char lines[20][200]; // 存储提取的行
    unsigned char line_count = 0;
    char *line = strtok((char *)data, "\r\n");
    while (line != NULL && line_count < 50) {
        strncpy((char *)lines[line_count], (const char *)line, strlen(line));
        lines[line_count][strlen(line)+1] = 0;
        line_count++;
        line = strtok(NULL, "\r\n");
    }
    // for (int i = 0; i < line_count; i++) {
    //     ESP_LOGI(TAG, "[%02d] %s\n", i + 1, lines[i]);
    // }    
    // 去掉头和尾部
    
    for(int i = 1; i < line_count - 1; i++) {
        // char *line = strtok((char *)lines[i], "$");
        if(strstr(lines[i], "$GNGGA")) {
            // ESP_LOGI(TAG, "[%02d] %s\n", i + 1, lines[i]);
            // parse_GGA(lines[i], (const char *)&current_data);
        }
        else if(strstr(lines[i], "$GNRMC")) {
            ESP_LOGI(TAG, "[%02d] %s\n", i + 1, lines[i]);
            parse_RMC(&current_data, (const char *)lines[i]);
        }
    }

    ESP_LOGI(TAG, "latitude %lf\n", current_data.latitude);
    ESP_LOGI(TAG, "longitude %lf\n", current_data.longitude);
    ESP_LOGI(TAG, "speed %lf\n", current_data.speed);
    ESP_LOGI(TAG, "course %lf\n", current_data.course);
    ESP_LOGI(TAG, "date %s\n", current_data.date);
    ESP_LOGI(TAG, "timestamp %s\n", current_data.timestamp);
    
}

static void app_gps(void *arg)
{
    unsigned int len = 0;
    gps_init();
    while(1) 
    {
        ESP_LOGI(TAG, "gps run");
        memset(read_buf, 0, GPS_BUF_SIZE);
        len = GpsReadData(read_buf);
        if( len ) {
            // ESP_LOGI(TAG, "len=%d %s", len, read_buf);

            Deal_Data(read_buf, len);
            // parse_nmea_sentence((const char *)read_buf, &current_data);
            // ESP_LOGI(TAG, "位置: %.6f°N, %.6f°W\n", 
            //     current_data.latitude,
            //     current_data.longitude);
            // ESP_LOGI(TAG, "速度: %.2f m/s\n", current_data.speed);
            // ESP_LOGI(TAG, "方向: %.1f°\n", current_data.course);
            // ESP_LOGI(TAG, "时间: %s UTC 日期: %s\n", current_data.timestamp, current_data.date);        
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_gps_start(void)
{
    xTaskCreate(app_gps, "app_gps", 10240, NULL, 10, NULL);
}