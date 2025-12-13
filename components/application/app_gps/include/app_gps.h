#ifndef __APP_GPS_H__
#define __APP_GPS_H__

// 定义天线状态枚举
typedef enum {
    ANTENNA_UNKNOWN,
    ANTENNA_OK,
    ANTENNA_SHORT,
    ANTENNA_OPEN
} AntennaStatus;

// 定义定位模式
typedef enum {
    MODE_UNKNOWN,
    MODE_AUTONOMOUS,   // 自主定位
    MODE_DIFFERENTIAL, // 差分定位
    MODE_INVALID,      // 无效定位
    MODE_VALIDATED     // 差分验证
} PositionMode;

// 卫星系统枚举
typedef enum {
    SYS_UNKNOWN,
    SYS_GPS,
    SYS_GLONASS,
    SYS_BEIDOU,
    SYS_GALILEO,
    SYS_GNSS
} SatelliteSystem;

// GNSS数据结构体
typedef struct {
    double latitude;       // 纬度 (度)
    double longitude;      // 经度 (度)
    float altitude;        // 海拔高度 (米)
    float geoid_separation;// 大地水准面高度 (米)
    float speed;           // 地面速度 (m/s)
    float course;          // 航向/方向 (度)
    int satellite_count;   // 使用卫星数量
    int satellite_total;   // 可见卫星总数
    float hdop;            // 水平精度因子
    char timestamp[10];    // UTC时间 (HHMMSS.sss)
    char date[7];          // 日期 (DDMMYY)
    AntennaStatus antenna_status;  // 天线状态
    PositionMode position_mode;     // 定位模式
    char is_valid;          // 数据有效标志 (1=有效,0=无效)
    SatelliteSystem system; // 使用的卫星系统
} GNSS_Data;

extern GNSS_Data current_data;

// 卫星信息结构
typedef struct {
    int prn;        // 伪随机码
    int elevation;  // 仰角 (度)
    int azimuth;    // 方位角 (度)
    int snr;        // 信噪比
} SatelliteInfo;

#pragma once
#ifdef __cplusplus
extern "C" {
#endif

void app_gps_start(void);
#define App_Gps_Task_Start app_gps_start

#ifdef __cplusplus
}
#endif

void App_Gps_Task_Start(void);

GNSS_Data GetGpsInfo(void);

#endif
