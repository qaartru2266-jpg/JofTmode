#ifndef APP_GPS_H
#define APP_GPS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ANTENNA_UNKNOWN,
    ANTENNA_OK,
    ANTENNA_SHORT,
    ANTENNA_OPEN
} AntennaStatus;

typedef enum {
    MODE_UNKNOWN,
    MODE_AUTONOMOUS,
    MODE_DIFFERENTIAL,
    MODE_INVALID,
    MODE_VALIDATED
} PositionMode;

typedef enum {
    SYS_UNKNOWN,
    SYS_GPS,
    SYS_GLONASS,
    SYS_BEIDOU,
    SYS_GALILEO,
    SYS_GNSS
} SatelliteSystem;

typedef struct {
    double latitude;
    double longitude;
    float altitude;
    float geoid_separation;
    float speed;
    float course;
    int satellite_count;
    int satellite_total;
    float hdop;
    char timestamp[10];
    char date[7];
    AntennaStatus antenna_status;
    PositionMode position_mode;
    char is_valid;
    SatelliteSystem system;
} GNSS_Data;

void app_gps_start(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_GPS_H */
