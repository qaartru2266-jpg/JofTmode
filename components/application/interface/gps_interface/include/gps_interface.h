#ifndef __GPS_INTERFACE_H__
#define __GPS_INTERFACE_H__

#define GPS_BUF_SIZE  1024

void gps_init(void);

unsigned int GpsReadData(unsigned char *r_data);

unsigned int GpsSendData(const char* logName, const char* data, const int len);

#endif