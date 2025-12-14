#ifndef APP_GPS_PARSER_H
#define APP_GPS_PARSER_H

#include <stdbool.h>

#include "app_gps.h"

typedef struct {
    GNSS_Data data;
} gps_parser_t;

void gps_parser_init(gps_parser_t *parser);
bool gps_parser_handle_sentence(gps_parser_t *parser, const char *sentence, GNSS_Data *out_data);

#endif /* APP_GPS_PARSER_H */
