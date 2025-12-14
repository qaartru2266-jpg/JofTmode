#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"

#include "app_gps_parser.h"

static const char *TAG = "gps_parser";

static double ddm_to_degrees(double ddm, char hemi)
{
    double abs_ddm = fabs(ddm);
    int degrees = (int)(abs_ddm / 100.0);
    double minutes = abs_ddm - (degrees * 100.0);
    double result = degrees + (minutes / 60.0);
    if (hemi == 'S' || hemi == 'W') {
        result = -result;
    }
    return result;
}

static SatelliteSystem parse_system_id(const char *sentence)
{
    if (!sentence || strlen(sentence) < 3) {
        return SYS_UNKNOWN;
    }
    char prefix[3] = {0};
    strncpy(prefix, sentence + 1, 2);

    if (strcmp(prefix, "GP") == 0) return SYS_GPS;
    if (strcmp(prefix, "GL") == 0) return SYS_GLONASS;
    if (strcmp(prefix, "BD") == 0) return SYS_BEIDOU;
    if (strcmp(prefix, "GA") == 0) return SYS_GALILEO;
    if (strcmp(prefix, "GN") == 0) return SYS_GNSS;
    return SYS_UNKNOWN;
}

static bool validate_checksum(const char *sentence)
{
    const char *asterisk = strchr(sentence, '*');
    if (!asterisk) return false;

    unsigned char calculated = 0;
    for (const char *p = sentence + 1; p < asterisk; ++p) {
        calculated ^= (unsigned char)(*p);
    }

    unsigned char received = (unsigned char)strtol(asterisk + 1, NULL, 16);
    return calculated == received;
}

static PositionMode parse_position_mode(char mode_char)
{
    switch (mode_char) {
        case 'A': return MODE_AUTONOMOUS;
        case 'D': return MODE_DIFFERENTIAL;
        case 'V': return MODE_INVALID;
        case 'R': return MODE_VALIDATED;
        default:  return MODE_UNKNOWN;
    }
}

static void parse_antenna_status(GNSS_Data *data, const char *sentence)
{
    char *antenna_pos = strstr((char *)sentence, "ANTENNA ");
    if (!antenna_pos || !data) return;

    antenna_pos += 8;
    if (strncmp(antenna_pos, "OPEN", 4) == 0) {
        data->antenna_status = ANTENNA_OPEN;
        data->is_valid = 0;
    } else if (strncmp(antenna_pos, "SHORT", 5) == 0) {
        data->antenna_status = ANTENNA_SHORT;
        data->is_valid = 0;
    } else if (strncmp(antenna_pos, "OK", 2) == 0) {
        data->antenna_status = ANTENNA_OK;
    }
}

static void copy_token(char *dst, size_t dst_sz, const char *src)
{
    if (!dst || dst_sz == 0) {
        return;
    }
    if (!src) {
        dst[0] = '\0';
        return;
    }
    strncpy(dst, src, dst_sz - 1);
    dst[dst_sz - 1] = '\0';
}

static void parse_GGA(gps_parser_t *parser, const char *sentence)
{
    char copy[128];
    strncpy(copy, sentence, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char *ctx = NULL;
    char *token = strtok_r(copy, ",", &ctx);
    int field = 0;
    double lat_ddm = 0.0;
    double lon_ddm = 0.0;
    char lat_hemi = 'N';
    char lon_hemi = 'E';

    while (token) {
        switch (field) {
            case 1:
                if (strlen(token) > 0) {
                    copy_token(parser->data.timestamp, sizeof(parser->data.timestamp), token);
                }
                break;
            case 2:
                if (token[0]) lat_ddm = atof(token);
                break;
            case 3:
                if (token[0]) lat_hemi = token[0];
                break;
            case 4:
                if (token[0]) lon_ddm = atof(token);
                break;
            case 5:
                if (token[0]) lon_hemi = token[0];
                break;
            case 7:
                parser->data.satellite_count = token[0] ? atoi(token) : 0;
                break;
            case 8:
                parser->data.hdop = token[0] ? atof(token) : parser->data.hdop;
                break;
            case 9:
                if (token[0]) parser->data.altitude = atof(token);
                break;
            case 11:
                if (token[0]) parser->data.geoid_separation = atof(token);
                break;
            default:
                break;
        }

        token = strtok_r(NULL, ",", &ctx);
        field++;
    }

    parser->data.latitude = ddm_to_degrees(lat_ddm, lat_hemi);
    parser->data.longitude = ddm_to_degrees(lon_ddm, lon_hemi);
}

static void parse_RMC(gps_parser_t *parser, const char *sentence)
{
    char copy[128];
    strncpy(copy, sentence, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char *ctx = NULL;
    char *token = strtok_r(copy, ",", &ctx);
    int field = 0;
    double lat_ddm = 0.0;
    double lon_ddm = 0.0;
    char lat_hemi = 'N';
    char lon_hemi = 'E';
    char mode_char = 'N';

    while (token) {
        switch (field) {
            case 1:
                if (strlen(token) > 0) {
                    copy_token(parser->data.timestamp, sizeof(parser->data.timestamp), token);
                }
                break;
            case 2:
                parser->data.is_valid = (token[0] == 'A') ? 1 : 0;
                break;
            case 3:
                if (token[0]) lat_ddm = atof(token);
                break;
            case 4:
                if (token[0]) lat_hemi = token[0];
                break;
            case 5:
                if (token[0]) lon_ddm = atof(token);
                break;
            case 6:
                if (token[0]) lon_hemi = token[0];
                break;
            case 7:
                if (token[0] && parser->data.is_valid) parser->data.speed = atof(token) * 0.5144f;
                break;
            case 8:
                if (token[0] && parser->data.is_valid) parser->data.course = atof(token);
                break;
            case 9:
                if (token[0]) copy_token(parser->data.date, sizeof(parser->data.date), token);
                break;
            case 12:
                if (token[0]) mode_char = token[0];
                break;
            default:
                break;
        }
        token = strtok_r(NULL, ",", &ctx);
        field++;
    }

    parser->data.position_mode = parse_position_mode(mode_char);
    parser->data.latitude = ddm_to_degrees(lat_ddm, lat_hemi);
    parser->data.longitude = ddm_to_degrees(lon_ddm, lon_hemi);
    if (parser->data.position_mode == MODE_INVALID) {
        parser->data.is_valid = 0;
    }
}

static void parse_VTG(gps_parser_t *parser, const char *sentence)
{
    char copy[128];
    strncpy(copy, sentence, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char *ctx = NULL;
    char *token = strtok_r(copy, ",", &ctx);
    int field = 0;

    while (token && parser->data.is_valid) {
        switch (field) {
            case 1:
                if (token[0]) parser->data.course = atof(token);
                break;
            case 5:
                if (token[0]) parser->data.speed = atof(token) * 0.5144f;
                break;
            case 7:
                if (token[0]) parser->data.speed = atof(token) / 3.6f;
                break;
            default:
                break;
        }
        token = strtok_r(NULL, ",", &ctx);
        field++;
    }
    parser->data.date[6] = '\0';
}

static void parse_GSA(gps_parser_t *parser, const char *sentence)
{
    char copy[196];
    strncpy(copy, sentence, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char *ctx = NULL;
    char *token = strtok_r(copy, ",", &ctx);
    int field = 0;
    int sat_count = 0;

    while (token) {
        if (field >= 3 && field <= 14) {
            if (token[0] && atoi(token) > 0) {
                sat_count++;
            }
        } else if (field == 16) {
            if (token[0]) parser->data.hdop = atof(token);
        }
        token = strtok_r(NULL, ",", &ctx);
        field++;
    }

    if (sat_count > parser->data.satellite_count) {
        parser->data.satellite_count = sat_count;
    }
}

static void parse_GSV(gps_parser_t *parser, const char *sentence)
{
    char copy[196];
    strncpy(copy, sentence, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    SatelliteSystem sys = parse_system_id(sentence);
    if (!(sys == SYS_GPS || sys == SYS_BEIDOU || sys == SYS_GNSS)) {
        return;
    }

    char *ctx = NULL;
    char *token = strtok_r(copy, ",", &ctx);
    int field = 0;

    while (token) {
        if (field == 3 && token[0]) {
            int total_sats = atoi(token);
            if (total_sats > parser->data.satellite_total) {
                parser->data.satellite_total = total_sats;
            }
        }
        token = strtok_r(NULL, ",", &ctx);
        field++;
    }
}

static void parse_ZDA(gps_parser_t *parser, const char *sentence)
{
    char copy[128];
    strncpy(copy, sentence, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char *ctx = NULL;
    char *token = strtok_r(copy, ",", &ctx);
    int field = 0;

    while (token) {
        switch (field) {
            case 1:
                if (strlen(token) > 0) {
                    copy_token(parser->data.timestamp, sizeof(parser->data.timestamp), token);
                }
                break;
            case 2:
                if (strlen(token) == 2) {
                    memcpy(parser->data.date, token, 2);
                }
                break;
            case 3:
                if (strlen(token) == 2) {
                    memcpy(parser->data.date + 2, token, 2);
                }
                break;
            case 4:
                if (strlen(token) == 4) {
                    memcpy(parser->data.date + 4, token + 2, 2);
                }
                break;
            default:
                break;
        }
        token = strtok_r(NULL, ",", &ctx);
        field++;
    }
}

void gps_parser_init(gps_parser_t *parser)
{
    if (!parser) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->data.hdop = 99.9f;
    parser->data.position_mode = MODE_UNKNOWN;
    parser->data.antenna_status = ANTENNA_UNKNOWN;
    parser->data.system = SYS_UNKNOWN;
}

bool gps_parser_handle_sentence(gps_parser_t *parser, const char *sentence, GNSS_Data *out_data)
{
    if (!parser || !sentence) {
        return false;
    }
    size_t len = strlen(sentence);
    if (len < 7 || sentence[0] != '$') {
        return false;
    }

    if (!validate_checksum(sentence)) {
        ESP_LOGW(TAG, "checksum failed: %s", sentence);
        return false;
    }

    parser->data.system = parse_system_id(sentence);

    bool updated = false;
    if (strstr(sentence, "$GNGGA") || strstr(sentence, "$GPGGA")) {
        parse_GGA(parser, sentence);
        updated = true;
    } else if (strstr(sentence, "$GNRMC") || strstr(sentence, "$GPRMC")) {
        parse_RMC(parser, sentence);
        updated = true;
    } else if (strstr(sentence, "$GNVTG") || strstr(sentence, "$GPVTG")) {
        parse_VTG(parser, sentence);
        updated = true;
    } else if (strstr(sentence, "$GNGSA") || strstr(sentence, "$GPGSA") || strstr(sentence, "$BDGSA")) {
        parse_GSA(parser, sentence);
        updated = true;
    } else if (strstr(sentence, "$GPGSV") || strstr(sentence, "$BDGSV") || strstr(sentence, "$GLGSV")) {
        parse_GSV(parser, sentence);
        updated = true;
    } else if (strstr(sentence, "$GNZDA") || strstr(sentence, "$GPZDA")) {
        parse_ZDA(parser, sentence);
        updated = true;
    } else if (strstr(sentence, "$GPTXT")) {
        parse_antenna_status(&parser->data, sentence);
        updated = true;
    }

    if (updated && out_data) {
        *out_data = parser->data;
    }
    return updated;
}
