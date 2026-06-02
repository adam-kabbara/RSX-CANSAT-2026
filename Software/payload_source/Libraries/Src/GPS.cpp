#include "GPS.hpp"
#include <cstring>
#include <cstdlib>

// void GPS::ublox_parse_GNS(char *line, struct gps_data &data) {
//     char *token;
//     int field = 0;

//     token = strtok(line, ",");
//     while (token != NULL) {
//         switch (field) {
//             case 1:
//                 if (token[0] != '\0' && token[0] != '*') {
//                     strncpy(data.time, token, DATA_SIZE - 1);
//                     data.time[DATA_SIZE - 1] = '\0';
//                 }
//                 break;
//             case 2:
//                 if (token[0] != '\0') data.latitude = nmeaToDecimalDegrees(token);
//                 break;
//             case 3: // N/S
//                 if (token[0] == 'S') data.latitude = -data.latitude;
//                 break;
//             case 4:
//                 if (token[0] != '\0') data.longitude = nmeaToDecimalDegrees(token);
//                 break;
//             case 5: // E/W
//                 if (token[0] == 'W') data.longitude = -data.longitude;
//                 break;
//             case 7:
//                 if (token[0] != '\0') data.sats = std::atoi(token);
//                 break;
//             case 9:
//                 if (token[0] != '\0') data.altitude = std::atof(token);
//                 break;
//             default:
//                 break;
//         }
//         token = strtok(NULL, ",");
//         field++;
//     }
// }

double nmeaToDecimalDegrees(const char* token) {
    if (token == NULL || token[0] == '\0') return NAN;
    double raw = std::atof(token);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    return degrees + (minutes / 60.0);
}

// $GPGGA,time,lat,N/S,lon,E/W,fix,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*cs
//   0      1    2   3   4   5   6    7    8    9  10  11 12   13       14
void GPS::ublox_parse_GGA(char *line, struct gps_data &data) {
    char *token = strtok(line, ",");
    int field = 0;
    while (token != NULL) {
        switch (field) {
            case 1:
                if (token[0] != '\0') {
                    strncpy(data.time, token, DATA_SIZE - 1);
                    data.time[DATA_SIZE - 1] = '\0';
                }
                break;
            case 2:
                if (token[0] != '\0') data.latitude = nmeaToDecimalDegrees(token);
                break;
            case 3:
                if (token[0] == 'S') data.latitude = -data.latitude;
                break;
            case 4:
                if (token[0] != '\0') data.longitude = nmeaToDecimalDegrees(token);
                break;
            case 5:
                if (token[0] == 'W') data.longitude = -data.longitude;
                break;
            case 7:
                if (token[0] != '\0') data.sats = std::atoi(token);
                break;
            default: break;
        }
        token = strtok(NULL, ",");
        field++;
    }
}

// $GPRMC,time,status,lat,N/S,lon,E/W,speed,course,date,magvar,magdir*cs
//   0      1     2    3   4   5   6    7      8     9    10     11
void GPS::ublox_parse_RMC(char *line, struct gps_data &data) {
    char *token = strtok(line, ",");
    int field = 0;
    // RMC has no sats field — only update time and lat/lon
    while (token != NULL) {
        switch (field) {
            case 1:
                if (token[0] != '\0') {
                    strncpy(data.time, token, DATA_SIZE - 1);
                    data.time[DATA_SIZE - 1] = '\0';
                }
                break;
            case 2:
                // status A=active/valid, V=void — skip position update if void
                if (token[0] == 'V') return;
                break;
            case 3:
                if (token[0] != '\0') data.latitude = nmeaToDecimalDegrees(token);
                break;
            case 4:
                if (token[0] == 'S') data.latitude = -data.latitude;
                break;
            case 5:
                if (token[0] != '\0') data.longitude = nmeaToDecimalDegrees(token);
                break;
            case 6:
                if (token[0] == 'W') data.longitude = -data.longitude;
                break;
            default: break;
        }
        token = strtok(NULL, ",");
        field++;
    }
}

// $GPGNS,time,lat,N/S,lon,E/W,mode,numSV,HDOP,alt,sep,diffAge,diffStation*cs
//   0      1    2   3   4   5    6    7    8    9   10    11       12
void GPS::ublox_parse_GNS(char *line, struct gps_data &data) {
    char *token = strtok(line, ",");
    int field = 0;
    while (token != NULL) {
        switch (field) {
            case 1:
                if (token[0] != '\0') {
                    strncpy(data.time, token, DATA_SIZE - 1);
                    data.time[DATA_SIZE - 1] = '\0';
                }
                break;
            case 2:
                if (token[0] != '\0') data.latitude = nmeaToDecimalDegrees(token);
                break;
            case 3:
                if (token[0] == 'S') data.latitude = -data.latitude;
                break;
            case 4:
                if (token[0] != '\0') data.longitude = nmeaToDecimalDegrees(token);
                break;
            case 5:
                if (token[0] == 'W') data.longitude = -data.longitude;
                break;
            case 7:
                if (token[0] != '\0') data.sats = std::atoi(token);
                break;
            default: break;
        }
        token = strtok(NULL, ",");
        field++;
    }
}

// $GPGLL,lat,N/S,lon,E/W,time,status*cs
//   0     1   2   3   4    5     6
void GPS::ublox_parse_GLL(char *line, struct gps_data &data) {
    char *token = strtok(line, ",");
    int field = 0;
    while (token != NULL) {
        switch (field) {
            case 1:
                if (token[0] != '\0') data.latitude = nmeaToDecimalDegrees(token);
                break;
            case 2:
                if (token[0] == 'S') data.latitude = -data.latitude;
                break;
            case 3:
                if (token[0] != '\0') data.longitude = nmeaToDecimalDegrees(token);
                break;
            case 4:
                if (token[0] == 'W') data.longitude = -data.longitude;
                break;
            case 5:
                if (token[0] != '\0') {
                    strncpy(data.time, token, DATA_SIZE - 1);
                    data.time[DATA_SIZE - 1] = '\0';
                }
                break;
            case 6:
                // status A=active/valid, V=void
                if (token[0] == 'V') return;
                break;
            default: break;
        }
        token = strtok(NULL, ",");
        field++;
    }
}

void GPS::ublox_parse(char *line, struct gps_data &data) {
    if (strlen(line) < 6) return;
    const char *type = line + 3;

    if      (strncmp(type, "GGA", 3) == 0) ublox_parse_GGA(line, data);
    else if (strncmp(type, "RMC", 3) == 0) ublox_parse_RMC(line, data);
    else if (strncmp(type, "GNS", 3) == 0) ublox_parse_GNS(line, data);
    else if (strncmp(type, "GLL", 3) == 0) ublox_parse_GLL(line, data);
    // GSA/GSV/ZDA omitted — not useful for your four fields
}