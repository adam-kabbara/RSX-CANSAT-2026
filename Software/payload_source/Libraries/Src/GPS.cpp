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

double GPS::nmeaToDecimalDegrees(const char* token) {
    if (token == NULL || token[0] == '\0') return NAN;
    double raw = std::atof(token);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    return degrees + (minutes / 60.0);
}

static int nmea_split(char *line, char **f, int maxf) {
    int n = 0;
    f[n++] = line;
    for (char *p = line; *p && n < maxf; ++p) {
        if (*p == ',') { *p = '\0'; f[n++] = p + 1; }
    }
    return n;
}

static bool nmea_checksum_ok(char *line) {
    if (line[0] != '$') return false;
    char *star = strchr(line, '*');
    if (!star) return false;
    uint8_t cs = 0;
    for (char *p = line + 1; p < star; ++p) cs ^= (uint8_t)*p;
    uint8_t given = (uint8_t)strtol(star + 1, nullptr, 16);
    *star = '\0';
    return cs == given;
}

static double nmea_coord(const char *s, char hemi) {
    if (s[0] == '\0') return NAN;
    double raw = atof(s);
    double deg = floor(raw / 100.0);          // works for 2- or 3-digit degrees
    double dec = deg + (raw - deg * 100.0) / 60.0;
    return (hemi == 'S' || hemi == 'W') ? -dec : dec;
}

static inline float fopt(const char *s) { return s[0] ? (float)atof(s) : NAN; }

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

void GPS::parse_GNS(char *line, gps_data &d) {
    char *f[16];
    int n = nmea_split(line, f, 16);
    if (n < 10) return;
    strncpy(d.time, f[1], DATA_SIZE - 1); d.time[DATA_SIZE - 1] = '\0';
    d.ns = f[3][0]; d.ew = f[5][0];
    d.latitude  = nmea_coord(f[2], d.ns);
    d.longitude = nmea_coord(f[4], d.ew);
    strncpy(d.pos_mode, f[6], sizeof d.pos_mode - 1);
    d.pos_mode[sizeof d.pos_mode - 1] = '\0';
    d.sats     = (uint8_t)atoi(f[7]);
    d.hdop     = fopt(f[8]);
    d.altitude = fopt(f[9]);
    if (n > 10) d.geoid_sep    = fopt(f[10]);
    if (n > 11) d.diff_age     = fopt(f[11]);
    if (n > 12) d.diff_station = atoi(f[12]);
    if (n > 13) d.nav_status   = f[13][0];
}

// $xxGST,time,rms,stdMaj,stdMin,orient,stdLat,stdLon,stdAlt
void GPS::parse_GST(char *line, gps_data &d) {
    char *f[12];
    if (nmea_split(line, f, 12) < 9) return;
    d.rms_range = fopt(f[2]);
    d.std_major = fopt(f[3]);
    d.std_minor = fopt(f[4]);
    d.orient    = fopt(f[5]);
    d.std_lat   = fopt(f[6]);
    d.std_lon   = fopt(f[7]);
    d.std_alt   = fopt(f[8]);
}

// $xxGSA,opMode,fixType,sv1..sv12,PDOP,HDOP,VDOP[,sysId]
void GPS::parse_GSA(char *line, gps_data &d) {
    char *f[24];
    if (nmea_split(line, f, 24) < 18) return;
    d.fix_type = f[2][0];
    d.pdop = fopt(f[15]);
    d.hdop = fopt(f[16]);   // overrides GNS hdop; same value
    d.vdop = fopt(f[17]);
}

void GPS::parse_nmea(char *line, gps_data &d) {
    if (line[1] == 'P') return;          // skip proprietary $P... (different layout)
    if (!nmea_checksum_ok(line)) return; // also strips the checksum
    const char *type = line + 3;         // "$" + 2-char talker, then 3-char type
    if      (!strncmp(type, "GNS", 3)) parse_GNS(line, d);
    else if (!strncmp(type, "GST", 3)) parse_GST(line, d);
    else if (!strncmp(type, "GSA", 3)) parse_GSA(line, d);
}