#include "GPS.hpp"
#include <cstring>
#include <cstdlib>

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

static bool nmea_assign_coord(double &dest, const char *s, char hemi) {
    if (s[0] != '\0') {
        dest = nmea_coord(s, hemi);
        return true;
    }
    return false;
}

static inline float fopt(const char *s) { return s[0] ? (float)atof(s) : NAN; }

void GPS::ublox_parse(char *line, struct gps_data &data) {
    if (strlen(line) < 6) return;
    const char *type = line + 3;

    if      (strncmp(type, "GGA", 3) == 0) ublox_parse_GGA(line, data);
    else if (strncmp(type, "RMC", 3) == 0) ublox_parse_RMC(line, data);
    else if (strncmp(type, "GNS", 3) == 0) ublox_parse_GNS(line, data);
    else if (strncmp(type, "GLL", 3) == 0) ublox_parse_GLL(line, data);
    else if (strncmp(type, "GST", 3) == 0) parse_GST(line, data);
    else if (strncmp(type, "GSA", 3) == 0) parse_GSA(line, data);
    else if (!strncmp(type, "VTG", 3)) parse_VTG(line, data);
}

void GPS::parse_GNS(char *line, gps_data &d) {
    char *f[16];
    int n = nmea_split(line, f, 16);
    if (n < 10) return;
    strncpy(d.time, f[1], DATA_SIZE - 1); d.time[DATA_SIZE - 1] = '\0';
    d.ns = f[3][0]; d.ew = f[5][0];
    bool got = false;
    got |= nmea_assign_coord(d.latitude,  f[2], d.ns);
    got |= nmea_assign_coord(d.longitude, f[4], d.ew);
    strncpy(d.pos_mode, f[6], sizeof d.pos_mode - 1);
    d.pos_mode[sizeof d.pos_mode - 1] = '\0';
    d.sats     = (uint8_t)atoi(f[7]);
    d.hdop     = fopt(f[8]);
    d.altitude = fopt(f[9]);
    if (n > 10) d.geoid_sep    = fopt(f[10]);
    if (n > 11) d.diff_age     = fopt(f[11]);
    if (n > 12) d.diff_station = atoi(f[12]);
    if (n > 13) d.nav_status   = f[13][0];
    if (got) d.data_ready = true;
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

// $GPGLL,lat,N/S,lon,E/W,time,status*cs
//   0     1   2   3   4    5     6
void GPS::ublox_parse_GLL(char *line, struct gps_data &data) {
    char *f[8];
    if (nmea_split(line, f, 8) < 6) return;
    data.ns = f[2][0]; 
    data.ew = f[4][0];
    bool got = false;
    got |= nmea_assign_coord(data.latitude,  f[1], data.ns);
    got |= nmea_assign_coord(data.longitude, f[3], data.ew);
    strncpy(data.time, f[5], DATA_SIZE - 1); 
    data.time[DATA_SIZE - 1] = '\0';
    if (got) data.data_ready = true;
}


// $GPGNS,time,lat,N/S,lon,E/W,mode,numSV,HDOP,alt,sep,diffAge,diffStation*cs
//   0      1    2   3   4   5    6    7    8    9   10    11       12
void GPS::ublox_parse_GNS(char *line, struct gps_data &data) {
    char *f[16];
    if (nmea_split(line, f, 16) < 10) return;
    strncpy(data.time, f[1], DATA_SIZE - 1);
    data.time[DATA_SIZE - 1] = '\0';
    data.ns = f[3][0]; data.ew = f[5][0];
    bool got = false;
    got |= nmea_assign_coord(data.latitude,  f[2], data.ns);
    got |= nmea_assign_coord(data.longitude, f[4], data.ew);
    data.sats = (uint8_t)atoi(f[7]);
    data.hdop = fopt(f[8]);
    data.altitude = fopt(f[9]);
    if (got) data.data_ready = true;
}


// $GPRMC,time,status,lat,N/S,lon,E/W,speed,course,date,magvar,magdir*cs
//   0      1     2    3   4   5   6    7      8     9    10     11
void GPS::ublox_parse_RMC(char *line, struct gps_data &data) {
    char *f[12];
    if (nmea_split(line, f, 12) < 9) return;
    strncpy(data.time, f[1], DATA_SIZE - 1);
    data.time[DATA_SIZE - 1] = '\0';
    if (f[2][0] == 'A') {
        data.ns = f[4][0]; data.ew = f[6][0];
        bool got = false;
        got |= nmea_assign_coord(data.latitude,  f[3], data.ns);
        got |= nmea_assign_coord(data.longitude, f[5], data.ew);
        if (f[7][0]) { data.sog_knots = fopt(f[7]); data.sog_ms = data.sog_knots * 0.514444f; got = true; }
        if (f[8][0]) data.cog_true = fopt(f[8]);
        if (got) data.data_ready = true;
    }
}


// $GPGGA,time,lat,N/S,lon,E/W,fix,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*cs
//   0      1    2   3   4   5   6    7    8    9  10  11 12   13       14
void GPS::ublox_parse_GGA(char *line, struct gps_data &data) {
    char *f[15];
    if (nmea_split(line, f, 15) < 10) return;
    strncpy(data.time, f[1], DATA_SIZE - 1);
    data.time[DATA_SIZE - 1] = '\0';
    data.ns = f[3][0]; data.ew = f[5][0];
    bool got = false;
    got |= nmea_assign_coord(data.latitude,  f[2], data.ns);
    got |= nmea_assign_coord(data.longitude, f[4], data.ew);
    data.sats = (uint8_t)atoi(f[7]);
    data.hdop = fopt(f[8]);
    data.altitude = fopt(f[9]);
    if (got) data.data_ready = true;
}

// $xxVTG,cogTrue,T,cogMag,M,sogKnots,N,sogKmh,K,posMode
void GPS::parse_VTG(char *line, gps_data &d) {
    char *f[12];
    int n = nmea_split(line, f, 12);
    if (n < 8) return;
    bool got = false;
    char *f[12];
    if (nmea_split(line, f, 12) < 8) return;
    if (f[1][0]) d.cog_true  = fopt(f[1]);
    if (f[5][0]) d.sog_knots = fopt(f[5]);
    if (f[7][0]) { d.sog_kmh = fopt(f[7]); d.sog_ms = d.sog_kmh / 3.6f; d.data_ready = true; }
}