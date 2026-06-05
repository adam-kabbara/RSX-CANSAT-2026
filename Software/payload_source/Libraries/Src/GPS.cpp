#include "GPS.hpp"

void GPS::GPS_configure_output(I2C_HandleTypeDef* i2c)
{
	uint8_t p[64];
	uint16_t n = 0;
	p[n++] = 0x00;   // version (0 = no transaction)
	p[n++] = 0x01;   // layers: RAM  (re-send each boot)
	p[n++] = 0x00;   // reserved
	p[n++] = 0x00;

	// --- disable what we don't parse (value 0) ---
	n = valset_add_u1(p, n, 0x209100BA, 0); // CFG-MSGOUT-NMEA_ID_GGA_I2C
	n = valset_add_u1(p, n, 0x209100C9, 0); // GLL
	n = valset_add_u1(p, n, 0x209100BF, 0); // GSA
	n = valset_add_u1(p, n, 0x209100C4, 0); // GSV  <-- the big one
	n = valset_add_u1(p, n, 0x209100B0, 0); // VTG
	n = valset_add_u1(p, n, 0x209100D8, 0); // ZDA
	// --- enable what we parse (value 1 = every epoch) ---
	n = valset_add_u1(p, n, 0x209100B5, 1); // GNS  (lat/lon/alt/sats/time)
	n = valset_add_u1(p, n, 0x209100AB, 1); // RMC  (speed/course)
	n = valset_add_u1(p, n, 0x209100D3, 1); // GST  (accuracy/rms)

	ubx_send(i2c, 0x06, 0x8A, p, n);        // CFG-VALSET
}

bool GPS::ubx_send(I2C_HandleTypeDef* i2c, uint8_t cls, uint8_t id,
                     const uint8_t* payload, uint16_t len)
{
    uint8_t buf[64];
    buf[0] = 0xB5; buf[1] = 0x62;       // UBX sync chars
    buf[2] = cls;  buf[3] = id;
    buf[4] = len & 0xFF; buf[5] = (len >> 8) & 0xFF;
    for (uint16_t i = 0; i < len; i++) buf[6 + i] = payload[i];

    // Fletcher checksum over class..end of payload
    uint8_t a = 0, b = 0;
    for (uint16_t i = 2; i < 6 + len; i++) { a += buf[i]; b += a; }
    buf[6 + len] = a; buf[7 + len] = b;

    return HAL_I2C_Master_Transmit(i2c, UBLOX_I2C_ADDR, buf, 8 + len, 100) == HAL_OK;
}

uint16_t GPS::valset_add_u1(uint8_t* p, uint16_t idx, uint32_t key, uint8_t val)
{
    p[idx++] =  key        & 0xFF;
    p[idx++] = (key >>  8) & 0xFF;
    p[idx++] = (key >> 16) & 0xFF;
    p[idx++] = (key >> 24) & 0xFF;
    p[idx++] = val;
    return idx;
}

void GPS::GPS_Init(I2C_HandleTypeDef *i2c)
{
	gps_hi2c = i2c;
	gps_buf_idx = 0;
	internal_gps_storage = gps_data{};
	HAL_Delay(100);
	//GPS_configure_output(i2c);
}

bool GPS::GPS_probe() 
{
    if (gps_hi2c == nullptr) return false;

    uint8_t reg = 0xFD;
    uint8_t avail[2] = {0, 0};
    // Write register address, then read 2 bytes
    if (HAL_I2C_Master_Transmit(gps_hi2c, UBLOX_I2C_ADDR, &reg, 1, 5) != HAL_OK)
        return false;
    if (HAL_I2C_Master_Receive(gps_hi2c, UBLOX_I2C_ADDR, avail, 2, 5) != HAL_OK)
        return false;
    return true; // device responded
}

void GPS::GPS_update(SerialManager &serial)
{
    if (gps_hi2c == nullptr) return;

    // count: read 0xFD/0xFE with proper register addressing (repeated-start)
    uint8_t avail[2];
    if (HAL_I2C_Mem_Read(gps_hi2c, UBLOX_I2C_ADDR, 0xFD,
                         I2C_MEMADD_SIZE_8BIT, avail, 2, 5) != HAL_OK) return;

    uint16_t n = ((uint16_t)avail[0] << 8) | avail[1];
    if (n == 0 || n == 0xFFFF) return;          // empty / no data

    static const uint16_t MAX_PER_CALL = 64;
    if (n > MAX_PER_CALL) n = MAX_PER_CALL;

    // stream: read N bytes from 0xFF
    uint8_t chunk[MAX_PER_CALL];
    if (HAL_I2C_Mem_Read(gps_hi2c, UBLOX_I2C_ADDR, 0xFF,
                         I2C_MEMADD_SIZE_8BIT, chunk, n, 50) != HAL_OK) return;

    for (uint16_t i = 0; i < n; ++i) {
        uint8_t b = chunk[i];
        if (b == 0xFF) continue;
        if (b == '$') gps_buf_idx = 0;
        if (gps_buf_idx < sizeof(gps_nmea_buffer) - 1)
            gps_nmea_buffer[gps_buf_idx++] = (char)b;
        if (b == '\n') {
            gps_nmea_buffer[gps_buf_idx] = '\0';
            ublox_parse(gps_nmea_buffer, internal_gps_storage);
            gps_buf_idx = 0;
        }
    }
}

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
    if (f[1][0]) d.cog_true  = fopt(f[1]);
    if (f[5][0]) d.sog_knots = fopt(f[5]);
    if (f[7][0]) { d.sog_kmh = fopt(f[7]); d.sog_ms = d.sog_kmh / 3.6f; d.data_ready = true; }
}
