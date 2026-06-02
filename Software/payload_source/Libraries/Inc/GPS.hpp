#ifndef GPS_H
#define GPS_H

#include "global_includes.hpp"

#ifdef __cplusplus

#define UBLOX_I2C_ADDR (0x42 << 1)
#define UBLOX_REG_DATA_STREAM  0xFF

class GPS {
    public:
        static double nmeaToDecimalDegrees(const char* token);
        static void ublox_parse_GGA(char *line, struct gps_data &data);
        static void ublox_parse_GNS(char *line, struct gps_data &data);
        static void ublox_parse_RMC(char *line, struct gps_data &data);
        static void ublox_parse_GLL(char *line, struct gps_data &data);
        static void ublox_parse(char *line, struct gps_data &data);
        static void parse_GNS(char *line, struct gps_data &data);
        static void parse_GST(char *line, struct gps_data &data);
        static void parse_GSA(char *line, struct gps_data &data);
        static void parse_VTG(char *line, struct gps_data &data);
};

#endif /* __cplusplus */
#endif /* GPS_H */