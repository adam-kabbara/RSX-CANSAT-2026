#ifndef GPS_H
#define GPS_H

#include "stm32g4xx_hal.h" 
#include "global_includes.hpp"

#ifdef __cplusplus

#define UBLOX_I2C_ADDR (0x42 << 1)
#define UBLOX_REG_DATA_STREAM  0xFF

class GPS {
    public:
        static double nmeaToDecimalDegrees(const char* token);
    
        void ublox_parse(char *line, struct gps_data &data);
        void ublox_parse_GGA(char *line, struct gps_data &data);
        void ublox_parse_RMC(char *line, struct gps_data &data);
        void ublox_parse_GNS(char *line, struct gps_data &data);
        void ublox_parse_GLL(char *line, struct gps_data &data);
    
        void parse_nmea(char *line, gps_data &d);
        void parse_GNS(char *line, gps_data &d);
        void parse_GST(char *line, gps_data &d);
        void parse_GSA(char *line, gps_data &d);
};

#endif /* __cplusplus */
#endif /* GPS_H */