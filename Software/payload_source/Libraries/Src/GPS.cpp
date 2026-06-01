#include "GPS.hpp"
#include <cstring>
#include <cstdlib>

void GPS::ublox_parse_GNS(char *line, struct gps_data &data) {
    char *token;
    int field = 0;

    token = strtok(line, ",");
    while (token != NULL) {
        switch (field) {
            case 1: // Time (hhmmss.ss)
                strncpy(data.time, token, DATA_SIZE - 1);
                data.time[DATA_SIZE - 1] = '\0';
                break;
            case 2: // Latitude
                data.latitude = std::atof(token);
                break;
            case 4: // Longitude
                data.longitude = std::atof(token);
                break;
            case 7: // numSV (Number of satellites used for fix)
                data.sats = std::atoi(token);
                break;
            case 9: // Altitude (Above mean sea level)
                data.altitude = std::atof(token);
                break;
            default:
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }
}