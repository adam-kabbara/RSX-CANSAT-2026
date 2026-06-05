#ifndef GPS_H
#define GPS_H

#include "global_includes.hpp"
#include "serial_manager.hpp"
#include <cstring>
#include <cstdlib>

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
}
#endif

#define UBLOX_I2C_ADDR (0x42 << 1)
#define UBLOX_REG_DATA_STREAM  0xFF

class GPS {
private:
	struct gps_data {
		char    time[DATA_SIZE] = "000000.00";   // UTC, "hhmmss.ss"
		double  latitude;          // decimal degrees, +N / -S   (double, not float)
		double  longitude;         // decimal degrees, +E / -W
		char    ns, ew;            // raw hemisphere chars
		char    pos_mode[8];       // GNS field 6: one char per constellation (N/A/D/F/R...)
		uint8_t fix_quality  = 0;  // 0 = invalid, 1 = GPS fix, 2 = DGPS fix, etc. (GGA field 6)
		uint8_t sats;              // numSV
		float   hdop;
		float   altitude;          // m above MSL
		float   geoid_sep;         // m
		float   diff_age;          // s   (NAN if absent)
		int     diff_station;
		char    nav_status;        // NMEA 4.10 nav status

		// --- GST: error statistics, all metres ---
		float   rms_range;
		float   std_major, std_minor, orient;   // error ellipse: axes (m), orientation (deg)
		float   std_lat, std_lon, std_alt;       // per-axis 1-sigma error (m)

		// --- GSA ---
		float   pdop, vdop;
		char    fix_type;          // '1' none, '2' 2D, '3' 3D

		// VTG / RMC — horizontal velocity
		float cog_true  = 0.0f;   // course over ground, deg true (0..360)
		float sog_knots = 0.0f;   // speed over ground, knots
		float sog_kmh   = 0.0f;   // km/h
		float sog_ms    = 0.0f;   // m/s (derived, the one you'll probably use)

		bool data_ready = false; // set to true when lat, lon, velocity
	};
	I2C_HandleTypeDef *gps_hi2c;
	char gps_nmea_buffer[100];
	uint8_t gps_buf_idx; // counter tracking string length

	void GPS_configure_output(I2C_HandleTypeDef* i2c);
	bool ubx_send(I2C_HandleTypeDef* i2c, uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len);
	uint16_t valset_add_u1(uint8_t* p, uint16_t idx, uint32_t key, uint8_t val);

public:
	struct gps_data internal_gps_storage;
	void GPS_Init(I2C_HandleTypeDef *i2c);
	bool GPS_probe();
	void GPS_update(SerialManager &serial);
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

#endif /* GPS_H */
