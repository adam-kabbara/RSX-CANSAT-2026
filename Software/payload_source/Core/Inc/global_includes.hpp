/**
******************************************************************************
* @file           : global_includes.h
* @author         : RSX 2025-2026
* @brief          : Adds general libraries and definitions for the whole project
******************************************************************************
*/

#ifndef INC_GLOBAL_INCLUDES_HPP_
#define INC_GLOBAL_INCLUDES_HPP_

#include <unordered_map>
#include <functional>
#include <cstring>
#include <stdarg.h>
#include <string>
#include <stdio.h>
#include <stdbool.h>
#include <cmath>
#include <algorithm>
#include <cstdint>

#define CMD_BUFF_SIZE 128
#define RESP_SIZE 128
#define SEA_LEVEL_PRESSURE_HPA 1013.25
#define SEA_LEVEL_PRESSURE_PA 101325.0
#define WORD_SIZE 64
#define DATA_SIZE 32
#define SENTENCE_SIZE 128
#define DATA_BUFF_SIZE 512
#define TEAM_ID 1011
#define SENSOR_SAMPLE_RATE_HZ 20
#define MAX_LOG_FILE_SIZE_BYTES 125000
#define ALTITUDE_SMOOTHING_WINDOW 5
#define ASCENT_ALT_THRESHOLD_M 8
#define DESCENT_FALL_THRESHOLD_M 2
#define EGG_ALT_THRESHOLD_M 4
#define EGG_TIMING_BUDGET_MS 100
#define TOF_SMOOTHING_WINOW 5
#define LANDED_THRESHOLD_M 2
#define MAX_EXP_CURRENT_A 2.5

enum SimModeStatus {
	SIM_OFF = 0,
	SIM_EN = 1,
	SIM_ON = 2
};

enum OperatingState {
	LAUNCH_PAD = 0,
	ASCENT = 1,
	APOGEE = 2,
	DESCENT = 3,
	PROBE_RELEASE = 4,
	PAYLOAD_RELEASE = 5,
	LANDED = 6,
	IDLE = 7
};

enum OperatingMode {
	OPMODE_FLIGHT = 0,
	OPMODE_SIM = 1
};

enum cam_status {
	CAM1_ON_CAM2_ON = 0,
	CAM1_ON_CAM2_OFF = 1,
	CAM1_OFF_CAM2_ON = 2,
	CAM1_OFF_CAM2_OFF = 3
};

enum FlightCtrl {
	AUTONOMOUS = 0,
	MANUAL = 1
};

struct rpy_data {
	float gyro_r;
	float gyro_p;
	float gyro_y;
	float accel_r;
	float accel_p;
	float accel_y;
};

struct bar_data {
	float pressure;
	float altitude;
};

struct gps_data {
	char    time[DATA_SIZE];   // UTC, "hhmmss.ss"
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

struct recovery_data {
	float launch_altitude;
	OperatingState state;
	OperatingMode mode;
	int packet_count;
	float max_alt;
	bool nosecone_flag;
	bool probe_flag;
	bool wing_flag;
	bool egg_flag;
};

inline const char* op_mode_to_string(OperatingMode mode, int full)
{
	if(full == 1)
	{
		if(mode == OPMODE_FLIGHT)
		{
			return "FLIGHT";
		}
		else
		{
			return "SIM";
		}
	}
	else
	{
		if(mode == OPMODE_FLIGHT)
		{
			return "F";
		}
		else
		{
			return "S";
		}
	}
}

inline const char* op_state_to_string(OperatingState state)
{
	static const char* states[] = {
		"LAUNCH_PAD",
		"ASCENT",
		"APOGEE",
		"DESCENT",
		"PROBE_RELEASE",
		"PAYLOAD_RELEASE",
		"LANDED",
		"IDLE"
	};

	return states[state];
}

inline const char* flight_ctrl_to_string(FlightCtrl ctrl)
{
	if(ctrl == AUTONOMOUS)
	{
		return "AUTONOMOUS";
	}

	return "MANUAL";
}

inline const float pressure_to_alt(const float pressure)
{
	return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE_PA, 0.1903));
}

#endif /* INC_GLOBAL_INCLUDES_HPP_ */