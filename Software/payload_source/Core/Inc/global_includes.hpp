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

#define CMD_BUFF_SIZE 128
#define RESP_SIZE 128
#define SEA_LEVEL_PRESSURE_HPA 1013.25
#define WORD_SIZE 64
#define DATA_SIZE 32
#define SENTENCE_SIZE 128
#define DATA_BUFF_SIZE 512
#define TEAM_ID 1011
#define SENSOR_SAMPLE_RATE_HZ 20
#define MAX_LOG_FILE_SIZE_BYTES 125000

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

struct rpy_data {
	int gyro_r;
	int gyro_p;
	int gyro_y;
	int accel_r;
	int accel_p;
	int accel_y;
};

struct bar_data {
	float pressure;
	float altitude;
};

struct gps_data {
	char[DATA_SIZE] time;
	float altitude;
	float latitude;
	float longitude;
	int sats;
};

struct recovery_data {
	float launch_altitude;
	OperatingState state;
	OperatingMode mode;
	int packet_count;
};

const char* op_mode_to_string(OperatingMode mode, int full)
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

const char* op_state_to_string(OperatingState state)
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

#endif /* INC_GLOBAL_INCLUDES_HPP_ */
