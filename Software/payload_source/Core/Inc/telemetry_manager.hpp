/*
 * telemetry_manager.hpp
 *
 *  Header for telemetry manager
 */

#ifndef INC_TELEMETRY_MANAGER_HPP_
#define INC_TELEMETRY_MANAGER_HPP_

#include "global_includes.hpp"
#include "serial_manager.hpp"
#include "sensor_manager.hpp"
#include "mission_manager.hpp"

class telemetryManager
{
private:
	struct transmission_packet {
		int TEAM_ID_PCKT = 0;
		char MISSION_TIME[DATA_SIZE] = "";
		int PACKET_COUNT = 0;
		char MODE[2] = "";
		char STATE[DATA_SIZE] = "";
		float ALTITUDE = 0.0;
		float TEMPERATURE = 0.0;
		float PRESSURE = 0.0;
		float VOLTAGE = 0.0;
		float CURRENT = 0.0;
		int GYRO_R = 0;
		int GYRO_P = 0;
		int GYRO_Y = 0;
		int ACCEL_R = 0;
		int ACCEL_P = 0;
		int ACCEL_Y = 0;
		char GPS_TIME[DATA_SIZE] = "";
		float GPS_ALTITUDE = 0.0;
		float GPS_LATITUDE = 0.0;
		float GPS_LONGITUDE = 0.0;
		int GPS_SATS = 0;
		char CMD_ECHO[CMD_BUFF_SIZE] = "";
		int CAMERA_STATUS = 0;
	};

public:
	OperatingState updateState(OperatingState curr_state);
	const char* sampleSensors(SensorManager &sensors, SerialManager &serial, MissionManager &mission_info);
	const char* cmd_buff_to_echo();
	const float pressure_to_alt(const float pressure);
	void build_data_str(char *buff, size_t size);
};

#endif /* INC_TELEMETRY_MANAGER_HPP_ */
