/*
 * telemetry_manager.cpp
 *
 *  Manages sending of telemetry packets
 */

#include "telemetry_manager.hpp"

OperatingState telemetryManager::updateState(OperatingState curr_state)
{
	switch(curr_state)
	{
		case LAUNCH_PAD: {

			break;
		}

		case ASCENT: {
			break;
		}

		case APOGEE: {
			break;
		}

		case DESCENT: {
			break;
		}

		case PROBE_RELEASE: {
			break;
		}

		case PAYLOAD_RELEASE: {

			break;
		}

	}

	return curr_state;
}

// TODO: Add faster sampling function
const char* telemetryManager::sampleSensors(SensorManager &sensors, SerialManager &serial, MissionManager &mission_info)
{
	struct transmission_packet packet;
	if(mission_info.op_mode == OPMODE_SIM)
	{
		packet.PRESSURE = mission_info.SIMP_DATA/1000.0;
	}
	else
	{
		packet.PRESSURE = sensors.getPressure();
	}

	packet.ALTITUDE = pressure_to_alt(packet.PRESSURE * 10.0) - mission_info.launch_altitude;

	OperatingState new_state = op_state_to_string(updateState(mission_info.op_state));
	strcpy(packet.STATE, new_state);

	packet.TEAM_ID_PCKT = TEAM_ID;

	strcpy(send_packet.MODE, op_mode_to_string(mission_info.op_mode, 0));

	packet.TEMPERATURE = sensors.getTemp();

	packet.VOLTAGE = sensors.getVoltage();

	packet.CURRENT = sensors.getCurrent();

	struct rpy_data gyro_accel_data = sensors.getIMUData();
	
	packet.GYRO_R = gyro_accel_data.gyro_r;
	packet.GYRO_P = gyro_accel_data.gyro_p;
	packet.GYRO_Y = gyro_accel_data.gyro_y;

	packet.ACCEL_R = gyro_accel_data.accel_r;
	packet.ACCEL_P = gyro_accel_data.accel_p;
	packet.ACCEL_Y = gyro_accel_data.accel_y;

	struct gps_data gps_data_vals = sensors.getGPSData();

	packet.GPS_TIME = gps_data_vals.time;
	packet.GPS_ALTITUDE = gps_data_vals.altitude;
	packet.GPS_LATITUDE = gps_data_vals.latitude;
	packet.GPS_LONGITUDE = gps_data_vals.longitude;
	packet.GPS_SATS = gps_data_vals.sats;

	packet.CMD_ECHO = mission_info.getLastCommand();

	char send_buffer[DATA_BUFF_SIZE];
	build_data_str(send_buffer, DATA_BUFF_SIZE);

	serial.sendTelemetry(send_buffer);

	if(!disable_logfile && !sensors.EEPROM_addLogLine(send_buffer))
	{
		serial.sendErrorMsg("Warning: Unable to add line to logfile!");
		disable_logfile = True;
	}
}

void TelemetryManager::build_data_str(char *buff, size_t size)
{
    snprintf(buff, size,
        "%d,%s,%d,%s,%s,"
        "%.1f,%.1f,%.1f,%.1f,%d,"
        "%d,%d,%d,%d,%d,"
        "%.1f,%.1f,%.1f,%.1f,%s,"
        "%.1f,%.4f,%.4f,%d,%s,"
        "%d",
        send_packet.TEAM_ID_PCKT, 
        send_packet.MISSION_TIME, 
        send_packet.PACKET_COUNT, 
        send_packet.MODE, 
        send_packet.STATE,
        send_packet.ALTITUDE, 
        send_packet.TEMPERATURE, 
        send_packet.PRESSURE, 
        send_packet.VOLTAGE, 
        send_packet.GYRO_R,
        send_packet.GYRO_P, 
        send_packet.GYRO_Y, 
        send_packet.ACCEL_R,
        send_packet.ACCEL_P, 
        send_packet.ACCEL_Y, 
        send_packet.MAG_R, 
        send_packet.MAG_P, 
        send_packet.MAG_Y, 
        send_packet.AUTO_GYRO_ROTATION_RATE, 
        send_packet.GPS_TIME,
        send_packet.GPS_ALTITUDE, 
        send_packet.GPS_LATITUDE, 
        send_packet.GPS_LONGITUDE, 
        send_packet.GPS_SATS, 
        send_packet.CMD_ECHO,
        send_packet.CAMERA_STATUS); 
}

const char* telemetryManager::cmd_buff_to_echo()
{
	int comma = 0;
	int echo_indx = 0;

	char buff[CMD_BUFF_SIZE];
	char *cmd_buff = mission_info.cmd_buff;

	for (int i = 0; cmd_buff[i] != '\0'; i++)
	{
		if (cmd_buff[i] == ',')
		{
			comma++;
			continue;
		}

		if (comma > 1)
		{
			buff[echo_indx++] = cmd_buff[i];
		}
	}

	buff[echo_indx] = '\0';
	return buff;
}

const float telemetryManager::pressure_to_alt(const float pressure)
{
	return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE_HPA, 0.1903));
}
