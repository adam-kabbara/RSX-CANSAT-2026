/*
 * telemetry_manager.cpp
 *
 *  Manages sending of telemetry packets
 */

#include "telemetry_manager.hpp"

void TelemetryManager::sampleSensors(SensorManager &sensors, MissionManager &mission_info, SerialManager &serial)
{
	packet.TEAM_ID_PCKT = TEAM_ID;

	sensors.getRTCTime(packet.MISSION_TIME);

	mission_info.incrPacketCount();
	sensors.EEPROM_updatePackets(mission_info.getPacketCount(), serial);
	packet.PACKET_COUNT = mission_info.getPacketCount();

	strcpy(packet.STATE, op_state_to_string(mission_info.getOpState()));

	strcpy(packet.MODE, op_mode_to_string(mission_info.getOpMode(), 0));

	sensors.updateBMP();

	float pressure;
	if(mission_info.getOpMode() == OPMODE_SIM)
	{
		pressure = mission_info.getSimpData();
	}
	else
	{
		pressure = sensors.getPressure();
	}

	packet.PRESSURE = pressure/1000.0;

	packet.ALTITUDE = pressure_to_alt(pressure) - mission_info.getLaunchAlt();

	packet.TEMPERATURE = sensors.getTemp();

	packet.VOLTAGE = sensors.getVoltage();

	packet.CURRENT = sensors.getCurrent();

	struct rpy_data gyro_accel_data = sensors.getIMUData();//sensors.getCalibratedGyro(); //;
	
	packet.GYRO_R = (int)lroundf(gyro_accel_data.gyro_r);
	packet.GYRO_P = (int)lroundf(gyro_accel_data.gyro_p);
	packet.GYRO_Y = (int)lroundf(gyro_accel_data.gyro_y);

	packet.ACCEL_R = (int)lroundf(gyro_accel_data.accel_r);
	packet.ACCEL_P = (int)lroundf(gyro_accel_data.accel_p);
	packet.ACCEL_Y = (int)lroundf(gyro_accel_data.accel_y);

	struct gps_data gps_data_vals = sensors.getGPSData();

	strcpy(packet.GPS_TIME, gps_data_vals.time);
	packet.GPS_ALTITUDE = gps_data_vals.altitude;
	packet.GPS_LATITUDE = gps_data_vals.latitude;
	packet.GPS_LONGITUDE = gps_data_vals.longitude;
	packet.GPS_SATS = gps_data_vals.sats;

	cmd_buff_to_echo(packet.CMD_ECHO, mission_info.getLastCommand());

	packet.QUATERNION_X = 0.0;
	packet.QUATERNION_Y = 0.0;
	packet.QUATERNION_Z = 0.0;

	packet.VELOCITY_X = 0.0;
	packet.VELOCITY_Y = 0.0;
	packet.VELOCITY_Z = 0.0;

	packet.ACCEL_XX = 0.0;
	packet.ACCEL_YY = 0.0;
	packet.ACCEL_ZZ = 0.0;
}

void TelemetryManager::build_data_str(char *buff, size_t size)
{
    snprintf(buff, size,
        "%d,%s,%d,%s,%s,"
        "%.1f,%.1f,%.1f,%.1f,%.1f,%d,"
        "%d,%d,%d,%d,%d,"
        "%s,"
        "%.1f,%.4f,%.4f,%d,%s,"
        "%.1f,%.1f,%.1f,%.1f,"
        "%.1f,%.1f,%.1f,"
        "%.1f,%.1f\r",
		packet.TEAM_ID_PCKT,
		packet.MISSION_TIME,
		packet.PACKET_COUNT,
        packet.MODE,
		packet.STATE,
		packet.ALTITUDE,
		packet.TEMPERATURE,
		packet.PRESSURE,
		packet.VOLTAGE,
		packet.CURRENT,
		packet.GYRO_R,
		packet.GYRO_P,
		packet.GYRO_Y,
		packet.ACCEL_R,
		packet.ACCEL_P,
		packet.ACCEL_Y,
		packet.GPS_TIME,
		packet.GPS_ALTITUDE,
		packet.GPS_LATITUDE,
		packet.GPS_LONGITUDE,
		packet.GPS_SATS,
		packet.CMD_ECHO,
		packet.QUATERNION_X,
		packet.QUATERNION_Y,
		packet.QUATERNION_Z,
		packet.VELOCITY_X,
		packet.VELOCITY_Y,
		packet.VELOCITY_Z,
		packet.ACCEL_XX,
		packet.ACCEL_YY,
		packet.ACCEL_ZZ);
}

void TelemetryManager::cmd_buff_to_echo(char buff[CMD_BUFF_SIZE], char *cmd_buff)
{
	int comma = 0;
	int echo_indx = 0;

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
}
