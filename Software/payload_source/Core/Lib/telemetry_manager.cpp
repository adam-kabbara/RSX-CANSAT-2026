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

		case LANDED: {
			break;
		}

	}

	return curr_state;
}

const char* telemetryManager::sampleSensors(SensorManager &sensors, SerialManager &serial)
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

	strcpy(packet.STATE, op_state_to_string(updateState(mission_info.op_state)));

	packet.TEAM_ID_PCKT = TEAM_ID;

	strcpy(send_packet.MODE, op_mode_to_string(mission_info.op_mode, 0));

	packet.TEMPERATURE = sensors.getTemp();

	packet.VOLTAGE = sensors.getVoltage();

	packet.CURRENT = sensors.getCurrent();


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

const char* telemetryManager::op_mode_to_string(OperatingMode mode, int full)
{
	if (full == 1)
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

const char* telemetryManager::op_state_to_string(OperatingState state)
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

void telemetryManager::resetPacketCount()
{
	packet_count = 0;
}

const float telemetryManager::pressure_to_alt(const float pressure)
{
	return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE_HPA, 0.1903));
}
