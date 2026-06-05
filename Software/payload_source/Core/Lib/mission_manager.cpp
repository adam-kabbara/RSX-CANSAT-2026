#include "mission_manager.hpp"
#include <stm32g4xx_hal.h>

bool MissionManager::is_apogee_packet_sent()
{
	return apogee_flag;
}

void MissionManager::apogee_packet_sent()
{
	apogee_flag = true;
}

bool MissionManager::nosecone_check()
{
	return nosecone_flag;
}

bool MissionManager::probe_check()
{
	return probe_flag;
}

bool MissionManager::egg_check()
{
	return egg_flag;
}

bool MissionManager::wing_check()
{
	return wing_flag;
}

void MissionManager::nosecone_rel()
{
	nosecone_flag = true;
}

void MissionManager::probe_rel()
{
	probe_flag = true;
}

void MissionManager::egg_rel()
{
	egg_flag = true;
}

void MissionManager::wing_rel()
{
	wing_flag = true;
}


void MissionManager::reset_params()
{
	ALT_CAL_CHK = false;
	packet_count = 0;
	launch_altitude = 0.0;
	SIMP_DATA = 0;
	memset(alt_buffer, 0, sizeof(alt_buffer));
	alt_buffer_idx = 0;
	max_alt = 0.0;
	descent_trigger_count = 0;
	apogee_flag = false;
	nosecone_flag = false;
	probe_flag = false;
	egg_flag = false;
	wing_flag = false;
	landed_trigger_count = 0;
	egg_alt_cal = 0.0f;
}

void MissionManager::init_params()
{
	packet_count = 0;
	SIMP_DATA = 0;
	memset(alt_buffer, 0, sizeof(alt_buffer));
	alt_buffer_idx = 0;
	max_alt = 0.0;
	descent_trigger_count = 0;
	apogee_flag = false;
	nosecone_flag = false;
	probe_flag = false;
	egg_flag = false;
	wing_flag = false;
	landed_trigger_count = 0;
}

bool MissionManager::landed_trigger(bool consecutive_check)
{
	if(consecutive_check)
	{
		landed_trigger_count++;
		if(landed_trigger_count >= 3)
		{
			return true;
		}
	}
	else
	{
		landed_trigger_count = 0;
	}

	return false;
}

bool MissionManager::descent_trigger(bool consecutive_check)
{
	if(consecutive_check)
	{
		descent_trigger_count++;
		if(descent_trigger_count >= 3)
		{
			return true;
		}
	}
	else
	{
		descent_trigger_count = 0;
	}

	return false;
}

float MissionManager::get_max_alt()
{
	return max_alt;
}

bool MissionManager::update_max_alt(float value)
{
	if(value > max_alt && value < 1000)
	{
		max_alt = value;
		return true;
	}
	return false;
}

void MissionManager::update_alt_buffer(float value)
{
    alt_buffer[alt_buffer_idx] = value;
    alt_buffer_idx = (alt_buffer_idx + 1) % ALTITUDE_SMOOTHING_WINDOW;
}

float MissionManager::calculate_median_alt()
{
    float temp_buffer[ALTITUDE_SMOOTHING_WINDOW];
    std::copy(std::begin(alt_buffer), std::end(alt_buffer), std::begin(temp_buffer));
    std::sort(std::begin(temp_buffer), std::end(temp_buffer));

    return temp_buffer[ALTITUDE_SMOOTHING_WINDOW / 2];
}

OperatingState MissionManager::getOpState()
{
    return op_state;
}

FlightCtrl MissionManager::getFlightCtrl()
{
	return flight_ctrl;
}

SimModeStatus MissionManager::getSimStatus()
{
    return sim_status;
}

OperatingMode MissionManager::getOpMode()
{
    return op_mode;
}

void MissionManager::setOpState(OperatingState state)
{
    op_state = state;
}

void MissionManager::setFlightCtrl(FlightCtrl ctrl)
{
	flight_ctrl = ctrl;
}

void MissionManager::setSimStatus(SimModeStatus status)
{
    sim_status = status;
}

void MissionManager::setOpMode(OperatingMode mode)
{
    op_mode = mode;
}

bool MissionManager::isAltCalibrated()
{
    return ALT_CAL_CHK;
}

void MissionManager::setAltCalibration(float alt)
{
    ALT_CAL_CHK = true;
    launch_altitude = alt;
    for(int i = 0; i < ALTITUDE_SMOOTHING_WINDOW; i++)
    {
    	alt_buffer[i] = 0.0f;
    }
}

void MissionManager::setEggAlt(float alt)
{
	egg_alt_cal = alt;
}

float MissionManager::getEggAlt()
{
	return egg_alt_cal;
}

float MissionManager::getLaunchAlt()
{
    return launch_altitude;
}

void MissionManager::setPacketCount(int count)
{
    packet_count = count;
}

int MissionManager::getPacketCount()
{
    return packet_count;
}

void MissionManager::incrPacketCount()
{
    packet_count++;
}

void MissionManager::waitingForSimp()
{
    waiting_for_simp = true;
}

void MissionManager::simpRecv()
{
    waiting_for_simp = false;
}

bool MissionManager::isWaitingSimp()
{
    return waiting_for_simp;
}

void MissionManager::setSimpData(int data)
{
    SIMP_DATA = data;
}

int MissionManager::getSimpData()
{
    return SIMP_DATA;
}

void MissionManager::setLastCommand(char *cmd)
{
	memcpy(last_command, (const char*)cmd, CMD_BUFF_SIZE);
}

char* MissionManager::getLastCommand()
{
	return last_command;
}

bool MissionManager::logfile_ok()
{
    return logfile_chk;
}

void MissionManager::disableLogfile()
{
    logfile_chk = false;
}

void MissionManager::enableLogfile()
{
    logfile_chk = true;
}

void MissionManager::set_landing_coords(float lat, float lon)
{
	landing_coords[0] = lat;
	landing_coords[1] = lon;
}

float MissionManager::get_landing_lat()
{
	return landing_coords[0];
}

float MissionManager::get_landing_lon()
{
	return landing_coords[1];
}
