#include "mission_manager.hpp"
#include <stm32g4xx_hal.h>

OperatingState MissionManager::getOpState()
{
    return op_state;
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
}

float MissionManager::getLaunchAlt()
{
    return launch_altitude;
}

void MissionManager::setAltCalOff()
{
    ALT_CAL_CHK = false;
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

void MissionManager::clearPacketCount()
{
    packet_count = 0;
}

void MissionManager::waitingForSimp()
{
    waiting_for_simp = 1;
}

void MissionManager::simpRecv()
{
    waiting_for_simp = 0;
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
