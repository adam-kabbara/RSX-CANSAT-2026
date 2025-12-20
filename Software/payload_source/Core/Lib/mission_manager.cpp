#include "mission_manager.h"
#include <stm32g4xx_hal.h>

OperatingState MissionManager::getOpState()
{
    return mission_info.op_state;
}

SimModeStatus MissionManager::getSimStatus()
{
    return mission_info.sim_status;
}

OperatingMode MissionManager::getOpMode()
{
    return mission_info.op_mode;
}

void MissionManager::setOpState(OperatingState state)
{
    mission_info.op_state = state;
}

void MissionManager::setSimStatus(SimModeStatus status)
{
    mission_info.sim_status = status;
}

void MissionManager::setOpMode(OperatingMode mode)
{
    mission_info.op_mode = mode;
}

uint8_t MissionManager::isAltCalibrated()
{
    return mission_info.ALT_CAL_CHK;
}

void MissionManager::setAltCalibration(float alt)
{
    mission_info.ALT_CAL_CHK = 1;
    mission_info.launch_altitude = alt;
}

float MissionManager::getLaunchAlt()
{
    return mission_info.launch_altitude;
}

void MissionManager::setAltCalOff()
{
    mission_info.ALT_CAL_CHK = 0;
}

void MissionManager::setPacketCount(int count)
{
    mission_info.packet_count = count;
}

int MissionManager::getPacketCount()
{
    return mission_info.packet_count;
}

void MissionManager::incrPacketCount()
{
    mission_info.packet_count++;
}

void MissionManager::clearPacketCount()
{
    mission_info.packet_count = 0;
}

void MissionManager::waitingForSimp()
{
    mission_info.waiting_for_simp = 1;
}

void MissionManager::simpRecv()
{
    mission_info.waiting_for_simp = 0;
}

uint8_t MissionManager::isWaitingSimp()
{
    return mission_info.waiting_for_simp;
}

void MissionManager::setSimpData(int data)
{
    mission_info.SIMP_DATA = data;
}

int MissionManager::getSimpData()
{
    return mission_info.SIMP_DATA;
}

void MissionManager::setLastCommand(char *cmd)
{
    last_command = cmd;
}

char* MissionManager::getLastCommand()
{
    char[CMD_BUFF_SIZE] command_ret;
    strncpy(last_command, command_ret, sizeof(last_command) - 1);
    command_ret[sizeof(command_ret) - 1] = '\0';
    return command_ret;
}

bool MissionManager::logfile_ok()
{
    return logfile_chk;
}

void MissionManager::disableLogfile()
{
    logfile_chk = False;
}

void MissionManager::enableLogfile()
{
    logfile_chk = True;
}
