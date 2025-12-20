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

void MissionManager::resetSeq(SerialManager &serial)
{

	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
		serial.sendErrorMsg("Reset Reason: low power reset");
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)){
		serial.sendErrorMsg("Reset Reason: window watchdog reset");
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)){
		serial.sendErrorMsg("Reset Reason: independent watchdog reset");
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)){
		serial.sendErrorMsg("Reset Reason: software reset");
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)){
		serial.sendErrorMsg("Reset Reason: external pin reset (NRST)");
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)){
		serial.sendErrorMsg("Reset Reason: brown-out reset (NRST)");
	}
	else {
		serial.sendErrorMsg("Reset Reason: unknown");
	}

	__HAL_RCC_CLEAR_RESET_FLAGS();

    beginPref("xb-set", true);
    mission_info.op_state = static_cast<OperatingState>(getPrefInt("opstate", 6));

    if(mission_info.op_state != IDLE)
    {
        serial.sendErrorMsg("Performing recovery as processor was not in IDLE state! Telemetry should resume!");
        // Get packet count, launch altitude
        mission_info.launch_altitude = getPrefFloat("grndalt", 0.0);
        setAltCalibration(mission_info.launch_altitude);
        mission_info.sim_status = static_cast<SimModeStatus>(getPrefInt("simst", 0));
        mission_info.op_mode = static_cast<OperatingMode>(getPrefInt("opmode", 0));
        mission_info.packet_count = getPrefInt("pckts", 0);
    }

    endPref();
}
