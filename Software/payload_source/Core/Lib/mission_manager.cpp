#include "mission_manager.h"
#include <stm32g4xx_hal.h>

void MissionManager::beginPref(const char *name, bool rw)
{
    preferences.begin(name, rw);
}

void MissionManager::endPref()
{
    preferences.end();
}

void MissionManager::putPrefInt(const char *key, int value)
{
    preferences.putInt(key, value);
}

int MissionManager::getPrefInt(const char *key, int def)
{
    return preferences.getInt(key, def);
}

void MissionManager::putPrefFloat(const char *key, float value)
{
    preferences.putFloat(key, value);
}

float MissionManager::getPrefFloat(const char *key, float def)
{
    return preferences.getFloat(key, def);
}

void MissionManager::clearPref()
{
    preferences.clear();
}

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
	// else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PWRRST)){
	// 	serial.sendErrorMsg("Reset Reason: power-on reset");
	// }
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
//    esp_reset_reason_t reset_reason = esp_reset_reason();
//
//    serial.sendErrorMsg("Processor Restarted!");
//
//    switch (reset_reason) {
//
//        case ESP_RST_SW:
//            serial.sendErrorMsg("Reset Reason: Manual software trigger");
//            break;
//        case ESP_RST_UNKNOWN:
//            serial.sendErrorMsg("Reset Reason: Unknown");
//            break;
//        case ESP_RST_POWERON:
//            serial.sendErrorMsg("Reset Reason: Power On Reset");
//            break;
//        case ESP_RST_EXT:
//            serial.sendErrorMsg("Reset Reason: External Reset");
//            break;
//        case ESP_RST_PANIC:
//            serial.sendErrorMsg("Reset Reason: Software Reset due to Panic/Exception");
//            break;
//        case ESP_RST_INT_WDT:
//            serial.sendErrorMsg("Reset Reason: Interrupt Watchdog Reset");
//            break;
//        case ESP_RST_TASK_WDT:
//            serial.sendErrorMsg("Reset Reason: Task Watchdog Reset");
//            break;
//        case ESP_RST_WDT:
//            serial.sendErrorMsg("Reset Reason: General Watchdog Reset");
//            break;
//        case ESP_RST_DEEPSLEEP:
//            serial.sendErrorMsg("Reset Reason: Deep Sleep Wakeup");
//            break;
//        case ESP_RST_BROWNOUT:
//            serial.sendErrorMsg("Reset Reason: Brownout Reset");
//            break;
//        case ESP_RST_SDIO:
//            serial.sendErrorMsg("Reset Reason: SDIO Reset");
//            break;
//        default:
//            serial.sendErrorMsg("Reset Reason: Unknown");
//            break;
//    }

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
