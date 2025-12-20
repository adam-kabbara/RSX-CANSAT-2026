#include "main.h"
#include "global_includes.hpp"
#include "mission_manager.hpp"
#include "sensor_manager.hpp"
#include "serial_manager.hpp"
#include "telemetry_manager.hpp"
#include "command_manager.hpp"

extern "C" volatile uint8_t send_flag;
extern "C" UART_HandleTypeDef huart1;
extern "C" TIM_HandleTypeDef htim1;

extern "C" void main_cpp()
{
    SerialManager serial(huart1);

    MissionManager mission_mgr;
    CommandManager cmd_mgr;
    TelemetryManager telemetry_mgr;

    SensorManager sensors;
    sensors.startSensors(serial);

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

    struct recovery_data recovery = sensors.EEPROM_getRecoveryData();

    mission_mgr.setOpState(recovery.state);

    if(recovery.state != IDLE)
    {
        serial.sendErrorMsg("Performing recovery as processor was not in IDLE state! Telemetry should resume!");
        // Get packet count, launch altitude
        mission_mgr.setAltCalibration(recovery.launch_altitude);
        mission_mgr.setOpMode(recovery.mode);
        mission_mgr.setPacketCount(recovery.packet_count);
    }

    serial.sendInfoMsg("Setup Completed.");

    char cmd_buff[CMD_BUFF_SIZE];

    while(1)
    {
        while(mission_mgr.getOpState() == IDLE)
        {
            if(serial.get_data(cmd_buff))
            {
                if(cmd_mgr.processCommand(cmd_buff, serial, mission_mgr, sensors))
                {
                    mission_mgr.setLastCommand(cmd_buff);
                }
            }
            
            HAL_Delay(100);
        }

        serial.sendInfoMsg("MISSION STARTING!");

        if(mission_mgr.getOpMode() == OPMODE_SIM)
        {
            serial.sendInfoMsg("BEGIN_SIMP");
        }

        HAL_TIM_Base_Start_IT(&htim1);

        while(mission_mgr.getOpState() != IDLE)
        {
            if(serial.get_data(cmd_buff))
            {
                if(cmd_mgr.processCommand(cmd_buff, serial, mission_mgr, sensors))
                {
                    mission_mgr.setLastCommand(cmd_buff);
                }
            }

            // Wait until first simulation packet is received
            if(mission_mgr.getOpMode() == OPMODE_SIM && mission_mgr.isWaitingSimp())
            {
                HAL_Delay(100);
                continue;
            }

            while(send_flag == 0)
            {
                HAL_Delay(25);
            }

            telemetry_mgr.sampleSensors(sensors, serial, mission_mgr);
            send_flag = 0;
        }

        HAL_TIM_Base_Stop_IT(&htim1);
        
        mission_mgr.setAltCalOff();
        mission_mgr.waitingForSimp();
        mission_mgr.enableLogfile();
    }
}
