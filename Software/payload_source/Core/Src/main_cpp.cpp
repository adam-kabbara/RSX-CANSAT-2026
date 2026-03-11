#include "main.h"
#include "global_includes.hpp"
#include "mission_manager.hpp"
#include "sensor_manager.hpp"
#include "serial_manager.hpp"
#include "telemetry_manager.hpp"
#include "command_manager.hpp"

extern "C" volatile uint8_t send_flag;
extern "C" volatile uint8_t pvd_flag;
extern "C" volatile uint8_t update_flag;
extern "C" UART_HandleTypeDef huart1;
extern "C" TIM_HandleTypeDef htim1;
extern "C" TIM_HandleTypeDef htim2;
extern "C" TIM_HandleTypeDef htim3;
extern "C" TIM_HandleTypeDef htim4;
extern "C" TIM_HandleTypeDef htim8;
extern "C" SPI_HandleTypeDef hspi1;
extern "C" I2C_HandleTypeDef hi2c1;
extern "C" volatile char rx_buff[128];
extern "C" volatile uint8_t cmd_ready;

extern "C" void main_cpp()
{

    SerialManager serial(huart1);

    MissionManager mission_mgr;
    CommandManager cmd_mgr;
    TelemetryManager telemetry_mgr;

    SensorManager sensors;

	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)){
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
		serial.sendErrorMsg("Reset Reason: external pin reset");
	}
	else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)){
		serial.sendErrorMsg("Reset Reason: brown-out reset");
	}
	else {
		serial.sendErrorMsg("Reset Reason: unknown");
	}

	__HAL_RCC_CLEAR_RESET_FLAGS();

	sensors.startSensors(serial, &hi2c1, &htim2, &htim3, &htim4);

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
    else
    {
    	serial.sendInfoMsg("Setup completed, entering IDLE mode");
    }

    char cmd_buff[CMD_BUFF_SIZE];
    char send_buff[DATA_BUFF_SIZE];

    while(1)
    {
        while(mission_mgr.getOpState() == IDLE)
        {
            if(cmd_ready)
            {
            	memcpy(cmd_buff, (const char*)rx_buff, CMD_BUFF_SIZE);
            	cmd_ready = 0;

                if(cmd_mgr.processCommand(cmd_buff, serial, mission_mgr, sensors))
                {
                    mission_mgr.setLastCommand(cmd_buff);
                }

            }

            /*
            if(pvd_flag == 1)
            {
                serial.sendErrorMsg("WARNING: POWER VOLTAGE DETECTOR TRIGGERED");
                pvd_flag = 0;
            }
            */
            
            HAL_Delay(10);
        }

        if(mission_mgr.getOpMode() == OPMODE_SIM)
        {
            serial.sendInfoMsg("SIM_START");

            // Wait until first simulation packet is received
            while(mission_mgr.getOpMode() == OPMODE_SIM && mission_mgr.isWaitingSimp())
            {
                HAL_Delay(100);
            }
        }

        serial.sendInfoMsg("MISSION STARTING!");

        HAL_TIM_Base_Start_IT(&htim1);

        while(mission_mgr.getOpState() != IDLE)
        {
            if(cmd_ready)
            {
            	memcpy(cmd_buff, (const char*)rx_buff, CMD_BUFF_SIZE);
            	cmd_ready = 0;

                if(cmd_mgr.processCommand(cmd_buff, serial, mission_mgr, sensors))
                {
                    mission_mgr.setLastCommand(cmd_buff);
                }
            }

            if(send_flag)
            {
            	telemetry_mgr.sampleSensors(sensors, mission_mgr);
            	telemetry_mgr.build_data_str(send_buff, sizeof(send_buff));

            	serial.sendTelemetry(send_buff);

            	if(mission_mgr.logfile_ok() && !sensors.EEPROM_addLogLine(send_buff))
				{
					serial.sendErrorMsg("Warning: Unable to add line to logfile!");
					mission_mgr.disableLogfile();
				}

            	send_flag = 0;
            }

        }

        HAL_TIM_Base_Stop_IT(&htim1);
        
        mission_mgr.setAltCalOff();
        mission_mgr.waitingForSimp();
        mission_mgr.enableLogfile();

        serial.sendInfoMsg("Transitioning back to IDLE mode...");
    }
}
