#include "main.h"
#include "global_includes.hpp"
#include "drv.h"
#include "mission_manager.hpp"
#include "sensor_manager.hpp"
#include "serial_manager.hpp"
#include "telemetry_manager.hpp"
#include "command_manager.hpp"
#include "controller.hpp"

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

uint32_t nosecone_rel__payload_rel_timer=0;
uint32_t wing_servo_timer=0;
uint32_t tof_timer=0;
OperatingState update_state(SensorManager &sensors, MissionManager &mgr, OperatingState current_state);

extern "C" void main_cpp()
{

    SerialManager serial(huart1);

    MissionManager mission_mgr;
    CommandManager cmd_mgr;
    TelemetryManager telemetry_mgr;

    SensorManager sensors;

    Controller pid_controller;

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
			motor_update();
            HAL_Delay(10);
        }

        if(mission_mgr.getOpMode() == OPMODE_SIM)
        {
            serial.sendInfoMsg("SIM_START");

            // Wait until first simulation packet is received
            while(mission_mgr.getOpMode() == OPMODE_SIM && mission_mgr.isWaitingSimp())
            {
				motor_update();
                HAL_Delay(100);
            }
        }

        serial.sendInfoMsg("MISSION STARTING!");

        HAL_TIM_Base_Start_IT(&htim1);
        HAL_TIM_Base_Start_IT(&htim8);

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

            	if(mission_mgr.getOpState() == ASCENT)
            	{
            		mission_mgr.apogee_packet_sent();
            	}
            }

            if(update_flag)
            {
            	sensors.updateBMP();

            	float pressure_val;
            	if(mission_mgr.getOpMode() == OPMODE_SIM)
				{
            		pressure_val = mission_mgr.getSimpData()/1000.0;
				}
            	else
            	{
            		pressure_val = sensors.getPressure();
            	}

            	mission_mgr.update_alt_buffer(pressure_val - mission_mgr.getLaunchAlt());

				if(mission_mgr.getOpState() == DESCENT || mission_mgr.getOpState() == PROBE_RELEASE || mission_mgr.getOpState() == PAYLOAD_RELEASE)
				{
					pid_controller.update();
				}

				OperatingState next_state = update_state(sensors, mission_mgr, mission_mgr.getOpState());
				if(next_state != mission_mgr.getOpState())
				{
					sensors.EEPROM_updateState(next_state);
					mission_mgr.setOpState(next_state);
				}

				update_flag = 0;
            }

            if(sensors.BNO_dataReady())
            {
            	sensors.updateBNO();
            }
            motor_update();
        }

        HAL_TIM_Base_Stop_IT(&htim1);
        HAL_TIM_Base_Stop_IT(&htim8);

        mission_mgr.reset_params();
        mission_mgr.waitingForSimp();
        mission_mgr.enableLogfile();

        serial.sendInfoMsg("Transitioning back to IDLE mode...");
    }
}

OperatingState update_state(SensorManager &sensors, MissionManager &mgr, OperatingState current_state)
{
	OperatingState new_state = current_state;
	switch(current_state)
	{
		case LAUNCH_PAD: {
			float alt = mgr.calculate_median_alt();
			mgr.update_max_alt(alt);
			if(alt > ASCENT_ALT_THRESHOLD_M)
			{
				new_state = ASCENT;
			}
			break;
		}

		case ASCENT: {
			float alt = mgr.calculate_median_alt();
			mgr.update_max_alt(alt);
			if(mgr.get_max_alt() - alt > DESCENT_FALL_THRESHOLD_M)
			{
				if(mgr.descent_trigger())
				{
					new_state = APOGEE;
				}
			}
			break;
		}

		case APOGEE: {
			mgr.update_max_alt(mgr.calculate_median_alt());
			if(mgr.is_apogee_packet_sent())
			{
				new_state = DESCENT;
			}
			break;
		}

		case DESCENT: {
			if(!mgr.nosecone_check() && mgr.calculate_median_alt() <= mgr.get_max_alt() * 0.82)
			{
				sensors.activate_nosecone_release();
				mgr.nosecone_rel();
				nosecone_rel__payload_rel_timer = HAL_GetTick();
				//BNO_enableAccel(50000, serial);
				//BNO_enableMag(50000, serial);
				//BNO_enableRotationVector(50000, serial);
			}

			if(!mgr.probe_check() && mgr.calculate_median_alt() <= mgr.get_max_alt() * 0.80 && (HAL_GetTick()-nosecone_rel__payload_rel_timer>=1000))
			{
				sensors.activate_probe_release();
				mgr.probe_rel();
				wing_servo_timer = HAL_GetTick();
				if(mgr.getOpMode() == OPMODE_FLIGHT)
				{
					tof_timer = HAL_GetTick();
					sensors.startTof();
				}
				new_state = PROBE_RELEASE;
			}

			break;
		}

		case PROBE_RELEASE: {
			if(mgr.getOpMode() == OPMODE_SIM)
			{
				if((HAL_GetTick()-wing_servo_timer>=1000) && !mgr.wing_check())
				{
					sensors.activate_wing_deployment();
					mgr.wing_rel();
				}
				if(mgr.wing_check())
				{
					if(mgr.calculate_median_alt() < EGG_ALT_THRESHOLD_MM && !mgr.egg_check())
					{
						sensors.activate_egg_release();
						mgr.egg_rel();
						new_state = PAYLOAD_RELEASE;
					}
				}
			}
			else if(HAL_GetTick()-tof_timer>=TOF_TIMING_BUDGET_MS && sensors.checkTof())
			{
				tof_timer = HAL_GetTick();
				if(!sensors.tofValid())
				{
					if(!mgr.wing_check())
					{
						sensors.activate_wing_deployment();
						mgr.wing_rel();
					}
				}
				else if(mgr.wing_check())
				{
					uint16_t dist = sensors.tofDistReading();
					if(dist < EGG_ALT_THRESHOLD_MM && !mgr.egg_check())
					{
						sensors.activate_egg_release();
						mgr.egg_rel();
						sensors.stopTof();
						new_state = PAYLOAD_RELEASE;
					}
				}
			}
			break;
		}

		case PAYLOAD_RELEASE: {
			if(mgr.calculate_median_alt() < LANDED_THRESHOLD_M)
			{
				if(mgr.landed_trigger())
				{
					new_state = LANDED;
					HAL_TIM_Base_Stop_IT(&htim8);
				}
			}
			break;
		}

		case LANDED: {
			break;
		}

		default: {
			break;
		}
	}

	return new_state;
}
