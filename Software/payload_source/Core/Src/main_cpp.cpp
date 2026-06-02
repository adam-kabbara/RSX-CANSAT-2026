#include "drv.hpp"
#include "main.h"
#include "global_includes.hpp"
#include "mission_manager.hpp"
#include "sensor_manager.hpp"
#include "serial_manager.hpp"
#include "telemetry_manager.hpp"
#include "command_manager.hpp"
#include "controller.hpp"
#include "glider_ekf.hpp"

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

uint32_t nosecone_rel__payload_rel_timer = 0;
uint32_t wing_servo_timer = 0;
uint32_t egg_timer = 0;
uint32_t bno_update_timer = 0;
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

	sensors.startSensors(serial, &hi2c1, &hspi1, SPI_CS_GPIO_OUT_GPIO_Port, SPI_CS_GPIO_OUT_Pin, &htim2, &htim3, &htim4);

	glider_ekf_init();

    struct recovery_data recovery = sensors.EEPROM_getRecoveryData();

    mission_mgr.setOpState(recovery.state);
	mission_mgr.setFlightCtrl(AUTONOMOUS); // always start in autonomous mode

    if(recovery.state != IDLE)
    {
        serial.sendErrorMsg("Performing recovery as processor was not in IDLE state! Telemetry should resume!");
		serial.sendErrorMsg("CPL forced into AUTONOMOUS FLIGHT CTRL for recovery.");
        // Get packet count, launch altitude
        mission_mgr.setAltCalibration(recovery.launch_altitude);
        mission_mgr.setOpMode(recovery.mode);
        mission_mgr.setPacketCount(recovery.packet_count);
        mission_mgr.update_max_alt(recovery.max_alt);
        if(recovery.nosecone_flag)
        {
        	mission_mgr.nosecone_rel();
        }
        if(recovery.egg_flag)
        {
        	mission_mgr.egg_rel();
        }
        if(recovery.probe_flag)
        {
        	mission_mgr.probe_rel();
        }
        if(recovery.wing_flag)
        {
        	mission_mgr.wing_rel();
        }

        // Reset the timers
        nosecone_rel__payload_rel_timer = HAL_GetTick();
        wing_servo_timer = HAL_GetTick();
        egg_timer = HAL_GetTick();
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
        { // todo add calibration code
			sensors.updateMotor();
			sensors.updateGPS();

            if(cmd_ready)
            {
            	memcpy(cmd_buff, (const char*)rx_buff, CMD_BUFF_SIZE);
            	cmd_ready = 0;

                if(cmd_mgr.processCommand(cmd_buff, serial, mission_mgr, sensors))
                {
                    mission_mgr.setLastCommand(cmd_buff);
                }

            }
            HAL_Delay(10);

            sensors.updateMotor();
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
        HAL_TIM_Base_Start_IT(&htim8);

        while(mission_mgr.getOpState() != IDLE)
        {
			sensors.updateMotor();
			sensors.updateGPS();
			
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
				float pos[3];
				float vel[3];
				float x_q[4];
				ekf_get_pos(pos);
				ekf_get_vel(vel);
				ekf_get_quaternion(x_q);
				serial.sendInfoDataMsg("EKF State: NED Position (%.1f, %.1f, %.1f), Velocity (%.1f, %.1f, %.1f)", pos[0], pos[1], pos[2], vel[0], vel[1], vel[2]);
				float rpy[3];
				quat_to_rpy(x_q, rpy);
				serial.sendInfoDataMsg("EKF State: RPY (%.3f, %.3f, %.3f)", rpy[0], rpy[1], rpy[2]);
				telemetry_mgr.sampleSensors(sensors, mission_mgr, serial);
            	telemetry_mgr.build_data_str(send_buff, sizeof(send_buff));

            	serial.sendTelemetry(send_buff);

				if(mission_mgr.logfile_ok() && !sensors.EEPROM_addLogLine(send_buff))
				{
					serial.sendErrorMsg("Warning: Unable to add line to logfile, disabling logfile writes!");
					mission_mgr.disableLogfile();
				}

            	send_flag = 0;

            	if(mission_mgr.getOpState() == APOGEE)
            	{
            		mission_mgr.apogee_packet_sent();
            	}
            }

            if(update_flag)
            {
            	float pressure_val;
            	if(mission_mgr.getOpMode() == OPMODE_SIM)
				{
            		pressure_val = mission_mgr.getSimpData();
				}
            	else
            	{
            		sensors.updateBMP();
            		pressure_val = sensors.getPressure();
            	}

            	mission_mgr.update_alt_buffer(pressure_to_alt(pressure_val) - mission_mgr.getLaunchAlt());
				glider_ekf_update_baro(pressure_to_alt(pressure_val) - mission_mgr.getLaunchAlt(), 1.0f);

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
				float raw_accel[3];
				sensors.getRawAccel(raw_accel);
				uint32_t current_time = HAL_GetTick();
				float dt = (current_time - bno_update_timer) / 1000.0f;
				if(dt <= 0) dt = 0.02f; // sanity check
				bno_update_timer = current_time;
				glider_ekf_predict_bno_mode(raw_accel, dt);

				float bno_quat[5];
				sensors.getGameRotationVector(bno_quat);
				glider_ekf_update_bno_quaternion(bno_quat, bno_quat[4]);
            }

			struct gps_data gps_data = sensors.getGPSData();
			if(gps_data.data_ready)
			{
				gps_data.data_ready = false;
				ekf_gps_update(&gps_data);
			}

            sensors.updateMotor();
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
			if(alt > ASCENT_ALT_THRESHOLD_M)
			{
				new_state = ASCENT;
			}
			break;
		}

		case ASCENT: {
			float alt = mgr.calculate_median_alt();
			if(mgr.update_max_alt(alt))
			{
				sensors.EEPROM_updateMaxAlt(alt);
			}
			bool threshold_check = mgr.get_max_alt() - alt > DESCENT_FALL_THRESHOLD_M;
			if(mgr.descent_trigger(threshold_check))
			{
				new_state = APOGEE;
			}
			break;
		}

		case APOGEE: {
			float alt = mgr.calculate_median_alt();
			if(mgr.update_max_alt(alt))
			{
				sensors.EEPROM_updateMaxAlt(alt);
			}
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
				sensors.EEPROM_updateNoseconeRel();
				nosecone_rel__payload_rel_timer = HAL_GetTick();
			}

			if(!mgr.probe_check() && mgr.calculate_median_alt() <= mgr.get_max_alt() * 0.80 && (HAL_GetTick()-nosecone_rel__payload_rel_timer>=1000))
			{
				sensors.activate_probe_release();
				mgr.probe_rel();
				sensors.EEPROM_updateProbeRel();
				wing_servo_timer = HAL_GetTick();
				if(mgr.getOpMode() == OPMODE_FLIGHT)
				{
					egg_timer = HAL_GetTick();
				}
			}

			if(mgr.probe_check())
			{
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
					sensors.EEPROM_updateWingRel();
				}
				if(mgr.wing_check())
				{
					if(mgr.calculate_median_alt() < EGG_ALT_THRESHOLD_M && !mgr.egg_check())
					{
						sensors.activate_egg_release();
						mgr.egg_rel();
						sensors.EEPROM_updateEggRel();
					}
				}
				if(mgr.egg_check())
				{
					new_state = PAYLOAD_RELEASE;
				}
			}
			else
			{
				if(HAL_GetTick()-egg_timer>=EGG_TIMING_BUDGET_MS && mgr.calculate_median_alt() < mgr.get_max_alt() * 0.77)
				{
					if(!mgr.wing_check())
					{
						sensors.activate_wing_deployment();
						mgr.wing_rel();
						sensors.EEPROM_updateWingRel();
					}
				}

				if(mgr.wing_check())
				{
					float alt = mgr.calculate_median_alt();
					if(alt < (EGG_ALT_THRESHOLD_M + mgr.getEggAlt()) && !mgr.egg_check())
					{
						sensors.activate_egg_release();
						mgr.egg_rel();
						sensors.EEPROM_updateEggRel();
					}
				}
				if(mgr.egg_check())
				{
					new_state = PAYLOAD_RELEASE;
				}
			}
			break;
		}

		case PAYLOAD_RELEASE: {
			bool threshold_check = mgr.calculate_median_alt() < LANDED_THRESHOLD_M;
			if(mgr.landed_trigger(threshold_check))
			{
				new_state = LANDED;
				HAL_TIM_Base_Stop_IT(&htim8);
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
