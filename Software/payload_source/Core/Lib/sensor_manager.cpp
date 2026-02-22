/*
 * sensor_manager.cpp
 *
 *  Manages the sensor code
 */

#include "sensor_manager.hpp"

SensorManager::SensorManager()
{
    /* Declare sensors */
}

float SensorManager::getPressure()
{
	return 0.0;
}

float SensorManager::getTemp()
{
	return 0.0;
}

float SensorManager::getVoltage()
{
	return 0.0;
}

float SensorManager::getCurrent()
{
	return 0.0;
}

struct rpy_data SensorManager::getIMUData()
{
	struct rpy_data data;
	data.gyro_r = 0.0;
	data.gyro_p = 0.0;
	data.gyro_y = 0.0;
	data.accel_r = 0.0;
	data.accel_p = 0.0;
	data.accel_y = 0.0;
	return data;
}

struct gps_data SensorManager::getGPSData()
{
	struct gps_data data;
	data.altitude = 0.0;
	data.latitude = 0.0;
	data.longitude = 0.0;
	data.sats = 0;
	char gps_time[DATA_SIZE] = "00:00:00";
	strcpy(data.time, gps_time);
	return data;
}

cam_status SensorManager::getCameraStatus()
{
	return cam_status::CAM1_OFF_CAM2_OFF;
}

void SensorManager::setRTCTime(int h, int m, int s)
{
	return;
}

void SensorManager::getRTCTime(char time_str[DATA_SIZE])
{
	snprintf(time_str, DATA_SIZE, "%02d:%02d:%02d", 0, 0, 0);
}

void SensorManager::getGPSTime(char time_str[DATA_SIZE])
{
	snprintf(time_str, DATA_SIZE, "%02d:%02d:%02d", 0, 0, 0);
}

void SensorManager::writeNoseconeServo(float val)
{
	servo_nosecone.SetAngle(val);
}

void SensorManager::writeContainerServo(float val)
{
	servo_container.SetAngle(val);
}

void SensorManager::writeWingDirServo(float val)
{
	servo_wing_dir.SetAngle(val);
}

void SensorManager::writeWingPWMServo(float val)
{
	servo_wing_pwm.SetAngle(val);
}

void SensorManager::writeElevatorServo(float val)
{
	servo_elevator.SetAngle(val);
}

void SensorManager::writeAileronServo(float val)
{
	servo_aileron.SetAngle(val);
}

void SensorManager::writeEggServo(float val)
{
	servo_egg.SetAngle(val);
}

void SensorManager::EEPROM_updateAltitude(float alt)
{
	return;
}

void SensorManager::EEPROM_updateState(OperatingState state)
{
	return;
}

void SensorManager::EEPROM_updateMode(OperatingMode mode)
{
	return;
}

void SensorManager::EEPROM_updatePackets(int count)
{
	return;
}

bool SensorManager::EEPROM_addLogLine(char *buffer)
{
	return true;
}

struct recovery_data SensorManager::EEPROM_getRecoveryData()
{
	struct recovery_data data;
	data.launch_altitude = 0.0;
	data.state = OperatingState::IDLE;
	data.mode = OperatingMode::OPMODE_FLIGHT;
	data.packet_count = 0;

	return data;
}

void SensorManager::startSensors(SerialManager &serial, I2C_HandleTypeDef *hi2c1,
		TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4)
{
	/* Start all sensors that need to be started
	 * Add a delay between each start and send an
	 * info message */

	/*
	if(!BMP5_Init(bmp_dev, hi2c1, BMP5_I2C_ADDR_FIRST))
	{
		serial.sendInfoMsg("BMP initialized successfully.");
	}
	else
	{
		serial.sendErrorMsg("BMP initialization failed.");
	}

	HAL_Delay(100);
	*/

	servo_nosecone.Init(htim4, TIM_CHANNEL_2, 1000, 2000, 180);
	servo_container.Init(htim4, TIM_CHANNEL_1, 1000, 2000, 180);
	servo_wing_dir.Init(htim2, TIM_CHANNEL_1, 1000, 2000, 180);
	servo_wing_pwm.Init(htim2, TIM_CHANNEL_2, 1000, 2000, 180);
	servo_elevator.Init(htim3, TIM_CHANNEL_1, 1000, 2000, 180);
	servo_aileron.Init(htim3, TIM_CHANNEL_2, 1000, 2000, 180);
	servo_egg.Init(htim3, TIM_CHANNEL_3, 1000, 2000, 180);

	HAL_Delay(100);

	serial.sendInfoMsg("Sensor initialization complete.");
}
