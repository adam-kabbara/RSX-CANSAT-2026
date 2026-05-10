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

int SensorManager::updateBMP()
{
	return BMP5_SaveConvData(&bmp_dev);
}

float SensorManager::getPressure()
{
	return bmp_dev.pressure;
}

float SensorManager::getTemp()
{
	return bmp_dev.temperature;
}

float SensorManager::getVoltage()
{
	return 0.0;
}

float SensorManager::getCurrent()
{
	return 0.0;
}

void SensorManager::BNO_enableGyro(int microsec, SerialManager &serial)
{
	if(BNO085_EnableGyro(&bno_dev, microsec) != BNO085_OK)
	{
		serial.sendErrorMsg("BNO GYRO ENABLE DID NOT RETURN OK STATUS");
	}
}

void SensorManager::BNO_enableAccel(int microsec, SerialManager &serial)
{
	if(BNO085_EnableAccelerometer(&bno_dev, microsec) != BNO085_OK)
	{
		serial.sendErrorMsg("BNO ACCELEROMETER ENABLE DID NOT RETURN OK STATUS");
	}
}

void SensorManager::BNO_enableMag(int microsec, SerialManager &serial)
{
	if(BNO085_EnableMagnetometer(&bno_dev, microsec) != BNO085_OK)
	{
		serial.sendErrorMsg("BNO MAGNOMETER ENABLE DID NOT RETURN OK STATUS");
	}
}

void SensorManager::BNO_enableRotationVector(int microsec, SerialManager &serial)
{
	if(BNO085_EnableRotationVector(&bno_dev, microsec) != BNO085_OK)
	{
		serial.sendErrorMsg("BNO ROTATION VECTOR ENABLE DID NOT RETURN OK STATUS");
	}
}

void SensorManager::updateBNO()
{
	BNO085_GetData(&bno_dev);
}

struct rpy_data SensorManager::getIMUData()
{
	struct rpy_data data;
	data.gyro_r = bno_dev.gyro.x * (180.0f / M_PI);
	data.gyro_p = -bno_dev.gyro.y * (180.0f / M_PI);
	data.gyro_y = -bno_dev.gyro.z * (180.0f / M_PI);
	// no idea if this is correct
	if(bno_last_t == 0.0)
	{
		bno_last_t = HAL_GetTick();
		data.accel_r = 0.0;
		data.accel_p = 0.0;
		data.accel_y = 0.0;
	}
	else
	{
		uint32_t now = HAL_GetTick();
		float dt = (now - bno_last_t) / 1000.0f;
		if(dt <= 0) dt = 0.02f;
		data.accel_r = (data.gyro_r - prev_gyro_r) / dt;
		data.accel_p = (data.gyro_p - prev_gyro_p) / dt;
		data.accel_y = (data.gyro_y - prev_gyro_y) / dt;
		prev_gyro_r = data.gyro_r;
		prev_gyro_p = data.gyro_p;
		prev_gyro_y = data.gyro_y;
		bno_last_t = now;
	}
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

	if(BMP5_Init(&bmp_dev, hi2c1, BMP5_I2C_ADDR_FIRST))
	{
		serial.sendErrorMsg("BMP Init failed");
	}

	if(BMP5_Start_Mode(&bmp_dev, 1, BMP5_ODR_120HZ, BMP5_OSR_X4, BMP5_OSR_X1))
	{
		serial.sendErrorMsg("BMP Start Mode Init Failed");
	}

	HAL_Delay(100);

	if(BNO085_Init(&bno_dev, hi2c1, BNO085_I2C_ADDR_DEFAULT) != BNO085_OK)
	{
		serial.sendErrorMsg("BN0 Init failed");
	}

	HAL_Delay(100);

	servo_nosecone.Init(htim4, TIM_CHANNEL_2, 1000, 2000, 180);
	servo_container.Init(htim4, TIM_CHANNEL_1, 1000, 2000, 180);
	servo_wing_dir.Init(htim2, TIM_CHANNEL_1, 1000, 2000, 180);
	servo_wing_pwm.Init(htim2, TIM_CHANNEL_2, 1000, 2000, 180);
	servo_elevator.Init(htim3, TIM_CHANNEL_1, 1000, 2000, 180);
	servo_aileron.Init(htim3, TIM_CHANNEL_2, 1000, 2000, 180);
	servo_egg.Init(htim3, TIM_CHANNEL_3, 1000, 2000, 180);

	HAL_Delay(100);

	BNO_enableGyro(20000, serial);

	//BNO_enableAccel(20000, serial);

	//BNO_enableMag(20000, serial);

	//BNO_enableRotationVector(20000, serial);

	serial.sendInfoMsg("Sensor initialization complete.");
}
