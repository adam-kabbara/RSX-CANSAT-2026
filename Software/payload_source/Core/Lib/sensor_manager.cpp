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

void SensorManager::startTof()
{
	VL53L1X_StartRanging(tof_dev);
}

void SensorManager::stopTof()
{
	VL53L1X_StopRanging(tof_dev);
}

bool SensorManager::checkTof()
{
	uint8_t data_ready;
	VL53L1X_CheckForDataReady(tof_dev, &data_ready);
	return (data_ready == 1);
}

bool SensorManager::tofValid()
{
	uint8_t range_status;
	VL53L1X_GetRangeStatus(tof_dev, &range_status);
	VL53L1X_ClearInterrupt(tof_dev); /* clear interrupt has to be called to enable next interrupt*/
	return (range_status == 0);
}

uint16_t SensorManager::tofDistReading()
{
	uint16_t distance;
	VL53L1X_GetDistance(tof_dev, &distance);
	VL53L1X_ClearInterrupt(tof_dev); /* clear interrupt has to be called to enable next interrupt*/
	return distance;
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
	return INA219getBusVoltage();
}

float SensorManager::getCurrent()
{
	return INA219getCurrent();
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

bool SensorManager::BNO_dataReady()
{
	return BNO085_DataReady(&bno_dev);
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

void SensorManager::setRTCTime(uint8_t h, uint8_t m, uint8_t s)
{
	DS1307_SetHour(h);
	DS1307_SetMinute(m);
	DS1307_SetSecond(s);
}

void SensorManager::getRTCTime(char time_str[DATA_SIZE])
{
	uint8_t h = DS1307_GetHour();
	uint8_t m = DS1307_GetMinute();
	uint8_t s = DS1307_GetSecond();
	snprintf(time_str, DATA_SIZE, "%02d:%02d:%02d", h, m, s);
}

void SensorManager::getGPSTime(char time_str[DATA_SIZE])
{
	snprintf(time_str, DATA_SIZE, "%02d:%02d:%02d", 0, 0, 0);
}

void SensorManager::activate_egg_release()
{
	// writeEggServo(0);
}

void SensorManager::activate_wing_deployment()
{
	//
}

void SensorManager::activate_nosecone_release()
{
	// writeNoseconeServo(0);
}
void SensorManager::activate_probe_release()
{
	// writeContainerServo(0);
}

void SensorManager::writeNoseconeServo(float val)
{
	servo_nosecone.SetAngle(val);
}

void SensorManager::writeContainerServo(float val)
{
	servo_container.SetAngle(val);
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
		TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4)
{
	/* Start all sensors that need to be started
	 * Add a delay between each start and send an
	 * info message */

	if(!DS1307_Init(hi2c1))
	{
		serial.sendErrorMsg("RTC Init failed");
	}

	if(!INA219setup(MAX_EXP_CURRENT_A, 0.1, 0))
	{
		serial.sendErrorMsg("INA Init failed");
	}

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

	BNO_enableGyro(50000, serial);

	HAL_Delay(100);

	servo_nosecone.Init(htim4, TIM_CHANNEL_2, 1000, 2000, 180);
	servo_container.Init(htim4, TIM_CHANNEL_1, 1000, 2000, 180);
	servo_elevator.Init(htim3, TIM_CHANNEL_1, 1000, 2000, 180);
	servo_aileron.Init(htim3, TIM_CHANNEL_2, 1000, 2000, 180);
	servo_egg.Init(htim3, TIM_CHANNEL_3, 1000, 2000, 180);

	// TODO
	// htim2 TIM_CHANNEL_1 and TIM_CHANNEL_2 available for wing driver

	HAL_Delay(100);

	uint32_t tof_bootup_start = HAL_GetTick();
	uint8_t tof_sensor_state = 0;

	while(tof_sensor_state == 0)
	{
	 	VL53L1X_BootState(tof_dev, &tof_sensor_state);
	 	HAL_Delay(2);
	 	if(HAL_GetTick() - tof_bootup_start > 100)
	 	{
	 		serial.sendErrorMsg("TOF init failed");
	 		break;
	 	}
	}

	if(tof_sensor_state != 0)
	{
		VL53L1X_SensorInit(tof_dev);
		VL53L1X_SetDistanceMode(tof_dev, 2); /* 1=short, 2=long */
		VL53L1X_SetTimingBudgetInMs(tof_dev, TOF_TIMING_BUDGET_MS); /* in ms possible values [20, 50, 100, 200, 500] */
		VL53L1X_SetInterMeasurementInMs(tof_dev, TOF_TIMING_BUDGET_MS); /* in ms, IM must be > = TB */
		// TODO replace calibrate with set (delete calibrate)
		VL53L1X_CalibrateOffset(tof_dev, 140, &offset);
		VL53L1X_CalibrateXtalk(tof_dev, 1000, &xtalk);
		serial.sendInfoDataMsg("Offset value=%d, xtalk value=%d. Delete these functions and uncomment set functions", offset, xtalk);
		//VL53L1X_SetOffset(tof_dev, offset);
		//VL53L1X_SetXtalk(tof_dev, xtalk);
	}

	HAL_Delay(100);

	serial.sendInfoMsg("Sensor initialization complete.");
}
