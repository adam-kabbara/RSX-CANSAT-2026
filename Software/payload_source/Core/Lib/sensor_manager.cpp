/*
 * sensor_manager.cpp
 *
 *  Manages the sensor code
 */

#include "sensor_manager.hpp"

SensorManager::SensorManager()
{
    /* Initialize sensors */
}

struct bar_data getBarData()
{
	struct bar_data data;
	data.altitude = 0.0;
	data.pressure = 0.0;
	return data;
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

struct rpy_data SensorManager::getGyroData()
{
	struct rpy_data data;
	data.data_r = 0.0;
	data.data_p = 0.0;
	data.data_y = 0.0;
	return data;
}

struct rpy_data SensorManager::getAccelData()
{
	struct rpy_data data;
	data.data_r = 0.0;
	data.data_p = 0.0;
	data.data_y = 0.0;
	return data;
}

struct gps_data SensorManager::getGPSData()
{
	struct gps_data data;
	data.altitude = 0.0;
	data.latitude = 0.0;
	data.longitude = 0.0;
	data.sats = 0;
	data.time = "00:00:00";
	return data;
}

cam_status SensorManager::getCameraStatus()
{
	return cam_status::CAM1_OFF_CAM2_OFF;
}

void SensorManager::setRTCTime(char *time)
{
	return;
}

char* SensorManager::getRTCTime()
{
	return "00:00:00";
}

void SensorManager::startSensors(SerialManager &serial)
{
	/* Start all sensors that need to be started
	 * Add a delay between each start and send an
	 * info message */
	HAL_Delay(1);
	serial.sendInfoMsg("Sensor initialization complete.");
}
