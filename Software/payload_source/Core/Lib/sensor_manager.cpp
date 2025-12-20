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
	return True;
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

void SensorManager::startSensors(SerialManager &serial)
{
	/* Start all sensors that need to be started
	 * Add a delay between each start and send an
	 * info message */
	HAL_Delay(1);
	serial.sendInfoMsg("Sensor initialization complete.");
}
