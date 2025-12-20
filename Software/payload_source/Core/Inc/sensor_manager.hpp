/*
 * sensor_manager.h
 *
 *  Header for sensor manager
 */

#ifndef INC_SENSOR_MANAGER_HPP_
#define INC_SENSOR_MANAGER_HPP_

/* Add include paths for sensor manager here */
#include "global_includes.hpp"
#include "serial_manager.hpp"
/* Only to be used for start sensors function,
 * it's easier than making individual functions...*/

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
}
#endif

class SensorManager
{
private:

	/* Any sensor specific variables should be private */
	/* Note: do not initialize a sensor here. Instead,
	 * create a pointer and initialize it in the constructor. */

public:
	SensorManager();

	float getPressure();
	float getTemp();
	float getVoltage();
	float getCurrent();
	struct rpy_data getIMUData();
	struct gps_data getGPSData();
	cam_status getCameraStatus();
	void setRTCTime(int h, int m, int s);
	void getRTCTime(char time_str[DATA_SIZE]);
	void getGPSTime(char time_str[DATA_SIZE]);
	void EEPROM_updateAltitude(float alt);
	void EEPROM_updateState(OperatingState state);
	void EEPROM_updateMode(OperatingMode mode);
	void EEPROM_updatePackets(int count);
	bool EEPROM_addLogLine(char *buffer);
	struct recovery_data EEPROM_getRecoveryData();
	void startSensors(SerialManager &serial);
};

#endif /* INC_SENSOR_MANAGER_HPP_ */
