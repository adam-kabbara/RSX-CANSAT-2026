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
	struct rpy_data getGyroData();
	struct rpy_data getAccelData();
	struct gps_data getGPSData();
	cam_status getCameraStatus();
	void setRTCTime(char *time);
	char *getRTCTime();
	void startSensors(SerialManager &serial);
};

#endif /* INC_SENSOR_MANAGER_HPP_ */
