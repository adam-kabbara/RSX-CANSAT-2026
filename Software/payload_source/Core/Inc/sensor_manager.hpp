/*
 * sensor_manager.h
 *
 *  Header for sensor manager
 */

#ifndef INC_SENSOR_MANAGER_HPP
#define INC_SENSOR_MANAGER_HPP

#include "global_includes.hpp"
#include "serial_manager.hpp"
#include "BMP581.hpp"
#include "BNO085.hpp"
#include "servo.hpp"

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
	BMP5 bmp_dev;
	BNO085_t bno_dev;
	uint32_t bno_last_t = 0.0;
	float prev_gyro_r;
	float prev_gyro_p;
	float prev_gyro_y;
	Servo servo_nosecone;
	Servo servo_container;
	Servo servo_wing_dir;
	Servo servo_wing_pwm;
	Servo servo_elevator;
	Servo servo_aileron;
	Servo servo_egg;

public:
	SensorManager();

	int updateBMP();
	float getPressure();
	float getTemp();
	float getVoltage();
	float getCurrent();
	void BNO_enableGyro(int microsec, SerialManager &serial);
	void BNO_enableAccel(int microsec, SerialManager &serial);
	void BNO_enableMag(int microsec, SerialManager &serial);
	void BNO_enableRotationVector(int microsec, SerialManager &serial);
	void updateBNO();
	struct rpy_data getIMUData();
	struct gps_data getGPSData();
	cam_status getCameraStatus();
	void setRTCTime(int h, int m, int s);
	void getRTCTime(char time_str[DATA_SIZE]);
	void getGPSTime(char time_str[DATA_SIZE]);
	void writeNoseconeServo(float val);
	void writeContainerServo(float val);
	void writeWingDirServo(float val);
	void writeWingPWMServo(float val);
	void writeElevatorServo(float val);
	void writeAileronServo(float val);
	void writeEggServo(float val);
	void EEPROM_updateAltitude(float alt);
	void EEPROM_updateState(OperatingState state);
	void EEPROM_updateMode(OperatingMode mode);
	void EEPROM_updatePackets(int count);
	bool EEPROM_addLogLine(char *buffer);
	struct recovery_data EEPROM_getRecoveryData();
	void startSensors(SerialManager &serial, I2C_HandleTypeDef *hi2c1,
			TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4);
};

#endif /* INC_SENSOR_MANAGER_HPP */
