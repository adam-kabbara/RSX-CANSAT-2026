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
#include "INA219.hpp"
#include "ds1307.hpp"
#include "drv.hpp"
#include "eeprom.hpp"
#include "GPS.hpp"
#include "runcam.hpp"

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
	Servo servo_elevator;
	Servo servo_aileron;
	Servo servo_egg;

	DRV motor;

	uint16_t tof_dev=0x52;
	int16_t offset;
	uint16_t xtalk;

	EEPROMsimple *eeprom_dev = nullptr;

	recovery_data recovery_cache;

	uint32_t eeprom_log_len = 0;

	static const uint8_t  EEPROM_MAGIC             = 0xA5;
	static const uint32_t EEPROM_PAGE_SIZE         = 256UL;
	static const uint32_t EEPROM_TOTAL_BYTES       = 131072UL;
	static const uint32_t EEPROM_ADDR_RECOVERY     = 0UL;
	static const uint32_t EEPROM_ADDR_LOG_LEN      = 64UL;
	static const uint32_t EEPROM_ADDR_LOG          = 68UL;
	static const uint32_t EEPROM_LOG_MAX           = EEPROM_TOTAL_BYTES - EEPROM_ADDR_LOG;

	void EEPROM_writeBytes(uint32_t addr, const uint8_t *data, uint32_t len);
	uint32_t EEPROM_readLogLen();
	void EEPROM_writeLogLen(uint32_t len);
	void EEPROM_Init();
	void EEPROM_saveRecovery();

	GPS gps_parser;

	RunCam ground_camera;
	RunCam payload_camera;

public:

	SensorManager();

	int updateBMP();
	float getPressure();
	float getTemp();

	float getVoltage();
	float getCurrent();

	void BNO_enableGyro(int microsec, SerialManager &serial);
	void BNO_enableAccel(int microsec, SerialManager &serial);
	void BNO_enableLinearAcceleration(int microsec, SerialManager &serial);
	void BNO_enableMag(int microsec, SerialManager &serial);
	void BNO_enableRotationVector(int microsec, SerialManager &serial);
	void BNO_enableGameRotationVector(int microsec, SerialManager &serial);
	void BNO_calibrate(SerialManager &serial);
	void BNO_saveCalibration(SerialManager &serial);
	void BNO_disableCalibration(SerialManager &serial);
	bool BNO_dataReady();
	void updateBNO();
	void rotate_vec3_y_ccw(BNO085_Vec3_t *v, float c, float s);
	void BNO_RotateY(BNO085_t *bno_dev, float angle_rad);
	void getRawGyro(float* data_out);
	struct rpy_data getCalibratedGyro(float* calib_bias);
	struct rpy_data getIMUData();

	void getGameRotationVector(float* data_out);
	void getEulerRotationVector(float* data_out);

	void getRawAccel(float* data_out);
	void getLinearAccel(float* data_out);
	struct rpy_data getCalibratedAccel(float* calib_bias, float* calib_scale);

	void getRawMag(float* data_out);

	void updateGPS(SerialManager &serial);
	float getGPS_alt();
	float getGPS_lat();
	float getGPS_lon();
	int getGPS_sat();
	float getGPS_cog();
	float getGPS_rms();
	float getGPS_sog();
	void GPS_dataReadyOff();
	bool GPS_dataReady();
	void getGPSTime(char time_str[DATA_SIZE]);

	cam_status getCameraStatus();

	void setRTCTime(uint8_t h, uint8_t m, uint8_t s);
	void getRTCTime(char time_str[DATA_SIZE]);

	void activate_egg_release();
	void activate_nosecone_release();
	void activate_probe_release();
	void activate_wing_deployment();
	void writeNoseconeServo(float val);
	void writeContainerServo(float val);
	void writeElevatorServo(float val);
	void writeAileronServo(float val);
	void writeEggServo(float val);
	void writeMotor(uint8_t dir, uint32_t time_ms);
	void stopMotor();
	void updateMotor();
	void writeAileronServoPPM(uint16_t val);
	void writeElevatorServoPPM(uint16_t val);

	void EEPROM_resetLog();
	void EEPROM_updateAltitude(float alt);
	void EEPROM_updateState(OperatingState state);
	void EEPROM_updateMode(OperatingMode mode);
	void EEPROM_updatePackets(int count);
	void EEPROM_updateMaxAlt(float alt);
	void EEPROM_updateEggRel();
	void EEPROM_updateWingRel();
	void EEPROM_updateProbeRel();
	void EEPROM_updateNoseconeRel();
	void EEPROM_resetData();
	struct recovery_data EEPROM_getRecoveryData();
	bool EEPROM_addLogLine(char *buffer);
	void EEPROM_replayLog(uint32_t line_delay_ms, SerialManager &serial);

	void ground_runcam_start();
	void payload_runcam_start();
	void ground_runcam_stop();
	void payload_runcam_stop();

	void startSensors(SerialManager &serial, I2C_HandleTypeDef *hi2c1,
			SPI_HandleTypeDef *hspi_eeprom, GPIO_TypeDef *cs_port, uint16_t cs_pin,
			TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4);
};

#endif /* INC_SENSOR_MANAGER_HPP */
