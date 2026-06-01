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
#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
#include "INA219.hpp"
#include "ds1307.hpp"
#include "drv.hpp"
#include "eeprom.hpp"

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
	static const uint32_t EEPROM_TOTAL_BYTES       = 131072UL; // priv
 
    static const uint32_t EEPROM_NUM_RECOVERY_FIELDS = 9UL; // priv
    static const uint32_t EEPROM_HEADER_ENTRY_SIZE  = 4UL;   // bytes per size field
    static const uint32_t EEPROM_HEADER_SIZE =
        (EEPROM_NUM_RECOVERY_FIELDS + 1UL) * EEPROM_HEADER_ENTRY_SIZE;  // 20 bytes
 
    static const uint32_t EEPROM_FIELD_BLOCK_SIZE  = 16UL;

	/* Starting addresses of each recovery-data block */
    static const uint32_t EEPROM_ADDR_ALT    = EEPROM_HEADER_SIZE;                          // 0x000014
    static const uint32_t EEPROM_ADDR_STATE  = EEPROM_ADDR_ALT   + EEPROM_FIELD_BLOCK_SIZE; // 0x000024
    static const uint32_t EEPROM_ADDR_MODE   = EEPROM_ADDR_STATE + EEPROM_FIELD_BLOCK_SIZE; // 0x000034
    static const uint32_t EEPROM_ADDR_PKTCNT = EEPROM_ADDR_MODE  + EEPROM_FIELD_BLOCK_SIZE; // 0x000044
	static const uint32_t EEPROM_ADDR_MAXALT      = EEPROM_ADDR_PKTCNT    + EEPROM_FIELD_BLOCK_SIZE; // 0x000054
	static const uint32_t EEPROM_ADDR_EGGREL      = EEPROM_ADDR_MAXALT    + EEPROM_FIELD_BLOCK_SIZE; // 0x000064
	static const uint32_t EEPROM_ADDR_WINGREL     = EEPROM_ADDR_EGGREL    + EEPROM_FIELD_BLOCK_SIZE; // 0x000074
	static const uint32_t EEPROM_ADDR_PROBEREL    = EEPROM_ADDR_WINGREL   + EEPROM_FIELD_BLOCK_SIZE; // 0x000084
	static const uint32_t EEPROM_ADDR_NOSECONEREL = EEPROM_ADDR_PROBEREL  + EEPROM_FIELD_BLOCK_SIZE; // 0x000094
	static const uint32_t EEPROM_ADDR_LOG         = EEPROM_ADDR_NOSECONEREL + EEPROM_FIELD_BLOCK_SIZE; // 0x0000A4
    static const uint32_t EEPROM_LOG_MAX     = EEPROM_TOTAL_BYTES - EEPROM_ADDR_LOG;
 
    /* Offsets into the header for each size field */
    static const uint32_t EEPROM_HDR_IDX_ALT    = 0UL;
    static const uint32_t EEPROM_HDR_IDX_STATE  = 1UL;
    static const uint32_t EEPROM_HDR_IDX_MODE   = 2UL;
    static const uint32_t EEPROM_HDR_IDX_PKTCNT = 3UL;
    static const uint32_t EEPROM_HDR_IDX_LOG    = 4UL;
	static const uint32_t EEPROM_HDR_IDX_MAXALT      = 5UL;
	static const uint32_t EEPROM_HDR_IDX_EGGREL      = 6UL;
	static const uint32_t EEPROM_HDR_IDX_WINGREL     = 7UL;
	static const uint32_t EEPROM_HDR_IDX_PROBEREL    = 8UL;
	static const uint32_t EEPROM_HDR_IDX_NOSECONEREL = 9UL;

	I2C_HandleTypeDef *gps_hi2c = nullptr;
	gps_data internal_gps_storage;
	char gps_nmea_buffer[100];
	uint8_t gps_buf_idx = 0; // counter tracking string length
	static const uint16_t UBLOX_I2C_ADDR = (0x42 << 1);


public:

	SensorManager();

	bool checkTof();
	bool tofValid();
	uint16_t tofDistReading();
	void startTof();
	void stopTof();

	int updateBMP();
	float getPressure();
	float getTemp();

	float getVoltage();
	float getCurrent();

	void BNO_enableGyro(int microsec, SerialManager &serial);
	void BNO_enableAccel(int microsec, SerialManager &serial);
	void BNO_enableMag(int microsec, SerialManager &serial);
	void BNO_enableRotationVector(int microsec, SerialManager &serial);
	bool BNO_dataReady();
	void updateBNO();
	struct rpy_data getIMUData();

	void updateGPS();
	struct gps_data getGPSData();

	cam_status getCameraStatus();

	void setRTCTime(uint8_t h, uint8_t m, uint8_t s);
	void getRTCTime(char time_str[DATA_SIZE]);
	void getGPSTime(char time_str[DATA_SIZE]);

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
	void updateMotor();
	void stopMotor();

	uint16_t EEPROM_readString(uint32_t start_addr, char *buf, uint16_t max_len);
	bool EEPROM_writeString(uint32_t start_addr, const char *str, uint16_t len);
	uint32_t EEPROM_readHeaderSize(uint32_t index);
	void EEPROM_writeHeaderSize(uint32_t index, uint32_t size);
	// void EEPROM_serialError(const char *msg);

	void EEPROM_updateAltitude(float alt, SerialManager &serial);
	void EEPROM_updateState(OperatingState state, SerialManager &serial);
	void EEPROM_updateMode(OperatingMode mode, SerialManager &serial);
	void EEPROM_updatePackets(int count, SerialManager &serial);
	bool EEPROM_addLogLine(char *buffer, SerialManager &serial);
	void EEPROM_updateMaxAlt(float alt);
	void EEPROM_updateEggRel();
	void EEPROM_updateWingRel();
	void EEPROM_updateProbeRel();
	void EEPROM_updateNoseconeRel();
	void EEPROM_resetData();
	struct recovery_data EEPROM_getRecoveryData();
	void EEPROM_replayLog(uint32_t line_delay_ms, SerialManager &serial);

	void startSensors(SerialManager &serial, I2C_HandleTypeDef *hi2c1,
			SPI_HandleTypeDef *hspi_eeprom, GPIO_TypeDef *cs_port, uint16_t cs_pin,
			TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4,
			GPIO_TypeDef *wing_dir_port, uint16_t wing_dir_pin);
};

#endif /* INC_SENSOR_MANAGER_HPP */
