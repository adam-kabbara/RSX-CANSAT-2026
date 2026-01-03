/*
 * sensor_manager.cpp
 *
 *  Manages the sensor code
 */

#include "sensor_manager.hpp"

// datasheet link for further ref: https://ww1.microchip.com/downloads/aemDocuments/documents/MPD/ProductDocuments/DataSheets/25LC1024-1-Mbit-SPI-Bus-Serial-EEPROM-20002064E.pdf

/* constants from datasheet! */
#define MEM_SIZE 131072UL // 128 kb x 8 (unsigned long for math)
#define PAGE_SIZE 256 // 256 bytes per page
#define TIMEOUT 100 // (in ms) some HAL funcs need timeout value

/* spi commands from datasheet */
#define READ_CMD  0x03
#define WRITE_CMD 0x02
#define WREN_CMD  0x06 // write enable
#define WRDI_CMD  0x04 // write disable
#define RDSR_CMD  0x05 // read status reg
#define WRSR_CMD  0x01 // write status reg
/*there is also page erase but can overwrite directly so not needed (?)*/ 

/* need to check status reg: bits for wpen, wip, wel, bp1, and bp0 bits  */
#define SR_WIP 0x01 // wip (write in progress)
#define SR_WEL 0x02 // wel (write enable latch)
/*there are bp1 and bp0 bits for block protect but not using them(?)*/

/* header needed at beginning of memory to define starting addr of each block and current size of data in each block */
#define HEADER_SIZE 64 // in bytes (would provide space for 8 blocks just in case)
#define NUM_RECOVERY_BLOCKS 4 // recovery data field has 4 fields: launch altitude, state, mode, packet count
#define RECOVERY_BLOCK_SIZE 16 
#define RECOVERY_DATA_START (HEADER_SIZE)
#define LOG_DATA_START (HEADER_SIZE + (NUM_RECOVERY_BLOCKS * RECOVERY_BLOCK_SIZE)) // log data starts after recovery data
#define LOG_DATA_SIZE (MEM_SIZE - LOG_DATA_START) // size available for log data
#define LOG_LINE_TERMINATOR '\r'

/* indices for recovery data blocks */
#define BLOCK_ALTITUDE 0
#define BLOCK_STATE 1
#define BLOCK_MODE 2
#define BLOCK_PACKET 3
#define BLOCK_LOG 4

SPI_HandleTypeDef hspi1; // is this correct way to access hspi1 from main.c?


void writeEnable()
{
	unsigned char cmd = WREN_CMD;
	
	HAL_GPIO_WritePin(SPI_CS_GPIO_OUT_GPIO_Port, SPI_CS_GPIO_OUT_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, &cmd, 1, TIMEOUT);
	HAL_GPIO_WritePin(SPI_CS_GPIO_OUT_GPIO_Port, SPI_CS_GPIO_OUT_Pin, GPIO_PIN_SET);
}


bool waitForWriteComplete()
{
	unsigned char status;
	unsigned int timeout = 0;
	unsigned char cmd = RDSR_CMD;
	
	while(timeout < TIMEOUT)
	{
		HAL_GPIO_WritePin(SPI_CS_GPIO_OUT_GPIO_Port, SPI_CS_GPIO_OUT_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &cmd, 1, TIMEOUT); // read the status register
		HAL_SPI_Receive(&hspi1, &status, 1, TIMEOUT); 
		HAL_GPIO_WritePin(SPI_CS_GPIO_OUT_GPIO_Port, SPI_CS_GPIO_OUT_Pin, GPIO_PIN_SET);
		
		// if write in progress bit is cleared, write is complete
		if((status & SR_WIP) == 0)
		{
			return true;
		}
		
		HAL_Delay(1);
		timeout++;
	}
	
	return false;
}

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

void SensorManager::startSensors(SerialManager &serial)
{
	/* Start all sensors that need to be started
	 * Add a delay between each start and send an
	 * info message */
	HAL_Delay(1);
	serial.sendInfoMsg("Sensor initialization complete.");
}
