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

extern SPI_HandleTypeDef hspi1; // is this correct way to access hspi1 from main.c?


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


/* ============================================================================
 * EEPROM PRIVATE FUNCTIONS
 * ========================================================================== */

bool SensorManager::readBytes(unsigned long address, unsigned char *buffer, unsigned int size)
{
	// checking for valid parameters
	if(address + size > MEM_SIZE || buffer == nullptr || size == 0)
	{
		return false;
	}

	HAL_GPIO_WritePin(SPI_CS_GPIO_OUT_GPIO_Port, SPI_CS_GPIO_OUT_Pin, GPIO_PIN_RESET);
	
	unsigned char cmd[4]; // command + 3 byte address
	cmd[0] = READ_CMD;
	cmd[1] = (unsigned char)((address >> 16) & 0xFF); // MSB (7 are don't care bits)
	cmd[2] = (unsigned char)((address >> 8) & 0xFF);
	cmd[3] = (unsigned char)(address & 0xFF); // LSB
	
	// if STM32 malfunctioned errors:
	if(HAL_SPI_Transmit(&hspi1, cmd, 4, TIMEOUT) != HAL_OK)
	{
		HAL_GPIO_WritePin(SPI_CS_GPIO_OUT_GPIO_Port, SPI_CS_GPIO_OUT_Pin, GPIO_PIN_SET);
		return false;
	}
	
	if(HAL_SPI_Receive(&hspi1, buffer, size, TIMEOUT) != HAL_OK)
	{
		HAL_GPIO_WritePin(SPI_CS_GPIO_OUT_GPIO_Port, SPI_CS_GPIO_OUT_Pin, GPIO_PIN_SET);
		return false;
	}
	
	HAL_GPIO_WritePin(SPI_CS_GPIO_OUT_GPIO_Port, SPI_CS_GPIO_OUT_Pin, GPIO_PIN_SET);
	return true;
}


bool SensorManager::readString(unsigned long address, unsigned int size, char *buffer)
{
	// check for errors and valid parameters while reading the bytes
	if(!readBytes(address, (unsigned char*)buffer, size))
	{
		return false;
	}
	
	buffer[size] = '\0';
	return true;
}

/* This is the lower-level function compared to writeByte. Can write up to 256 bytes, within page boundary*/
bool SensorManager::writePage(unsigned long address, const unsigned char *buffer, unsigned int size)
{
	// checking for valid parameters
	if(address + size > MEM_SIZE || buffer == nullptr || size == 0 || size > PAGE_SIZE)
	{
		return false;
	}
	
	// Check if write crosses page boundary
	if((address / PAGE_SIZE) != ((address + size - 1) / PAGE_SIZE))
	{
		return false;
	}
	
	writeEnable();

	HAL_GPIO_WritePin(SPI_CS_GPIO_OUT_GPIO_Port, SPI_CS_GPIO_OUT_Pin, GPIO_PIN_RESET);
	
	unsigned char cmd[4];
	cmd[0] = WRITE_CMD;
	cmd[1] = (unsigned char)((address >> 16) & 0xFF);
	cmd[2] = (unsigned char)((address >> 8) & 0xFF);
	cmd[3] = (unsigned char)(address & 0xFF);
		
	if(HAL_SPI_Transmit(&hspi1, cmd, 4, TIMEOUT) != HAL_OK)
	{
		HAL_GPIO_WritePin(SPI_CS_GPIO_OUT_GPIO_Port, SPI_CS_GPIO_OUT_Pin, GPIO_PIN_SET);
		return false;
	}
	
	if(HAL_SPI_Transmit(&hspi1, (unsigned char*)buffer, size, TIMEOUT) != HAL_OK)
	{
		HAL_GPIO_WritePin(SPI_CS_GPIO_OUT_GPIO_Port, SPI_CS_GPIO_OUT_Pin, GPIO_PIN_SET);
		return false;
	}
	
	HAL_GPIO_WritePin(SPI_CS_GPIO_OUT_GPIO_Port, SPI_CS_GPIO_OUT_Pin, GPIO_PIN_SET);
	
	return waitForWriteComplete();
}


/* Higher level compared to writePage. Can write any number of bytes so can span multiple pgs*/
bool SensorManager::writeBytes(unsigned long address, const unsigned char *buffer, unsigned int size)
{
	// checking for valid parameters
	if(address + size > MEM_SIZE || buffer == nullptr || size == 0)
	{
		// not checking for page size limit here since can span multiple pages
		return false;
	}

	int bytes_to_write = size;
	unsigned long cur_address = address;
	const unsigned char *cur_buffer = buffer;

	while (bytes_to_write > 0)
	{
		// need to calculate how many bytes can be written in current page
		unsigned int page_offset = cur_address % PAGE_SIZE;
		unsigned int bytes_in_page = PAGE_SIZE - page_offset;
		
		// determine chunk size to write
		unsigned int chunk_size;
		if (bytes_to_write < bytes_in_page)
		{
			chunk_size = bytes_to_write;
		}
		else
		{
			chunk_size = bytes_in_page;
		}
		
		if(!writePage(cur_address, cur_buffer, chunk_size))
		{
			return false;
		}
		
		bytes_to_write -= chunk_size;
		cur_address += chunk_size;
		cur_buffer += chunk_size;
	}

	return true;
}


bool SensorManager::writeString(unsigned long address, unsigned int size, const char *buffer)
{
	if (buffer == nullptr)
	{
		return false;
	}

	return writeBytes(address, (const unsigned char*)buffer, size);
}


bool SensorManager::updateHeader(unsigned int block_index, unsigned int new_size)
{
	if(block_index > BLOCK_LOG)
	{
		return false;
	}
	
	// Header structure: [start_addr0][size0][start_addr1][size1]...
	// Each entry is 8 bytes: 4 for address, 4 for size
	unsigned long header_offset = block_index * 8 + 4;
	unsigned char size_bytes[4];
	
	size_bytes[0] = (unsigned char)((new_size >> 24) & 0xFF);
	size_bytes[1] = (unsigned char)((new_size >> 16) & 0xFF);
	size_bytes[2] = (unsigned char)((new_size >> 8) & 0xFF);
	size_bytes[3] = (unsigned char)(new_size & 0xFF);
	
	return writeBytes(header_offset, size_bytes, 4);
}


unsigned int SensorManager::getBlockSize(unsigned int block_index)
{
	// reads the size of a block from the header

	if(block_index > BLOCK_LOG)
	{
		return 0;
	}
	
	// header structure: [start_addr0][size0][start_addr1][size1]...
	// each entry is 8 bytes: 4 for address, 4 for size
	unsigned long header_offset = block_index * 8 + 4;
	unsigned char size_bytes[4];
	
	if(!readBytes(header_offset, size_bytes, 4))
	{
		return 0;
	}
	
	return ((unsigned int)size_bytes[0] << 24) |
	       ((unsigned int)size_bytes[1] << 16) |
	       ((unsigned int)size_bytes[2] << 8) |
	       ((unsigned int)size_bytes[3]);
}


/* ============================================================================
 * EEPROM PUBLIC FUNCTIONS
 * ========================================================================== */

void SensorManager::EEPROM_updateAltitude(float alt)
{
	char buffer[RECOVERY_BLOCK_SIZE];
	snprintf(buffer, RECOVERY_BLOCK_SIZE, "%.2f", alt); 
	
	// ensure string fits in block
	unsigned int str_len = strlen(buffer);
	if(str_len >= RECOVERY_BLOCK_SIZE)
	{
		str_len = RECOVERY_BLOCK_SIZE - 1; // for null terminator (kinda not needed lolsies but just in case)
	}
	
	unsigned long addr = RECOVERY_DATA_START + (BLOCK_ALTITUDE * RECOVERY_BLOCK_SIZE);
	
	if(writeString(addr, str_len, buffer))
	{
		updateHeader(BLOCK_ALTITUDE, str_len);
	}
}


void SensorManager::EEPROM_updateState(OperatingState state)
{
	char buffer[RECOVERY_BLOCK_SIZE];
	snprintf(buffer, RECOVERY_BLOCK_SIZE, "%d", (int)state);
	
	unsigned int str_len = strlen(buffer);
	if(str_len >= RECOVERY_BLOCK_SIZE)
	{
		str_len = RECOVERY_BLOCK_SIZE - 1;
	}
	
	unsigned long addr = RECOVERY_DATA_START + (BLOCK_STATE * RECOVERY_BLOCK_SIZE);
	
	if(writeString(addr, str_len, buffer))
	{
		updateHeader(BLOCK_STATE, str_len);
	}
}


void SensorManager::EEPROM_updateMode(OperatingMode mode)
{
	char buffer[RECOVERY_BLOCK_SIZE];
	snprintf(buffer, RECOVERY_BLOCK_SIZE, "%d", (int)mode);
	
	unsigned int str_len = strlen(buffer);
	if(str_len >= RECOVERY_BLOCK_SIZE)
	{
		str_len = RECOVERY_BLOCK_SIZE - 1;
	}
	
	unsigned long addr = RECOVERY_DATA_START + (BLOCK_MODE * RECOVERY_BLOCK_SIZE);
	
	if(EEPROM_writeString(addr, str_len, buffer))
	{
		EEPROM_updateHeader(BLOCK_MODE, str_len);
	}
}


void SensorManager::EEPROM_updatePackets(int count)
{
	char buffer[RECOVERY_BLOCK_SIZE];
	snprintf(buffer, RECOVERY_BLOCK_SIZE, "%d", count);
	
	unsigned int str_len = strlen(buffer);
	if(str_len >= RECOVERY_BLOCK_SIZE)
	{
		str_len = RECOVERY_BLOCK_SIZE - 1;
	}
	
	unsigned long addr = RECOVERY_DATA_START + (BLOCK_PACKET * RECOVERY_BLOCK_SIZE);
	
	if(EEPROM_writeString(addr, str_len, buffer))
	{
		EEPROM_updateHeader(BLOCK_PACKET, str_len);
	}
}


struct recovery_data SensorManager::EEPROM_getRecoveryData()
{
	struct recovery_data data;
	char buffer[RECOVERY_BLOCK_SIZE + 1];
	unsigned int size;
	unsigned long addr;
	
	// Read altitude
	size = EEPROM_getBlockSize(BLOCK_ALTITUDE);
	if(size > 0 && size < RECOVERY_BLOCK_SIZE)
	{
		addr = RECOVERY_DATA_START + (BLOCK_ALTITUDE * RECOVERY_BLOCK_SIZE);
		if(EEPROM_readString(addr, size, buffer))
		{
			data.launch_altitude = atof(buffer); // convert to float
		}
		else
		{
			data.launch_altitude = 0.0; // what to do on error?
		}
	}
	else
	{
		data.launch_altitude = 0.0; // default if no data (?)
	}
	
	// Read state
	size = EEPROM_getBlockSize(BLOCK_STATE);
	if(size > 0 && size < RECOVERY_BLOCK_SIZE)
	{
		addr = RECOVERY_DATA_START + (BLOCK_STATE * RECOVERY_BLOCK_SIZE);
		if(EEPROM_readString(addr, size, buffer))
		{
			data.state = (OperatingState)atoi(buffer); // convert to OperatingState
		}
		else
		{
			data.state = OperatingState::IDLE; // default if no data (?)
		}
	}
	else
	{
		data.state = OperatingState::IDLE; // default if no data (?)
	}
	
	// Read mode
	size = EEPROM_getBlockSize(BLOCK_MODE);
	if(size > 0 && size < RECOVERY_BLOCK_SIZE)
	{
		addr = RECOVERY_DATA_START + (BLOCK_MODE * RECOVERY_BLOCK_SIZE);
		if(EEPROM_readString(addr, size, buffer))
		{
			data.mode = (OperatingMode)atoi(buffer); // convert to OperatingMode
		}
		else
		{
			data.mode = OperatingMode::OPMODE_FLIGHT; // default if no data (?)
		}
	}
	else
	{
		data.mode = OperatingMode::OPMODE_FLIGHT; // default if no data (?)
	}
	
	// Read packet count
	size = EEPROM_getBlockSize(BLOCK_PACKET);
	if(size > 0 && size < RECOVERY_BLOCK_SIZE)
	{
		addr = RECOVERY_DATA_START + (BLOCK_PACKET * RECOVERY_BLOCK_SIZE);
		if(EEPROM_readString(addr, size, buffer))
		{
			data.packet_count = atoi(buffer); // convert to int
		}
		else
		{
			data.packet_count = 0; // default if no data (?)
		}
	}
	else
	{
		data.packet_count = 0; // default if no data (?)
	}
	
	return data;
}

