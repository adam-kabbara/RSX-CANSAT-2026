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
    /* Declare sensors */
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
	return 0.0;
}

float SensorManager::getCurrent()
{
	return 0.0;
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

void SensorManager::writeNoseconeServo(float val)
{
	servo_nosecone.SetAngle(val);
}

void SensorManager::writeContainerServo(float val)
{
	servo_container.SetAngle(val);
}

void SensorManager::writeWingDirServo(float val)
{
	servo_wing_dir.SetAngle(val);
}

void SensorManager::writeWingPWMServo(float val)
{
	servo_wing_pwm.SetAngle(val);
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
	
	if(writeString(addr, str_len, buffer))
	{
		updateHeader(BLOCK_MODE, str_len);
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
	
	if(writeString(addr, str_len, buffer))
	{
		updateHeader(BLOCK_PACKET, str_len);
	}
}


struct recovery_data SensorManager::EEPROM_getRecoveryData()
{
	struct recovery_data data;
	char buffer[RECOVERY_BLOCK_SIZE + 1];
	unsigned int size;
	unsigned long addr;
	
	// Read altitude
	size = getBlockSize(BLOCK_ALTITUDE);
	if(size > 0 && size < RECOVERY_BLOCK_SIZE)
	{
		addr = RECOVERY_DATA_START + (BLOCK_ALTITUDE * RECOVERY_BLOCK_SIZE);
		if(readString(addr, size, buffer))
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
	size = getBlockSize(BLOCK_STATE);
	if(size > 0 && size < RECOVERY_BLOCK_SIZE)
	{
		addr = RECOVERY_DATA_START + (BLOCK_STATE * RECOVERY_BLOCK_SIZE);
		if(readString(addr, size, buffer))
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
	size = getBlockSize(BLOCK_MODE);
	if(size > 0 && size < RECOVERY_BLOCK_SIZE)
	{
		addr = RECOVERY_DATA_START + (BLOCK_MODE * RECOVERY_BLOCK_SIZE);
		if(readString(addr, size, buffer))
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
	size = getBlockSize(BLOCK_PACKET);
	if(size > 0 && size < RECOVERY_BLOCK_SIZE)
	{
		addr = RECOVERY_DATA_START + (BLOCK_PACKET * RECOVERY_BLOCK_SIZE);
		if(readString(addr, size, buffer))
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

void SensorManager::startSensors(SerialManager &serial, I2C_HandleTypeDef *hi2c1,
		TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4)
{
	/* Start all sensors that need to be started
	 * Add a delay between each start and send an
	 * info message */

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

	HAL_Delay(100);

	servo_nosecone.Init(htim4, TIM_CHANNEL_2, 1000, 2000, 180);
	servo_container.Init(htim4, TIM_CHANNEL_1, 1000, 2000, 180);
	servo_wing_dir.Init(htim2, TIM_CHANNEL_1, 1000, 2000, 180);
	servo_wing_pwm.Init(htim2, TIM_CHANNEL_2, 1000, 2000, 180);
	servo_elevator.Init(htim3, TIM_CHANNEL_1, 1000, 2000, 180);
	servo_aileron.Init(htim3, TIM_CHANNEL_2, 1000, 2000, 180);
	servo_egg.Init(htim3, TIM_CHANNEL_3, 1000, 2000, 180);

	HAL_Delay(100);

	BNO_enableGyro(20000, serial);

	//BNO_enableAccel(20000, serial);

	//BNO_enableMag(20000, serial);

	//BNO_enableRotationVector(20000, serial);

	serial.sendInfoMsg("Sensor initialization complete.");
}
