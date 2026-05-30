/*
 * sensor_manager.cpp
 *
 *  Manages the sensor code
 */

#include "sensor_manager.hpp"

SensorManager::SensorManager()
{
	/* Declare sensors */
}

void SensorManager::startTof()
{
	VL53L1X_StartRanging(tof_dev);
}

void SensorManager::stopTof()
{
	VL53L1X_StopRanging(tof_dev);
}

bool SensorManager::checkTof()
{
	uint8_t data_ready;
	VL53L1X_CheckForDataReady(tof_dev, &data_ready);
	return (data_ready == 1);
}

bool SensorManager::tofValid()
{
	uint8_t range_status;
	VL53L1X_GetRangeStatus(tof_dev, &range_status);
	VL53L1X_ClearInterrupt(tof_dev); /* clear interrupt has to be called to enable next interrupt*/
	return (range_status == 0);
}

uint16_t SensorManager::tofDistReading()
{
	uint16_t distance;
	VL53L1X_GetDistance(tof_dev, &distance);
	VL53L1X_ClearInterrupt(tof_dev); /* clear interrupt has to be called to enable next interrupt*/
	return distance;
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
	return INA219getBusVoltage();
}

float SensorManager::getCurrent()
{
	return INA219getCurrent();
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

bool SensorManager::BNO_dataReady()
{
	return BNO085_DataReady(&bno_dev);
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

void SensorManager::setRTCTime(uint8_t h, uint8_t m, uint8_t s)
{
	DS1307_SetHour(h);
	DS1307_SetMinute(m);
	DS1307_SetSecond(s);
}

void SensorManager::getRTCTime(char time_str[DATA_SIZE])
{
	uint8_t h = DS1307_GetHour();
	uint8_t m = DS1307_GetMinute();
	uint8_t s = DS1307_GetSecond();
	snprintf(time_str, DATA_SIZE, "%02d:%02d:%02d", h, m, s);
}

void SensorManager::getGPSTime(char time_str[DATA_SIZE])
{
	snprintf(time_str, DATA_SIZE, "%02d:%02d:%02d", 0, 0, 0);
}

void SensorManager::activate_egg_release()
{
	// writeEggServo(0);
}

void SensorManager::activate_wing_deployment()
{
	// writeMotor(0, 0);
}

void SensorManager::activate_nosecone_release()
{
	// writeNoseconeServo(0);
}
void SensorManager::activate_probe_release()
{
	// writeContainerServo(0);
}

void SensorManager::writeNoseconeServo(float val)
{
	servo_nosecone.SetAngle(val);
}

void SensorManager::writeContainerServo(float val)
{
	servo_container.SetAngle(val);
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

void SensorManager::writeMotor(uint8_t dir, uint32_t time_ms)
{
	motor.motor_run(dir, time_ms);
}

void SensorManager::updateMotor()
{
    motor.motor_update();
}

void SensorManager::stopMotor()
{
	motor.motor_stop();
}

uint32_t SensorManager::EEPROM_readHeaderSize(uint32_t index)
{
    /* Each size is stored as a raw uint32_t (4 bytes, big-endian via
     * ReadUnsignedLong) at address  index * 4  within the header block. */
    uint32_t addr = index * EEPROM_HEADER_ENTRY_SIZE;
	return (eeprom_dev != nullptr) ? eeprom_dev->ReadUnsignedLong(addr) : 0UL;
}
 
void SensorManager::EEPROM_writeHeaderSize(uint32_t index, uint32_t size)
{
    uint32_t addr = index * EEPROM_HEADER_ENTRY_SIZE;
	if (eeprom_dev != nullptr)
	{
		eeprom_dev->WriteUnsignedLong(addr, size);
	}
}

uint16_t SensorManager::EEPROM_readString(uint32_t start_addr, char *buf, uint16_t max_len)
{
    if (buf == nullptr || max_len == 0)
	{
        return 0;
	}
 
    /* Read raw bytes then null-terminate. */
	if (eeprom_dev == nullptr)
	{
		buf[0] = '\0';
		return 0;
	}

	eeprom_dev->ReadByteArray(start_addr, reinterpret_cast<uint8_t *>(buf), max_len);
    buf[max_len - 1] = '\0';   // safety terminator

    for (uint16_t i = 0; i < max_len - 1; i++)
    {
        if (buf[i] < 0x20 || buf[i] > 0x7E)
        {
            buf[i] = '\0';
            break;
        }
    }
 
    /* Return length of the string actually stored. */
    return static_cast<uint16_t>(strlen(buf));
}

bool SensorManager::EEPROM_writeString(uint32_t start_addr, const char *str, uint16_t len)
{
    if (str == nullptr || len == 0)
	{
        return false;
	}
 
	if (eeprom_dev == nullptr)
	{
		return false;
	}

	eeprom_dev->WriteByteArray(start_addr, reinterpret_cast<uint8_t *>(const_cast<char *>(str)), len);
    return true;
}

// void SensorManager::EEPROM_serialError(const char *msg)
// {
// 	serial.sendErrorMsg(msg);
// }

void SensorManager::EEPROM_updateAltitude(float alt, SerialManager &serial)
{
	char buf[EEPROM_FIELD_BLOCK_SIZE];
    int written = snprintf(buf, sizeof(buf), "%.4f", static_cast<double>(alt));
    if (written <= 0)
    {
        serial.sendErrorMsg("[EEPROM] ERR: altitude format failed\r\n");
        return;
    }
 
    uint16_t len = static_cast<uint16_t>(strlen(buf));
    if (!EEPROM_writeString(EEPROM_ADDR_ALT, buf, len))
    {
        serial.sendErrorMsg("[EEPROM] ERR: altitude write failed\r\n");
        return;
    }
 
    EEPROM_writeHeaderSize(EEPROM_HDR_IDX_ALT, len);
}

void SensorManager::EEPROM_updateState(OperatingState state, SerialManager &serial)
{
	char buf[EEPROM_FIELD_BLOCK_SIZE];
    int written = snprintf(buf, sizeof(buf), "%d", static_cast<int>(state));
    if (written <= 0)
    {
        serial.sendErrorMsg("[EEPROM] ERR: state format failed\r\n");
        return;
    }
 
    uint16_t len = static_cast<uint16_t>(strlen(buf));
    if (!EEPROM_writeString(EEPROM_ADDR_STATE, buf, len))
    {
        serial.sendErrorMsg("[EEPROM] ERR: state write failed\r\n");
        return;
    }
 
    EEPROM_writeHeaderSize(EEPROM_HDR_IDX_STATE, len);
}

void SensorManager::EEPROM_updateMode(OperatingMode mode, SerialManager &serial)
{
	char buf[EEPROM_FIELD_BLOCK_SIZE];
    int written = snprintf(buf, sizeof(buf), "%d", static_cast<int>(mode));
    if (written <= 0)
    {
        serial.sendErrorMsg("[EEPROM] ERR: mode format failed\r\n");
        return;
    }
 
    uint16_t len = static_cast<uint16_t>(strlen(buf));
    if (!EEPROM_writeString(EEPROM_ADDR_MODE, buf, len))
    {
        serial.sendErrorMsg("[EEPROM] ERR: mode write failed\r\n");
        return;
    }
 
    EEPROM_writeHeaderSize(EEPROM_HDR_IDX_MODE, len);
}

void SensorManager::EEPROM_updatePackets(int count, SerialManager &serial)
{
	char buf[EEPROM_FIELD_BLOCK_SIZE];
    int written = snprintf(buf, sizeof(buf), "%d", count);
    if (written <= 0)
    {
        serial.sendErrorMsg("[EEPROM] ERR: packet_count format failed\r\n");
        return;
    }
 
    uint16_t len = static_cast<uint16_t>(strlen(buf));
    if (!EEPROM_writeString(EEPROM_ADDR_PKTCNT, buf, len))
    {
        serial.sendErrorMsg("[EEPROM] ERR: packet_count write failed\r\n");
        return;
    }
 
    EEPROM_writeHeaderSize(EEPROM_HDR_IDX_PKTCNT, len);
}

bool SensorManager::EEPROM_addLogLine(char *buffer, SerialManager &serial)
{
	if (buffer == nullptr)
	{
        return false;
	}
 
    uint32_t log_used = EEPROM_readHeaderSize(EEPROM_HDR_IDX_LOG);
    uint16_t line_len = static_cast<uint16_t>(strlen(buffer));
 
    /* +1 for the '\r' delimiter */
    uint32_t needed = static_cast<uint32_t>(line_len) + 1UL;
 
    if (log_used + needed > EEPROM_LOG_MAX)
    {
        serial.sendErrorMsg("[EEPROM] ERR: log block full\r\n");
        return false;
    }
 
    uint32_t write_addr = EEPROM_ADDR_LOG + log_used;
 
    /* Write the line content */
    EEPROM_writeString(write_addr, buffer, line_len);
 
    /* Append '\r' delimiter */
    char delim = '\r';
	if (eeprom_dev != nullptr)
	{
		eeprom_dev->WriteByte(write_addr + line_len, static_cast<uint8_t>(delim));
	}
 
    /* Update log size in header */
    EEPROM_writeHeaderSize(EEPROM_HDR_IDX_LOG, log_used + needed);
 
    return true;
}

void SensorManager::EEPROM_updateMaxAlt(float alt)
{
    char buf[EEPROM_FIELD_BLOCK_SIZE];
    int written = snprintf(buf, sizeof(buf), "%.4f", static_cast<double>(alt));
    if (written <= 0) return;

    uint16_t len = static_cast<uint16_t>(strlen(buf));
    if (EEPROM_writeString(EEPROM_ADDR_MAXALT, buf, len))
        EEPROM_writeHeaderSize(EEPROM_HDR_IDX_MAXALT, len);
}

// Release flags store "1" (released) or nothing/0 (not released).
// A header size of 0 means never written == not released.

void SensorManager::EEPROM_updateEggRel()
{
    const char val[] = "1";
    uint16_t len = 1;
    if (EEPROM_writeString(EEPROM_ADDR_EGGREL, val, len))
        EEPROM_writeHeaderSize(EEPROM_HDR_IDX_EGGREL, len);
}

void SensorManager::EEPROM_updateWingRel()
{
    const char val[] = "1";
    uint16_t len = 1;
    if (EEPROM_writeString(EEPROM_ADDR_WINGREL, val, len))
        EEPROM_writeHeaderSize(EEPROM_HDR_IDX_WINGREL, len);
}

void SensorManager::EEPROM_updateProbeRel()
{
    const char val[] = "1";
    uint16_t len = 1;
    if (EEPROM_writeString(EEPROM_ADDR_PROBEREL, val, len))
        EEPROM_writeHeaderSize(EEPROM_HDR_IDX_PROBEREL, len);
}

void SensorManager::EEPROM_updateNoseconeRel()
{
    const char val[] = "1";
    uint16_t len = 1;
    if (EEPROM_writeString(EEPROM_ADDR_NOSECONEREL, val, len))
        EEPROM_writeHeaderSize(EEPROM_HDR_IDX_NOSECONEREL, len);
}

void SensorManager::EEPROM_resetData()
{
	// NOTE: ONLY RESETS MAX ALT AND RELEASE FIELDS
    uint8_t zeros[EEPROM_FIELD_BLOCK_SIZE] = {0};

    if (eeprom_dev != nullptr)
    {
        eeprom_dev->WriteByteArray(EEPROM_ADDR_MAXALT,      zeros, EEPROM_FIELD_BLOCK_SIZE);
        eeprom_dev->WriteByteArray(EEPROM_ADDR_NOSECONEREL, zeros, EEPROM_FIELD_BLOCK_SIZE);
        eeprom_dev->WriteByteArray(EEPROM_ADDR_PROBEREL,    zeros, EEPROM_FIELD_BLOCK_SIZE);
        eeprom_dev->WriteByteArray(EEPROM_ADDR_WINGREL,     zeros, EEPROM_FIELD_BLOCK_SIZE);
        eeprom_dev->WriteByteArray(EEPROM_ADDR_EGGREL,      zeros, EEPROM_FIELD_BLOCK_SIZE);
    }

    EEPROM_writeHeaderSize(EEPROM_HDR_IDX_MAXALT,      0UL);
    EEPROM_writeHeaderSize(EEPROM_HDR_IDX_NOSECONEREL, 0UL);
    EEPROM_writeHeaderSize(EEPROM_HDR_IDX_PROBEREL,    0UL);
    EEPROM_writeHeaderSize(EEPROM_HDR_IDX_WINGREL,     0UL);
    EEPROM_writeHeaderSize(EEPROM_HDR_IDX_EGGREL,      0UL);
}

struct recovery_data SensorManager::EEPROM_getRecoveryData()
{
	// struct recovery_data data;
	// data.launch_altitude = 0.0;
	// data.state = OperatingState::IDLE;
	// data.mode = OperatingMode::OPMODE_FLIGHT;
	// data.packet_count = 0;

	// return data;

	struct recovery_data data;
	data.launch_altitude = 0.0;
	data.state = OperatingState::IDLE;
	data.mode = OperatingMode::OPMODE_FLIGHT;
	data.packet_count = 0;
	data.max_alt = 0.0;
	data.nosecone_flag = false;
	data.probe_flag    = false;
	data.wing_flag     = false;
	data.egg_flag      = false;
 
    char buf[EEPROM_FIELD_BLOCK_SIZE];
 
    /* ── launch_altitude ──────────────────────────────────── */
    if (EEPROM_readHeaderSize(EEPROM_HDR_IDX_ALT) > 0)
    {
        memset(buf, 0, sizeof(buf));
        EEPROM_readString(EEPROM_ADDR_ALT, buf, static_cast<uint16_t>(sizeof(buf)));
        char *endptr = nullptr;
        double parsed = strtod(buf, &endptr);
        if (endptr != buf)  // at least one character was consumed
        {
            data.launch_altitude = static_cast<float>(parsed);
        }
    }
 
    /* ── state ────────────────────────────────────────────── */
    if (EEPROM_readHeaderSize(EEPROM_HDR_IDX_STATE) > 0)
    {
        memset(buf, 0, sizeof(buf));
        EEPROM_readString(EEPROM_ADDR_STATE, buf, static_cast<uint16_t>(sizeof(buf)));
        int parsed = 0;
        if (sscanf(buf, "%d", &parsed) == 1) 
		{
            data.state = static_cast<OperatingState>(parsed);
		}
    }
 
    /* ── mode ─────────────────────────────────────────────── */
    if (EEPROM_readHeaderSize(EEPROM_HDR_IDX_MODE) > 0)
    {
        memset(buf, 0, sizeof(buf));
        EEPROM_readString(EEPROM_ADDR_MODE, buf, static_cast<uint16_t>(sizeof(buf)));
        int parsed = 0;
        if (sscanf(buf, "%d", &parsed) == 1)
		{
            data.mode = static_cast<OperatingMode>(parsed);
		}
    }
 
    /* ── packet_count ─────────────────────────────────────── */
    if (EEPROM_readHeaderSize(EEPROM_HDR_IDX_PKTCNT) > 0)
    {
        memset(buf, 0, sizeof(buf));
        EEPROM_readString(EEPROM_ADDR_PKTCNT, buf, static_cast<uint16_t>(sizeof(buf)));
        int parsed = 0;
        if (sscanf(buf, "%d", &parsed) == 1)
		{
            data.packet_count = parsed;
		}
    }

	/* ── max_altitude ─────────────────────────────────────── */
	if (EEPROM_readHeaderSize(EEPROM_HDR_IDX_MAXALT) > 0)
	{
		memset(buf, 0, sizeof(buf));
		EEPROM_readString(EEPROM_ADDR_MAXALT, buf, static_cast<uint16_t>(sizeof(buf)));
		char *endptr = nullptr;
		double parsed = strtod(buf, &endptr);
		if (endptr != buf)
			data.max_alt = static_cast<float>(parsed);
	}

	/* ── release flags (header size > 0 == released) ─────── */
	data.nosecone_flag = (EEPROM_readHeaderSize(EEPROM_HDR_IDX_NOSECONEREL) > 0);
	data.probe_flag    = (EEPROM_readHeaderSize(EEPROM_HDR_IDX_PROBEREL)    > 0);
	data.wing_flag     = (EEPROM_readHeaderSize(EEPROM_HDR_IDX_WINGREL)     > 0);
	data.egg_flag      = (EEPROM_readHeaderSize(EEPROM_HDR_IDX_EGGREL)      > 0);
 
    return data;
}

void SensorManager::EEPROM_replayLog(uint32_t line_delay_ms, SerialManager &serial)
{
    uint32_t log_used = EEPROM_readHeaderSize(EEPROM_HDR_IDX_LOG);
    if (log_used == 0)
	{
        return;
	}
 
    /* Reusable line buffer – sized to SENTENCE_SIZE from global_includes */
    char line_buf[SENTENCE_SIZE];
    uint16_t line_pos = 0;
 
    for (uint32_t offset = 0; offset < log_used; offset++)
    {
		if (eeprom_dev == nullptr)
		{
			return;
		}

		uint8_t byte = eeprom_dev->ReadByte(EEPROM_ADDR_LOG + offset);
 
        if (byte == '\r' || line_pos >= (SENTENCE_SIZE - 1))
        {
            /* Null-terminate and dispatch the completed line */
            line_buf[line_pos] = '\0';
 
            if (line_pos > 0)
            {
				serial.sendInfoMsg(line_buf); 
                HAL_Delay(line_delay_ms);
            }
 
            line_pos = 0;
        }
        else
        {
            line_buf[line_pos++] = static_cast<char>(byte);
        }
    }
}

void SensorManager::startSensors(SerialManager &serial, I2C_HandleTypeDef *hi2c1,
		SPI_HandleTypeDef *hspi_eeprom, GPIO_TypeDef *cs_port, uint16_t cs_pin,
		TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4,
		GPIO_TypeDef *wing_dir_port, uint16_t wing_dir_pin)
{
	/* Start all sensors that need to be started
	 * Add a delay between each start and send an
	 * info message */

	static EEPROMsimple eeprom_storage(hspi_eeprom, cs_port, cs_pin);
	eeprom_dev = &eeprom_storage;

	if(!DS1307_Init(hi2c1))
	{
		serial.sendErrorMsg("RTC Init failed");
	}

	if(!INA219setup(MAX_EXP_CURRENT_A, 0.1, 0))
	{
		serial.sendErrorMsg("INA Init failed");
	}

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

	BNO_enableGyro(50000, serial);

	HAL_Delay(100);

	servo_nosecone.Init(htim4, TIM_CHANNEL_2, 500, 2500, 90, -90);
	servo_container.Init(htim4, TIM_CHANNEL_1, 500, 2500, 90, -90);
	servo_elevator.Init(htim3, TIM_CHANNEL_1, 500, 2500, 90, -90);
	servo_aileron.Init(htim3, TIM_CHANNEL_2, 500, 2500, 90, -90);
	servo_egg.Init(htim3, TIM_CHANNEL_3, 500, 2500, 90, -90);

	motor.Init(htim2, TIM_CHANNEL_2, wing_dir_port, wing_dir_pin);

	HAL_Delay(100);

	uint32_t tof_bootup_start = HAL_GetTick();
	uint8_t tof_sensor_state = 0;

	while(tof_sensor_state == 0)
	{
	 	VL53L1X_BootState(tof_dev, &tof_sensor_state);
	 	HAL_Delay(2);
	 	if(HAL_GetTick() - tof_bootup_start > 100)
	 	{
	 		serial.sendErrorMsg("TOF init failed");
	 		break;
	 	}
	}

	if(tof_sensor_state != 0)
	{
		VL53L1X_SensorInit(tof_dev);
		VL53L1X_SetDistanceMode(tof_dev, 2); /* 1=short, 2=long */
		VL53L1X_SetTimingBudgetInMs(tof_dev, TOF_TIMING_BUDGET_MS); /* in ms possible values [20, 50, 100, 200, 500] */
		VL53L1X_SetInterMeasurementInMs(tof_dev, TOF_TIMING_BUDGET_MS); /* in ms, IM must be > = TB */
		// TODO replace calibrate with set (delete calibrate)
		VL53L1X_CalibrateOffset(tof_dev, 140, &offset);
		VL53L1X_CalibrateXtalk(tof_dev, 1000, &xtalk);
		serial.sendInfoDataMsg("Offset value=%d, xtalk value=%d. Delete these functions and uncomment set functions", offset, xtalk);
		//VL53L1X_SetOffset(tof_dev, offset);
		//VL53L1X_SetXtalk(tof_dev, xtalk);
	}

	HAL_Delay(100);

	serial.sendInfoMsg("Sensor initialization complete.");
}
