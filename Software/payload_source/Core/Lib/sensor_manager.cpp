/*
 * sensor_manager.cpp
 *
 *  Manages the sensor code
 */

#include "sensor_manager.hpp"
#include <math.h>
#include "serial_manager.hpp"

extern "C" UART_HandleTypeDef huart1;
SerialManager serial(huart1);

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

void SensorManager::BNO_enableLinearAcceleration(int microsec, SerialManager &serial)
{
	if(BNO085_EnableLinearAcceleration(&bno_dev, microsec) != BNO085_OK)
	{
		serial.sendErrorMsg("BNO LINEAR ACCELERATION ENABLE DID NOT RETURN OK STATUS");
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

void SensorManager::BNO_enableGameRotationVector(int microsec, SerialManager &serial)
{
	if(BNO085_EnableGameRotationVector(&bno_dev, microsec) != BNO085_OK)
	{
		serial.sendErrorMsg("BNO GAME ROTATION VECTOR ENABLE DID NOT RETURN OK STATUS");
	}
}

void SensorManager::BNO_calibrate(SerialManager &serial){
	if (BNO085_Calibrate(&bno_dev, 7) != BNO085_OK)
	{
		serial.sendErrorMsg("BNO AUTO CALIBRATION START FAILED");
	}
}

void SensorManager::BNO_disableCalibration(SerialManager &serial){
	if (BNO085_DisableCalibration(&bno_dev) != BNO085_OK)
	{
		serial.sendErrorMsg("BNO AUTO CALIBRATION DISABLE FAILED");
	}
}

void SensorManager::BNO_saveCalibration(SerialManager &serial){
	if (BNO085_SaveCalibration(&bno_dev) != BNO085_OK)
	{
		serial.sendErrorMsg("BNO SAVE CALIBRATION FAILED");
	}
}

bool SensorManager::BNO_dataReady()
{
	return BNO085_DataReady(&bno_dev);
}

void SensorManager::rotate_vec3_y_ccw(BNO085_Vec3_t *v, float c, float s)
{
    float x = v->x;
    float z = v->z;
    v->x =  c * x + s * z;
    v->z = -s * x + c * z;
    /* y and accuracy left untouched */
}

void SensorManager::BNO_RotateY(BNO085_t *bno_dev, float angle_rad)
{
    float c = cosf(angle_rad);
    float s = sinf(angle_rad);

    rotate_vec3_y_ccw(&bno_dev->accel,        c, s);
    rotate_vec3_y_ccw(&bno_dev->gyro,         c, s);
    rotate_vec3_y_ccw(&bno_dev->mag,          c, s);
    rotate_vec3_y_ccw(&bno_dev->linear_accel, c, s);
    rotate_vec3_y_ccw(&bno_dev->gravity,      c, s);
}

void SensorManager::updateBNO()
{
	BNO085_GetData(&bno_dev);
	BNO_RotateY(&bno_dev, M_PI / 2.0f); // rotate sensor data 90 degrees around Y axis to match CPL's frame of reference
	//printf("Gyro=%.4f\r\n", bno_dev.gyro.x);
}

void SensorManager::getRawGyro(float* data_out)
{
	data_out[0] = bno_dev.gyro.x;
	data_out[1] = bno_dev.gyro.y;
	data_out[2] = bno_dev.gyro.z; // need the raw not sensor fusion ones
	data_out[3] = bno_dev.gyro.accuracy;
}

struct rpy_data SensorManager::getCalibratedGyro(float* calib_bias)
{
	float raw[3];
	getRawGyro(raw);
	struct rpy_data data;
	data.gyro_r = raw[0] - calib_bias[0];
	data.gyro_p = raw[1] - calib_bias[1];
	data.gyro_y = raw[2] - calib_bias[2];
	return data;
}

void SensorManager::getGameRotationVector(float* data_out)
{
	data_out[0] = bno_dev.quat.real;
	data_out[1] = bno_dev.quat.i;
	data_out[2] = bno_dev.quat.j;
	data_out[3] = bno_dev.quat.k;
	data_out[4] = bno_dev.quat_accuracy;
}


void SensorManager::getEulerRotationVector(float* data_out)
{
	data_out[0] = bno_dev.euler.roll;
	data_out[1] = bno_dev.euler.pitch;
	data_out[2] = bno_dev.euler.yaw;
	data_out[3] = bno_dev.quat_accuracy;
}
struct rpy_data SensorManager::getIMUData() // out of date
{
	struct rpy_data data;
	data.gyro_r = -bno_dev.gyro.x * (180.0f / M_PI);
	data.gyro_p = bno_dev.gyro.y * (180.0f / M_PI);
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
	return data; // garbage collection????
}

void SensorManager::getRawAccel(float* data_out)
{
	data_out[0] = bno_dev.accel.x;
	data_out[1] = bno_dev.accel.y;
	data_out[2] = bno_dev.accel.z; // need the raw not sensor fusion ones
	data_out[3] = bno_dev.accel.accuracy;
}

void SensorManager::getLinearAccel(float* data_out)
{
	data_out[0] = bno_dev.linear_accel.x;
	data_out[1] = bno_dev.linear_accel.y;
	data_out[2] = bno_dev.linear_accel.z; // need the raw not sensor fusion ones
	data_out[3] = bno_dev.linear_accel.accuracy;
}

struct rpy_data SensorManager::getCalibratedAccel(float* calib_bias, float* calib_scale)
{
	float raw[3];
	getRawAccel(raw);
	struct rpy_data data;
	data.accel_r = (raw[0] - calib_bias[0]) * calib_scale[0];
	data.accel_p = (raw[1] - calib_bias[1]) * calib_scale[1];
	data.accel_y = (raw[2] - calib_bias[2]) * calib_scale[2];
	return data;
}

void SensorManager::getRawMag(float* data_out)
{
	data_out[0] = bno_dev.mag.x;
	data_out[1] = bno_dev.mag.y;
	data_out[2] = bno_dev.mag.z; // need the raw not sensor fusion ones
	data_out[3] = bno_dev.mag.accuracy;
}

void SensorManager::updateGPS(SerialManager &serial)
{
    gps_parser.GPS_update(serial);
}

void SensorManager::getGPSTime(char time_str[DATA_SIZE])
{
	const char *t = gps_parser.internal_gps_storage.time;

	if(strlen(t) >= 6)
	{
		snprintf(time_str, DATA_SIZE, "%c%c:%c%c:%c%c",
				t[0], t[1], t[2], t[3], t[4], t[5]);
	}
	else
	{
		snprintf(time_str, DATA_SIZE, "00:00:00");
	}
}

float SensorManager::getGPS_alt()
{
	return gps_parser.internal_gps_storage.altitude;
}

float SensorManager::getGPS_lat()
{
	return gps_parser.internal_gps_storage.latitude;
}

float SensorManager::getGPS_lon()
{
	return gps_parser.internal_gps_storage.longitude;
}

int SensorManager::getGPS_sat()
{
	return gps_parser.internal_gps_storage.sats;
}

float SensorManager::getGPS_cog()
{
	return gps_parser.internal_gps_storage.cog_true;
}

float SensorManager::getGPS_rms()
{
	return gps_parser.internal_gps_storage.rms_range;
}

float SensorManager::getGPS_sog()
{
	return gps_parser.internal_gps_storage.sog_ms;
}

bool SensorManager::GPS_dataReady()
{
	return gps_parser.internal_gps_storage.data_ready;
}

void SensorManager::GPS_dataReadyOff()
{
	gps_parser.internal_gps_storage.data_ready = false;
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

void SensorManager::activate_egg_release()
{
	writeEggServo(90);
}

void SensorManager::activate_wing_deployment()
{
	writeMotor(0, 4000);
}

void SensorManager::activate_nosecone_release()
{
	writeNoseconeServo(-30);
}
void SensorManager::activate_probe_release()
{
	writeContainerServo(90);
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

void SensorManager::writeElevatorServoPPM(uint16_t val)
{
	servo_elevator.SetPPMPulseWidth(val);
}

void SensorManager::writeAileronServoPPM(uint16_t val)
{
	servo_aileron.SetPPMPulseWidth(val);
}

void SensorManager::writeEggServo(float val)
{
	servo_egg.SetAngle(val);
}

void SensorManager::writeMotor(uint8_t dir, uint32_t time_ms)
{
	motor.motor_run(dir, time_ms);
}

void SensorManager::stopMotor()
{
	motor.motor_stop();
}

void SensorManager::updateMotor()
{
	motor.motor_update();
}

void SensorManager::EEPROM_writeBytes(uint32_t addr, const uint8_t *data, uint32_t len)
{
	if (eeprom_dev == nullptr) return;

	// Write over multiple pages (256)
	while(len > 0)
	{
		uint32_t space = EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE);
		uint32_t chunk = (len < space) ? len : space;
		eeprom_dev->WriteByteArray(addr, const_cast<uint8_t*>(data), static_cast<uint16_t>(chunk));
		addr += chunk;
		data += chunk;
		len  -= chunk;
	}
}

void SensorManager::EEPROM_saveRecovery()
{
	if (eeprom_dev == nullptr) return;

	uint8_t buf[1 + sizeof(recovery_data)];
	buf[0] = EEPROM_MAGIC; // Mark data as saved
	memcpy(&buf[1], &recovery_cache, sizeof(recovery_data));

	EEPROM_writeBytes(EEPROM_ADDR_RECOVERY, buf, sizeof(buf));
}

uint32_t SensorManager::EEPROM_readLogLen()
{
	if (eeprom_dev == nullptr) return 0;

	uint8_t buff[4] = {0};

	eeprom_dev->ReadByteArray(EEPROM_ADDR_LOG_LEN, buff, 4);

	return static_cast<uint32_t>(buff[0])          |
			(static_cast<uint32_t>(buff[1]) << 8)  |
			(static_cast<uint32_t>(buff[2]) << 16) |
			(static_cast<uint32_t>(buff[3]) << 24);
}

void SensorManager::EEPROM_writeLogLen(uint32_t len)
{
	uint8_t buff[4] = {
			static_cast<uint8_t>(len),
			static_cast<uint8_t>(len >> 8),
			static_cast<uint8_t>(len >> 16),
			static_cast<uint8_t>(len >> 24)
	};
	EEPROM_writeBytes(EEPROM_ADDR_LOG_LEN, buff, 4);
}

void SensorManager::EEPROM_updateAltitude(float alt)
{
	recovery_cache.launch_altitude = alt;
	EEPROM_saveRecovery();
}

void SensorManager::EEPROM_updateState(OperatingState state)
{
	recovery_cache.state = state;
	EEPROM_saveRecovery();
}

void SensorManager::EEPROM_updateMode(OperatingMode mode)
{
	recovery_cache.mode = mode;
	EEPROM_saveRecovery();
}

void SensorManager::EEPROM_updatePackets(int count)
{
	recovery_cache.packet_count = count;
}

void SensorManager::EEPROM_updateMaxAlt(float alt)
{
    recovery_cache.max_alt = alt;
}

void SensorManager::EEPROM_updateEggRel()
{
    recovery_cache.egg_flag = true;
    EEPROM_saveRecovery();
}

void SensorManager::EEPROM_updateWingRel()
{
    recovery_cache.wing_flag = true;
    EEPROM_saveRecovery();
}

void SensorManager::EEPROM_updateProbeRel()
{
    recovery_cache.probe_flag = true;
    EEPROM_saveRecovery();
}

void SensorManager::EEPROM_updateNoseconeRel()
{
    recovery_cache.nosecone_flag = true;
    EEPROM_saveRecovery();
}

void SensorManager::EEPROM_resetData()
{
	recovery_cache.egg_flag = false;
	recovery_cache.wing_flag = false;
	recovery_cache.probe_flag = false;
	recovery_cache.nosecone_flag = false;
	recovery_cache.max_alt = 0.0f;
	EEPROM_saveRecovery();
}

void SensorManager::EEPROM_resetLog()
{
	eeprom_log_len = 0;
}

struct recovery_data SensorManager::EEPROM_getRecoveryData()
{
	recovery_cache.launch_altitude = 0.0;
	recovery_cache.state           = OperatingState::IDLE;
	recovery_cache.mode            = OperatingMode::OPMODE_FLIGHT;
	recovery_cache.packet_count    = 0;
	recovery_cache.max_alt         = 0.0;
	recovery_cache.nosecone_flag   = false;
	recovery_cache.probe_flag      = false;
	recovery_cache.wing_flag       = false;
	recovery_cache.egg_flag        = false;

	if(eeprom_dev == nullptr) return recovery_cache;
 
    uint8_t buff[1 + sizeof(recovery_data)];

    eeprom_dev->ReadByteArray(EEPROM_ADDR_RECOVERY, buff, sizeof(buff));

    // Check data was written
    if(buff[0] == EEPROM_MAGIC)
    {
    	memcpy(&recovery_cache, &buff[1], sizeof(recovery_data));
    }

    return recovery_cache;
}

void SensorManager::EEPROM_Init()
{
	eeprom_log_len = EEPROM_readLogLen();
	if(eeprom_log_len > EEPROM_LOG_MAX) eeprom_log_len = 0;
}

bool SensorManager::EEPROM_addLogLine(char *buffer)
{
	if(buffer == nullptr || eeprom_dev == nullptr) return false;

	uint16_t line_len = static_cast<uint16_t>(strlen(buffer));

	if(eeprom_log_len + line_len > EEPROM_LOG_MAX) return false;

	EEPROM_writeBytes(EEPROM_ADDR_LOG + eeprom_log_len, reinterpret_cast<const uint8_t *>(buffer), line_len);

	eeprom_log_len += line_len;

	EEPROM_writeLogLen(eeprom_log_len);
	return true;
}

void SensorManager::EEPROM_replayLog(uint32_t line_delay_ms, SerialManager &serial)
{
	serial.sendLogBegin();
	HAL_Delay(500);

	if(eeprom_dev == nullptr || eeprom_log_len == 0)
	{
    	serial.sendLogEnd();
        return;
	}
 
    char line_buf[DATA_BUFF_SIZE];
    uint16_t line_pos = 0;
 
    for (uint32_t offset = 0; offset < eeprom_log_len; offset++)
    {
		uint8_t byte = eeprom_dev->ReadByte(EEPROM_ADDR_LOG + offset);
 
        if (byte == '\r' || line_pos >= (DATA_BUFF_SIZE - 1))
        {
            /* Null-terminate and dispatch the completed line */
            line_buf[line_pos] = '\0';
 
            if (line_pos > 0)
            {
				serial.sendLogLine(line_buf);
				serial.sendLogLine("\r");
                HAL_Delay(line_delay_ms);
            }
 
            line_pos = 0;
        }
        else
        {
            line_buf[line_pos++] = static_cast<char>(byte);
        }
    }
    serial.sendLogEnd();
}

void SensorManager::ground_runcam_start()
{
	ground_camera.startRecording();
}

void SensorManager::ground_runcam_stop()
{
	ground_camera.stopRecording();
}

void SensorManager::payload_runcam_start()
{
	payload_camera.startRecording();
}

void SensorManager::payload_runcam_stop()
{
	payload_camera.stopRecording();
}

void SensorManager::startSensors(SerialManager &serial, I2C_HandleTypeDef *hi2c1,
		SPI_HandleTypeDef *hspi_eeprom, GPIO_TypeDef *cs_port, uint16_t cs_pin,
		TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4
		)
{
	/* Start all sensors that need to be started
	 * Add a delay between each start and send an
	 * info message */
    
    gps_parser.GPS_Init(hi2c1);

	if (!gps_parser.GPS_probe())
	{
		serial.sendErrorMsg("GPS Init failed");
	}

    HAL_Delay(100);

	if(!DS1307_Init(hi2c1))
	{
		serial.sendErrorMsg("RTC Init failed");
	}

	HAL_Delay(100);

	if(!INA219setup(MAX_EXP_CURRENT_A, 0.1, 0))
	{
		serial.sendErrorMsg("INA Init failed");
	}

	HAL_Delay(100);

	if(BMP5_Init(&bmp_dev, hi2c1, BMP5_I2C_ADDR_FIRST))
	{
		serial.sendErrorMsg("BMP Init failed");
	}

	HAL_Delay(100);

	if(BMP5_Start_Mode(&bmp_dev, 1, BMP5_ODR_120HZ, BMP5_OSR_X4, BMP5_OSR_X1))
	{
		serial.sendErrorMsg("BMP Start Mode Init Failed");
	}

	HAL_Delay(100);

	if(BNO085_Init(&bno_dev, hi2c1, BNO085_I2C_ADDR_DEFAULT) != BNO085_OK)
	{
		serial.sendErrorMsg("BN0 Init failed");
	}

	BNO_enableGyro(5000, serial);
	//BNO_enableAccel(5000, serial);
	BNO_enableLinearAcceleration(5000, serial);
	BNO_enableMag(10000, serial);
	BNO_enableRotationVector(10000, serial);

	HAL_Delay(100);

	static EEPROMsimple eeprom_storage(hspi_eeprom, cs_port, cs_pin);
	eeprom_dev = &eeprom_storage;

	uint8_t eeprom_status = eeprom_dev->ReadStatus();
	if (eeprom_status == 0xFF)
	{
		serial.sendErrorMsg("[EEPROM] SPI not responding (0xFF)");
	}
	EEPROM_Init();

	HAL_Delay(100);

	servo_nosecone.Init(htim4, TIM_CHANNEL_2, 500, 2500, 90, -90);
	servo_container.Init(htim4, TIM_CHANNEL_1, 500, 2500, 90, -90);
	servo_elevator.Init(htim3, TIM_CHANNEL_1, 500, 2500, 90, -90);
	servo_aileron.Init(htim3, TIM_CHANNEL_2, 500, 2500, 90, -90);
	servo_egg.Init(htim3, TIM_CHANNEL_3, 500, 2500, 90, -90);

	HAL_Delay(100);

	ground_camera.Init(CameraID::GROUND_CAMERA);
	payload_camera.Init(CameraID::PAYLOAD_CAMERA);

	HAL_Delay(100);

	serial.sendInfoMsg("Sensor initialization complete.");
}
