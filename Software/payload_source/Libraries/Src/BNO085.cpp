#include "bno085.hpp"
#include <string.h>
#include <math.h>

/* ========== Private constants ========== */
#define BNO085_I2C_TIMEOUT		100		// HAL I2C timeout (ms)
#define SHTP_HEADER_SIZE		4
#define MAX_PACKET_SIZE			512
#define TIMESTAMP_RECORD_SIZE	5		// [0xFB][ts0][ts1][ts2][ts3]

/* ========== Private variables ========== */

// I/O buffers
static uint8_t shtpHeader[SHTP_HEADER_SIZE];
static uint8_t shtpData[MAX_PACKET_SIZE];
static uint8_t s_txBuf[SHTP_HEADER_SIZE + MAX_PACKET_SIZE];
static uint8_t s_rxBuf[SHTP_HEADER_SIZE + MAX_PACKET_SIZE];

/* ========== Private functions ========== */

static BNO085_Status_t BNO085_WaitForPacket(BNO085_t *bno, uint8_t channel, uint8_t report_id, uint32_t timeout_ms);
static BNO085_Status_t BNO085_ReceivePacket(BNO085_t *bno);
static BNO085_Status_t BNO085_SendPacket(BNO085_t *bno, uint8_t channel, uint8_t data_length);
static BNO085_Status_t BNO085_SetFeatureCommand(BNO085_t *bno, uint8_t report_id, uint32_t micro_between_reports, uint32_t specific_config);
static void            BNO085_ParseInputReport(BNO085_t *bno, uint16_t cargo_length);
static bool            BNO085_ParseReport(BNO085_t *bno, uint8_t *rpt);
static uint16_t        BNO085_GetU16(const uint8_t *d);
static uint32_t        BNO085_GetU32(const uint8_t *d);
static float           BNO085_QToFloat(int16_t val, uint8_t q);


/* ========== Function implementations ========== */

/**
 * 	@brief Initialise the BNO085 and confirm comms via a Product ID request
 */
BNO085_Status_t BNO085_Init(BNO085_t *bno, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr) {
	if(bno == NULL || hi2c == NULL) { return BNO085_ERROR; }

	memset(bno, 0, sizeof(BNO085_t));
	bno->hi2c        = hi2c;
	bno->i2c_address = (uint8_t)(i2c_addr << 1);	// HAL expects 8-bit form

	HAL_Delay(300);	// Wait for BNO085 power-on boot to complete

	// WaitForPacket inside GetProductID drains all queued boot packets until the 0xF8 response appears
	return BNO085_GetProductID(bno);
}

bool BNO085_DataReady(BNO085_t *bno) {
	if(bno == NULL) { return false; }
	return HAL_GPIO_ReadPin(IMU_INT_GPIO_Port, IMU_INT_Pin) == GPIO_PIN_RESET;
}

/**
 * 	@brief Send a soft reset on the executable channel
 */
BNO085_Status_t BNO085_SoftReset(BNO085_t *bno) {
	if(bno == NULL) { return BNO085_ERROR; }
	shtpData[0] = 1;
	return BNO085_SendPacket(bno, CHANNEL_EXECUTABLE, 1);
}

/**
 * 	@brief Poll for new data — returns true if a recognised report was decoded
 */
bool BNO085_DataAvailable(BNO085_t *bno) {
	if(bno == NULL) { return false; }
	bno->data_available = false;

	if(BNO085_ReceivePacket(bno) == BNO085_OK) {
		uint16_t cargo = bno->packet_length > SHTP_HEADER_SIZE
		                 ? (uint16_t)(bno->packet_length - SHTP_HEADER_SIZE) : 0U;
		BNO085_ParseInputReport(bno, cargo);
	}
	return bno->data_available;
}

/**
 * 	@brief Read and parse one packet
 */
BNO085_Status_t BNO085_GetData(BNO085_t *bno) {
	if(bno == NULL) { return BNO085_ERROR; }
	bno->data_available = false;

	if(BNO085_ReceivePacket(bno) != BNO085_OK) { return BNO085_ERROR; }

	uint16_t cargo = bno->packet_length > SHTP_HEADER_SIZE
	                 ? (uint16_t)(bno->packet_length - SHTP_HEADER_SIZE) : 0U;
	BNO085_ParseInputReport(bno, cargo);
	//printf("[BNO] ch=%d len=%d d0=%02X d1=%02X d2=%02X d5=%02X\n",
	       //shtpHeader[2], bno->packet_length,
	       //shtpData[0], shtpData[1], shtpData[2], shtpData[5]);
	return BNO085_OK;
}

// --- Sensor enable ---

/**
 * 	@brief Enable rotation vector output (fused accel + gyro + mag)
 */
BNO085_Status_t BNO085_EnableRotationVector(BNO085_t *bno, uint32_t us)
{ return BNO085_SetFeatureCommand(bno, SENSOR_REPORTID_ROTATION_VECTOR, us, 0); }

/**
 * 	@brief Enable game rotation vector output (fused accel + gyro, no mag)
 */
BNO085_Status_t BNO085_EnableGameRotationVector(BNO085_t *bno, uint32_t us)
{ return BNO085_SetFeatureCommand(bno, SENSOR_REPORTID_GAME_ROTATION_VECTOR, us, 0); }

/**
 * 	@brief Enable accelerometer output
 */
BNO085_Status_t BNO085_EnableAccelerometer(BNO085_t *bno, uint32_t us)
{ return BNO085_SetFeatureCommand(bno, SENSOR_REPORTID_ACCELEROMETER, us, 0); }

/**
 * 	@brief Enable linear acceleration output (compensated gravity)
 */
BNO085_Status_t BNO085_EnableLinearAcceleration(BNO085_t *bno, uint32_t us)
{ return BNO085_SetFeatureCommand(bno, SENSOR_REPORTID_LINEAR_ACCELERATION, us, 0); }

/**
 * 	@brief Enable gyroscope output
 */
BNO085_Status_t BNO085_EnableGyro(BNO085_t *bno, uint32_t us)
{ return BNO085_SetFeatureCommand(bno, SENSOR_REPORTID_GYROSCOPE_CALIBRATED, us, 0); }

/**
 * 	@brief Enable magnetometer output
 */
BNO085_Status_t BNO085_EnableMagnetometer(BNO085_t *bno, uint32_t us)
{ return BNO085_SetFeatureCommand(bno, SENSOR_REPORTID_MAGNETIC_FIELD, us, 0); }

/**
 * 	@brief Enable gravity vector output
 */
BNO085_Status_t BNO085_EnableGravity(BNO085_t *bno, uint32_t us)
{ return BNO085_SetFeatureCommand(bno, SENSOR_REPORTID_GRAVITY, us, 0); }

// --- Data access ---

/**
 * 	@brief Get latest accelerometer values (m/s²)
 */
BNO085_Status_t BNO085_GetAccel(BNO085_t *bno, float *x, float *y, float *z, float *accuracy) {
	if(!bno || !x || !y || !z) { return BNO085_ERROR; }
	*x = bno->accel.x;  *y = bno->accel.y;  *z = bno->accel.z;
	if(accuracy) { *accuracy = bno->accel.accuracy; }
	return BNO085_OK;
}

/**
 * 	@brief Get latest gyroscope values (rad/s)
 */
BNO085_Status_t BNO085_GetGyro(BNO085_t *bno, float *x, float *y, float *z, float *accuracy) {
	if(!bno || !x || !y || !z) { return BNO085_ERROR; }
	*x = bno->gyro.x;  *y = bno->gyro.y;  *z = bno->gyro.z;
	if(accuracy) { *accuracy = bno->gyro.accuracy; }
	return BNO085_OK;
}

/**
 * 	@brief Get latest magnetometer values (µT)
 */
BNO085_Status_t BNO085_GetMag(BNO085_t *bno, float *x, float *y, float *z, float *accuracy) {
	if(!bno || !x || !y || !z) { return BNO085_ERROR; }
	*x = bno->mag.x;  *y = bno->mag.y;  *z = bno->mag.z;
	if(accuracy) { *accuracy = bno->mag.accuracy; }
	return BNO085_OK;
}

/**
 * 	@brief Get latest linear acceleration values (m/s², gravity removed)
 */
BNO085_Status_t BNO085_GetLinearAccel(BNO085_t *bno, float *x, float *y, float *z, float *accuracy) {
	if(!bno || !x || !y || !z) { return BNO085_ERROR; }
	*x = bno->linear_accel.x;  *y = bno->linear_accel.y;  *z = bno->linear_accel.z;
	if(accuracy) { *accuracy = bno->linear_accel.accuracy; }
	return BNO085_OK;
}

/**
 * 	@brief Get latest gravity vector (m/s²)
 */
BNO085_Status_t BNO085_GetGravity(BNO085_t *bno, float *x, float *y, float *z, float *accuracy) {
	if(!bno || !x || !y || !z) { return BNO085_ERROR; }
	*x = bno->gravity.x;  *y = bno->gravity.y;  *z = bno->gravity.z;
	if(accuracy) { *accuracy = bno->gravity.accuracy; }
	return BNO085_OK;
}

/**
 * 	@brief Get latest quaternion orientation
 */
BNO085_Status_t BNO085_GetQuaternion(BNO085_t *bno, BNO085_Quaternion_t *quat) {
	if(!bno || !quat) { return BNO085_ERROR; }
	*quat = bno->quat;
	return BNO085_OK;
}

/**
 * 	@brief Get latest Euler angles (rad)
 */
BNO085_Status_t BNO085_GetEuler(BNO085_t *bno, float *roll, float *pitch, float *yaw) {
	if(!bno || !roll || !pitch || !yaw) { return BNO085_ERROR; }
	*roll = bno->euler.roll;  *pitch = bno->euler.pitch;  *yaw = bno->euler.yaw;
	return BNO085_OK;
}

/**
 * 	@brief Convert a quaternion to Euler angles (ZYX - rad)
 */
void BNO085_QuaternionToEuler(BNO085_Quaternion_t *quat, BNO085_Euler_t *euler) {
	if(!quat || !euler) { return; }

	float w = quat->real, x = quat->i, y = quat->j, z = quat->k;

	float sinr_cosp = 2.0f * (w * x + y * z);
	float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
	euler->roll = atan2f(sinr_cosp, cosr_cosp);

	float sinp = 2.0f * (w * y - z * x);
	euler->pitch = (fabsf(sinp) >= 1.0f)
	               ? copysignf((float)M_PI / 2.0f, sinp) : asinf(sinp);

	float siny_cosp = 2.0f * (w * z + x * y);
	float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
	euler->yaw = atan2f(siny_cosp, cosy_cosp);
}

// --- Calibration / configuration ---

/**
 * 	@brief Start sensor calibration
 */
BNO085_Status_t BNO085_Calibrate(BNO085_t *bno, uint8_t sensor) {
	if(!bno) { return BNO085_ERROR; }
	memset(shtpData, 0, 12);
	shtpData[0] = SHTP_REPORT_COMMAND_REQUEST;
	shtpData[1] = bno->sequence_number[CHANNEL_CONTROL]++;
	shtpData[2] = COMMAND_ME_CALIBRATE;
	shtpData[3] = sensor;	// P0 per SH-2 reference manual
	return BNO085_SendPacket(bno, CHANNEL_CONTROL, 12);
}

/**
 * 	@brief Save the current calibration data to flash
 */
BNO085_Status_t BNO085_SaveCalibration(BNO085_t *bno) {
	if(!bno) { return BNO085_ERROR; }
	memset(shtpData, 0, 12);
	shtpData[0] = SHTP_REPORT_COMMAND_REQUEST;
	shtpData[1] = bno->sequence_number[CHANNEL_CONTROL]++;
	shtpData[2] = COMMAND_DCD;
	return BNO085_SendPacket(bno, CHANNEL_CONTROL, 12);
}

/**
 * @brief Disable the onboard background calibration routine for all sensors
 */
BNO085_Status_t BNO085_DisableCalibration(BNO085_t *bno) {
    if(!bno) { return BNO085_ERROR; }
    
    memset(shtpData, 0, 12);
    shtpData[0] = SHTP_REPORT_COMMAND_REQUEST;
    shtpData[1] = bno->sequence_number[CHANNEL_CONTROL]++;
    shtpData[2] = COMMAND_ME_CALIBRATE;
    
    // Bytes 3-5 are the sub-parameters (P0, P1, P2) for the calibration command.
    // Setting them all to 0 disables calibration for Accelerometer, Gyroscope, and Magnetometer.
    shtpData[3] = 0; // P0: Accel (Bit 0), Gyro (Bit 1), Mag (Bit 2) -> 0 disables all
    shtpData[4] = 0; // P1: Planar Accelerometer -> 0 disables
    shtpData[5] = 0; // P2: Reserved / On-table calibration -> 0 disables
    
    return BNO085_SendPacket(bno, CHANNEL_CONTROL, 12);
}

/**
 * 	@brief Request Product ID
 */
BNO085_Status_t BNO085_GetProductID(BNO085_t *bno) {
	if(!bno) { return BNO085_ERROR; }
	shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST;
	shtpData[1] = 0;
	if(BNO085_SendPacket(bno, CHANNEL_CONTROL, 2) != BNO085_OK) { return BNO085_ERROR; }
	return BNO085_WaitForPacket(bno, CHANNEL_CONTROL, SHTP_REPORT_PRODUCT_ID_RESPONSE, 10000);
}

/**
 * 	@brief Tare — re-zero all motion outputs to the current heading
 * 	@param axis_sel              TARE_AXIS_ALL or TARE_AXIS_Z_ONLY
 * 	@param rotation_vector_basis Report ID of the rotation vector to use as the tare basis
 */
BNO085_Status_t BNO085_Tare(BNO085_t *bno, uint8_t axis_sel, uint8_t rotation_vector_basis) {
	if(!bno) { return BNO085_ERROR; }
	memset(shtpData, 0, 12);
	shtpData[0] = SHTP_REPORT_COMMAND_REQUEST;
	shtpData[1] = bno->sequence_number[CHANNEL_CONTROL]++;
	shtpData[2] = COMMAND_TARE;
	shtpData[3] = 0;
	shtpData[4] = axis_sel;
	shtpData[5] = rotation_vector_basis;
	return BNO085_SendPacket(bno, CHANNEL_CONTROL, 12);
}

// --- Private function implementations ---

/**
 * 	@brief Poll until a packet with the expected report ID arrives
 *
 * 	Channel is not used for filtering — the BNO085 does not guarantee responses
 * 	arrive on the same channel as the request. Non-matching packets are silently
 * 	discarded, making this a natural drain for queued boot messages.
 */
static BNO085_Status_t BNO085_WaitForPacket(BNO085_t *bno, uint8_t channel, uint8_t report_id, uint32_t timeout_ms) {
	(void)channel;
	uint32_t t0 = HAL_GetTick();

	while((HAL_GetTick() - t0) < timeout_ms) {
		if(BNO085_ReceivePacket(bno) == BNO085_OK) {
			if(bno->packet_length <= SHTP_HEADER_SIZE) { HAL_Delay(1); continue; }	// Header-only, skip

			if(report_id == 0xFF || shtpData[0] == report_id) {
				uint16_t cargo = (uint16_t)(bno->packet_length - SHTP_HEADER_SIZE);
				BNO085_ParseInputReport(bno, cargo);
				return BNO085_OK;
			}
			// Non-matching — discard, read next
		}
		HAL_Delay(1);
	}
	return BNO085_TIMEOUT;
}

/**
 * 	@brief Read one SHTP packet over I2C
 *
 * 	Two-step read required by SHTP-over-I2C:
 * 	  1. Read first 4-bytes to gte packet length
 * 	  2. Full read (header + payload) in a single message (START...STOP)
 *
 * 	The BNO085 resets its byte counter on every new START condition, so step 2
 * 	always begins at byte 0.
 *
 * 	Oversized packets are consumed up to MAX_PACKET_SIZE
 * 	bytes to clear them from the device, then returned as ERROR — without this the
 * 	same packet is presented on every subsequent read, blocking all comms.
 */
static BNO085_Status_t BNO085_ReceivePacket(BNO085_t *bno) {
	// Step 1: peek header
	if(HAL_I2C_Master_Receive(bno->hi2c, bno->i2c_address, shtpHeader, SHTP_HEADER_SIZE, BNO085_I2C_TIMEOUT) != HAL_OK) {
		return BNO085_ERROR;
	}

	uint16_t pkt_len = ((uint16_t)shtpHeader[1] << 8) | shtpHeader[0];
	pkt_len &= 0x7FFF;	// Clear MSB continuation bit

	if(pkt_len < SHTP_HEADER_SIZE) { return BNO085_ERROR; }

	bno->packet_length = pkt_len;
	uint16_t cargo_len = pkt_len - SHTP_HEADER_SIZE;

	if(cargo_len == 0) { return BNO085_OK; }

	uint16_t read_cargo = (cargo_len > MAX_PACKET_SIZE) ? (uint16_t)MAX_PACKET_SIZE : cargo_len;
	uint16_t total      = SHTP_HEADER_SIZE + read_cargo;

	// Step 2: full read in one transaction
	if(HAL_I2C_Master_Receive(bno->hi2c, bno->i2c_address, s_rxBuf, total, BNO085_I2C_TIMEOUT) != HAL_OK) {
		return BNO085_ERROR;
	}

	memcpy(shtpHeader, s_rxBuf,                    SHTP_HEADER_SIZE);
	memcpy(shtpData,   s_rxBuf + SHTP_HEADER_SIZE, read_cargo);

	return (cargo_len > MAX_PACKET_SIZE) ? BNO085_ERROR : BNO085_OK;
}

/**
 * 	@brief Transmit one SHTP packet over I2C
 */
static BNO085_Status_t BNO085_SendPacket(BNO085_t *bno, uint8_t channel, uint8_t data_length) {
	uint16_t total = (uint16_t)data_length + SHTP_HEADER_SIZE;

	s_txBuf[0] = (uint8_t)(total & 0xFF);
	s_txBuf[1] = (uint8_t)(total >> 8);
	s_txBuf[2] = channel;
	s_txBuf[3] = bno->sequence_number[channel]++;

	memcpy(&s_txBuf[SHTP_HEADER_SIZE], shtpData, data_length);

	HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(bno->hi2c, bno->i2c_address, s_txBuf, total, BNO085_I2C_TIMEOUT);
	return (st == HAL_OK) ? BNO085_OK : BNO085_ERROR;
}

/**
 * 	@brief Build and send a Set Feature Command for the given sensor report ID
 */
static BNO085_Status_t BNO085_SetFeatureCommand(BNO085_t *bno, uint8_t report_id, uint32_t micro_between_reports, uint32_t specific_config) {
	if(!bno) { return BNO085_ERROR; }

	const uint32_t batch_interval = 0;

	shtpData[0]  = SHTP_REPORT_SET_FEATURE_COMMAND;
	shtpData[1]  = report_id;
	shtpData[2]  = 0;
	shtpData[3]  = 0;
	shtpData[4]  = 0;
	shtpData[5]  = (uint8_t)((micro_between_reports >>  0) & 0xFF);
	shtpData[6]  = (uint8_t)((micro_between_reports >>  8) & 0xFF);
	shtpData[7]  = (uint8_t)((micro_between_reports >> 16) & 0xFF);
	shtpData[8]  = (uint8_t)((micro_between_reports >> 24) & 0xFF);
	shtpData[9]  = (uint8_t)((batch_interval        >>  0) & 0xFF);
	shtpData[10] = (uint8_t)((batch_interval        >>  8) & 0xFF);
	shtpData[11] = (uint8_t)((batch_interval        >> 16) & 0xFF);
	shtpData[12] = (uint8_t)((batch_interval        >> 24) & 0xFF);
	shtpData[13] = (uint8_t)((specific_config       >>  0) & 0xFF);
	shtpData[14] = (uint8_t)((specific_config       >>  8) & 0xFF);
	shtpData[15] = (uint8_t)((specific_config       >> 16) & 0xFF);
	shtpData[16] = (uint8_t)((specific_config       >> 24) & 0xFF);

	return BNO085_SendPacket(bno, CHANNEL_CONTROL, 17);
}

/**
 * 	@brief Iterate through one SHTP payload, handling BASE_TIMESTAMP (0xFB) wrappers
 *
 * 	Every sensor report on CHANNEL_REPORTS is preceded by a 5-byte timestamp record.
 * 	Control-channel responses arrive without a timestamp. One packet may contain
 * 	multiple [timestamp + report] pairs.
 */
static void BNO085_ParseInputReport(BNO085_t *bno, uint16_t cargo_length) {
    if(!bno || cargo_length == 0) { return; }

    uint16_t offset = 0;
    while(offset < cargo_length) {
        uint8_t id = shtpData[offset];

        if(id == SHTP_REPORT_BASE_TIMESTAMP) {     // 0xFB: skip the 5-byte timestamp
            offset += TIMESTAMP_RECORD_SIZE;
            continue;
        }
        if(id == SHTP_REPORT_PRODUCT_ID_RESPONSE ||
           id == SHTP_REPORT_COMMAND_RESPONSE     ||
           id == SHTP_REPORT_GET_FEATURE_RESPONSE) {
            BNO085_ParseReport(bno, &shtpData[offset]);
            break;
        }

        if(BNO085_ParseReport(bno, &shtpData[offset])) { bno->data_available = true; }

        // advance by this report's on-wire length
        switch(id) {
            case SENSOR_REPORTID_ROTATION_VECTOR:
            case SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR: offset += 14; break;
            case SENSOR_REPORTID_GAME_ROTATION_VECTOR:        offset += 12; break;  // no accuracy field
            case SENSOR_REPORTID_ACCELEROMETER:
            case SENSOR_REPORTID_LINEAR_ACCELERATION:
            case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
            case SENSOR_REPORTID_MAGNETIC_FIELD:
            case SENSOR_REPORTID_GRAVITY:                     offset += 10; break;
            default:
                return;   // unknown id mid-packet: bail to avoid runaway
        }
    }
}

/**
 * 	@brief Parse a report starting at *rpt
 *
 * 	3-axis layout:          [0] ID  [1] seq  [2] status  [3] delay  [4-9] xyz (Q-scaled int16 LE)
 * 	Rotation vector layout: [0] ID  [1] seq  [2] status  [3] delay, [4-11] ijkr  [12-13] accuracy (Q12)
 */
static bool BNO085_ParseReport(BNO085_t *bno, uint8_t *rpt) {
	if(!bno || !rpt) { return false; }

	uint8_t report_id = rpt[0];
	uint8_t status    = rpt[2] & 0x03;

	switch(report_id) {

		case SENSOR_REPORTID_ACCELEROMETER:
			bno->accel.x        = -BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[4]), 8);
			bno->accel.y        = -BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[6]), 8);
			bno->accel.z        = -BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[8]), 8);
			bno->accel.accuracy = (float)status;
			return true;

		case SENSOR_REPORTID_LINEAR_ACCELERATION:
			bno->linear_accel.x        = BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[4]), 8);
			bno->linear_accel.y        = BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[6]), 8);
			bno->linear_accel.z        = BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[8]), 8);
			bno->linear_accel.accuracy = (float)status;
			return true;

		case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
			bno->gyro.x        = BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[4]), 9);
			bno->gyro.y        = BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[6]), 9);
			bno->gyro.z        = BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[8]), 9);
			bno->gyro.accuracy = (float)status;
			return true;

		case SENSOR_REPORTID_MAGNETIC_FIELD:
			bno->mag.x        = BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[4]), 4);
			bno->mag.y        = BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[6]), 4);
			bno->mag.z        = BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[8]), 4);
			bno->mag.accuracy = (float)status;
			return true;

		case SENSOR_REPORTID_GRAVITY:
			bno->gravity.x        = BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[4]), 8);
			bno->gravity.y        = BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[6]), 8);
			bno->gravity.z        = BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[8]), 8);
			bno->gravity.accuracy = (float)status;
			return true;

		case SENSOR_REPORTID_ROTATION_VECTOR:
		case SENSOR_REPORTID_GAME_ROTATION_VECTOR:
		case SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR:
			bno->quat.i        = BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[4]),  14);
			bno->quat.j        = BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[6]),  14);
			bno->quat.k        = BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[8]),  14);
			bno->quat.real     = BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[10]), 14);
			bno->quat.accuracy = BNO085_QToFloat((int16_t)BNO085_GetU16(&rpt[12]), 12);

			bno->quat_accuracy = status;
			BNO085_QuaternionToEuler(&bno->quat, &bno->euler);
			return true;

		case SHTP_REPORT_PRODUCT_ID_RESPONSE:
			bno->product_id.reset_cause      = rpt[1];
			bno->product_id.sw_version_major = rpt[2];
			bno->product_id.sw_version_minor = rpt[3];
			bno->product_id.sw_part_number   = BNO085_GetU32(&rpt[4]);
			bno->product_id.sw_build_number  = BNO085_GetU32(&rpt[8]);
			bno->product_id.sw_version_patch = (uint16_t)rpt[12] | ((uint16_t)rpt[13] << 8);
			return true;

		default: { return false; }
	}
}

static uint16_t BNO085_GetU16(const uint8_t *d) {
	return (uint16_t)d[0] | ((uint16_t)d[1] << 8);
}

static uint32_t BNO085_GetU32(const uint8_t *d) {
	return (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
}

static float BNO085_QToFloat(int16_t val, uint8_t q) {
	return (float)val * SCALE_Q(q);
}
