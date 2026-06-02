#ifndef BNO085_H
#define BNO085_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#include "stm32g4xx_hal.h"

/* ========== Defines ========== */

// I2C addresses (7-bit)
#define BNO085_I2C_ADDR_DEFAULT		0x4A
#define BNO085_I2C_ADDR_ALT			0x4B

// SHTP channel IDs
#define CHANNEL_COMMAND				0
#define CHANNEL_EXECUTABLE			1
#define CHANNEL_CONTROL				2
#define CHANNEL_REPORTS				3
#define CHANNEL_WAKE_REPORTS		4
#define CHANNEL_GYRO				5

// SHTP report IDs
#define SHTP_REPORT_PRODUCT_ID_REQUEST		0xF9
#define SHTP_REPORT_PRODUCT_ID_RESPONSE		0xF8
#define SHTP_REPORT_FRS_READ_REQUEST		0xF4
#define SHTP_REPORT_FRS_READ_RESPONSE		0xF3
#define SHTP_REPORT_COMMAND_REQUEST			0xF2
#define SHTP_REPORT_COMMAND_RESPONSE		0xF1
#define SHTP_REPORT_GET_FEATURE_REQUEST		0xFE
#define SHTP_REPORT_SET_FEATURE_COMMAND		0xFD
#define SHTP_REPORT_GET_FEATURE_RESPONSE	0xFC
#define SHTP_REPORT_BASE_TIMESTAMP			0xFB

// Sensor report IDs
#define SENSOR_REPORTID_ACCELEROMETER				0x01
#define SENSOR_REPORTID_GYROSCOPE_CALIBRATED		0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD				0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION			0x04
#define SENSOR_REPORTID_ROTATION_VECTOR				0x05
#define SENSOR_REPORTID_GRAVITY						0x06
#define SENSOR_REPORTID_GYROSCOPE_UNCALIBRATED		0x07
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR		0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR	0x09
#define SENSOR_REPORTID_MAGNETIC_FIELD_UNCALIBRATED	0x0B
#define SENSOR_REPORTID_TAP_DETECTOR				0x10
#define SENSOR_REPORTID_STEP_COUNTER				0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER		0x13
#define SENSOR_REPORTID_GYRO_INTEGRATED_RV			0x2A

// Command IDs
#define COMMAND_ERRORS				1
#define COMMAND_COUNTER				2
#define COMMAND_TARE				3
#define COMMAND_INITIALIZE			4
#define COMMAND_DCD					6
#define COMMAND_ME_CALIBRATE		7
#define COMMAND_DCD_PERIOD_SAVE		9
#define COMMAND_OSCILLATOR			10
#define COMMAND_CLEAR_DCD			11

// Calibration sub-commands
#define CALIBRATE_ACCEL				0
#define CALIBRATE_GYRO				1
#define CALIBRATE_MAG				2
#define CALIBRATE_PLANAR_ACCEL		3
#define CALIBRATE_ACCEL_GYRO_MAG	4
#define CALIBRATE_STOP				5

// Tare axis selection
#define TARE_AXIS_ALL				0x07
#define TARE_AXIS_Z_ONLY			0x04

// Q-point fixed-point scaling
#define SCALE_Q(n)	(1.0f / (float)(1 << (n)))


/* ========== Enums ========== */

/**
 * 	@brief Driver return codes
 */
typedef enum {
	BNO085_OK		= 0x0,
	BNO085_ERROR	= 0x1,
	BNO085_BUSY		= 0x2,
	BNO085_TIMEOUT	= 0x3,
} BNO085_Status_t;


/* ========== Structs ========== */

/**
 * 	@brief Quaternion (i, j, k, real) + accuracy estimate (rad)
 */
typedef struct {
	float i;
	float j;
	float k;
	float real;
	float accuracy;
} BNO085_Quaternion_t;

/**
 * 	@brief Euler angles (rad)
 */
typedef struct {
	float roll;
	float pitch;
	float yaw;
} BNO085_Euler_t;

/**
 * 	@brief Generic 3-axis vector + accuracy/status field
 */
typedef struct {
	float x;
	float y;
	float z;
	float accuracy;
} BNO085_Vec3_t;

/**
 * 	@brief Product ID block returned by the sensor on request
 */
typedef struct {
	uint8_t  reset_cause;
	uint8_t  sw_version_major;
	uint8_t  sw_version_minor;
	uint32_t sw_part_number;
	uint32_t sw_build_number;
	uint16_t sw_version_patch;
} BNO085_ProductID_t;

/**
 * 	@brief Main driver handle — allocate one per physical sensor
 *
 * 	╔══════════════════╦══════════════════════════════════════════════╗
 * 	║      Field       ║                     Notes		              ║
 * 	╠══════════════════╬══════════════════════════════════════════════╣
 * 	║ accel            ║ Accelerometer (m/s²), all axes negated       ║
 * 	║ gyro             ║ Gyroscope (rad/s)                            ║
 * 	║ mag              ║ Magnetometer (µT)                            ║
 * 	║ linear_accel     ║ Linear acceleration, gravity removed (m/s²)  ║
 * 	║ gravity          ║ Gravity vector (m/s²)                        ║
 * 	║ quat             ║ Rotation vector as quaternion                ║
 * 	║ euler            ║ Roll/pitch/yaw derived from quat (rad)       ║
 * 	║ data_available   ║ Data available flag					      ║
 * 	╚══════════════════╩══════════════════════════════════════════════╝
 */
typedef struct {
	I2C_HandleTypeDef	*hi2c;
	uint8_t				i2c_address;
	uint8_t				sequence_number[6];	// Per-channel SHTP seq. counter
	uint32_t			time_stamp;

	BNO085_Vec3_t		accel;
	BNO085_Vec3_t		gyro;
	BNO085_Vec3_t		mag;
	BNO085_Vec3_t		linear_accel;
	BNO085_Vec3_t		gravity;
	BNO085_Quaternion_t	quat;
	BNO085_Euler_t		euler;

	uint8_t			quat_accuracy;		// Separate field for easier access to rotation vector accuracy

	BNO085_ProductID_t	product_id;

	uint16_t	packet_length;
	bool		data_available;
} BNO085_t;


/* ========== Functions ========== */

BNO085_Status_t BNO085_Init(BNO085_t *bno, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
BNO085_Status_t BNO085_SoftReset(BNO085_t *bno);

bool			BNO085_DataAvailable(BNO085_t *bno);
BNO085_Status_t BNO085_GetData(BNO085_t *bno);
bool            BNO085_DataReady(BNO085_t *bno);

// Sensor enable — time_between_reports in microseconds
BNO085_Status_t BNO085_EnableRotationVector(BNO085_t *bno, uint32_t time_between_reports);
BNO085_Status_t BNO085_EnableGameRotationVector(BNO085_t *bno, uint32_t time_between_reports);
BNO085_Status_t BNO085_EnableAccelerometer(BNO085_t *bno, uint32_t time_between_reports);
BNO085_Status_t BNO085_EnableLinearAcceleration(BNO085_t *bno, uint32_t time_between_reports);
BNO085_Status_t BNO085_EnableGyro(BNO085_t *bno, uint32_t time_between_reports);
BNO085_Status_t BNO085_EnableMagnetometer(BNO085_t *bno, uint32_t time_between_reports);
BNO085_Status_t BNO085_EnableGravity(BNO085_t *bno, uint32_t time_between_reports);

// Data access
BNO085_Status_t BNO085_GetAccel(BNO085_t *bno, float *x, float *y, float *z, float *accuracy);
BNO085_Status_t BNO085_GetGyro(BNO085_t *bno, float *x, float *y, float *z, float *accuracy);
BNO085_Status_t BNO085_GetMag(BNO085_t *bno, float *x, float *y, float *z, float *accuracy);
BNO085_Status_t BNO085_GetLinearAccel(BNO085_t *bno, float *x, float *y, float *z, float *accuracy);
BNO085_Status_t BNO085_GetGravity(BNO085_t *bno, float *x, float *y, float *z, float *accuracy);
BNO085_Status_t BNO085_GetQuaternion(BNO085_t *bno, BNO085_Quaternion_t *quat);
BNO085_Status_t BNO085_GetEuler(BNO085_t *bno, float *roll, float *pitch, float *yaw);

// Utility
void BNO085_QuaternionToEuler(BNO085_Quaternion_t *quat, BNO085_Euler_t *euler);

// Calibration / configuration
BNO085_Status_t BNO085_Calibrate(BNO085_t *bno, uint8_t sensor);
BNO085_Status_t BNO085_DisableCalibration(BNO085_t *bno);
BNO085_Status_t BNO085_SaveCalibration(BNO085_t *bno);
BNO085_Status_t BNO085_GetProductID(BNO085_t *bno);
BNO085_Status_t BNO085_Tare(BNO085_t *bno, uint8_t axis_sel, uint8_t rotation_vector_basis);


#ifdef __cplusplus
}
#endif

#endif // BNO085_H
