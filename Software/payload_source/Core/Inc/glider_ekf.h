#ifndef GLIDER_EKF_H_
#define GLIDER_EKF_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <arm_math.h>
#include <stdbool.h>
#include <math.h>

extern float home_lat_rad;
extern float home_lon_rad;
extern float home_alt_m;
extern bool  is_home_set;

void quat_to_rpy(const float32_t* q, float32_t* rpy);
void ekf_get_pos(float32_t* pos_out);
void ekf_get_vel(float32_t* vel_out);
void ekf_get_quaternion(float32_t* quat_out);

void CPL_IMU_to_NED(float32_t* accel, float32_t* quat);

// ============================================================================
// PUBLIC FUNCTION DECLARATIONS
// ============================================================================

/**
 * @brief Initializes the EKF state vectors, covariance matrices, and noise profiles.
 * Call this once during your STM32 setup phase before the main loop.
 */
void glider_ekf_init(void);

/**
 * @brief High-rate state propagation step using IMU inputs.
 * @param raw_accel Pointer to a 3-element float array containing [ax, ay, az] in m/s^2.
 * @param raw_gyro  Pointer to a 3-element float array containing [wx, wy, wz] in rad/s.
 * @param dt        The time delta since the last IMU sample in seconds (e.g., 0.01f for 100Hz).
 */
void glider_ekf_predict(float32_t* raw_accel, float32_t* raw_gyro, float32_t dt);

void glider_ekf_predict_bno_mode(float32_t* raw_accel, float32_t dt);

void glider_ekf_update_bno_quaternion(float32_t* bno_q, float32_t r_noise);

/**
 * @brief Asynchronous correction step using Barometer data.
 * @param baro_alt The relative or absolute altitude measured by the barometer in meters.
 * @param r_noise  The measurement noise variance of the barometer (from datasheet/tuning).
 */
void glider_ekf_update_baro(float32_t baro_alt, float32_t r_noise);

void ekf_gps_update(double lat, double lon, float alt, float sog_ms, float cog_true, float rms_range);

/**
 * @brief Asynchronous correction step using GPS data.
 * @param gps_pos_ne  Pointer to a 2-element float array containing [Position_North, Position_East] in meters.
 * @param gps_vel_ned Pointer to a 3-element float array containing [Vel_North, Vel_East, Vel_Down] in m/s.
 * @param r_pos       The position measurement noise variance.
 * @param r_vel       The velocity measurement noise variance.
 */
void glider_ekf_update_gps(const float32_t* gps_pos_ne, const float32_t* gps_vel_ned, float32_t r_pos, float32_t r_vel);

/**
 * @brief Asynchronous correction step using the Magnetic Compass.
 * @param compass_yaw_rad The tilt-compensated absolute heading in radians (-PI to +PI).
 * @param r_noise         The heading measurement noise variance.
 */
void glider_ekf_update_compass(float32_t compass_yaw_rad, float32_t r_noise);

void convert_gps_to_local_ned2(float curr_lat, float curr_lon, float curr_alt, float* current_pos_ne);

#ifdef __cplusplus
}
#endif

#endif /* GLIDER_EKF_H_ */
