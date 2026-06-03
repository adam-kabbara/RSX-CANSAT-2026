#include "glider_ekf.h"

#define STATE_DIM 12  // Error-state size (Pos, Vel, Att_err, Accel_bias)
#define QUAT_DIM  4   // Attitude state tracking size

// ============================================================================
// GLOBAL STATE MEMORY (Static allocation to prevent stack overflow)
// ============================================================================
// State vectors
static float32_t x_pos_vel_biases[9]; // Pos(3), Vel(3), Accel_Bias(3), Gyro_Bias(2) (Yaw/Pitch/Roll parsed separately)
static float32_t x_q[QUAT_DIM] = {1.0f, 0.0f, 0.0f, 0.0f}; // Identity Quaternion

// Covariance Matrices
static float32_t P_data[STATE_DIM * STATE_DIM];
static float32_t Q_data[STATE_DIM * STATE_DIM];
static float32_t F_data[STATE_DIM * STATE_DIM];

// CMSIS-DSP Matrix Instances
static arm_matrix_instance_f32 P_mat;
static arm_matrix_instance_f32 Q_mat;
static arm_matrix_instance_f32 F_mat;

// Temporary Buffers for Prediction step (P = F*P*F^T + Q)
static float32_t F_T_data[STATE_DIM * STATE_DIM];
static float32_t FP_data[STATE_DIM * STATE_DIM];
static float32_t FPF_T_data[STATE_DIM * STATE_DIM];
static arm_matrix_instance_f32 F_T_mat;
static arm_matrix_instance_f32 FP_mat;
static arm_matrix_instance_f32 FPF_T_mat;

const float32_t G_ACCEL = 9.81f;
float home_lat_rad = 0.0f;
float home_lon_rad = 0.0f;
float home_alt_m   = 0.0f;
bool  is_home_set  = false;

const float EARTH_RADIUS = 6378137.0f; // WGS84 Earth equatorial radius in meters
const float DEG_TO_RAD   = M_PI / 180.0f;

// ============================================================================
// CORE MATH UTILITIES
// ============================================================================
static void quaternion_to_rotation_matrix(const float32_t* q, float32_t* R) {
   float32_t qw = q[0], qx = q[1], qy = q[2], qz = q[3];
   R[0] = 1.0f - 2.0f*(qy*qy + qz*qz); R[1] = 2.0f*(qx*qy - qw*qz);        R[2] = 2.0f*(qx*qz + qw*qy);
   R[3] = 2.0f*(qx*qy + qw*qz);        R[4] = 1.0f - 2.0f*(qx*qx + qz*qz); R[5] = 2.0f*(qy*qz - qw*qx);
   R[6] = 2.0f*(qx*qz - qw*qy);        R[7] = 2.0f*(qy*qz + qw*qx);        R[8] = 1.0f - 2.0f*(qx*qx + qy*qy);
}

void CPL_IMU_to_NED(float32_t* accel, float32_t* quat) {
   // IMU is mounted with X backward, Y right, Z up
   // NED frame: X North, Y East, Z Down
   accel[0] = -accel[0]; // Backward to North
   accel[2] = -accel[2]; // Up to Down
   quat[1] = -quat[1];   // Backward to North (Invert Roll)
   quat[3] = -quat[3];   // Up to Down (Invert Yaw)
}

void convert_gps_to_local_ned(float curr_lat, float curr_lon, float curr_alt, float* current_pos_ne) {
    // 1. If this is the very first GPS lock, lock this coordinate as (0,0,0) Home
    if (!is_home_set) {
        home_lat_rad = curr_lat * DEG_TO_RAD;
        home_lon_rad = curr_lon * DEG_TO_RAD;
        home_alt_m   = curr_alt;
        is_home_set  = true;
    }

    // 2. Convert current coordinates to radians
    float curr_lat_rad = curr_lat * DEG_TO_RAD;
    float curr_lon_rad = curr_lon * DEG_TO_RAD;

    // 3. Calculate coordinate deltas
    float d_lat = curr_lat_rad - home_lat_rad;
    float d_lon = curr_lon_rad - home_lon_rad;

    // 4. Flat-Earth Meridian/Prime-Vertical Radius Approximation
    // Computes meters per radian at your current latitude location
    float meters_per_rad_lat = EARTH_RADIUS; 
    float meters_per_rad_lon = EARTH_RADIUS * cosf(home_lat_rad);

    // 5. Output positions in linear meters
    current_pos_ne[0] = d_lat * meters_per_rad_lat; // Distance North (meters)
    current_pos_ne[1] = d_lon * meters_per_rad_lon; // Distance East (meters)
}

void quat_to_rpy(const float32_t* q, float32_t* rpy) {
    float32_t qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    // Roll (phi)
    float32_t sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float32_t cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    rpy[0] = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (theta)
    float32_t sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1)
        rpy[1] = copysignf(M_PI / 2.0f, sinp); // Use 90 degrees if out of range
    else
        rpy[1] = asinf(sinp);

    // Yaw (psi)
    float32_t siny_cosp = 2.0f * (qw * qz + qx * qy);
    float32_t cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    rpy[2] = atan2f(siny_cosp, cosy_cosp);
}

void ekf_get_pos(float32_t* pos_out) {
    pos_out[0] = x_pos_vel_biases[0]; // North
    pos_out[1] = x_pos_vel_biases[1]; // East
    pos_out[2] = x_pos_vel_biases[2]; // Down
}

void ekf_get_vel(float32_t* vel_out) {
    vel_out[0] = x_pos_vel_biases[3]; // North
    vel_out[1] = x_pos_vel_biases[4]; // East
    vel_out[2] = x_pos_vel_biases[5]; // Down
}

void ekf_get_quaternion(float32_t* quat_out) {
    quat_out[0] = x_q[0];
    quat_out[1] = x_q[1];
    quat_out[2] = x_q[2];
    quat_out[3] = x_q[3];
}

// ============================================================================
// INITIALIZATION
// ============================================================================
void glider_ekf_init(void) {
   // Link instances to memory arrays
   arm_mat_init_f32(&P_mat, STATE_DIM, STATE_DIM, P_data);
   arm_mat_init_f32(&Q_mat, STATE_DIM, STATE_DIM, Q_data);
   arm_mat_init_f32(&F_mat, STATE_DIM, STATE_DIM, F_data);
   
   arm_mat_init_f32(&F_T_mat,   STATE_DIM, STATE_DIM, F_T_data);
   arm_mat_init_f32(&FP_mat,    STATE_DIM, STATE_DIM, FP_data);
   arm_mat_init_f32(&FPF_T_mat, STATE_DIM, STATE_DIM, FPF_T_data);

   // Initialize P and Q as diagonal matrices with baseline uncertainties
   for(int i = 0; i < STATE_DIM * STATE_DIM; i++) {
       P_data[i] = 0.0f;
       Q_data[i] = 0.0f;
   }
   for(int i = 0; i < STATE_DIM; i++) {
       P_data[i * STATE_DIM + i] = 0.1f;  // Initial state uncertainty
       Q_data[i * STATE_DIM + i] = 0.01f; // Process noise
   }
}

// ============================================================================
// 1. MOTION MODEL UPDATE (High Rate: Call at IMU rate, e.g., 100Hz)
// ============================================================================
void glider_ekf_predict(float32_t* raw_accel, float32_t* raw_gyro, float32_t dt) {
    // Convert IMU readings to NED frame
    CPL_IMU_to_NED(raw_accel, raw_gyro);
   // Unbias IMU values
   float32_t accel[3] = { raw_accel[0] - x_pos_vel_biases[6], raw_accel[1] - x_pos_vel_biases[7], raw_accel[2] - x_pos_vel_biases[8] };
   float32_t gyro[3]  = { raw_gyro[0]  - x_pos_vel_biases[9], raw_gyro[1]  - x_pos_vel_biases[10], raw_gyro[2] - 0.0f }; // Assuming compass handles heading bias

   // 1. Propagate Position: p = p + v*dt
   x_pos_vel_biases[0] += x_pos_vel_biases[3] * dt;
   x_pos_vel_biases[1] += x_pos_vel_biases[4] * dt;
   x_pos_vel_biases[2] += x_pos_vel_biases[5] * dt;

   // 2. Propagate Velocity: v = v + (R(q)*accel + g)*dt
   float32_t R[9];
   quaternion_to_rotation_matrix(x_q, R);
   
   float32_t accel_ned[3];
   accel_ned[0] = R[0]*accel[0] + R[1]*accel[1] + R[2]*accel[2];
   accel_ned[1] = R[3]*accel[0] + R[4]*accel[1] + R[5]*accel[2];
   accel_ned[2] = R[6]*accel[0] + R[7]*accel[1] + R[8]*accel[2] + G_ACCEL;

   x_pos_vel_biases[3] += accel_ned[0] * dt;
   x_pos_vel_biases[4] += accel_ned[1] * dt;
   x_pos_vel_biases[5] += accel_ned[2] * dt;

   // 3. Propagate Attitude Quaternion
   float32_t qw = x_q[0], qx = x_q[1], qy = x_q[2], qz = x_q[3];
   x_q[0] += 0.5f * (-qx*gyro[0] - qy*gyro[1] - qz*gyro[2]) * dt;
   x_q[1] += 0.5f * ( qw*gyro[0] - qz*gyro[1] + qy*gyro[2]) * dt;
   x_q[2] += 0.5f * ( qz*gyro[0] + qw*gyro[1] - qx*gyro[2]) * dt;
   x_q[3] += 0.5f * (-qy*gyro[0] + qx*gyro[1] + qw*gyro[2]) * dt;
   
   // Fast Inverse Square Root for Quaternion Normalization
   float32_t sum = x_q[0]*x_q[0] + x_q[1]*x_q[1] + x_q[2]*x_q[2] + x_q[3]*x_q[3];
   float32_t norm;
   arm_sqrt_f32(sum, &norm);
   x_q[0] /= norm; x_q[1] /= norm; x_q[2] /= norm; x_q[3] /= norm;

   // 4. Construct Transition Jacobian (F Matrix)
   for(int i = 0; i < STATE_DIM * STATE_DIM; i++) F_data[i] = 0.0f;
   for(int i = 0; i < STATE_DIM; i++) F_data[i * STATE_DIM + i] = 1.0f; // Identity base

   F_data[0*STATE_DIM + 3] = dt; F_data[1*STATE_DIM + 4] = dt; F_data[2*STATE_DIM + 5] = dt; // dPos/dVel
   
   // dVel/dAttitude cross coupling (simplified skew symmetric * dt)
   F_data[3*STATE_DIM + 7] =  accel_ned[2]*dt; F_data[3*STATE_DIM + 8] = -accel_ned[1]*dt;
   F_data[4*STATE_DIM + 6] = -accel_ned[2]*dt; F_data[4*STATE_DIM + 8] =  accel_ned[0]*dt;
   F_data[5*STATE_DIM + 6] =  accel_ned[1]*dt; F_data[5*STATE_DIM + 7] = -accel_ned[0]*dt;

   // dVel/dAccelBias (-R * dt)
   for(int i=0; i<3; i++) {
       for(int j=0; j<3; j++) {
           F_data[(3+i)*STATE_DIM + (9+j)] = -R[i*3 + j] * dt;
       }
   }

   // 5. Covariance Propagation using CMSIS-DSP: P = F*P*F_T + Q
   arm_mat_trans_f32(&F_mat, &F_T_mat);
   arm_mat_mult_f32(&F_mat, &P_mat, &FP_mat);
   arm_mat_mult_f32(&FP_mat, &F_T_mat, &FPF_T_mat);
   arm_mat_add_f32(&FPF_T_mat, &Q_mat, &P_mat);
}

// ============================================================================
// SIMPLIFIED MOTION MODEL UPDATE (High Rate: Call at Accelerometer rate)
// ============================================================================
void glider_ekf_predict_bno_mode(float32_t* bno_linear_accel, float32_t dt) {
    
    // 1. Unbias the BNO085 linear acceleration values using EKF tracked biases
    float32_t accel[3] = { 
        bno_linear_accel[0] - x_pos_vel_biases[6], 
        bno_linear_accel[1] - x_pos_vel_biases[7], 
        bno_linear_accel[2] - x_pos_vel_biases[8] 
    };

    // 2. Rotate the acceleration from Body Frame to Local NED Navigation Frame
    float32_t R[9];
    quaternion_to_rotation_matrix(x_q, R);
    
    float32_t accel_ned[3];
    accel_ned[0] = R[0]*accel[0] + R[1]*accel[1] + R[2]*accel[2]; // True North Accel
    accel_ned[1] = R[3]*accel[0] + R[4]*accel[1] + R[5]*accel[2]; // True East Accel
    accel_ned[2] = R[6]*accel[0] + R[7]*accel[1] + R[8]*accel[2]; // True Down Accel

    // 3. INTEGRATION STEP: Semi-Implicit Euler (Velocity First)
    // First, calculate the brand-new velocities
    x_pos_vel_biases[3] += accel_ned[0] * dt; // Vel North
    x_pos_vel_biases[4] += accel_ned[1] * dt; // Vel East
    x_pos_vel_biases[5] += accel_ned[2] * dt; // Vel Down

    // Next, update positions using those newly updated velocities
    x_pos_vel_biases[0] += x_pos_vel_biases[3] * dt; // Pos North
    x_pos_vel_biases[1] += x_pos_vel_biases[4] * dt; // Pos East
    x_pos_vel_biases[2] += x_pos_vel_biases[5] * dt; // Pos Down

    // 4. CLEAN JACOBIAN F MATRIX (12x12)
    // Clear out any old cross-talk math completely
    for(int i = 0; i < STATE_DIM * STATE_DIM; i++) F_data[i] = 0.0f;
    for(int i = 0; i < STATE_DIM; i++) F_data[i * STATE_DIM + i] = 1.0f; // Identity Base

    // Map Kinematics: dPos / dVel = dt
    F_data[0*STATE_DIM + 3] = dt; // dPosN / dVelN
    F_data[1*STATE_DIM + 4] = dt; // dPosE / dVelE
    F_data[2*STATE_DIM + 5] = dt; // dPosD / dVelD
    
    // Map Kinematics: dVel / dAccelBias = -R * dt
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            F_data[(3 + i)*STATE_DIM + (9 + j)] = -R[i*3 + j] * dt;
        }
    }

    // 5. MATRIX MATHEMATICS EXECUTION (CMSIS-DSP)
    arm_mat_trans_f32(&F_mat, &F_T_mat);
    arm_mat_mult_f32(&F_mat, &P_mat, &FP_mat);
    arm_mat_mult_f32(&FP_mat, &F_T_mat, &FPF_T_mat);
    arm_mat_add_f32(&FPF_T_mat, &Q_mat, &P_mat);
}



// ============================================================================
// 2. SCALAR SENSOR UPDATE (High Efficiency: Matrix Inversion Free)
// ============================================================================
// Applies a singular scalar measurement update to the 12-state covariance
static void execute_scalar_update(uint8_t state_index, float32_t innovation, float32_t r_noise) {
   float32_t PH_T[STATE_DIM];
   
   // Extract column from P corresponding to the updated state element
   for(int i = 0; i < STATE_DIM; i++) {
       PH_T[i] = P_data[i * STATE_DIM + state_index];
   }

   // S = H*P*H^T + R -> Reduces down to index mapping
   float32_t S = P_data[state_index * STATE_DIM + state_index] + r_noise;
   
   if (S == 0.0f) return; // Prevent zero division safety check
   float32_t S_inv = 1.0f / S; 

   // Compute Kalman Gain K (12x1 vector) and update states concurrently
   float32_t K[STATE_DIM];
   float32_t dx[STATE_DIM];
   for(int i = 0; i < STATE_DIM; i++) {
       K[i] = PH_T[i] * S_inv;
       dx[i] = K[i] * innovation;
   }

   // Inject dx error-state directly back into global states
   x_pos_vel_biases[0] += dx[0]; x_pos_vel_biases[1] += dx[1]; x_pos_vel_biases[2] += dx[2]; // Pos (Indices 0, 1, 2)
   x_pos_vel_biases[3] += dx[3]; x_pos_vel_biases[4] += dx[4]; x_pos_vel_biases[5] += dx[5]; // Vel (Indices 3, 4, 5)
   
   // Correct attitude quaternion using small-angle error state approximation
   // Error states: dx[6] = roll error, dx[7] = pitch error, dx[8] = yaw error
   float32_t qw = x_q[0], qx = x_q[1], qy = x_q[2], qz = x_q[3];
   x_q[0] += 0.5f * (-qx*dx[6] - qy*dx[7] - qz*dx[8]);
   x_q[1] += 0.5f * ( qw*dx[6] - qz*dx[7] + qy*dx[8]);
   x_q[2] += 0.5f * ( qz*dx[6] + qw*dx[7] - qx*dx[8]);
   x_q[3] += 0.5f * (-qy*dx[6] + qx*dx[7] + qw*dx[8]);
   
   float32_t sum = x_q[0]*x_q[0] + x_q[1]*x_q[1] + x_q[2]*x_q[2] + x_q[3]*x_q[3];
   float32_t norm;
   arm_sqrt_f32(sum, &norm);
   x_q[0] /= norm; x_q[1] /= norm; x_q[2] /= norm; x_q[3] /= norm;

   // Correct Accelerometer Biases (Indices 9, 10, 11 of the 12-state dx vector)
   x_pos_vel_biases[6] += dx[9];  // Accel Bias X
   x_pos_vel_biases[7] += dx[10]; // Accel Bias Y
   x_pos_vel_biases[8] += dx[11]; // Accel Bias Z

   // --- Gyro Bias Update Deleted Here ---

   // Update Covariance Matrix: P = P - K*H*P
   for(int i = 0; i < STATE_DIM; i++) {
       for(int j = 0; j < STATE_DIM; j++) {
           P_data[i * STATE_DIM + j] -= K[i] * P_data[state_index * STATE_DIM + j];
       }
   }
}

// ============================================================================
// SENSOR UPDATE: BNO085 ORIENTATION
// ============================================================================
void glider_ekf_update_bno_quaternion(float32_t* bno_q, float32_t r_noise) {
    // bno_q vector format: [qw, qx, qy, qz]

    // Calculate quaternion innovation (difference between EKF quaternion and BNO quaternion)
    // Represented as small-angle errors across Roll (index 6), Pitch (index 7), and Yaw (index 8)
    float32_t qw_e = x_q[0], qx_e = x_q[1], qy_e = x_q[2], qz_e = x_q[3];
    float32_t qw_b = bno_q[0], qx_b = bno_q[1], qy_b = bno_q[2], qz_b = bno_q[3];

    // Compute error quaternion: q_error = q_ekf^-1 * q_bno
    //float32_t qe_w =  qw_e*qw_b + qx_e*qx_b + qy_e*qy_b + qz_e*qz_b;
    float32_t qe_x =  qw_e*qx_b - qx_e*qw_b - qy_e*qz_b + qz_e*qy_b;
    float32_t qe_y =  qw_e*qy_b + qx_e*qz_b - qy_e*qw_b - qz_e*qx_b;
    float32_t qe_z =  qw_e*qz_b - qx_e*qy_b + qy_e*qx_b - qz_e*qw_b;

    // Convert small-angle error quaternion component straight to localized error-states
    // q_error approx maps to [1, 0.5*dx, 0.5*dy, 0.5*dz]
    float32_t roll_innovation  = 2.0f * qe_x;
    float32_t pitch_innovation = 2.0f * qe_y;
    float32_t yaw_innovation   = 2.0f * qe_z;

    // Cleanly execute scalar updates across all three orientation axes
    execute_scalar_update(6, roll_innovation,  r_noise);
    execute_scalar_update(7, pitch_innovation, r_noise);
    execute_scalar_update(8, yaw_innovation,   r_noise);
}
// ============================================================================
// 4. SENSOR UPDATE: MAGNETIC COMPASS (Yaw / Heading)
// ============================================================================
// Call when the compass parses a new absolute heading reading (in radians)
void glider_ekf_update_compass(float32_t compass_yaw_rad, float32_t r_noise) {
   
   // 1. Calculate the EKF's currently predicted Yaw from its quaternion
   float32_t qw = x_q[0], qx = x_q[1], qy = x_q[2], qz = x_q[3];
   
   // Standard conversion from Quaternion to Euler Yaw (psi)
   float32_t siny_cosp = 2.0f * (qw * qz + qx * qy);
   float32_t cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
   
   // ATAN2 calculation using ARM CMSIS-DSP math extension (or standard atan2f)
   float32_t predicted_yaw = atan2f(siny_cosp, cosy_cosp);

   // 2. Compute Innovation (Measurement - Predicted)
   float32_t innovation = compass_yaw_rad - predicted_yaw;

   // 3. Handle Angular Wrap-Around (-pi to +pi)
   // This is critical! If the plane is at 179 deg and spins to -179 deg, 
   // the innovation shouldn't be -358 deg; it should be +2 deg.
   while (innovation >  PI) innovation -= 2.0f * PI;
   while (innovation < -PI) innovation += 2.0f * PI;

   // 4. Execute Scalar Update targeting State Index 8 (Attitude Error Z / Yaw)
   // State Index 6 = Roll Error, 7 = Pitch Error, 8 = Yaw Error
   execute_scalar_update(8, innovation, r_noise);
}

// ============================================================================
// ASYNCHRONOUS PUBLIC HARDWARE INTERFACES
// ============================================================================

// Call when Barometer converts altitude
void glider_ekf_update_baro(float32_t baro_alt, float32_t r_noise) {
   // In NED, Down position = -Altitude
   float32_t innovation = (-baro_alt) - x_pos_vel_biases[2];
   execute_scalar_update(2, innovation, r_noise); // State index 2 = Down Position
}

void ekf_gps_update(double lat, double lon, float alt, float sog_ms, float cog_true, float rms_range) {
   // Convert GPS lat/lon to local NED coordinates
   float32_t current_pos_ne[2];
   convert_gps_to_local_ned(lat, lon, alt, current_pos_ne);

   // GPS velocity is already in NED frame (assuming it was parsed that way)

   float32_t gps_vel_ned[2];
   gps_vel_ned[0] = sog_ms * cosf(cog_true * DEG_TO_RAD); // North Velocity
   gps_vel_ned[1] = sog_ms * sinf(cog_true * DEG_TO_RAD); // East Velocity



   // Call the EKF GPS update with appropriate noise parameters (these would be tuned based on GPS specs)
   glider_ekf_update_gps(current_pos_ne, gps_vel_ned, rms_range, rms_range * 2.0f); // Example noise values
}

// Call when GPS pulls a structural coordinate parse update (Sequential cascading execution)
void glider_ekf_update_gps(const float32_t* gps_pos_ne, const float32_t* gps_vel_ned, float32_t r_pos, float32_t r_vel) {
   execute_scalar_update(0, gps_pos_ne[0]  - x_pos_vel_biases[0], r_pos); // Update North Pos
   execute_scalar_update(1, gps_pos_ne[1]  - x_pos_vel_biases[1], r_pos); // Update East Pos
   execute_scalar_update(3, gps_vel_ned[0] - x_pos_vel_biases[3], r_vel); // Update North Vel
   execute_scalar_update(4, gps_vel_ned[1] - x_pos_vel_biases[4], r_vel); // Update East Vel
   //execute_scalar_update(5, gps_vel_ned[2] - x_pos_vel_biases[5], r_vel); // Update Down Vel
}
