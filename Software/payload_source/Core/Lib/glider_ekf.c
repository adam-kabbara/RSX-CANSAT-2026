#include "glider_ekf.h"

#define STATE_DIM 12  // Error-state size (Pos, Vel, Att_err, Accel_bias)
#define QUAT_DIM  4   // Attitude state tracking size

// ============================================================================
// GLOBAL STATE MEMORY
// ============================================================================
static float32_t x_pos_vel_biases[9]; // Pos(3), Vel(3), Accel_Bias(3)
static float32_t x_q[QUAT_DIM] = {1.0f, 0.0f, 0.0f, 0.0f}; // Identity Quaternion

// Covariance Matrices
static float32_t P_data[STATE_DIM * STATE_DIM];
static float32_t Q_data[STATE_DIM * STATE_DIM];
static float32_t F_data[STATE_DIM * STATE_DIM];

// CMSIS-DSP Matrix Instances
static arm_matrix_instance_f32 P_mat;
static arm_matrix_instance_f32 Q_mat;
static arm_matrix_instance_f32 F_mat;

// Temporary Buffers for Prediction step
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

const float EARTH_RADIUS = 6378137.0f; 
const float DEG_TO_RAD   = M_PI / 180.0f;

// Saved course-over-ground vector from the latest GPS string parse
static float32_t last_gps_course_rad = 0.0f;
static bool last_gps_course_valid = false;

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
   accel[0] = -accel[0]; 
   accel[2] = -accel[2]; 
   quat[1] = -quat[1];   
   quat[3] = -quat[3];   
}

void quat_to_rpy(const float32_t* q, float32_t* rpy) {
    float32_t qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    // Roll (phi)
    float32_t sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float32_t cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    rpy[0] = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (theta)
    float32_t sinp = 2.0f * (qw * qy - qz * qx);

    if (fabsf(sinp) >= 1) {
        rpy[1] = copysignf(M_PI / 2.0f, sinp); // Use 90 degrees if out of range
    } else {
        rpy[1] = asinf(sinp);
    }

    // Yaw (psi)
    float32_t siny_cosp = 2.0f * (qw * qz + qx * qy);
    float32_t cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    rpy[2] = atan2f(siny_cosp, cosy_cosp);
}

void zero_ekf_pos(float curr_lat, float curr_lon, float curr_alt) {
    home_lat_rad = curr_lat * DEG_TO_RAD;
    home_lon_rad = curr_lon * DEG_TO_RAD;
    home_alt_m   = curr_alt;
    is_home_set  = true;
}

void convert_gps_to_local_ned(float curr_lat, float curr_lon, float curr_alt, float* current_pos_ne) {
    float curr_lat_rad = curr_lat * DEG_TO_RAD;
    float curr_lon_rad = curr_lon * DEG_TO_RAD;

    float d_lat = curr_lat_rad - home_lat_rad;
    float d_lon = curr_lon_rad - home_lon_rad;

    float meters_per_rad_lat = EARTH_RADIUS; 
    float meters_per_rad_lon = EARTH_RADIUS * cosf(home_lat_rad);

    current_pos_ne[0] = d_lat * meters_per_rad_lat; 
    current_pos_ne[1] = d_lon * meters_per_rad_lon; 
}

void ekf_get_pos(float32_t* pos_out) {
    pos_out[0] = x_pos_vel_biases[0]; 
    pos_out[1] = x_pos_vel_biases[1]; 
    pos_out[2] = x_pos_vel_biases[2]; 
}

void ekf_get_vel(float32_t* vel_out) {
    vel_out[0] = x_pos_vel_biases[3]; 
    vel_out[1] = x_pos_vel_biases[4]; 
    vel_out[2] = x_pos_vel_biases[5]; 
}

void ekf_get_quaternion(float32_t* quat_out) {
    memcpy(quat_out, x_q, sizeof(x_q));
}

void glider_ekf_init(void) {
   arm_mat_init_f32(&P_mat, STATE_DIM, STATE_DIM, P_data);
   arm_mat_init_f32(&Q_mat, STATE_DIM, STATE_DIM, Q_data);
   arm_mat_init_f32(&F_mat, STATE_DIM, STATE_DIM, F_data);
   
   arm_mat_init_f32(&F_T_mat,   STATE_DIM, STATE_DIM, F_T_data);
   arm_mat_init_f32(&FP_mat,    STATE_DIM, STATE_DIM, FP_data);
   arm_mat_init_f32(&FPF_T_mat, STATE_DIM, STATE_DIM, FPF_T_data);

   memset(P_data, 0, sizeof(P_data));
   memset(Q_data, 0, sizeof(Q_data));
   
   for(int i = 0; i < STATE_DIM; i++) {
       P_data[i * STATE_DIM + i] = 0.05f;  
       Q_data[i * STATE_DIM + i] = 0.002f; // Low tracking noise since kinematics are driven by velocity
   }
}

// ============================================================================
// 1. VELOCITY-DRIVEN PREDICTION STEP (Replaces direct raw Accel dead-reckoning)
// ============================================================================
void glider_ekf_predict(float32_t dt) {
    if (dt <= 0.0f) return;

    // Kinematics: Propagate position coordinates using the latest filter velocity states
    x_pos_vel_biases[0] += x_pos_vel_biases[3] * dt; // Pos North
    x_pos_vel_biases[1] += x_pos_vel_biases[4] * dt; // Pos East
    x_pos_vel_biases[2] += x_pos_vel_biases[5] * dt; // Pos Down

    // Build the 12x12 Kinematic State Transition Jacobian (F Matrix)
    memset(F_data, 0, sizeof(F_data));
    for(int i = 0; i < STATE_DIM; i++) F_data[i * STATE_DIM + i] = 1.0f; // Diagonal base

    // Position updates map directly to velocity states: dPos / dVel = dt
    F_data[0 * STATE_DIM + 3] = dt; 
    F_data[1 * STATE_DIM + 4] = dt; 
    F_data[2 * STATE_DIM + 5] = dt; 

    // Execute CMSIS-DSP Matrix Calculations: P = F * P * F^T + Q
    arm_mat_trans_f32(&F_mat, &F_T_mat);
    arm_mat_mult_f32(&F_mat, &P_mat, &FP_mat);
    arm_mat_mult_f32(&FP_mat, &F_T_mat, &FPF_T_mat);
    arm_mat_add_f32(&FPF_T_mat, &Q_mat, &P_mat);
}

// ============================================================================
// 2. SCALAR SENSOR CORRECTION EXTENSION
// ============================================================================
static void execute_scalar_update(uint8_t state_index, float32_t innovation, float32_t r_noise) {
   float32_t PH_T[STATE_DIM];
   
   for(int i = 0; i < STATE_DIM; i++) {
       PH_T[i] = P_data[i * STATE_DIM + state_index];
   }

   float32_t S = P_data[state_index * STATE_DIM + state_index] + r_noise;
   if (S <= 0.0f) return; 
   float32_t S_inv = 1.0f / S; 

   float32_t K[STATE_DIM];
   float32_t dx[STATE_DIM];
   for(int i = 0; i < STATE_DIM; i++) {
       K[i] = PH_T[i] * S_inv;
       dx[i] = K[i] * innovation;
   }

   // Apply calculated error corrections directly to the velocity states
   x_pos_vel_biases[0] += dx[0]; x_pos_vel_biases[1] += dx[1]; x_pos_vel_biases[2] += dx[2]; 
   x_pos_vel_biases[3] += dx[3]; x_pos_vel_biases[4] += dx[4]; x_pos_vel_biases[5] += dx[5]; 
   
   // Apply small-angle orientation modifications to the active quaternion
   float32_t qw = x_q[0], qx = x_q[1], qy = x_q[2], qz = x_q[3];
   x_q[0] += 0.5f * (-qx*dx[6] - qy*dx[7] - qz*dx[8]);
   x_q[1] += 0.5f * ( qw*dx[6] - qz*dx[7] + qy*dx[8]);
   x_q[2] += 0.5f * ( qz*dx[6] + qw*dx[7] - qx*dx[8]);
   x_q[3] += 0.5f * (-qy*dx[6] + qx*dx[7] + qw*dx[8]);
   
   float32_t sum = x_q[0]*x_q[0] + x_q[1]*x_q[1] + x_q[2]*x_q[2] + x_q[3]*x_q[3];
   float32_t norm;
   arm_sqrt_f32(sum, &norm);
   if(norm > 0.0f) {
       x_q[0] /= norm; x_q[1] /= norm; x_q[2] /= norm; x_q[3] /= norm;
   }

   x_pos_vel_biases[6] += dx[9];  
   x_pos_vel_biases[7] += dx[10]; 
   x_pos_vel_biases[8] += dx[11]; 

   // Update Covariance Matrix: P = P - K * H * P
   for(int i = 0; i < STATE_DIM; i++) {
       for(int j = 0; j < STATE_DIM; j++) {
           P_data[i * STATE_DIM + j] -= K[i] * P_data[state_index * STATE_DIM + j];
       }
   }
}

// ============================================================================
// 3. FUSED IMU + GPS ORIENTATION CRADLE
// ============================================================================
void glider_ekf_update_bno_quaternion(float32_t* bno_q, float32_t r_noise) {
    float32_t qw_e = x_q[0], qx_e = x_q[1], qy_e = x_q[2], qz_e = x_q[3];
    float32_t qw_b = bno_q[0], qx_b = bno_q[1], qy_b = bno_q[2], qz_b = bno_q[3];

    // Extract the error quaternion vector components
    float32_t qe_x =  qw_e*qx_b - qx_e*qw_b - qy_e*qz_b + qz_e*qy_b;
    float32_t qe_y =  qw_e*qy_b + qx_e*qz_b - qy_e*qw_b - qz_e*qx_b;
    float32_t qe_z =  qw_e*qz_b - qx_e*qy_b + qy_e*qx_b - qz_e*qw_b;

    float32_t roll_innovation  = 2.0f * qe_x;
    float32_t pitch_innovation = 2.0f * qe_y;
    float32_t yaw_innovation   = 2.0f * qe_z;

    // Correct Roll and Pitch axes using the high-rate IMU vector fields
    execute_scalar_update(6, roll_innovation,  r_noise);
    execute_scalar_update(7, pitch_innovation, r_noise);

    // --- FUSED HEADING YAW SELECTION ---
    // If the glider has forward velocity, overwrite the IMU's raw yaw drift with the GPS Course Over Ground.
    // If stationary or flying too slowly, fall back safely to the IMU orientation.
    float32_t ground_speed_sq = (x_pos_vel_biases[3] * x_pos_vel_biases[3]) + (x_pos_vel_biases[4] * x_pos_vel_biases[4]);
    
    if (last_gps_course_valid && (ground_speed_sq > 4.0f)) { // 4.0f maps to > 2.0 m/s threshold
        float32_t siny_cosp = 2.0f * (x_q[0] * x_q[3] + x_q[1] * x_q[2]);
        float32_t cosy_cosp = 1.0f - 2.0f * (x_q[2] * x_q[2] + x_q[3] * x_q[3]);
        float32_t predicted_yaw = atan2f(siny_cosp, cosy_cosp);

        float32_t heading_innovation = last_gps_course_rad - predicted_yaw;
        while (heading_innovation >  M_PI) heading_innovation -= 2.0f * M_PI;
        while (heading_innovation < -M_PI) heading_innovation += 2.0f * M_PI;

        // Apply heading update with a mixed noise weighting to fuse both inputs cleanly
        execute_scalar_update(8, heading_innovation, 0.5f);
    } else {
        // Fallback to internal IMU tracking if GPS velocity drops below threshold
        execute_scalar_update(8, yaw_innovation, r_noise);
    }
}

void glider_ekf_update_baro(float32_t baro_alt, float32_t r_noise) {
   // In NED, Down position = -Altitude
   float32_t innovation = (-baro_alt) - x_pos_vel_biases[2];
   
   // FORCE HIGH TRUST: Override incoming r_noise to prioritize the barometer
   float32_t trusted_baro_noise = 0.15f; 
   execute_scalar_update(2, innovation, trusted_baro_noise); // State index 2 = Down Position
}

// ============================================================================
// 4. GPS PARSING DATA STRATIFICATION
// ============================================================================
void ekf_gps_update(double lat, double lon, float alt, float sog_ms, float cog_true, float rms_range) {
   float32_t current_pos_ne[2];
   convert_gps_to_local_ned(lat, lon, alt, current_pos_ne);

   // Construct GPS Down Position from Altitude relative to launch-pad home
   // Remember: In NED, Down = -Altitude
   float32_t gps_pos_down = -(alt - home_alt_m);

   // Extract horizontal velocities
   float32_t gps_vel_ned[3];
   last_gps_course_rad = cog_true * DEG_TO_RAD;
   last_gps_course_valid = (last_gps_course_rad != 0.0f);

   gps_vel_ned[0] = sog_ms * cosf(last_gps_course_rad); // North Velocity
   gps_vel_ned[1] = sog_ms * sinf(last_gps_course_rad); // East Velocity
   gps_vel_ned[2] = 0.0f;                               // Default Down Velocity if not provided by GPS

   // Pass values to the filter update step
   // We pass gps_pos_down directly into our explicit position array update below
   glider_ekf_update_gps_3d(current_pos_ne, gps_pos_down, gps_vel_ned, rms_range); 
}

void glider_ekf_update_gps_3d(const float32_t* gps_pos_ne, float32_t gps_pos_down, const float32_t* gps_vel_ned, float32_t r_pos) {
   // 1. Horizontal Updates (Standard trust based on GPS signal quality metrics)
   execute_scalar_update(0, gps_pos_ne[0]  - x_pos_vel_biases[0], r_pos); // North Pos
   execute_scalar_update(1, gps_pos_ne[1]  - x_pos_vel_biases[1], r_pos); // East Pos
   execute_scalar_update(3, gps_vel_ned[0] - x_pos_vel_biases[3], 0.1f);  // North Vel
   execute_scalar_update(4, gps_vel_ned[1] - x_pos_vel_biases[4], 0.1f);  // East Vel

   // 2. Vertical Update (LOW TRUST / HEAVILY PENALIZED)
   // We artificially inflate the GPS vertical noise parameter to 15.0 meters.
   // This allows the barometer to dominate the state while GPS handles long-term drift.
   float32_t high_gps_alt_noise = 15.0f; 
   execute_scalar_update(2, gps_pos_down - x_pos_vel_biases[2], high_gps_alt_noise); // Down Pos
}