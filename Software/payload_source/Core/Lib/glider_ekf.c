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

// Last GPS speed-over-ground — used to clamp horizontal velocity magnitude
static float32_t last_sog_ms = 0.0f;

// Baro velocity estimation — least-squares fit over a ring buffer of readings
#define BARO_VEL_WINDOW 5
static float32_t baro_alt_history[BARO_VEL_WINDOW];
static float32_t baro_time_history[BARO_VEL_WINDOW];
static uint8_t   baro_history_idx   = 0;
static uint8_t   baro_history_count = 0;
static float32_t baro_time_accum    = 0.0f; // incremented each predict tick

// ============================================================================
// UPDATE MASK — controls which states are corrected by a given measurement
// ============================================================================
typedef enum {
    UPDATE_MASK_ALL      = 0xFF, // GPS: correct position, velocity, attitude, bias
    UPDATE_MASK_ATT_ONLY = 0x02  // IMU: correct attitude and bias only, never position/velocity
} UpdateMask;

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
    quat[1]  = -quat[1];
    quat[3]  = -quat[3];
}

void quat_to_rpy(const float32_t* q, float32_t* rpy) {
    float32_t qw = q[0], qx = q[1], qy = q[2], qz = q[3];

    // Roll (phi)
    float32_t sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float32_t cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    rpy[0] = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (theta)
    float32_t sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f) {
        rpy[1] = copysignf(M_PI / 2.0f, sinp);
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

    current_pos_ne[0] = d_lat * meters_per_rad_lat; // North
    current_pos_ne[1] = d_lon * meters_per_rad_lon; // East
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

// ============================================================================
// INIT
// ============================================================================
void glider_ekf_init(void) {
    arm_mat_init_f32(&P_mat, STATE_DIM, STATE_DIM, P_data);
    arm_mat_init_f32(&Q_mat, STATE_DIM, STATE_DIM, Q_data);
    arm_mat_init_f32(&F_mat, STATE_DIM, STATE_DIM, F_data);

    arm_mat_init_f32(&F_T_mat,   STATE_DIM, STATE_DIM, F_T_data);
    arm_mat_init_f32(&FP_mat,    STATE_DIM, STATE_DIM, FP_data);
    arm_mat_init_f32(&FPF_T_mat, STATE_DIM, STATE_DIM, FPF_T_data);

    memset(P_data, 0, sizeof(P_data));
    memset(Q_data, 0, sizeof(Q_data));

    // Initial covariance — modest uncertainty across all states
    for (int i = 0; i < STATE_DIM; i++) {
        P_data[i * STATE_DIM + i] = 0.05f;
    }

    // Process noise — tuned per state type:
    // Position: low, kinematics are well-defined
    Q_data[0 * STATE_DIM + 0] = 0.01f;
    Q_data[1 * STATE_DIM + 1] = 0.01f;
    Q_data[2 * STATE_DIM + 2] = 0.01f;
    // Velocity: small — IMU integration provides real velocity tracking at 100Hz.
    // This mainly covers attitude-error propagation and vibration.
    Q_data[3 * STATE_DIM + 3] = 0.01f;
    Q_data[4 * STATE_DIM + 4] = 0.01f;
    Q_data[5 * STATE_DIM + 5] = 0.02f;
    // Attitude error: tight, BNO constrains these at high rate
    Q_data[6 * STATE_DIM + 6] = 0.001f;
    Q_data[7 * STATE_DIM + 7] = 0.001f;
    Q_data[8 * STATE_DIM + 8] = 0.005f; // Yaw noisier than roll/pitch
    // Accel bias: very slow-moving
    Q_data[9  * STATE_DIM + 9]  = 0.0001f;
    Q_data[10 * STATE_DIM + 10] = 0.0001f;
    Q_data[11 * STATE_DIM + 11] = 0.0001f;
}

// ============================================================================
// 1. IMU-DRIVEN PREDICTION STEP
// ============================================================================
void glider_ekf_predict(const float32_t* linear_accel_body, float32_t dt) {
    if (dt <= 0.0f) return;

    // Rotation matrix: body frame → NED, derived from current attitude estimate.
    // linear_accel_body is already gravity-compensated by BNO085 onboard fusion,
    // so we only need to rotate it into NED and subtract the estimated bias.
    float32_t R[9];
    quaternion_to_rotation_matrix(x_q, R);

    float32_t accel_ned[3];
    accel_ned[0] = R[0]*linear_accel_body[0] + R[1]*linear_accel_body[1] + R[2]*linear_accel_body[2] - x_pos_vel_biases[6];
    accel_ned[1] = R[3]*linear_accel_body[0] + R[4]*linear_accel_body[1] + R[5]*linear_accel_body[2] - x_pos_vel_biases[7];
    accel_ned[2] = R[6]*linear_accel_body[0] + R[7]*linear_accel_body[1] + R[8]*linear_accel_body[2] - x_pos_vel_biases[8];

    // Integrate: velocity from acceleration, position from velocity.
    x_pos_vel_biases[3] += accel_ned[0] * dt;
    x_pos_vel_biases[4] += accel_ned[1] * dt;
    x_pos_vel_biases[5] += accel_ned[2] * dt;
    x_pos_vel_biases[0] += x_pos_vel_biases[3] * dt;
    x_pos_vel_biases[1] += x_pos_vel_biases[4] * dt;
    x_pos_vel_biases[2] += x_pos_vel_biases[5] * dt;

    // Build Jacobian F: identity + kinematic coupling + velocity/bias coupling.
    memset(F_data, 0, sizeof(F_data));
    for (int i = 0; i < STATE_DIM; i++) F_data[i * STATE_DIM + i] = 1.0f;

    // dPos/dVel = dt
    F_data[0*STATE_DIM + 3] = dt;
    F_data[1*STATE_DIM + 4] = dt;
    F_data[2*STATE_DIM + 5] = dt;

    // dVel/dBias = -R*dt (a bias error of db causes a velocity error of -R*db*dt)
    F_data[3*STATE_DIM +  9] = -R[0]*dt; F_data[3*STATE_DIM + 10] = -R[1]*dt; F_data[3*STATE_DIM + 11] = -R[2]*dt;
    F_data[4*STATE_DIM +  9] = -R[3]*dt; F_data[4*STATE_DIM + 10] = -R[4]*dt; F_data[4*STATE_DIM + 11] = -R[5]*dt;
    F_data[5*STATE_DIM +  9] = -R[6]*dt; F_data[5*STATE_DIM + 10] = -R[7]*dt; F_data[5*STATE_DIM + 11] = -R[8]*dt;

    // P = F * P * F^T + Q
    arm_mat_trans_f32(&F_mat, &F_T_mat);
    arm_mat_mult_f32(&F_mat, &P_mat, &FP_mat);
    arm_mat_mult_f32(&FP_mat, &F_T_mat, &FPF_T_mat);
    arm_mat_add_f32(&FPF_T_mat, &Q_mat, &P_mat);

    baro_time_accum += dt;
}

// ============================================================================
// 2. SCALAR SENSOR CORRECTION
// ============================================================================
static void execute_scalar_update(uint8_t state_index, float32_t innovation,
                                  float32_t r_noise, UpdateMask mask) {
    float32_t PH_T[STATE_DIM];
    for (int i = 0; i < STATE_DIM; i++) {
        PH_T[i] = P_data[i * STATE_DIM + state_index];
    }

    float32_t S = P_data[state_index * STATE_DIM + state_index] + r_noise;
    if (S <= 0.0f) return;
    float32_t S_inv = 1.0f / S;

    float32_t K[STATE_DIM];
    float32_t dx[STATE_DIM];
    for (int i = 0; i < STATE_DIM; i++) {
        K[i]  = PH_T[i] * S_inv;
        dx[i] = K[i] * innovation;
    }

    // Position + Velocity corrections — skipped for attitude-only updates.
    // Without this gate, rotating the board fires attitude corrections that
    // bleed into position/velocity through dense off-diagonal P terms.
    if (mask != UPDATE_MASK_ATT_ONLY) {
        x_pos_vel_biases[0] += dx[0];
        x_pos_vel_biases[1] += dx[1];
        x_pos_vel_biases[2] += dx[2];
        x_pos_vel_biases[3] += dx[3];
        x_pos_vel_biases[4] += dx[4];
        x_pos_vel_biases[5] += dx[5];
    }

    // Attitude correction — always applied
    float32_t qw = x_q[0], qx = x_q[1], qy = x_q[2], qz = x_q[3];
    x_q[0] += 0.5f * (-qx*dx[6] - qy*dx[7] - qz*dx[8]);
    x_q[1] += 0.5f * ( qw*dx[6] - qz*dx[7] + qy*dx[8]);
    x_q[2] += 0.5f * ( qz*dx[6] + qw*dx[7] - qx*dx[8]);
    x_q[3] += 0.5f * (-qy*dx[6] + qx*dx[7] + qw*dx[8]);

    float32_t sum = x_q[0]*x_q[0] + x_q[1]*x_q[1] + x_q[2]*x_q[2] + x_q[3]*x_q[3];
    float32_t norm;
    arm_sqrt_f32(sum, &norm);
    if (norm > 0.0f) {
        x_q[0] /= norm; x_q[1] /= norm; x_q[2] /= norm; x_q[3] /= norm;
    }

    // Accel bias correction — always applied
    x_pos_vel_biases[6] += dx[9];
    x_pos_vel_biases[7] += dx[10];
    x_pos_vel_biases[8] += dx[11];

    // Covariance update: P = P - K * H * P
    // Runs unconditionally — attitude measurements still reduce uncertainty
    // in position/velocity through the covariance cross-terms, which is correct.
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            P_data[i * STATE_DIM + j] -= K[i] * P_data[state_index * STATE_DIM + j];
        }
    }
}

// ============================================================================
// 3. FUSED IMU + GPS ORIENTATION UPDATE
// ============================================================================
void glider_ekf_update_bno_quaternion(float32_t* bno_q, float32_t r_noise) {
    float32_t qw_e = x_q[0], qx_e = x_q[1], qy_e = x_q[2], qz_e = x_q[3];
    float32_t qw_b = bno_q[0], qx_b = bno_q[1], qy_b = bno_q[2], qz_b = bno_q[3];

    // Error quaternion vector components
    float32_t qe_x =  qw_e*qx_b - qx_e*qw_b - qy_e*qz_b + qz_e*qy_b;
    float32_t qe_y =  qw_e*qy_b + qx_e*qz_b - qy_e*qw_b - qz_e*qx_b;
    float32_t qe_z =  qw_e*qz_b - qx_e*qy_b + qy_e*qx_b - qz_e*qw_b;

    float32_t roll_innovation  = 2.0f * qe_x;
    float32_t pitch_innovation = 2.0f * qe_y;
    float32_t yaw_innovation   = 2.0f * qe_z;

    // Roll and pitch: high-rate IMU, attitude-only mask
    execute_scalar_update(6, roll_innovation,  r_noise, UPDATE_MASK_ATT_ONLY);
    execute_scalar_update(7, pitch_innovation, r_noise, UPDATE_MASK_ATT_ONLY);

    // Yaw: fuse GPS course-over-ground when moving, fall back to IMU otherwise
    float32_t ground_speed_sq = (x_pos_vel_biases[3] * x_pos_vel_biases[3])
                              + (x_pos_vel_biases[4] * x_pos_vel_biases[4]);

    if (last_gps_course_valid && (ground_speed_sq > 4.0f)) { // > ~2 m/s
        float32_t siny_cosp = 2.0f * (x_q[0] * x_q[3] + x_q[1] * x_q[2]);
        float32_t cosy_cosp = 1.0f - 2.0f * (x_q[2] * x_q[2] + x_q[3] * x_q[3]);
        float32_t predicted_yaw = atan2f(siny_cosp, cosy_cosp);

        float32_t heading_innovation = last_gps_course_rad - predicted_yaw;
        while (heading_innovation >  M_PI) heading_innovation -= 2.0f * M_PI;
        while (heading_innovation < -M_PI) heading_innovation += 2.0f * M_PI;

        execute_scalar_update(8, heading_innovation, 0.5f, UPDATE_MASK_ATT_ONLY);
    } else {
        execute_scalar_update(8, yaw_innovation, r_noise, UPDATE_MASK_ATT_ONLY);
    }
}

void glider_ekf_update_baro(float32_t baro_alt, float32_t r_noise) {
    // Don't process baro until home altitude is established — otherwise
    // Down position is computed relative to 0.0 AGL which is wrong by ~442m
    if (!is_home_set) return;

    // Convert absolute baro altitude to AGL relative to home
    baro_alt = baro_alt - home_alt_m;

    // Position update — NED: Down = -Altitude
    float32_t pos_innovation = (-baro_alt) - x_pos_vel_biases[2];
    execute_scalar_update(2, pos_innovation, 0.15f, UPDATE_MASK_ALL);

    // Store reading in ring buffer for least-squares velocity estimation
    baro_alt_history[baro_history_idx]  = baro_alt;
    baro_time_history[baro_history_idx] = baro_time_accum;
    baro_history_idx = (baro_history_idx + 1) % BARO_VEL_WINDOW;
    if (baro_history_count < BARO_VEL_WINDOW) baro_history_count++;

    if (baro_history_count < 2) {
        // Not enough history yet — soft zero anchor to prevent free drift
        execute_scalar_update(5, 0.0f - x_pos_vel_biases[5], 2.0f, UPDATE_MASK_ALL);
        return;
    }

    // Least-squares linear fit: alt = a*t + b, slope = d(alt)/dt
    // vel_down (NED) = -slope. Fitting over multiple samples makes this
    // robust to individual noisy readings — one bad sample shifts the slope
    // by only 1/n rather than dominating the estimate.
    float32_t sum_t  = 0.0f, sum_a  = 0.0f;
    float32_t sum_tt = 0.0f, sum_ta = 0.0f;
    float32_t n = (float32_t)baro_history_count;

    for (uint8_t i = 0; i < baro_history_count; i++) {
        float32_t t = baro_time_history[i];
        float32_t a = baro_alt_history[i];
        sum_t  += t;
        sum_a  += a;
        sum_tt += t * t;
        sum_ta += t * a;
    }

    float32_t denom = (n * sum_tt - sum_t * sum_t);
    if (fabsf(denom) < 1e-6f) {
        execute_scalar_update(5, 0.0f - x_pos_vel_biases[5], 2.0f, UPDATE_MASK_ALL);
        return;
    }

    float32_t slope    = (n * sum_ta - sum_t * sum_a) / denom; // d(alt)/dt
    float32_t vel_down = -slope; // NED: Down = -altitude rate

    // Noise inversely scales with window fill — more samples = more confidence.
    // Floor at 0.1 to avoid overconfidence even with a full window.
    float32_t vel_noise = fmaxf(1.0f / n, 0.1f);
    execute_scalar_update(5, vel_down - x_pos_vel_biases[5], vel_noise, UPDATE_MASK_ALL);
}

// ============================================================================
// 4. GPS UPDATE
// ============================================================================
void ekf_gps_update(double lat, double lon, float alt, float sog_ms, float cog_true, float rms_range) {
    // Reject fixes with no altitude — GPS hasn't achieved a 3D lock yet.
    // This prevents home_alt_m being set to 0.0, which would cause a permanent
    // ~442m offset in Down position for the rest of the flight.
    if (alt == 0.0f) return;

    // Auto-set home position on first valid 3D fix.
    // Keeps zero_ekf_pos() as an optional manual override but ensures
    // the filter always has a valid baseline before processing measurements.
    if (!is_home_set) {
        zero_ekf_pos((float)lat, (float)lon, alt);
        return; // don't use this fix as a measurement, just establish home
    }

    last_gps_course_rad = cog_true * DEG_TO_RAD;
    last_gps_course_valid = (sog_ms > 1.0f); // raised from 0.5 — COG is unreliable below 1 m/s
    last_sog_ms = sog_ms;

    float32_t current_pos_ne[2];
    convert_gps_to_local_ned((float)lat, (float)lon, alt, current_pos_ne);
    float32_t gps_pos_down = -(alt - home_alt_m);

    float32_t gps_vel_ned[3];
    gps_vel_ned[0] = sog_ms * cosf(last_gps_course_rad); // North
    gps_vel_ned[1] = sog_ms * sinf(last_gps_course_rad); // East
    gps_vel_ned[2] = 0.0f;

    // PDOP is not position error in metres — scale it up and enforce a 5m floor.
    // Your GPS is showing ±10m position noise while stationary; trusting raw PDOP
    // causes the filter to jump to every noisy fix rather than smoothing over them.
    float32_t effective_r = fmaxf(rms_range * 3.0f, 5.0f);

    glider_ekf_update_gps_3d(current_pos_ne, gps_pos_down, gps_vel_ned, effective_r);
}

void glider_ekf_update_gps_3d(const float32_t* gps_pos_ne, float32_t gps_pos_down,
                               const float32_t* gps_vel_ned, float32_t r_pos) {
    // Position updates always — even noisy GPS position is a useful long-term anchor
    execute_scalar_update(0, gps_pos_ne[0] - x_pos_vel_biases[0], r_pos, UPDATE_MASK_ALL);
    execute_scalar_update(1, gps_pos_ne[1] - x_pos_vel_biases[1], r_pos, UPDATE_MASK_ALL);

    // Velocity updates only above 1 m/s SOG. Below that, GPS COG is dominated
    // by receiver noise and derived velocity is meaningless — feeding it in
    // drives position away from the fix rather than toward it.
    // When slow/stationary, apply a soft zero-velocity anchor instead.
    if (last_sog_ms > 1.0f) {
        // NMEA SOG/COG-derived velocity has ~2 m/s noise at low speeds — be conservative.
        execute_scalar_update(3, gps_vel_ned[0] - x_pos_vel_biases[3], 2.0f, UPDATE_MASK_ALL);
        execute_scalar_update(4, gps_vel_ned[1] - x_pos_vel_biases[4], 2.0f, UPDATE_MASK_ALL);
    } else {
        // Below 1 m/s COG is unreliable; apply a soft zero-velocity anchor.
        execute_scalar_update(3, 0.0f - x_pos_vel_biases[3], 0.25f, UPDATE_MASK_ALL);
        execute_scalar_update(4, 0.0f - x_pos_vel_biases[4], 0.25f, UPDATE_MASK_ALL);
    }

    // Vertical: barometer dominates altitude; GPS handles long-term drift only
    // float32_t high_gps_alt_noise = 15.0f;
    // execute_scalar_update(2, gps_pos_down - x_pos_vel_biases[2], high_gps_alt_noise, UPDATE_MASK_ALL);
}