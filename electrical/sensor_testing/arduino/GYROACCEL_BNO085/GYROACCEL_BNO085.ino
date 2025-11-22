// BNO085: GYRO (deg/s), ACCEL (deg/s^2), LIN_ACCEL (m/s^2)
// Developed with the help of AI by John Yfantis (discord: jeanvaljean5934)
// Refer to the Adafruit BNO08x Library examples: rotation_vector, quaternion_yaw_pitch_roll

#include <Adafruit_BNO08x.h>

// -------- CONFIGURATION -------
// Comment out to disable FILTERING 
#define FILTERING
#define BNO08X_RESET -1
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// previous angular velocity (deg/s) and timestamp (us)
float prev_omega_x = 0.0f, prev_omega_y = 0.0f, prev_omega_z = 0.0f;
unsigned long prev_gyro_time_us = 0;

#ifdef FILTERING
// EMA smoothing factor (0..1). Larger alpha = less smoothing
const float EMA_ALPHA = 0.2f;
float smoothed_omega_x = 0.0f, smoothed_omega_y = 0.0f, smoothed_omega_z = 0.0f;
#endif

// linear acceleration placeholders
float ax = 0.0f, ay = 0.0f, az = 0.0f;

// quaternion yaw/pitch/roll
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

// wrapped angles (-180, 180]
float roll_deg_wrapped = 0.0f;
float pitch_deg_wrapped = 0.0f;
float yaw_deg_wrapped = 0.0f;

// ----------------------------
// Convert quaternion to Euler angles
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw   = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll  = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw   *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll  *= RAD_TO_DEG;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("BNO085: GYRO, ACCEL, LIN_ACCEL, QUATERNION YAW");

  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) delay(10);
  }
  Serial.println("BNO08x Found!");

  const long interval_us = 4000; // ~250 Hz target
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, interval_us);
  bno08x.enableReport(SH2_ACCELEROMETER, interval_us);
  bno08x.enableReport(SH2_ARVR_STABILIZED_RV, interval_us);  // quaternion report

  prev_gyro_time_us = micros();
}

void loop() {
  if (!bno08x.getSensorEvent(&sensorValue)) return;
  unsigned long now_us = micros();

  // ------------------ GYROSCOPE ------------------
  if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
    // GYRO (rad/s -> deg/s)
    float gx_dps = sensorValue.un.gyroscope.x * RAD_TO_DEG;
    float gy_dps = sensorValue.un.gyroscope.y * RAD_TO_DEG;
    float gz_dps = sensorValue.un.gyroscope.z * RAD_TO_DEG;

#ifdef FILTERING
    // apply EMA filter to gyro before diff
    smoothed_omega_x = (1 - EMA_ALPHA) * smoothed_omega_x + EMA_ALPHA * gx_dps;
    smoothed_omega_y = (1 - EMA_ALPHA) * smoothed_omega_y + EMA_ALPHA * gy_dps;
    smoothed_omega_z = (1 - EMA_ALPHA) * smoothed_omega_z + EMA_ALPHA * gz_dps;
    gx_dps = smoothed_omega_x;
    gy_dps = smoothed_omega_y;
    gz_dps = smoothed_omega_z;
#endif

    // compute dt
    float dt = (now_us - prev_gyro_time_us) * 1e-6f;
    if (dt <= 0.0f) dt = 1e-6f;

    // backward difference for angular acceleration
    float alpha_x = (gx_dps - prev_omega_x) / dt;
    float alpha_y = (gy_dps - prev_omega_y) / dt;
    float alpha_z = (gz_dps - prev_omega_z) / dt;

    // update state
    prev_omega_x = gx_dps;
    prev_omega_y = gy_dps;
    prev_omega_z = gz_dps;
    prev_gyro_time_us = now_us;

    // ------------------ PRINT ON ONE LINE FOR SERIAL MONITOR ------------------
    Serial.print("GYRO [dps]: ");
    Serial.print(gx_dps, 2); Serial.print(" ");
    Serial.print(gy_dps, 2); Serial.print(" ");
    Serial.print(gz_dps, 2); Serial.print(" | ");

    Serial.print("ANG_ACC [dps^2]: ");
    Serial.print(alpha_x, 2); Serial.print(" ");
    Serial.print(alpha_y, 2); Serial.print(" ");
    Serial.print(alpha_z, 2); Serial.print(" | ");

    Serial.print("ACCEL [m/s^2]: ");
    Serial.print(ax, 2); Serial.print(" ");
    Serial.print(ay, 2); Serial.print(" ");
    Serial.print(az, 2); Serial.print(" | ");

    Serial.print("ROLL/PITCH/YAW [deg]: ");
    Serial.print(roll_deg_wrapped, 2); Serial.print(" ");
    Serial.print(pitch_deg_wrapped, 2); Serial.print(" ");
    Serial.println(yaw_deg_wrapped, 2);
    // ------------------ PRINT ON ONE LINE FOR SERIAL PLOTTER ------------------
    // Serial.print(gx_dps, 3); Serial.print(","); Serial.print(gy_dps, 3); Serial.print(","); Serial.print(gz_dps, 3); Serial.print(","); // GYRO
    // Serial.print(alpha_x, 3); Serial.print(","); Serial.print(alpha_y, 3); Serial.print(","); Serial.print(alpha_z, 3); Serial.print(","); // Angular ACCEL
    // Serial.print(ax, 3); Serial.print(","); Serial.print(ay, 3); Serial.print(","); Serial.println(az, 3); // Linear ACCEL

  }

  // ------------------ ACCELEROMETER ------------------
  if (sensorValue.sensorId == SH2_ACCELEROMETER) {
    ax = sensorValue.un.accelerometer.x;
    ay = sensorValue.un.accelerometer.y;
    az = sensorValue.un.accelerometer.z;
  }

  // ------------------ QUATERNION REPORT ------------------
  if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
    auto rv = sensorValue.un.arvrStabilizedRV;
    quaternionToEuler(rv.real, rv.i, rv.j, rv.k, &ypr, true);

    // wrap yaw to (-180, 180]
    roll_deg_wrapped = ypr.roll;
    pitch_deg_wrapped = ypr.pitch;
    yaw_deg_wrapped = ypr.yaw;

    if (roll_deg_wrapped > 180.0f) roll_deg_wrapped -= 360.0f;
    if (roll_deg_wrapped <= -180.0f) roll_deg_wrapped += 360.0f;

    if (pitch_deg_wrapped > 180.0f) pitch_deg_wrapped -= 360.0f;
    if (pitch_deg_wrapped <= -180.0f) pitch_deg_wrapped += 360.0f;

    if (yaw_deg_wrapped > 180.0f) yaw_deg_wrapped -= 360.0f;
    if (yaw_deg_wrapped <= -180.0f) yaw_deg_wrapped += 360.0f;
    
  }

  delay(10);
}
