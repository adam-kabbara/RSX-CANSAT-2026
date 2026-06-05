#include "controller.hpp"

FlightControllers::FlightControllers(const RollControllerConfig& roll_cfg,
                                     const PitchControllerConfig& pitch_cfg,
                                     const ServoHardwareConfig& servo_cfg)
    : rc(roll_cfg), pc(pitch_cfg), sc(servo_cfg),
      roll_integral(0.0f), last_roll_error(0.0f), last_pitch_error(0.0f) {}

void FlightControllers::reset() {
    roll_integral = 0.0f;
    last_roll_error = 0.0f;
    last_pitch_error = 0.0f;
}

float FlightControllers::clamp(float value, float min_val, float max_val) {
    return std::max(min_val, std::min(value, max_val));
}

uint16_t FlightControllers::update_roll_control(float target_heading, float current_heading, float current_roll, float dt) {
    if (dt <= 0.0f) return sc.center_pwm;

    // ------------------------------------------------------------------------
    // OUTER LOOP: HEADING-TO-ROLL
    // ------------------------------------------------------------------------
    float heading_error = target_heading - current_heading;

    // Wrap heading error to (-PI to +PI) so the plane always turns the shortest way
    while (heading_error >  M_PI) heading_error -= 2.0f * M_PI;
    while (heading_error < -M_PI) heading_error += 2.0f * M_PI;

    // Command a target roll angle proportional to heading error
    float target_roll = heading_error * rc.p_heading;

    // Cap the roll demand to structural/aerodynamic bank limits
    target_roll = clamp(target_roll, -rc.max_roll, rc.max_roll);

    // ------------------------------------------------------------------------
    // INNER LOOP: ROLL-TO-SERVO (PID)
    // ------------------------------------------------------------------------
    float roll_error = target_roll - current_roll;

    // Proportional
    float p_term = rc.kp * roll_error;

    // Integral with anti-windup clamping
    roll_integral += roll_error * dt;
    float i_term = rc.ki * roll_integral;
    i_term = clamp(i_term, -rc.i_term_clamp, rc.i_term_clamp);

    // Derivative
    float d_term = rc.kd * ((roll_error - last_roll_error) / dt);
    last_roll_error = roll_error;

    // Combined control deflection command
    float total_output = p_term + i_term + d_term;

    // Map control output to hardware PWM microseconds
    int16_t servo_offset = static_cast<int16_t>(total_output * sc.scale_factor);
    servo_offset = clamp(servo_offset, -sc.max_trim, sc.max_trim);

    return static_cast<uint16_t>(sc.center_pwm + servo_offset);
}

uint16_t FlightControllers::update_pitch_control(float target_sink, float current_sink, float current_speed, float current_pitch, float dt) {
    if (dt <= 0.0f) return sc.center_pwm;

    float target_pitch = 0.0f;

    // ------------------------------------------------------------------------
    // LAYER 1: CRITICAL ENVELOPE PROTECTION (Airspeed Priority)
    // ------------------------------------------------------------------------
    if (current_speed < pc.v_min_safe) {
        // Stall Prevention Alpha: Force nose down proportionally to recover speed
        float speed_error = current_speed - pc.v_min_safe; // Negative error
        target_pitch = speed_error * pc.p_speed;
    }
    else if (current_speed > pc.v_max_safe) {
        // Overspeed Prevention Beta: Pull nose up proportionally to protect airframe
        float overspeed_error = current_speed - pc.v_max_safe; // Positive error
        target_pitch = overspeed_error * pc.p_speed;
    }
    // ------------------------------------------------------------------------
    // LAYER 2: COMPETITION MISSION LOOP (Constant Descent Rate Tracking)
    // ------------------------------------------------------------------------
    else {
        // Airspeed is safe -> track constant sink relative to ground.
        // Invert the tracking logic to fit glider competition rules:
        // Ex: Entering a thermal drops current_sink below target_sink.
        // sink_error = positive -> requires a NEGATIVE target pitch to force nose down.
        float sink_error = target_sink - current_sink;
        target_pitch = -1.0f * (sink_error * pc.p_sink);
    }

    // Apply strict nose limits to prevent vertical lawn-darting or dramatic loops
    target_pitch = clamp(target_pitch, pc.max_pitch_dn, pc.max_pitch_up);

    // ------------------------------------------------------------------------
    // LAYER 3: INNER PITCH-TO-SERVO LOOP (PD Controller)
    // ------------------------------------------------------------------------
    float pitch_error = target_pitch - current_pitch;

    float p_term = pc.kp * pitch_error;
    float d_term = pc.kd * ((pitch_error - last_pitch_error) / dt);
    last_pitch_error = pitch_error;

    float total_output = p_term + d_term;

    // Map control output to hardware PWM microseconds
    int16_t servo_offset = static_cast<int16_t>(total_output * sc.scale_factor);
    servo_offset = clamp(servo_offset, -sc.max_trim, sc.max_trim);

    return static_cast<uint16_t>(sc.center_pwm + servo_offset);
}
