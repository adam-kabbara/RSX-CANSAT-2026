#ifndef FLIGHT_CONTROLLERS_HPP
#define FLIGHT_CONTROLLERS_HPP

#include <cstdint>
#include <cmath>
#include <algorithm>

// Configuration structures for easy tuning
struct RollControllerConfig {
    float p_heading     = 1.5f;   // Outer loop: Heading error to target roll
    float kp            = 3.0f;   // Inner loop: Roll proportional gain
    float ki            = 0.0f;   // Inner loop: Roll integral gain
    float kd            = 0.1f;   // Inner loop: Roll derivative gain
    float max_roll      = 0.523f; // Maximum bank limit (~30 degrees in radians)
    float i_term_clamp  = 100.0f; // Anti-windup clamp for integral term
};

struct PitchControllerConfig {
    float p_sink        = 0.20f;  // Outer loop: Sink rate error to target pitch
    float p_speed       = 0.40f;  // Safety loop: Airspeed deviation to target pitch
    float kp            = 2.5f;   // Inner loop: Pitch proportional gain
    float kd            = 0.2f;   // Inner loop: Pitch derivative gain
    float max_pitch_up  = 0.174f; // Max nose-up limit (~10 degrees in radians)
    float max_pitch_dn  = -0.436f;// Max nose-down limit (~-25 degrees in radians)
    float v_min_safe    = 8.0f;   // Stall airspeed threshold (m/s)
    float v_max_safe    = 22.0f;  // Structural overspeed limit (m/s)
};

struct ServoHardwareConfig {
    int16_t center_pwm  = 1500;   // Standard servo center in microseconds
    int16_t max_trim    = 400;    // Maximum allowed deflection offset from center
    float scale_factor  = 500.0f; // Scale factor converting control output to PWM units
};

class FlightControllers {
public:
    FlightControllers(const RollControllerConfig& roll_cfg,
                      const PitchControllerConfig& pitch_cfg,
                      const ServoHardwareConfig& servo_cfg);

    /**
     * @brief Computes the Roll (Aileron) servo command.
     * @param target_heading Target heading angle from path planner (rad, -PI to +PI)
     * @param current_heading Current estimated heading from EKF (rad, -PI to +PI)
     * @param current_roll Current estimated roll angle from EKF (rad, -PI to +PI)
     * @param dt Sampling time delta in seconds
     * @return PWM pulse width in microseconds (typically 1100 - 1900)
     */
    uint16_t update_roll_control(float target_heading, float current_heading, float current_roll, float dt);

    /**
     * @brief Computes the Pitch (Elevator) servo command tailored for a strict constant descent rate.
     * Pitches down in thermals to maintain sink, with absolute airspeed guards.
     * @param target_sink Target descent rate (m/s, positive downward)
     * @param current_sink Current descent rate from EKF velocity Down (m/s)
     * @param current_speed Current airspeed proxy from EKF velocity magnitude (m/s)
     * @param current_pitch Current absolute pitch angle from EKF (rad, -PI to +PI)
     * @param dt Sampling time delta in seconds
     * @return PWM pulse width in microseconds (typically 1100 - 1900)
     */
    uint16_t update_pitch_control(float target_sink, float current_sink, float current_speed, float current_pitch, float dt);

    // Resets internal integrators and derivatives (call upon landing or on the launch line)
    void reset();

private:
    RollControllerConfig rc;
    PitchControllerConfig pc;
    ServoHardwareConfig sc;

    // Roll PID state variables
    float roll_integral;
    float last_roll_error;

    // Pitch PD state variables
    float last_pitch_error;

    // Helper to constrain values safely
    float clamp(float value, float min_val, float max_val);
};

#endif // FLIGHT_CONTROLLERS_HPP
