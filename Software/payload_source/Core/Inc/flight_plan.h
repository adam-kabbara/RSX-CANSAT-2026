#ifndef FLIGHT_PLAN_H
#define FLIGHT_PLAN_H

#include <stdbool.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */
#define AIRSPEED            17.0f
#define GLIDE_RATIO         6.0f                       /* clean aero glide ratio (no extra sink) */
#define V_DESCENT_CLEAN     (AIRSPEED / GLIDE_RATIO)   /* ~2.83 m/s natural sink — MEASURE THIS */
#define V_DESCENT_TARGET    5.0f                       /* commanded sink in cruise/spiral (competition req.) */
#define EGG_DROP_ALT        2.0f
#define SPIRAL_RADIUS       60.0f
#define SPIRAL_ENTRY_DIST   80.0f
#define TP_ENTRY_ALT        30.0f                      /* must be below this to commit to terminal run */

/*
 * Effective glide ratio while descending at the *commanded* cruise/spiral
 * sink rate (V_DESCENT_TARGET), not the clean aero ratio. This is what
 * can_reach_target must use to predict range, because that's the descent
 * profile actually being flown when the check is consulted.
 *   AIRSPEED / V_DESCENT_TARGET = 17 / 5 = 3.4
 */
#define GLIDE_RATIO_EFF     (AIRSPEED / V_DESCENT_TARGET)

#define K_ROLL_P            1.5f
#define K_ROLL_D            0.3f
#define K_PITCH_P           1.2f
#define MAX_AILERON         30.0f
#define MAX_ELEVATOR        25.0f

#define GLIDE_TOLERANCE     0.20f   /* 20% band on can_reach check */

#define EARTH_RADIUS_M      6371000.0f

#define FP_PI               3.14159265358979323846f
#define RAD2DEG(r)          ((r) * 180.0f / FP_PI)
#define DEG2RAD(d)          ((d) * FP_PI / 180.0f)

/* ------------------------------------------------------------------ */
/*  Phase enum                                                         */
/* ------------------------------------------------------------------ */
typedef enum {
    PHASE_LAUNCH_PAD,
    PHASE_PHASE1_CRUISE,
    PHASE_PHASE2_SPIRAL,
    PHASE_TERMINAL,
    PHASE_LANDING
} Phase_t;

/* ------------------------------------------------------------------ */
/*  Data structures                                                    */
/* ------------------------------------------------------------------ */
typedef struct {
    float x;   /* East  (m) from launch origin */
    float y;   /* North (m) from launch origin */
} Target_t;

typedef struct {
    /* --- EKF output 1: position (local ENU, metres) --- */
    float x;     /* East  */
    float y;     /* North */
    float alt;   /* Up (AGL after CAL command at launch) */

    /* --- EKF output 2: velocity (m/s, ENU frame) --- */
    float vx;    /* East  */
    float vy;    /* North */
    float vz;    /* Up (negative = descending) */

    /* --- EKF output 3: quaternion (body orientation) --- */
    float q1;    /* w (scalar part) */
    float q2;    /* x */
    float q3;    /* y */
    float q4;    /* z */

    /* --- EKF output 4: gyro angular rates (rad/s, body frame) --- */
    float omega_x;   /* roll rate  */
    float omega_y;   /* pitch rate */
    float omega_z;   /* yaw rate   */

    /* --- Derived from quaternion (computed each tick) --- */
    float roll;      /* rad */
    float pitch;     /* rad */
    float heading;   /* rad, 0 = East, CCW positive */

    /* --- Mission bookkeeping --- */
    Phase_t phase;
    float   t;
    bool    egg_released;
    float   drop_heading;   /* heading captured at egg release, flown during landing */
} FlightState_t;

/* Velocity command returned by the planner (ENU, m/s) */
typedef struct {
    float vx;
    float vy;
    float vz;
} VelCmd_t;

/* ------------------------------------------------------------------ */
/*  API                                                                */
/* ------------------------------------------------------------------ */
void     FlightState_Init(FlightState_t *s);
void     FlightState_UpdateDerived(FlightState_t *s);
float    FlightState_HorizontalDistTo(const FlightState_t *s, const Target_t *t);
float    FlightState_BearingTo(const FlightState_t *s, const Target_t *t);

void     latlon_to_enu(float lat, float lon,
                       float lat0, float lon0,
                       float *east, float *north);

bool     can_reach_target(const FlightState_t *s, const Target_t *t);
Phase_t  classify_phase(const FlightState_t *s, const Target_t *t);

void     maybe_release_egg(FlightState_t *s, const Target_t *t);
VelCmd_t plan_v(FlightState_t *s, const Target_t *t);

bool     mission_active(const FlightState_t *s);

void     trigger_egg_release(void);

#endif /* FLIGHT_PLAN_H */