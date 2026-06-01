#include "flight_plan.h"
#include <math.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/*  FlightState helpers                                                */
/* ------------------------------------------------------------------ */
void FlightState_Init(FlightState_t *s)
{
    s->x = 0.0f;  s->y = 0.0f;  s->alt = 0.0f;
    s->vx = 0.0f; s->vy = 0.0f; s->vz = 0.0f;
    s->q1 = 1.0f; s->q2 = 0.0f; s->q3 = 0.0f; s->q4 = 0.0f;
    s->omega_x = 0.0f; s->omega_y = 0.0f; s->omega_z = 0.0f;
    s->roll = 0.0f; s->pitch = 0.0f; s->heading = 0.0f;
    s->phase = PHASE_LAUNCH_PAD;
    s->t = 0.0f;
    s->egg_released = false;
    s->drop_heading = 0.0f;
}

/*
 * Call once per tick after populating q1-q4 from the EKF.
 * Extracts roll, pitch, heading from the quaternion.
 *
 * Quaternion convention assumed: q1=w, q2=x, q3=y, q4=z
 * (Hamilton convention — confirm this matches your EKF output)
 */
void FlightState_UpdateDerived(FlightState_t *s)
{
    float w = s->q1, x = s->q2, y = s->q3, z = s->q4;

    s->roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));

    float sin_pitch = 2.0f * (w * y - z * x);
    if (sin_pitch >  1.0f) sin_pitch =  1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    s->pitch = asinf(sin_pitch);

    s->heading = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

float FlightState_HorizontalDistTo(const FlightState_t *s, const Target_t *t)
{
    return hypotf(t->x - s->x, t->y - s->y);
}

/* Bearing from current position to target (rad, 0=East, CCW+). */
float FlightState_BearingTo(const FlightState_t *s, const Target_t *t)
{
    return atan2f(t->y - s->y, t->x - s->x);
}

/* ------------------------------------------------------------------ */
/*  Coordinate utility                                                 */
/* ------------------------------------------------------------------ */
void latlon_to_enu(float lat, float lon,
                   float lat0, float lon0,
                   float *east, float *north)
{
    *north = DEG2RAD(lat - lat0) * EARTH_RADIUS_M;
    *east  = DEG2RAD(lon - lon0) * EARTH_RADIUS_M * cosf(DEG2RAD(lat0));
}

/* ------------------------------------------------------------------ */
/*  Glide reachability check                                           */
/* ------------------------------------------------------------------ */
/*
 * True if current altitude and distance put the target inside the glide
 * envelope — i.e. descending at the commanded sink rate we will arrive at
 * approximately EGG_DROP_ALT.
 *
 * GAP-3 FIX: uses GLIDE_RATIO_EFF (AIRSPEED / V_DESCENT_TARGET) instead of
 * the clean GLIDE_RATIO, so the predicted range matches the descent profile
 * actually being commanded in cruise/spiral.
 */
bool can_reach_target(const FlightState_t *s, const Target_t *t)
{
    float dist_h      = FlightState_HorizontalDistTo(s, t);
    float alt_to_lose = s->alt - EGG_DROP_ALT;
    float glide_range = alt_to_lose * GLIDE_RATIO_EFF;

    bool within_range   = glide_range >= dist_h * (1.0f - GLIDE_TOLERANCE);
    bool wont_overshoot = glide_range <= dist_h * (1.0f + GLIDE_TOLERANCE);
    return within_range && wont_overshoot;
}

/* ------------------------------------------------------------------ */
/*  Phase classifier — evaluated fresh every tick                      */
/* ------------------------------------------------------------------ */
/*
 * GAP-1 FIX: LANDING is latched once the egg is released and stays latched.
 * GAP-2 FIX: TERMINAL requires BOTH the glide band (can_reach_target) AND
 *            being below TP_ENTRY_ALT, so the straight final run only starts
 *            when low and lined up. Until then we spiral, bleeding altitude.
 */
Phase_t classify_phase(const FlightState_t *s, const Target_t *t)
{
    if (s->egg_released)
        return PHASE_LANDING;

    bool low_enough = (s->alt <= TP_ENTRY_ALT);

    if (low_enough && can_reach_target(s, t))
        return PHASE_TERMINAL;

    if (FlightState_HorizontalDistTo(s, t) <= SPIRAL_ENTRY_DIST)
        return PHASE_PHASE2_SPIRAL;

    return PHASE_PHASE1_CRUISE;
}

/* ------------------------------------------------------------------ */
/*  Egg release — checked every tick (GAP-5 FIX)                       */
/* ------------------------------------------------------------------ */
/*
 * Release when at/below drop altitude and essentially over the target,
 * regardless of which phase we're in. Captures the current heading so the
 * landing roll-out flies straight ahead onto the strip past the target.
 */
void maybe_release_egg(FlightState_t *s, const Target_t *t)
{
    if (s->egg_released)
        return;

    bool low       = (s->alt <= EGG_DROP_ALT);
    bool over_tgt  = (FlightState_HorizontalDistTo(s, t) <= SPIRAL_RADIUS); /* close enough */

    if (low && over_tgt) {
        trigger_egg_release();
        s->egg_released = true;
        s->drop_heading = s->heading;   /* fly this out during landing */
    }
}

/* ------------------------------------------------------------------ */
/*  Phase velocity planners                                            */
/* ------------------------------------------------------------------ */

/* Cruise: fly straight toward target. */
static VelCmd_t p1(const FlightState_t *s, const Target_t *t)
{
    float bearing = FlightState_BearingTo(s, t);
    VelCmd_t v;
    v.vx = AIRSPEED * cosf(bearing);
    v.vy = AIRSPEED * sinf(bearing);
    v.vz = -V_DESCENT_TARGET;
    return v;
}

/*
 * Spiral: vector field orbit around target at SPIRAL_RADIUS.
 * Tangential component keeps it on the circle; radial component
 * corrects radius error.
 */
static VelCmd_t p2(const FlightState_t *s, const Target_t *t)
{
    float dx = s->x - t->x;
    float dy = s->y - t->y;
    float r  = hypotf(dx, dy);
    if (r < 1e-3f) r = 1e-3f;

    float r_hat_x = dx / r;
    float r_hat_y = dy / r;
    float t_hat_x = -dy / r;          /* 90 deg CCW = left-hand orbit */
    float t_hat_y =  dx / r;

    float k           = 0.6f;
    float radial_gain = k * (r - SPIRAL_RADIUS) / SPIRAL_RADIUS;
    if (radial_gain >  0.8f) radial_gain =  0.8f;
    if (radial_gain < -0.8f) radial_gain = -0.8f;

    float dir_x = t_hat_x - radial_gain * r_hat_x;
    float dir_y = t_hat_y - radial_gain * r_hat_y;
    float mag   = hypotf(dir_x, dir_y);

    VelCmd_t v;
    v.vx = AIRSPEED * dir_x / mag;
    v.vy = AIRSPEED * dir_y / mag;
    v.vz = -V_DESCENT_TARGET;
    return v;
}

/* Terminal: fly straight at target on the clean glide down to drop alt. */
static VelCmd_t tp(const FlightState_t *s, const Target_t *t)
{
    float bearing = FlightState_BearingTo(s, t);
    VelCmd_t v;
    v.vx = AIRSPEED * cosf(bearing);
    v.vy = AIRSPEED * sinf(bearing);
    v.vz = -V_DESCENT_CLEAN;          /* clean glide; GLIDE_RATIO=6 holds here */
    return v;
}

/*
 * Landing: egg is already gone. Fly the heading captured at drop, wings
 * level, clean glide, straight out onto the strip ahead of the target.
 * (GAP-6 FIX: hold drop_heading instead of steering back to the target,
 *  which would circle it rather than roll out past it.)
 */
static VelCmd_t lp(const FlightState_t *s, const Target_t *t)
{
    (void)t;
    VelCmd_t v;
    v.vx = AIRSPEED * cosf(s->drop_heading);
    v.vy = AIRSPEED * sinf(s->drop_heading);
    v.vz = -V_DESCENT_CLEAN;
    return v;
}

/* TODO: send CMD,<TEAM_ID>,MEC,EGG,ON to servo layer. */
void trigger_egg_release(void)
{
}

/* ------------------------------------------------------------------ */
/*  plan_v — selects phase, handles egg release, returns velocity      */
/* ------------------------------------------------------------------ */
VelCmd_t plan_v(FlightState_t *s, const Target_t *t)
{
    /* Egg release is checked every tick, before phase dispatch (GAP-5). */
    maybe_release_egg(s, t);

    Phase_t phase = classify_phase(s, t);
    s->phase = phase;

    switch (phase) {
        case PHASE_TERMINAL:      return tp(s, t);
        case PHASE_PHASE2_SPIRAL: return p2(s, t);
        case PHASE_PHASE1_CRUISE: return p1(s, t);
        default:                  return lp(s, t);   /* LANDING */
    }
}

/* ------------------------------------------------------------------ */
/*  Mission termination (GAP-1 FIX)                                    */
/* ------------------------------------------------------------------ */
/*
 * The loop keeps running through LANDING and only stops once the glider is
 * actually on the ground. Landing therefore really executes (lp runs),
 * rather than the loop exiting the instant LANDING is entered.
 */
bool mission_active(const FlightState_t *s)
{
    if (s->egg_released && s->alt <= 0.0f)
        return false;   /* touched down after the drop */
    return true;
}

/* ------------------------------------------------------------------ */
/*  Main loop                                                          */
/* ------------------------------------------------------------------ */
static const char *phase_name(Phase_t p)
{
    switch (p) {
        case PHASE_LAUNCH_PAD:    return "LAUNCH_PAD";
        case PHASE_PHASE1_CRUISE: return "PHASE1_CRUISE";
        case PHASE_PHASE2_SPIRAL: return "PHASE2_SPIRAL";
        case PHASE_TERMINAL:      return "TERMINAL";
        case PHASE_LANDING:       return "LANDING";
        default:                  return "UNKNOWN";
    }
}

int main(void)
{
    const float TARGET_LAT = 38.0000f, TARGET_LON = -97.0000f;
    const float ORIGIN_LAT = 38.0010f, ORIGIN_LON = -97.0010f;

    Target_t target;
    latlon_to_enu(TARGET_LAT, TARGET_LON, ORIGIN_LAT, ORIGIN_LON,
                  &target.x, &target.y);

    FlightState_t state;
    FlightState_Init(&state);

    while (mission_active(&state)) {
        /* --- Populate from EKF each tick --- (to be finalized) ---
         * ekf_position(&state.x, &state.y, &state.alt);
         * ekf_velocity(&state.vx, &state.vy, &state.vz);
         * ekf_quaternion(&state.q1, &state.q2, &state.q3, &state.q4);
         * ekf_gyro_rates(&state.omega_x, &state.omega_y, &state.omega_z);
         */

        FlightState_UpdateDerived(&state);  /* roll, pitch, heading */
        state.t += 1.0f;

        VelCmd_t v = plan_v(&state, &target);

        /* TODO: send to servo driver */

        printf("t=%.0fs | phase=%s | alt=%.1fm | hdg=%.1f deg | egg=%d | "
               "VX=%.1f VY=%.1f VZ=%.1f\n",
               state.t, phase_name(state.phase), state.alt,
               RAD2DEG(state.heading), (int)state.egg_released,
               v.vx, v.vy, v.vz);

        /* DEMO ONLY: no EKF feeding state, so nothing changes and the loop
         * cannot terminate naturally. Remove this break once ekf_* calls
         * above are populating position/altitude each tick. */
        if (state.t > 5.0f) break;
    }

    return 0;
}