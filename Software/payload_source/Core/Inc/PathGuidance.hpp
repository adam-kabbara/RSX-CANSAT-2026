#pragma once
//
// PathGuidance.hpp  --  RSX CanSat 2026 "Autonomous Paraglider"
//
// Three-phase heuristic descent guidance:
//   Phase 1 (Homing)   : straight descending leg  start -> loiter entry
//   Phase 2 (Loiter)   : constant-radius descending helix that burns the
//                        leftover altitude budget (continuous sweep Theta)
//   Phase 3 (Approach) : straight descending leg  loiter exit -> landing,
//                        tangent to the desired landing heading.
//
// The whole reference path is single-valued in altitude (z = D, "down",
// monotonically increasing), so it is a function  P(D) -> (N, E).
// Guidance is pure-pursuit: chase a "carrot" point that sits `lookahead_drop`
// metres of altitude below the glider, on the reference path.
//
// Frame: NED (North-East-Down). z = D points down (start is the *smallest* D,
// landing the *largest*). Heading is measured from North toward East (rad).
//
// Embedded notes (STM32G431 / Cortex-M4F, single-precision FPU):
//   - everything is float (no double -> no soft-float fallback)
//   - no heap, no exceptions, no STL containers
//   - plan() does the (rare) geometry solve; getHeading() is cheap per tick
//
#include <cstdint>

namespace rsx {

struct Vec2 { float n = 0.f; float e = 0.f; };   // horizontal NED point/vector

enum class Phase : uint8_t {
    Init     = 0,
    Homing   = 1,   // phase 1
    Loiter   = 2,   // phase 2
    Approach = 3,   // phase 3
    Landed   = 4
};

enum class PlanStatus : uint8_t {
    Ok               = 0,   // geometry closed with the requested params
    AdjustedApproach = 1,   // had to shorten phase 3 to fit the altitude budget
    Infeasible       = 2    // not enough altitude even after shortening phase 3
};

struct GuidanceParams {
    // --- mission geometry (NED, metres) ------------------------------------
    float start_n = 0.f,  start_e = 0.f,  start_d = 0.f;   // pt1 (top)
    float land_n  = 0.f,  land_e  = 0.f,  land_d  = 0.f;   // pt4 (bottom)
    float land_heading = 0.f;        // final-approach direction, rad (into wind)

    // --- performance -------------------------------------------------------
    float glide_ratio = 3.0f;        // horizontal / vertical (nominal)

    // --- phase geometry ----------------------------------------------------
    float approach_len    = 60.f;    // phase 3 horizontal length (m)
    float loiter_radius   = 40.f;    // phase 2 loiter radius ("max radius", m)
    float min_turn_radius = 25.f;    // hard floor (used by flex / transitions)
    int8_t loiter_dir     = +1;      // +1 = right/CW, -1 = left/CCW (viewed top-down)

    // --- guidance ----------------------------------------------------------
    float lookahead_drop  = 8.f;     // "var": carrot altitude below glider (m)
};

struct State {
    float n = 0.f, e = 0.f, d = 0.f;        // position  (NED)
    float vn = 0.f, ve = 0.f, vd = 0.f;     // velocity  (NED)
    float roll = 0.f, pitch = 0.f, yaw = 0.f; // rad (replaces the quaternion)
};

struct HeadingCmd {
    float heading     = 0.f;            // commanded ground-track heading, rad (NED)
    Vec2  carrot      {};               // carrot point (N,E) -- for debugging/plot
    Phase phase       = Phase::Init;
    float glide_angle = 0.f;            // measured glide angle, rad (monitoring)
    bool  valid       = false;
};

class PathGuidance {
public:
    PathGuidance() = default;
    explicit PathGuidance(const GuidanceParams& p) { setParams(p); }

    void       setParams(const GuidanceParams& p);
    PlanStatus plan();                              // solve geometry (call at each main waypoint)
    HeadingCmd getHeading(const State& s) const;    // call every control tick

    // evaluate the reference path at altitude d -> (N,E). Handy for plotting.
    Vec2 pathAt(float d) const;

    // --- accessors (debug / visualisation) ---------------------------------
    const GuidanceParams& params() const { return p_; }
    PlanStatus status()      const { return status_; }
    Vec2  entry()            const { return pt2_; }    // loiter entry  (pt2)
    Vec2  exit()             const { return pt3_; }    // loiter exit   (pt3)
    Vec2  center()           const { return c_;  }
    float loiterSweep()      const { return theta_; }  // total sweep, rad
    float loiterTurns()      const { return theta_ * 0.15915494309f; } // /2pi
    float homingHeading()    const { return psi1_; }
    float dEntry()           const { return d_pt2_; }
    float dExit()            const { return d_pt3_; }
    float resolvedApproachLen() const { return resolved_L3_; }

private:
    // returns f(theta) = L1 + R*theta + L3 - GR*Htot ; also outputs geometry
    float closeError(float L3, float theta,
                     Vec2& pt2_out, Vec2& c_out, float& a3_out) const;
    bool  solveTheta(float L3, float& theta_out) const;

    GuidanceParams p_{};
    PlanStatus     status_ = PlanStatus::Infeasible;

    Vec2  start_{}, land_{};
    Vec2  pt2_{}, pt3_{}, c_{};
    float theta_ = 0.f;       // loiter sweep (rad, >= 0)
    float psi1_  = 0.f;       // homing heading (start -> pt2)
    float psi4_  = 0.f;       // approach heading
    float a3_    = 0.f;       // loiter angle at exit
    float d_pt2_ = 0.f, d_pt3_ = 0.f;
    float resolved_L3_ = 0.f;
};

} // namespace rsx
