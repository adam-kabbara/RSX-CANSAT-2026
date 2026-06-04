#pragma once
//
// PathGuidance.hpp  --  RSX CanSat 2026 "Autonomous Paraglider"  (v1 + replan)
//
// Three-phase heuristic descent guidance:
//   Phase 1 (Homing)   : straight descending leg  start -> loiter entry
//   Phase 2 (Loiter)   : constant-radius descending helix that burns the
//                        leftover altitude budget (continuous sweep Theta)
//   Phase 3 (Approach) : straight descending leg  loiter exit -> landing,
//                        tangent to the desired landing heading (into wind).
//
// The reference path is single-valued in altitude (z = D, "down", increasing),
// so it is a function P(D) -> (N,E). Guidance is pure pursuit: chase a carrot
// `lookahead_drop` metres of altitude below the glider.
//
// The homing heading is DERIVED (aim at the loiter entry) -- not an input.
// The loiter sweep is continuous, so the horizontal budget closes exactly.
//
// NEW in this build:
//   * replan(state): receding-horizon re-plan from the current state, keeping
//     the landing anchored. Re-derives the homing leg (in Homing) or the
//     remaining loiter sweep (in Loiter). Adopts the MEASURED glide ratio
//     (|v_h|/v_d) so the energy budget closes against actual performance.
//   * arc-length debug accessors (evalS / curvatureAt / segments) for the sim.
//
// Frame: NED. Heading North->East (rad). Embedded: float-only, no heap/except/STL.
//
#include <cstdint>

namespace rsx {

struct Vec2 { float n = 0.f; float e = 0.f; };

enum class Phase : uint8_t { Init = 0, Homing = 1, Loiter = 2, Approach = 3, Landed = 4 };

enum class PlanStatus : uint8_t {
    Ok               = 0,
    AdjustedApproach = 1,   // shortened phase 3 to fit the altitude budget
    Infeasible       = 2    // not enough altitude even after shortening
};

enum class SegType : uint8_t { Line = 0, Arc = 1 };   // v1 has no clothoids

struct GuidanceParams {
    float start_n = 0.f, start_e = 0.f, start_d = 0.f;   // pt1 (top)
    float land_n  = 0.f, land_e  = 0.f, land_d  = 0.f;   // pt4 (bottom)
    float land_heading = 0.f;        // final-approach direction, rad (into wind)

    float glide_ratio = 3.0f;        // horizontal / vertical (nominal)

    float approach_len    = 60.f;    // phase 3 horizontal length (m)
    float loiter_radius   = 40.f;    // loiter radius (m)
    float min_turn_radius = 25.f;    // floor used when flexing approach
    int8_t loiter_dir     = +1;      // +1 right/CW, -1 left/CCW (top-down)

    float lookahead_drop  = 8.f;     // carrot altitude below glider (m)
};

struct State {
    float n = 0.f, e = 0.f, d = 0.f;
    float vn = 0.f, ve = 0.f, vd = 0.f;
    float roll = 0.f, pitch = 0.f, yaw = 0.f;
};

struct HeadingCmd {
    float heading     = 0.f;
    Vec2  carrot      {};
    Phase phase       = Phase::Init;
    float glide_angle = 0.f;
    bool  valid       = false;
};

class PathGuidance {
public:
    PathGuidance() = default;
    explicit PathGuidance(const GuidanceParams& p) { setParams(p); }

    void       setParams(const GuidanceParams& p);
    PlanStatus plan();                              // solve from the mission start
    PlanStatus replan(const State& s);              // re-solve from the current state
    HeadingCmd getHeading(const State& s) const;
    Vec2       pathAt(float d) const;               // reference path at altitude d

    // arc-length views (debug / sim): s in [0, totalLength()]
    Vec2  evalS(float s, float* psi = nullptr, float* kappa = nullptr) const;
    float curvatureAt(float s) const;
    float totalLength() const { return gr_ * (d_pt2_ - start_d_) + p_.loiter_radius * theta_ + resolved_L3_; }
    float arcFromAlt(float d) const { return gr_ * (d - start_d_); }
    int   segCount() const { return 3; }
    SegType segType(int i) const { return (i == 1) ? SegType::Arc : SegType::Line; }
    float segS0(int i)  const;
    float segLen(int i) const;

    // accessors (debug / visualisation)
    const GuidanceParams& params() const { return p_; }
    PlanStatus status()      const { return status_; }
    Vec2  entry()            const { return pt2_; }
    Vec2  exit()             const { return pt3_; }
    Vec2  center()           const { return c_;  }
    float loiterSweep()      const { return theta_; }
    float loiterTurns()      const { return theta_ * 0.15915494309f; }
    float homingHeading()    const { return psi1_; }
    float dEntry()           const { return d_pt2_; }
    float dExit()            const { return d_pt3_; }
    float resolvedApproachLen() const { return resolved_L3_; }
    float workingGlideRatio()   const { return gr_; }
    float budgetResidual()   const { return residual_; }

private:
    PlanStatus solveFrom(Vec2 start, float start_d);   // shared by plan()/replan()

    GuidanceParams p_{};
    PlanStatus     status_ = PlanStatus::Infeasible;

    Vec2  start_{}, land_{};
    Vec2  pt2_{}, pt3_{}, c_{};
    float theta_ = 0.f;
    float psi1_  = 0.f, psi4_ = 0.f, a3_ = 0.f;
    float d_pt2_ = 0.f, d_pt3_ = 0.f;
    float resolved_L3_ = 0.f;
    float start_d_ = 0.f;        // altitude at the plan's start (current d on replan)
    float gr_ = 3.f;             // working glide ratio (nominal on plan, measured on replan)
    float residual_ = 0.f;
};

} // namespace rsx
