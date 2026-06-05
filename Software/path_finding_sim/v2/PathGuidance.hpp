#pragma once
//
// PathGuidance.hpp  (v2)  --  RSX CanSat 2026 "Autonomous Paraglider"
//
// Three-phase descent guidance, built from arc-length segments with
// curvature-continuous (G2) clothoid transitions:
//
//   LINE (homing) -> CLOTHOID -> ARC (loiter) -> CLOTHOID -> LINE (approach)
//   kappa: 0          0 -> 1/R   1/R            1/R -> 0      0
//
// Path is parameterised by arc length s; altitude maps to arc length by
// s = glide_ratio * (z - z0)  (descent is monotonic in z = D, "down").
// Guidance is pure pursuit on a carrot `lookahead_drop` metres below.
//
// replan(state) rebuilds only the *remaining* phases from the current state,
// keeping the landing anchor fixed -- receding-horizon re-closing of the
// altitude budget against what actually happened.
//
// Frame: NED. Heading North->East (rad). kappa = dpsi/ds (signed).
// Embedded: float-only (M4F FPU), no heap, no exceptions.
//
#include <cstdint>

namespace rsx {

struct Vec2 { float n = 0.f; float e = 0.f; };

enum class Phase : uint8_t { Init = 0, Homing = 1, Loiter = 2, Approach = 3, Landed = 4 };

enum class PlanStatus : uint8_t {
    Ok               = 0,
    AdjustedApproach = 1,   // flexed approach length to close the budget
    EntryInfeasible  = 2,   // homing heading cannot reach the loiter tangentially
    Infeasible       = 3    // not enough altitude even after flexing
};

enum class SegType : uint8_t { Line = 0, Clothoid = 1, Arc = 2 };

struct Segment {
    SegType type   = SegType::Line;
    float   s0     = 0.f;     // global arc length at segment start
    float   len    = 0.f;
    float   n0     = 0.f, e0 = 0.f, psi0 = 0.f;
    float   kappa0 = 0.f;     // curvature at local s=0 (signed)
    float   dkappa = 0.f;     // d(kappa)/ds; 0 for line/arc
};

struct GuidanceParams {
    float start_n = 0.f, start_e = 0.f, start_d = 0.f;
    float land_n  = 0.f, land_e  = 0.f, land_d  = 0.f;
    float start_heading = 0.f;       // homing heading INPUT (rad)
    float land_heading  = 0.f;       // approach heading INPUT (rad, into wind)

    float glide_ratio      = 3.0f;
    float approach_len     = 60.f;
    float approach_len_min = 25.f;
    float loiter_radius    = 40.f;
    float min_turn_radius  = 25.f;
    float transition_len   = 12.f;   // nominal clothoid length (roll-rate limited)
    float entry_clothoid_max = 0.f;  // cap on solved entry clothoid (0 = no cap);
                                     // flags EntryInfeasible if the homing ray is so
                                     // far off-tangent it needs a longer spiral
    int8_t loiter_dir      = +1;     // +1 right/CW, -1 left/CCW

    float lookahead_drop = 8.f;
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
    static constexpr int kMaxSeg = 5;

    PathGuidance() = default;
    explicit PathGuidance(const GuidanceParams& p) { setParams(p); }

    void       setParams(const GuidanceParams& p);
    PlanStatus plan();
    PlanStatus replan(const State& s);
    HeadingCmd getHeading(const State& s) const;

    Vec2  pathAt(float d) const;
    Vec2  evalS(float s, float* psi = nullptr, float* kappa = nullptr) const;
    float curvatureAt(float s) const;
    float totalLength() const { return total_len_; }
    float arcFromAlt(float d) const { return gr_ * (d - z0_); }

    const GuidanceParams& params() const { return p_; }
    PlanStatus status()  const { return status_; }
    int   segCount()     const { return n_seg_; }
    SegType segType(int i) const { return seg_[i].type; }
    float segStartN(int i) const { return seg_[i].n0; }
    float segStartE(int i) const { return seg_[i].e0; }
    float segS0(int i)     const { return seg_[i].s0; }
    float segLen(int i)    const { return seg_[i].len; }
    Vec2  center()       const { return c_; }
    float loiterTurns()  const { return loiter_sweep_ * 0.15915494309f; }
    float loiterRadiusResolved() const { return loiter_R_; }
    float resolvedApproachLen() const { return resolved_L3_; }
    float budgetResidual() const { return residual_; }

private:
    PlanStatus buildFromStart(Vec2 start, float psi1, float z0);
    PlanStatus buildLoiterTail(Vec2 cur, float z0);
    bool  solveEntry(Vec2 start, float psi1, float& L1, float& Lc, float& a_in) const;
    void  pushSeg(SegType t, float len, float n0, float e0, float psi0,
                  float kappa0, float dkappa);
    void  rebuildArcLengths();

    GuidanceParams p_{};
    PlanStatus     status_ = PlanStatus::Infeasible;

    Segment seg_[kMaxSeg];
    int     n_seg_ = 0;

    Vec2  c_{};
    Vec2  x_exit_{};
    float psi4_ = 0.f;
    float a_exit_ = 0.f;
    float loiter_sweep_ = 0.f;
    float loiter_R_ = 0.f;
    float resolved_L3_ = 0.f;
    float residual_ = 0.f;
    float z0_ = 0.f;
    float gr_ = 3.f;            // working glide ratio (nominal on plan, measured on replan)
    float total_len_ = 0.f;
};

} // namespace rsx
