#pragma once
//
// PathGuidance.hpp  --  RSX CanSat 2026 "Autonomous Paraglider"  (v3)
//
// "Solve from current state" descent guidance. One routine, solve(state), is
// called at deploy AND at every replan: from wherever the glider is right now
// (position, altitude, heading) it computes a feasible path to the landing.
//
//   ARC(align) -> LINE(homing) -> SPIRAL(N loops, radius R) -> LINE(approach) -> land
//   every junction tangent (G1); curvature steps at corners.
//
// Inputs are boundary conditions only: current state + landing spec (position,
// AXIS, bounding box). The planner SOLVES the path parameters:
//   * landing direction (which end of the axis)   -- chosen by geometry
//   * spiral side (CW/CCW)                         -- chosen by geometry
//   * spiral radius R                              -- CONTINUOUS closure knob,
//       sized so total arc == glide_ratio*(z - z_land)  => exact budget closure
//   * loop count N                                 -- coarse closure
//   * Dubins align entry that respects the deploy/current heading.
// Hard bounding box: candidates whose path leaves the box are rejected.
//
// Altitude is the independent, monotone-decreasing variable. replan() is just
// solve() from the current state with the MEASURED glide ratio, so an
// off-nominal deploy z or a mid-flight z-shift is handled by re-solving.
//
// Frame: NED. Heading North->East (rad). Embedded: float-only, no heap/except/STL.
//
#include <cstdint>

namespace rsx {

struct Vec2 { float n = 0.f; float e = 0.f; };

enum class Phase : uint8_t { Init = 0, Align = 1, Homing = 2, Loiter = 3, Approach = 4, Landed = 5 };
enum class PlanStatus : uint8_t { Ok = 0, Degraded = 1, Infeasible = 2 };
enum class SegType : uint8_t { Line = 0, Arc = 1 };

struct Segment {
    SegType type = SegType::Line;
    float s0 = 0.f, len = 0.f;
    float n0 = 0.f, e0 = 0.f, psi0 = 0.f, kappa0 = 0.f;   // kappa0 = dpsi/ds (signed)
};

struct GuidanceParams {
    // ── Deploy / current-state position ──────────────────────────────────────
    // Position in the EKF local NED frame (metres from home/launch origin).
    // Fill from ekf_get_pos() at the moment the wings deploy.
    float start_n = 0.f;       // North offset from home (m)
    float start_e = 0.f;       // East  offset from home (m)
    float start_d = 0.f;       // Down  offset from home (m), positive = below home

    // Deploy heading in NED radians (0 = North, +π/2 = East).
    // Prefer course-over-ground (atan2(vE, vN)) when moving; fall back to yaw.
    float start_heading = 0.f;

    // ── Landing target ────────────────────────────────────────────────────────
    // Landing position in the EKF local NED frame (metres).
    // Convert from GPS lat/lon with convert_gps_to_local_ned().
    float land_n  = 0.f;       // North offset from home (m)
    float land_e  = 0.f;       // East  offset from home (m)
    float land_d  = 0.f;       // Down at landing — 0.0 = same altitude as home

    // Landing axis orientation in NED radians (0 = N-S axis, +π/2 = E-W axis).
    // The solver picks which end of the axis to approach from (180° ambiguity resolved).
    // Compute via gps_bearing_rad(landing_lat, landing_lon, axis_ref_lat, axis_ref_lon);
    // use the "AXIS" ground command to set the axis reference GPS point.
    float landing_axis = 0.f;

    // ── Glider aerodynamics ───────────────────────────────────────────────────
    // Nominal glide ratio (horizontal distance / altitude drop).
    // replan() updates this in-flight using the measured L/D.
    float glide_ratio = 3.0f;

    // ── Path geometry ─────────────────────────────────────────────────────────
    // Length of the final straight approach leg leading into the landing point (m).
    float approach_len = 40.f;

    // Minimum turn radius (m). Used for the Dubins align arc and as the lower
    // bound on the spiral radius. Must match the glider's physical minimum.
    float min_turn_radius = 25.f;

    // Maximum allowed spiral radius (m). Caps how wide the loiter circle grows.
    float max_radius = 70.f;

    // ── Hard bounding box (NED, metres from EKF home) ─────────────────────────
    // Any candidate path that exits this rectangle is rejected.
    // Set to the competition field boundary converted to the NED frame.
    float box_n_min = -250.f, box_n_max = 250.f;
    float box_e_min = -250.f, box_e_max = 250.f;

    // ── Lookahead tracker ─────────────────────────────────────────────────────
    // How far ahead (in altitude drop, m) the "carrot" point is placed on the
    // path for pure-pursuit tracking. Larger = smoother, slower to respond.
    float lookahead_drop = 8.f;
};

struct State {
    float n = 0.f, e = 0.f, d = 0.f;
    float vn = 0.f, ve = 0.f, vd = 0.f;
    float roll = 0.f, pitch = 0.f, yaw = 0.f;
};

struct HeadingCmd {
    float heading = 0.f;
    Vec2  carrot {};
    Phase phase = Phase::Init;
    float glide_angle = 0.f;
    bool  valid = false;
};

class PathGuidance {
public:
    static constexpr int kMaxSeg = 4;

    PathGuidance() = default;
    explicit PathGuidance(const GuidanceParams& p) { setParams(p); }

    void       setParams(const GuidanceParams& p);
    PlanStatus plan();                          // solve from the deploy state
    PlanStatus replan(const State& s);          // solve from the current state
    HeadingCmd getHeading(const State& s) const;

    Vec2  pathAt(float d) const { return evalS(arcFromAlt(d)); }
    Vec2  evalS(float s, float* psi = nullptr, float* kappa = nullptr) const;
    float curvatureAt(float s) const;
    float totalLength() const { return total_len_; }
    float arcFromAlt(float d) const { return gr_ * (d - start_d_); }

    int   segCount() const { return n_seg_; }
    SegType segType(int i) const { return seg_[i].type; }
    float segS0(int i)  const { return seg_[i].s0; }
    float segLen(int i) const { return seg_[i].len; }

    const GuidanceParams& params() const { return p_; }
    PlanStatus status()  const { return status_; }
    Vec2  center()       const { return c_; }
    float spiralRadius() const { return R_; }
    int   loops()        const { return N_; }
    float landingDirection() const { return psi4_; }
    int   spiralSide()   const { return sdir_; }
    float budgetResidual() const { return residual_; }
    float workingGlideRatio() const { return gr_; }

private:
    struct Tangent { bool ok=false; Vec2 TA{}, TB{}; float legHeading=0.f; };
    struct AlignEntry { bool ok=false; Vec2 O1{}; float sA=0.f, aP0=0.f, aTA=0.f, sweepA=0.f;
                        Vec2 TA{}, TB{}; float leg=0.f, aEntry=0.f, entryLen=0.f, legHeading=0.f; };
    struct Candidate { bool ok=false; float landDir=0.f; int sdir=1; float R=0.f; int N=0;
                       Vec2 C{}, pt3{}; float psi4=0.f, aExit=0.f, aEntry=0.f, partial=0.f;
                       AlignEntry ent{}; float cost=1e30f, budget=0.f, clear=0.f; };

    Tangent    circleTangent(Vec2 O1, float r1, int s1, Vec2 O2, float r2, int s2) const;
    AlignEntry alignEntry(Vec2 P0, float chi0, Vec2 C, float R, int sdir) const;
    Candidate  buildCandidate(Vec2 P0, float chi0, float dStart, float landDir, int sdir) const;
    void       pathExtents(const Candidate& c, Vec2 P0, float chi0,
                           float& nmin, float& nmax, float& emin, float& emax) const;
    PlanStatus solveState(Vec2 P0, float chi0, float dStart);
    PlanStatus buildLoiterTail(Vec2 cur, float dCur);   // replan while established in the loiter
    void       emitSegments(const Candidate& c, Vec2 P0, float chi0);
    void       pushSeg(SegType t, float len, float n0, float e0, float psi0, float kappa0);

    GuidanceParams p_{};
    PlanStatus     status_ = PlanStatus::Infeasible;
    Segment seg_[kMaxSeg];
    int     n_seg_ = 0;

    Vec2  c_{}, start_{};
    float R_ = 0.f; int N_ = 0; int sdir_ = 1;
    float psi4_ = 0.f;
    float start_d_ = 0.f, gr_ = 3.f, total_len_ = 0.f, residual_ = 0.f;
};

} // namespace rsx
