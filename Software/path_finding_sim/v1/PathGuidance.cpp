//
// PathGuidance.cpp  --  RSX CanSat 2026 "Autonomous Paraglider"
//
#include "PathGuidance.hpp"
#include <cmath>

namespace rsx {

static constexpr float kPi    = 3.14159265358979f;
static constexpr float kTwoPi = 6.28318530717959f;

static inline Vec2  dirOf (float psi) { return { cosf(psi),  sinf(psi) }; }
// "left perpendicular" in (N,E): for psi=North(1,0) -> (0,1)=East.
static inline Vec2  perpL (float psi) { return { -sinf(psi), cosf(psi) }; }

static inline float wrapPi(float a) {
    a = fmodf(a + kPi, kTwoPi);
    if (a < 0.f) a += kTwoPi;
    return a - kPi;
}

void PathGuidance::setParams(const GuidanceParams& p) {
    p_      = p;
    start_  = { p.start_n, p.start_e };
    land_   = { p.land_n,  p.land_e  };
    psi4_   = p.land_heading;
    status_ = PlanStatus::Infeasible;
}

// f(theta) = L1(theta) + R*theta + L3 - GR*H_total
// This is just the horizontal arc-length budget: homing + loiter + approach
// must equal the total horizontal distance the glider can fly for H_total of
// altitude. Outputs the geometry that goes with (L3, theta).
float PathGuidance::closeError(float L3, float theta,
                               Vec2& pt2_out, Vec2& c_out, float& a3_out) const {
    const float R   = p_.loiter_radius;
    const float GR  = p_.glide_ratio;
    const float dir = static_cast<float>(p_.loiter_dir);
    const float Htot = p_.land_d - p_.start_d;

    const Vec2 d4 = dirOf(psi4_);
    const Vec2 pt3 { p_.land_n - L3 * d4.n, p_.land_e - L3 * d4.e };

    // Loiter centre: tangent to the approach line at pt3, on the side set by
    // loiter_dir. (Derived from requiring the exit tangent to equal psi4.)
    const Vec2 pl = perpL(psi4_);
    const Vec2 c { pt3.n + R * dir * pl.n, pt3.e + R * dir * pl.e };

    // Loiter angle at the exit (radial from centre to pt3).
    const float a3 = atan2f(-dir * cosf(psi4_), dir * sinf(psi4_));

    // Walk back around the circle by the full sweep to find the entry (pt2).
    const float a2 = a3 - dir * theta;
    const Vec2 pt2 { c.n + R * cosf(a2), c.e + R * sinf(a2) };

    const float L1 = hypotf(pt2.n - start_.n, pt2.e - start_.e);

    pt2_out = pt2; c_out = c; a3_out = a3;
    return L1 + R * theta + L3 - GR * Htot;
}

// Find the smallest theta >= 0 that closes the budget for a given L3.
// f is dominated by the +R*theta ramp (one loop adds 2*pi*R of arc, far more
// than the +/-2R wiggle in L1), so it crosses zero once -> bisection.
bool PathGuidance::solveTheta(float L3, float& theta_out) const {
    Vec2 tmp2, tmpc; float tmpa;
    const float R = p_.loiter_radius;
    const float GR = p_.glide_ratio;
    const float Htot = p_.land_d - p_.start_d;

    const float f0 = closeError(L3, 0.f, tmp2, tmpc, tmpa);
    if (f0 > 0.f) return false;                  // even straight-in overshoots -> need shorter L3

    float lo = 0.f;
    float hi = (GR * Htot) / R + kTwoPi;         // generous upper bound
    // ensure bracket
    for (int i = 0; i < 8 && closeError(L3, hi, tmp2, tmpc, tmpa) < 0.f; ++i) hi *= 1.5f;

    for (int i = 0; i < 80; ++i) {               // fixed iterations -> deterministic
        const float mid = 0.5f * (lo + hi);
        const float fm = closeError(L3, mid, tmp2, tmpc, tmpa);
        if (fm < 0.f) lo = mid; else hi = mid;
    }
    theta_out = 0.5f * (lo + hi);
    return true;
}

PlanStatus PathGuidance::plan() {
    const float R    = p_.loiter_radius;
    const float GR   = p_.glide_ratio;
    const float Htot = p_.land_d - p_.start_d;
    psi4_ = p_.land_heading;

    if (Htot <= 0.f || GR <= 0.f || R <= 0.f) { status_ = PlanStatus::Infeasible; return status_; }

    // Normal case: enough altitude -> solve theta at the requested L3.
    float L3 = p_.approach_len;
    float theta = 0.f;
    PlanStatus st = PlanStatus::Ok;

    if (!solveTheta(L3, theta)) {
        // Infeasible-high: not enough altitude even at theta=0. Shorten the
        // approach (Adam's fallback) down to a floor until it closes.
        st = PlanStatus::AdjustedApproach;
        const float floorL3 = (p_.min_turn_radius > 1.f) ? p_.min_turn_radius : 1.f;
        bool ok = false;
        const float step = 0.5f;
        for (float cand = p_.approach_len; cand >= floorL3; cand -= step) {
            if (solveTheta(cand, theta)) { L3 = cand; ok = true; break; }
        }
        if (!ok) { status_ = PlanStatus::Infeasible; return status_; }
    }

    // Lock in the resolved geometry.
    Vec2 pt2, c; float a3;
    closeError(L3, theta, pt2, c, a3);

    resolved_L3_ = L3;
    theta_  = theta;
    a3_     = a3;
    c_      = c;
    pt3_    = { p_.land_n - L3 * dirOf(psi4_).n, p_.land_e - L3 * dirOf(psi4_).e };
    pt2_    = pt2;
    psi1_   = atan2f(pt2.e - start_.e, pt2.n - start_.n);

    d_pt3_  = p_.land_d - L3 / GR;
    d_pt2_  = d_pt3_ - (R * theta) / GR;

    status_ = st;
    return status_;
}

// Reference path P(d) -> (N,E). Valid for d in [start_d, land_d].
Vec2 PathGuidance::pathAt(float d) const {
    const float GR  = p_.glide_ratio;
    const float R   = p_.loiter_radius;
    const float dir = static_cast<float>(p_.loiter_dir);

    if (d <= d_pt2_) {                              // phase 1: homing
        float s = GR * (d - p_.start_d);
        if (s < 0.f) s = 0.f;
        const Vec2 u = dirOf(psi1_);
        return { start_.n + s * u.n, start_.e + s * u.e };
    }
    if (d <= d_pt3_) {                              // phase 2: loiter
        const float sweep = (R > 0.f) ? (d - d_pt2_) * GR / R : 0.f;
        const float a2 = a3_ - dir * theta_;
        const float a  = a2 + dir * sweep;
        return { c_.n + R * cosf(a), c_.e + R * sinf(a) };
    }
    // phase 3: approach
    float s = GR * (d - d_pt3_);
    if (s < 0.f) s = 0.f;
    if (s > resolved_L3_) s = resolved_L3_;
    const Vec2 u = dirOf(psi4_);
    return { pt3_.n + s * u.n, pt3_.e + s * u.e };
}

HeadingCmd PathGuidance::getHeading(const State& s) const {
    HeadingCmd cmd;
    cmd.valid = (status_ == PlanStatus::Ok || status_ == PlanStatus::AdjustedApproach);

    // phase from current altitude band
    if      (s.d <  d_pt2_)      cmd.phase = Phase::Homing;
    else if (s.d <  d_pt3_)      cmd.phase = Phase::Loiter;
    else if (s.d <  p_.land_d)   cmd.phase = Phase::Approach;
    else                          cmd.phase = Phase::Landed;

    // carrot: same path, lookahead_drop metres lower, clamped at landing
    float d_car = s.d + p_.lookahead_drop;
    if (d_car > p_.land_d) d_car = p_.land_d;
    const Vec2 carrot = (cmd.phase == Phase::Landed) ? land_ : pathAt(d_car);
    cmd.carrot = carrot;

    cmd.heading = atan2f(carrot.e - s.e, carrot.n - s.n);

    // measured glide angle, for feed-forward / energy monitoring only
    const float vh = hypotf(s.vn, s.ve);
    cmd.glide_angle = atan2f(s.vd, vh);

    (void)wrapPi; // available for downstream heading-error math
    return cmd;
}

} // namespace rsx
