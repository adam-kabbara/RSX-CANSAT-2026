//
// PathGuidance.cpp  --  RSX CanSat 2026 "Autonomous Paraglider"  (v1 + replan)
//
#include "PathGuidance.hpp"
#include <cmath>

namespace rsx {

static constexpr float kPi    = 3.14159265358979f;
static constexpr float kTwoPi = 6.28318530717959f;

static inline Vec2  dirOf(float psi) { return { cosf(psi),  sinf(psi) }; }
static inline Vec2  perpL(float psi) { return { -sinf(psi), cosf(psi) }; } // North->East
static inline float wrapPi(float a) {
    a = fmodf(a + kPi, kTwoPi); if (a < 0.f) a += kTwoPi; return a - kPi;
}
static inline float wrap2pi(float a) {
    a = fmodf(a, kTwoPi); if (a < 0.f) a += kTwoPi; return a;
}

void PathGuidance::setParams(const GuidanceParams& p) {
    p_      = p;
    start_  = { p.start_n, p.start_e };
    land_   = { p.land_n,  p.land_e  };
    psi4_   = p.land_heading;
    gr_     = p.glide_ratio;
    start_d_ = p.start_d;
    status_ = PlanStatus::Infeasible;
}

// f(theta) = L1(theta) + R*theta + L3 - GR*H_total  (horizontal arc-length budget)
PlanStatus PathGuidance::solveFrom(Vec2 start, float start_d) {
    const float R    = p_.loiter_radius;
    const float dir  = static_cast<float>(p_.loiter_dir);
    const float Htot = p_.land_d - start_d;
    psi4_ = p_.land_heading;
    start_ = start; start_d_ = start_d;

    if (Htot <= 0.f || gr_ <= 0.f || R <= 0.f) return (status_ = PlanStatus::Infeasible);

    const float budget = gr_ * Htot;
    const float L3min  = (p_.min_turn_radius > 1.f) ? p_.min_turn_radius : 1.f;
    const float L3max  = p_.approach_len * 2.5f;

    // exit anchor: loiter circle tangent to the approach line at pt3 (heading psi4)
    auto exitGeom = [&](float L3, Vec2& pt3, Vec2& C, float& a3) {
        const Vec2 d4 = dirOf(psi4_);
        pt3 = { p_.land_n - L3 * d4.n, p_.land_e - L3 * d4.e };
        const Vec2 pl = perpL(psi4_);
        C  = { pt3.n + R * dir * pl.n, pt3.e + R * dir * pl.e };
        a3 = atan2f(-dir * cosf(psi4_), dir * sinf(psi4_));
    };
    // TANGENT entry: external tangent from `start` to circle C. Returns the
    // entry angle a2, tangent point pt2, leg length L1 and heading psi1 so that
    // the homing line is tangent to the loiter (G1, no heading jump). The two
    // tangents correspond to the two turn senses; pick the one whose tangent
    // heading matches the loiter direction `dir`.
    auto solveTangent = [&](const Vec2& C, float& a2o, Vec2& pt2o, float& L1o, float& psi1o)->bool {
        const float dn = start.n - C.n, de = start.e - C.e;
        const float dpc = hypotf(dn, de);
        if (dpc <= R + 0.05f) return false;                 // start inside circle: no tangent
        const float gamma = atan2f(de, dn);                 // bearing C -> start
        const float dang  = acosf(R / dpc);
        float bestErr = 1e30f;
        for (int sgn = -1; sgn <= 1; sgn += 2) {
            const float a2 = gamma + sgn * dang;
            const Vec2  p2 { C.n + R * cosf(a2), C.e + R * sinf(a2) };
            const float psi_line = atan2f(p2.e - start.e, p2.n - start.n);
            const float htan = atan2f(dir * cosf(a2), -dir * sinf(a2));
            const float err = fabsf(wrapPi(psi_line - htan));
            if (err < bestErr) {
                bestErr = err; a2o = a2; pt2o = p2;
                L1o = hypotf(p2.n - start.n, p2.e - start.e); psi1o = htan;
            }
        }
        return bestErr < 0.05f;                              // tangency achieved
    };

    // closure: theta = partial(a2->a3) + 2*pi*k ; choose k + L3 to close the
    // budget L1 + R*theta + L3 = budget. The centre moves with L3, so iterate.
    float L3 = p_.approach_len;
    PlanStatus st = PlanStatus::Ok;
    Vec2  pt3, C; float a3, a2, pt2L1psi_a2; (void)pt2L1psi_a2;
    Vec2  pt2; float L1, psi1, sweep = 0.f;

    bool feas = false;
    for (int it = 0; it < 8; ++it) {
        exitGeom(L3, pt3, C, a3);
        if (!solveTangent(C, a2, pt2, L1, psi1)) {
            // try shrinking the approach to move the circle into reach
            bool ok = false;
            for (float cand = L3 - 4.f; cand >= L3min; cand -= 4.f) {
                exitGeom(cand, pt3, C, a3);
                if (solveTangent(C, a2, pt2, L1, psi1)) { L3 = cand; ok = true; st = PlanStatus::AdjustedApproach; break; }
            }
            if (!ok) return (status_ = PlanStatus::Infeasible);
        }
        const float partial = (dir > 0.f) ? wrap2pi(a3 - a2) : wrap2pi(a2 - a3);
        const float k_nom = ((budget - L1 - L3) / R - partial) / kTwoPi;
        int   k0 = (int)floorf(k_nom);
        float bestAbs = 1e30f, bestL3 = L3, bestSweep = partial;
        for (int kk = k0 - 1; kk <= k0 + 2; ++kk) {
            if (kk < 0) continue;
            const float sw = partial + kTwoPi * (float)kk;
            float l3 = budget - L1 - R * sw;
            if (l3 < L3min) l3 = L3min;
            if (l3 > L3max) l3 = L3max;
            const float resid = budget - (L1 + R * sw + l3);
            if (fabsf(resid) < bestAbs) { bestAbs = fabsf(resid); bestL3 = l3; bestSweep = sw; }
        }
        sweep = bestSweep; feas = true;
        if (fabsf(bestL3 - L3) > 0.5f && st == PlanStatus::Ok) st = PlanStatus::AdjustedApproach;
        if (fabsf(bestL3 - L3) < 0.1f) { L3 = bestL3; break; }
        L3 = bestL3;
    }
    if (!feas) return (status_ = PlanStatus::Infeasible);

    exitGeom(L3, pt3, C, a3);
    solveTangent(C, a2, pt2, L1, psi1);
    c_ = C; pt3_ = pt3; a3_ = a3; pt2_ = pt2; psi1_ = psi1;
    theta_ = sweep; resolved_L3_ = L3;
    d_pt3_ = p_.land_d - L3 / gr_;
    d_pt2_ = d_pt3_ - (R * theta_) / gr_;
    residual_ = (budget - (L1 + R * theta_ + L3)) / gr_;
    return (status_ = st);
}

PlanStatus PathGuidance::plan() {
    gr_ = p_.glide_ratio;
    return solveFrom({ p_.start_n, p_.start_e }, p_.start_d);
}

PlanStatus PathGuidance::replan(const State& s) {
    // adopt the measured glide ratio so the budget closes against reality
    const float vh = hypotf(s.vn, s.ve);
    if (s.vd > 0.3f && vh > 0.5f) {
        float gm = vh / s.vd;
        if (gm < 1.0f) gm = 1.0f; if (gm > 8.0f) gm = 8.0f;
        gr_ = gm;
    }

    // snapshot for transactional rollback if the rebuild fails
    const Vec2 sStart = start_, sPt2 = pt2_, sPt3 = pt3_, sC = c_;
    const float sTheta = theta_, sPsi1 = psi1_, sA3 = a3_;
    const float sD2 = d_pt2_, sD3 = d_pt3_, sL3 = resolved_L3_, sZ0 = start_d_, sRes = residual_;
    const PlanStatus sSt = status_;

    Phase ph = (s.d < d_pt2_) ? Phase::Homing : (s.d < d_pt3_) ? Phase::Loiter : Phase::Approach;
    PlanStatus r;

    if (ph == Phase::Loiter) {
        // keep the anchored circle/exit; recompute the remaining sweep from the
        // current angle so we still roll out at pt3 into wind. Exit angle a3_ is
        // fixed -> remaining sweep is quantised; pick loops to best close budget.
        const float R = p_.loiter_radius, dir = (float)p_.loiter_dir;
        const float budget = gr_ * (p_.land_d - s.d);
        const float L3min = p_.min_turn_radius, L3max = p_.approach_len * 2.5f;
        const float a_now = atan2f(s.e - c_.e, s.n - c_.n);
        const float partial = (dir > 0.f) ? wrap2pi(a3_ - a_now) : wrap2pi(a_now - a3_);
        float L3 = resolved_L3_;
        const float k_nom = ((budget - L3) / R - partial) / kTwoPi;
        int k0 = (int)floorf(k_nom);
        float bestAbs = 1e30f, bestL3 = L3, bestSweep = partial;
        for (int kk = k0 - 1; kk <= k0 + 2; ++kk) {
            if (kk < 0) continue;
            const float sw = partial + kTwoPi * (float)kk;
            float l3 = budget - R * sw;
            if (l3 < L3min) l3 = L3min; if (l3 > L3max) l3 = L3max;
            const float resid = budget - (R * sw + l3);
            if (fabsf(resid) < bestAbs) { bestAbs = fabsf(resid); bestL3 = l3; bestSweep = sw; }
        }
        theta_ = bestSweep; resolved_L3_ = bestL3;
        pt3_   = { p_.land_n - bestL3 * dirOf(psi4_).n, p_.land_e - bestL3 * dirOf(psi4_).e };
        start_ = { s.n, s.e }; start_d_ = s.d;
        d_pt2_ = s.d;                                  // remaining loiter starts here
        d_pt3_ = d_pt2_ + (R * theta_) / gr_;
        // entry of the remaining loiter is the current position; keep a3_ so that
        // a2 = a3_ - dir*theta_ == a_now (consistent with pathAt's loiter eval)
        pt2_ = { c_.n + R * cosf(a3_ - dir * theta_), c_.e + R * sinf(a3_ - dir * theta_) };
        residual_ = (budget - (R * theta_ + bestL3)) / gr_;
        r = (status_ = PlanStatus::Ok);
    } else if (ph == Phase::Approach) {
        // keep flying the into-wind approach line; just re-anchor its start to the
        // current altitude band (cross-track is handled by pure pursuit)
        start_d_ = s.d; gr_ = gr_;
        r = (status_ = (status_ == PlanStatus::Ok || status_ == PlanStatus::AdjustedApproach)
                       ? status_ : PlanStatus::Ok);
    } else {
        r = solveFrom({ s.n, s.e }, s.d);              // Homing: full re-derive from here
    }

    if (!(r == PlanStatus::Ok || r == PlanStatus::AdjustedApproach)) {
        start_ = sStart; pt2_ = sPt2; pt3_ = sPt3; c_ = sC;
        theta_ = sTheta; psi1_ = sPsi1; a3_ = sA3;
        d_pt2_ = sD2; d_pt3_ = sD3; resolved_L3_ = sL3; start_d_ = sZ0; residual_ = sRes;
        status_ = sSt;
    }
    return r;
}

Vec2 PathGuidance::pathAt(float d) const {
    const float R   = p_.loiter_radius;
    const float dir = static_cast<float>(p_.loiter_dir);
    if (d <= d_pt2_) {                                 // homing
        float s = gr_ * (d - start_d_); if (s < 0.f) s = 0.f;
        const Vec2 u = dirOf(psi1_);
        return { start_.n + s * u.n, start_.e + s * u.e };
    }
    if (d <= d_pt3_) {                                 // loiter
        const float sweep = (R > 0.f) ? (d - d_pt2_) * gr_ / R : 0.f;
        const float a = (a3_ - dir * theta_) + dir * sweep;
        return { c_.n + R * cosf(a), c_.e + R * sinf(a) };
    }
    float s = gr_ * (d - d_pt3_);                      // approach
    if (s < 0.f) s = 0.f; if (s > resolved_L3_) s = resolved_L3_;
    const Vec2 u = dirOf(psi4_);
    return { pt3_.n + s * u.n, pt3_.e + s * u.e };
}

float PathGuidance::segS0(int i) const {
    const float L1 = gr_ * (d_pt2_ - start_d_);
    const float Lo = p_.loiter_radius * theta_;
    if (i <= 0) return 0.f;
    if (i == 1) return L1;
    return L1 + Lo;
}
float PathGuidance::segLen(int i) const {
    const float L1 = gr_ * (d_pt2_ - start_d_);
    const float Lo = p_.loiter_radius * theta_;
    if (i == 0) return L1;
    if (i == 1) return Lo;
    return resolved_L3_;
}

Vec2 PathGuidance::evalS(float s, float* psi, float* kappa) const {
    const float R = p_.loiter_radius, dir = (float)p_.loiter_dir;
    const float L1 = gr_ * (d_pt2_ - start_d_);
    const float Lo = R * theta_;
    if (s < 0.f) s = 0.f;
    const float tot = L1 + Lo + resolved_L3_;
    if (s > tot) s = tot;
    if (s <= L1) {
        if (psi) *psi = psi1_; if (kappa) *kappa = 0.f;
        const Vec2 u = dirOf(psi1_);
        return { start_.n + s * u.n, start_.e + s * u.e };
    }
    if (s <= L1 + Lo) {
        const float a = (a3_ - dir * theta_) + dir * (s - L1) / R;
        if (psi) *psi = atan2f(dir * cosf(a), -dir * sinf(a));
        if (kappa) *kappa = dir / R;
        return { c_.n + R * cosf(a), c_.e + R * sinf(a) };
    }
    if (psi) *psi = psi4_; if (kappa) *kappa = 0.f;
    const Vec2 u = dirOf(psi4_);
    const float ls = s - (L1 + Lo);
    return { pt3_.n + ls * u.n, pt3_.e + ls * u.e };
}

float PathGuidance::curvatureAt(float s) const { float k; evalS(s, nullptr, &k); return k; }

HeadingCmd PathGuidance::getHeading(const State& s) const {
    HeadingCmd cmd;
    cmd.valid = (status_ == PlanStatus::Ok || status_ == PlanStatus::AdjustedApproach);
    if      (s.d <  d_pt2_)    cmd.phase = Phase::Homing;
    else if (s.d <  d_pt3_)    cmd.phase = Phase::Loiter;
    else if (s.d <  p_.land_d) cmd.phase = Phase::Approach;
    else                       cmd.phase = Phase::Landed;

    float d_car = s.d + p_.lookahead_drop;
    if (d_car > p_.land_d) d_car = p_.land_d;
    const Vec2 carrot = (cmd.phase == Phase::Landed) ? land_ : pathAt(d_car);
    cmd.carrot = carrot;
    cmd.heading = atan2f(carrot.e - s.e, carrot.n - s.n);

    const float vh = hypotf(s.vn, s.ve);
    cmd.glide_angle = atan2f(s.vd, vh);
    (void)wrapPi;
    return cmd;
}

} // namespace rsx
