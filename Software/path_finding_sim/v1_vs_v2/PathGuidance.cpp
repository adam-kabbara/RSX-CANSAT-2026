//
// PathGuidance.cpp  (v2)  --  RSX CanSat 2026 "Autonomous Paraglider"
//
#include "PathGuidance.hpp"
#include <cmath>

namespace rsx {

static constexpr float kPi    = 3.14159265358979f;
static constexpr float kTwoPi = 6.28318530717959f;

static inline Vec2  dirOf(float psi) { return { cosf(psi), sinf(psi) }; }
static inline float wrapPi(float a) {
    a = fmodf(a + kPi, kTwoPi);
    if (a < 0.f) a += kTwoPi;
    return a - kPi;
}
static inline float wrap2pi(float a) {           // -> [0, 2pi)
    a = fmodf(a, kTwoPi);
    if (a < 0.f) a += kTwoPi;
    return a;
}

// Integrate a clothoid/line/arc element from local s=0 to s=len.
// psi(t) = psi0 + kappa0*t + 0.5*dkappa*t^2 ; returns end pos + end heading.
// Simpson's rule, fixed steps -> deterministic, allocation-free.
static void integrateElem(float n0, float e0, float psi0, float kappa0,
                          float dkappa, float len,
                          float& n_out, float& e_out, float& psi_out) {
    const int N = 32;                       // even
    const float h = len / N;
    float sn = 0.f, se = 0.f;               // accumulated displacement integrals
    auto psiAt = [&](float t){ return psi0 + kappa0 * t + 0.5f * dkappa * t * t; };
    // Simpson over cos(psi) and sin(psi)
    float fcn0 = cosf(psiAt(0.f)), fse0 = sinf(psiAt(0.f));
    float accN = fcn0, accE = fse0;
    float endcn = cosf(psiAt(len)), endse = sinf(psiAt(len));
    for (int i = 1; i < N; ++i) {
        const float t = i * h;
        const float w = (i & 1) ? 4.f : 2.f;
        accN += w * cosf(psiAt(t));
        accE += w * sinf(psiAt(t));
    }
    accN += endcn; accE += endse;
    sn = accN * h / 3.f;
    se = accE * h / 3.f;
    n_out = n0 + sn;
    e_out = e0 + se;
    psi_out = psiAt(len);
}

void PathGuidance::setParams(const GuidanceParams& p) {
    p_ = p;
    gr_ = p.glide_ratio;
    psi4_ = p.land_heading;
    loiter_R_ = p.loiter_radius;
    status_ = PlanStatus::Infeasible;
}

void PathGuidance::pushSeg(SegType t, float len, float n0, float e0, float psi0,
                           float kappa0, float dkappa) {
    if (n_seg_ >= kMaxSeg) return;
    Segment& s = seg_[n_seg_++];
    s.type = t; s.len = len; s.n0 = n0; s.e0 = e0; s.psi0 = psi0;
    s.kappa0 = kappa0; s.dkappa = dkappa; s.s0 = 0.f;
}

void PathGuidance::rebuildArcLengths() {
    float acc = 0.f;
    for (int i = 0; i < n_seg_; ++i) { seg_[i].s0 = acc; acc += seg_[i].len; }
    total_len_ = acc;
}

// Solve the entry transition: from (start, psi1) reach the anchored loiter
// circle (c_, loiter_R_) tangentially via a clothoid of length Lc preceded by
// a straight homing leg of length L1. Bisect on Lc for tangency; L1 from the
// circle-intersection quadratic. Returns false if no tangent entry exists.
// Transition-curve (line->clothoid->circle) entry. The homing ray along psi1
// is the tangent; a clothoid of length Lc eases curvature 0 -> dir/R and joins
// the loiter circle tangentially. The standard "shift" geometry: the circle's
// centre sits at perpendicular offset perp(Lc) from the tangent line, where
// perp(Lc) is MONOTONIC -- so a single clean bisection finds Lc. The clothoid
// begins (TS point) at along-ray distance L1 = f - k, where f is the foot of
// the perpendicular from C and k is the clothoid's along-tangent abscissa.
// Infeasible if the ray cuts the circle (perp <= R), is on the wrong side, or
// the contact lies behind the start (L1 < 0).
bool PathGuidance::solveEntry(Vec2 start, float psi1, float& L1_out,
                              float& Lc_out, float& a_in_out) const {
    const float R   = loiter_R_;
    const float dir = static_cast<float>(p_.loiter_dir);
    const Vec2  u   = dirOf(psi1);
    const Vec2  rn  { -sinf(psi1), cosf(psi1) };       // right normal of heading
    const float wn  = c_.n - start.n, we = c_.e - start.e;
    const float perp_t = dir * (wn * rn.n + we * rn.e); // + when C on the turn side
    const float f      = wn * u.n + we * u.e;           // along-ray foot distance
    if (perp_t <= R) return false;                       // ray cuts circle / wrong side

    // canonical clothoid offsets: integrate from origin, heading 0, kappa 0->dir/R
    auto offs = [&](float Lc, float& k, float& perp,
                    float& xn, float& xe, float& th) {
        const float dk = dir / (R * Lc);
        integrateElem(0.f, 0.f, 0.f, 0.f, dk, Lc, xn, xe, th);
        const float On = xn + R * dir * (-sinf(th));
        const float Oe = xe + R * dir * ( cosf(th));
        k = On; perp = dir * Oe;
    };

    float lo = 0.05f, hi = 4000.f, k, perp, xn, xe, th;
    offs(hi, k, perp, xn, xe, th);
    if (perp < perp_t) return false;                    // unreachable (degenerate)
    for (int i = 0; i < 50; ++i) {
        const float mid = 0.5f * (lo + hi);
        offs(mid, k, perp, xn, xe, th);
        if (perp < perp_t) lo = mid; else hi = mid;
    }
    const float Lc = 0.5f * (lo + hi);
    offs(Lc, k, perp, xn, xe, th);
    const float L1 = f - k;
    if (L1 < 0.f) return false;                          // contact behind start

    const float c = cosf(psi1), s = sinf(psi1);          // rotate clothoid end into world
    const float dn = c * xn - s * xe, de = s * xn + c * xe;
    const Vec2  SC { start.n + L1 * u.n + dn, start.e + L1 * u.e + de };
    a_in_out = atan2f(SC.e - c_.e, SC.n - c_.n);
    L1_out = L1; Lc_out = Lc;
    return true;
}

PlanStatus PathGuidance::buildFromStart(Vec2 start, float psi1, float z0) {
    n_seg_ = 0;
    z0_ = z0;
    const float R    = loiter_R_;
    const float GR   = gr_;
    const float dir  = static_cast<float>(p_.loiter_dir);
    const float Lcx  = p_.transition_len;            // exit clothoid length (design)
    const Vec2  land { p_.land_n, p_.land_e };
    psi4_ = p_.land_heading;

    if (R <= 0.f || GR <= 0.f) return (status_ = PlanStatus::Infeasible);

    // ---- exit side, anchored to the landing/approach line ----
    // approach line start (for L3 length): A0 = land - L3 * dir(psi4)
    // exit clothoid (kappa dir/R -> 0) ends at A0 heading psi4; integrate it
    // forward from origin to get its displacement, then place X_exit.
    auto buildExit = [&](float L3, Vec2& A0, Vec2& Xexit, float& a_exit, Vec2& C) {
        const Vec2 d4 = dirOf(psi4_);
        A0 = { land.n - L3 * d4.n, land.e - L3 * d4.e };
        const float psi_xs = wrapPi(psi4_ - dir * Lcx / (2.f * R)); // heading at X_exit
        const float dkx = -dir / (R * Lcx);
        float dn, de, pend;
        integrateElem(0.f, 0.f, psi_xs, dir / R, dkx, Lcx, dn, de, pend); // disp of exit clothoid
        Xexit = { A0.n - dn, A0.e - de };
        // centre: from X_exit, turn-centre side at distance R (dir +1 -> right)
        C = { Xexit.n + R * dir * (-sinf(psi_xs)), Xexit.e + R * dir * (cosf(psi_xs)) };
        a_exit = atan2f(Xexit.e - C.e, Xexit.n - C.n);
    };

    // iterate L3 <-> integer-loop closure (centre shifts with L3, mild coupling).
    // Strategy: solve entry at nominal L3 first; if that is infeasible the homing
    // heading genuinely cannot reach the loiter -> flag. Then try to absorb the
    // budget residual by flexing L3, but only ACCEPT a flex that keeps the entry
    // feasible -- never let closure turn a good plan infeasible.
    float L3 = p_.approach_len;
    PlanStatus st = PlanStatus::Ok;
    float L1 = 0.f, Lc = Lcx, a_in = 0.f;
    Vec2 A0, Xexit, C; float a_exit;

    const float budget  = GR * (p_.land_d - z0);     // total horizontal arc available
    const float L3min   = p_.approach_len_min;
    const float L3max   = p_.approach_len * 2.5f;

    // closeAt: given a trial L3, build exit geometry + entry, then pick the
    // integer loop count k so the *required* L3 lands in [L3min,L3max] as close
    // to nominal as possible -- this closes the altitude budget exactly when a
    // feasible k exists. Returns the suggested L3 for the next iterate.
    auto closeAt = [&](float L3try, float& sweep, float& L3req, float& resid_arc,
                       float& L1o, float& Lco, float& a_ino,
                       Vec2& Co, Vec2& Xo, float& aexo)->bool {
        buildExit(L3try, A0, Xexit, a_exit, C);
        c_ = C; x_exit_ = Xexit; a_exit_ = a_exit;     // solveEntry reads c_
        if (!solveEntry(start, psi1, L1o, Lco, a_ino)) return false;
        const float partial = (dir > 0.f) ? wrap2pi(a_exit - a_ino)
                                          : wrap2pi(a_ino - a_exit);
        const float base = L1o + Lco + Lcx;            // length independent of L3 & loops
        // choose loops k minimising the closure residual (L3 flex is bounded, so
        // when one loop's required L3 is out of range we keep the better-residual k)
        const float k_nom = ((budget - base - L3try) / R - partial) / kTwoPi;
        int   k0 = (int)floorf(k_nom);
        int   kbest = 0; float bestAbs = 1e30f, bestL3 = L3try, bestSweep = partial;
        for (int kk = k0 - 1; kk <= k0 + 2; ++kk) {
            if (kk < 0) continue;
            const float sw = partial + kTwoPi * (float)kk;
            float l3 = budget - base - R * sw;
            if (l3 < L3min) l3 = L3min;
            if (l3 > L3max) l3 = L3max;
            const float resid = budget - (base + R * sw + l3);
            if (fabsf(resid) < bestAbs) {
                bestAbs = fabsf(resid); kbest = kk; bestL3 = l3; bestSweep = sw;
            }
        }
        (void)kbest;
        sweep  = bestSweep;
        L3req  = bestL3;
        resid_arc = budget - (base + R * sweep + L3req);
        Co = C; Xo = Xexit; aexo = a_exit;
        return true;
    };

    float sweep, L3req, resid_arc;
    Vec2 bC, bX; float baex;
    if (!closeAt(L3, sweep, L3req, resid_arc, L1, Lc, a_in, bC, bX, baex))
        return (status_ = PlanStatus::EntryInfeasible);
    L3 = L3req;

    // iterate to a fixed point (entry depends weakly on L3 via the centre shift);
    // keep only feasible iterates.
    for (int it = 0; it < 6; ++it) {
        float s2, l3r2, r2, l12, lc2, ai2; Vec2 C2, X2; float ae2;
        if (!closeAt(L3, s2, l3r2, r2, l12, lc2, ai2, C2, X2, ae2)) break;
        sweep = s2; resid_arc = r2; L1 = l12; Lc = lc2; a_in = ai2;
        bC = C2; bX = X2; baex = ae2;
        if (fabsf(l3r2 - L3) < 0.1f) { L3 = l3r2; break; }
        L3 = l3r2;
    }
    if (fabsf(L3 - p_.approach_len) > 0.5f) st = PlanStatus::AdjustedApproach;

    // restore the chosen feasible geometry
    c_ = bC; x_exit_ = bX; a_exit_ = baex;
    // reject only the converged solution if the entry spiral is impractically long
    // (homing ray far off-tangent) -- avoids disrupting the L3-flex iteration
    if (p_.entry_clothoid_max > 0.f && Lc > p_.entry_clothoid_max)
        return (status_ = PlanStatus::EntryInfeasible);
    loiter_sweep_ = sweep;
    resolved_L3_ = L3;
    residual_ = (budget - (L1 + Lc + Lcx + L3 + R * loiter_sweep_)) / GR;
    if (fabsf(residual_) > 0.5f && st == PlanStatus::Ok)
        st = PlanStatus::AdjustedApproach;

    // ---- emit segments ----
    const float dk_in = dir / (R * Lc);
    // 1) homing line
    pushSeg(SegType::Line, L1, start.n, start.e, psi1, 0.f, 0.f);
    // 2) entry clothoid (kappa 0 -> dir/R)
    {
        const Vec2 e0 { start.n + L1 * cosf(psi1), start.e + L1 * sinf(psi1) };
        pushSeg(SegType::Clothoid, Lc, e0.n, e0.e, psi1, 0.f, dk_in);
    }
    // 3) loiter arc (kappa dir/R)
    {
        float n1, e1, p1;
        integrateElem(start.n + L1 * cosf(psi1), start.e + L1 * sinf(psi1),
                      psi1, 0.f, dk_in, Lc, n1, e1, p1);
        pushSeg(SegType::Arc, R * loiter_sweep_, n1, e1, p1, dir / R, 0.f);
    }
    // 4) exit clothoid (kappa dir/R -> 0): starts at X_exit
    {
        const float psi_xs = wrapPi(psi4_ - dir * Lcx / (2.f * R));
        pushSeg(SegType::Clothoid, Lcx, x_exit_.n, x_exit_.e, psi_xs, dir / R, -dir / (R * Lcx));
    }
    // 5) approach line to landing
    {
        Vec2 A0 { land.n - L3 * cosf(psi4_), land.e - L3 * sinf(psi4_) };
        pushSeg(SegType::Line, L3, A0.n, A0.e, psi4_, 0.f, 0.f);
    }
    rebuildArcLengths();
    status_ = st;
    return status_;
}

// Replan while inside the loiter: keep the anchored circle/exit/approach,
// recompute remaining sweep from the current angle so we still exit on the
// approach line at the right altitude.
PlanStatus PathGuidance::buildLoiterTail(Vec2 cur, float z0) {
    n_seg_ = 0;
    z0_ = z0;
    const float R   = loiter_R_;
    const float GR  = gr_;
    const float dir = static_cast<float>(p_.loiter_dir);
    const float Lcx = p_.transition_len;
    const Vec2  land { p_.land_n, p_.land_e };

    const float budget = GR * (p_.land_d - z0);
    const float L3min  = p_.approach_len_min;
    const float L3max  = p_.approach_len * 2.5f;

    // current angle on the circle (snap radius)
    float a_now = atan2f(cur.e - c_.e, cur.n - c_.n);
    float partial = (dir > 0.f) ? wrap2pi(a_exit_ - a_now) : wrap2pi(a_now - a_exit_);

    // pick loops k minimising the closure residual
    float L3 = resolved_L3_;
    const float k_nom = ((budget - Lcx - L3) / R - partial) / kTwoPi;
    int   k0 = (int)floorf(k_nom);
    float bestAbs = 1e30f, bestL3 = L3, bestSweep = partial;
    for (int kk = k0 - 1; kk <= k0 + 2; ++kk) {
        if (kk < 0) continue;
        const float sw = partial + kTwoPi * (float)kk;
        float l3 = budget - Lcx - R * sw;
        if (l3 < L3min) l3 = L3min;
        if (l3 > L3max) l3 = L3max;
        const float resid = budget - (Lcx + R * sw + l3);
        if (fabsf(resid) < bestAbs) { bestAbs = fabsf(resid); bestL3 = l3; bestSweep = sw; }
    }
    loiter_sweep_ = bestSweep;
    L3 = bestL3;
    if (loiter_sweep_ <= 0.f && (budget - Lcx - L3) < 0.f)
        return (status_ = PlanStatus::Infeasible);
    resolved_L3_ = L3;
    residual_ = (budget - (Lcx + L3 + R * loiter_sweep_)) / GR;

    // remaining arc starts at current angle, heading = circle tangent
    const float head0 = atan2f(dir * cosf(a_now), -dir * sinf(a_now));
    const Vec2  start_pos { c_.n + R * cosf(a_now), c_.e + R * sinf(a_now) };
    pushSeg(SegType::Arc, R * loiter_sweep_, start_pos.n, start_pos.e, head0, dir / R, 0.f);
    {
        const float psi_xs = wrapPi(p_.land_heading - dir * Lcx / (2.f * R));
        pushSeg(SegType::Clothoid, Lcx, x_exit_.n, x_exit_.e, psi_xs, dir / R, -dir / (R * Lcx));
    }
    {
        Vec2 A0 { land.n - L3 * cosf(p_.land_heading), land.e - L3 * sinf(p_.land_heading) };
        pushSeg(SegType::Line, L3, A0.n, A0.e, p_.land_heading, 0.f, 0.f);
    }
    rebuildArcLengths();
    status_ = PlanStatus::Ok;
    return status_;
}

PlanStatus PathGuidance::plan() {
    if (p_.legacy_v1)
        return buildV1FromStart({ p_.start_n, p_.start_e }, p_.start_d);
    return buildFromStart({ p_.start_n, p_.start_e }, p_.start_heading, p_.start_d);
}

// ----- v1 legacy: line -> arc -> line, derived homing heading ---------------
// Loiter anchored to the approach line (tangent exit at psi4, no exit clothoid).
// The homing leg aims straight at the loiter entry pt2, found by walking back
// around the circle by the continuous sweep theta; theta closes the budget
// exactly (L1(theta) + R*theta + L3 = budget) -- no loop quantum.
PlanStatus PathGuidance::buildV1FromStart(Vec2 start, float z0) {
    n_seg_ = 0; z0_ = z0;
    const float R   = loiter_R_;
    const float GR  = gr_;
    const float dir = static_cast<float>(p_.loiter_dir);
    const float psi4 = p_.land_heading;
    const Vec2  land { p_.land_n, p_.land_e };
    if (R <= 0.f || GR <= 0.f) return (status_ = PlanStatus::Infeasible);
    const float budget = GR * (p_.land_d - z0);

    auto exitGeom = [&](float L3, Vec2& pt3, Vec2& C, float& a_ex) {
        const Vec2 d4 = dirOf(psi4);
        pt3 = { land.n - L3 * d4.n, land.e - L3 * d4.e };
        C   = { pt3.n + R * dir * (-sinf(psi4)), pt3.e + R * dir * (cosf(psi4)) };
        a_ex = atan2f(pt3.e - C.e, pt3.n - C.n);
    };
    auto solveTheta = [&](float L3, float& theta)->bool {
        Vec2 pt3, C; float a_ex; exitGeom(L3, pt3, C, a_ex);
        auto f = [&](float t) {
            const float a2 = a_ex - dir * t;
            const Vec2 p2 { C.n + R * cosf(a2), C.e + R * sinf(a2) };
            return hypotf(p2.n - start.n, p2.e - start.e) + R * t + L3 - budget;
        };
        if (f(0.f) > 0.f) return false;              // too long even with no loiter
        float hi = 0.5f;
        while (f(hi) < 0.f && hi < 628.f) hi *= 1.6f;
        float lo = 0.f;
        for (int i = 0; i < 60; ++i) {
            const float mid = 0.5f * (lo + hi);
            if (f(mid) < 0.f) lo = mid; else hi = mid;
        }
        theta = 0.5f * (lo + hi);
        return true;
    };

    float L3 = p_.approach_len, theta = 0.f;
    PlanStatus st = PlanStatus::Ok;
    if (!solveTheta(L3, theta)) {
        bool ok = false;
        for (float cand = L3 - 4.f; cand >= p_.approach_len_min; cand -= 4.f) {
            if (solveTheta(cand, theta)) { L3 = cand; ok = true; st = PlanStatus::AdjustedApproach; break; }
        }
        if (!ok) { L3 = p_.approach_len_min; theta = 0.f; st = PlanStatus::AdjustedApproach; }
    }

    Vec2 pt3, C; float a_ex; exitGeom(L3, pt3, C, a_ex);
    c_ = C; x_exit_ = pt3; a_exit_ = a_ex;
    const float a2 = a_ex - dir * theta;
    const Vec2  pt2 { C.n + R * cosf(a2), C.e + R * sinf(a2) };
    const float psi1 = atan2f(pt2.e - start.e, pt2.n - start.n);   // DERIVED heading
    const float L1   = hypotf(pt2.n - start.n, pt2.e - start.e);
    loiter_sweep_ = theta; resolved_L3_ = L3;
    residual_ = (budget - (L1 + R * theta + L3)) / GR;

    pushSeg(SegType::Line, L1, start.n, start.e, psi1, 0.f, 0.f);
    const float head2 = atan2f(dir * cosf(a2), -dir * sinf(a2));   // circle tangent at pt2
    pushSeg(SegType::Arc, R * theta, pt2.n, pt2.e, head2, dir / R, 0.f);
    pushSeg(SegType::Line, L3, pt3.n, pt3.e, psi4, 0.f, 0.f);
    rebuildArcLengths();
    return (status_ = st);
}

PlanStatus PathGuidance::buildV1LoiterTail(Vec2 cur, float z0) {
    n_seg_ = 0; z0_ = z0;
    const float R   = loiter_R_;
    const float GR  = gr_;
    const float dir = static_cast<float>(p_.loiter_dir);
    const float psi4 = p_.land_heading;
    const Vec2  land { p_.land_n, p_.land_e };
    const float budget = GR * (p_.land_d - z0);
    const float L3min = p_.approach_len_min, L3max = p_.approach_len * 2.5f;

    float a_now = atan2f(cur.e - c_.e, cur.n - c_.n);
    float partial = (dir > 0.f) ? wrap2pi(a_exit_ - a_now) : wrap2pi(a_now - a_exit_);
    float L3 = resolved_L3_;
    const float k_nom = ((budget - L3) / R - partial) / kTwoPi;
    int   k0 = (int)floorf(k_nom);
    float bestAbs = 1e30f, bestL3 = L3, bestSweep = partial;
    for (int kk = k0 - 1; kk <= k0 + 2; ++kk) {
        if (kk < 0) continue;
        const float sw = partial + kTwoPi * (float)kk;
        float l3 = budget - R * sw;
        if (l3 < L3min) l3 = L3min;
        if (l3 > L3max) l3 = L3max;
        const float resid = budget - (R * sw + l3);
        if (fabsf(resid) < bestAbs) { bestAbs = fabsf(resid); bestL3 = l3; bestSweep = sw; }
    }
    loiter_sweep_ = bestSweep; L3 = bestL3; resolved_L3_ = L3;
    residual_ = (budget - (L3 + R * loiter_sweep_)) / GR;

    const float head0 = atan2f(dir * cosf(a_now), -dir * sinf(a_now));
    const Vec2  sp { c_.n + R * cosf(a_now), c_.e + R * sinf(a_now) };
    pushSeg(SegType::Arc, R * loiter_sweep_, sp.n, sp.e, head0, dir / R, 0.f);
    Vec2 pt3 { land.n - L3 * cosf(psi4), land.e - L3 * sinf(psi4) };
    pushSeg(SegType::Line, L3, pt3.n, pt3.e, psi4, 0.f, 0.f);
    rebuildArcLengths();
    return (status_ = PlanStatus::Ok);
}

PlanStatus PathGuidance::replan(const State& s) {
    const Vec2 cur { s.n, s.e };
    // adopt the measured glide ratio so the energy budget closes against ACTUAL
    // performance (banked turns / wind make real sink differ from nominal)
    const float vh = hypotf(s.vn, s.ve);
    if (s.vd > 0.3f && vh > 0.5f) {
        float gm = vh / s.vd;
        if (gm < 1.0f) gm = 1.0f;
        if (gm > 8.0f) gm = 8.0f;
        gr_ = gm;
    }
    const float s_now = arcFromAlt(s.d);
    Phase ph = Phase::Homing;
    if (n_seg_ > 0) {
        int idx = 0;
        for (int i = 0; i < n_seg_; ++i)
            if (s_now >= seg_[i].s0) idx = i;
        const SegType t = seg_[idx].type;
        if (idx >= n_seg_ - 1)       ph = Phase::Approach;
        else if (t == SegType::Arc || (t == SegType::Clothoid && idx >= 1)) ph = Phase::Loiter;
        else                         ph = Phase::Homing;
    }
    const float cur_heading = (fabsf(s.vn) + fabsf(s.ve) > 1e-3f)
                              ? atan2f(s.ve, s.vn) : s.yaw;

    // --- snapshot so a failed rebuild is a no-op (keep flying the valid plan) ---
    Segment   snapSeg[kMaxSeg];
    for (int i = 0; i < n_seg_; ++i) snapSeg[i] = seg_[i];
    const int        snap_n   = n_seg_;
    const Vec2       snap_c   = c_, snap_x = x_exit_;
    const float      snap_ae  = a_exit_, snap_sw = loiter_sweep_, snap_L3 = resolved_L3_;
    const float      snap_res = residual_, snap_z0 = z0_, snap_len = total_len_, snap_gr = gr_;
    const PlanStatus snap_st  = status_;

    PlanStatus r;
    if (ph == Phase::Loiter) {
        r = p_.legacy_v1 ? buildV1LoiterTail(cur, s.d) : buildLoiterTail(cur, s.d);
    } else if (ph == Phase::Approach) {
        n_seg_ = 0; z0_ = s.d;
        const float L   = hypotf(p_.land_n - s.n, p_.land_e - s.e);
        const float psi = atan2f(p_.land_e - s.e, p_.land_n - s.n);
        pushSeg(SegType::Line, L, s.n, s.e, psi, 0.f, 0.f);
        rebuildArcLengths();
        r = (status_ = PlanStatus::Ok);
    } else {
        r = p_.legacy_v1 ? buildV1FromStart(cur, s.d)
                         : buildFromStart(cur, cur_heading, s.d);
    }

    const bool ok = (r == PlanStatus::Ok || r == PlanStatus::AdjustedApproach);
    if (!ok) {                                   // restore previous valid plan
        for (int i = 0; i < snap_n; ++i) seg_[i] = snapSeg[i];
        n_seg_ = snap_n; c_ = snap_c; x_exit_ = snap_x; a_exit_ = snap_ae;
        loiter_sweep_ = snap_sw; resolved_L3_ = snap_L3; residual_ = snap_res;
        z0_ = snap_z0; total_len_ = snap_len; status_ = snap_st; gr_ = snap_gr;
    }
    return r;
}

Vec2 PathGuidance::evalS(float s, float* psi, float* kappa) const {
    if (n_seg_ == 0) { if (psi) *psi = 0.f; if (kappa) *kappa = 0.f; return {}; }
    if (s < 0.f) s = 0.f;
    if (s > total_len_) s = total_len_;
    int idx = 0;
    for (int i = 0; i < n_seg_; ++i) if (s >= seg_[i].s0) idx = i;
    const Segment& g = seg_[idx];
    float t = s - g.s0; if (t > g.len) t = g.len;

    if (g.type == SegType::Line) {
        if (psi) *psi = g.psi0;
        if (kappa) *kappa = 0.f;
        return { g.n0 + t * cosf(g.psi0), g.e0 + t * sinf(g.psi0) };
    }
    if (g.type == SegType::Arc) {
        const float k = g.kappa0;
        if (psi) *psi = g.psi0 + k * t;
        if (kappa) *kappa = k;
        if (fabsf(k) < 1e-6f) return { g.n0 + t * cosf(g.psi0), g.e0 + t * sinf(g.psi0) };
        const float p1 = g.psi0 + k * t;
        return { g.n0 + (sinf(p1) - sinf(g.psi0)) / k,
                 g.e0 + (cosf(g.psi0) - cosf(p1)) / k };
    }
    // clothoid
    float n1, e1, p1;
    integrateElem(g.n0, g.e0, g.psi0, g.kappa0, g.dkappa, t, n1, e1, p1);
    if (psi) *psi = p1;
    if (kappa) *kappa = g.kappa0 + g.dkappa * t;
    return { n1, e1 };
}

float PathGuidance::curvatureAt(float s) const { float k; evalS(s, nullptr, &k); return k; }

Vec2 PathGuidance::pathAt(float d) const { return evalS(arcFromAlt(d)); }

HeadingCmd PathGuidance::getHeading(const State& s) const {
    HeadingCmd cmd;
    cmd.valid = (status_ == PlanStatus::Ok || status_ == PlanStatus::AdjustedApproach);

    const float s_now = arcFromAlt(s.d);
    // phase by segment under s_now
    int idx = 0;
    for (int i = 0; i < n_seg_; ++i) if (s_now >= seg_[i].s0) idx = i;
    if (s.d >= p_.land_d - 1e-3f) cmd.phase = Phase::Landed;
    else if (n_seg_ == 0)          cmd.phase = Phase::Init;
    else if (idx >= n_seg_ - 1)    cmd.phase = Phase::Approach;
    else if (seg_[idx].type == SegType::Arc ||
             (seg_[idx].type == SegType::Clothoid && idx >= 1)) cmd.phase = Phase::Loiter;
    else                           cmd.phase = Phase::Homing;

    float s_car = arcFromAlt(s.d + p_.lookahead_drop);
    if (s_car > total_len_) s_car = total_len_;
    const Vec2 carrot = evalS(s_car);
    cmd.carrot = carrot;
    cmd.heading = atan2f(carrot.e - s.e, carrot.n - s.n);

    const float vh = hypotf(s.vn, s.ve);
    cmd.glide_angle = atan2f(s.vd, vh);
    return cmd;
}

} // namespace rsx
