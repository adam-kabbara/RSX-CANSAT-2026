//
// PathGuidance.cpp  --  RSX CanSat 2026 "Autonomous Paraglider"  (v3)
//
#include "PathGuidance.hpp"
#include <cmath>

namespace rsx {

static constexpr float kPi    = 3.14159265358979f;
static constexpr float kTwoPi = 6.28318530717959f;

static inline Vec2  dirOf(float a)  { return { cosf(a), sinf(a) }; }
static inline Vec2  rightN(float a) { return { -sinf(a), cosf(a) }; }
static inline Vec2  leftN(float a)  { return {  sinf(a), -cosf(a) }; }
static inline float ang(Vec2 v)     { return atan2f(v.e, v.n); }
static inline float norm(Vec2 v)    { return hypotf(v.n, v.e); }
static inline Vec2  add(Vec2 a, Vec2 b){ return { a.n+b.n, a.e+b.e }; }
static inline Vec2  sub(Vec2 a, Vec2 b){ return { a.n-b.n, a.e-b.e }; }
static inline Vec2  mul(Vec2 a, float k){ return { a.n*k, a.e*k }; }
static inline float wrapPi(float a){ a=fmodf(a+kPi,kTwoPi); if(a<0)a+=kTwoPi; return a-kPi; }
static inline float wrap2pi(float a){ a=fmodf(a,kTwoPi); if(a<0)a+=kTwoPi; return a; }
static inline float headOnCircle(float a, int s){ return atan2f(s*cosf(a), -s*sinf(a)); }

void PathGuidance::setParams(const GuidanceParams& p) {
    p_ = p; gr_ = p.glide_ratio; start_d_ = p.start_d;
    psi4_ = p.landing_axis; status_ = PlanStatus::Infeasible;
}

PathGuidance::Tangent
PathGuidance::circleTangent(Vec2 O1, float r1, int s1, Vec2 O2, float r2, int s2) const {
    Tangent t;
    const Vec2 V = sub(O2, O1); const float d = norm(V);
    if (d < 1e-4f) return t;
    const float theta = ang(V);
    const bool same = (s1 == s2);
    const float val = same ? (r1 - r2) / d : (r1 + r2) / d;
    if (fabsf(val) > 1.f) return t;
    for (int sgn = +1; sgn >= -1; sgn -= 2) {
        const float phi = theta + sgn * acosf(val);
        const Vec2 u { cosf(phi), sinf(phi) };
        const Vec2 TA = add(O1, mul(u, r1));
        const Vec2 TB = same ? add(O2, mul(u, r2)) : sub(O2, mul(u, r2));
        const Vec2 leg = sub(TB, TA);
        if (norm(leg) < 1e-4f) continue;
        const float lh = ang(leg);
        const float hA = headOnCircle(phi, s1);
        const float aB = ang(sub(TB, O2));
        const float hB = headOnCircle(aB, s2);
        if (fabsf(wrapPi(lh - hA)) < 2e-3f && fabsf(wrapPi(lh - hB)) < 2e-3f) {
            t.ok = true; t.TA = TA; t.TB = TB; t.legHeading = lh; return t;
        }
    }
    return t;
}

PathGuidance::AlignEntry
PathGuidance::alignEntry(Vec2 P0, float chi0, Vec2 C, float R, int sdir) const {
    const float r0 = p_.min_turn_radius;
    AlignEntry best;
    for (int sA = +1; sA >= -1; sA -= 2) {
        const Vec2 O1 = add(P0, mul(sA > 0 ? rightN(chi0) : leftN(chi0), r0));
        const Tangent tg = circleTangent(O1, r0, sA, C, R, sdir);
        if (!tg.ok) continue;
        const float aP0 = ang(sub(P0, O1));
        const float aTA = ang(sub(tg.TA, O1));
        const float sweepA = wrap2pi(sA * (aTA - aP0));
        const float leg = norm(sub(tg.TB, tg.TA));
        AlignEntry c;
        c.ok = true; c.O1 = O1; c.sA = (float)sA; c.aP0 = aP0; c.aTA = aTA;
        c.sweepA = sweepA; c.TA = tg.TA; c.TB = tg.TB; c.leg = leg;
        c.aEntry = ang(sub(tg.TB, C)); c.entryLen = r0 * sweepA + leg; c.legHeading = tg.legHeading;
        if (!best.ok || c.entryLen < best.entryLen) best = c;
    }
    return best;
}

void PathGuidance::pathExtents(const Candidate& c, Vec2 P0, float chi0,
                               float& nmin, float& nmax, float& emin, float& emax) const {
    const float r0 = p_.min_turn_radius;
    nmin = emin = 1e30f; nmax = emax = -1e30f;
    auto acc = [&](Vec2 q){ if(q.n<nmin)nmin=q.n; if(q.n>nmax)nmax=q.n; if(q.e<emin)emin=q.e; if(q.e>emax)emax=q.e; };
    // align arc: sample
    const AlignEntry& e = c.ent;
    const int na = 8;
    for (int i = 0; i <= na; ++i) {
        const float a = e.aP0 + e.sA * (e.sweepA * i / na);
        acc(add(e.O1, mul(Vec2{cosf(a), sinf(a)}, r0)));
    }
    acc(e.TA); acc(e.TB);
    // spiral: full circle extents cover all loops
    acc(Vec2{ c.C.n - c.R, c.C.e }); acc(Vec2{ c.C.n + c.R, c.C.e });
    acc(Vec2{ c.C.n, c.C.e - c.R }); acc(Vec2{ c.C.n, c.C.e + c.R });
    // approach endpoints
    acc(c.pt3); acc(add(c.pt3, mul(dirOf(c.psi4), p_.approach_len)));
}

PathGuidance::Candidate
PathGuidance::buildCandidate(Vec2 P0, float chi0, float dStart, float landDir, int sdir) const {
    Candidate best;
    const float L3 = p_.approach_len;
    const Vec2  land { p_.land_n, p_.land_e };
    const Vec2  ad = dirOf(landDir);
    const Vec2  pt3 = sub(land, mul(ad, L3));
    const float budget = gr_ * (p_.land_d - dStart);
    if (budget <= 0.f) return best;

    const float boxfit = fminf(p_.max_radius,
                         fminf(0.5f*(p_.box_n_max - p_.box_n_min) - 1.f,
                               0.5f*(p_.box_e_max - p_.box_e_min) - 1.f));
    const float minR = p_.min_turn_radius;
    const float hiR  = fmaxf(minR + 0.5f, boxfit);

    auto geom = [&](float R, AlignEntry& ent, Vec2& C, float& aExit, float& partial)->bool {
        C = add(pt3, mul(rightN(landDir), R * sdir));
        aExit = ang(sub(pt3, C));
        ent = alignEntry(P0, chi0, C, R, sdir);
        if (!ent.ok) return false;
        partial = wrap2pi(sdir * (aExit - ent.aEntry));
        return true;
    };

    for (int N = 0; N < 8; ++N) {
        auto resid = [&](float R, bool& ok)->float {
            AlignEntry ent; Vec2 C; float aExit, partial;
            ok = geom(R, ent, C, aExit, partial);
            if (!ok) return 0.f;
            return ent.entryLen + R * (partial + kTwoPi * (float)N) + L3 - budget;
        };
        bool oklo, okhi;
        const float flo = resid(minR, oklo);
        const float fhi = resid(hiR,  okhi);
        if (!oklo || !okhi) continue;
        if (flo > 0.f) continue;           // smallest R already overshoots -> fewer loops
        if (fhi < 0.f) continue;           // largest R can't burn enough  -> more loops
        float rlo = minR, rhi = hiR;
        for (int it = 0; it < 60; ++it) {
            const float rm = 0.5f * (rlo + rhi);
            bool ok; const float fm = resid(rm, ok);
            if (!ok) break;
            if (fm < 0.f) rlo = rm; else rhi = rm;
        }
        const float R = 0.5f * (rlo + rhi);
        AlignEntry ent; Vec2 C; float aExit, partial;
        if (!geom(R, ent, C, aExit, partial)) continue;

        Candidate c;
        c.ok = true; c.landDir = landDir; c.sdir = sdir; c.R = R; c.N = N;
        c.C = C; c.pt3 = pt3; c.psi4 = landDir; c.aExit = aExit; c.aEntry = ent.aEntry;
        c.partial = partial; c.ent = ent; c.budget = budget;

        float nmin, nmax, emin, emax;
        pathExtents(c, P0, chi0, nmin, nmax, emin, emax);
        if (nmin < p_.box_n_min || nmax > p_.box_n_max ||
            emin < p_.box_e_min || emax > p_.box_e_max) continue;     // hard box
        c.clear = fminf(fminf(nmin - p_.box_n_min, p_.box_n_max - nmax),
                        fminf(emin - p_.box_e_min, p_.box_e_max - emax));

        const float Rmid = 0.5f*(minR + hiR), Rspan = fmaxf(1e-3f, 0.5f*(hiR - minR));
        const float cR = ((R - Rmid)/Rspan) * ((R - Rmid)/Rspan);
        const float cTurn = 0.04f * (ent.sweepA + partial + kTwoPi * (float)N);
        const float cClear = 2.0f * fmaxf(0.f, 8.f - c.clear);
        c.cost = cR + cTurn + cClear;
        if (!best.ok || c.cost < best.cost) best = c;
    }
    return best;
}

void PathGuidance::pushSeg(SegType t, float len, float n0, float e0, float psi0, float kappa0) {
    if (n_seg_ >= kMaxSeg) return;
    Segment& s = seg_[n_seg_++];
    s.type = t; s.len = len; s.n0 = n0; s.e0 = e0; s.psi0 = psi0; s.kappa0 = kappa0; s.s0 = 0.f;
}

void PathGuidance::emitSegments(const Candidate& c, Vec2 P0, float chi0) {
    n_seg_ = 0;
    const float r0 = p_.min_turn_radius;
    const AlignEntry& e = c.ent;
    if (e.sweepA > 1e-3f)
        pushSeg(SegType::Arc, r0 * e.sweepA, P0.n, P0.e, chi0, e.sA / r0);
    pushSeg(SegType::Line, e.leg, e.TA.n, e.TA.e, e.legHeading, 0.f);
    // spiral
    const float head_in = headOnCircle(c.aEntry, c.sdir);
    pushSeg(SegType::Arc, c.R * (c.partial + kTwoPi * (float)c.N),
            e.TB.n, e.TB.e, head_in, (float)c.sdir / c.R);
    // approach
    pushSeg(SegType::Line, p_.approach_len, c.pt3.n, c.pt3.e, c.psi4, 0.f);

    float acc = 0.f;
    for (int i = 0; i < n_seg_; ++i) { seg_[i].s0 = acc; acc += seg_[i].len; }
    total_len_ = acc;
    c_ = c.C; R_ = c.R; N_ = c.N; sdir_ = c.sdir; psi4_ = c.psi4;
}

PlanStatus PathGuidance::solveState(Vec2 P0, float chi0, float dStart) {
    start_ = P0; start_d_ = dStart;
    Candidate best;
    for (int li = 0; li < 2; ++li) {
        const float landDir = p_.landing_axis + (li ? kPi : 0.f);
        for (int sdir = +1; sdir >= -1; sdir -= 2) {
            Candidate c = buildCandidate(P0, chi0, dStart, landDir, sdir);
            if (c.ok && (!best.ok || c.cost < best.cost)) best = c;
        }
    }
    if (!best.ok) {
        // degraded fallback: straight best-glide toward nearest point on the axis
        n_seg_ = 0;
        const Vec2 land { p_.land_n, p_.land_e };
        const float psi = atan2f(land.e - P0.e, land.n - P0.n);
        const float L = norm(sub(land, P0));
        pushSeg(SegType::Line, L, P0.n, P0.e, psi, 0.f);
        seg_[0].s0 = 0.f; total_len_ = L;
        residual_ = (gr_ * (p_.land_d - dStart) - L) / gr_;
        c_ = P0; R_ = 0.f; N_ = 0; psi4_ = psi;
        return (status_ = PlanStatus::Degraded);
    }
    emitSegments(best, P0, chi0);
    const float used = total_len_;
    residual_ = (best.budget - used) / gr_;
    return (status_ = PlanStatus::Ok);
}

PlanStatus PathGuidance::plan() {
    gr_ = p_.glide_ratio;
    return solveState({ p_.start_n, p_.start_e }, p_.start_heading, p_.start_d);
}

PlanStatus PathGuidance::buildLoiterTail(Vec2 cur, float dCur) {
    // Keep the CURRENT spiral (same side, same landing direction, anchored to the
    // approach). Re-close the remaining budget by re-choosing loops + radius,
    // biased to the current radius so an on-track replan barely changes anything.
    // No Dubins entry -- we just continue the loiter we're already flying.
    const float dir  = (float)sdir_;
    const float psi4 = psi4_;
    const Vec2  land { p_.land_n, p_.land_e };
    const Vec2  ad   = dirOf(psi4);
    const float L3   = p_.approach_len;
    const Vec2  pt3  = sub(land, mul(ad, L3));
    const float budget = gr_ * (p_.land_d - dCur);
    if (budget <= 0.f) return PlanStatus::Infeasible;

    const float boxfit = fminf(p_.max_radius,
                         fminf(0.5f*(p_.box_n_max - p_.box_n_min) - 1.f,
                               0.5f*(p_.box_e_max - p_.box_e_min) - 1.f));
    const float minR = p_.min_turn_radius, hiR = fmaxf(minR + 0.5f, boxfit);

    auto sweepAt = [&](float R, int N, Vec2& C, float& aNow, float& aExit)->float {
        C = add(pt3, mul(rightN(psi4), R * dir));
        aExit = ang(sub(pt3, C));
        aNow  = ang(sub(cur, C));
        const float partial = wrap2pi(dir * (aExit - aNow));
        return partial + kTwoPi * (float)N;
    };

    float bestR = 0.f; int bestN = -1; float bestScore = 1e30f;
    Vec2 bestC{}; float bestANow = 0.f, bestSweep = 0.f;
    for (int N = 0; N < 8; ++N) {
        auto resid = [&](float R){ Vec2 C; float an, ae; const float sw = sweepAt(R,N,C,an,ae);
                                   return R*sw + L3 - budget; };
        const float flo = resid(minR), fhi = resid(hiR);
        if (flo > 0.f || fhi < 0.f) continue;          // this N can't bracket a root
        float rlo = minR, rhi = hiR;
        for (int it = 0; it < 60; ++it) { const float rm = 0.5f*(rlo+rhi);
            if (resid(rm) < 0.f) rlo = rm; else rhi = rm; }
        const float R = 0.5f*(rlo+rhi);
        Vec2 C; float aNow, aExit; const float sweep = sweepAt(R, N, C, aNow, aExit);
        if (C.n-R < p_.box_n_min || C.n+R > p_.box_n_max ||
            C.e-R < p_.box_e_min || C.e+R > p_.box_e_max) continue;   // box
        const float score = (R - R_)*(R - R_);          // stay near current radius
        if (score < bestScore) { bestScore=score; bestR=R; bestN=N; bestC=C; bestANow=aNow; bestSweep=sweep; }
    }
    if (bestN < 0) return PlanStatus::Infeasible;        // caller rolls back

    start_ = cur; start_d_ = dCur;
    n_seg_ = 0;
    const Vec2 sp { bestC.n + bestR*cosf(bestANow), bestC.e + bestR*sinf(bestANow) };
    pushSeg(SegType::Arc, bestR*bestSweep, sp.n, sp.e, headOnCircle(bestANow, sdir_), (float)sdir_/bestR);
    pushSeg(SegType::Line, L3, pt3.n, pt3.e, psi4, 0.f);
    float acc = 0.f; for (int i=0;i<n_seg_;++i){ seg_[i].s0=acc; acc+=seg_[i].len; }
    total_len_ = acc; c_ = bestC; R_ = bestR; N_ = bestN;
    residual_ = (budget - total_len_) / gr_;
    return (status_ = PlanStatus::Ok);
}

PlanStatus PathGuidance::replan(const State& s) {
    // "In the loiter" = physically on/near the current spiral circle (robust to a
    // sudden z-shift, which would fool an altitude-band test).
    const Vec2 cur { s.n, s.e };
    const float radial = (R_ > 1.f) ? hypotf(cur.n - c_.n, cur.e - c_.e) : 1e9f;
    const bool nearSpiral = (R_ > 1.f) && (fabsf(radial - R_) < fmaxf(15.f, 0.5f * R_));

    const float vh = hypotf(s.vn, s.ve);
    if (s.vd > 0.3f && vh > 0.5f) {
        float gm = vh / s.vd; if (gm < 1.f) gm = 1.f; if (gm > 8.f) gm = 8.f; gr_ = gm;
    }
    const float chi = (vh > 0.5f) ? atan2f(s.ve, s.vn) : s.yaw;

    Segment snap[kMaxSeg]; for (int i=0;i<n_seg_;++i) snap[i]=seg_[i];
    const int sn=n_seg_; const Vec2 sc=c_, sst=start_; const float sR=R_, sp=psi4_;
    const int sN=N_, ssd=sdir_; const float sz=start_d_, sl=total_len_, sr=residual_;
    const PlanStatus sS=status_;

    PlanStatus r;
    if (nearSpiral) {
        r = buildLoiterTail(cur, s.d);              // keep & resize the loiter we're on
        if (r == PlanStatus::Infeasible)            // can't absorb on this circle ->
            r = solveState(cur, chi, s.d);          //   full re-solve (may relocate)
    } else {
        r = solveState(cur, chi, s.d);              // align/homing/approach: full re-solve
    }

    if (r == PlanStatus::Infeasible) {
        for (int i=0;i<sn;++i) seg_[i]=snap[i];
        n_seg_=sn; c_=sc; start_=sst; R_=sR; psi4_=sp; N_=sN; sdir_=ssd;
        start_d_=sz; total_len_=sl; residual_=sr; status_=sS;
    }
    return r;
}

Vec2 PathGuidance::evalS(float s, float* psi, float* kappa) const {
    if (n_seg_ == 0) { if(psi)*psi=0; if(kappa)*kappa=0; return {}; }
    if (s < 0.f) s = 0.f;
    if (s > total_len_) s = total_len_;
    int idx = 0;
    for (int i = 0; i < n_seg_; ++i) if (s >= seg_[i].s0) idx = i;
    const Segment& g = seg_[idx];
    float t = s - g.s0; if (t > g.len) t = g.len;
    if (g.type == SegType::Line) {
        if(psi)*psi=g.psi0;
        if(kappa)*kappa=0.f;
        return { g.n0 + t*cosf(g.psi0), g.e0 + t*sinf(g.psi0) };
    }
    const float k = g.kappa0;
    if(psi)*psi = g.psi0 + k*t;
    if(kappa)*kappa = k;
    if (fabsf(k) < 1e-6f) return { g.n0 + t*cosf(g.psi0), g.e0 + t*sinf(g.psi0) };
    const float p1 = g.psi0 + k*t;
    return { g.n0 + (sinf(p1) - sinf(g.psi0))/k, g.e0 + (cosf(g.psi0) - cosf(p1))/k };
}

float PathGuidance::curvatureAt(float s) const { float k; evalS(s, nullptr, &k); return k; }

HeadingCmd PathGuidance::getHeading(const State& s) const {
    HeadingCmd cmd;
    cmd.valid = (status_ == PlanStatus::Ok || status_ == PlanStatus::Degraded);
    const float s_now = arcFromAlt(s.d);
    int idx = 0;
    for (int i = 0; i < n_seg_; ++i) if (s_now >= seg_[i].s0) idx = i;
    if (s.d >= p_.land_d - 1e-3f) cmd.phase = Phase::Landed;
    else if (n_seg_ == 0) cmd.phase = Phase::Init;
    else if (idx == 0 && seg_[0].type == SegType::Arc) cmd.phase = Phase::Align;
    else if (seg_[idx].type == SegType::Arc) cmd.phase = Phase::Loiter;
    else if (idx >= n_seg_ - 1) cmd.phase = Phase::Approach;
    else cmd.phase = Phase::Homing;

    float d_car = s.d + p_.lookahead_drop;
    if (d_car > p_.land_d) d_car = p_.land_d;
    const Vec2 carrot = evalS(arcFromAlt(d_car));
    cmd.carrot = carrot;
    cmd.heading = atan2f(carrot.e - s.e, carrot.n - s.n);
    cmd.glide_angle = atan2f(s.vd, hypotf(s.vn, s.ve));
    return cmd;
}

} // namespace rsx
