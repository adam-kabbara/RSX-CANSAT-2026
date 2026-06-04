#!/usr/bin/env python3
"""
v3proto.py -- prototype of the v3 ("solve from current state") descent planner.
Pure geometry, validate before porting to C++.

Path:  ARC(align) -> LINE(homing) -> SPIRAL(N loops, radius R) -> LINE(approach) -> land
All junctions tangent (G1). Inputs = current state (pos, z, heading) + landing
spec (pos, axis, box). Solves {landing dir, spiral side, R, N, entry} to close
the budget arc = GR*(z - z_land) inside a hard bounding box.
"""
import math, numpy as np

def dirOf(a):      return np.array([math.cos(a), math.sin(a)])   # (n,e)
def rightN(a):     return np.array([-math.sin(a), math.cos(a)])  # right of heading a
def leftN(a):      return np.array([ math.sin(a), -math.cos(a)])
def ang(v):        return math.atan2(v[1], v[0])                 # atan2(e,n)
def wrap2pi(a):    a%= 2*math.pi; return a+2*math.pi if a<0 else a
def wrappi(a):     return (a+math.pi)%(2*math.pi)-math.pi
def head_on_circle(a, s): return math.atan2(s*math.cos(a), -s*math.sin(a))  # travel heading at radial a, sense s

def circle_tangent(O1, r1, s1, O2, r2, s2):
    """Straight leg leaving circle1 (sense s1) tangent, arriving circle2 (sense s2) tangent.
       Returns (TA, TB, leg_heading) or None."""
    V = O2 - O1; d = math.hypot(*V)
    if d < 1e-6: return None
    theta = ang(V)
    same = (s1 == s2)
    val = (r1 - r2)/d if same else (r1 + r2)/d
    if abs(val) > 1.0: return None
    for sgn in (+1, -1):
        phi = theta + sgn*math.acos(val)
        u = np.array([math.cos(phi), math.sin(phi)])
        TA = O1 + r1*u
        TB = O2 + r2*u if same else O2 - r2*u
        leg = TB - TA
        if math.hypot(*leg) < 1e-6: continue
        lh = ang(leg)
        # validate: leg heading must equal the travel heading at both tangent points
        hA = head_on_circle(phi, s1)
        aB = ang(TB - O2)
        hB = head_on_circle(aB, s2)
        if abs(wrappi(lh-hA)) < 1e-3 and abs(wrappi(lh-hB)) < 1e-3:
            return TA, TB, lh
    return None

def align_entry(P0, chi0, C, R, sdir, r0):
    """Dubins ARC(align,r0)+LINE from (P0,chi0) onto spiral (C,R,sdir).
       Returns dict(align O1,s,aP0,sweep, TA, TB, a_entry, entry_len) for the
       feasible align option with the shortest path, or None."""
    best = None
    for sA in (+1, -1):
        O1 = P0 + r0*(rightN(chi0) if sA > 0 else leftN(chi0))
        tg = circle_tangent(O1, r0, sA, C, R, sdir)
        if tg is None: continue
        TA, TB, lh = tg
        aP0 = ang(P0 - O1); aTA = ang(TA - O1)
        sweepA = wrap2pi(sA*(aTA - aP0))            # align arc sweep (>=0)
        leg = math.hypot(*(TB - TA))
        a_entry = ang(TB - C)
        entry_len = r0*sweepA + leg
        cand = dict(O1=O1, sA=sA, aP0=aP0, aTA=aTA, sweepA=sweepA, TA=TA, TB=TB,
                    leg=leg, a_entry=a_entry, entry_len=entry_len)
        if best is None or entry_len < best["entry_len"]:
            best = cand
    return best

def build(P0, chi0, z, land, zland, land_dir, sdir, GR, r0, minR, maxR, L3, box):
    """Build one candidate {land_dir, spiral side sdir}. Returns dict or None."""
    psi4 = land_dir
    ad = dirOf(psi4)
    pt3 = land - L3*ad                                  # rollout point
    budget = GR*(z - zland)
    a_exit = None

    def geom(R):
        C = pt3 + R*sdir*np.array([-math.sin(psi4), math.cos(psi4)])  # tangent to approach
        ae = ang(pt3 - C)
        ent = align_entry(P0, chi0, C, R, sdir, r0)
        return C, ae, ent

    # for each loop count N, root-find R so the budget closes
    boxfit_max = min(maxR,
                     0.5*(box[1]-box[0]) - 1.0, 0.5*(box[3]-box[2]) - 1.0)
    hiR = max(minR+0.5, boxfit_max)
    best = None
    for N in range(0, 8):
        def resid(R):
            C, ae, ent = geom(R)
            if ent is None: return None
            partial = wrap2pi(sdir*(ae - ent["a_entry"]))
            arc = R*(partial + 2*math.pi*N)
            return ent["entry_len"] + arc + L3 - budget
        # bracket in [minR, hiR]
        rlo, rhi = minR, hiR
        flo, fhi = resid(rlo), resid(rhi)
        if flo is None or fhi is None: continue
        if flo > 0:           # even smallest R overshoots budget at this N -> need fewer loops
            continue
        if fhi < 0:           # even largest R can't burn enough -> need more loops
            continue
        for _ in range(60):
            rm = 0.5*(rlo+rhi); fm = resid(rm)
            if fm is None: break
            if fm < 0: rlo = rm
            else: rhi = rm
        R = 0.5*(rlo+rhi)
        C, ae, ent = geom(R)
        if ent is None: continue
        partial = wrap2pi(sdir*(ae - ent["a_entry"]))
        cand = dict(land_dir=land_dir, sdir=sdir, R=R, N=N, C=C, pt3=pt3, psi4=psi4,
                    a_exit=ae, a_entry=ent["a_entry"], partial=partial, ent=ent,
                    budget=budget)
        pts = sample_path(P0, chi0, cand, r0)
        if pts is None: continue
        cand["pts"] = pts
        if not inside_box(pts, box):  # hard box constraint
            continue
        cand["clear"] = box_clearance(pts, box)
        cand["cost"] = cost(cand, minR, hiR, box)
        if best is None or cand["cost"] < best["cost"]:
            best = cand
    return best

def sample_path(P0, chi0, c, r0):
    pts = [P0.copy()]
    ent = c["ent"]
    # align arc
    O1, sA, aP0, sweepA = ent["O1"], ent["sA"], ent["aP0"], ent["sweepA"]
    for t in np.linspace(0, sweepA, max(2, int(sweepA*r0/2)+2))[1:]:
        a = aP0 + sA*t
        pts.append(O1 + r0*np.array([math.cos(a), math.sin(a)]))
    # homing line
    TA, TB = ent["TA"], ent["TB"]
    pts.append(TB.copy())
    # spiral
    sdir, C, R = c["sdir"], c["C"], c["R"]
    sweep = c["partial"] + 2*math.pi*c["N"]
    a0 = c["a_entry"]
    for t in np.linspace(0, sweep, max(4, int(sweep*R/3)+3))[1:]:
        a = a0 + sdir*t
        pts.append(C + R*np.array([math.cos(a), math.sin(a)]))
    # approach line to land
    land = c["pt3"] + L3_GLOBAL*dirOf(c["psi4"])
    pts.append(land.copy())
    return np.array(pts)

def inside_box(pts, box):
    return (pts[:,0].min() >= box[0]-1e-6 and pts[:,0].max() <= box[1]+1e-6 and
            pts[:,1].min() >= box[2]-1e-6 and pts[:,1].max() <= box[3]+1e-6)

def box_clearance(pts, box):
    return min(pts[:,0].min()-box[0], box[1]-pts[:,0].max(),
               pts[:,1].min()-box[2], box[3]-pts[:,1].max())

def cost(c, minR, maxR, box):
    Rmid = 0.5*(minR+maxR); Rspan = max(1e-3, 0.5*(maxR-minR))
    c_R = ((c["R"]-Rmid)/Rspan)**2                 # prefer mid-range radius (room to flex)
    c_turn = 0.04*(c["ent"]["sweepA"] + c["partial"] + 2*math.pi*c["N"])  # gentle
    c_clear = 2.0*max(0.0, 8.0 - c["clear"])       # want >= 8 m wall clearance
    return c_R + c_turn + c_clear

def solve(P0, chi0, z, land, zland, axis, GR, r0, minR, maxR, L3, box):
    global L3_GLOBAL; L3_GLOBAL = L3
    cands, best = [], None
    for land_dir in (axis, axis+math.pi):
        for sdir in (+1, -1):
            c = build(P0, chi0, z, land, zland, land_dir, sdir, GR, r0, minR, maxR, L3, box)
            if c is not None:
                cands.append(c)
                if best is None or c["cost"] < best["cost"]:
                    best = c
    return best, cands

# ----------------------------------------------------------------------------
if __name__ == "__main__":
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    GR, r0, minR, maxR, L3 = 3.0, 25.0, 25.0, 70.0, 40.0
    land = np.array([0.0, 0.0]); zland = 0.0
    axis = math.radians(30)                     # landing axis (line)
    box = (-220, 220, -220, 220)

    scenarios = [
        ("nominal",        np.array([-150.,-120.]), math.radians(45),  200),
        ("adverse heading",np.array([-150.,-120.]), math.radians(225), 200),  # pointing away
        ("deployed high",  np.array([-90.,-70.]),   math.radians(60),  320),  # lots of excess alt
        ("deployed low",   np.array([-120.,-90.]),  math.radians(40),  120),  # little alt
    ]
    fig, axs = plt.subplots(2, 2, figsize=(13, 13))
    for ax,(name,P0,chi0,z) in zip(axs.ravel(), scenarios):
        best, cands = solve(P0, chi0, -(-z), land, zland, axis, GR, r0, minR, maxR, L3, box)
        # NOTE z passed as altitude AGL; budget uses (z-zland), so pass z positive AGL:
        best, cands = solve(P0, chi0, z, land, zland, axis, GR, r0, minR, maxR, L3, box)
        ax.add_patch(plt.Rectangle((box[2],box[0]), box[3]-box[2], box[1]-box[0],
                     fill=False, ec="0.7", ls="--"))
        for c in cands:
            ax.plot(c["pts"][:,1], c["pts"][:,0], "-", color="0.8", lw=1)
        if best is not None:
            ax.plot(best["pts"][:,1], best["pts"][:,0], "-", color="#1f6feb", lw=2.2)
            th=np.linspace(0,2*math.pi,80)
            ax.plot(best["C"][1]+best["R"]*np.sin(th), best["C"][0]+best["R"]*np.cos(th), ":", color="0.6", lw=1)
            tot = best["ent"]["entry_len"]+best["R"]*(best["partial"]+2*math.pi*best["N"])+L3
            ax.set_title("%s\nland_dir=%.0f sdir=%+d R=%.1f N=%d resid=%.3f"%(
                name, math.degrees(best["land_dir"])%360, best["sdir"], best["R"], best["N"],
                tot-best["budget"]), fontsize=9)
        else:
            ax.set_title(name+"  -- NO FEASIBLE CANDIDATE", fontsize=9)
        ax.annotate("", xy=(P0[1]+25*math.sin(chi0), P0[0]+25*math.cos(chi0)), xytext=(P0[1],P0[0]),
                    arrowprops=dict(arrowstyle="->", color="green", lw=1.5))
        ax.scatter([P0[1]],[P0[0]], c="green", s=45)
        ax.scatter([land[1]],[land[0]], c="red", s=90, marker="*")
        ax.annotate("", xy=(land[1]+30*math.sin(axis), land[0]+30*math.cos(axis)),
                    xytext=(land[1]-30*math.sin(axis), land[0]-30*math.cos(axis)),
                    arrowprops=dict(arrowstyle="-", color="red", lw=1, ls=":"))
        ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.set_xlabel("E"); ax.set_ylabel("N")
    fig.suptitle("v3 prototype: solve {land dir, spiral side, R, N, Dubins entry} from current state", fontsize=12)
    fig.tight_layout(); fig.savefig("v3proto.png", dpi=110)
    print("saved v3proto.png")
