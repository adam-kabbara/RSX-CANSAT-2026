#!/usr/bin/env python3
"""
sim.py -- RSX CanSat 2026 descent-guidance debug front-end (v2: clothoid + replan)

Closes the loop on the *real* C++ guidance (pathguidance pybind11 module):
    simulated EKF state -> get_heading() -> coordinated-turn glider -> repeat.

Two things this version demonstrates:
  1. Curvature-continuous (G2) path: LINE -> CLOTHOID -> ARC -> CLOTHOID -> LINE.
     The curvature-vs-arclength panel is the proof: no steps => no roll-command
     steps for your rate controller.
  2. Receding-horizon replan(): inject extra sink (glider descends faster than
     the nominal glide ratio) and periodically call replan(state). Without
     replanning the energy budget drifts; with it, the remaining loiter sweep is
     recomputed so the exit still lands into-wind on target.

Interactive sliders need a GUI backend (run `python3 sim.py` on your machine).
When run headless (MPLBACKEND=Agg) it renders a static PNG instead.

Build the module first (see README.md).
"""
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import pathguidance as pg

G = 9.81
PHASE_NAMES = {0: "Init", 1: "Homing", 2: "Loiter", 3: "Approach", 4: "Landed"}
SEG_CHAR = {0: "Line", 1: "Clothoid", 2: "Arc"}


def make_params():
    p = pg.GuidanceParams()
    p.start_n, p.start_e, p.start_d = 0.0, 0.0, -500.0
    p.land_n,  p.land_e,  p.land_d  = 60.0, 90.0, 0.0
    p.start_heading  = math.radians(64)     # homing heading INPUT (feasible window ~60-74 deg)
    p.land_heading   = math.radians(30)     # into-wind final approach INPUT
    p.glide_ratio    = 3.0
    p.approach_len    = 60.0
    p.approach_len_min = 25.0
    p.loiter_radius   = 40.0
    p.min_turn_radius = 25.0
    p.transition_len  = 12.0
    p.entry_clothoid_max = 2.0 * p.loiter_radius   # reject far-off-tangent homing headings
    p.loiter_dir      = +1
    p.lookahead_drop  = 11.0
    return p


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def reference_path(g, npts=700):
    """Sample the *current* plan along arc length -> true geometry incl. clothoids."""
    L = g.total_length()
    if L <= 0:
        return np.array([]), np.array([])
    ss = np.linspace(0, L, npts)
    pts = [g.eval_s(float(s)) for s in ss]
    return np.array([q[0] for q in pts]), np.array([q[1] for q in pts])


def curvature_profile(g, npts=900):
    L = g.total_length()
    if L <= 0:
        return np.array([]), np.array([])
    ss = np.linspace(0, L, npts)
    ks = np.array([g.curvature_at(float(s)) for s in ss])
    return ss, ks


def simulate(p, descent_rate=5.0, dt=0.05, kp_bank=2.2,
             sink_bias=1.0, replan_every=0.0):
    """
    Coordinated-turn kinematic glider flying the C++ guidance.
      sink_bias    : actual sink / nominal sink (1.15 => sinks 15% faster than the
                     glide_ratio the planner assumed -> energy budget drifts).
      replan_every : call replan(state) every this many metres of altitude lost
                     (0 disables). Receding-horizon re-closing of the budget.
    Returns (guidance, status, log, Vh, phi_max, g_initial_ref).
    """
    g = pg.PathGuidance(p)
    status = g.plan()
    init_ref = reference_path(g)
    sname = str(status).split('.')[-1]
    if sname not in ("Ok", "AdjustedApproach"):
        return g, status, None, 0.0, 0.0, init_ref

    Vh = p.glide_ratio * descent_rate
    phi_max = math.atan(Vh * Vh / (G * p.min_turn_radius))
    actual_sink = descent_rate * sink_bias

    n, e, d = p.start_n, p.start_e, p.start_d
    yaw = p.start_heading
    log = {k: [] for k in ("n", "e", "d", "yaw", "phase", "cn", "ce", "xte", "replan_d")}
    last_replan_d = d
    steps, max_steps = 0, int(600.0 / dt)

    while d < p.land_d - 1e-3 and steps < max_steps:
        st = pg.State(); st.n, st.e, st.d = n, e, d
        st.vn = Vh * math.cos(yaw); st.ve = Vh * math.sin(yaw); st.vd = actual_sink
        st.yaw = yaw
        cmd = g.get_heading(st)

        # receding-horizon replan: re-close the energy budget where the slack is
        # (the loiter). Replanning the homing leg just thrashes the entry solve.
        if (replan_every > 0.0 and int(cmd.phase) == 2
                and (d - last_replan_d) >= replan_every):
            rs = g.replan(st)
            if str(rs).split('.')[-1] in ("Ok", "AdjustedApproach"):
                log["replan_d"].append(d)
                cmd = g.get_heading(st)
            last_replan_d = d

        err = wrap_pi(cmd.heading - yaw)
        bank = max(-phi_max, min(phi_max, kp_bank * err))
        yaw = wrap_pi(yaw + (G * math.tan(bank) / Vh) * dt)

        # cross-track vs nearest point on the *current* reference (sampled)
        rn, re_ = reference_path(g, 250)
        xte = float(np.min(np.hypot(rn - n, re_ - e))) if rn.size else 0.0

        log["n"].append(n); log["e"].append(e); log["d"].append(d)
        log["yaw"].append(yaw); log["phase"].append(int(cmd.phase))
        log["cn"].append(cmd.carrot.n); log["ce"].append(cmd.carrot.e); log["xte"].append(xte)

        n += Vh * math.cos(yaw) * dt
        e += Vh * math.sin(yaw) * dt
        d += actual_sink * dt
        steps += 1

    out = {k: np.array(v) for k, v in log.items()}
    return g, status, out, Vh, phi_max, init_ref


# ----------------------------------------------------------------------------
# rendering
# ----------------------------------------------------------------------------
def draw(fig, axes, p, descent_rate, sink_bias, replan_every):
    axP, axK, axX = axes
    for a in axes:
        a.clear()

    g, status, log, Vh, phi_max, init_ref = simulate(
        p, descent_rate=descent_rate, sink_bias=sink_bias, replan_every=replan_every)
    sname = str(status).split('.')[-1]
    c = g.center()

    # ---- top-down ----
    th = np.linspace(0, 2 * math.pi, 120)
    if init_ref[0].size:
        axP.plot(init_ref[1], init_ref[0], "--", color="0.65", lw=1.5, label="initial plan")
    if g.total_length() > 0:
        axP.plot(c.e + p.loiter_radius * np.sin(th), c.n + p.loiter_radius * np.cos(th),
                 ":", color="0.8", lw=1.0)
    if log is not None:
        # final (replanned) reference
        fn, fe = reference_path(g)
        if replan_every > 0 and fn.size:
            axP.plot(fe, fn, ":", color="#2da44e", lw=1.5, label="final plan (replanned)")
        axP.plot(log["e"], log["n"], "-", color="#1f6feb", lw=2.0, label="flown")
        if log["replan_d"].size:
            for rd in log["replan_d"]:
                idx = int(np.argmin(np.abs(log["d"] - rd)))
                axP.scatter([log["e"][idx]], [log["n"][idx]], c="#2da44e", s=18, zorder=6)
        miss = math.hypot(log["n"][-1] - p.land_n, log["e"][-1] - p.land_e)
    else:
        miss = float("nan")
    axP.scatter([p.start_e], [p.start_n], c="green", s=70, zorder=5, label="start")
    axP.scatter([p.land_e], [p.land_n], c="red", s=90, marker="*", zorder=5, label="landing")
    dl = 22.0
    axP.annotate("", xy=(p.land_e + dl * math.sin(p.land_heading),
                         p.land_n + dl * math.cos(p.land_heading)),
                 xytext=(p.land_e, p.land_n),
                 arrowprops=dict(arrowstyle="->", color="red", lw=1.6))
    # homing heading arrow at start
    axP.annotate("", xy=(p.start_e + dl * math.sin(p.start_heading),
                         p.start_n + dl * math.cos(p.start_heading)),
                 xytext=(p.start_e, p.start_n),
                 arrowprops=dict(arrowstyle="->", color="green", lw=1.4))
    axP.set_xlabel("East (m)"); axP.set_ylabel("North (m)")
    title = f"status={sname}"
    if log is not None:
        title += (f"  |  loiter {g.loiter_turns():.2f} turns  |  miss {miss:.1f} m"
                  f"  |  resid {g.budget_residual():+.1f} m")
    if sname == "EntryInfeasible":
        title += "  -- homing heading can't reach the loiter (try ~60-74 deg)"
    axP.set_title(title, fontsize=9)
    axP.axis("equal"); axP.grid(alpha=0.3); axP.legend(loc="best", fontsize=7)

    # ---- curvature vs arc length (G2 proof) ----
    ss, ks = curvature_profile(g)
    if ss.size:
        axK.plot(ss, ks, "-", color="#8250df", lw=1.6)
        axK.axhline(p.loiter_dir / p.loiter_radius, ls=":", color="0.6", lw=1.0)
        axK.axhline(0, ls="-", color="0.85", lw=0.8)
        # shade segment spans
        x0 = 0.0
        cols = {0: "#eef3ff", 1: "#fff3e6", 2: "#eafbef"}
        for i in range(g.seg_count()):
            s0 = g.seg_s0(i); ln = g.seg_len(i)
            axK.axvspan(s0, s0 + ln, color=cols.get(int(g.seg_type(i)), "white"), alpha=0.5)
    axK.set_xlabel("arc length s (m)"); axK.set_ylabel("curvature kappa (1/m)")
    axK.set_title("Curvature profile -- continuous => G2 (shaded: Line/Clothoid/Arc)",
                  fontsize=9)
    axK.grid(alpha=0.3)

    # ---- cross-track error vs altitude ----
    if log is not None and log["xte"].size:
        axX.plot(-log["d"], log["xte"], "-", color="#d1242f", lw=1.5)
        if log["replan_d"].size:
            for rd in log["replan_d"]:
                axX.axvline(-rd, ls=":", color="#2da44e", lw=1.0)
    axX.set_xlabel("altitude AGL (m)"); axX.set_ylabel("cross-track (m)")
    axX.invert_xaxis()
    axX.set_title(f"Tracking error  (Vh={Vh:.1f} m/s, bank_max={math.degrees(phi_max):.0f} deg,"
                  f" sink x{sink_bias:.2f})", fontsize=9)
    axX.grid(alpha=0.3)

    fig.canvas.draw_idle()
    return g, status, log


def build_interactive(p):
    from matplotlib.widgets import Slider, Button, RadioButtons
    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.5, 1.0],
                          left=0.06, right=0.98, top=0.95, bottom=0.32, hspace=0.35, wspace=0.22)
    axP = fig.add_subplot(gs[:, 0]); axK = fig.add_subplot(gs[0, 1]); axX = fig.add_subplot(gs[1, 1])
    axes = (axP, axK, axX)

    state = dict(descent_rate=5.0, sink_bias=1.0, replan_every=0.0)

    def add_slider(x, y, w, label, lo, hi, val, fmt="%.0f"):
        ax = fig.add_axes([x, y, w, 0.025])
        return Slider(ax, label, lo, hi, valinit=val, valfmt=fmt)

    s_home  = add_slider(0.08, 0.24, 0.34, "homing hdg (deg)", 0, 359, math.degrees(p.start_heading))
    s_land  = add_slider(0.08, 0.20, 0.34, "land hdg (deg)",   0, 359, math.degrees(p.land_heading))
    s_R     = add_slider(0.08, 0.16, 0.34, "loiter R (m)",     20, 80, p.loiter_radius)
    s_gr    = add_slider(0.08, 0.12, 0.34, "glide ratio",      2.0, 6.0, p.glide_ratio, "%.2f")
    s_appr  = add_slider(0.08, 0.08, 0.34, "approach len (m)", 25, 120, p.approach_len)
    s_tr    = add_slider(0.08, 0.04, 0.34, "transition Lc (m)",4, 30, p.transition_len)

    s_look  = add_slider(0.56, 0.24, 0.34, "lookahead drop (m)", 2, 25, p.lookahead_drop)
    s_alt   = add_slider(0.56, 0.20, 0.34, "start alt (m)",     80, 400, -p.start_d)
    s_sink  = add_slider(0.56, 0.16, 0.34, "sink bias x",       0.8, 1.4, 1.0, "%.2f")
    s_rep   = add_slider(0.56, 0.12, 0.34, "replan every (m)",  0, 60, 0.0)

    ax_dir = fig.add_axes([0.56, 0.02, 0.12, 0.08]); ax_dir.set_title("loiter dir", fontsize=8)
    r_dir = RadioButtons(ax_dir, ("right (+1)", "left (-1)"), active=0 if p.loiter_dir > 0 else 1)
    ax_btn = fig.add_axes([0.78, 0.045, 0.12, 0.045]); b_reset = Button(ax_btn, "reset view")

    def recompute(_=None):
        p.start_heading = math.radians(s_home.val)
        p.land_heading  = math.radians(s_land.val)
        p.loiter_radius = s_R.val
        p.glide_ratio   = s_gr.val
        p.approach_len  = s_appr.val
        p.transition_len = s_tr.val
        p.lookahead_drop = s_look.val
        p.start_d       = -s_alt.val
        p.loiter_dir    = +1 if r_dir.value_selected.startswith("right") else -1
        draw(fig, axes, p, descent_rate=5.0, sink_bias=s_sink.val, replan_every=s_rep.val)

    for s in (s_home, s_land, s_R, s_gr, s_appr, s_tr, s_look, s_alt, s_sink, s_rep):
        s.on_changed(recompute)
    r_dir.on_clicked(recompute)
    b_reset.on_clicked(lambda _: (axP.relim(), axP.autoscale(), fig.canvas.draw_idle()))

    recompute()
    plt.show()


if __name__ == "__main__":
    p = make_params()
    backend = matplotlib.get_backend().lower()
    headless = backend.endswith("agg")
    if headless:
        # static validation render: show a replan demo with extra sink
        fig = plt.figure(figsize=(14, 8))
        gs = fig.add_gridspec(2, 2, width_ratios=[1.5, 1.0],
                              left=0.06, right=0.98, top=0.93, bottom=0.08, hspace=0.3, wspace=0.22)
        axes = (fig.add_subplot(gs[:, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, 1]))
        g, status, log = draw(fig, axes, p, descent_rate=5.0, sink_bias=1.0, replan_every=0.0)
        fig.suptitle("RSX CanSat 2026 -- v2 guidance: G2 clothoid path "
                     "(LINE-CLOTHOID-ARC-CLOTHOID-LINE), clean flight",
                     fontsize=11)
        fig.savefig("path_plot_v2.png", dpi=130)
        print("saved path_plot_v2.png   (headless; run on your machine for sliders)")
        print("status         :", str(status).split('.')[-1])
        if log is not None:
            miss = math.hypot(log["n"][-1] - p.land_n, log["e"][-1] - p.land_e)
            print("loiter turns   : %.3f" % g.loiter_turns())
            print("touchdown miss : %.2f m" % miss)
            print("max cross-track: %.2f m" % log["xte"].max())
            print("budget residual: %+.2f m" % g.budget_residual())
            print("replans fired  : %d" % log["replan_d"].size)
    else:
        build_interactive(p)
