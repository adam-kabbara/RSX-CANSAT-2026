#!/usr/bin/env python3
"""
sim.py -- RSX CanSat 2026 descent-guidance interactive simulator (v2)

A real-time front-end on the *real* C++ guidance (pathguidance pybind11 module):

  * watch the glider fly the path forward in time (Play / Pause / Reset),
  * change every initial value live with sliders (re-plans on change),
  * replan during flight -- a button for "replan now" and an "auto-replan in
    loiter every N m" slider (receding-horizon energy re-closing, using the
    measured glide ratio),
  * GRAB THE GLIDER WITH THE MOUSE and drag it off course to inject an external
    disturbance (gust / push); release and watch it track back, hit Replan to
    re-close the budget from the new state,
  * live curvature panel (the G2 proof) + cross-track trace.

Run:  python3 sim.py
Needs an interactive matplotlib backend. On Linux, if no window appears:
    sudo apt-get install python3-tk      (then re-run)
Build the pathguidance module first (see README.md).
"""
import sys
import math
import numpy as np

# ---- pick an interactive backend by actually trying to create a figure -------
import matplotlib
import matplotlib.pyplot as plt                    # noqa: E402
INTERACTIVE = False
for _bk in ("TkAgg", "QtAgg", "Qt5Agg", "MacOSX", "GTK3Agg"):
    try:
        plt.switch_backend(_bk)
        _f = plt.figure(); plt.close(_f)            # forces real canvas creation
        INTERACTIVE = True
        break
    except Exception:
        continue
if not INTERACTIVE:
    plt.switch_backend("Agg")

from matplotlib.widgets import Slider, Button      # noqa: E402
import pathguidance as pg                          # noqa: E402

G = 9.81
PHASE = {0: "Init", 1: "Homing", 2: "Loiter", 3: "Approach", 4: "Landed"}
SEGCOL = {0: "#eef3ff", 1: "#fff3e6", 2: "#eafbef"}   # Line / Clothoid / Arc


def default_params():
    p = pg.GuidanceParams()
    p.start_n, p.start_e, p.start_d = 0.0, 0.0, -200.0
    p.land_n,  p.land_e,  p.land_d  = 60.0, 90.0, 0.0
    p.start_heading  = math.radians(64)
    p.land_heading   = math.radians(30)
    p.glide_ratio    = 3.0
    p.approach_len    = 60.0
    p.approach_len_min = 25.0
    p.loiter_radius   = 40.0
    p.min_turn_radius = 25.0
    p.transition_len  = 12.0
    p.entry_clothoid_max = 2.0 * 40.0
    p.loiter_dir      = +1
    p.lookahead_drop  = 11.0
    return p


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def reference_xy(g, npts=600):
    L = g.total_length()
    if L <= 0:
        return np.array([]), np.array([])
    ss = np.linspace(0, L, npts)
    q = [g.eval_s(float(s)) for s in ss]
    return np.array([w[0] for w in q]), np.array([w[1] for w in q])


def curvature_xy(g, npts=700):
    L = g.total_length()
    if L <= 0:
        return np.array([]), np.array([])
    ss = np.linspace(0, L, npts)
    return ss, np.array([g.curvature_at(float(s)) for s in ss])


class Sim:
    """Headless-testable simulation state. step() advances the glider one dt."""
    def __init__(self, p, descent_rate=5.0, dt=0.05, kp_bank=2.2):
        self.p = p
        self.dt = dt
        self.kp_bank = kp_bank
        self.descent_rate = descent_rate
        self.g = pg.PathGuidance(p)
        self.status = self.g.plan()
        self.sink_bias = 1.0
        self.wind_n = 0.0
        self.wind_e = 0.0
        self.reset_state()

    def feasible(self):
        return str(self.status).split('.')[-1] in ("Ok", "AdjustedApproach")

    def reset_state(self):
        self.n, self.e, self.d = self.p.start_n, self.p.start_e, self.p.start_d
        self.yaw = self.p.start_heading
        self.Vh = self.p.glide_ratio * self.descent_rate
        self.phi_max = math.atan(self.Vh * self.Vh / (G * self.p.min_turn_radius))
        self.trail_n, self.trail_e = [self.n], [self.e]
        self.alt_log, self.xte_log = [], []
        self.replan_marks = []
        self.landed = False
        self.last_replan_d = self.d
        self.cmd = None

    def replan_params(self):
        """Re-read params into a fresh plan, reset the flight."""
        self.g = pg.PathGuidance(self.p)
        self.status = self.g.plan()
        self.reset_state()

    def state_msg(self):
        st = pg.State()
        st.n, st.e, st.d = self.n, self.e, self.d
        st.vn = self.Vh * math.cos(self.yaw)
        st.ve = self.Vh * math.sin(self.yaw)
        st.vd = self.descent_rate * self.sink_bias
        st.yaw = self.yaw
        return st

    def replan_now(self):
        if self.landed:
            return None
        r = self.g.replan(self.state_msg())
        if str(r).split('.')[-1] in ("Ok", "AdjustedApproach"):
            self.replan_marks.append((self.e, self.n))
        return r

    def step(self, auto_replan_m=0.0):
        if self.landed or not self.feasible():
            return
        st = self.state_msg()
        self.cmd = self.g.get_heading(st)
        if (auto_replan_m > 0.0 and int(self.cmd.phase) == 2
                and (self.d - self.last_replan_d) >= auto_replan_m):
            self.replan_now()
            self.last_replan_d = self.d
            self.cmd = self.g.get_heading(self.state_msg())

        err = wrap_pi(self.cmd.heading - self.yaw)
        bank = max(-self.phi_max, min(self.phi_max, self.kp_bank * err))
        self.yaw = wrap_pi(self.yaw + (G * math.tan(bank) / self.Vh) * self.dt)

        sink = self.descent_rate * self.sink_bias
        self.n += (self.Vh * math.cos(self.yaw) + self.wind_n) * self.dt
        self.e += (self.Vh * math.sin(self.yaw) + self.wind_e) * self.dt
        self.d += sink * self.dt

        rn, re_ = reference_xy(self.g, 250)
        xte = float(np.min(np.hypot(rn - self.n, re_ - self.e))) if rn.size else 0.0
        self.trail_n.append(self.n); self.trail_e.append(self.e)
        self.alt_log.append(-self.d); self.xte_log.append(xte)
        if self.d >= self.p.land_d - 1e-3:
            self.landed = True

    def miss(self):
        return math.hypot(self.n - self.p.land_n, self.e - self.p.land_e)


# ----------------------------------------------------------------------------
def run_gui():
    p = default_params()
    sim = Sim(p)

    fig = plt.figure(figsize=(14.5, 8.6))
    fig.canvas.manager.set_window_title("RSX CanSat 2026 -- descent guidance (v2)")
    gs = fig.add_gridspec(2, 2, width_ratios=[1.55, 1.0],
                          left=0.055, right=0.985, top=0.95, bottom=0.40,
                          hspace=0.34, wspace=0.20)
    axP = fig.add_subplot(gs[:, 0]); axK = fig.add_subplot(gs[0, 1]); axX = fig.add_subplot(gs[1, 1])

    # persistent artists ------------------------------------------------------
    (ref_line,)   = axP.plot([], [], "--", color="0.6", lw=1.5, label="plan")
    (trail_line,) = axP.plot([], [], "-", color="#1f6feb", lw=2.0, label="flown")
    (carrot_line,) = axP.plot([], [], "-", color="#fb8500", lw=1.0, alpha=0.8)
    (glider_pt,)  = axP.plot([], [], "o", color="#1f6feb", ms=9, mec="white", mew=1.2, zorder=8)
    (replan_pts,) = axP.plot([], [], "o", color="#2da44e", ms=6, ls="none", zorder=7)
    (circle_line,) = axP.plot([], [], ":", color="0.8", lw=1.0)
    start_pt = axP.scatter([p.start_e], [p.start_n], c="green", s=60, zorder=6)
    land_pt = axP.scatter([p.land_e], [p.land_n], c="red", s=90, marker="*", zorder=6)
    land_arrow = axP.annotate("", xy=(0, 0), xytext=(0, 0),
                              arrowprops=dict(arrowstyle="->", color="red", lw=1.6))
    axP.set_xlabel("East (m)"); axP.set_ylabel("North (m)")
    axP.grid(alpha=0.3); axP.legend(loc="upper left", fontsize=8)
    axP.set_aspect("equal", adjustable="datalim")

    (kappa_line,) = axK.plot([], [], "-", color="#8250df", lw=1.7)
    axK.set_xlabel("arc length s (m)"); axK.set_ylabel("kappa (1/m)")
    axK.set_title("Curvature -- continuous => G2", fontsize=9); axK.grid(alpha=0.3)
    seg_spans = []

    (xte_line,) = axX.plot([], [], "-", color="#d1242f", lw=1.4)
    axX.set_xlabel("altitude AGL (m)"); axX.set_ylabel("cross-track (m)")
    axX.set_title("Tracking error", fontsize=9); axX.grid(alpha=0.3); axX.invert_xaxis()

    # sliders -----------------------------------------------------------------
    def mkslider(x, y, w, lbl, lo, hi, val, fmt="%.0f"):
        ax = fig.add_axes([x, y, w, 0.022]); return Slider(ax, lbl, lo, hi, valinit=val, valfmt=fmt)

    cL, cR = 0.075, 0.575; w = 0.33
    s_home = mkslider(cL, 0.31, w, "homing hdg (deg)", 0, 359, math.degrees(p.start_heading))
    s_land = mkslider(cL, 0.275, w, "land hdg (deg)", 0, 359, math.degrees(p.land_heading))
    s_R    = mkslider(cL, 0.24, w, "loiter R (m)", 20, 80, p.loiter_radius)
    s_gr   = mkslider(cL, 0.205, w, "glide ratio", 2.0, 6.0, p.glide_ratio, "%.2f")
    s_appr = mkslider(cL, 0.17, w, "approach len (m)", 25, 120, p.approach_len)
    s_tr   = mkslider(cL, 0.135, w, "transition Lc (m)", 4, 30, p.transition_len)
    s_dir  = mkslider(cL, 0.10, w, "loiter dir (-1/+1)", -1, 1, p.loiter_dir, "%+.0f")

    s_look = mkslider(cR, 0.31, w, "lookahead drop (m)", 2, 25, p.lookahead_drop)
    s_alt  = mkslider(cR, 0.275, w, "start alt (m)", 80, 400, -p.start_d)
    s_sink = mkslider(cR, 0.24, w, "sink bias x", 0.8, 1.4, 1.0, "%.2f")
    s_rep  = mkslider(cR, 0.205, w, "auto-replan every (m)", 0, 60, 0.0)
    s_spd  = mkslider(cR, 0.17, w, "sim speed (x)", 0.5, 6.0, 2.0, "%.1f")

    # buttons -----------------------------------------------------------------
    b_play  = Button(fig.add_axes([cR, 0.105, 0.10, 0.045]), "Play")
    b_reset = Button(fig.add_axes([cR + 0.115, 0.105, 0.10, 0.045]), "Reset")
    b_rep   = Button(fig.add_axes([cR + 0.23, 0.105, 0.10, 0.045]), "Replan now")

    fig.text(cL, 0.055, "Drag the glider (blue dot) with the mouse to inject a gust / external push, "
                        "then Replan to re-close from the new state.", fontsize=8.5, color="0.3")

    state = {"running": False, "dragging": False}

    # ---- drawing ----
    def redraw_plan():
        for s in seg_spans:
            s.remove()
        seg_spans.clear()
        rn, re_ = reference_xy(sim.g)
        ref_line.set_data(re_, rn)
        ss, ks = curvature_xy(sim.g)
        kappa_line.set_data(ss, ks)
        if ss.size:
            axK.set_xlim(0, ss[-1]); axK.set_ylim(min(-0.001, ks.min() * 1.2), max(0.001, ks.max() * 1.3))
            for i in range(sim.g.seg_count()):
                s0 = sim.g.seg_s0(i); ln = sim.g.seg_len(i)
                seg_spans.append(axK.axvspan(s0, s0 + ln, color=SEGCOL.get(int(sim.g.seg_type(i)), "white"), alpha=0.5))
        c = sim.g.center(); th = np.linspace(0, 2 * math.pi, 90)
        circle_line.set_data(c.e + sim.p.loiter_radius * np.sin(th),
                             c.n + sim.p.loiter_radius * np.cos(th))
        start_pt.set_offsets([[sim.p.start_e, sim.p.start_n]])
        land_pt.set_offsets([[sim.p.land_e, sim.p.land_n]])
        dl = 22.0
        land_arrow.xy = (sim.p.land_e + dl * math.sin(sim.p.land_heading),
                         sim.p.land_n + dl * math.cos(sim.p.land_heading))
        land_arrow.set_position((sim.p.land_e, sim.p.land_n))
        axP.relim(); axP.autoscale_view()

    def redraw_state():
        trail_line.set_data(sim.trail_e, sim.trail_n)
        glider_pt.set_data([sim.e], [sim.n])
        if sim.cmd is not None:
            carrot_line.set_data([sim.e, sim.cmd.carrot.e], [sim.n, sim.cmd.carrot.n])
        if sim.replan_marks:
            re_, rn = zip(*sim.replan_marks)
            replan_pts.set_data(re_, rn)
        else:
            replan_pts.set_data([], [])
        if sim.alt_log:
            xte_line.set_data(sim.alt_log, sim.xte_log)
            axX.set_xlim(max(sim.alt_log), 0); axX.set_ylim(0, max(2.0, max(sim.xte_log) * 1.15))
        ph = PHASE.get(int(sim.cmd.phase), "-") if sim.cmd else "-"
        sname = str(sim.status).split('.')[-1]
        title = f"status={sname}"
        if sim.feasible():
            title += (f"  |  {ph}  |  alt {-sim.d:.0f} m  |  loiter {sim.g.loiter_turns():.2f}t"
                      f"  |  resid {sim.g.budget_residual():+.0f} m")
            if sim.landed:
                title += f"  |  TOUCHDOWN miss {sim.miss():.1f} m"
        else:
            title += "  --  homing heading can't reach the loiter (try ~60-74 deg)"
        axP.set_title(title, fontsize=9)
        fig.canvas.draw_idle()

    # ---- timer loop ----
    timer = fig.canvas.new_timer(interval=33)

    def on_tick():
        if not state["running"] or state["dragging"]:
            return
        steps = max(1, int(round(s_spd.val * 2)))
        for _ in range(steps):
            sim.step(auto_replan_m=s_rep.val)
            if sim.landed:
                break
        redraw_state()
        if sim.landed:
            state["running"] = False
            b_play.label.set_text("Play")
    timer.add_callback(on_tick)
    timer.start()

    # ---- controls ----
    def apply_params(_=None):
        p.start_heading = math.radians(s_home.val)
        p.land_heading  = math.radians(s_land.val)
        p.loiter_radius = s_R.val
        p.glide_ratio   = s_gr.val
        p.approach_len  = s_appr.val
        p.transition_len = s_tr.val
        p.loiter_dir    = +1 if s_dir.val >= 0 else -1
        p.lookahead_drop = s_look.val
        p.start_d       = -s_alt.val
        p.entry_clothoid_max = 2.0 * p.loiter_radius
        sim.sink_bias = s_sink.val
        sim.replan_params()
        redraw_plan(); redraw_state()

    for s in (s_home, s_land, s_R, s_gr, s_appr, s_tr, s_dir, s_look, s_alt, s_sink):
        s.on_changed(apply_params)

    def on_play(_):
        state["running"] = not state["running"]
        b_play.label.set_text("Pause" if state["running"] else "Play")
    b_play.on_clicked(on_play)

    def on_reset(_):
        sim.reset_state(); state["running"] = False; b_play.label.set_text("Play")
        redraw_state()
    b_reset.on_clicked(on_reset)

    def on_replan(_):
        sim.replan_now(); redraw_plan(); redraw_state()
    b_rep.on_clicked(on_replan)

    # ---- mouse drag to perturb ----
    def near_glider(ev):
        if ev.inaxes is not axP or ev.xdata is None:
            return False
        return math.hypot(ev.xdata - sim.e, ev.ydata - sim.n) < max(6.0, sim.p.loiter_radius * 0.18)

    def on_press(ev):
        if near_glider(ev):
            state["dragging"] = True
    def on_motion(ev):
        if state["dragging"] and ev.inaxes is axP and ev.xdata is not None:
            sim.e, sim.n = ev.xdata, ev.ydata
            sim.trail_n.append(sim.n); sim.trail_e.append(sim.e)
            redraw_state()
    def on_release(ev):
        state["dragging"] = False
    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("motion_notify_event", on_motion)
    fig.canvas.mpl_connect("button_release_event", on_release)

    redraw_plan(); redraw_state()
    plt.show()


def save_static():
    """Fallback when no interactive backend: fly clean and save a PNG."""
    p = default_params(); sim = Sim(p)
    while not sim.landed and sim.feasible():
        sim.step()
    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.55, 1.0], left=0.06, right=0.98,
                          top=0.93, bottom=0.08, hspace=0.3, wspace=0.2)
    axP = fig.add_subplot(gs[:, 0]); axK = fig.add_subplot(gs[0, 1]); axX = fig.add_subplot(gs[1, 1])
    rn, re_ = reference_xy(sim.g); axP.plot(re_, rn, "--", color="0.6", lw=1.5, label="plan")
    axP.plot(sim.trail_e, sim.trail_n, "-", color="#1f6feb", lw=2, label="flown")
    axP.scatter([p.start_e], [p.start_n], c="green", s=60)
    axP.scatter([p.land_e], [p.land_n], c="red", s=90, marker="*")
    axP.set_aspect("equal"); axP.grid(alpha=0.3); axP.legend(fontsize=8)
    axP.set_title(f"miss {sim.miss():.1f} m  (no interactive backend -- install python3-tk)", fontsize=9)
    ss, ks = curvature_xy(sim.g); axK.plot(ss, ks, color="#8250df"); axK.grid(alpha=0.3)
    axK.set_title("Curvature (G2)", fontsize=9)
    axX.plot(sim.alt_log, sim.xte_log, color="#d1242f"); axX.invert_xaxis(); axX.grid(alpha=0.3)
    axX.set_title("Cross-track", fontsize=9)
    fig.savefig("path_plot_v2.png", dpi=130); print("saved path_plot_v2.png")


if __name__ == "__main__":
    if not INTERACTIVE:
        print("No interactive matplotlib backend found (window can't open here).")
        print("  Linux:  sudo apt-get install python3-tk     then re-run  python3 sim.py")
        print("  macOS/conda: usually works out of the box; else  pip install pyqt5")
        print("  (saving a static figure as a fallback)")
        save_static()
    else:
        print(f"interactive backend: {matplotlib.get_backend()}  --  opening window")
        run_gui()
