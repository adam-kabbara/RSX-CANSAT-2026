#!/usr/bin/env python3
"""
sim.py -- RSX CanSat 2026 descent-guidance interactive simulator (v1 vs v2)

Real-time, side-by-side comparison of the two guidance modes, both running on
the same C++ class (pathguidance pybind11 module):

  v1 (legacy)  LINE -> ARC -> LINE       homing heading DERIVED, sharp corners
               (curvature steps 0<->1/R, G1), continuous sweep -> exact budget.
  v2           LINE -> CLOTHOID -> ARC -> CLOTHOID -> LINE   homing heading INPUT,
               curvature-continuous (G2), loop-quantised budget + approach flex.

Controls:
  * Play / Pause / Reset    -- fly both forward in lockstep, watch them track.
  * sliders                 -- change every initial value live (re-plans both).
  * Replan now / auto-replan-- receding-horizon re-close (measured glide ratio).
  * DRAG either glider dot  -- injects the SAME external push/gust into BOTH,
                               so you can compare how each recovers (then Replan).

The curvature panel overlays both modes -- the step (v1) vs ramp (v2) is the
whole point. The cross-track panel overlays both flown errors.

Run:  python3 sim.py     (needs an interactive backend; see README if no window)
"""
import math
import numpy as np

import matplotlib
import matplotlib.pyplot as plt
INTERACTIVE = False
for _bk in ("TkAgg", "QtAgg", "Qt5Agg", "MacOSX", "GTK3Agg"):
    try:
        plt.switch_backend(_bk)
        _f = plt.figure(); plt.close(_f)
        INTERACTIVE = True
        break
    except Exception:
        continue
if not INTERACTIVE:
    plt.switch_backend("Agg")
from matplotlib.widgets import Slider, Button       # noqa: E402
import pathguidance as pg                            # noqa: E402

G = 9.81
PHASE = {0: "Init", 1: "Homing", 2: "Loiter", 3: "Approach", 4: "Landed"}
SEGCOL = {0: "#eef3ff", 1: "#fff3e6", 2: "#eafbef"}
COL_V1, COL_V2 = "#d1242f", "#1f6feb"


def build_params(v, legacy):
    p = pg.GuidanceParams()
    p.start_n, p.start_e, p.start_d = 0.0, 0.0, -v["alt"]
    p.land_n, p.land_e, p.land_d = 60.0, 90.0, 0.0
    p.start_heading = math.radians(v["home"])
    p.land_heading  = math.radians(v["land"])
    p.glide_ratio   = v["gr"]
    p.approach_len  = v["appr"]
    p.approach_len_min = 25.0
    p.loiter_radius = v["R"]
    p.min_turn_radius = 25.0
    p.transition_len = v["tr"]
    p.entry_clothoid_max = 2.0 * v["R"]
    p.loiter_dir = +1 if v["dir"] >= 0 else -1
    p.lookahead_drop = v["look"]
    p.legacy_v1 = legacy
    return p


def default_vals():
    return dict(home=64.0, land=30.0, R=40.0, gr=3.0, appr=60.0, tr=12.0,
                dir=1.0, look=11.0, alt=200.0, sink=1.0)


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def reference_xy(g, npts=550):
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
    """One guidance mode; headless-testable. step() advances the glider one dt."""
    def __init__(self, p, descent_rate=5.0, dt=0.05, kp_bank=2.2):
        self.dt, self.kp_bank, self.descent_rate = dt, kp_bank, descent_rate
        self.sink_bias = 1.0
        self.set_params(p)

    def set_params(self, p):
        self.p = p
        self.g = pg.PathGuidance(p)
        self.status = self.g.plan()
        self.reset_state()

    def feasible(self):
        return str(self.status).split('.')[-1] in ("Ok", "AdjustedApproach")

    def reset_state(self):
        self.n, self.e, self.d = self.p.start_n, self.p.start_e, self.p.start_d
        self.yaw = self.p.start_heading
        self.Vh = self.p.glide_ratio * self.descent_rate
        self.phi_max = math.atan(self.Vh * self.Vh / (G * self.p.min_turn_radius))
        self.trail_n, self.trail_e = [self.n], [self.e]
        self.alt_log, self.xte_log, self.replan_marks = [], [], []
        self.landed, self.last_replan_d, self.cmd = False, self.d, None

    def _state(self):
        st = pg.State()
        st.n, st.e, st.d = self.n, self.e, self.d
        st.vn = self.Vh * math.cos(self.yaw); st.ve = self.Vh * math.sin(self.yaw)
        st.vd = self.descent_rate * self.sink_bias; st.yaw = self.yaw
        return st

    def replan_now(self):
        if self.landed or not self.feasible():
            return None
        r = self.g.replan(self._state())
        if str(r).split('.')[-1] in ("Ok", "AdjustedApproach"):
            self.replan_marks.append((self.e, self.n))
        return r

    def step(self, auto_replan_m=0.0):
        if self.landed or not self.feasible():
            return
        self.cmd = self.g.get_heading(self._state())
        if (auto_replan_m > 0.0 and int(self.cmd.phase) == 2
                and (self.d - self.last_replan_d) >= auto_replan_m):
            self.replan_now(); self.last_replan_d = self.d
            self.cmd = self.g.get_heading(self._state())
        err = wrap_pi(self.cmd.heading - self.yaw)
        bank = max(-self.phi_max, min(self.phi_max, self.kp_bank * err))
        self.yaw = wrap_pi(self.yaw + (G * math.tan(bank) / self.Vh) * self.dt)
        self.n += self.Vh * math.cos(self.yaw) * self.dt
        self.e += self.Vh * math.sin(self.yaw) * self.dt
        self.d += self.descent_rate * self.sink_bias * self.dt
        rn, re_ = reference_xy(self.g, 220)
        xte = float(np.min(np.hypot(rn - self.n, re_ - self.e))) if rn.size else 0.0
        self.trail_n.append(self.n); self.trail_e.append(self.e)
        self.alt_log.append(-self.d); self.xte_log.append(xte)
        if self.d >= self.p.land_d - 1e-3:
            self.landed = True

    def miss(self):
        return math.hypot(self.n - self.p.land_n, self.e - self.p.land_e)


def make_map_artists(ax, color, label):
    a = {}
    (a["ref"],)    = ax.plot([], [], "--", color="0.6", lw=1.4)
    (a["trail"],)  = ax.plot([], [], "-", color=color, lw=2.0, label=label)
    (a["carrot"],) = ax.plot([], [], "-", color="#fb8500", lw=1.0, alpha=0.8)
    (a["circle"],) = ax.plot([], [], ":", color="0.8", lw=1.0)
    (a["glider"],) = ax.plot([], [], "o", color=color, ms=10, mec="white", mew=1.3, zorder=9)
    (a["replan"],) = ax.plot([], [], "o", color="#2da44e", ms=5, ls="none", zorder=7)
    ax.scatter([0], [0], c="green", s=55, zorder=6)
    ax.scatter([90], [60], c="red", s=90, marker="*", zorder=6)
    a["arrow"] = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                             arrowprops=dict(arrowstyle="->", color="red", lw=1.5))
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.grid(alpha=0.3); ax.set_aspect("equal", adjustable="box")
    return a


def run_gui():
    vals = default_vals()
    simA = Sim(build_params(vals, legacy=True))    # v1
    simB = Sim(build_params(vals, legacy=False))   # v2

    fig = plt.figure(figsize=(16, 9))
    fig.canvas.manager.set_window_title("RSX CanSat 2026 -- v1 vs v2 descent guidance")
    gs = fig.add_gridspec(2, 3, width_ratios=[1.0, 1.0, 0.95],
                          left=0.05, right=0.985, top=0.95, bottom=0.34,
                          hspace=0.32, wspace=0.26)
    axA = fig.add_subplot(gs[:, 0]); axB = fig.add_subplot(gs[:, 1], sharex=axA, sharey=axA)
    axK = fig.add_subplot(gs[0, 2]); axX = fig.add_subplot(gs[1, 2])
    artA = make_map_artists(axA, COL_V1, "v1 flown")
    artB = make_map_artists(axB, COL_V2, "v2 flown")

    (kA,) = axK.plot([], [], "-", color=COL_V1, lw=1.7, label="v1 (steps)")
    (kB,) = axK.plot([], [], "-", color=COL_V2, lw=1.7, label="v2 (smooth)")
    axK.set_xlabel("arc length s (m)"); axK.set_ylabel("kappa (1/m)")
    axK.set_title("Curvature: v1 steps vs v2 ramps (G2)", fontsize=9)
    axK.grid(alpha=0.3); axK.legend(fontsize=7, loc="lower center")

    (xA,) = axX.plot([], [], "-", color=COL_V1, lw=1.4, label="v1")
    (xB,) = axX.plot([], [], "-", color=COL_V2, lw=1.4, label="v2")
    axX.set_xlabel("altitude AGL (m)"); axX.set_ylabel("cross-track (m)")
    axX.set_title("Tracking error", fontsize=9); axX.grid(alpha=0.3)
    axX.invert_xaxis(); axX.legend(fontsize=7)

    # sliders -----------------------------------------------------------------
    def mkslider(x, y, w, lbl, lo, hi, val, fmt="%.0f"):
        return Slider(fig.add_axes([x, y, w, 0.02]), lbl, lo, hi, valinit=val, valfmt=fmt)
    cL, cM, cR = 0.06, 0.40, 0.74; w = 0.22
    s = {}
    s["home"] = mkslider(cL, 0.265, w, "homing hdg (deg) [v2]", 0, 359, vals["home"])
    s["land"] = mkslider(cL, 0.230, w, "land hdg (deg)", 0, 359, vals["land"])
    s["R"]    = mkslider(cL, 0.195, w, "loiter R (m)", 20, 80, vals["R"])
    s["gr"]   = mkslider(cL, 0.160, w, "glide ratio", 2.0, 6.0, vals["gr"], "%.2f")
    s["appr"] = mkslider(cM, 0.265, w, "approach len (m)", 25, 120, vals["appr"])
    s["tr"]   = mkslider(cM, 0.230, w, "transition Lc (m) [v2]", 4, 30, vals["tr"])
    s["dir"]  = mkslider(cM, 0.195, w, "loiter dir (-1/+1)", -1, 1, vals["dir"], "%+.0f")
    s["look"] = mkslider(cM, 0.160, w, "lookahead drop (m)", 2, 25, vals["look"])
    s["alt"]  = mkslider(cR, 0.265, w, "start alt (m)", 80, 400, vals["alt"])
    s["sink"] = mkslider(cR, 0.230, w, "sink bias x", 0.8, 1.4, vals["sink"], "%.2f")
    s_rep = mkslider(cR, 0.195, w, "auto-replan every (m)", 0, 60, 0.0)
    s_spd = mkslider(cR, 0.160, w, "sim speed (x)", 0.5, 6.0, 2.0, "%.1f")

    b_play  = Button(fig.add_axes([cL, 0.085, 0.10, 0.05]), "Play")
    b_reset = Button(fig.add_axes([cL + 0.12, 0.085, 0.10, 0.05]), "Reset")
    b_rep   = Button(fig.add_axes([cL + 0.24, 0.085, 0.12, 0.05]), "Replan now")
    fig.text(cL, 0.04, "Drag either glider dot to inject the SAME gust into both modes, "
                       "then 'Replan now' to re-close from the displaced state.",
             fontsize=9, color="0.3")

    st = {"run": False, "drag": False, "graspA": None, "graspB": None, "anchor": None}

    def redraw_plan():
        for ax, art, sim in ((axA, artA, simA), (axB, artB, simB)):
            rn, re_ = reference_xy(sim.g)
            art["ref"].set_data(re_, rn)
            c = sim.g.center(); th = np.linspace(0, 2 * math.pi, 90)
            art["circle"].set_data(c.e + sim.p.loiter_radius * np.sin(th),
                                   c.n + sim.p.loiter_radius * np.cos(th))
            dl = 22.0
            art["arrow"].xy = (90 + dl * math.sin(sim.p.land_heading),
                               60 + dl * math.cos(sim.p.land_heading))
            art["arrow"].set_position((90, 60))
        ssA, ksA = curvature_xy(simA.g); kA.set_data(ssA, ksA)
        ssB, ksB = curvature_xy(simB.g); kB.set_data(ssB, ksB)
        mx = max(ssA[-1] if ssA.size else 1, ssB[-1] if ssB.size else 1)
        ks_all = np.concatenate([k for k in (ksA, ksB) if k.size]) if (ksA.size or ksB.size) else np.array([0, 0.025])
        axK.set_xlim(0, mx); axK.set_ylim(min(-0.001, ks_all.min() * 1.2), max(0.001, ks_all.max() * 1.3))
        axA.relim(); axA.autoscale_view()

    def redraw_state():
        miss_txt = {}
        for ax, art, sim, name in ((axA, artA, simA, "v1"), (axB, artB, simB, "v2")):
            art["trail"].set_data(sim.trail_e, sim.trail_n)
            art["glider"].set_data([sim.e], [sim.n])
            if sim.cmd is not None:
                art["carrot"].set_data([sim.e, sim.cmd.carrot.e], [sim.n, sim.cmd.carrot.n])
            if sim.replan_marks:
                re_, rn = zip(*sim.replan_marks); art["replan"].set_data(re_, rn)
            else:
                art["replan"].set_data([], [])
            ph = PHASE.get(int(sim.cmd.phase), "-") if sim.cmd else "-"
            sname = str(sim.status).split('.')[-1]
            t = f"{name}: {sname}"
            if sim.feasible():
                t += f" | {ph} | {sim.g.loiter_turns():.2f}t | resid {sim.g.budget_residual():+.0f}m"
                if sim.landed:
                    t += f" | MISS {sim.miss():.1f}m"
            else:
                t += "  (homing hdg infeasible -- v2 needs ~60-74 deg)"
            ax.set_title(t, fontsize=8.5)
        xA.set_data(simA.alt_log, simA.xte_log); xB.set_data(simB.alt_log, simB.xte_log)
        allx = simA.xte_log + simB.xte_log
        if allx:
            top = max(simA.alt_log + simB.alt_log)
            axX.set_xlim(top, 0); axX.set_ylim(0, max(2.0, max(allx) * 1.15))
        # Force an immediate draw — some backends don't flush draw_idle()
        try:
            fig.canvas.draw()
        except Exception:
            fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=33)

    def on_tick():
        if not st["run"] or st["drag"]:
            return
        for _ in range(max(1, int(round(s_spd.val * 2)))):
            simA.step(s_rep.val); simB.step(s_rep.val)
            if simA.landed and simB.landed:
                break
        redraw_state()
        if simA.landed and simB.landed:
            st["run"] = False; b_play.label.set_text("Play")
    timer.add_callback(on_tick); timer.start()

    def apply(_=None):
        v = {k: s[k].val for k in s}
        v["sink"] = s["sink"].val
        simA.sink_bias = simB.sink_bias = v["sink"]
        simA.set_params(build_params(v, True))
        simB.set_params(build_params(v, False))
        st["run"] = False; b_play.label.set_text("Play")
        redraw_plan(); redraw_state()
    for k in s:
        s[k].on_changed(apply)

    def on_play(_):
        st["run"] = not st["run"]; b_play.label.set_text("Pause" if st["run"] else "Play")
    b_play.on_clicked(on_play)

    def on_reset(_):
        simA.reset_state(); simB.reset_state()
        st["run"] = False; b_play.label.set_text("Play"); redraw_state()
    b_reset.on_clicked(on_reset)

    def on_replan(_):
        simA.replan_now(); simB.replan_now(); redraw_plan(); redraw_state()
    b_rep.on_clicked(on_replan)

    # ---- drag: same gust into both ----
    def hit(ev, sim):
        return (ev.xdata is not None
                and math.hypot(ev.xdata - sim.e, ev.ydata - sim.n) < max(7.0, sim.p.loiter_radius * 0.2))

    def on_press(ev):
        if ev.inaxes in (axA, axB) and (hit(ev, simA) or hit(ev, simB)):
            st["drag"] = True
            st["graspA"] = (simA.e, simA.n); st["graspB"] = (simB.e, simB.n)
            st["anchor"] = (ev.xdata, ev.ydata)

    def on_motion(ev):
        if st["drag"] and ev.inaxes in (axA, axB) and ev.xdata is not None:
            de = ev.xdata - st["anchor"][0]; dn = ev.ydata - st["anchor"][1]
            simA.e, simA.n = st["graspA"][0] + de, st["graspA"][1] + dn
            simB.e, simB.n = st["graspB"][0] + de, st["graspB"][1] + dn
            for sim in (simA, simB):
                sim.trail_e.append(sim.e); sim.trail_n.append(sim.n)
            redraw_state()

    def on_release(ev):
        st["drag"] = False
    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("motion_notify_event", on_motion)
    fig.canvas.mpl_connect("button_release_event", on_release)

    redraw_plan(); redraw_state()
    # Ensure the initial frame is shown
    try:
        plt.pause(0.001)
    except Exception:
        pass
    plt.show()


def save_static():
    vals = default_vals()
    simA = Sim(build_params(vals, True)); simB = Sim(build_params(vals, False))
    for sim in (simA, simB):
        while not sim.landed and sim.feasible():
            sim.step()
    fig = plt.figure(figsize=(16, 8))
    gs = fig.add_gridspec(2, 3, width_ratios=[1, 1, 0.95], left=0.05, right=0.985,
                          top=0.92, bottom=0.08, hspace=0.3, wspace=0.26)
    axA = fig.add_subplot(gs[:, 0]); axB = fig.add_subplot(gs[:, 1]); axK = fig.add_subplot(gs[0, 2]); axX = fig.add_subplot(gs[1, 2])
    for ax, sim, col, name in ((axA, simA, COL_V1, "v1 (Line-Arc-Line, G1)"),
                               (axB, simB, COL_V2, "v2 (clothoid, G2)")):
        rn, re_ = reference_xy(sim.g); ax.plot(re_, rn, "--", color="0.6", lw=1.4)
        ax.plot(sim.trail_e, sim.trail_n, "-", color=col, lw=2)
        ax.scatter([0], [0], c="green", s=55); ax.scatter([90], [60], c="red", s=90, marker="*")
        ax.set_aspect("equal"); ax.grid(alpha=0.3)
        ax.set_title(f"{name}  |  miss {sim.miss():.1f} m", fontsize=9)
    for sim, col, lbl in ((simA, COL_V1, "v1"), (simB, COL_V2, "v2")):
        ss, ks = curvature_xy(sim.g); axK.plot(ss, ks, color=col, label=lbl)
        axX.plot(sim.alt_log, sim.xte_log, color=col, label=lbl)
    axK.set_title("Curvature: v1 steps vs v2 ramps", fontsize=9); axK.grid(alpha=0.3); axK.legend(fontsize=8)
    axX.invert_xaxis(); axX.grid(alpha=0.3); axX.legend(fontsize=8); axX.set_title("Cross-track", fontsize=9)
    fig.suptitle("RSX CanSat 2026 -- v1 vs v2 (static; install python3-tk for the interactive window)", fontsize=11)
    fig.savefig("path_plot_v2.png", dpi=120); print("saved path_plot_v2.png")


if __name__ == "__main__":
    if not INTERACTIVE:
        print("No interactive matplotlib backend found (window can't open here).")
        print("  Linux:  sudo apt-get install python3-tk     then re-run  python3 sim.py")
        print("  macOS/conda: usually fine; else  pip install pyqt5")
        save_static()
    else:
        print(f"interactive backend: {matplotlib.get_backend()}  --  opening window")
        run_gui()
