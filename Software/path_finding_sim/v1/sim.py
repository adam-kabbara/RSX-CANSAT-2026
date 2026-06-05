#!/usr/bin/env python3
"""
sim.py -- RSX CanSat 2026 descent-guidance debug front-end.

Closes the loop on the *real* C++ guidance (via the pathguidance pybind11
module): feed simulated EKF state -> get_heading() -> integrate a simple
coordinated-turn glider model -> repeat. Plots the planned reference path
against the flown path with cross-track error, so geometry + carrot-tuning
bugs show up visually.

Build the module first (see README.md), then:  python3 sim.py
"""
import math
import numpy as np
import matplotlib.pyplot as plt
import pathguidance as pg

G = 9.81  # m/s^2


def make_params():
    p = pg.GuidanceParams()
    p.start_n, p.start_e, p.start_d = 0.0, 0.0, -500.0     # 500 m AGL
    p.land_n,  p.land_e,  p.land_d  = 60.0, 90.0, 0.0
    p.land_heading   = math.radians(30)                     # into-wind final approach
    p.glide_ratio    = 3.0
    p.approach_len    = 60.0 #phase 1
    p.loiter_radius   = 40.0 #phase 2
    p.min_turn_radius = 25.0 
    p.loiter_dir      = -1 # +1 = CW, -1 = CCW
    p.lookahead_drop  = 8.0 # carrot
    return p


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def simulate(p, descent_rate=5.0, dt=0.05, kp_bank=2.0, kp_track=1.0):
    """Coordinated-turn kinematic glider. descent_rate = vertical speed (m/s)."""
    g = pg.PathGuidance(p)
    status = g.plan()

    Vh = p.glide_ratio * descent_rate                     # horizontal airspeed
    phi_max = math.atan(Vh * Vh / (G * p.min_turn_radius))  # bank that gives min radius

    # initial state: at start, pointed along the homing heading, descending
    n, e, d = p.start_n, p.start_e, p.start_d
    yaw = g.homing_heading()
    log = {"n": [], "e": [], "d": [], "yaw": [], "phase": [],
           "cn": [], "ce": [], "head_cmd": [], "xte": []}

    steps = 0
    max_steps = int(400.0 / dt)
    while d < p.land_d - 1e-3 and steps < max_steps:
        st = pg.State()
        st.n, st.e, st.d = n, e, d
        st.vn, st.ve, st.vd = Vh * math.cos(yaw), Vh * math.sin(yaw), descent_rate
        st.yaw = yaw
        cmd = g.get_heading(st)

        err = wrap_pi(cmd.heading - yaw)
        bank = max(-phi_max, min(phi_max, kp_bank * err))
        yaw_rate = G * math.tan(bank) / Vh                 # coordinated turn
        yaw = wrap_pi(yaw + yaw_rate * dt)

        # cross-track error vs reference point at this altitude (for the plot)
        ref = g.path_at(d)
        xte = math.hypot(n - ref.n, e - ref.e)

        log["n"].append(n); log["e"].append(e); log["d"].append(d)
        log["yaw"].append(yaw); log["phase"].append(int(cmd.phase))
        log["cn"].append(cmd.carrot.n); log["ce"].append(cmd.carrot.e)
        log["head_cmd"].append(cmd.heading); log["xte"].append(xte)

        n += Vh * math.cos(yaw) * dt
        e += Vh * math.sin(yaw) * dt
        d += descent_rate * dt
        steps += 1

    return g, status, {k: np.array(v) for k, v in log.items()}, Vh, phi_max


def reference_path(g, p, npts=600):
    ds = np.linspace(p.start_d, p.land_d, npts)
    pts = [g.path_at(float(dd)) for dd in ds]
    return np.array([q.n for q in pts]), np.array([q.e for q in pts]), ds


def plot(g, status, log, p, Vh, phi_max):
    rn, re, rd = reference_path(g, p)
    pt2, pt3, c = g.entry(), g.exit(), g.center()

    fig = plt.figure(figsize=(13, 6.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.4, 1.0])

    # ---- top-down (East = x, North = y) ----
    ax = fig.add_subplot(gs[:, 0])
    ax.plot(re, rn, "--", color="0.6", lw=1.6, label="reference path")
    ax.plot(log["e"], log["n"], "-", color="#1f6feb", lw=2.0, label="flown path")
    th = np.linspace(0, 2 * math.pi, 100)
    ax.plot(c.e + p.loiter_radius * np.sin(th), c.n + p.loiter_radius * np.cos(th),
            ":", color="0.75", lw=1.0)
    ax.scatter([p.start_e], [p.start_n], c="green", s=70, zorder=5, label="start (pt1)")
    ax.scatter([pt2.e], [pt2.n], c="orange", s=55, zorder=5, label="loiter entry (pt2)")
    ax.scatter([pt3.e], [pt3.n], c="purple", s=55, zorder=5, label="loiter exit (pt3)")
    ax.scatter([p.land_e], [p.land_n], c="red", s=80, marker="*", zorder=5, label="landing (pt4)")
    dl = 22.0
    ax.annotate("", xy=(p.land_e + dl * math.sin(p.land_heading),
                        p.land_n + dl * math.cos(p.land_heading)),
                xytext=(p.land_e, p.land_n),
                arrowprops=dict(arrowstyle="->", color="red", lw=1.5))
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.set_title(f"Top-down  |  status={str(status).split('.')[-1]}  |  "
                 f"loiter {g.loiter_turns():.2f} turns")
    ax.axis("equal"); ax.grid(alpha=0.3); ax.legend(loc="best", fontsize=8)

    # ---- altitude profile (altitude = -D) ----
    ax2 = fig.add_subplot(gs[0, 1])
    horiz = np.cumsum(np.hypot(np.diff(log["n"], prepend=log["n"][0]),
                               np.diff(log["e"], prepend=log["e"][0])))
    ax2.plot(horiz, -log["d"], "-", color="#1f6feb", lw=1.8)
    for dd, lbl, col in [(g.d_entry(), "pt2", "orange"), (g.d_exit(), "pt3", "purple")]:
        idx = int(np.argmin(np.abs(log["d"] - dd)))
        ax2.scatter([horiz[idx]], [-dd], c=col, s=40, zorder=5)
    ax2.set_xlabel("horizontal distance flown (m)"); ax2.set_ylabel("altitude AGL (m)")
    ax2.set_title("Altitude profile"); ax2.grid(alpha=0.3)

    # ---- cross-track error ----
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.plot(-log["d"], log["xte"], "-", color="#d1242f", lw=1.5)
    ax3.set_xlabel("altitude AGL (m)"); ax3.set_ylabel("cross-track err (m)")
    ax3.invert_xaxis()
    ax3.set_title(f"Tracking error  (Vh={Vh:.1f} m/s, "
                  f"bank_max={math.degrees(phi_max):.0f} deg)")
    ax3.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig("path_plot.png", dpi=130)
    print("saved path_plot.png")


if __name__ == "__main__":
    p = make_params()
    g, status, log, Vh, phi_max = simulate(p)
    print("status            :", status)
    print("loiter turns      : %.3f" % g.loiter_turns())
    print("resolved approach : %.1f m" % g.resolved_approach_len())
    print("touchdown miss    : %.2f m  (final cross-track %.2f m)"
          % (math.hypot(log["n"][-1] - p.land_n, log["e"][-1] - p.land_e), log["xte"][-1]))
    print("max cross-track   : %.2f m" % log["xte"].max())
    plot(g, status, log, p, Vh, phi_max)
