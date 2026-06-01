"""
Glider EKF — 3D Flight Path Visualizer
Simulates IMU + GPS data, runs a Python port of the C EKF,
and renders the estimated vs. ground-truth path live in 3D.

Ground-truth trajectory is driven by flight_plan.py logic:
  PHASE1_CRUISE → PHASE2_SPIRAL → TERMINAL → LANDING
Target position is set below; start altitude is FLIGHT_START_ALT_M.
"""

# ─────────────────────────────────────────────────────────────
# MISSION PARAMETERS  ← edit these
# ─────────────────────────────────────────────────────────────
# Target in local ENU (East/North metres from launch origin)
TARGET_EAST_M  =  400.0
TARGET_NORTH_M =  62

# Start offset from launch origin, metres ENU
START_EAST_M   =  364.0
START_NORTH_M  =  -62.0
FLIGHT_START_ALT_M = 200.0   # AGL at launch

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.gridspec import GridSpec

# ─────────────────────────────────────────────────────────────
# EKF CONSTANTS
# ─────────────────────────────────────────────────────────────
STATE_DIM = 15
G_ACCEL   = 9.81
DT_IMU    = 0.01    # 100 Hz
DT_GPS    = 1.0     # 1 Hz
DT_BARO   = 0.1     # 10 Hz

# ─────────────────────────────────────────────────────────────
# FLIGHT PLAN CONSTANTS  (mirrored from flight_plan.py)
# ─────────────────────────────────────────────────────────────
AIRSPEED          = 17.0
GLIDE_RATIO       = 6.0
V_DESCENT_CLEAN   = AIRSPEED / GLIDE_RATIO        # ~2.83 m/s
V_DESCENT_TARGET  = 5.0
EGG_DROP_ALT      = 2.0
SPIRAL_RADIUS     = 20.0
SPIRAL_ENTRY_DIST = 80.0
TP_ENTRY_ALT      = 30.0
GLIDE_RATIO_EFF   = AIRSPEED / V_DESCENT_TARGET   # 3.4
GLIDE_TOLERANCE   = 0.20

# ─────────────────────────────────────────────────────────────
# MATH HELPERS
# ─────────────────────────────────────────────────────────────
def quat_to_R(q):
    qw, qx, qy, qz = q
    return np.array([
        [1-2*(qy**2+qz**2),   2*(qx*qy-qw*qz),   2*(qx*qz+qw*qy)],
        [  2*(qx*qy+qw*qz), 1-2*(qx**2+qz**2),   2*(qy*qz-qw*qx)],
        [  2*(qx*qz-qw*qy),   2*(qy*qz+qw*qx), 1-2*(qx**2+qy**2)],
    ])

def quat_norm(q):
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else q

def wrap_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

# ─────────────────────────────────────────────────────────────
# PYTHON EKF  (unchanged)
# ─────────────────────────────────────────────────────────────
class GliderEKF:
    def __init__(self):
        self.x = np.zeros(11)
        self.q = np.array([1., 0., 0., 0.])
        self.P = np.eye(STATE_DIM) * 0.1
        self.Q = np.eye(STATE_DIM) * 0.01

    def predict(self, raw_accel, raw_gyro, dt):
        a = raw_accel - self.x[6:9]
        g = raw_gyro  - np.array([self.x[9], self.x[10], 0.0])
        R = quat_to_R(self.q)
        a_ned = R @ a;  a_ned[2] += G_ACCEL
        self.x[0:3] += self.x[3:6] * dt
        self.x[3:6] += a_ned * dt
        qw, qx, qy, qz = self.q;  gx, gy, gz = g
        self.q += 0.5*dt*np.array([
            -qx*gx-qy*gy-qz*gz, qw*gx-qz*gy+qy*gz,
             qz*gx+qw*gy-qx*gz,-qy*gx+qx*gy+qw*gz])
        self.q = quat_norm(self.q)
        F = np.eye(STATE_DIM)
        F[0,3]=dt; F[1,4]=dt; F[2,5]=dt
        F[3,7]= a_ned[2]*dt; F[3,8]=-a_ned[1]*dt
        F[4,6]=-a_ned[2]*dt; F[4,8]= a_ned[0]*dt
        F[5,6]= a_ned[1]*dt; F[5,7]=-a_ned[0]*dt
        F[3:6, 9:12] = -R * dt
        self.P = F @ self.P @ F.T + self.Q

    def _scalar_update(self, idx, innov, r):
        PH_T = self.P[:, idx]
        S = self.P[idx, idx] + r
        if abs(S) < 1e-12: return
        K = PH_T / S;  dx = K * innov
        self.x[0:3] += dx[0:3];  self.x[3:6] += dx[3:6]
        qw,qx,qy,qz = self.q;  ex,ey,ez = dx[6],dx[7],dx[8]
        self.q += 0.5*np.array([-qx*ex-qy*ey-qz*ez, qw*ex-qz*ey+qy*ez,
                                  qz*ex+qw*ey-qx*ez,-qy*ex+qx*ey+qw*ez])
        self.q = quat_norm(self.q)
        self.x[6:9]  += dx[9:12]
        self.x[9:11] += dx[12:14]
        self.P -= np.outer(K, self.P[idx, :])

    def update_baro(self, baro_alt, r=0.5):
        self._scalar_update(2, -baro_alt - self.x[2], r)

    def update_gps(self, pos_ne, vel_ned, r_pos=2.0, r_vel=0.1):
        self._scalar_update(0, pos_ne[0] - self.x[0], r_pos)
        self._scalar_update(1, pos_ne[1] - self.x[1], r_pos)
        self._scalar_update(3, vel_ned[0] - self.x[3], r_vel)
        self._scalar_update(4, vel_ned[1] - self.x[4], r_vel)
        self._scalar_update(5, vel_ned[2] - self.x[5], r_vel)

    def update_compass(self, yaw_rad, r=0.05):
        qw,qx,qy,qz = self.q
        pred = np.arctan2(2*(qw*qz+qx*qy), 1-2*(qy**2+qz**2))
        self._scalar_update(8, wrap_pi(yaw_rad - pred), r)

# ─────────────────────────────────────────────────────────────
# FLIGHT-PLAN DRIVEN GROUND-TRUTH SIMULATOR
# ─────────────────────────────────────────────────────────────
class FlightSim:
    """
    Integrates position using the velocity commands produced by the
    flight_plan.py planners (p1/p2/tp/lp) selected by classify_phase().

    Frame convention: internally uses ENU (x=East, y=North, z=Up/alt).
    Outputs are converted to NED for the EKF and the 3-D plot.

    Phase log is recorded so phases can be colour-coded in the plot.
    """

    def __init__(self):
        # ENU state
        self.x   = float(START_EAST_M)
        self.y   = float(START_NORTH_M)
        self.alt = float(FLIGHT_START_ALT_M)

        # ENU velocity (initialised toward target)
        dx = TARGET_EAST_M - self.x
        dy = TARGET_NORTH_M - self.y
        brg = math.atan2(dy, dx)
        self.vx = AIRSPEED * math.cos(brg)
        self.vy = AIRSPEED * math.sin(brg)
        self.vz = -V_DESCENT_TARGET

        self.t            = 0.0
        self.egg_released = False
        self.drop_heading = 0.0
        self.phase_name   = 'PHASE1_CRUISE'

        # Attitude quaternion (w,x,y,z) — starts level, heading = bearing to target
        self.q = self._euler_to_quat(0.0, 0.0, brg)
        self.prev_heading = brg

        # Sensor biases
        self.accel_bias = np.array([ 0.05, -0.03,  0.07])
        self.gyro_bias  = np.array([ 0.002, -0.001])

        # Phase log: list of (t, phase_name) for colour banding
        self.phase_log = []

    # ── Attitude helpers ──────────────────────────────────────
    @staticmethod
    def _euler_to_quat(roll, pitch, yaw):
        cr,sr = math.cos(roll/2),  math.sin(roll/2)
        cp,sp = math.cos(pitch/2), math.sin(pitch/2)
        cy,sy = math.cos(yaw/2),   math.sin(yaw/2)
        return np.array([cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                         cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy])

    # ── flight_plan.py planners (ENU output) ─────────────────
    def _bearing_to_target(self):
        return math.atan2(TARGET_NORTH_M - self.y, TARGET_EAST_M - self.x)

    def _dist_to_target(self):
        return math.hypot(TARGET_EAST_M - self.x, TARGET_NORTH_M - self.y)

    def _can_reach(self):
        dist_h      = self._dist_to_target()
        alt_to_lose = self.alt - EGG_DROP_ALT
        glide_range = alt_to_lose * GLIDE_RATIO_EFF
        return (glide_range >= dist_h*(1-GLIDE_TOLERANCE) and
                glide_range <= dist_h*(1+GLIDE_TOLERANCE))

    def _classify_phase(self):
        if self.egg_released:
            return 'LANDING'
        if self.alt <= TP_ENTRY_ALT and self._can_reach():
            return 'TERMINAL'
        if self._dist_to_target() <= SPIRAL_ENTRY_DIST:
            return 'PHASE2_SPIRAL'
        return 'PHASE1_CRUISE'

    def _velocity_command(self, phase):
        if phase == 'PHASE1_CRUISE':
            b = self._bearing_to_target()
            return AIRSPEED*math.cos(b), AIRSPEED*math.sin(b), -V_DESCENT_TARGET

        if phase == 'PHASE2_SPIRAL':
            dx = self.x - TARGET_EAST_M
            dy = self.y - TARGET_NORTH_M
            r  = max(math.hypot(dx, dy), 1e-3)
            r_hat = (dx/r, dy/r)
            t_hat = (-dy/r, dx/r)    # CCW orbit
            k = 0.6
            gain = max(-0.8, min(0.8, k*(r-SPIRAL_RADIUS)/SPIRAL_RADIUS))
            dir_x = t_hat[0] - gain*r_hat[0]
            dir_y = t_hat[1] - gain*r_hat[1]
            mag   = math.hypot(dir_x, dir_y)
            return AIRSPEED*dir_x/mag, AIRSPEED*dir_y/mag, -V_DESCENT_TARGET

        if phase == 'TERMINAL':
            b = self._bearing_to_target()
            return AIRSPEED*math.cos(b), AIRSPEED*math.sin(b), -V_DESCENT_CLEAN

        # LANDING
        return (AIRSPEED*math.cos(self.drop_heading),
                AIRSPEED*math.sin(self.drop_heading),
                -V_DESCENT_CLEAN)

    # ── Main step ─────────────────────────────────────────────
    def step(self, dt):
        self.t += dt

        # Egg release check
        if (not self.egg_released and
                self.alt <= EGG_DROP_ALT and
                self._dist_to_target() <= SPIRAL_RADIUS):
            self.egg_released = True
            self.drop_heading = math.atan2(self.vy, self.vx)

        phase = self._classify_phase()
        if phase != self.phase_name:
            self.phase_name = phase
            self.phase_log.append((self.t, phase))

        # Velocity command
        vx_cmd, vy_cmd, vz_cmd = self._velocity_command(phase)

        # Low-pass smooth actual velocity toward command (mimics airframe lag)
        tau = 0.5   # s time constant
        alpha = dt / (tau + dt)
        self.vx += alpha * (vx_cmd - self.vx)
        self.vy += alpha * (vy_cmd - self.vy)
        self.vz += alpha * (vz_cmd - self.vz)

        # Integrate position (ENU)
        self.x   += self.vx * dt
        self.y   += self.vy * dt
        self.alt  = max(0.0, self.alt + self.vz * dt)

        # ── Attitude from velocity direction ──────────────────
        horiz   = math.hypot(self.vx, self.vy) + 1e-6
        heading = math.atan2(self.vy, self.vx)   # ENU yaw
        pitch   = math.atan2(-self.vz, horiz)    # nose up when climbing

        # Spiral: add bank angle proportional to turn rate
        d_heading = wrap_pi(heading - self.prev_heading)
        turn_rate = d_heading / dt
        bank = math.atan2(AIRSPEED * turn_rate, G_ACCEL)
        bank = max(-math.radians(45), min(math.radians(45), bank))
        self.prev_heading = heading

        self.q = quat_norm(self._euler_to_quat(bank, pitch, heading))

        # ── Convert ENU pos/vel → NED for EKF/plot ───────────
        # NED: N=y(ENU), E=x(ENU), D=-alt(ENU)
        pos_ned = np.array([self.y, self.x, -self.alt])
        vel_ned = np.array([self.vy, self.vx, -self.vz])

        # ── True body-frame IMU readings ──────────────────────
        # Build NED rotation matrix from attitude
        R = quat_to_R(self.q)
        g_ned  = np.array([0., 0., G_ACCEL])
        # Kinematic NED accel (approximate: centripetal ignored)
        a_ned  = np.array([0., 0., 0.])      # glider in steady state
        a_specific_ned = a_ned - g_ned
        accel_body = R.T @ a_specific_ned

        # Angular rate in body frame
        omega_ned = np.array([0., pitch/max(dt, 1e-6)*0.0, turn_rate])
        omega_body = R.T @ np.array([0., 0., turn_rate])
        omega_body[0] = bank / max(dt, 0.01)   # rough roll rate

        yaw_ned = math.atan2(2*(self.q[0]*self.q[3]+self.q[1]*self.q[2]),
                             1-2*(self.q[2]**2+self.q[3]**2))

        return accel_body, omega_body, pos_ned, vel_ned, yaw_ned, phase

    # ── Noise wrappers ────────────────────────────────────────
    def noisy_imu(self, accel_true, gyro_true):
        a  = accel_true + self.accel_bias + np.random.normal(0, 0.02, 3)
        g  = gyro_true[:2] + self.gyro_bias + np.random.normal(0, 0.001, 2)
        return a, np.append(g, gyro_true[2] + np.random.normal(0, 0.001))

    def noisy_gps(self, pos_ne):
        return pos_ne + np.random.normal(0, 3.0, 2)

    def noisy_baro(self, alt):
        return alt + np.random.normal(0, 1.5)

    def noisy_compass(self, yaw):
        return yaw + np.random.normal(0, np.radians(2))

    def done(self):
        return self.egg_released and self.alt <= 0.0

# ─────────────────────────────────────────────────────────────
# SETUP & RUN SIMULATION
# ─────────────────────────────────────────────────────────────
GPS_EVERY  = int(DT_GPS  / DT_IMU)
BARO_EVERY = int(DT_BARO / DT_IMU)
MAX_SIM_T  = 600.0   # safety ceiling (s)

sim = FlightSim()
ekf = GliderEKF()

# Seed EKF at start position (NED)
ekf.x[0] = START_NORTH_M
ekf.x[1] = START_EAST_M
ekf.x[2] = -FLIGHT_START_ALT_M

gt_path   = []
est_path  = []
gps_pts   = []
t_hist    = []
alt_gt    = []
alt_est   = []
vel_gt    = []
vel_est   = []
phase_hist = []

step = 0
t    = 0.0

while not sim.done() and t < MAX_SIM_T:
    accel_true, gyro_true, pos_ned, vel_ned, yaw_true, phase = sim.step(DT_IMU)
    a_noisy, g_noisy = sim.noisy_imu(accel_true, gyro_true)

    ekf.predict(a_noisy, g_noisy, DT_IMU)

    if step % BARO_EVERY == 0:
        ekf.update_baro(sim.noisy_baro(sim.alt))
        ekf.update_compass(sim.noisy_compass(yaw_true))

    if step % GPS_EVERY == 0:
        gps_ne  = sim.noisy_gps(pos_ned[:2])
        vel_gps = vel_ned + np.random.normal(0, 0.2, 3)
        ekf.update_gps(gps_ne, vel_gps)
        gps_pts.append(np.array([gps_ne[0], gps_ne[1], pos_ned[2]]))

    gt_path.append(pos_ned.copy())
    est_path.append(np.array([ekf.x[0], ekf.x[1], ekf.x[2]]))
    t_hist.append(t)
    alt_gt.append(sim.alt)
    alt_est.append(-ekf.x[2])
    vel_gt.append(math.hypot(sim.vx, sim.vy))
    vel_est.append(np.linalg.norm(ekf.x[3:5]))
    phase_hist.append(phase)

    t    += DT_IMU
    step += 1

SIM_DURATION = t

gt_path   = np.array(gt_path)
est_path  = np.array(est_path)
gps_pts   = np.array(gps_pts)
t_hist    = np.array(t_hist)
phase_hist = np.array(phase_hist)

# ─────────────────────────────────────────────────────────────
# PHASE COLOUR MAP
# ─────────────────────────────────────────────────────────────
PHASE_COLORS = {
    'PHASE1_CRUISE': '#00e5ff',
    'PHASE2_SPIRAL': '#ffe066',
    'TERMINAL':      '#ff9944',
    'LANDING':       '#ff4488',
}

# ─────────────────────────────────────────────────────────────
# FIGURE
# ─────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(16, 9), facecolor='#0a0e1a')
fig.suptitle("Glider EKF — Flight Plan Simulation", 
             color='#e0e8ff', fontsize=15, fontweight='bold', y=0.97)

gs = GridSpec(2, 3, figure=fig, left=0.05, right=0.97,
              top=0.92, bottom=0.07, hspace=0.38, wspace=0.35)

ax3d   = fig.add_subplot(gs[:, :2], projection='3d')
ax_alt = fig.add_subplot(gs[0, 2])
ax_vel = fig.add_subplot(gs[1, 2])

for ax in [ax_alt, ax_vel]:
    ax.set_facecolor('#111827')
    ax.tick_params(colors='#8899bb', labelsize=8)
    for sp in ax.spines.values(): sp.set_color('#2a3550')
    ax.xaxis.label.set_color('#8899bb')
    ax.yaxis.label.set_color('#8899bb')
    ax.title.set_color('#c0d0ff')
    ax.grid(True, color='#1e2d45', linewidth=0.5)

ax3d.set_facecolor('#0d1220')
for pane in [ax3d.xaxis.pane, ax3d.yaxis.pane, ax3d.zaxis.pane]:
    pane.fill = False
    pane.set_edgecolor('#1e2d45')
ax3d.tick_params(colors='#8899bb', labelsize=7)
ax3d.set_xlabel('North (m)', color='#8899bb', labelpad=8)
ax3d.set_ylabel('East (m)',  color='#8899bb', labelpad=8)
ax3d.set_zlabel('Altitude (m)', color='#8899bb', labelpad=8)

# ── 3D ground truth coloured by phase ────────────────────────
phases_ordered = ['PHASE1_CRUISE','PHASE2_SPIRAL','TERMINAL','LANDING']
for ph in phases_ordered:
    mask = phase_hist == ph
    if not np.any(mask): continue
    ax3d.plot(gt_path[mask,0], gt_path[mask,1], -gt_path[mask,2],
              color=PHASE_COLORS[ph], lw=1.8, alpha=0.9,
              label=ph.replace('_',' ').title())

# EKF estimate
ax3d.plot(est_path[:,0], est_path[:,1], -est_path[:,2],
          color='#ff6b35', lw=1.1, alpha=0.75, linestyle='--', label='EKF Estimate')

# GPS fixes
if len(gps_pts):
    ax3d.scatter(gps_pts[:,0], gps_pts[:,1], -gps_pts[:,2],
                 c='#ffffff', s=12, alpha=0.5, marker='.', label='GPS Fixes', zorder=5)

# Target marker (flat on the ground)
ax3d.scatter([TARGET_NORTH_M], [TARGET_EAST_M], [0],
             c='#ff4488', s=120, marker='*', zorder=10, label='Target')

# Start / end
ax3d.scatter([gt_path[0,0]],  [gt_path[0,1]],  [-gt_path[0,2]],
             c='#44ff88', s=70, marker='o', zorder=10)
ax3d.scatter([gt_path[-1,0]], [gt_path[-1,1]], [-gt_path[-1,2]],
             c='#ff4466', s=70, marker='s', zorder=10)

ax3d.legend(loc='upper left', facecolor='#111827', edgecolor='#2a3550',
            labelcolor='#c0d0ff', fontsize=7)
ax3d.set_title('3D Trajectory — coloured by flight phase', color='#c0d0ff',
               fontsize=10, pad=10)

# ── Altitude panel with phase bands ──────────────────────────
for ph in phases_ordered:
    mask = phase_hist == ph
    if not np.any(mask): continue
    ax_alt.fill_between(t_hist, 0, alt_gt,
                        where=mask, alpha=0.18, color=PHASE_COLORS[ph])
ax_alt.plot(t_hist, alt_gt,  color='#c0d8ff', lw=1.2, label='True Alt')
ax_alt.plot(t_hist, alt_est, color='#ff6b35', lw=1.0, ls='--', label='EKF Alt')
ax_alt.axhline(EGG_DROP_ALT, color='#ff4488', lw=0.8, ls=':', label='Drop alt')
ax_alt.axhline(TP_ENTRY_ALT, color='#ffe066', lw=0.8, ls=':', label='TP entry alt')
ax_alt.set_title('Altitude (m)', fontsize=9)
ax_alt.set_xlabel('Time (s)', fontsize=8)
ax_alt.legend(facecolor='#111827', edgecolor='#2a3550',
              labelcolor='#c0d0ff', fontsize=6)

# ── Speed panel ───────────────────────────────────────────────
ax_vel.plot(t_hist, vel_gt,  color='#c0d8ff', lw=1.2, label='True Spd')
ax_vel.plot(t_hist, vel_est, color='#ff6b35', lw=1.0, ls='--', label='EKF Spd')
ax_vel.set_title('Ground Speed (m/s)', fontsize=9)
ax_vel.set_xlabel('Time (s)', fontsize=8)
ax_vel.legend(facecolor='#111827', edgecolor='#2a3550',
              labelcolor='#c0d0ff', fontsize=7)

# ── Stats ─────────────────────────────────────────────────────
pos_err  = np.linalg.norm(gt_path[:,:2] - est_path[:,:2], axis=1)
stats_txt = (f"EKF pos error  mean={pos_err.mean():.1f}m  max={pos_err.max():.1f}m  "
             f"final={pos_err[-1]:.1f}m  |  t={SIM_DURATION:.0f}s  |  "
             f"egg={'released' if sim.egg_released else 'NOT released'}")
fig.text(0.52, 0.01, stats_txt, ha='center', va='bottom',
         color='#8899bb', fontsize=8, family='monospace',
         bbox=dict(boxstyle='round,pad=0.3', facecolor='#111827',
                   edgecolor='#2a3550', alpha=0.8))

# ── Animated glider dot ───────────────────────────────────────
dot_gt,  = ax3d.plot([], [], [], 'o', color='#44ff88', ms=8, zorder=20)
dot_est, = ax3d.plot([], [], [], 'o', color='#ff4466', ms=7, zorder=20)
ANIM_STEPS = len(gt_path)
ANIM_SKIP  = max(1, ANIM_STEPS // 300)

def animate(frame):
    i = min(frame * ANIM_SKIP, ANIM_STEPS - 1)
    dot_gt.set_data ([gt_path[i,0]],  [gt_path[i,1]])
    dot_gt.set_3d_properties([-gt_path[i,2]])
    dot_est.set_data([est_path[i,0]], [est_path[i,1]])
    dot_est.set_3d_properties([-est_path[i,2]])
    return dot_gt, dot_est

ani = animation.FuncAnimation(fig, animate, frames=ANIM_STEPS//ANIM_SKIP,
                               interval=30, blit=False, repeat=True)

plt.savefig('outputs/glider_ekf_path.png', dpi=150,
            bbox_inches='tight', facecolor=fig.get_facecolor())
print("Static PNG saved.")
plt.show()