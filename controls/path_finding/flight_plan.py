import math
from dataclasses import dataclass
from typing import Tuple
from enum import Enum, auto

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
AIRSPEED          = 17.0
GLIDE_RATIO       = 6.0                       # clean aero glide ratio (no extra sink)
V_DESCENT_CLEAN   = AIRSPEED / GLIDE_RATIO    # ~2.83 m/s natural sink — MEASURE THIS
V_DESCENT_TARGET  = 5.0                       # commanded sink in cruise/spiral (competition req.)
EGG_DROP_ALT      = 2.0
SPIRAL_RADIUS     = 60.0
SPIRAL_ENTRY_DIST = 80.0
TP_ENTRY_ALT      = 30.0                      # must be below this to commit to terminal run

# Effective glide ratio while descending at the *commanded* cruise/spiral sink
# rate (V_DESCENT_TARGET), not the clean aero ratio. can_reach_target uses this
# so the predicted range matches the descent profile actually being flown.
#   AIRSPEED / V_DESCENT_TARGET = 17 / 5 = 3.4
GLIDE_RATIO_EFF   = AIRSPEED / V_DESCENT_TARGET

K_ROLL_P    = 1.5
K_ROLL_D    = 0.3
K_PITCH_P   = 1.2
MAX_AILERON  = 30.0
MAX_ELEVATOR = 25.0

GLIDE_TOLERANCE = 0.20  # 20% band on can_reach check

# ---------------------------------------------------------------------------
# Phase enum
# ---------------------------------------------------------------------------
class Phase(Enum):
    LAUNCH_PAD    = auto()
    PHASE1_CRUISE = auto()
    PHASE2_SPIRAL = auto()
    TERMINAL      = auto()
    LANDING       = auto()

# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------
@dataclass
class Target:
    x: float   # East  (m) from launch origin
    y: float   # North (m) from launch origin

@dataclass
class FlightState:
    """Populated every tick directly from EKF outputs."""
    # --- EKF output 1: position (local ENU, metres) ---
    x:   float = 0.0   # East
    y:   float = 0.0   # North
    alt: float = 0.0   # Up (AGL after CAL command at launch)

    # --- EKF output 2: velocity (m/s, ENU frame) ---
    vx: float = 0.0    # East
    vy: float = 0.0    # North
    vz: float = 0.0    # Up (negative = descending)

    # --- EKF output 3: quaternion (body orientation) ---
    q1: float = 1.0    # w  (scalar part)
    q2: float = 0.0    # x
    q3: float = 0.0    # y
    q4: float = 0.0    # z

    # --- EKF output 4: gyro angular rates (rad/s, body frame) ---
    omega_x: float = 0.0   # roll rate
    omega_y: float = 0.0   # pitch rate
    omega_z: float = 0.0   # yaw rate

    # --- Derived from quaternion (computed each tick) ---
    roll:    float = 0.0   # rad
    pitch:   float = 0.0   # rad
    heading: float = 0.0   # rad, 0 = East, CCW positive

    # --- Mission bookkeeping ---
    phase:        Phase = Phase.LAUNCH_PAD
    t:            float = 0.0
    egg_released: bool  = False
    drop_heading: float = 0.0   # heading captured at egg release, flown during landing

    def update_derived(self) -> None:
        """
        Call once per tick after populating q1-q4 from the EKF.
        Quaternion convention assumed: q1=w, q2=x, q3=y, q4=z
        (Hamilton convention — confirm this matches your EKF output)
        """
        w, x, y, z = self.q1, self.q2, self.q3, self.q4

        self.roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))

        sin_pitch  = max(-1.0, min(1.0, 2*(w*y - z*x)))
        self.pitch = math.asin(sin_pitch)

        self.heading = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

    def horizontal_dist_to(self, target: 'Target') -> float:
        return math.hypot(target.x - self.x, target.y - self.y)

    def bearing_to(self, target: 'Target') -> float:
        """Bearing from current position to target (rad, 0=East, CCW+)."""
        return math.atan2(target.y - self.y, target.x - self.x)

# ---------------------------------------------------------------------------
# Coordinate utility
# ---------------------------------------------------------------------------
def latlon_to_enu(lat: float, lon: float,
                  lat0: float, lon0: float) -> Tuple[float, float]:
    R = 6_371_000.0
    north = math.radians(lat - lat0) * R
    east  = math.radians(lon - lon0) * R * math.cos(math.radians(lat0))
    return east, north

# ---------------------------------------------------------------------------
# Glide reachability check
# ---------------------------------------------------------------------------
def can_reach_target(state: FlightState, target: Target) -> bool:
    """
    True if current altitude and distance put the target inside the glide
    envelope — i.e. descending at the commanded sink rate we will arrive at
    approximately EGG_DROP_ALT.

    GAP-3 FIX: uses GLIDE_RATIO_EFF instead of the clean GLIDE_RATIO, so the
    predicted range matches the descent profile actually being commanded.
    """
    dist_h      = state.horizontal_dist_to(target)
    alt_to_lose = state.alt - EGG_DROP_ALT
    glide_range = alt_to_lose * GLIDE_RATIO_EFF

    within_range   = glide_range >= dist_h * (1 - GLIDE_TOLERANCE)
    wont_overshoot = glide_range <= dist_h * (1 + GLIDE_TOLERANCE)
    return within_range and wont_overshoot

# ---------------------------------------------------------------------------
# Phase classifier — evaluated fresh every tick
# ---------------------------------------------------------------------------
def classify_phase(state: FlightState, target: Target) -> Phase:
    """
    GAP-1 FIX: LANDING is latched once the egg is released.
    GAP-2 FIX: TERMINAL requires BOTH the glide band (can_reach_target) AND
               being below TP_ENTRY_ALT, so the straight final run only starts
               when low and lined up. Until then we spiral, bleeding altitude.
    """
    if state.egg_released:
        return Phase.LANDING

    low_enough = state.alt <= TP_ENTRY_ALT

    if low_enough and can_reach_target(state, target):
        return Phase.TERMINAL

    if state.horizontal_dist_to(target) <= SPIRAL_ENTRY_DIST:
        return Phase.PHASE2_SPIRAL

    return Phase.PHASE1_CRUISE

# ---------------------------------------------------------------------------
# Egg release — checked every tick (GAP-5 FIX)
# ---------------------------------------------------------------------------
def maybe_release_egg(state: FlightState, target: Target) -> None:
    """
    Release when at/below drop altitude and essentially over the target,
    regardless of phase. Captures heading so the landing roll-out flies
    straight ahead onto the strip past the target.
    """
    if state.egg_released:
        return

    low      = state.alt <= EGG_DROP_ALT
    over_tgt = state.horizontal_dist_to(target) <= SPIRAL_RADIUS

    if low and over_tgt:
        trigger_egg_release()
        state.egg_released = True
        state.drop_heading = state.heading

# ---------------------------------------------------------------------------
# Phase velocity planners
# ---------------------------------------------------------------------------
def p1(state: FlightState, target: Target) -> Tuple[float, float, float]:
    """Cruise: fly straight toward target."""
    bearing = state.bearing_to(target)
    vx = AIRSPEED * math.cos(bearing)
    vy = AIRSPEED * math.sin(bearing)
    vz = -V_DESCENT_TARGET
    return vx, vy, vz

def p2(state: FlightState, target: Target) -> Tuple[float, float, float]:
    """Spiral: vector field orbit around target at SPIRAL_RADIUS."""
    dx = state.x - target.x
    dy = state.y - target.y
    r  = max(math.hypot(dx, dy), 1e-3)

    r_hat = (dx / r, dy / r)
    t_hat = (-dy / r, dx / r)          # 90° CCW = left-hand orbit

    k           = 0.6
    radial_gain = max(-0.8, min(0.8, k * (r - SPIRAL_RADIUS) / SPIRAL_RADIUS))
    dir_x       = t_hat[0] - radial_gain * r_hat[0]
    dir_y       = t_hat[1] - radial_gain * r_hat[1]
    mag         = math.hypot(dir_x, dir_y)

    vx = AIRSPEED * dir_x / mag
    vy = AIRSPEED * dir_y / mag
    vz = -V_DESCENT_TARGET
    return vx, vy, vz

def tp(state: FlightState, target: Target) -> Tuple[float, float, float]:
    """Terminal: fly straight at target on the clean glide down to drop alt."""
    bearing = state.bearing_to(target)
    vx = AIRSPEED * math.cos(bearing)
    vy = AIRSPEED * math.sin(bearing)
    vz = -V_DESCENT_CLEAN              # clean glide; GLIDE_RATIO=6 holds here
    return vx, vy, vz

def lp(state: FlightState, target: Target) -> Tuple[float, float, float]:
    """
    Landing: egg is already gone. Fly the heading captured at drop, clean
    glide, straight out onto the strip ahead of the target.

    GAP-6 FIX: hold drop_heading instead of steering back to the target,
    which would circle it rather than roll out past it.
    """
    vx = AIRSPEED * math.cos(state.drop_heading)
    vy = AIRSPEED * math.sin(state.drop_heading)
    vz = -V_DESCENT_CLEAN
    return vx, vy, vz

def trigger_egg_release() -> None:
    """TODO: send CMD,<TEAM_ID>,MEC,EGG,ON to servo layer."""
    pass

# ---------------------------------------------------------------------------
# plan_v — handles egg release, selects phase, returns desired velocity
# ---------------------------------------------------------------------------
def plan_v(state: FlightState, target: Target) -> Tuple[float, float, float]:
    maybe_release_egg(state, target)   # checked every tick, before dispatch (GAP-5)

    phase = classify_phase(state, target)
    state.phase = phase

    if phase == Phase.TERMINAL:
        return tp(state, target)
    elif phase == Phase.PHASE2_SPIRAL:
        return p2(state, target)
    elif phase == Phase.PHASE1_CRUISE:
        return p1(state, target)
    else:
        return lp(state, target)       # LANDING

# ---------------------------------------------------------------------------
# Mission termination (GAP-1 FIX)
# ---------------------------------------------------------------------------
def mission_active(state: FlightState) -> bool:
    """
    Keep running through LANDING; stop only once on the ground after the drop.
    Landing therefore really executes (lp runs), rather than the loop exiting
    the instant LANDING is entered.
    """
    if state.egg_released and state.alt <= 0.0:
        return False
    return True

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    TARGET_LAT, TARGET_LON = 38.0000, -97.0000
    ORIGIN_LAT, ORIGIN_LON = 38.0010, -97.0010

    tx, ty = latlon_to_enu(TARGET_LAT, TARGET_LON, ORIGIN_LAT, ORIGIN_LON)
    target = Target(x=tx, y=ty)
    state  = FlightState()

    while mission_active(state):
        # --- Populate from EKF each tick --- (to be finalized)
        # state.x, state.y, state.alt   = ekf.position()
        # state.vx, state.vy, state.vz  = ekf.velocity()
        # state.q1, state.q2, state.q3, state.q4 = ekf.quaternion()
        # state.omega_x, state.omega_y, state.omega_z = ekf.gyro_rates()

        state.update_derived()    # extracts roll, pitch, heading from quaternion
        state.t += 1.0

        vx, vy, vz = plan_v(state, target)

        # TODO: send to servo driver

        print(f"t={state.t:.0f}s | phase={state.phase.name} | "
              f"alt={state.alt:.1f}m | "
              f"hdg={math.degrees(state.heading):.1f}° | egg={int(state.egg_released)} | "
              f"VX={vx:.1f} VY={vy:.1f} VZ={vz:.1f}")

        
