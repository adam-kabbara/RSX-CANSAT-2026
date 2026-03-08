import numpy as np

# =============================================================================
# ENVIRONMENT
# =============================================================================
G = 9.81                        # m/s² gravitational acceleration
AIR_DENSITY = 1.225             # kg/m³, sea level standard ISA

# =============================================================================
# WING GEOMETRY
# =============================================================================
WING_SPAN_MM    = 250           # mm
ROOT_CHORD_MM   = 70            # mm
TIP_CHORD_MM    = 60            # mm
MEAN_CHORD_MM   = 65            # mm
WING_AREA_M2    = 0.0325        # m²  (32,500 mm²)
AOA_DEG         = 4.5           # degrees, fixed angle of attack
DIHEDRAL_DEG    = 5.0           # degrees

# Aerodynamic coefficients (SG6043 airfoil)
CL_CRUISE       = 1.0           # lift coefficient at 4.5° AoA
CL_MAX          = 1.55          # max lift coefficient (at stall ~11°)

# =============================================================================
# MASS & LOADING
# =============================================================================
MASS_KG         = 0.620         # kg, total all-up weight

# =============================================================================
# STALL SPEED  (derived from lift equation at 1g, L = W)
#
#   L = 0.5 * rho * V² * S * CL_max   →   V_stall = sqrt(2mg / rho*S*CL_max)
# =============================================================================
STALL_SPEED_MS = np.sqrt(
    (2 * MASS_KG * G) /
    (AIR_DENSITY * WING_AREA_M2 * CL_MAX)
)
# default ≈ 14.0 m/s  (50.4 km/h)

# =============================================================================
# CRUISE SPEED & DESCENT
# =============================================================================
CRUISE_SPEED_MS     = 19.44     # m/s  (≈ 70 km/h)
DESCENT_RATE_MS     = 5.0       # m/s  nominal descent rate
DESCENT_RATE_MIN    = 3.0       # m/s
DESCENT_RATE_MAX    = 7.0       # m/s

# Glide ratio derived from cruise state
GLIDE_RATIO = CRUISE_SPEED_MS / DESCENT_RATE_MS   # ≈ 3.9

# =============================================================================
# MAX BANK ANGLE  (aerodynamic limit — banked stall speed = cruise speed)
#
#   In a banked turn, stall speed increases: V_stall_banked = V_stall / sqrt(cos φ)
#   Limit reached when V_stall_banked = V_cruise
#
#   cos(φ_max) = (V_stall / V_cruise)²
#   φ_max = arccos((V_stall / V_cruise)²)
# =============================================================================
_bank_cos = (STALL_SPEED_MS / CRUISE_SPEED_MS) ** 2
AERO_MAX_BANK_ANGLE_DEG = np.degrees(np.arccos(_bank_cos))
# default ≈ 58.7°  — absolute aerodynamic limit

# Operational limit — apply safety margin below aerodynamic limit
# Also bounded by structural g-limit:  n = 1/cos(φ),  φ = arccos(1/n)
STRUCTURAL_G_LIMIT = 3.0
STRUCTURAL_MAX_BANK_DEG = np.degrees(np.arccos(1.0 / STRUCTURAL_G_LIMIT))
# default ≈ 70.5°

# Operational max = conservative minimum of both limits
OPERATIONAL_MAX_BANK_DEG = min(AERO_MAX_BANK_ANGLE_DEG * 0.85, 45.0)
# default = 45°  (1.41g, well inside both limits)
OPERATIONAL_MAX_BANK_RAD = np.radians(OPERATIONAL_MAX_BANK_DEG)

# =============================================================================
# TURN GEOMETRY  (derived)
#
#   r = V² / (g * tan(φ))
# =============================================================================
MIN_TURN_RADIUS_M = CRUISE_SPEED_MS**2 / (G * np.tan(OPERATIONAL_MAX_BANK_RAD))
# default ≈ 38.5m

# Orbit settings — add margin on top of minimum
ORBIT_RADIUS_M          = MIN_TURN_RADIUS_M * 1.2  # default ≈ 46m
ORBIT_BANK_ANGLE_DEG    = 35.0                     # conservative for sustained orbit

# =============================================================================
# MISSION
# =============================================================================
START_ALTITUDE_M    = 500.0     # m AGL

# Landing strip (default)
STRIP_LENGTH_FT     = 200
STRIP_WIDTH_FT      = 40
STRIP_LENGTH_M      = STRIP_LENGTH_FT * 0.3048     # 60.96m
STRIP_WIDTH_M       = STRIP_WIDTH_FT  * 0.3048     # 12.19m
STRIP_HEADING_DEG   = 0.0       # orientation (degrees from north, 0 = strip runs N/S)

# Strip centre (set at runtime, placeholder here)
STRIP_CENTER_LAT    = 0.0
STRIP_CENTER_LON    = 0.0

# Target inside strip (randomised at runtime)
TARGET_LAT          = None
TARGET_LON          = None
TARGET_ALTITUDE_M   = 0.0       # touch-down at ground level

# =============================================================================
# SIMULATOR DEFAULTS
# =============================================================================
SIM_DT              = 0.05      # s  (20 Hz update rate)
SIM_RENDER_FPS      = 60

# Keyboard control rates
KB_BANK_RATE_DEG_S  = 30.0      # deg/s roll rate from keyboard
KB_PITCH_RATE_DEG_S = 10.0      # deg/s pitch rate from keyboard

# =============================================================================
# DEBUG / PRINT SUMMARY
# =============================================================================
if __name__ == "__main__":
    print("===== GLIDER PARAMETERS =====")
    print(f"  Mass:                  {MASS_KG} kg")
    print(f"  Wing area:             {WING_AREA_M2} m²")
    print(f"  Stall speed:           {STALL_SPEED_MS:.2f} m/s  ({STALL_SPEED_MS*3.6:.1f} km/h)")
    print(f"  Cruise speed:          {CRUISE_SPEED_MS:.2f} m/s  ({CRUISE_SPEED_MS*3.6:.1f} km/h)")
    print(f"  Speed margin:          {CRUISE_SPEED_MS/STALL_SPEED_MS:.2f}x stall")
    print(f"  Aero max bank:         {AERO_MAX_BANK_ANGLE_DEG:.1f}°")
    print(f"  Structural max bank:   {STRUCTURAL_MAX_BANK_DEG:.1f}°")
    print(f"  Operational max bank:  {OPERATIONAL_MAX_BANK_DEG:.1f}°")
    print(f"  Min turn radius:       {MIN_TURN_RADIUS_M:.1f} m")
    print(f"  Orbit radius:          {ORBIT_RADIUS_M:.1f} m")
    print(f"  Glide ratio:           {GLIDE_RATIO:.1f}:1")