"""
Glider Flight Simulator — Manual Control Mode
=============================================
Controls:
  A / D          — Roll left / right  (bank angle)
  W / S          — Pitch up / down    (flight path angle)
  R              — Reset glider to start position
  TAB            — Toggle view (3D perspective / top-down / side)
  SPACE          — Toggle HUD overlay  /  Resume after landing
  BACKSPACE      — Rewind (hold to scrub back through history)
  ESC            — Quit

Mouse:
  Hold RMB + drag — Rotate 3D camera
  Scroll wheel    — Zoom in/out

Future:
  SeligSIM / external controller hookup via vars.CONTROLLER_PORT
"""

import pygame
import numpy as np
import sys
import math
from dataclasses import dataclass, field
from typing import Optional
import vars

# ─────────────────────────────────────────────────────────────────────────────
# COLOURS
# ─────────────────────────────────────────────────────────────────────────────
C_BG            = (10,  12,  18)
C_GRID          = (25,  30,  42)
C_GRID_MAJOR    = (40,  48,  65)
C_HORIZON       = (50,  60,  80)
C_SKY_TOP       = (8,   18,  45)
C_SKY_BOT       = (20,  40,  80)
C_GROUND        = (22,  28,  18)
C_GROUND_DARK   = (12,  16,  10)
C_GLIDER        = (220, 230, 255)
C_GLIDER_WING   = (180, 195, 240)
C_TRAIL         = (80,  120, 200)
C_TRAIL_OLD     = (30,  50,  90)
C_TRAIL_REWIND  = (200, 100, 60)
C_STRIP         = (60,  55,  40)
C_STRIP_EDGE    = (200, 180, 80)
C_TARGET        = (255, 80,  80)
C_TARGET_RING   = (255, 120, 60)
C_HUD_TEXT      = (200, 220, 255)
C_HUD_WARN      = (255, 160, 40)
C_HUD_CRIT      = (255, 60,  60)
C_HUD_OK        = (80,  220, 120)
C_ACCENT        = (60,  160, 255)
C_REWIND        = (255, 140, 40)
C_WHITE         = (255, 255, 255)
C_DIM           = (80,  90,  110)

SCREEN_W, SCREEN_H = 1280, 800

# Rewind history
HISTORY_MAX  = 4000   # frames  (~200 s at 20 Hz)
REWIND_SPEED = 8      # frames to step back per tick when held

# ─────────────────────────────────────────────────────────────────────────────
# REAL GLIDER DIMENSIONS (metres)
# ─────────────────────────────────────────────────────────────────────────────
BODY_LENGTH_M = 0.353
BODY_RADIUS_M = 0.046
WING_SPAN_M   = vars.WING_SPAN_MM / 1000.0   # 0.25 m

# Visual scale so the tiny glider is readable on screen.
# This only affects rendering — physics use real values from vars.py.
GLIDER_VISUAL_SCALE = 6.0


# ─────────────────────────────────────────────────────────────────────────────
# GLIDER STATE
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class GliderState:
    x:     float = 0.0
    y:     float = 0.0
    z:     float = vars.START_ALTITUDE_M
    vx:    float = 0.0
    vy:    float = vars.CRUISE_SPEED_MS
    vz:    float = -vars.DESCENT_RATE_MS
    roll:  float = 0.0
    pitch: float = -math.degrees(math.atan2(vars.DESCENT_RATE_MS, vars.CRUISE_SPEED_MS))
    yaw:   float = 0.0
    cmd_roll:  float = 0.0
    cmd_pitch: float = 0.0
    trail: list = field(default_factory=list)

    def airspeed(self):
        return math.sqrt(self.vx**2 + self.vy**2 + self.vz**2)

    def ground_speed(self):
        return math.sqrt(self.vx**2 + self.vy**2)

    def load_factor(self):
        return 1.0 / max(math.cos(math.radians(self.roll)), 0.01)

    def turn_radius(self):
        phi = math.radians(abs(self.roll))
        if phi < 0.01:
            return float('inf')
        return self.airspeed()**2 / (vars.G * math.tan(phi))

    def stall_speed_banked(self):
        return vars.STALL_SPEED_MS / math.sqrt(max(0.01, math.cos(math.radians(self.roll))))

    def is_stalled(self):
        return self.airspeed() < self.stall_speed_banked()

    def snapshot(self):
        return {k: getattr(self, k) for k in
                ('x','y','z','vx','vy','vz','roll','pitch','yaw','cmd_roll','cmd_pitch')}

    def restore(self, snap: dict):
        for k, v in snap.items():
            setattr(self, k, v)


# ─────────────────────────────────────────────────────────────────────────────
# PHYSICS
# ─────────────────────────────────────────────────────────────────────────────
def update_physics(state: GliderState, dt: float):
    # Roll
    roll_err = state.cmd_roll - state.roll
    state.roll += float(np.clip(roll_err, -vars.KB_BANK_RATE_DEG_S * dt, vars.KB_BANK_RATE_DEG_S * dt))
    state.roll  = float(np.clip(state.roll, -vars.OPERATIONAL_MAX_BANK_DEG, vars.OPERATIONAL_MAX_BANK_DEG))

    # Pitch
    pitch_err = state.cmd_pitch - state.pitch
    state.pitch += float(np.clip(pitch_err, -vars.KB_PITCH_RATE_DEG_S * dt, vars.KB_PITCH_RATE_DEG_S * dt))
    state.pitch  = float(np.clip(state.pitch, -30.0, 15.0))

    # Yaw from coordinated turn
    phi = math.radians(state.roll)
    if abs(phi) > 0.01:
        state.yaw += math.degrees((vars.G * math.tan(phi)) / vars.CRUISE_SPEED_MS) * dt
    state.yaw %= 360.0

    # Flight path angle
    nominal_fpa = math.atan2(-vars.DESCENT_RATE_MS, vars.CRUISE_SPEED_MS)
    fpa = float(np.clip(math.radians(state.pitch) + nominal_fpa, math.radians(-60), math.radians(20)))

    lf = state.load_factor()
    h_speed = vars.CRUISE_SPEED_MS * math.cos(fpa) * (1.0 - 0.05 * (lf - 1.0))

    yaw_rad = math.radians(state.yaw)
    state.vx = h_speed * math.sin(yaw_rad)
    state.vy = h_speed * math.cos(yaw_rad)
    state.vz = vars.CRUISE_SPEED_MS * math.sin(fpa)

    state.x += state.vx * dt
    state.y += state.vy * dt
    state.z  = max(0.0, state.z + state.vz * dt)
    if state.z == 0.0:
        state.vz = 0.0

    state.trail.append((state.x, state.y, state.z))
    if len(state.trail) > HISTORY_MAX:
        state.trail.pop(0)


# ─────────────────────────────────────────────────────────────────────────────
# CAMERA
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class Camera:
    azimuth:   float = 225.0
    elevation: float = 25.0
    distance:  float = 300.0
    target:    np.ndarray = field(default_factory=lambda: np.array([0., 0., 250.]))

    def project(self, point_3d: np.ndarray, w: int, h: int) -> Optional[tuple]:
        az = math.radians(self.azimuth)
        el = math.radians(self.elevation)

        cx = self.target[0] + self.distance * math.sin(az) * math.cos(el)
        cy = self.target[1] - self.distance * math.cos(az) * math.cos(el)
        cz = self.target[2] + self.distance * math.sin(el)

        dx = point_3d[0] - cx
        dy = point_3d[1] - cy
        dz = point_3d[2] - cz

        rx =  math.cos(az);               ry = math.sin(az);               rz = 0.0
        ux = -math.sin(az)*math.sin(el);  uy = math.cos(az)*math.sin(el);  uz = math.cos(el)
        fx = -math.sin(az)*math.cos(el);  fy = math.cos(az)*math.cos(el);  fz = math.sin(el)

        cam_x =  dx*rx + dy*ry + dz*rz
        cam_y = -(dx*ux + dy*uy + dz*uz)
        cam_z =  dx*fx + dy*fy + dz*fz

        if cam_z < 0.5:
            return None

        fov_scale = h * 0.8
        return (int(w/2 + cam_x/cam_z * fov_scale),
                int(h/2 + cam_y/cam_z * fov_scale))

    def follow(self, state: GliderState):
        self.target = np.array([state.x, state.y, state.z])


# ─────────────────────────────────────────────────────────────────────────────
# DRAW HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def lerp_colour(c1, c2, t):
    t = max(0.0, min(1.0, t))
    return tuple(int(c1[i] + (c2[i] - c1[i]) * t) for i in range(3))


def draw_gradient_rect(surf, rect, c_top, c_bot):
    x, y, w, h = rect
    for i in range(h):
        t = i / max(h-1, 1)
        pygame.draw.line(surf, lerp_colour(c_top, c_bot, t), (x, y+i), (x+w, y+i))


def draw_sky_ground(surf, cam: Camera, w, h):
    horizon_y = int(h * (0.5 + cam.elevation / 90.0 * 0.4))
    horizon_y = max(0, min(h, horizon_y))
    draw_gradient_rect(surf, (0, 0, w, horizon_y), C_SKY_TOP, C_SKY_BOT)
    draw_gradient_rect(surf, (0, horizon_y, w, h-horizon_y), C_GROUND, C_GROUND_DARK)
    pygame.draw.line(surf, C_HORIZON, (0, horizon_y), (w, horizon_y), 1)


def project_grid(surf, cam: Camera, w, h, state: GliderState):
    gs   = 50
    size = 500
    ox   = round(state.x / gs) * gs
    oy   = round(state.y / gs) * gs
    for gx in range(int(ox-size), int(ox+size+1), gs):
        pts = [cam.project(np.array([float(gx), float(gy), 0.0]), w, h)
               for gy in range(int(oy-size), int(oy+size+1), gs)]
        pts = [p for p in pts if p]
        if len(pts) >= 2:
            pygame.draw.lines(surf, C_GRID_MAJOR if gx % 200 == 0 else C_GRID, False, pts, 1)
    for gy in range(int(oy-size), int(oy+size+1), gs):
        pts = [cam.project(np.array([float(gx), float(gy), 0.0]), w, h)
               for gx in range(int(ox-size), int(ox+size+1), gs)]
        pts = [p for p in pts if p]
        if len(pts) >= 2:
            pygame.draw.lines(surf, C_GRID_MAJOR if gy % 200 == 0 else C_GRID, False, pts, 1)


def draw_landing_strip(surf, cam: Camera, w, h, tx, ty):
    hw = vars.STRIP_WIDTH_M  / 2
    hl = vars.STRIP_LENGTH_M / 2
    hr = math.radians(vars.STRIP_HEADING_DEG)
    raw = [(-hw,-hl,0), (hw,-hl,0), (hw,hl,0), (-hw,hl,0)]
    corners = []
    for cx_, cy_, cz_ in raw:
        rx = cx_*math.cos(hr) - cy_*math.sin(hr) + tx
        ry = cx_*math.sin(hr) + cy_*math.cos(hr) + ty
        corners.append(np.array([rx, ry, 0.0]))

    pts = [cam.project(c, w, h) for c in corners]
    if all(pts):
        pygame.draw.polygon(surf, C_STRIP, pts)
        pygame.draw.polygon(surf, C_STRIP_EDGE, pts, 2)

    # Centre-line dashes
    for i in range(-4, 5):
        p1 = cam.project(np.array([tx, ty + i*(hl/5) - hl/12, 0.01]), w, h)
        p2 = cam.project(np.array([tx, ty + i*(hl/5) + hl/12, 0.01]), w, h)
        if p1 and p2:
            pygame.draw.line(surf, C_STRIP_EDGE, p1, p2, 1)

    # Target marker
    tp = cam.project(np.array([tx, ty, 0.01]), w, h)
    if tp:
        pygame.draw.circle(surf, C_TARGET,      tp, 7)
        pygame.draw.circle(surf, C_TARGET_RING, tp, 13, 2)
        pygame.draw.circle(surf, C_TARGET_RING, tp, 20, 1)


def draw_trail(surf, cam: Camera, w, h, state: GliderState, rewinding=False):
    trail = state.trail
    if len(trail) < 2:
        return
    pts = [cam.project(np.array(p), w, h) for p in trail]
    n = len(pts)
    for i in range(1, n):
        if pts[i-1] is None or pts[i] is None:
            continue
        t = i / n
        c = lerp_colour(C_TRAIL_OLD, C_TRAIL_REWIND if rewinding else C_TRAIL, t)
        pygame.draw.line(surf, c, pts[i-1], pts[i], 1 + (1 if t > 0.8 else 0))


def draw_glider_icon(surf, cam: Camera, w, h, state: GliderState):
    """
    Renders glider using real dimensions × GLIDER_VISUAL_SCALE.
    Body: 353 mm long, 92 mm diameter.  Wings: 250 mm span.
    """
    pos   = np.array([state.x, state.y, state.z])
    yaw_r = math.radians(state.yaw)
    roll_r  = math.radians(state.roll)
    pitch_r = math.radians(state.pitch)

    fwd_h = np.array([math.sin(yaw_r), math.cos(yaw_r), 0.0])
    fwd   = np.array([math.sin(yaw_r) * math.cos(pitch_r),
                      math.cos(yaw_r) * math.cos(pitch_r),
                     -math.sin(pitch_r)])
    fwd   = fwd / (np.linalg.norm(fwd) + 1e-9)

    right_flat = np.array([ math.cos(yaw_r), -math.sin(yaw_r), 0.0])
    up_flat    = np.array([0.0, 0.0, 1.0])
    right = right_flat * math.cos(roll_r) + up_flat * math.sin(roll_r)

    S = GLIDER_VISUAL_SCALE
    body_half = BODY_LENGTH_M * S / 2.0
    wing_half = WING_SPAN_M   * S / 2.0

    nose      = pos + fwd * body_half
    tail      = pos - fwd * body_half
    wing_root = pos - fwd * body_half * 0.15
    lwing     = wing_root - right * wing_half
    rwing     = wing_root + right * wing_half
    hstab_root = pos - fwd * body_half * 0.82
    lhstab    = hstab_root - right * wing_half * 0.32
    rhstab    = hstab_root + right * wing_half * 0.32

    def p(pt):
        return cam.project(pt, w, h)

    pn, pt_, pl, pr = p(nose), p(tail), p(lwing), p(rwing)
    plh, prh, pc    = p(lhstab), p(rhstab), p(pos)

    fuselage_w = max(2, int(BODY_RADIUS_M * S * 0.25))

    if pn and pt_:
        pygame.draw.line(surf, C_GLIDER, pn, pt_, fuselage_w)
    if pl and pr:
        pygame.draw.line(surf, C_GLIDER_WING, pl, pr, 2)
    if pc and pl:
        pygame.draw.line(surf, (120, 135, 175), pl, pc, 1)
    if pc and pr:
        pygame.draw.line(surf, (120, 135, 175), pr, pc, 1)
    if plh and prh:
        pygame.draw.line(surf, C_GLIDER_WING, plh, prh, 1)
    if pn:
        pygame.draw.circle(surf, C_ACCENT, pn, 3)
    if pc:
        pygame.draw.circle(surf, C_GLIDER, pc, 3)

    # Altitude shadow on ground
    pg = p(np.array([state.x, state.y, 0.0]))
    if pc and pg:
        pygame.draw.line(surf, (35, 42, 55), pc, pg, 1)
        shadow_r = max(1, int(3 * min(1.0, 80.0 / max(state.z, 1))))
        pygame.draw.circle(surf, (40, 50, 35), pg, shadow_r)


# ─────────────────────────────────────────────────────────────────────────────
# TOP-DOWN VIEW
# ─────────────────────────────────────────────────────────────────────────────
def draw_topdown(surf, state: GliderState, tx, ty, w, h, fonts, rewinding=False):
    surf.fill(C_BG)
    scale_m = 200.0
    cx, cy  = w//2, h//2

    def ws(wx, wy):
        return (cx + int((wx-state.x)/scale_m*(w//2)),
                cy - int((wy-state.y)/scale_m*(h//2)))

    for gx in range(-500, 501, 50):
        c = C_GRID_MAJOR if gx % 200 == 0 else C_GRID
        pygame.draw.line(surf, c, ws(state.x+gx, state.y-scale_m), ws(state.x+gx, state.y+scale_m), 1)
    for gy in range(-500, 501, 50):
        c = C_GRID_MAJOR if gy % 200 == 0 else C_GRID
        pygame.draw.line(surf, c, ws(state.x-scale_m, state.y+gy), ws(state.x+scale_m, state.y+gy), 1)

    hw = vars.STRIP_WIDTH_M/2;  hl = vars.STRIP_LENGTH_M/2
    strip_pts = [ws(tx-hw,ty-hl), ws(tx+hw,ty-hl), ws(tx+hw,ty+hl), ws(tx-hw,ty+hl)]
    pygame.draw.polygon(surf, C_STRIP, strip_pts)
    pygame.draw.polygon(surf, C_STRIP_EDGE, strip_pts, 2)

    tp = ws(tx, ty)
    pygame.draw.circle(surf, C_TARGET,      tp, 7)
    pygame.draw.circle(surf, C_TARGET_RING, tp, 13, 2)

    gp = ws(state.x, state.y)
    r_pix = int(vars.MIN_TURN_RADIUS_M / scale_m * (w//2))
    o_pix = int(vars.ORBIT_RADIUS_M    / scale_m * (w//2))
    pygame.draw.circle(surf, (40, 50, 70), gp, r_pix, 1)
    pygame.draw.circle(surf, (30, 50, 80), gp, o_pix, 1)

    if len(state.trail) >= 2:
        tpts = [ws(p[0], p[1]) for p in state.trail]
        n = len(tpts)
        for i in range(1, n):
            t = i/n
            pygame.draw.line(surf, lerp_colour(C_TRAIL_OLD, C_TRAIL_REWIND if rewinding else C_TRAIL, t),
                             tpts[i-1], tpts[i], 1)

    yaw_r = math.radians(state.yaw)
    ax = gp[0] + int(math.sin(yaw_r)*20)
    ay = gp[1] - int(math.cos(yaw_r)*20)
    pygame.draw.line(surf, C_ACCENT, gp, (ax, ay), 2)
    pygame.draw.circle(surf, C_GLIDER, gp, 5)
    pygame.draw.circle(surf, C_ACCENT,  gp, 5, 2)

    surf.blit(fonts['sm'].render("TOP-DOWN    inner=min turn radius    outer=orbit radius", True, C_DIM), (10, 10))


# ─────────────────────────────────────────────────────────────────────────────
# SIDE VIEW
# ─────────────────────────────────────────────────────────────────────────────
def draw_side(surf, state: GliderState, tx, ty, w, h, fonts):
    surf.fill(C_BG)
    max_dist = 2000.0
    max_alt  = vars.START_ALTITUDE_M * 1.1
    dist_now = math.sqrt(state.x**2 + state.y**2)

    def ws(d, alt):
        return (int(70 + (d/max_dist)*(w-90)),
                int(h-40 - (alt/max_alt)*(h-70)))

    pygame.draw.line(surf, C_GROUND, (70, h-40), (w-20, h-40), 2)
    for alt in range(0, int(max_alt)+1, 100):
        y = ws(0, alt)[1]
        pygame.draw.line(surf, C_GRID, (70, y), (w-20, y), 1)
        surf.blit(fonts['sm'].render(f"{alt}m", True, C_DIM), (5, y-8))

    glide_pts = []
    for d in range(0, int(max_dist)+1, 20):
        alt = vars.START_ALTITUDE_M - d / vars.GLIDE_RATIO
        if alt < 0: break
        glide_pts.append(ws(d, alt))
    if len(glide_pts) >= 2:
        pygame.draw.lines(surf, C_GRID_MAJOR, False, glide_pts, 1)

    if len(state.trail) >= 2:
        tpts = [ws(math.sqrt(p[0]**2+p[1]**2), p[2]) for p in state.trail]
        for i in range(1, len(tpts)):
            t = i/len(tpts)
            pygame.draw.line(surf, lerp_colour(C_TRAIL_OLD, C_TRAIL, t), tpts[i-1], tpts[i], 1)

    gp = ws(dist_now, state.z)
    pygame.draw.circle(surf, C_GLIDER, gp, 5)
    pygame.draw.circle(surf, C_ACCENT, gp, 5, 2)

    surf.blit(fonts['sm'].render("SIDE VIEW — Distance vs Altitude    ── ideal glide slope", True, C_DIM), (70, 10))


# ─────────────────────────────────────────────────────────────────────────────
# HUD
# ─────────────────────────────────────────────────────────────────────────────
def draw_hud(surf, state: GliderState, tx, ty, fonts, w, h, rewinding, paused):
    hud_w, hud_h = 285, 345
    hud = pygame.Surface((hud_w, hud_h), pygame.SRCALPHA)
    hud.fill((0, 0, 0, 160))

    border = C_REWIND if rewinding else (C_HUD_WARN if paused else C_ACCENT)
    pygame.draw.rect(hud, (*border, 120), (0, 0, hud_w, hud_h), 1)

    def label(text, value, y, colour=C_HUD_TEXT, unit=""):
        hud.blit(fonts['sm'].render(text, True, C_DIM), (10, y))
        t2 = fonts['md'].render(f"{value}{unit}", True, colour)
        hud.blit(t2, (hud_w - t2.get_width() - 10, y-2))

    stall_b  = state.stall_speed_banked()
    spd_col  = C_HUD_CRIT if state.is_stalled() else (C_HUD_WARN if state.airspeed() < stall_b*1.15 else C_HUD_OK)
    bank_col = C_HUD_CRIT if abs(state.roll) > vars.AERO_MAX_BANK_ANGLE_DEG*0.95 else (
               C_HUD_WARN if abs(state.roll) > vars.OPERATIONAL_MAX_BANK_DEG*0.9  else C_HUD_TEXT)

    title     = "<<< REWINDING" if rewinding else ("PAUSED" if paused else "GLIDER SIM")
    title_col = C_REWIND if rewinding else (C_HUD_WARN if paused else C_ACCENT)
    lbl = fonts['lg'].render(title, True, title_col)
    hud.blit(lbl, (hud_w//2 - lbl.get_width()//2, 6))
    pygame.draw.line(hud, (*border, 80), (10, 28), (hud_w-10, 28), 1)

    y, dy = 36, 26
    label("ALTITUDE",       f"{state.z:.1f}",                y);                     y += dy
    label("AIRSPEED",       f"{state.airspeed()*3.6:.1f}",   y, spd_col,  " km/h");  y += dy
    label("STALL (banked)", f"{stall_b*3.6:.1f}",            y, C_HUD_WARN," km/h"); y += dy
    label("GND SPEED",      f"{state.ground_speed()*3.6:.1f}",y, C_HUD_TEXT," km/h");y += dy
    label("DESCENT",        f"{-state.vz:.1f}",              y, C_HUD_TEXT," m/s");  y += dy
    label("HEADING",        f"{state.yaw:.1f}",              y, C_HUD_TEXT,"°");     y += dy
    label("ROLL",           f"{state.roll:.1f}",             y, bank_col,  "°");     y += dy
    label("PITCH",          f"{state.pitch:.1f}",            y, C_HUD_TEXT,"°");     y += dy
    label("LOAD FACTOR",    f"{state.load_factor():.2f}",    y, C_HUD_TEXT," g");    y += dy
    dist = math.sqrt((tx-state.x)**2 + (ty-state.y)**2)
    tr   = state.turn_radius()
    label("TGT DIST",    f"{dist:.0f}",                      y, C_HUD_TEXT," m");   y += dy
    label("TURN RADIUS", f"{tr:.0f}" if tr < 9999 else "INF",y, C_HUD_TEXT," m")

    surf.blit(hud, (10, 10))
    draw_artificial_horizon(surf, state, w, h, fonts)

    for i, line in enumerate(["A/D  bank      W/S  pitch",
                               "BKSP  rewind   R  reset",
                               "SPACE  HUD/resume   TAB  view"]):
        surf.blit(fonts['sm'].render(line, True, C_DIM),
                  (10, h - 10 - (2-i)*18))


def draw_artificial_horizon(surf, state: GliderState, w, h, fonts):
    cx, cy = 155, h-115
    r = 70

    clip = pygame.Surface((r*2, r*2), pygame.SRCALPHA)
    clip.fill((0,0,0,0))
    pygame.draw.circle(clip, C_SKY_BOT, (r, r), r)

    roll_r   = math.radians(-state.roll)
    pitch_px = int(math.radians(state.pitch) * r * 2)
    gpts = []
    for a in range(181):
        ar = math.radians(a)
        gpts.append((int(r + math.cos(ar+roll_r)*r),
                     int(r - math.sin(ar+roll_r)*r + pitch_px)))
    gpts += [(r*2, r*2), (0, r*2)]
    if len(gpts) >= 3:
        pygame.draw.polygon(clip, C_GROUND, gpts)

    hx1 = int(r + math.cos(roll_r+math.pi)*r)
    hy1 = int(r - math.sin(roll_r+math.pi)*r + pitch_px)
    hx2 = int(r + math.cos(roll_r)*r)
    hy2 = int(r - math.sin(roll_r)*r + pitch_px)
    pygame.draw.line(clip, C_HORIZON, (hx1,hy1), (hx2,hy2), 2)
    pygame.draw.line(clip, C_WHITE, (r-30,r), (r-10,r), 3)
    pygame.draw.line(clip, C_WHITE, (r+10,r), (r+30,r), 3)
    pygame.draw.circle(clip, C_WHITE, (r,r), 3)

    surf.blit(clip, (cx-r, cy-r))
    pygame.draw.circle(surf, C_ACCENT, (cx, cy), r, 2)
    surf.blit(fonts['sm'].render("AH", True, C_DIM), (cx-8, cy+r+4))


def draw_stall_warning(surf, w, h, tick):
    if (tick//15) % 2 == 0:
        s = pygame.Surface((w, h), pygame.SRCALPHA)
        s.fill((255, 0, 0, 30))
        surf.blit(s, (0,0))
        font = pygame.font.SysFont("monospace", 48, bold=True)
        t = font.render("!! STALL !!", True, (255, 60, 60))
        surf.blit(t, (w//2 - t.get_width()//2, h//2 - 24))


def draw_landed_overlay(surf, state: GliderState, tx, ty, w, h, fonts):
    overlay = pygame.Surface((w, h), pygame.SRCALPHA)
    overlay.fill((0, 0, 0, 110))
    surf.blit(overlay, (0,0))

    dist = math.sqrt((tx-state.x)**2 + (ty-state.y)**2)
    on_strip = (abs(state.x - tx) < vars.STRIP_WIDTH_M/2 and
                abs(state.y - ty) < vars.STRIP_LENGTH_M/2)
    col = C_HUD_OK if on_strip else C_HUD_WARN

    panel_w, panel_h = 400, 200
    px = w//2 - panel_w//2
    py = h//2 - panel_h//2
    panel = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
    panel.fill((0,0,0,180))
    pygame.draw.rect(panel, (*col, 160), (0,0,panel_w,panel_h), 2)
    surf.blit(panel, (px, py))

    lines = [
        ("LANDED",                                 fonts['lg'], C_ACCENT),
        (f"{dist:.1f} m from target",              fonts['md'], col),
        ("ON STRIP" if on_strip else "MISSED STRIP", fonts['md'], col),
        ("",                                        fonts['sm'], C_DIM),
        ("SPACE resume    R reset    BKSP rewind",  fonts['sm'], C_DIM),
    ]
    yy = py + 20
    for text, font, colour in lines:
        if text:
            t = font.render(text, True, colour)
            surf.blit(t, (w//2 - t.get_width()//2, yy))
        yy += font.get_height() + 6


def draw_rewind_bar(surf, history_len, history_max, w, h, fonts):
    bw, bh = w-40, 8
    bx, by = 20, h-30
    pygame.draw.rect(surf, C_GRID_MAJOR, (bx, by, bw, bh), 1)
    fill = int(bw * history_len / max(history_max, 1))
    pygame.draw.rect(surf, C_REWIND, (bx, by, fill, bh))
    surf.blit(fonts['sm'].render(f"<<< REWIND  {history_len} / {history_max} frames", True, C_REWIND),
              (bx, by-16))


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────
def main():
    pygame.init()
    pygame.display.set_caption("Glider Guidance Simulator — Manual Control")
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H), pygame.RESIZABLE)
    clock  = pygame.time.Clock()

    fonts = {
        'sm': pygame.font.SysFont("monospace", 12),
        'md': pygame.font.SysFont("monospace", 14, bold=True),
        'lg': pygame.font.SysFont("monospace", 16, bold=True),
    }

    def make_state():
        return GliderState()

    state   = make_state()
    camera  = Camera()
    camera.target = np.array([state.x, state.y, state.z])

    rng      = np.random.default_rng(42)
    target_x = float(rng.uniform(-vars.STRIP_LENGTH_M/4, vars.STRIP_LENGTH_M/4))
    target_y = float(rng.uniform(-vars.STRIP_WIDTH_M/4,  vars.STRIP_WIDTH_M/4))

    view_mode = 0
    show_hud  = True
    paused    = False
    rewinding = False
    history   = []          # list of snapshot dicts

    mouse_dragging = False
    last_mouse     = (0, 0)
    tick = 0

    while True:
        w, h = screen.get_size()

        # ── Events ──────────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit(); sys.exit()

                elif event.key == pygame.K_r:
                    state    = make_state()
                    history  = []
                    paused   = False
                    rewinding= False
                    camera.target = np.array([state.x, state.y, state.z])

                elif event.key == pygame.K_TAB:
                    view_mode = (view_mode + 1) % 3

                elif event.key == pygame.K_SPACE:
                    if paused:
                        paused = False   # resume after landing
                    else:
                        show_hud = not show_hud

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 3:
                    mouse_dragging = True;  last_mouse = event.pos
                elif event.button == 4:
                    camera.distance = max(20,   camera.distance * 0.9)
                elif event.button == 5:
                    camera.distance = min(5000, camera.distance * 1.1)

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 3:
                    mouse_dragging = False

            elif event.type == pygame.MOUSEMOTION:
                if mouse_dragging:
                    dx = event.pos[0] - last_mouse[0]
                    dy = event.pos[1] - last_mouse[1]
                    camera.azimuth   += dx * 0.4
                    camera.elevation  = max(-10, min(80, camera.elevation - dy*0.3))
                    last_mouse = event.pos

        # ── Held keys ───────────────────────────────────────────────────────
        keys      = pygame.key.get_pressed()
        rewinding = bool(keys[pygame.K_BACKSPACE]) and len(history) > 0

        if rewinding:
            steps = min(REWIND_SPEED, len(history))
            for _ in range(steps):
                if history:
                    state.restore(history.pop())
                    if state.trail:
                        state.trail.pop()
            paused = False

        elif not paused:
            # Control inputs
            brate = vars.KB_BANK_RATE_DEG_S * vars.SIM_DT
            if keys[pygame.K_a]:
                state.cmd_roll = max(-vars.OPERATIONAL_MAX_BANK_DEG, state.cmd_roll - brate*6)
            elif keys[pygame.K_d]:
                state.cmd_roll = min( vars.OPERATIONAL_MAX_BANK_DEG, state.cmd_roll + brate*6)
            else:
                state.cmd_roll *= 0.95

            prate = vars.KB_PITCH_RATE_DEG_S * vars.SIM_DT
            if keys[pygame.K_w]:
                state.cmd_pitch = min( 15.0, state.cmd_pitch + prate*4)
            elif keys[pygame.K_s]:
                state.cmd_pitch = max(-25.0, state.cmd_pitch - prate*4)
            else:
                state.cmd_pitch *= 0.95

            # Save snapshot then step
            history.append(state.snapshot())
            if len(history) > HISTORY_MAX:
                history.pop(0)

            update_physics(state, vars.SIM_DT)

            if state.z <= 0.0:
                paused = True

        # Camera
        if view_mode == 0:
            camera.follow(state)

        # ── Render ──────────────────────────────────────────────────────────
        if view_mode == 1:
            draw_topdown(screen, state, target_x, target_y, w, h, fonts, rewinding)
        elif view_mode == 2:
            draw_side(screen, state, target_x, target_y, w, h, fonts)
        else:
            screen.fill(C_BG)
            draw_sky_ground(screen, camera, w, h)
            project_grid(screen, camera, w, h, state)
            draw_landing_strip(screen, camera, w, h, target_x, target_y)
            draw_trail(screen, camera, w, h, state, rewinding)
            draw_glider_icon(screen, camera, w, h, state)

        if show_hud:
            draw_hud(screen, state, target_x, target_y, fonts, w, h, rewinding, paused)

        if state.is_stalled() and not paused and not rewinding:
            draw_stall_warning(screen, w, h, tick)

        if paused and not rewinding:
            draw_landed_overlay(screen, state, target_x, target_y, w, h, fonts)

        if rewinding:
            draw_rewind_bar(screen, len(history), HISTORY_MAX, w, h, fonts)

        # View label (bottom-right)
        mode_names = ["3D PERSPECTIVE", "TOP-DOWN", "SIDE PROFILE"]
        lbl = fonts['sm'].render(f"[TAB] {mode_names[view_mode]}", True, C_DIM)
        screen.blit(lbl, (w - lbl.get_width() - 10, h - 20))

        pygame.display.flip()
        clock.tick(vars.SIM_RENDER_FPS)
        tick += 1


if __name__ == "__main__":
    main()