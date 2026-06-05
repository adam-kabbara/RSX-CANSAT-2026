# RSX CanSat 2026 — Descent Path-Guidance (v2: clothoid G2 + replan)

Heuristic three-phase descent guidance for the autonomous paraglider CanSat.
Returns a 2D ground-track **heading** command each tick (pure-pursuit carrot);
descent is a consequence of the glide/pitch loop, not commanded here.

```
LINE (homing) → CLOTHOID → ARC (loiter) → CLOTHOID → LINE (approach) → landing
 κ: 0            0 → 1/R     1/R           1/R → 0     0
```

The clothoids make **curvature continuous (G2)** at the two corners, so the
commanded heading rate — and therefore the roll command your rate controller
tracks — never steps. The loiter is the energy-management slack: its sweep is
chosen so the total path length equals `glide_ratio × altitude_drop`.

## What changed from v1
- **Clothoid transitions** at homing→loiter and loiter→approach (was sharp corners).
- **Arc-length segment model** (`LINE / CLOTHOID / ARC`, fixed array, no heap).
  Altitude maps to arc length by `s = glide_ratio·(z − z0)`; the path is still
  single-valued in altitude so `pathAt(z)` is a cheap lookup.
- **`start_heading` (homing heading) is now an INPUT**, alongside `land_heading`.
  The entry is solved as a transition curve: the homing ray is the tangent and a
  clothoid eases onto the loiter circle. If the homing ray can't reach the loiter
  tangentially the plan reports `EntryInfeasible` (it does not silently bend).
- **`replan(state)`** — receding-horizon re-planning. Detects the phase from the
  current altitude and rebuilds only the *remaining* phases from the current
  state, keeping the landing anchor fixed. In the loiter it recomputes the
  remaining sweep; it also adopts the **measured glide ratio** (`|v_h|/v_d`) so
  the energy budget closes against actual performance, not the nominal number.
  `pathAt`/`getHeading` use the most recent plan.

## Files
- `PathGuidance.hpp` / `PathGuidance.cpp` — the guidance class (portable C++17,
  float-only, no heap/exceptions/STL containers → drops into the STM32G431KB project).
- `bindings.cpp` — pybind11 wrapper exposing the real class to Python.
- `sim.py` — interactive debug sim (sliders + replan demo) that closes the loop
  on the real C++ guidance.
- `path_plot_v2.png` — validation render (top-down + curvature profile + cross-track).

## Build
```
pip install pybind11 matplotlib numpy --break-system-packages
c++ -O2 -std=c++17 -shared -fPIC $(python3 -m pybind11 --includes) \
    bindings.cpp PathGuidance.cpp -o pathguidance$(python3-config --extension-suffix)
python3 sim.py
```

`sim.py` opens a **real-time interactive window** on the real C++ guidance:
- Play / Pause / Reset to fly the path forward in time and watch the glider.
- Sliders for every initial value (homing/land heading, loiter R, glide ratio,
  approach len, transition Lc, loiter dir, lookahead, start alt, sink bias,
  sim speed). Changing one re-plans and resets the run.
- "Replan now" button and an "auto-replan every N m" slider (in-loiter,
  receding-horizon, measured-glide-ratio).
- **Drag the glider (blue dot) with the mouse** to inject an external push / gust,
  then Replan to re-close the budget from the displaced state.

It needs an interactive matplotlib backend. If no window appears it falls back to
a static `path_plot_v2.png` and prints what to install:
```
Linux:  sudo apt-get install python3-tk     # then re-run python3 sim.py
macOS/conda: usually fine; else  pip install pyqt5
```

## Key parameters (`GuidanceParams`)
- `start_heading`, `land_heading` — homing and into-wind approach headings (rad), INPUTS.
- `glide_ratio`, `loiter_radius`, `min_turn_radius`, `approach_len`(+`_min`).
- `transition_len` — nominal exit-clothoid length (set by your max roll rate:
  `Lc ≈ V_h · φ_loiter / roll_rate_max`). Entry clothoid length is solved.
- `entry_clothoid_max` — reject homing headings so far off-tangent they'd need an
  impractically long entry spiral (0 = no cap). The sim sets `2·loiter_radius`.
- `lookahead_drop` — carrot lead, in metres of altitude. Bigger = less terminal
  along-track lag, more cross-track on tight turns (classic pursuit trade-off).

## Status / validation
- Geometry: lands **exactly** on target with the exact into-wind heading; curvature
  is continuous (G2) by construction — see the curvature panel.
- Feasible homing-heading window is narrow and scenario-dependent (the homing ray
  must thread onto the loiter circle); out-of-window headings flag `EntryInfeasible`.
- Closed-loop clean tracking ≈ **2.2 m** touchdown miss (matches v1).
- Energy budget: residual closes to ~0 when the approach-length flex can absorb it;
  otherwise it is bounded by the loiter-loop quantum (one loop ≈ 2πR) and re-closed
  by replanning.

## Known next step
Replanning re-closes the geometry exactly and adopts the measured glide ratio, but
calling it on a fixed cadence in the loiter accumulates small along-track lag
(each replan adds a minor tracking transient) and a large single replan can drop a
whole loop. Add **hysteresis** (replan only when the predicted budget error exceeds
a threshold) and consider scheduling `lookahead_drop` (larger on the final approach)
to suppress terminal lag. Homing-phase replan is intentionally disabled in the sim —
it just thrashes the entry solve.
