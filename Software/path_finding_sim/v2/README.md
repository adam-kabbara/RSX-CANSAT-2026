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

## Build & run the debug front-end

### Prerequisites
- Miniconda or any Python 3.8+ environment
- CMake 3.15+
- **Windows**: Visual Studio 2022 Build Tools — install the "Desktop development with C++" workload

Install Python dependencies into your environment:
```
pip install pybind11 matplotlib numpy
```

### Windows (PowerShell)

> Make sure you are using your conda/venv Python, **not** the msys64 one.
> Adjust paths below if your env name or Python version differs.

```powershell
# 1. Check the pybind11 cmake dir for your environment
C:\Users\adamk\miniconda3\envs\cansat\python.exe -m pybind11 --cmakedir

# 2. Configure (replace python313.lib if your Python version differs —
#    check with: dir C:\Users\adamk\miniconda3\envs\cansat\libs\)
cmake -B build -G "Visual Studio 17 2022" `
  -Dpybind11_DIR="C:\Users\adamk\\miniconda3\envs\cansat\Lib\site-packages\pybind11\share\cmake\pybind11" `
  -DPYTHON_EXECUTABLE="C:\Users\adamk\miniconda3\envs\cansat\python.exe" `
  -DPYTHON_LIBRARIES="C:\Users\adamk\miniconda3\envs\cansat\libs\python313.lib" `
  -DPYTHON_INCLUDE_DIR="C:\Users\adamk\miniconda3\envs\cansat\include"

# 3. Build
cmake --build build --config Release

# 4. Copy the extension module next to sim.py
copy build\Release\pathguidance*.pyd .

# 5. Run
C:\Users\adamk\miniconda3\envs\cansat\python.exe sim.py        # writes path_plot_v2.png
```

### Linux / macOS

```bash
# Configure & build
cmake -B build -Dpybind11_DIR=$(python3 -m pybind11 --cmakedir)
cmake --build build

# Copy the extension module next to sim.py
cp build/pathguidance*.so .       # Linux
# cp build/pathguidance*.dylib .  # macOS (if needed)

python3 sim.py        # writes path_plot_v2.png
```

On macOS, if using the one-liner instead of CMake, replace `-shared` with
`-undefined dynamic_lookup -dynamiclib`.

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
