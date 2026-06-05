# RSX CanSat 2026 — Descent Path-Guidance (v3: solve-from-current-state)

One routine, `solve(state)`, runs at deploy *and* at every replan: from wherever
the glider is **right now** (position, altitude, heading) it computes a feasible
path to the landing. Altitude is the independent, monotone-decreasing variable,
so an off-nominal deploy z or a mid-flight z-shift is just a different "now" to
solve from — nothing depends on stored altitude bands.

```
ARC(align) → LINE(homing) → SPIRAL(N loops, radius R) → LINE(approach) → land
```
Every junction tangent (G1); curvature steps at corners (no clothoids).

## What's an input vs solved
**Inputs (boundary conditions only):** current state (pos, z, **heading**),
landing position, landing **axis** (a line), bounding box, glide ratio,
min_turn_radius, max_radius, approach length.

**Solved by the planner:**
- **landing direction** — which end of the axis (chosen by geometry/cost);
- **spiral side** (CW/CCW) — chosen by geometry;
- **spiral radius R** — the *continuous closure knob*, sized so total arc =
  `glide_ratio·(z − z_land)` ⇒ **exact budget closure, zero residual**;
- **loop count N** — coarse closure;
- **Dubins align entry** — an initial turn (≤ min_turn_radius) that respects the
  deploy/current heading and rolls onto a straight leg tangent to the spiral.

**Hard bounding box:** candidates whose path leaves the box are rejected.
**Objective among feasible:** maximize robustness margin — prefer R mid-range
(room to flex on the next replan) and clearance from the box walls; tie-break on
least turning.

## replan = solve from current state
`replan(state)` is `solve` from the current state, adopting the **measured glide
ratio** (`|v_h|/v_d`) so the budget closes against actual sink. A gust (horizontal
displacement) or a sudden z-shift just changes the "current state"; re-solving
re-closes the budget and **keeps the landing fixed** by re-choosing R / N / entry.
Transactional: a fully infeasible re-solve rolls back to the last valid plan.

Degraded fallback: if no candidate is feasible (deployed too low/far, box too
tight), fly best-glide straight at the landing (`status = Degraded`).

## Files
- `PathGuidance.hpp/.cpp` — the guidance class (portable C++17, float-only, no
  heap/exceptions/STL, fixed 4-segment array → drops into the STM32G431KB project).
- `bindings.cpp` — pybind11 wrapper.
- `sim.py` — real-time interactive simulator.
- `v3proto.py` — the Python geometry prototype (validation reference).
- `path_plot_v3.png` — static render.

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
C:\Users\adamk\miniconda3\envs\cansat\python.exe sim.py
```

### Linux / macOS

```bash
# Configure & build
cmake -B build -Dpybind11_DIR=$(python3 -m pybind11 --cmakedir)
cmake --build build

# Copy the extension module next to sim.py
cp build/pathguidance*.so .       # Linux
# cp build/pathguidance*.dylib .  # macOS (if needed)

python3 sim.py
```

On macOS, if using the one-liner instead of CMake, replace `-shared` with
`-undefined dynamic_lookup -dynamiclib`.

## Validation
C++ matches the Python prototype exactly across scenarios (nominal / adverse
deploy heading / deployed-high / deployed-low): same landing-dir, side, R, N, all
closing to residual 0.000, all landing on target, all junctions tangent, all
inside the box. Mid-flight gust and +60 m z-shift both re-solve to exact closure.
```
nominal  land_dir 210 side +1 R 34.6 N 1 resid 0.000
adverse  land_dir  30 side +1 R 50.5 N 1 resid 0.000   (big align turn to recover)
high     land_dir 210 side -1 R 46.5 N 2 resid 0.000   (extra loop burns excess)
low      land_dir 210 side +1 R 38.6 N 0 resid 0.000   (near-direct)
```

## Notes / next
- Clean closed-loop tracking ≈ 5–6 m (terminal along-track lag from the carrot
  lookahead, same pure-pursuit property as before; raise lookahead to trade
  cross-track for terminal accuracy).
- Auto-replan on a fixed cadence still adds small transients (each re-solve can
  shift the spiral); hysteresis (re-solve only when budget error crosses a
  threshold) is the cleanup if you want it.
