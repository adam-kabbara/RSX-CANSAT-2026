# RSX CanSat 2026 — Descent Path-Guidance (v1 + replan)

Heuristic three-phase descent guidance for the autonomous paraglider CanSat.
`getHeading()` returns a 2D ground-track heading each tick (pure-pursuit carrot);
descent is a consequence of the glide/pitch loop, not commanded here.

```
LINE (homing) → ARC (loiter) → LINE (approach) → landing
 derived ψ1      sweep θ          into-wind ψ4
```

- The reference path is single-valued in altitude (`P(d) → N,E`); the carrot sits
  `lookahead_drop` metres of altitude below the glider.
- The **homing heading is derived** (aim at the loiter entry `pt2`), so the
  geometry always finds a feasible entry. The loiter sweep `θ` is **continuous**,
  so the horizontal budget `L1 + Rθ + L3 = glide_ratio·H` closes **exactly**.
- The loiter is anchored to the landing/approach line (tangent rollout at `ψ4`).
- Curvature steps 0↔1/R at the two corners (G1) — see the curvature panel.

## New since the original v1
`replan(state)` — receding-horizon re-plan from the current state, landing kept
anchored:
- **Homing**: re-derives the whole line→arc→line from the current position.
- **Loiter**: keeps the anchored circle/exit, recomputes the remaining sweep to
  roll out at `pt3`. The exit angle is fixed, so the remaining sweep is quantised
  (integer loops) and the small leftover is the budget residual.
- Adopts the **measured glide ratio** (`|v_h|/v_d`) so the budget closes against
  actual sink, not the nominal number. Transactional: a failed re-solve is a
  no-op (keeps the valid plan).

Plus arc-length debug accessors (`eval_s`, `curvature_at`, `seg_*`) for the sim.
`pathAt(d)` is unchanged — a frozen lookup against the current plan.

## Files
- `PathGuidance.hpp` / `PathGuidance.cpp` — the guidance class (portable C++17,
  float-only, no heap/exceptions/STL → drops into the STM32G431KB project).
- `bindings.cpp` — pybind11 wrapper.
- `sim.py` — real-time interactive simulator on the real C++ class.
- `path_plot_v1.png` — static render (top-down + curvature + cross-track).

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

`sim.py` opens a real-time window:
- Play / Pause / Reset fly the path forward; sliders (land heading, loiter R,
  glide ratio, approach len, loiter dir, lookahead, start alt, sink bias,
  auto-replan, sim speed) change every init value and re-plan.
- "Replan now" / auto-replan re-close from the current state (measured glide ratio).
- **Drag the glider dot** to inject a gust/push, then Replan to re-close.

The homing heading is derived (no slider); it's shown read-only in the title.
Needs an interactive matplotlib backend; if none, it saves `path_plot_v1.png` and
prints: `sudo apt-get install python3-tk` (Linux) / `pip install pyqt5`.

## Status
- Plan: lands exactly on target at the into-wind heading; residual 0 (continuous sweep).
- Replan: lands on target geometrically; residual small & bounded by the loop
  quantum, re-closed each replan.
- Clean closed-loop tracking ≈ 4–5 m miss (kink at the loiter entry is the main
  transient). Known: auto-replan on a fixed cadence accumulates a little
  along-track lag — hysteresis is the fix if needed.
