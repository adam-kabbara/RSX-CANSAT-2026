# RSX CanSat 2026 — Descent Path Guidance (v1)

Three-phase heuristic descent guidance for the autonomous paraglider.

```
Phase 1  Homing    straight descending leg   start -> loiter entry (pt2)
Phase 2  Loiter     constant-radius descending helix, burns the leftover
                    altitude budget (continuous sweep angle Theta)
Phase 3  Approach  straight descending leg   loiter exit (pt3) -> landing,
                    tangent to the desired (into-wind) landing heading
```

The whole reference path is single-valued in altitude (NED, z = D, down,
monotonically increasing), so it is `P(D) -> (N, E)`. Guidance is **pure
pursuit**: chase a carrot point `lookahead_drop` metres of altitude below the
glider, on that path. `get_heading()` returns the ground-track heading for the
rate controller; descent comes from the pitch loop / glide.

## Files
- `PathGuidance.hpp/.cpp` — the guidance class. Portable, `float`-only, no heap,
  no exceptions. **This is what goes into the STM32CubeIDE project.**
- `bindings.cpp` — pybind11 wrapper (desktop only).
- `sim.py` — closed-loop simulator + matplotlib debug plot.
- `CMakeLists.txt` — desktop build of the Python module.

## Build & run the debug front-end

### Prerequisites
- [Miniconda](https://docs.conda.io/en/latest/miniconda.html) or any Python 3.8+ environment
- [CMake](https://cmake.org/download/) 3.15+
- **Windows**: [Visual Studio 2022 Build Tools](https://aka.ms/vs/17/release/vs_BuildTools.exe)
  — install with the **"Desktop development with C++"** workload

Install Python dependencies into your environment:
```
pip install pybind11 matplotlib numpy
```

### Windows (PowerShell)

> Make sure you are using your conda/venv Python, **not** the msys64 one.
> Adjust paths below if your env name or Python version differs.

```powershell
# 1. Check the pybind11 cmake dir for your environment
C:\Users\<you>\miniconda3\envs\cansat\python.exe -m pybind11 --cmakedir

# 2. Configure (replace python313.lib if your Python version differs —
#    check with: dir C:\Users\<you>\miniconda3\envs\cansat\libs\)
cmake -B build -G "Visual Studio 17 2022" `
  -Dpybind11_DIR="C:\Users\<you>\\miniconda3\envs\cansat\Lib\site-packages\pybind11\share\cmake\pybind11" `
  -DPYTHON_EXECUTABLE="C:\Users\<you>\miniconda3\envs\cansat\python.exe" `
  -DPYTHON_LIBRARIES="C:\Users\<you>\miniconda3\envs\cansat\libs\python313.lib" `
  -DPYTHON_INCLUDE_DIR="C:\Users\<you>\miniconda3\envs\cansat\include"

# 3. Build
cmake --build build --config Release

# 4. Copy the extension module next to sim.py
copy build\Release\pathguidance*.pyd .

# 5. Run
C:\Users\<you>\miniconda3\envs\cansat\python.exe sim.py        # writes path_plot.png
```

### Linux / macOS

```bash
# Configure & build
cmake -B build -Dpybind11_DIR=$(python3 -m pybind11 --cmakedir)
cmake --build build

# Copy the extension module next to sim.py
cp build/pathguidance*.so .       # Linux
# cp build/pathguidance*.dylib .  # macOS (if needed)

python3 sim.py        # writes path_plot.png
```

On macOS, if using the one-liner instead of CMake, replace `-shared` with
`-undefined dynamic_lookup -dynamiclib`.

---

## Using it in the flight controller (STM32CubeIDE)
Add `PathGuidance.hpp/.cpp` to the project (no other dependencies). The class is
`float`-only on purpose: the G431 FPU is single-precision, so keep feeding it
floats and don't introduce `double` math around it.

```cpp
rsx::GuidanceParams p;            // fill from mission config
rsx::PathGuidance   guide(p);
guide.plan();                     // call once, and again at each main waypoint

// every control tick, from the EKF:
rsx::State s; /* n,e,d, vn,ve,vd, roll,pitch,yaw (rad) */
rsx::HeadingCmd cmd = guide.getHeading(s);
// cmd.heading -> heading-error -> desired roll -> your roll rate controller
```

`plan()` does the (rare) geometry solve — bounded, allocation-free bisection.
`getHeading()` is a handful of trig calls per tick.

## Tuning note
The stability knob in pure pursuit is `lookahead_drop` relative to the turn
radius. Effective horizontal lookahead ≈ `lookahead_drop * glide_ratio`; keep it
on the order of `min_turn_radius`. Too small → oscillation; too large → corner
cutting.

## Known v1 limitation / next step
Transitions are **C1 (tangent)**, not yet C2 (curvature). The homing→loiter
entry has a heading corner; the carrot low-passes it but it shows up as a
cross-track transient at loiter entry in `path_plot.png`. The fix is an Euler
spiral (clothoid) entry blend — that's the planned v2 addition. The loiter exit
is already tangent to the approach line.