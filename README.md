# NMPC for PX4-ROS2 Deployment (C++)
![Status](https://img.shields.io/badge/Status-Hardware_Validated-blue)
[![ROS 2 Compatible](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![PX4 Compatible](https://img.shields.io/badge/PX4-Autopilot-pink)](https://github.com/PX4/PX4-Autopilot)
[![evannsmc.com](https://img.shields.io/badge/evannsmc.com-Project%20Page-blue)](https://www.evannsmc.com/projects)

C++ port of [`nmpc_acados_px4`](https://github.com/evannsm/nmpc_acados_px4) — NMPC for quadrotors using the [Acados](https://docs.acados.org/) solver. Same Euler-state formulation, error-based cost, and wrapped yaw error as the Python package, implemented in C++ against the Acados C API.

This package was created during a PhD at Georgia Tech's FACTSLab as a high-performance C++ counterpart to the Python NMPC baseline used for comparisons against Newton-Raphson Flow controllers.

> **Critical:** The Python package must be run at least once before building this package. The Python run generates the C solver code that this package compiles against. If you skip this step, `colcon build` will fail with a clear error message.

## Key Features

- **Acados C API** — links directly against the generated C solver for minimal overhead
- **Dual-timer control loop** — decouples MPC solve latency from the publish rate via a 50-step control buffer
- **Error-state cost formulation** — references passed as stage-wise parameters, not embedded in the cost
- **Differential-flatness feedforward** — `--ff` flag enables full feedforward state+control for `f8_contraction` via `autodiff::real3rd`
- **Input constraints** — hard bounds on thrust `[0, 27] N` and body rates `[-0.8, 0.8] rad/s`
- **PX4 integration** — publishes attitude setpoints and offboard commands via `px4_msgs`
- **Structured logging** — optional CSV logging via ros2_logger_cpp

## MPC Formulation

| Parameter | Value |
|-----------|-------|
| State | 9D `[x, y, z, vx, vy, vz, roll, pitch, yaw]` |
| Control | 4D `[thrust (N), p, q, r (rad/s)]` |
| Horizon | 2.0 s, N=50 steps, dt=0.04 s |
| Solver | SQP_RTI, PARTIAL_CONDENSING_HPIPM, ERK |
| Cost type | NONLINEAR_LS, error-based |
| Yaw error | `atan2(sin(yaw−yaw_ref), cos(yaw−yaw_ref))` |
| Thrust bounds | `[0, 27] N` |
| Rate bounds | `[−0.8, 0.8] rad/s` |

## Build Order (Do This First)

### Step 1 — Generate the Acados C solver (Python)

The C++ package links against a shared library produced by the Python package's code-generation step. This only needs to be done once (or whenever the MPC formulation changes).

```bash
source install/setup.bash

# Any trajectory/platform works — we just need the solver to be generated.
ros2 run nmpc_acados_px4 run_node --platform sim --trajectory hover --hover-mode 1
```

The node will print:

```
[acados] Generating/compiling fresh MPC (euler_err)...
[acados] Done! Control stack should begin in two seconds...
```

Press `Ctrl+C` to stop. The generated files are now at:

```
src/nmpc_acados_px4/nmpc_acados_px4_utils/controller/nmpc/acados_generated_files/
└── holybro_euler_err_mpc_c_generated_code/
    ├── acados_solver_holybro_euler_err.h          ← C++ includes this
    ├── libacados_ocp_solver_holybro_euler_err.so  ← C++ links against this
    └── ...
```

### Step 2 — Build the C++ package

```bash
colcon build --packages-select nmpc_acados_px4_cpp
source install/setup.bash
```

If Step 1 was skipped, CMake will stop with:

```
[nmpc_acados_px4_cpp] Generated solver header not found at: ...
Run the Python package first to generate the acados C solver.
```

### Step 3 — Run

```bash
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory helix --spin
```

## Rebuilding After MPC Formulation Changes

If you modify `acados_model.py` or `generate_nmpc.py` in the Python package (e.g., change weights, horizon, or constraints), the generated C code is stale. Regenerate:

```bash
# Force regeneration by deleting the old files
rm -rf src/nmpc_acados_px4/nmpc_acados_px4_utils/controller/nmpc/acados_generated_files/

# Re-run the Python node to regenerate
ros2 run nmpc_acados_px4 run_node --platform sim --trajectory hover --hover-mode 1
# Ctrl+C after "Done! Control stack should begin..."

# Rebuild the C++ package
colcon build --packages-select nmpc_acados_px4_cpp
source install/setup.bash
```

## Usage

```bash
# Simulation
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory circle_horz
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory helix --spin --double-speed
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory fig8_vert --short

# Figure-8 contraction (no feedforward)
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory f8_contraction

# Figure-8 contraction with differential-flatness feedforward
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory f8_contraction --ff

# Hardware with logging
ros2 run nmpc_acados_px4_cpp run_node --platform hw --trajectory circle_horz --log
ros2 run nmpc_acados_px4_cpp run_node --platform hw --trajectory helix --spin --log --log-file my_run
ros2 run nmpc_acados_px4_cpp run_node --platform hw --trajectory f8_contraction --ff --log
```

### CLI Options

| Flag | Description |
|------|-------------|
| `--platform {sim,hw}` | Target platform **(required)** |
| `--trajectory TRAJ` | Trajectory type **(required)** |
| `--hover-mode N` | Hover sub-mode (required when `--trajectory=hover`) |
| `--double-speed` | 2× trajectory speed |
| `--short` | Short variant (fig8_vert only) |
| `--spin` | Enable yaw rotation (circle_horz, helix) |
| `--ff` | Differential-flatness feedforward (only valid with `f8_contraction`) |
| `--flight-period SEC` | Override default duration (sim: 30 s, hw: 60 s) |
| `--log` | Enable CSV data logging |
| `--log-file NAME` | Custom log filename stem (requires `--log`) |

**Trajectories:** `hover`, `yaw_only`, `circle_horz`, `circle_vert`, `fig8_horz`, `fig8_vert`, `helix`, `sawtooth`, `triangle`, `f8_contraction`

## Feedforward for `f8_contraction`

Pass `--ff` to enable differential-flatness feedforward when running the `f8_contraction` trajectory.

**How it works:**

1. `generate_feedforward_trajectory` (in `quad_trajectories_cpp`) evaluates `flat_to_x_u` at each of the `N` horizon time steps.
2. `flat_to_x_u` uses a single `autodiff::real3rd` forward pass to recover all derivatives up to 3rd order from the trajectory function, then computes:
   - Velocity `[vx, vy, vz]` (1st derivative)
   - Specific thrust `f = sqrt(ax² + ay² + (az - g)²)` and Euler angles `[phi, th, psi]` (from 2nd derivatives)
   - `u_ff = [df, dphi, dth, dpsi]` via chain rule on the 3rd derivatives (jerk)
3. The NMPC reference is updated per stage:
   - **Euler reference** columns 6–7 (`roll_ref`, `pitch_ref`) are replaced with the feedforward `[phi, th]` instead of zeros — the NMPC tracks the physically correct attitude the trajectory demands.
   - **Control reference** `u_ref = [F (N), p, q, r]` is computed by converting `[dphi, dth, dpsi]` to body rates via the inverse ZYX Euler kinematic matrix, and passed as the stage parameter instead of hover thrust.

This gives the NMPC a fully consistent reference in both state and control space rather than treating every stage as a hover equilibrium.

## Architecture

### Two-timer control loop

| Timer | Rate | Role |
|-------|------|------|
| `compute_control_timer` | 100 Hz | Runs MPC solve, writes to 50-step control buffer |
| `publish_control_timer` | 100 Hz | Reads from control buffer and publishes one step |
| `offboard_mode_timer` | 10 Hz | Manages arm/offboard/heartbeat |

The control buffer decouples solver latency from the publish rate. If the MPC solve takes >10 ms, the buffer plays back the previous solution until the next solve completes.

### Flight phases

```
t=0         t=10s              t=10+flight_period    t=10+flight_period+10s
|-- HOVER --|------ CUSTOM ----|------ RETURN --------|-- LAND --|
   position     body-rate ctrl     position setpoint    descend
```

## Package Structure

```
nmpc_acados_px4_cpp/
├── CMakeLists.txt
├── package.xml
├── include/nmpc_acados_px4_cpp/
│   ├── nmpc_solver.hpp              # Acados solver wrapper class
│   ├── offboard_control_node.hpp    # ROS 2 node class
│   ├── px4_utils/
│   │   ├── core_funcs.hpp           # PX4 interface helpers
│   │   └── flight_phases.hpp        # Flight phase state machine
│   └── transformations/
│       └── adjust_yaw.hpp           # Yaw wrapping utilities
└── src/
    ├── nmpc_solver.cpp              # Acados C API solver implementation
    ├── offboard_control_node.cpp    # ROS 2 node (subscriptions, publishers, control loop)
    └── run_node.cpp                 # CLI entry point and argument parsing
```

## Dependencies

- [quad_platforms_cpp](https://github.com/evannsm/quad_platforms_cpp) — platform mass and throttle mapping
- [quad_trajectories_cpp](https://github.com/evannsm/quad_trajectories_cpp) — trajectory definitions and autodiff velocities
- [ros2_logger_cpp](https://github.com/evannsm/ROS2Logger_cpp) — CSV logging
- [px4_msgs](https://github.com/PX4/px4_msgs) — PX4 message types
- [nmpc_acados_px4](https://github.com/evannsm/nmpc_acados_px4) — Python package for C solver code generation
- Eigen3
- [Acados](https://docs.acados.org/) — installed at `~/acados/`

## Installation

```bash
# Inside a ROS 2 workspace src/ directory
git clone git@github.com:evannsm/nmpc_acados_px4_cpp.git
# Run the Python package first to generate the Acados C solver (see Build Order above)
cd .. && colcon build --packages-select nmpc_acados_px4_cpp
```

## Acados Installation

If Acados is not yet installed, follow the [Python package README](https://github.com/evannsm/nmpc_acados_px4#acados-setup) for the full setup steps, or the quick summary below.

```bash
git clone https://github.com/acados/acados.git ~/acados
cd ~/acados && git submodule update --recursive --init
mkdir build && cd build
cmake -DACADOS_WITH_QPOASES=ON .. && make install -j$(nproc)

pip install -e ~/acados/interfaces/acados_template

# Add to ~/.bashrc
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/acados/lib
export ACADOS_SOURCE_DIR=$HOME/acados
```

Download the `t_renderer` binary from [tera_renderer releases](https://github.com/acados/tera_renderer/releases/), place it at `~/acados/bin/t_renderer`, and `chmod +x` it.

## Papers and Repositories

American Control Conference 2024 — [paper](https://coogan.ece.gatech.edu/papers/pdf/cuadrado2024tracking.pdf)
| [Personal repo](https://github.com/evannsm/MoralesCuadrado_ACC2024)
| [FACTSLab repo](https://github.com/gtfactslab/MoralesCuadrado_Llanes_ACC2024)

Transactions on Control Systems Technology 2025 — [paper](https://arxiv.org/abs/2508.14185)
| [Personal repo](https://github.com/evannsm/MoralesCuadrado_Baird_TCST2025)
| [FACTSLab repo](https://github.com/gtfactslab/Baird_MoralesCuadrado_TRO_2025)

Transactions on Robotics 2025
| [Personal repo](https://github.com/evannsm/MoralesCuadrado_Baird_TCST2025)
| [FACTSLab repo](https://github.com/gtfactslab/MoralesCuadrado_Baird_TCST2025)

## License

MIT

## Website

This project is part of the [evannsmc open-source portfolio](https://www.evannsmc.com/projects).
