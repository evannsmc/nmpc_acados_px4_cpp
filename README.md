# nmpc_acados_px4_cpp

C++ port of [`nmpc_acados_px4`](../nmpc_acados_px4/README.md) — NMPC for quadrotors using the [Acados](https://docs.acados.org/) solver. Same Euler-state formulation, error-based cost, and wrapped yaw error, implemented in C++ against the Acados C API.

> **Critical:** The Python package must be run at least once before you can build this package. The Python run generates the C solver code that this package compiles against. If you skip this step, `colcon build` will fail with a clear error message.

---

## Build Order (Do This First)

### Step 1 — Generate the Acados C solver (Python)

The C++ package links against a shared library that is produced by the Python package's code-generation step. This only needs to be done once (or whenever the MPC formulation changes).

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

Then press `Ctrl+C` to stop it. The generated files are now at:

```
src/nmpc_acados_px4/nmpc_acados_px4_utils/controller/nmpc/acados_generated_files/
└── holybro_euler_err_mpc_c_generated_code/
    ├── acados_solver_holybro_euler_err.h   ← C++ includes this
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

---

## Rebuilding After MPC Formulation Changes

If you modify `acados_model.py` or `generate_nmpc.py` in the Python package (e.g., change weights, horizon, or constraints), the generated C code is stale. You must regenerate:

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

---

## Usage

```bash
# Simulation
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory circle_horz
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory helix --spin --double-speed
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory fig8_vert --short

# Hardware with logging
ros2 run nmpc_acados_px4_cpp run_node --platform hw --trajectory circle_horz --log
ros2 run nmpc_acados_px4_cpp run_node --platform hw --trajectory helix --spin --log --log-file my_run
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
| `--flight-period SEC` | Override default duration (sim: 30 s, hw: 60 s) |
| `--log` | Enable CSV data logging |
| `--log-file NAME` | Custom log filename stem (requires `--log`) |

**Trajectories:** `hover`, `yaw_only`, `circle_horz`, `circle_vert`, `fig8_horz`, `fig8_vert`, `helix`, `sawtooth`, `triangle`

---

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

### MPC formulation (mirrors Python package)

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

---

## Dependencies

- [quad_platforms_cpp](../quad_platforms_cpp/) — platform mass and throttle mapping
- [quad_trajectories_cpp](../quad_trajectories_cpp/) — trajectory definitions and autodiff velocities
- [ROS2Logger_cpp](../ROS2Logger_cpp/) — CSV logging
- [px4_msgs](https://github.com/PX4/px4_msgs) — PX4 message types
- Eigen3
- [Acados](https://docs.acados.org/) — installed at `/home/egmc/acados/`
- `nmpc_acados_px4` (Python package) — for code generation

---

## Acados Installation

If Acados is not yet installed, follow the [Python package README](../nmpc_acados_px4/README.md#acados-setup) for the full setup steps, or the quick summary below.

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
# nmpc_acados_px4_cpp
