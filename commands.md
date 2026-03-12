# NMPC Acados Euler Error Commands (C++)

NMPC controller using an **Euler-angle state representation** with a **wrapped yaw error** in the cost function.

---

## Build Package

```bash
cd ~/ws_px4_work
colcon build --packages-select nmpc_acados_px4_cpp
source install/setup.bash
```

---

## Run Commands

### Simulation

```bash
# Hover (requires --hover-mode)
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory hover --hover-mode 1

# Circle Horizontal
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory circle_horz
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory circle_horz --double-speed
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory circle_horz --spin
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory circle_horz --double-speed --spin

# Circle Vertical
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory circle_vert
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory circle_vert --double-speed

# Figure-8 Horizontal
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory fig8_horz
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory fig8_horz --double-speed

# Figure-8 Vertical
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory fig8_vert
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory fig8_vert --double-speed
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory fig8_vert --short
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory fig8_vert --double-speed --short

# Helix
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory helix
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory helix --double-speed
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory helix --spin
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory helix --double-speed --spin

# Yaw Only
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory yaw_only
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory yaw_only --double-speed

# Sawtooth
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory sawtooth
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory sawtooth --double-speed

# Triangle
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory triangle
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory triangle --double-speed

# Figure-8 Contraction (no feedforward)
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory f8_contraction

# Figure-8 Contraction with differential-flatness feedforward
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory f8_contraction --ff
```

---

### Hardware

```bash
# Hover (modes 1–4 only on hardware)
ros2 run nmpc_acados_px4_cpp run_node --platform hw --trajectory hover --hover-mode 1
ros2 run nmpc_acados_px4_cpp run_node --platform hw --trajectory hover --hover-mode 2
ros2 run nmpc_acados_px4_cpp run_node --platform hw --trajectory hover --hover-mode 3
ros2 run nmpc_acados_px4_cpp run_node --platform hw --trajectory hover --hover-mode 4

# Circle Horizontal
ros2 run nmpc_acados_px4_cpp run_node --platform hw --trajectory circle_horz
ros2 run nmpc_acados_px4_cpp run_node --platform hw --trajectory circle_horz --double-speed

# Helix
ros2 run nmpc_acados_px4_cpp run_node --platform hw --trajectory helix
ros2 run nmpc_acados_px4_cpp run_node --platform hw --trajectory helix --double-speed

# Figure-8 Contraction
ros2 run nmpc_acados_px4_cpp run_node --platform hw --trajectory f8_contraction
ros2 run nmpc_acados_px4_cpp run_node --platform hw --trajectory f8_contraction --ff
```

---

## With Logging

Add `--log` to enable data logging.
If no filename is provided, a log name is **auto-generated** from the configuration.

```bash
# Auto-generated filename: sim_nmpc_acados_px4_helix_2x_spin_cpp.csv
ros2 run nmpc_acados_px4_cpp run_node \
  --platform sim \
  --trajectory helix \
  --double-speed \
  --spin \
  --log

# Auto-generated filename: sim_nmpc_acados_px4_f8_contraction_ff_1x_cpp.csv
ros2 run nmpc_acados_px4_cpp run_node --platform sim --trajectory f8_contraction --ff --log

# Hardware with custom filename
ros2 run nmpc_acados_px4_cpp run_node \
  --platform hw \
  --trajectory f8_contraction \
  --ff \
  --log \
  --log-file my_f8_ff_run
```

---

## Arguments Reference

| Argument           | Required | Values | Description |
| ------------------ | -------- | ------ | ----------- |
| `--platform`       | Yes      | `sim`, `hw` | Platform type |
| `--trajectory`     | Yes      | `hover`, `yaw_only`, `circle_horz`, `circle_vert`, `fig8_horz`, `fig8_vert`, `helix`, `sawtooth`, `triangle`, `f8_contraction` | Trajectory type |
| `--hover-mode`     | If hover | `1–8` (sim), `1–4` (hw) | Hover position |
| `--double-speed`   | No       | flag | Use 2× trajectory speed |
| `--short`          | No       | flag | Short `fig8_vert` variant |
| `--spin`           | No       | flag | Enable yaw rotation |
| `--ff`             | No       | flag | Differential-flatness feedforward (only valid with `f8_contraction`) |
| `--flight-period`  | No       | seconds | Override default duration (sim: 30 s, hw: 60 s) |
| `--log`            | No       | flag | Enable data logging |
| `--log-file`       | No       | string | Custom log filename (no extension) |
