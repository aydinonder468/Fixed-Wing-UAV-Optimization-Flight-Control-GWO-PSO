# Fixed-Wing UAV Autonomous Flight Simulation

A comprehensive MATLAB-based simulation framework for fixed-wing UAV autonomous flight, featuring cascaded PID control, Battery Management System (BMS) integration, meta-heuristic PID auto-tuning (GWO & PSO), and real-time 3D visualization dashboard.

![MATLAB](https://img.shields.io/badge/MATLAB-R2025a-orange)


## Features

- **Cascaded PID Control Architecture** — 7 PID controllers (21 parameters) in a 3-layer hierarchy for longitudinal and lateral control
- **Bank-to-Turn (BTT)** — Rudderless coordinated turns using aileron-only bank angle control
- **Battery Management System** — 4S LiPo model with SOC tracking, voltage protection, and control surface coupling
- **Meta-Heuristic Auto-Tuning** — Grey Wolf Optimizer (GWO) and Particle Swarm Optimization (PSO) for simultaneous 21-parameter PID optimization
- **Multiple Flight Routes** — Mission, Figure-6, and Sharp Square patrol profiles
- **Real-Time 3D Dashboard** — STL mesh support, chase camera, live signal plots, battery monitor, and manual gain tuning
- **Post-Analysis** — 8 analysis figures covering tracking errors, attitude, control inputs, aerodynamics, and energy

## Requirements

- **MATLAB R2025a** or later
- **Control System Toolbox** (for `c2d` discretization)
- **Optimization Toolbox** (optional, for comparison)

## Quick Start

```matlab
% Navigate to project directory
cd FIXED_WING_den1

% Run main simulation (opens UI for control type and route selection)
main_fw
```

### First-Time Setup (Optional)

Run optimization once to generate cached gain files. Subsequent runs will load these instantly:

```matlab
GWO_autotune_pid    % ~14 minutes, saves gwo_best_gains.mat
PSO_autotune_pid    % ~14 minutes, saves pso_best_gains.mat
```

## Usage

### 1. Control Type Selection

When `main_fw` runs, a dialog appears with three options:

| Option | Description |
|--------|-------------|
| **1 — Manual PID** | Uses default gains from `fw_params.m`. Optional CLI input for per-gain tuning. |
| **2 — GWO** | Loads cached GWO-optimized gains from `gwo_best_gains.mat`. Runs simulation instantly. |
| **3 — PSO** | Loads cached PSO-optimized gains from `pso_best_gains.mat`. Runs simulation instantly. |

> If the `.mat` file is missing, the script prompts to run optimization now.

### 2. Route Selection

After control type selection, choose a flight route:

| Route | Description | Duration |
|-------|-------------|----------|
| **Mission (Standard)** | Takeoff → Level → Loiter (R=96m, 2 turns) → Approach → Landing | ~98s |
| **Figure-6 Mission** | Takeoff → Figure-6 pattern (R=80m loop + straight tail) → Landing | ~90s |
| **Sharp Square** | Takeoff → Square patrol (100m sides, R=10m sharp 90° corners, 2 laps) → RTL Landing | ~75s |

### 3. Dashboard

After simulation, a unified dashboard opens with:

- **3D Animation** — Aircraft model (STL or built-in), reference trajectory, chase camera
- **Signal Plots** — Position tracking, errors, attitude, control inputs
- **Battery Monitor** — SOC, voltage, current, power, remaining flight time
- **Control Panel** — Switch between GWO/PSO/Manual, tune gains manually, re-simulate

Click **Start Animation** to play. Use the speed slider (0.1x–100x) to adjust playback.

### 4. Manual Gain Tuning

In the dashboard, select **PID-Manuel** and click **Tune...** to open the gain editor. Adjust Kp/Ki/Kd per PID loop, then click **Apply & Re-Simulate**.

## Project Structure

```
FIXED_WING_den1/
├── main_fw.m                    # Entry point — control type + route selection → sim → UI
├── simulate_fw.m                # Full coupled simulation (UAV dynamics + BMS)
├── simulate_fw_fast.m           # Lightweight sim for GWO/PSO fitness evaluation
├── GWO_autotune_pid.m           # Grey Wolf Optimizer — 21-parameter PID tuning
├── PSO_autotune_pid.m           # Particle Swarm Optimization — 21-parameter PID tuning
├── gwo_best_gains.mat           # Cached GWO-optimized gains (21-element vector)
├── pso_best_gains.mat           # Cached PSO-optimized gains (21-element vector)
│
├── fixedwing/
│   ├── fw_params.m              # Physical, aerodynamic, controller, and trajectory parameters
│   ├── pid_ct.m                 # Continuous-time PID with anti-windup and derivative filter
│   ├── create_plant_fw.m        # State-space model construction and ZOH discretization
│   ├── trajectory_fw.m          # Reference trajectory generation (9 route types)
│   └── apply_pid_gain_vector.m  # Maps 21-element gain vector to 7 PID structs
│
├── bms/
│   ├── bms_params.m             # 4S LiPo battery and BMS protection parameters
│   ├── bms_step_fw.m            # BMS step simulation (SOC, voltage, current, coupling)
│   └── mission_feasibility.m    # Post-sim energy analysis and safety status
│
└── ui/
    ├── fw_monitor.m             # Main dashboard — 3D animation + live plots + controls
    ├── post_analysis_plots_fw.m # 8 post-analysis figures (tracking, attitude, aero, battery)
    └── load_aircraft_mesh.m     # STL mesh loader with built-in procedural fallback
```

## Control Architecture

### Cascaded PID Hierarchy

```
Longitudinal:
  Altitude Error ──[pid_alt]──→ θ_cmd ──[pid_pitch_att]──→ q_ref ──[pid_pitch_rate]──→ δ_e (Elevator)
  Speed Error  ──[pid_speed]──→ δ_t (Throttle)

Lateral (Bank-to-Turn, no rudder):
  Heading Error ──[pid_hdg]──→ φ_cmd ──[pid_roll_att]──→ p_ref ──[pid_roll_rate]──→ δ_a (Aileron)
```

### PID Controllers

| # | Controller | Input | Output | Type |
|---|-----------|-------|--------|------|
| 1 | `pid_alt` | Altitude error (m) | θ_cmd (rad) | PID |
| 2 | `pid_speed` | Speed error (m/s) | δ_t (0–1) | PI |
| 3 | `pid_hdg` | Heading error (rad) | φ_cmd (rad) | PID |
| 4 | `pid_pitch_att` | Pitch error (rad) | q_ref (rad/s) | P/PI |
| 5 | `pid_pitch_rate` | Pitch rate error | δ_e (rad) | Fast PID |
| 6 | `pid_roll_att` | Roll error (rad) | p_ref (rad/s) | P/PI |
| 7 | `pid_roll_rate` | Roll rate error | δ_a (rad) | Fast PID |

### State-Space Plant

**Longitudinal:** `x_lon = [du; dw; dq; dθ]`, `u_lon = [δ_t; δ_e]`
**Lateral:** `x_lat = [dv; dp; dr; dφ]`, `u_lat = [δ_a]`

Continuous-time models are discretized via Zero-Order Hold (`c2d`) at `dt = 0.005s`.

## Optimization

### GWO (Grey Wolf Optimizer)

- **Population:** 60 wolves
- **Iterations:** 80
- **Dimensions:** 21 (7 PIDs × 3 gains)
- **Warm start:** Wolf #1 seeded with manual gains
- **Fitness:** `cost = 1.0·pos_error + 0.5·control_effort + 0.5·oscillation + 0.5·saturation`
- **Runtime:** ~14 minutes

### PSO (Particle Swarm Optimization)

- **Population:** 60 particles
- **Iterations:** 80
- **Inertia:** W = 0.9 → 0.4 (linear decay)
- **Coefficients:** C1 = 2.0, C2 = 2.0
- **Warm start:** Particle #1 seeded with manual gains
- **Runtime:** ~14 minutes

### Performance Comparison

| Metric | Manual | GWO | PSO |
|--------|--------|-----|-----|
| **X RMSE** | 16.38m | **7.33m** | 9.74m |
| **Y RMSE** | 6.73m | **2.28m** | 2.75m |
| **Z RMSE** | 1.48m | 1.80m | 1.87m |
| **Elevator chatter (Δ std)** | 0.0093 | **0.0007** | 0.0025 |
| **Throttle saturation** | 45.0% | **37.5%** | 46.6% |
| **Mission status** | SAFE | SAFE | SAFE |

> GWO produces smoother control signals and better tracking. PSO is a viable alternative with slightly higher chatter.

## Battery Model

**4S LiPo, 3000mAh:**

| Parameter | Value |
|-----------|-------|
| Nominal voltage | 14.8V |
| Full charge | 16.8V |
| Empty | 12.0V |
| Internal resistance | 0.030Ω |
| Max current | 50A |
| Base avionics power | 25W |
| Undervoltage cutoff | 12.5V |

The BMS couples battery state to control effectiveness via `k_avail = I_act / I_req`. When battery is depleted, control surfaces lose authority proportionally.

## Known Limitations

1. **Linear model** — State-space is linearized around trim. Accuracy degrades during aggressive maneuvers (e.g., Sharp Square route).
2. **No rudder** — Bank-to-Turn produces sideslip (β ≈ 20°). A yaw damper or rudder coordination would improve this.
3. **No wind/disturbance** — Atmospheric disturbances are not modeled.
4. **First-order actuator model** — Rate limiting is applied, but second-order actuator dynamics are not modeled.

## Suggested Improvements

- Nonlinear 6-DOF equations of motion
- Yaw rate feedback (yaw damper) for coordinated turns
- Gain scheduling across flight regimes (climb, cruise, descent)
- Model Predictive Control (MPC) for constrained optimization
- Hardware-in-the-Loop (HIL) integration for real flight controller testing



## References

1. Mirjalili, S., Mirjalili, S. M., & Lewis, A. (2014). Grey Wolf Optimizer. *Advances in Engineering Software*, 69, 46–61.
2. Kennedy, J., & Eberhart, R. (1995). Particle Swarm Optimization. *Proceedings of ICNN'95*, 1942–1948.
3. Stevens, B. L., & Lewis, F. L. (2003). *Aircraft Control and Simulation* (2nd ed.). Wiley.
4. Beard, R. W., & McLain, T. W. (2012). *Small Unmanned Aircraft: Theory and Practice*. Princeton University Press.

---

*Built with MATLAB R2025a. For detailed technical documentation, see `PROJE_DOKUMANTASYONU.md`.*
