# Table of Contents

- [Table of Contents](#table-of-contents)
- [Nav2 Parameter Optimization](#nav2-parameter-optimization)
	- [Overview](#overview)
	- [Bayesian Optimization Pipeline (Scenario 1 \& 2)](#bayesian-optimization-pipeline-scenario-1--2)
	- [Scenario Differences (S1 vs S2)](#scenario-differences-s1-vs-s2)
	- [Parameter Initialization](#parameter-initialization)
	- [Search Space (pbounds)](#search-space-pbounds)
	- [Optimization Loop](#optimization-loop)
	- [In Summary — Key Motion Parameters](#in-summary--key-motion-parameters)
	- [Concept](#concept)

# Nav2 Parameter Optimization

## Overview

This workflow automates Nav2 tuning to reduce the **Sim2Real gap** using Bayesian Optimization.

Two scripts are used for the same pipeline:
- `BO.py` for Scenario 1
- `BO_s2.py` for Scenario 2

Both scripts follow one common loop: generate parameters, run a full sim test via `agv_orchestrator`, read KPI `J`, log results, and let Bayesian Optimization propose the next candidate.

Search ranges are defined directly inside each Python script through `pbounds` (not through `bounds.yaml`).

## Bayesian Optimization Pipeline (Scenario 1 & 2)

1. Start from a base Nav2 template (`nav2_params.yaml` or `nav2_params_modified.yaml`).
2. Apply optimizer-proposed values into key controller and velocity smoother fields.
3. Write an override file (`nav2_params_bo.yaml`) used only for that iteration.
4. Launch one full run:
	- `ros2 launch agv_orchestrator isaac_and_nav2.launch.py`
	- `rviz:=true run_test:=true compute_kpi:=true`
	- `params_file:=<absolute path to nav2_params_bo.yaml>`
5. Read the latest KPI file `logs/J_nav2_run_*.txt` associated with the newest run data.
6. Append the trial to `bo_evals.csv` (iteration, parameters, KPI).
7. Return `-J` to the optimizer (because `bayes_opt` maximizes while we want to minimize `J`).
8. Repeat for initialization points and BO-guided iterations, then report the best configuration.

## Scenario Differences (S1 vs S2)

- **Tuning dimension**: Scenario 1 optimizes 2 variables (`max_vel_x`, `acc_lim_x`); Scenario 2 optimizes 4 (adds `max_vel_theta`, `acc_lim_theta`).
- **Template**: Scenario 1 uses `nav2_params.yaml`; Scenario 2 uses `nav2_params_modified.yaml`.
- **Search focus**: Scenario 2 uses tighter, manually informed low-acceleration bounds.
- **KPI fallback behavior**: Scenario 2 returns penalty `J=10.0` on KPI timeout; Scenario 1 raises timeout error.
- **CSV columns**: Scenario 2 logs angular parameters in addition to linear ones.


## Parameter Initialization
All initial parameters in `nav2_params.yaml` are derived from the real robot’s configuration:

| Robot UI Param | Converted Nav2 Param | Units | Source |
|----------------|----------------------|--------|---------|
| TransVelMax = 1200 mm/s | max_vel_x = 1.20 | m/s | MobilePlanner |
| TransAccel = 300 mm/s² | acc_lim_x = 0.30 | m/s² | MobilePlanner |
| RotVelMax = 60 deg/s | max_vel_theta = 1.05 | rad/s | MobilePlanner |
| RotAccel = 90 deg/s² | acc_lim_theta = 1.57 | rad/s² | MobilePlanner |

## Search Space (pbounds)

The optimizer search space is defined in code (`pbounds`) in each script:

- **Scenario 1 (`BO.py`)**
	- `max_vel_x`: `0.20` to `0.40`
	- `acc_lim_x`: `2.0` to `3.0`

- **Scenario 2 (`BO_s2.py`)**
	- `max_vel_x`: `0.12` to `0.22`
	- `acc_lim_x`: `0.02` to `0.22`
	- `max_vel_theta`: `0.10` to `0.50`
	- `acc_lim_theta`: `0.03` to `0.50`

This means each BO iteration proposes parameters only inside these ranges.

## Optimization Loop
1. Start with `nav2_params.yaml`
2. Run Nav2 → compute J
3. Optimizer proposes new params within script-defined `pbounds`
4. Write `nav2_params_bo.yaml`
5. Relaunch → recompute J → iterate



| Robot UI Parameter (from MobilePlanner) | Meaning / Units                    | Converted Nav2 Parameter(s)                                                         | Converted Value (from Robot)              | Default Value (Nav2 file) |
| --------------------------------------- | ---------------------------------- | ----------------------------------------------------------------------------------- | ----------------------------------------- | ------------------------- |
| **TransVelMax = 1200 mm/s**             | Max forward translational velocity | `controller_server.FollowPath.max_vel_x`<br>`velocity_smoother.max_velocity[0]`     | **1.20 m/s**                              | **0.26 m/s**              |
| **TransNegVelMax = -200 mm/s**          | Max reverse translational velocity | `controller_server.FollowPath.min_vel_x`<br>`velocity_smoother.min_velocity[0]`     | **-0.20 m/s**                             | **-0.26 m/s**             |
| **TransAccel = 300 mm/s²**              | Max linear acceleration            | `controller_server.FollowPath.acc_lim_x`<br>`velocity_smoother.max_accel[0]`        | **0.30 m/s²**                             | **2.50 m/s²**             |
| **TransDecel = 600 mm/s²**              | Max linear deceleration            | `controller_server.FollowPath.decel_lim_x`<br>`velocity_smoother.max_decel[0]`      | **-0.60 m/s²** (negative in controller)   | **-2.50 m/s²**            |
| **RotVelMax = 60°/s**                   | Max angular velocity               | `controller_server.FollowPath.max_vel_theta`<br>`velocity_smoother.max_velocity[2]` | **1.05 rad/s**                            | **1.00 rad/s**            |
| **RotAccel = 90°/s²**                   | Angular acceleration               | `controller_server.FollowPath.acc_lim_theta`<br>`velocity_smoother.max_accel[2]`    | **1.57 rad/s²**                           | **3.20 rad/s²**           |
| **RotDecel = 90°/s²**                   | Angular deceleration               | `controller_server.FollowPath.decel_lim_theta`<br>`velocity_smoother.max_decel[2]`  | **-1.57 rad/s²** (negative in controller) | **-3.20 rad/s²**          |
| *(derived)*                             | Goal tolerance radius              | `controller_server.FollowPath.xy_goal_tolerance`                                    | —                                         | **0.25 m**                |
| *(derived)*                             | Time horizon for local planner     | `controller_server.FollowPath.sim_time`                                             | —                                         | **1.7 s**                 |
| *(derived)*                             | Linear stop detection threshold    | `controller_server.FollowPath.trans_stopped_velocity`                               | ~0.05 × `max_vel_x` ≈ **0.06 m/s**        | **0.25 m/s**              |



## In Summary — Key Motion Parameters

| Term | Intuitive Meaning | Formula / Effect |
|------|--------------------|------------------|
| **max_vel_x** | “How fast can I go?” | Defines the **maximum linear speed** the robot can reach (m/s). |
| **acc_lim_x** | “How quickly can I get there?” | Sets the **rate of acceleration** (m/s²). A higher value → snappier motion; lower value → smoother, slower ramp-up. |
| **Ramp time** | “How long does it take to reach top speed?” | ```t = v_max / a_lim```  →  shorter = snappier motion, longer = smoother motion |



## Concept

During navigation, the robot begins each motion at 0 m/s and ramps its velocity until reaching max_vel_x, respecting the limit imposed by ```acc_lim_x```.
The pair (```max_vel_x, acc_lim_x```) directly shapes how aggressively or smoothly the robot accelerates and decelerates in simulation versus reality.

In the Sim2Real optimization, these are the primary tuning knobs:

```max_vel_x``` controls the steady-state speed the simulated AGV maintains.

```acc_lim_x``` controls the shape and timing of the velocity curve (the ramp).

The goal of optimization is to find parameter values that reproduce the real robot’s acceleration profile and motion timing inside simulation, minimizing the measured Sim2Real gap (J).

