# Schaeffler AGV Simulation and Navigation Issues

This document records significant technical issues encountered in the Isaac Sim + ROS 2 Nav2 workflow, including root causes, attempted mitigations, and final resolutions.

## Table of Contents

- [Schaeffler AGV Simulation and Navigation Issues](#schaeffler-agv-simulation-and-navigation-issues)
  - [Table of Contents](#table-of-contents)
  - [Issue 1: Differential Drive Tuning and Goal Tolerance](#issue-1-differential-drive-tuning-and-goal-tolerance)
    - [1.1 Nav2 Goal Tolerance: Robot Stops Short of Exact Goal](#11-nav2-goal-tolerance-robot-stops-short-of-exact-goal)
    - [1.2 Conflict: High Damping (Stability) vs Low Damping (Responsiveness)](#12-conflict-high-damping-stability-vs-low-damping-responsiveness)
  - [Issue 2: KPI Computation from Launch (Latest Log Selection)](#issue-2-kpi-computation-from-launch-latest-log-selection)
    - [Summary](#summary)
    - [Root cause](#root-cause)
    - [What was tried](#what-was-tried)
    - [Resolution](#resolution)
    - [Why this works](#why-this-works)
    - [Outcome](#outcome)
    - [ABI clarification](#abi-clarification)
  - [Issue 3: `psutil` Missing in venv and System-Wide `bayesian-optimization` Conflicts](#issue-3-psutil-missing-in-venv-and-system-wide-bayesian-optimization-conflicts)
    - [Summary](#summary-1)
    - [Root cause](#root-cause-1)
    - [What was tried and rejected](#what-was-tried-and-rejected)
    - [Resolution strategy](#resolution-strategy)
    - [Why this works](#why-this-works-1)
    - [Outcome](#outcome-1)
  - [References](#references)

## Issue 1: Differential Drive Tuning and Goal Tolerance

### 1.1 Nav2 Goal Tolerance: Robot Stops Short of Exact Goal

**Summary**
- The robot stops slightly short of the exact goal coordinate (for example, target `x = 5.0 m`, observed around `x = 4.79 m` with `xy_goal_tolerance = 0.25`, and around `x = 4.89 m` with `xy_goal_tolerance = 0.1`).

**Root cause**
- This behavior is expected in Nav2. A goal is considered reached once the robot enters the configured tolerance region.

**What was tried**
- Reduced `xy_goal_tolerance` from `0.25` to `0.1`.

**Outcome**
- The robot stopped closer to the goal but still within tolerance, as designed.

**Recommendation**
- No action is required unless exact stop-at-point behavior is a strict requirement.

### 1.2 Conflict: High Damping (Stability) vs Low Damping (Responsiveness)

**Problem statement**
- For a custom differential drive robot in Isaac Sim, setting `maxLinearSpeed = 0` and `maxAngularSpeed = 0` (to disable internal caps) created a trade-off between stability and responsiveness governed by angular drive damping.

**Configuration tested**

| Parameter | Node | Value | Intended effect |
| :--- | :--- | :--- | :--- |
| `maxLinearSpeed` | `DifferentialController` | `0` | Disable Isaac Sim velocity capping |
| `maxAngularSpeed` | `DifferentialController` | `0` | Disable Isaac Sim velocity capping |
| `damping` | Angular Drive | Varied (for example `1e9`) | Stabilize wheel behavior |

**Observed behavior**

| Scenario | Controller cap | Damping | Result |
| :--- | :--- | :--- | :--- |
| Unstable | `0` | High (`1e9`) | Robot became unstable when receiving `/cmd_vel` |
| Unresponsive | `0` | Low/reduced | Robot became stable but did not rotate/stop correctly |
| Stable but capped | `> 0` (for example `0.5`) | High (`1e9`) | Good behavior, but Isaac cap overrode Nav2-only control objective |

**Desired outcome**
- Keep Isaac controller caps disabled (`0`) while preserving both stability and responsiveness so Nav2 remains the primary authority.

**Resolution**
- The differential drive controller expects unscaled values from the ROS 2 subscriber path.
- Removing the `to_scale` node from the default differential drive action graph aligned command mapping with expected simulated wheel behavior.

## Issue 2: KPI Computation from Launch (Latest Log Selection)

### Summary
- During full orchestrator launch (Isaac Sim + Nav2 + optional test), `compute_kpi_normalized.py` did not start reliably even though the script existed and was executable.

### Root cause
- A top-level Matplotlib import triggered a Python ABI mismatch in the system environment:
  - System NumPy: `2.x`
  - Matplotlib wheel built against NumPy `1.x`
- This caused an immediate import failure at script startup.

### What was tried
- Reproduced failure by running the script manually from workspace and install locations.
- Compared system and virtual environment package compatibility.
- Considered changing system packages (rejected due to Isaac/ROS stability risk).

### Resolution
- Removed top-level Matplotlib imports from KPI execution path.
- Made plotting optional and disabled by default for headless launch execution.
- Replaced timer-based launch trigger with event-driven execution using `OnProcessExit`.

```python
RegisterEventHandler(
    OnProcessExit(
        target_action=nav2_test_node,
        on_exit=[
            ExecuteProcess(
                cmd=[
                    'python3',
                    PathJoinSubstitution([
                        FindPackageShare('agv_orchestrator'),
                        'scripts',
                        'compute_kpi_normalized.py',
                    ]),
                ]
            )
        ],
    )
)
```

### Why this works
- The script no longer imports incompatible plotting dependencies at startup.
- `OnProcessExit` guarantees KPI execution immediately after `nav2_test` completion.
- Script path is resolved from installed package location, avoiding hard-coded paths.

### Outcome
- Reliable sequence: Isaac Sim -> Nav2 bringup -> `nav2_test` -> KPI computation.
- KPI script selects the latest `logs/nav2_run_*.csv`, computes `RMSE_pos`, `RMSE_psi`, and `J_tilde`, prints results, and exits cleanly.

### ABI clarification
- ABI stands for Application Binary Interface.
- Here, ABI mismatch means compiled Python extensions were built against a different NumPy binary interface than the one available at runtime.

## Issue 3: `psutil` Missing in venv and System-Wide `bayesian-optimization` Conflicts

### Summary
- Two environment-level issues affected BO pipeline integration:
  1. `psutil` was missing in `ros2env`, causing Isaac launch failure.
  2. Installing `bayesian-optimization` system-wide exposed a NumPy/SciPy ABI incompatibility.

### Root cause

**Part 1: Missing `psutil` in `ros2env`**
- `run_isaacsim.py` imports `psutil`.
- System Python had `psutil`, but `~/ros2env` did not.
- Launching from venv executed with venv interpreter and failed with:

```text
ModuleNotFoundError: No module named 'psutil'
```

**Part 2: System-level BO installation conflict**
- System package state at failure time:
  - `numpy 2.2.6` in user site packages
  - `scipy 1.8.0` from Ubuntu distro packages
  - `bayesian-optimization 3.1.0` installed system-wide
- SciPy `1.8.0` expected NumPy `< 1.25.0`; runtime found NumPy `2.x`.
- Import failed due to ABI incompatibility.

### What was tried and rejected
- Verified package locations for system and venv.
- Reproduced SciPy import crash after system-wide BO install.
- Rejected system SciPy/NumPy upgrades to avoid destabilizing ROS 2 and Isaac Sim dependencies.

### Resolution strategy

**1) Fix `psutil` in venv**

```bash
source ~/ros2env/bin/activate
pip install "psutil==5.9.0"
```

**2) Isolate BO to `ros2env`**
- Removed system-wide `bayesian-optimization` installation.
- Kept BO dependencies only inside `ros2env`.
- Ran BO scripts exclusively from `ros2env`.

### Why this works
- Ensures launch-time dependencies are available in the interpreter actually used by `ros2 launch`.
- Avoids modifying system scientific packages required by ROS/Isaac stack.
- Maintains a consistent, isolated optimization environment.

### Outcome
- End-to-end BO workflow runs reliably from `ros2env`:

```bash
source ~/ros2env/bin/activate
cd ~/schaeffler/src/GetSetParams
python3 BO.py
```

- Workflow:
  1. BO proposes Nav2 parameters and writes `nav2_params_bo.yaml`.
  2. Launch runs Isaac Sim + Nav2 test.
  3. KPI script computes `J_tilde` and records result.
  4. BO reads KPI and appends run data to `bo_evals.csv`.

## References

- NVIDIA forum discussion:
  [Isaac Sim ROS 2 Diff Drive Tuning: Conflict between High Damping and Low Damping](https://forums.developer.nvidia.com/t/isaac-sim-ros-2-diff-drive-tuning-conflict-between-high-damping-stability-and-low-damping-responsiveness/351293)
- Isaac Sim joint tuning:
  [Tuning Joint Drive Gains](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/joint_tuning.html)
- Isaac Sim gain tuner guide:
  [Gain Tuner Extension Guide](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/ext_isaacsim_robot_setup_gain_tuner.html)

