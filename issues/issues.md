# 🤖 Isaac Sim ROS 2 Differential Drive Tuning Issue

## Title
**Conflict: High Damping (Stability) vs. Low Damping (Responsiveness) in Isaac Sim ROS 2 Differential Drive Control**

---

## 📝 Description of the Issue

When attempting to control a custom differential drive robot in **Isaac Sim** using **ROS 2 Nav2** and disabling the internal speed caps, a fundamental conflict arises between physical stability and command responsiveness, primarily governed by the **damping** parameter on the angular drive.

The goal is to allow the robot's movement to be *solely* influenced by the ROS 2 Nav2 stack parameters (e.g., those from `controller_server` and `velocity_smoother`) without interference from Isaac Sim's internal capping.

### ⚙️ Configuration Details

| Parameter | Node | Value Set | Intended Outcome |
| :--- | :--- | :--- | :--- |
| **`maxLinearSpeed`** | `DifferentialController` | **0** | Disable Isaac Sim capping. |
| **`maxAngularSpeed`** | `DifferentialController` | **0** | Disable Isaac Sim capping. |
| **`damping`** | `Angular Drive` | Varied (e.g., **1e9**) | High value used for testing stability. |

### 💥 Observed Conflict

| Scenario | Differential Controller Caps | Angular Drive Damping | Result |
| :--- | :--- | :--- | :--- |
| **1 (Unstable)** | `maxSpeed` = **0** | **High** (e.g., 1e9) | Robot becomes **uncontrollable** and unstable upon receiving velocity commands (`/cmd_vel`). |
| **2 (Unresponsive)** | `maxSpeed` = **0** | **Low/Reduced** | Robot is stable but **unresponsive**; fails to rotate or stop correctly after initial velocity command. |
| **3 (Stable/Capped)** | `maxSpeed` = **> 0** (e.g., 0.5) | **High** (e.g., 1e9) | Robot moves and behaves correctly, but is now constrained by the **Isaac Sim internal cap**, defeating the goal of Nav2-only control. |

### 🎯 Desired Outcome

A configuration in Isaac Sim that allows the `maxLinearSpeed` and `maxAngularSpeed` in the `DifferentialController` node to be set to **0** while maintaining a state of both **stability** and **responsiveness**, thus permitting the **ROS 2 Nav2 stack** to be the sole authority over the robot's velocity limits and motion parameters.

---

## 📚 Reference Link

The full discussion and context for this issue can be found on the NVIDIA Developer Forums:

[Isaac Sim ROS 2 Diff Drive Tuning: Conflict between High Damping (Stability) and Low Damping (Responsiveness)](https://forums.developer.nvidia.com/t/isaac-sim-ros-2-diff-drive-tuning-conflict-between-high-damping-stability-and-low-damping-responsiveness/351293)



Referring to 

[Tuning Joint Drive Gainsation](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/joint_tuning.html)

[Tuning guide](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/ext_isaacsim_robot_setup_gain_tuner.html)


## ✅ Solution

The differential drive controller in isaac sim expects the value unscaled from ros2 subscriber node. To get the exact velocities mapped on the simulated robot, remove the to_scale node from the default differential drive action graph......


# Issue 2 — compute KPI from launch (dynamic latest-log selection)

## Summary
When running the full orchestrator launch (Isaac Sim + Nav2 + optional test), the `compute_kpi_normalized.py` script failed to start reliably from the launch despite being present in the workspace. The script computes RMSE and a combined KPI (J_tilde) from the latest generated simulation log (`nav2_run_*.csv`).

## Root cause
- A plotting import (Matplotlib) in `compute_kpi_normalized.py` triggered a binary ABI mismatch on the system Python: system NumPy (v2.x) was incompatible with the Matplotlib wheel compiled against NumPy 1.x. This caused an immediate ImportError when the launch attempted to start the script. The failure was environmental (Python package ABI), not a logic bug in the KPI code.

## What was tried
- Executed the script manually in the workspace and in the installed location to reproduce the error.
- Verified Python environments: system Python had `numpy 2.x` + `matplotlib` compiled for `numpy 1.x`; a development virtualenv contained compatible versions.
- Tried running KPI under the venv (works), considered changing system packages (rejected to avoid breaking Isaac Sim), and considered lazy-importing plotting.

## Resolution
- Removed top-level Matplotlib imports and disabled plotting by default. The KPI code was adjusted so plotting is optional and not imported during headless execution. This avoids the ABI mismatch under system Python.
- Updated the launch to run the KPI deterministically when the nav test finishes by replacing the TimerAction with an event-driven handler. Example used in the repo:

```python
# Run KPI when nav2_test process exits (example)
RegisterEventHandler(
	OnProcessExit(
		target_action=nav2_test_node,
		on_exit=[ExecuteProcess(cmd=['python3', PathJoinSubstitution([FindPackageShare('agv_orchestrator'), 'scripts', 'compute_kpi_normalized.py'])])],
	)
)
```

The launch uses `PathJoinSubstitution([FindPackageShare('agv_orchestrator'), 'scripts', 'compute_kpi_normalized.py'])` so the script is located via the package install path rather than a hard-coded absolute path.

## Why this works
- The KPI script no longer imports Matplotlib at startup, removing the dependency that caused the ImportError under system Python.\
- The `OnProcessExit` event handler guarantees the KPI script is executed immediately after `nav2_test` finishes (rather than relying on a fixed timer), eliminating race conditions where the script could start too early or not at all.

## Outcome / Behaviour
- Launch sequence: IsaacSim -> Nav2 bringup -> `nav2_test` (runs and records `logs/nav2_run_N.csv`) -> `compute_kpi_normalized.py` executes on `OnProcessExit`, automatically discovers the latest `nav2_run_*.csv`, computes `RMSE_pos`, `RMSE_psi` and `J_tilde`, prints results to stdout, and exits cleanly.
- Verified manually and via launch logs; the KPI process starts and finishes cleanly and prints numeric results.

## Notes for thesis write-up (recommended wording)
The compute-KPI failure was caused by an environment ABI mismatch triggered by plotting libraries. The pragmatic resolution was to decouple plotting from KPI computation (headless mode) and make the launch trigger deterministic using process-exit events. This preserved Isaac Sim's system environment while ensuring reproducible KPI evaluation integrated in the orchestration pipeline.

**ABI clarification:** ABI stands for "Application Binary Interface". In this context it means compiled extension modules (e.g., parts of Matplotlib or NumPy) were built against a different binary interface version of NumPy. That mismatch causes import-time errors. This is unrelated to KPI — KPI is the metric we compute (J_tilde); ABI is a low-level binary compatibility issue in Python extensions.


