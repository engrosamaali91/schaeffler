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

# Issue 3 — `psutil` missing in venv and system-wide `bayesian-optimization` conflicts

## Summary
While integrating Bayesian Optimization (BO) into the Isaac Sim + Nav2 + KPI pipeline, two environment-related issues occurred:

1. Isaac Sim failed to start when the orchestrator was run from the `ros2env` virtual environment due to a missing `psutil` module.
2. Installing `bayesian-optimization` system-wide exposed an incompatibility between the system NumPy and SciPy versions, causing SciPy to crash on import.

Both issues were related to Python environment management, not to the core logic of the BO or KPI code.

---

## Root cause

### Part 1 — `psutil` missing in `ros2env`
- `run_isaacsim.py` imports `psutil`.
- System Python already had `psutil 5.9.0` installed, but the `~/ros2env` virtual environment did **not**.
- When the pipeline was started inside `ros2env` and `ros2 launch` was invoked, `run_isaacsim.py` ran under the venv’s Python interpreter, where `psutil` did not exist.
- This produced:
  ```text
  ModuleNotFoundError: No module named 'psutil'

### ⚙️ Part 2 — System-wide bayesian-optimization breaking SciPy

-----

### **The Problem: ABI Mismatch**

  * **System Python State:**
      * **`numpy`**: **2.2.6** (from `~/.local/lib/...`, user-installed)
      * **`scipy`**: **1.8.0** (from `/usr/lib/python3/dist-packages`, Ubuntu package)
      * **`bayesian-optimization`**: **3.1.0** (installed system-wide)
  * **The Conflict:**
      * `bayesian-optimization`'s metadata required `numpy >= 1.25` and `scipy >= 1.0.0`.
      * SciPy **1.8.0** was compiled against an older version of NumPy (specifically, **`< 1.25.0`**).
      * When system Python imported SciPy, it found the user-installed **NumPy 2.2.6** at runtime.
      * **Resulting Error:** The **ABI mismatch** caused SciPy to crash on import:
        ```
        A NumPy version >=1.17.3 and <1.25.0 is required for this version of SciPy (detected version 2.2.6)
        ImportError: cannot import name 'Inf' from 'numpy'
        ```

-----

### **What Was Tried & Rejected**

  * **Verified:** System Python had `psutil`, but `ros2env` did not.
  * **Confirmed:** Installing `bayesian-optimization` system-wide immediately caused system Python to crash upon importing SciPy due to the NumPy 2.x / SciPy 1.8.0 mismatch.
  * **Rejected Solution:** Upgrading system SciPy to match NumPy.
      * **Reason:** ROS 2 and Isaac Sim rely on the distro-provided scientific stack. Modifying these core system packages was deemed **too risky** and could destabilize the environment.

-----

### **✅ Resolution Strategy**

#### 1\. Fix for `psutil` in `ros2env`

The dependency issue for Isaac Sim was resolved by explicitly installing `psutil` into the **`ros2env`** virtual environment:

```bash
source ~/ros2env/bin/activate
pip install "psutil==5.9.0"
```

  * **Verification:** `which pip` pointed to `~/ros2env/bin/pip`, and `pip show psutil` confirmed the venv location.
  * **Outcome:** `run_isaacsim.py` could now execute successfully from `ros2env`.

#### 2\. Strategy for `bayesian-optimization`

To avoid the NumPy/SciPy ABI mismatch, the package was confined:

  * **Removed:** System-wide installation of `bayesian-optimization` was **removed/uninstalled**.
  * **Confined:** `bayesian-optimization` was kept installed **only in `ros2env`**.
  * **Decision:** Do not use system Python for the Bayesian Optimization (BO) logic; always run BO scripts from **`ros2env`**.

-----

### **💡 Why This Works**

  * **`psutil` in `ros2env`:** Ensures all Python processes launched from the venv (including those invoked by `ros2 launch`) have the necessary dependencies.
  * **BO Limited to `ros2env`:**
      * It avoids modifying the core scientific packages in the **system environment**, maintaining stability for ROS 2 and Isaac Sim.
      * The `ros2env` virtual environment contains an **internally consistent** stack of NumPy, SciPy, scikit-learn, and BO, preventing the ABI mismatch.
  * **Clean Separation:**
      * **System-level:** Simulation and Navigation environment (ROS/Isaac).
      * **`ros2env` venv:** Optimization logic (Bayesian Optimization).

-----

### **📈 Outcome / Behaviour**

The complete pipeline now runs correctly from `ros2env`:

```bash
source ~/ros2env/bin/activate
cd ~/schaeffler/src/GetSetParams
python3 BO.py
```

  * **Workflow:**
    1.  **BO (in `ros2env`)** proposes new Nav2 parameters and writes `nav2_params_bo.yaml`.
    2.  `ros2 launch` starts Isaac Sim + Nav2 and runs the test.
    3.  KPI script computes the Sim2Real metric $J_{tilde}$ and writes the result to a log file.
    4.  **BO** reads the latest $J$ value and logs the iteration data in `bo_evals.csv`.
  * **Stability:** System Python remains untouched regarding BO and scientific package upgrades, which keeps the core ROS/Isaac environment stable.

-----

### **✍️ Notes for Thesis Write-up**

The integration of Bayesian Optimization exposed environment-related issues rather than algorithmic problems:

  * **First,** Isaac Sim failed from within the virtual environment because **`psutil` was not installed there**, even though it existed in system Python. The fix was to explicitly add `psutil` to the `ros2env` venv, aligning the venv with Isaac’s Python dependencies.
  * **Second,** installing `bayesian-optimization` system-wide revealed an **ABI mismatch** between the system’s SciPy (compiled against NumPy \< 1.25) and a newer NumPy 2.x version in the user site-packages. Instead of modifying the system scientific stack (which could break ROS 2 and Isaac Sim), the solution was to confine BO to the dedicated **`ros2env` environment**, where NumPy, SciPy, scikit-learn, and BO are internally consistent.

This approach cleanly separates:

1.  **the simulation and navigation environment** (system-level ROS/Isaac), and
2.  **the optimization logic** (inside `ros2env`),

which is important for **reproducibility and stability** of the Sim2Real optimization pipeline.