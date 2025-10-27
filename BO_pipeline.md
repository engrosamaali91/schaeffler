# Sim2Real calibration — BO pipeline

Summary
- Use Bayesian Optimization (BO) to tune Nav2 parameters in simulation so the simulated behaviour matches a fixed real-world trajectory.
- BO proposes parameter sets; each set is evaluated in Isaac Sim + Nav2 and scored against the real data.

Prerequisites
- Workspace and orchestrator built and sourced:
  - source /opt/ros/humble/setup.bash
  - source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
  - source ~/schaeffler/install/setup.bash
- Scripts:
  - nav2_test.py — runs the 3 m straight test and records odometry to CSV
  - compute_kpi_normalized.py — computes normalized KPI J̃ vs fixed real CSV
- agv_orchestrator launch: `ros2 launch agv_orchestrator isaac_and_nav2.launch.py`

Pipeline (simple)
1. BO proposes a parameter vector (e.g., max_vel_x, acc_lim_x, ...).
2. Create a temporary Nav2 params YAML (or point to an existing file). Example:
   - write `/tmp/nav2_params.yaml` with proposed values
   - or pass a package/local path via `params_file` launch arg
3. Launch the orchestrator with the temporary params:
   - ros2 launch agv_orchestrator isaac_and_nav2.launch.py params_file:=/tmp/nav2_params.yaml rviz:=false
4. Run the test scenario (3 m straight) and record odometry:
   - python3 nav2_test.py --out /tmp/sim_odom.csv
   - (nav2_test.py may call ros2 bags or subscribe to /odom and write CSV)
5. Compute the normalized KPI against the fixed real CSV:
   - python3 compute_kpi_normalized.py --sim /tmp/sim_odom.csv --real /path/to/real_odom.csv
   - output: J̃ (lower is better)
6. Shutdown the orchestrator cleanly (terminate the launch process).
7. Return reward = -J̃ to BO (BO maximizes reward).
8. BO repeats steps 1–7 until J̃ < 1 (early stop) or max iterations reached.

Notes / tips
- Use absolute paths for temp files when invoking ros2 launch to avoid cwd issues.
- Make the orchestrator launch arguments (params_file, rviz, nav2_delay) configurable for automation.
- Enforce timeouts for each evaluation to prevent stuck runs.
- Log each trial: params, J̃, sim CSV path, timestamps.
- Clean up temp files after each trial.

Example loop (pseudocode)
- for iter in BO:
    - params = BO.propose()
    - write /tmp/nav2_params.yaml
    - launch orchestrator (subprocess)
    - run nav2_test.py -> /tmp/sim_odom.csv
    - Jt = compute_kpi_normalized(/tmp/sim_odom.csv, real.csv)
    - terminate orchestrator
    - BO.update(params, reward=-Jt)
    - if Jt < 1: break

This document describes the minimal, repeatable BO evaluation flow to perform Sim2Real calibration.