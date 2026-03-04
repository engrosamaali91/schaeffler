# Table of Contents

- [AGV Orchestrator](#agv-orchestrator)
- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Build and Environment Setup](#build-and-environment-setup)
- [Launch Usage](#launch-usage)
  - [1) Default launch](#1-default-launch)
  - [2) Launch with custom USD and Nav2 parameters](#2-launch-with-custom-usd-and-nav2-parameters)
  - [3) Launch with RViz enabled](#3-launch-with-rviz-enabled)
  - [4) Full test + KPI workflow](#4-full-test--kpi-workflow)
- [Launch Arguments](#launch-arguments)
- [Notes](#notes)


# AGV Orchestrator

`agv_orchestrator` provides a single ROS 2 launch entry point to start Isaac Sim (Omron Emma scene) together with Nav2 bringup. It is intended to run end-to-end simulation and navigation workflows with minimal manual orchestration.

## Overview

This package coordinates assets from two workspaces:

- Isaac Sim ROS workspace: `~/IsaacSim-ros_workspaces/humble_ws`
- Local simulation workspace: `~/schaeffler`

Main responsibilities:

- Launch Isaac Sim with a configurable USD scene.
- Launch Nav2 with configurable parameter and map files.
- Optionally launch RViz.
- Optionally execute automated test motion and KPI computation.

## Prerequisites

Ensure the following are available before launching:

- ROS 2 Humble
- Isaac Sim with ROS bridge support
- Built Isaac Sim ROS workspace (`~/IsaacSim-ros_workspaces/humble_ws`)
- Built local workspace containing `agv_orchestrator`
- Nav2 bringup package (`nav_bringup`) in the local workspace
- Valid USD scene (example: `/home/schaeffler/Downloads/omron_emma/emma.usd`)

## Build and Environment Setup

Source base environments:

```bash
source /opt/ros/humble/setup.bash
source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
```

Build and source the local workspace:

```bash
cd ~/schaeffler
colcon build --packages-select agv_orchestrator
source ~/schaeffler/install/setup.bash
```

## Launch Usage

### 1) Default launch

```bash
ros2 launch agv_orchestrator isaac_and_nav2.launch.py
```

### 2) Launch with custom USD and Nav2 parameters

```bash
ros2 launch agv_orchestrator isaac_and_nav2.launch.py \
  usd_path:=/home/schaeffler/Downloads/omron_emma/emma.usd \
  play_sim_on_start:=true \
  params_file:=/path/to/custom_nav2_params.yaml
```

### 3) Launch with RViz enabled

```bash
ros2 launch agv_orchestrator isaac_and_nav2.launch.py rviz:=true run_test:=false
```

### 4) Full test + KPI workflow

```bash
ros2 launch agv_orchestrator isaac_and_nav2.launch.py \
  rviz:=true \
  run_test:=true \
  compute_kpi:=true
```

In this mode, the launch can:

- Run RViz using the configured/default RViz profile.
- Execute test motion (for example, localize near `(0, 0, 0)` and perform straight-line traversal).
- Record odometry outputs to the workspace logs.
- Compute and save KPI values (`J`) to files such as `logs/J_nav2_run_x.txt`.

## Launch Arguments

Inspect all available arguments:

```bash
ros2 launch agv_orchestrator isaac_and_nav2.launch.py --show-args
```

Commonly used arguments:

- `params_file`: path to Nav2 parameters YAML (defaults to package configuration)
- `map`: path to map YAML
- `usd_path`: path to USD scene
- `play_sim_on_start`: start simulation immediately (`true`/`false`)
- `rviz`: enable RViz (`true`/`false`)
- `rviz_config`: custom RViz config path
- `run_test`: execute automated test sequence (`true`/`false`)
- `compute_kpi`: compute KPI after test execution (`true`/`false`)

## Notes

- Use absolute paths for `usd_path`, `params_file`, and `map` to avoid path-resolution issues.
- If Nav2 initializes before simulator topics are ready, tune the startup delay argument defined in `isaac_and_nav2.launch.py`.
- This launch file is designed to cover the complete simulation workflow from startup through evaluation.
