# Table of Contents

- [nav_bringup](#nav_bringup)
- [Overview](#overview)
- [Package Structure](#package-structure)
- [Launch Architecture](#launch-architecture)
- [Execution Flow](#execution-flow)
- [Prerequisites](#prerequisites)
- [How to Launch](#how-to-launch)
  - [Localization mode (map-based)](#localization-mode-map-based)
  - [SLAM mode](#slam-mode)
- [Launch Arguments](#launch-arguments)
- [Operational Notes](#operational-notes)
- [Troubleshooting](#troubleshooting)
- [References](#references)

# nav_bringup

## Overview

`nav_bringup` is the package-level entry point for running Nav2 with Isaac Sim in this workspace.
Its purpose is to provide one main launch file that orchestrates the full navigation stack while keeping localization and navigation logic modular.

Core idea:
- `bringup_launch.py` is the top-level orchestrator.
- It internally includes `localization_launch.py` and `navigation_launch.py`.
- This keeps the system easy to customize, test, and maintain.

## Package Structure

```text
src/nav_bringup/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── nav2_params.yaml
├── launch/
│   ├── bringup_launch.py
│   ├── localization_launch.py
│   └── navigation_launch.py
└── maps/
    ├── my_map.yaml
    ├── slam_map.pgm
    └── slam_map.yaml
```

## Launch Architecture

- **`bringup_launch.py`**
  - Main launch entry point.
  - Handles top-level arguments (for example `map`, `slam`, `use_sim_time`, `params_file`).
  - Includes the localization and navigation launch files.
  - Launches RViz2 by default so you do not need a second terminal.

- **`localization_launch.py`**
  - Starts localization components (typically AMCL and map server) when running in map-based mode.
  - Consumes the map YAML argument.

- **`navigation_launch.py`**
  - Starts the core Nav2 runtime (planner, controller, behavior, BT navigator, lifecycle manager, etc.).

## Execution Flow

1. User launches `bringup_launch.py`.
2. Launch arguments are resolved at the top level.
3. If `slam:=false`, `localization_launch.py` is included and loads map-based localization.
4. `navigation_launch.py` is included to start the navigation stack.
5. All nodes run with simulation clock when `use_sim_time:=true`.

This design gives a single command for runtime operation while preserving internal separation of concerns.

## Prerequisites

- ROS 2 Humble environment is sourced.
- Workspace is built and sourced.
- Isaac Sim is running and publishing required TF/odometry topics.
- A valid map YAML is available when using localization mode.

## How to Launch

### Localization mode (map-based)

```bash
ros2 launch nav_bringup bringup_launch.py \
  use_sim_time:=true \
  slam:=false \
  map:=/home/schaeffler/schaeffler/src/nav_bringup/maps/slam_map.yaml
```

### SLAM mode

```bash
ros2 launch nav_bringup bringup_launch.py \
  use_sim_time:=true \
  slam:=true
```

In SLAM mode, a static map file is typically not required.

## Launch Arguments

Common arguments used with `bringup_launch.py`:

- `use_sim_time`: Use Isaac Sim clock (`true`/`false`).
- `slam`: Toggle SLAM mode (`true`) vs map-based localization (`false`).
- `map`: Path to map YAML (used in localization mode).
- `params_file`: Path to Nav2 parameter file (for example custom tuned YAML).
- `rviz`: Whether to launch RViz2 automatically (`true`/`false`).
- `rviz_config`: Path to the RViz config file.

To inspect all available arguments:

```bash
ros2 launch nav_bringup bringup_launch.py --show-args
```

## Operational Notes

- Prefer absolute paths for `map` and `params_file` to avoid path resolution issues.
- Ensure frame IDs in Nav2 parameters match frames published by Isaac Sim.
- Keep `use_sim_time:=true` for simulation-based workflows.
- Use `bringup_launch.py` as the standard entry point; avoid launching localization/navigation independently unless debugging.
- If you do not want RViz on startup, pass `rviz:=false`.

## Troubleshooting

- **No localization / poor pose estimate**
  - Check map path, TF tree consistency, and AMCL-related parameters.
- **Navigation nodes not activating**
  - Verify lifecycle transitions and parameter file validity.
- **Robot not moving in simulation**
  - Confirm command topics, controller parameters, and Isaac Sim bridge connectivity.

## References

- [ROS 2 Navigation Stack (Nav2)](https://navigation.ros.org/)
- [Isaac Sim ROS 2 Bridge](https://docs.nvidia.com/isaac/isaac-sim/)
