# agv_orchestrator

agv_orchestrator launches Isaac Sim with the Omron Emma USD scene and starts the Nav2 bringup stack so you can run simulation + navigation with a single command. It orchestrates assets from two separate workspaces:

- Isaac Sim ROS workspace: `~/IsaacSim-ros_workspaces/humble_ws`
- Local simulation workspace: `~/schaeffler`

## Prerequisites

- ROS 2 Humble installed
- Isaac Sim (with ROS bridge) installed and its ROS workspace built
- Nav2 packages available in your workspace (`nav_bringup`)
- USD scene file: `/home/schaeffler/Downloads/omron_emma/emma.usd`

## Quick start (build & run)

1. Source ROS 2 and Isaac Sim ROS workspace environments:
```bash
source /opt/ros/humble/setup.bash
source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
```

2. Build this package from your workspace root (`~/schaeffler`):
```bash
cd ~/schaeffler
colcon build --packages-select agv_orchestrator
source ~/schaeffler/install/setup.bash
```

3. Launch the orchestrated startup (example):
```bash
ros2 launch agv_orchestrator isaac_and_nav2.launch.py \
  usd_path:=/home/schaeffler/Downloads/omron_emma/emma.usd \
  play_sim_on_start:=true \
  map:=src/nav_bringup/maps/slam_map.yaml

```

Or to run Isaac Sim directly (example parameters):
```bash
ros2 launch isaacsim run_isaacsim.launch.py gui:="/home/schaeffler/Downloads/omron_emma/emma.usd" play_sim_on_start:=true
```

## What this package does

- Includes the Isaac Sim launch from the Isaac Sim ROS workspace.
- Includes Nav2 bringup from `nav_bringup`.
- Applies a configurable delay to Nav2 startup so ISAAC topics are available before Nav2 initializes.
- Exposes configurable launch arguments in `launch/isaac_and_nav2.launch.py`.

