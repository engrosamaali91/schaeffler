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

3. Launch examples:

Basic launch with defaults:
```bash
ros2 launch agv_orchestrator isaac_and_nav2.launch.py
```

Launch with custom USD and Nav2 params:
```bash
ros2 launch agv_orchestrator isaac_and_nav2.launch.py \
  usd_path:=/home/schaeffler/Downloads/omron_emma/emma.usd \
  play_sim_on_start:=true \
  params_file:=/path/to/custom_nav2_params.yaml
```

Launch with RViz2 (optional):
```bash
# Launch with default RViz configuration
ros2 launch agv_orchestrator isaac_and_nav2.launch.py rviz:=true run_test:=false
```

## What this package does

- Includes the Isaac Sim launch from the Isaac Sim ROS workspace.
- Includes Nav2 bringup from `nav_bringup` with configurable parameter files.
- Applies a configurable delay to Nav2 startup so ISAAC topics are available before Nav2 initializes.
- Optional RViz2 launch with customizable configuration.
- Exposes configurable launch arguments in `launch/isaac_and_nav2.launch.py`.

## Launch Arguments

Key arguments you can customize:
- `params_file`: Path to Nav2 parameters YAML file (default: uses nav_bringup's params)
- `rviz`: Enable/disable RViz2 visualization (default: false)
- `rviz_config`: Custom RViz configuration file (optional)
- `map`: Path to the map file
- `usd_path`: Path to the USD scene file
- `play_sim_on_start`: Whether to start simulation automatically
- `run_test`: whether to run the script and move the robot to the goal

You can check all available arguments with:
```bash
ros2 launch agv_orchestrator isaac_and_nav2.launch.py --show-args
```



