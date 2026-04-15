# ROS 2 Docker Deployment

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Setup](#setup)
3. [Build Docker Image](#build-docker-image)
4. [Run Container](#run-container)
5. [Configuration](#configuration)
6. [Workflow](#workflow)

---

## Prerequisites

### Docker
> [!NOTE]
> Refer to [README_ISAAC_DOCKER.md](README_ISAAC_DOCKER.md#installation) for Docker Installation 
>
> Refer to [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) for ros2 installation 

### Workstation ROS 2 & Packages

```bash
# Update package list
sudo apt update

# Install ROS 2 Humble
sudo apt install ros-humble-gazebo-ros-pkgs
source /opt/ros/humble/setup.bash

# Required packages
sudo apt install \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-image-transport-plugins \
  ros-humble-rqt-image-view \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-twist-mux
```

### Verify Installation

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

---

## Setup

### Clone Repository

```bash
git clone https://github.com/engrosamaali91/schaeffler.git
cd schaeffler
```

### Build Workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

> [!IMPORTANT]
> If build fails due to missing packages, install with `sudo apt install <package_name>` and rebuild.

---

## Build Docker Image

```bash
cd schaeffler
docker build -t emma_in_gazebo:latest .
```

### Verify Build

```bash
docker image ls | grep emma_in_gazebo
```

---

## Run Container

### Start Container

```bash
xhost +local:
docker run -it --rm --network host \
  --ipc host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  emma_in_gazebo:latest bash
```

### Inside Container

```bash
source install/setup.bash
ros2 topic list  # Verify topic discovery
```

### Additional Container Instances

**Attach to running container:**
```bash
docker exec -it <container_id> bash
```

**Restart existing container:**
```bash
docker start -ai <container_name>
```

---

## Configuration

### ROS Domain ID

> [!IMPORTANT]
> Both host and container must use the same `ROS_DOMAIN_ID` for topic discovery.

**Check current ID:**
```bash
echo $ROS_DOMAIN_ID
```

**Set to default (0):**
```bash
export ROS_DOMAIN_ID=0
```

**Run container with specific ID:**
```bash
docker run -it --rm --network host \
  -e ROS_DOMAIN_ID=51 \
  emma_in_gazebo:latest bash
```

---

## Workflow

### 1. Launch Gazebo on Host

```bash
ros2 launch emma_visualization launch_sim.launch.py \
  world:=./src/emma_visualization/worlds/powerplant.world
```

### 2. Open Three Container Terminals

**Terminal 1 - Localization:**
```bash
ros2 launch emma_visualization localization_launch.py \
  use_sim_time:=true \
  map:=./maps/my_map_save.yaml
```

**Terminal 2 - Navigation:**
```bash
ros2 launch emma_visualization navigation_launch.py \
  use_sim_time:=true
```

**Terminal 3 - Visualization:**
```bash
rviz2
```

### 3. RViz Configuration

Load config (optional):
```
/gzemma_ws/src/emma_visualization/config/emma_view.rviz
```

Or add manually:
- `/tf` (transform)
- `/robot_model`
- `/local_costmap`
- `/global_costmap`

### 4. Send Navigation Goals

1. **Set initial pose:** Select "2D Pose Estimate" → click on map
2. **Send goal:** Select "2D Goal Pose" → click destination

---
