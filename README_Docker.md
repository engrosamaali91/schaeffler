# Docker Setup: ROS 2 Nav2 + Isaac Sim

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Build Image](#build-image)
3. [Run Container](#run-container)
4. [Workflow](#workflow)
5. [Troubleshooting](#troubleshooting) 


## Prerequisites

- Linux (Ubuntu 22.04)
- Docker installed
- ROS 2 Humble installed
- Isaac Sim running with ROS 2 bridge
- X11 display server

### Install Docker

```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
newgrp docker

# Verify installation
docker run hello-world
```

### Install ROS 2 Humble

Refer to [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) for ros2 installation 

### Clone Repository

```bash
git clone https://github.com/engrosamaali91/schaeffler.git
cd schaeffler
```
> [!NOTE]
> Checkout to the isaac_sim branch
---

## Build Image

```bash
docker build -t emma_in_isaacsim:latest .
```

Verify:
```bash
docker image ls | grep emma_in_isaacsim
```

---


## Run Container
#### Run Isaac sim on host host machine before running the container   
- run isaac sim 
- load emma usd file
- click play


### Start Container with X11 Forwarding

Enable X11 access:
```bash
xhost +local:
```

Run container:
```bash
docker run -it --rm \
  --network host \
  --ipc host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  emma_in_isaacsim:latest
```

Inside container:
```bash
source install/setup.bash
ros2 topic list  # Verify Isaac Sim topics
```

---

## Workflow

**Terminal 1 — Launch Nav2:**
```bash
cd /isaac_emma_ws
source install/setup.bash
ros2 launch nav_bringup bringup_launch.py \
  use_sim_time:=true \
  map:=src/nav_bringup/maps/slam_map.yaml
```

**Terminal 2 — Launch RViz2:**
```bash
docker exec -it <container_id> bash
source install/setup.bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

**Set Goal:** In RViz2, select "Nav2 Goal" tool and click on map

---

## Troubleshooting

**Isaac Sim topics not visible:**
- Verify Isaac Sim ROS 2 bridge is active on host
- Check `ROS_DOMAIN_ID` matches on host and container
- Ensure container runs with `--network host` flag

**RViz2 doesn't display:**
- Run `xhost +local:` on host before docker run
- Verify `echo $DISPLAY` returns a value

**Container name not found:**
```bash
docker ps  # Find container ID
docker exec -it <container_id> bash
```

**Topics on different machines:**
```bash
docker run -it --rm --network host \
  -e ROS_LOCALHOST_ONLY=0 \
  emma_in_isaacsim:latest
```
