# NVIDIA Isaac Sim Docker Setup Guide

## Table of Contentss:

- [NVIDIA Isaac Sim Docker Setup Guide](#nvidia-isaac-sim-docker-setup-guide)
  - [Table of Contentss:](#table-of-contentss)
  - [Overview](#overview)
  - [Installation](#installation)
    - [Docker Installation](#docker-installation)
    - [NVIDIA Container Toolkit Installation](#nvidia-container-toolkit-installation)
  - [Container Setup](#container-setup)
    - [GPU Verification](#gpu-verification)
    - [Image Retrieval](#image-retrieval)
    - [Volume Mount Configuration](#volume-mount-configuration)
  - [Container Deployment](#container-deployment)
    - [Running Interactive Bash Session](#running-interactive-bash-session)
    - [Running with GUI](#running-with-gui)
    - [System Compatibility Check](#system-compatibility-check)
  - [Important Notes](#important-notes)
    - [ROS 2 Domain ID Configuration](#ros-2-domain-id-configuration)

---

## Overview

This guide provides instructions for setting up NVIDIA Isaac Sim (version 5.1.0) in a Docker container. The containerized version ensures consistency across different development environments and simplifies dependency management.

**Reference Documentation:** [NVIDIA Isaac Sim Container Documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.html#isaac-sim-requirements-isaac-sim-container)

---
## Installation

### Docker Installation

Follow these steps to install Docker on your system:

```bash
# Download and run the Docker convenience script
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add your user to the docker group (post-install configuration)
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker

# Verify Docker installation
docker run hello-world
```

### NVIDIA Container Toolkit Installation

The NVIDIA Container Toolkit enables Docker containers to access the host GPU.

```bash
# Configure the NVIDIA Container Toolkit repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Update package lists
sudo apt-get update

# Install NVIDIA Container Toolkit packages
sudo apt-get install -y nvidia-container-toolkit

# Configure the container runtime for Docker
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Verify NVIDIA Container Toolkit installation
docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
```

---

## Container Setup

### GPU Verification

Before deploying the Isaac Sim container, verify that your GPU is properly recognized:

```bash
nvidia-smi
```

This command displays your GPU driver version, CUDA version, and available GPUs.

### Image Retrieval

Pull the official NVIDIA Isaac Sim container image:

```bash
docker pull nvcr.io/nvidia/isaac-sim:5.1.0
```

### Volume Mount Configuration

Create the necessary directories and configure volume mounts on the host system for persistent data:

```bash
# Create cache and configuration directories
mkdir -p ~/docker/isaac-sim/cache/main/ov
mkdir -p ~/docker/isaac-sim/cache/main/warp
mkdir -p ~/docker/isaac-sim/cache/computecache
mkdir -p ~/docker/isaac-sim/config
mkdir -p ~/docker/isaac-sim/data/documents
mkdir -p ~/docker/isaac-sim/data/Kit
mkdir -p ~/docker/isaac-sim/logs
mkdir -p ~/docker/isaac-sim/pkg

# Set appropriate permissions (uid:gid 1234 is the container user)
sudo chown -R 1234:1234 ~/docker/isaac-sim
```

---

## Container Deployment

### Running Interactive Bash Session

To run Isaac Sim in interactive mode with access to a Bash shell:

```bash
# Enable X11 forwarding for GUI support
xhost +local:

# Run the Isaac Sim container
docker run --name isaac-sim \
  --entrypoint bash \
  -it \
  --gpus all \
  -e "ACCEPT_EULA=Y" \
  -e "PRIVACY_CONSENT=Y" \
  --rm \
  --network=host \
  -v $HOME/.Xauthority:/isaac-sim/.Xauthority \
  -e DISPLAY \
  -v ~/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
  -v ~/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
  -u 1234:1234 \
  nvcr.io/nvidia/isaac-sim:5.1.0
```

### Running with GUI

To launch Isaac Sim with the graphical user interface:

```bash
./runapp.sh
```

### System Compatibility Check

Verify that your system meets Isaac Sim requirements:

```bash
./isaac-sim.compatibility_check.sh
```

---

## Important Notes

> [!WARNING]  
> **Running Isaac Sim with GUI in the container is generally not recommended.**
> 
> - The application experience may not be as expected. For a full GUI app experience please run Isaac Sim with the [Workstation Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html).
> - X11 forwarding limitations and GPU rendering constraints may impact performance and visual quality.

### ROS 2 Domain ID Configuration

> [!IMPORTANT]
> To ensure ROS 2 topic discovery between the container and the host machine, use the same `ROS_DOMAIN_ID` on both environments.

Check your current domain ID:

```bash
echo $ROS_DOMAIN_ID
```

If the value is not `0`, set it to `0` (default):

```bash
export ROS_DOMAIN_ID=0
```

If the container and host use different domain IDs, ROS 2 nodes may not discover each other and topics may not be visible across environments.