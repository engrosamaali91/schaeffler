# Use the official ROS 2 Humble image as the base image
FROM osrf/ros:humble-desktop-full

# Install necessary system dependencies
RUN apt-get update && apt-get install -y \
        nano \
        vim \
        build-essential \
        git \
        curl \
        wget && \
    rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble packages for Navigation stack and Localization
RUN apt-get update && apt-get install -y \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-twist-mux \
        ros-humble-image-transport-plugins \
        ros-humble-rqt-image-view \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-xacro \
        ros-humble-teleop-twist-keyboard && \
    rm -rf /var/lib/apt/lists/*

# Create a ros2 workspace
WORKDIR /gzemma_ws

# Copy the package source code to the workspace
COPY ./src/ ./src

# Copy maps directory
COPY ./maps/ ./maps

# Source the ros2 environment
SHELL ["/bin/bash", "-c"]

# Build the workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install
