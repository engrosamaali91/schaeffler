# Use the official ROS 2 Humble image as the base image
FROM osrf/ros:humble-desktop-full

# Install necessary system dependencies
RUN apt-get update && apt-get install -y \
        nano \
        vim \
        build-essential \
        python3-colcon-common-extensions \
        python3-rosdep \
        git \
        curl \
        wget && \
    rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble packages for Navigation stack and Localization
RUN apt-get update && apt-get install -y \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-nav2-common \
        ros-humble-launch-ros \
        ros-humble-twist-mux \
        ros-humble-image-transport-plugins \
        ros-humble-rqt-image-view \
        ros-humble-rqt-tf-tree \
        ros-humble-xacro \
        ros-humble-teleop-twist-keyboard \
        ros-humble-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Create a ros2 workspace
WORKDIR /isaac_emma_ws

# Copy only required packages
COPY ./src/localization ./src/localization
COPY ./src/navigation ./src/navigation
COPY ./src/nav_bringup ./src/nav_bringup

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  colcon build --symlink-install"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /isaac_emma_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=0" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc