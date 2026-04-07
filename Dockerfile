FROM osrf/ros:humble-desktop-full

# Install ros packages
RUN apt-get update && apt-get install -y \
        nano \
        build-essential && \
    rm -rf /var/lib/apt/lists/*  


# Create a ros2 workspace
WORKDIR /emma_ws

# launch ros package 
# CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener.launch.py"]

# CMD [ "ros2", "launch", "gazebo_ros", "gazebo.launch.py"]
