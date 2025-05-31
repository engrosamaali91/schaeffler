- [Required Installations](#required-installations)
- [Important ROS2 Commands](#important-ros2-commands)


# Required Installations 

- **Robot State Publisher**: Publishes the state (pose) of a robot's links using its URDF and joint states.
- **Joint State Publisher**: Simulates and publishes joint positions for robots, typically used when no hardware is available.


Install join state piblisher 
```bash
sudo apt install ros-humble-joint-state-publisher
```

install joint install publisher gui
```bash
sudo apt install ros-humble-joint-state-publisher-gui
```

install xacro package
```
sudo apt install ros-humble-xacro
```


# Important ROS2 Commands
``` 
ros2 run rqt_tf_tree rqt_tf_tree 
```
