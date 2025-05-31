- [Required Installations](#required-installations)
- [Important ROS2 Commands](#important-ros2-commands)


# Required Installations 

- **Robot State Publisher**: Publishes the state (pose) of a robot's links using its URDF and joint states.
- **Joint State Publisher**: Simulates and publishes joint positions for robots, typically used when no hardware is available.


Install join state piblisher 
```
sudo apt install ros-humble-joint-state-publisher
```

install joint install publisher gui
```
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


Testing robot state publisher directly through terminal, before doing that download ```example_robot.urdf.xacro``` file as this is the robot_description file
```
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ./Downloads/example_robot.urdf.xacro)"
```

Run to provide joint values to the robot arm
```
ros2 run joint_state_publisher_gui joint_state_publisher_gui 
```

open rviz2 to visualize add tf and robot model-> add the topic robot_description 
```
rviz
```

to visualize frame_id and transforms live
```
ros2 run rqt_tf_tree rqt_tf_tree 
```

or using tf2 tool but this would save pdf in home directory
```
ros2 run tf2_tools view_frames 
```