- [Required Installations](#required-installations)
- [Create Package](#create-package)
- [Gazebo](#gazebo)
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


# Create Package
Now create ROS2 package using 
```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 emma_visualization
```

And then add directories description launch and meshes

Add your urdf.xacro files in description.

> **_NOTE:_**  Load rviz2 using ```ros2 run rviz2 rviz2``` after sourcing terminal properly 

Create a launch file and include robot state publisher as this node provide topics like /tf /tf_static and /robot_description however provide a path of a xacro file from description directory to robto_state_publisher as a paramter

```
cd schaeffler_ws && colcon build --symlink-install 
source install/setup.bash
ros2 launch emma_visualization rsp.launch.py
```

Then run rviz2 ```ros2 run rviz2 rviz2```

Provide joint states as robot wheels are expecting some join values
```
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

In Rviz2 change frame_id from world to base_link becuase the root of the robot is set as base_link
add TF and robot_model and check if transform tree looks correct.

![Robot frames](src/media/frames.png)

Ensure the robot wheels are perfectly aligned with z axis upward of allsix wheels 


![Robot model](src/media/Wheels.png)


# Gazebo
Now that we have written robot state publisher node and we can visualize robot in rivz now is the time to run the robot in gazebo and spawn the robot in gazebo. 

> **_NOTE:_** Ensure to launch robot state publisher with use_sim_time:= true becuase we gonna be running it on gazebo time 

Launch gazebo and see if it is loading 
```
ros2 launch gazebo_ros gazebo.launch.py
```


# Launch rsp, spawn robot and gazebo launch file together 
```
ros2 launch emma_visualization launch_sim.launch.py 
```

Move robot with teleop twist keyboard this will publish command velocities on cmd_vel topic 
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

If you want create a world in gazebo and save the world in worlds directory after deleting robot

Load the gazebo world 

```
ros2 launch emma_visualization launch_sim.launch.py world:=./src/emma_visualization/worlds/powerplant.world
```

This would load the gazebo world with a robot at origin 

> **Note:** I have kept the supporting wheel fixed as this is a diff drive robot but in case due to large friction i should add mu values as zero in corresponding gazebo tags


```
    <gazebo reference="caster_wheel">
        <material>Gazebo/Black</material>
        <mu1 value="0.001"/>
        <mu2 value="0.001"/>
    </gazebo>
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