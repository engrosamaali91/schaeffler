# ROS2 Tutorials

## Driving TurtleBot via ROS2 messages [Setup link](https://docs.omniverse.nvidia.com/isaacsim/latest/ros_tutorials/tutorial_ros_drive_turtlebot.html)

- Drive the robot using the Differential Controller and the Articulation Controller

- Subscribing to a ROS2 Twist message on /cmd_vel topic 


## ROS2 Clock [Setup link](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_clock.html)

- Explanation for using the /clock topic and the use_sim_time ROS parameter for time synchronization.

- Creating and using ROS2 Clock Publisher and Subscriber nodes.


## ROS2 Publish Real Time Factor [Setup link](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_rtf.html)

it should be close to 1. if RTF <1 it means simulation time is running slower than the real time.

```ros2 topic echo /RTF_topic```


##  ROS2 Cameras [setup link](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_camera.html)

Dont forget to move the robot into the 

Learn how to publish camera programatically [publishing camera data ]()


## ROS2 Lidar [Lidar sensor](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_rtx_lidar.html)

- Briefly introduce how to use RTX Lidar sensors.

- Create a RTX Lidar sensor.

- Publish sensor data to ROS2 as LaserScan and PointCloud2 messages.

- Use the menu shortcut to create RTX Lidar sensor publishers.

- Put it all together and visualize multiple sensors in RViz2.


## [ROS2 Transform Trees and Odometry](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_tf.html#isaac-sim-app-tutorial-ros2-tf)



### How i rigged emma robot 

- import usd
- Inser physics ground plane and physics scene and in physics scene increase the Time steps per second value from 60 to 290
- created xforms of body and wheels
- local corrdinates of all xforms set as y pointing sideways, xoforward and z upward. To ensure body and wheel local rotations are aligned. It does not matter the orientation of the child prims as long as the orientation of xforms are aligned.
- Give rigid body API to xforms and collider API to child of xforms
- Select body then xforms individually and create a revelote joint 
- Add angular derive to Revolute joints and change the rotation axis to Y because in local coordinates the rotation is around Y
- For organization i moved the joints to joints scope and renamed it 
- Using measure tool i measure the distance between wheel and radius of the wheel
- give articulationa root api to the root of the emma robot. in my case Emma_cobot
- In differential drive omnigraph i added these values and the names of the joints. Emma_cobot has articulation root api so this would be the target prim 
- Run  


### Encountered Issue 
- The damping was too low and because of that i was not able to rotate and move with teleop twist keyboard [Configure link](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_joint_tuning.html)



- The reason why laser was moving along with robot was becuase i used the wrong frame id, the frameid is supposed to be the xform under which the lidar is placed.



### Runnning process
This would match isaac sim time and ros2 time.
```ros2 param set /rviz use_sim_time true```

##### Issue: Discrepancy in Robot Motion Visualization (Isaac Sim vs. RViz)

**Problem:**
* The robot's actual direction of motion within Isaac Sim appeared opposite to its displayed movement in RViz.
* This occurred despite verifying that `cmd_vel` commands were being sent correctly and interpreted as "forward" in Isaac Sim.

**Root Cause:**
* The primary reason was a misalignment in coordinate frame conventions. Specifically, the forward axis (+X) of the robot's base link (or the lidar's frame which dictates the base's orientation) within the Isaac Sim model was oriented differently from the expected ROS REP 103 standard for the `base_link` frame in RViz.

**Solution:**
* The orientation of the lidar (or the robot's base link itself) was adjusted within the Isaac Sim stage (or its corresponding URDF/USD definition).
* This adjustment ensured that the +X axis of the `base_link` frame, as published via TF to RViz, accurately represented the robot's forward direction of travel in the simulation.

**Outcome:**
* The robot's motion in Isaac Sim and its visualization in RViz now correctly correspond, providing an accurate representation of the robot's state.

Right now i am going through [Core API Tutorial Series](https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/index.html#isaac-sim-core-api-tutorials-page)


