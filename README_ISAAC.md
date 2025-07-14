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





### Navigation stack process
# ROS 2 Navigation Workflow with Isaac Sim Turtlebot

## Workflow Steps

1. **Create the Map**
   - Launch SLAM Toolbox:
     ```
     ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
     ```
   - Launch RViz2:
     ```
     ros2 run rviz2 rviz2
     ```
   - In RViz2:
     - Set **Fixed Frame** to `map`.
     - Enable **Use Sim Time**:
       ```
       Panels > Displays > Global Options > Use Sim Time: True
       ```
     - If the occupancy grid does not show, set **Durability Policy** of the Map display to **Transient Local**.
   - Save the map:
     ```
     ros2 run nav2_map_server map_saver_cli -f ~/nav2_config/maps/my_map
     ```

2. **Localization**
   - Launch localization with your saved map:
     ```
     ros2 launch nav2_bringup localization_launch.py \
       map:=/home/schaeffler/nav2_config/maps/my_map.yaml \
       use_sim_time:=true
     ```
   - In RViz2, click **2D Pose Estimate** to set the initial pose.
   - Confirm laser scan and point cloud align with the map.

3. **Navigation**
   - Launch navigation:
     ```
     ros2 launch nav2_bringup navigation_launch.py \
       use_sim_time:=true \
       param_file:=/home/schaeffler/nav2_config/nav2_params.yaml
     ```
   - In RViz2, click **2D Nav Goal** to send a goal.

## Notes
- Keep `base_frame_id` consistent (`base_footprint` or `base_link`).
- Make sure Isaac Sim publishes `/clock`.
- Always set **Use Sim Time** in RViz2.
- Confirm `/tf` tree shows:
  ```
  map -> odom -> base_link
  ```
- If the map or transforms are missing, re-check:
  - Durability Policy (`Transient Local`).
  - Sim time synchronization.
  - Initial pose estimate.

✅ Done! Your robot should now localize and navigate in simulation.



# ROS 2 Navigation Workflow with Isaac Sim Emma robot

This guide documents the complete workflow to create a map, localize, and navigate using Nav2 with simulation time in Isaac Sim.

---

## 🟢 1. Start Isaac Sim
Launch Isaac Sim and ensure `/clock` is being published for simulation time.

---

## 2. Build the Map with SLAM Toolbox
Launch SLAM Toolbox:

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```
Move the robot around in the environment

## 3. Save the Map
Launch SLAM Toolbox:
```
ros2 run nav2_map_server map_saver_cli -f ~/nav2_config/maps/my_map
```

This creates:
- my_map.yaml
- my_map.pgm


## 4. Launch RViz

```
rviz2 
```
```
ros2 param set /rviz use_sim_time true
```

## 5. Launch Localization
```
ros2 launch nav2_bringup localization_launch.py \
  map:=/home/schaeffler/nav2_config/maps/my_map.yaml \
  use_sim_time:=true
```

Important:

- In RViz, set the Map display Durability Policy to Transient Local so the map appears.

- Use 2D Pose Estimate in RViz to set the initial robot pose.


## 6. Launch Navigation 
```
ros2 launch nav2_bringup bringup_launch.py \
  map:=/home/schaeffler/nav2_config/maps/my_map.yaml \
  use_sim_time:=true \
  params_file:=/home/schaeffler/nav2_config/nav2_params.yaml
```

And in rviz now you can give navigation goal 

## Recap of Common Issues and Fixes

- Map Not Showing in RViz: Change the Durability Policy to Transient Local.

- Time Sync Errors: Always set use_sim_time to true everywhere and run ros2 param set /rviz use_sim_time true.

- TF Issues: Make sure you use consistent base_frame_id (e.g., base_footprint) and odom/map frames in AMCL and Nav2 configs.

- Startup Order: Always launch Isaac Sim first, then SLAM, then save the map, then Localization, and finally Navigation.





# Manual Tuning 
## Sim2Real Parameter Tuning for AGV Simulation

The objective is to tune simulation parameters (especially damping) to ensure the robot's simulated motion accurately replicates commanded velocities.

## 📈 Objectives

- Compare `/cmd_vel` commands vs. `/odom` measured velocities.
- Quantify error using RMSE.
- Iteratively adjust physics parameters to minimize RMSE.
- Document results at each step.



5. **Adjust Parameters:**
- Reduce damping iteratively.
- Re-run the process.

6. **Track Results:**
- Record RMSE for each damping value.

## 🧪 Iterative Tuning Process

Below is the table tracking each experiment:

| Iteration | Damping Value | Max Force | RMSE Linear Velocity (m/s) | RMSE Linear Velocity (m/s) | 
|-----------|---------------|-----------|----------------------------|----------------------------|
| 1         | 1e9           | unlimited | 0.4447                     | 0.7639                     | 
| 2         | 1e8           |     unlimited      | 0.4075            | 0.6656                     |  
| 3         | 1e7           |      unlimited    |                   Robot stoped           |             Robot stoped               

