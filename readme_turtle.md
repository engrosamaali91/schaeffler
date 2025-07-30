# Launch turtlebot in gazebo

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 
```

# Make map 

```
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

# Save map 
```
ros2 run nav2_map_server map_saver_cli -f my_map
```

# install cyclone dds for humble 

```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

Add to bashrc
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
Cross check it is changed

```
echo $RMW_IMPLEMENTATION
```



# visualize the urdf model in rviz
Install
```
sudo apt install ros-humble-urdf-tutorial 
```
Run
```
ros2 launch urdf_tutorial display.launch.py model:=/home/schaeffler/Downloads/my_robot.urdf
```