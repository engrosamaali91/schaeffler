# nav_bringup with Isaac Sim: Setup and Usage Guide

This document explains how to set up and use the custom `nav_bringup` package for running the ROS 2 Navigation Stack (Nav2) with Isaac Sim. It covers the process of creating the package, copying and adapting launch files, and launching the navigation stack with a map for localization.

---

## 1. Package Creation: `nav_bringup`

The `nav_bringup` package was created to provide a custom bringup solution for navigation with Isaac Sim. It contains:
- Custom launch files
- Configuration files
- Map files

**Directory structure:**
```
src/nav_bringup/
├── CMakeLists.txt
├── package.xml
├── config/
├── include/
├── launch/
│   ├── bringup_launch.py
│   ├── localization_launch.py
│   └── navigation_launch.py
├── maps/
└── src/
```

---

## 2. Copying and Adapting Launch Files

The following launch files were copied from the standard Nav2 stack (or adapted as needed):
- `bringup_launch.py`: Main entry point for launching the navigation stack.
- `localization_launch.py`: Handles localization using AMCL and loads the map.
- `navigation_launch.py`: Launches the navigation nodes (controller, planner, etc.).

**Why?**
- `bringup_launch.py` internally calls both `localization_launch.py` and `navigation_launch.py`.
- This modular approach allows for easier customization and maintenance.

---

## 3. Launching the Navigation Stack with Isaac Sim

### **Prerequisites**
- Isaac Sim is running and publishing the robot's TF and odometry.
- The map file (YAML) exists in `src/nav_bringup/maps/`.
- The workspace is built and sourced.

### **Launch Command**
To launch the navigation stack with a pre-existing map for localization, use:

```bash
ros2 launch nav_bringup bringup_launch.py use_sim_time:=true map:=src/nav_bringup/maps/slam_map.yaml
```

- `use_sim_time:=true` ensures all nodes use the simulation clock from Isaac Sim.
- `map:=src/nav_bringup/maps/slam_map.yaml` provides the path to the map file for localization.

### **How it Works**
- `bringup_launch.py` is the main launch file.
- It internally includes `localization_launch.py` (if SLAM is not enabled), which loads the map and starts AMCL for localization.
- It also includes `navigation_launch.py` to start the rest of the navigation stack (planner, controller, etc.).

---

## 4. Notes
- If you want to use SLAM instead of localization, launch with `slam:=true` and omit the `map` argument.
- Make sure the frame names in your configuration match those published by Isaac Sim.
- The map file must be a valid ROS 2 map YAML file.

---

## 5. Example Directory Structure
```
src/nav_bringup/
├── launch/
│   ├── bringup_launch.py
│   ├── localization_launch.py
│   └── navigation_launch.py
├── maps/
│   └── slam_map.yaml
├── config/
│   └── nav2_params.yaml
...
```

---

## 6. References
- [ROS 2 Navigation Stack (Nav2) Documentation](https://navigation.ros.org/)
- [Isaac Sim ROS2 Bridge Documentation](https://docs.nvidia.com/isaac/isaac-sim/)

---

**This README documents the process and rationale for setting up the custom `nav_bringup` package for use with Isaac Sim and the ROS 2 Navigation Stack.** 




# Tuning 


🟢 Isaac Sim Parameters

| Parameter Group             | Parameter              | Current / Old Value | New Value                          | Reason                                                      |
| --------------------------- | ---------------------- | ------------------- | ---------------------------------- | ----------------------------------------------------------- |
| **Differential Controller** | maxLinearSpeed         | 0.6 m/s             | **1.2 m/s**                        | Match LD-250 spec max linear speed.                         |
|                             | maxAngularSpeed        | 1.0 rad/s           | **2.094 rad/s**                    | Match LD-250 spec max rotation speed (120°/s).              |
|                             | maxAcceleration        | 0.0                 | **→ To be decided**                | Needed to reduce jerk; tune from real robot logs.           |
|                             | maxDeceleration        | 0.0                 | **→ To be decided**                | Needed to avoid instant braking; tune from real robot logs. |
|                             | maxAngularAcceleration | 0.0                 | **→ To be decided**                | Needed to reduce yaw overshoot.                             |
| **Wheel Joint Drive**       | Damping                | 1e9                 | **→ To be decided (~2–5)**         | Unrealistic lock-up; needs realistic value.                 |
|                             | Max Force              | Not Limited         | **→ To be decided (~200–300 N·m)** | Prevent unrealistic torque spikes.                          |
|                             | Stiffness              | 0.0                 | 0.0                                | Leave at zero (not needed for wheels).                      |
| **Wheel Material**          | Density                | 100                 | **50**                             | Reduce wheel inertia → less drift.                          |
|                             | Static Friction        | 1.0                 | **→ To be decided (~1.2)**         | Increase grip between wheel & floor.                        |
|                             | Dynamic Friction       | 1.0                 | **→ To be decided (~1.0)**         | Ensure realistic rolling friction.                          |
|                             | Restitution            | 0.0                 | 0.0                                | Leave at zero (no bounce).                                  |


🟠 ROS 2 / Nav2 Parameters

| Component                          | Parameter          | Current / Old Value | New Value                               | Reason                                    |
| ---------------------------------- | ------------------ | ------------------- | --------------------------------------- | ----------------------------------------- |
| **DWB Local Planner (FollowPath)** | max_vel_x          | 0.26                | **1.2**                                 | Match LD-250 linear speed cap.            |
|                                    | max_speed_xy       | 0.26                | **1.2**                                 | Keep consistent with `max_vel_x`.         |
|                                    | max_vel_theta      | 1.0                 | **2.094**                               | Match LD-250 angular cap.                 |
|                                    | acc_lim_x          | 2.5                 | **→ To be decided (~0.3)**              | Realistic linear accel, prevents jerk.    |
|                                    | decel_lim_x        | -2.5                | **→ To be decided (~-0.4)**             | Realistic braking limit.                  |
|                                    | acc_lim_theta      | 3.2                 | **→ To be decided (~0.8)**              | Realistic angular accel.                  |
|                                    | decel_lim_theta    | -3.2                | **→ To be decided (~-1.0)**             | Realistic angular decel.                  |
|                                    | sim_time           | 1.7                 | **→ To be decided (~2.0)**              | Longer horizon helps at higher speed.     |
| **Velocity Smoother**              | max_velocity       | [0.26, 0.0, 1.0]    | **[1.2, 0.0, 2.094]**                   | Consistent speed caps with LD-250.        |
|                                    | max_accel          | [2.5, 0.0, 3.2]     | **→ To be decided ([0.3, 0.0, 0.8])**   | Smooth ramping of /cmd_vel.               |
|                                    | max_decel          | [-2.5, 0.0, -3.2]   | **→ To be decided ([-0.4, 0.0, -1.0])** | Smooth braking.                           |
| **Behavior Server**                | max_rotational_vel | 1.0                 | **2.094**                               | Consistent with real robot’s angular cap. |
