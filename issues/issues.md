# 🤖 Isaac Sim ROS 2 Differential Drive Tuning Issue

## Title
**Conflict: High Damping (Stability) vs. Low Damping (Responsiveness) in Isaac Sim ROS 2 Differential Drive Control**

---

## 📝 Description of the Issue

When attempting to control a custom differential drive robot in **Isaac Sim** using **ROS 2 Nav2** and disabling the internal speed caps, a fundamental conflict arises between physical stability and command responsiveness, primarily governed by the **damping** parameter on the angular drive.

The goal is to allow the robot's movement to be *solely* influenced by the ROS 2 Nav2 stack parameters (e.g., those from `controller_server` and `velocity_smoother`) without interference from Isaac Sim's internal capping.

### ⚙️ Configuration Details

| Parameter | Node | Value Set | Intended Outcome |
| :--- | :--- | :--- | :--- |
| **`maxLinearSpeed`** | `DifferentialController` | **0** | Disable Isaac Sim capping. |
| **`maxAngularSpeed`** | `DifferentialController` | **0** | Disable Isaac Sim capping. |
| **`damping`** | `Angular Drive` | Varied (e.g., **1e9**) | High value used for testing stability. |

### 💥 Observed Conflict

| Scenario | Differential Controller Caps | Angular Drive Damping | Result |
| :--- | :--- | :--- | :--- |
| **1 (Unstable)** | `maxSpeed` = **0** | **High** (e.g., 1e9) | Robot becomes **uncontrollable** and unstable upon receiving velocity commands (`/cmd_vel`). |
| **2 (Unresponsive)** | `maxSpeed` = **0** | **Low/Reduced** | Robot is stable but **unresponsive**; fails to rotate or stop correctly after initial velocity command. |
| **3 (Stable/Capped)** | `maxSpeed` = **> 0** (e.g., 0.5) | **High** (e.g., 1e9) | Robot moves and behaves correctly, but is now constrained by the **Isaac Sim internal cap**, defeating the goal of Nav2-only control. |

### 🎯 Desired Outcome

A configuration in Isaac Sim that allows the `maxLinearSpeed` and `maxAngularSpeed` in the `DifferentialController` node to be set to **0** while maintaining a state of both **stability** and **responsiveness**, thus permitting the **ROS 2 Nav2 stack** to be the sole authority over the robot's velocity limits and motion parameters.

---

## 📚 Reference Link

The full discussion and context for this issue can be found on the NVIDIA Developer Forums:

[Isaac Sim ROS 2 Diff Drive Tuning: Conflict between High Damping (Stability) and Low Damping (Responsiveness)](https://forums.developer.nvidia.com/t/isaac-sim-ros-2-diff-drive-tuning-conflict-between-high-damping-stability-and-low-damping-responsiveness/351293)



Referring to 

[Tuning Joint Drive Gainsation](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/joint_tuning.html)

[Tuning guide](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/ext_isaacsim_robot_setup_gain_tuner.html)


## ✅ Potential Solution

The differential drive controller in isaac sim expects the value unscaled from ros2 subscriber node. To get the exact velocities mapped on the simulated robot, remove the to_scale node from the default differential drive action graph.
