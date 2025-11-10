#!/usr/bin/env python3
# nav2_test.py — minimal deps (no tf_transformations)

import math
import rclpy
from rclpy.node import Node as RclNode  # just for typing hints if needed

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


def quat_from_yaw(yaw_rad: float):
    """
    Convert yaw (in radians) to a quaternion with roll=pitch=0.
    """
    half = 0.5 * yaw_rad
    qx = 0.0
    qy = 0.0
    qz = math.sin(half)
    qw = math.cos(half)
    return qx, qy, qz, qw


def create_pose_stamped(navigator: BasicNavigator, position_x: float, position_y: float, yaw_rad: float) -> PoseStamped:
    q_x, q_y, q_z, q_w = quat_from_yaw(yaw_rad)

    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = float(position_x)
    pose.pose.position.y = float(position_y)
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose


def main():
    # 1) Initialize ROS 2
    rclpy.init()

    # 2) Create a Nav2 helper that talks to the Nav2 stack (BT Navigator, etc.)
    nav = BasicNavigator()

    # 3) Set initial pose (map frame)
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)

    # 4) Block until Nav2 is active
    nav.waitUntilNav2Active()

    # 5) Define goal(s)
    goal_pose1 = create_pose_stamped(nav, 5.00, 0.00, 0.00)

    # 6) Send as waypoints (equivalent to a single goal here)
    waypoints = [goal_pose1]
    nav.followWaypoints(waypoints)

    while not nav.isTaskComplete():
        _ = nav.getFeedback()
        # (optional) print or log feedback here

    # 7) Report result
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal Succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # 8) Clean shutdown
    rclpy.shutdown()


if __name__ == '__main__':
    main()
