#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import time
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def quat_from_yaw(yaw_rad: float):
    """Convert yaw (in radians) to a quaternion with roll=pitch=0."""
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


class OdomLogger(Node):
    def __init__(self, filename):
        super().__init__("odom_logger")
        self.is_recording = False
        self.goal_reached = False

        # Open the CSV file in append mode
        self.csv_file = open(filename, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.create_subscription(PoseStamped, "/odom", self.odom_callback, 1)

    def odom_callback(self, msg):
        """Callback function to log the data"""
        if self.is_recording:
            # Write data to CSV
            self.csv_writer.writerow([time.time(), msg.pose.position.x, msg.pose.position.y, msg.pose.orientation.z])
            if self.goal_reached:
                self.is_recording = False  # Stop recording once the goal is reached
                self.csv_file.close()

    def start_recording(self):
        """Start recording data"""
        self.is_recording = True
        self.get_logger().info("Started recording odom data.")

    def stop_recording(self):
        """Stop recording data"""
        self.is_recording = False
        self.get_logger().info("Stopped recording odom data.")

    def set_goal_reached(self):
        """Mark the goal as reached"""
        self.goal_reached = True
        self.get_logger().info("Goal reached! Stopping data recording.")


def main():
    rclpy.init()

    # Create and start the OdomLogger node for CSV recording
    odom_logger = OdomLogger("/home/schaeffler/sim_bags/odom_data.csv")

    # 1) Create a Nav2 helper that talks to the Nav2 stack (BT Navigator, etc.)
    nav = BasicNavigator()

    # 2) Set initial pose (map frame)
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)

    # 3) Block until Nav2 is active
    nav.waitUntilNav2Active()

    # 4) Define goal(s)
    goal_pose1 = create_pose_stamped(nav, 5.00, 0.00, 0.00)

    # 5) Send as waypoints (equivalent to a single goal here)
    waypoints = [goal_pose1]

    # Start recording odom data as soon as the goal is received
    odom_logger.start_recording()

    # Begin following waypoints
    nav.followWaypoints(waypoints)

    while not nav.isTaskComplete():
        _ = nav.getFeedback()
        # (optional) print or log feedback here

    # 6) Mark goal as reached and stop recording
    odom_logger.set_goal_reached()

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
