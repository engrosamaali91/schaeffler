#!/usr/bin/env python3

import rclpy  # Core ROS 2 Python client library
import tf_transformations  # For converting Euler angles to quaternions

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult  # High-level Nav2 helper
from geometry_msgs.msg import PoseStamped  # Standard stamped pose message

def create_pose_stamped(navigator: BasicNavigator, position_x , position_y, orientation_z):
	q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
	pose = PoseStamped()
	pose.header.frame_id = 'map'
	pose.header.stamp = navigator.get_clock().now().to_msg()  # Timestamp from node clock
	# Position (meters) in the map frame
	pose.pose.position.x = position_x
	pose.pose.position.y = position_y
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

	initial_pose = create_pose_stamped(nav, 0.0, 0.0 , 0.0)
	# 4) Publish the initial pose to Nav2 (goes to /initialpose for AMCL)
	nav.setInitialPose(initial_pose)

	# 5) Block until the Nav2 stack reports it is active (all lifecycle nodes activated)
	nav.waitUntilNav2Active()

	goal_pose1 = create_pose_stamped(nav, 7.00, 3.00, 1.57)
	goal_pose2 = create_pose_stamped(nav, 8.00, 3.00, 0.00	)
	# goal_pose3 = create_pose_stamped(nav, 2.00, 3.00, 1.57)
	
	
	# nav.goToPose(goal_pose1)

	# while not nav.isTaskComplete():
	# 	feedback = nav.getFeedback()
	# 	# print(feedback)

	waypoints = [goal_pose1, goal_pose2]
	nav.followWaypoints(waypoints)
	# nav.goThroughPoses(waypoints)
	while not nav.isTaskComplete():
		feedback = nav.getFeedback()
		# print(feedback)

	
	# print(nav.getResult())

	result = nav.getResult()
	if result == TaskResult.SUCCEEDED:
		print('Goal Succeeded!')
	elif result == TaskResult.CANCELED:
		print('Goal was canceled!')
	elif result == TaskResult.FAILED:
		print('Goal failed!{error_code}:{error_msg}')
	else:
		print('Goal has an invalid return status!')


	# 6) Clean shutdown
	rclpy.shutdown()


if __name__ == '__main__':
	main()
