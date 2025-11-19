#!/usr/bin/env python3
# nav2_test.py — minimal deps (no tf_transformations)

import csv
import math
import threading
import time
import os
import rclpy
from rclpy.node import Node as RclNode  # just for typing hints if needed
import glob
import re

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


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


def yaw_from_quat(qx, qy, qz, qw):
    """Return yaw angle (rad) from quaternion."""
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


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


class OdomCsvRecorder:
    """Record `/odom` to CSV while `recording` is True.

    Writes rows: t, x, y, yaw, vx (t = seconds since first recorded odom message).
    Designed to start after the goal is sent and stop when the task completes.
    """

    def __init__(self, filename: str = None):
        self.node = None
        self.sub = None
        self._thread = None
        self._file = None
        self._writer = None
        self._recording = False
        self._start_stamp = None
        self._lock = threading.Lock()
        if filename is None:
            ts = time.strftime('%Y%m%d_%H%M%S')
            filename = f'nav2_odom_{ts}.csv'
        # ensure directory exists
        os.makedirs(os.path.dirname(filename) or '.', exist_ok=True)
        self.filename = filename

    def _odom_callback(self, msg: Odometry):
        with self._lock:
            if not self._recording:
                return
            # set start stamp on first received message
            if self._start_stamp is None:
                self._start_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - self._start_stamp
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            vx = msg.twist.twist.linear.x
            # write row including vx
            try:
                self._writer.writerow([f"{t:.6f}", f"{x:.6f}", f"{y:.6f}", f"{yaw:.6f}", f"{vx:.6f}"])
            except Exception:
                # swallow I/O errors to avoid crashing nav loop; user can check file later
                pass

    def start(self):
        """Create node, subscription and begin spinning in background thread."""
        """Create node, subscription and open CSV file. Do NOT start a background spin.

        Use `spin_once()` from the main loop to process incoming odom messages.
        """
        if self.node is not None:
            return
        # create node and subscription
        self.node = rclpy.create_node('nav2_odom_csv_recorder')
        self.sub = self.node.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        # open file and csv writer (include vx column)
        self._file = open(self.filename, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(['t', 'x', 'y', 'yaw', 'vx'])

    def start_recording(self):
        with self._lock:
            self._recording = True
            self._start_stamp = None

    def stop_recording(self):
        with self._lock:
            self._recording = False

    def shutdown(self):
        # stop recording and close resources
        try:
            self.stop_recording()
            if self.node is not None:
                # destroy node to stop rclpy.spin
                try:
                    self.node.destroy_node()
                except Exception:
                    pass
                self.node = None
            if self._file is not None:
                try:
                    self._file.flush()
                    self._file.close()
                except Exception:
                    pass
                self._file = None
        finally:
            # nothing to join since we don't spin in a background thread
            pass

    def spin_once(self, timeout: float = 0.0):
        """Process pending callbacks once for the recorder node.

        Call this periodically from the main loop (e.g., inside the while not nav.isTaskComplete()).
        """
        if self.node is None:
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=timeout)
        except Exception:
            # swallow exceptions during spin_once to avoid crashing the navigator loop
            pass


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

    # prepare CSV recorder but don't start recording until goal is sent
    # create a new run file each time: logs/nav2_run_1.csv, nav2_run_2.csv, ...
    def _next_run_filename(dirpath: str = 'logs', prefix: str = 'nav2_run_', ext: str = '.csv') -> str:
        os.makedirs(dirpath, exist_ok=True)
        pattern = os.path.join(dirpath, f"{prefix}*{ext}")
        files = glob.glob(pattern)
        maxn = 0
        for f in files:
            bn = os.path.basename(f)
            m = re.match(rf"{re.escape(prefix)}(\d+){re.escape(ext)}$", bn)
            if m:
                try:
                    n = int(m.group(1))
                    if n > maxn:
                        maxn = n
                except ValueError:
                    pass
        return os.path.join(dirpath, f"{prefix}{maxn+1}{ext}")

    run_filename = _next_run_filename()
    recorder = OdomCsvRecorder(filename=run_filename)
    recorder.start()

    try:
        # 5) Define goal(s)
        goal_pose1 = create_pose_stamped(nav, 5.00, 0.00, 0.00)

        # 6) Send as waypoints (equivalent to a single goal here)
        waypoints = [goal_pose1]

        # send waypoints (this may block briefly while the goal is sent)
        nav.followWaypoints(waypoints)

        # start recording as soon as the goal is sent
        recorder.start_recording()

        # process feedback and recorder callbacks in the same thread using spin_once
        while not nav.isTaskComplete():
            _ = nav.getFeedback()
            # let the recorder process incoming /odom messages
            recorder.spin_once(timeout=0.05)
            time.sleep(0.02)


        grace_sec = 1.0   # 0.5–1.0 s is usually enough
        end_t = time.time() + grace_sec
        while time.time() < end_t:
            recorder.spin_once(timeout=0.05)   # if your recorder uses spin_once
            time.sleep(0.02)
        # stop recording after task completes
        recorder.stop_recording()

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

    finally:
        # ensure recorder shutdown and ROS shutdown
        try:
            recorder.shutdown()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
