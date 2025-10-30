#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import csv, os, time

OUTPUT_CSV = f"sim_odom_{int(time.time())}.csv"
TOPIC = "/odom"   # change if your odom topic name differs

class OdomLogger(Node):
    def __init__(self):
        super().__init__('odom_to_csv')
        # open CSV and write header
        self.f = open(OUTPUT_CSV, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t","x","y","yaw","vx"])
        self.last_stamp = None

        # simple subscription
        self.sub = self.create_subscription(Odometry, TOPIC, self.cb, 10)
        self.get_logger().info(f"Writing to {OUTPUT_CSV}")

    def cb(self, msg: Odometry):
        # time in seconds from header stamp
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == self.last_stamp:
            return
        self.last_stamp = t

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        vx = msg.twist.twist.linear.x  # forward velocity from odom

        self.w.writerow([t, p.x, p.y, yaw, vx])
        self.f.flush()

    def destroy_node(self):
        try:
            self.f.close()
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = OdomLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
