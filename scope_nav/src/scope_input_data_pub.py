#!/usr/bin/env python3
# ROS2 port of scope_input_data_pub.py (same behavior)
# - Sync: scan (sensor_msgs/LaserScan) + odom (nav_msgs/Odometry)
# - Pub:  scope_input_data (scope_msgs/ScopeInputData)
# - TF:   map -> base_footprint pose via tf2

import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile

from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from scope_msgs.msg import ScopeInputData

import tf2_ros
from tf_transformations import euler_from_quaternion


class ScopeInputDataPub(Node):
    def __init__(self):
        super().__init__('scope_input_data_pub')

        # data buffers
        self.scan_ranges = np.zeros(1080, dtype=np.float32)
        self.curr_vel = np.zeros(2, dtype=np.float32)  # [linear.x, angular.z]
        self.curr_pos = np.zeros(3, dtype=np.float32)  # [x, y, theta] in /map
        self.curr_odom = np.zeros(3, dtype=np.float32)
        self.header = Header()

        # base frame parameter (default: base_footprint)
        # If your robot uses a different base frame (e.g. base_link),
        # set this parameter when launching the node.
        self.declare_parameter('base_frame', 'base_footprint')
        self.base_frame = (
            self.get_parameter('base_frame')
            .get_parameter_value()
            .string_value
        )

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # message_filters subscribers (ROS2)
        self.scan_sub = Subscriber(self, LaserScan, 'scan', qos_profile=qos_profile_sensor_data)
        self.odom_sub = Subscriber(self, Odometry, 'odom', qos_profile=qos_profile_sensor_data)

        self.sync = ApproximateTimeSynchronizer(
            fs=[self.scan_sub, self.odom_sub],
            queue_size=5,
            slop=0.5,
            allow_headerless=True,
        )
        self.sync.registerCallback(self.scope_data_callback)

        self.scope_input_data_pub = self.create_publisher(
            ScopeInputData,
            'scope_input_data',
            QoSProfile(depth=1),
        )

        self.lock = threading.Lock()

        self.rate = 20.0
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def get_current_pose(self):
        """map -> base frame pose. If TF missing, return last known."""
        try:
            # latest available transform
            t = self.tf_buffer.lookup_transform(
                'map',
                self.base_frame,
                rclpy.time.Time()
            )
            q = t.transform.rotation
            _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return np.array([t.transform.translation.x, t.transform.translation.y, theta, 0, 0, 0], dtype=np.float32)
        except Exception:
            # keep behavior: warn and return previous pose
            self.get_logger().warn('Could not get robot pose (map -> base_footprint)')
            return np.concatenate([self.curr_pos, np.zeros(3, dtype=np.float32)])

    def scope_data_callback(self, scan_msg: LaserScan, odom_msg: Odometry):
        # store scan + vel; pose is updated in timer for regular publishing (same as original)
        with self.lock:
            self.header = scan_msg.header
            ranges = np.array(scan_msg.ranges, dtype=np.float32)
            ranges[np.isnan(ranges)] = 20.0
            ranges[np.isinf(ranges)] = 20.0
            self.scan_ranges = ranges

            self.curr_vel[0] = float(odom_msg.twist.twist.linear.x)
            self.curr_vel[1] = float(odom_msg.twist.twist.angular.z)

    def timer_callback(self):
        robot_pose = self.get_current_pose()
        self.curr_pos = robot_pose[:3]

        msg = ScopeInputData()
        with self.lock:
            msg.header = self.header
            msg.scan_ranges = self.scan_ranges.tolist()  # keep msg type compatible
            msg.curr_vel = self.curr_vel.tolist()

        msg.curr_pos = self.curr_pos.tolist()
        msg.curr_odom = self.curr_odom.tolist()

        self.scope_input_data_pub.publish(msg)


def main():
    rclpy.init()
    node = ScopeInputDataPub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()