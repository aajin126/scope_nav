#!/usr/bin/env python3
# ROS2 port of track_ped_pub.py (same behavior)
# - Sub:  /pedsim_visualizer/tracked_persons (pedsim_msgs/TrackedPersons)
# - Pub:  /track_ped (pedsim_msgs/TrackedPersons), in base_footprint frame
# - Uses Gazebo entity state service to get robot pose/vel

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from pedsim_msgs.msg import TrackedPersons, TrackedPerson
from gazebo_msgs.srv import GetEntityState

from tf_transformations import euler_from_quaternion


class TrackPed(Node):
    PED_NUM = 34

    def __init__(self):
        super().__init__('track_ped')

        qos = QoSProfile(depth=10)

        self.ped_sub = self.create_subscription(
            TrackedPersons,
            '/pedsim_visualizer/tracked_persons',
            self.ped_callback,
            qos,
        )

        self.track_ped_pub = self.create_publisher(
            TrackedPersons,
            '/track_ped',
            qos,
        )

        # Gazebo entity-state service client (Gazebo Classic + gazebo_ros_state plugin)
        # Some setups expose '/get_entity_state', others '/gazebo/get_entity_state'
        self._srv_candidates = ['/get_entity_state', '/gazebo/get_entity_state']
        self._srv_idx = 0
        self._get_state_cli = None
        self._ensure_service_client()

    def _ensure_service_client(self):
        """Create a client and wait briefly for it; rotate candidates if needed."""
        if self._get_state_cli is None:
            self._get_state_cli = self.create_client(GetEntityState, self._srv_candidates[self._srv_idx])

        # do not block too long inside callbacks
        if not self._get_state_cli.wait_for_service(timeout_sec=0.2):
            # rotate to the other name and try again next time
            self._srv_idx = (self._srv_idx + 1) % len(self._srv_candidates)
            if self._srv_candidates[self._srv_idx] != self._get_state_cli.srv_name:
                self._get_state_cli = self.create_client(GetEntityState, self._srv_candidates[self._srv_idx])

    def get_robot_state(self):
        """
        Equivalent to rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        but using GetEntityState in ROS2.
        """
        self._ensure_service_client()
        if self._get_state_cli is None or not self._get_state_cli.service_is_ready():
            return None

        req = GetEntityState.Request()
        req.name = 'mobile_base'
        req.reference_frame = 'world'

        future = self._get_state_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.2)
        if not future.done():
            return None

        try:
            return future.result()
        except Exception as e:
            self.get_logger().warn(f'GetEntityState call failed: {e}')
            return None

    def ped_callback(self, peds_msg: TrackedPersons):
        state = self.get_robot_state()
        if state is None:
            return

        # Robot pose
        robot_pos = np.zeros(3, dtype=np.float32)
        robot_pos[0] = float(state.state.pose.position.x)
        robot_pos[1] = float(state.state.pose.position.y)

        q = state.state.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        robot_pos[2] = float(yaw)

        # Robot twist (not used in transform, but kept for parity)
        # robot_vel = np.array([state.state.twist.linear.x, state.state.twist.linear.y], dtype=np.float32)

        # Homogeneous transform map_T_robot
        c = np.cos(robot_pos[2])
        s = np.sin(robot_pos[2])

        map_R_robot = np.array([[c, -s],
                                [s,  c]], dtype=np.float32)

        map_T_robot = np.array([[c, -s, robot_pos[0]],
                                [s,  c, robot_pos[1]],
                                [0,  0, 1]], dtype=np.float32)

        robot_R_map = np.linalg.inv(map_R_robot)
        robot_T_map = np.linalg.inv(map_T_robot)

        tracked_peds = TrackedPersons()
        tracked_peds.header.frame_id = 'base_footprint'
        tracked_peds.header.stamp = self.get_clock().now().to_msg()

        for ped in peds_msg.tracks:
            ped_pos = np.array([ped.pose.pose.position.x, ped.pose.pose.position.y, 1.0], dtype=np.float32)
            ped_vel = np.array([ped.twist.twist.linear.x, ped.twist.twist.linear.y], dtype=np.float32)

            ped_pos_in_robot = robot_T_map @ ped_pos.T
            ped_vel_in_robot = robot_R_map @ ped_vel.T

            tracked_ped = TrackedPerson()
            tracked_ped = ped  # keep original fields (id, track info, etc.)
            tracked_ped.pose.pose.position.x = float(ped_pos_in_robot[0])
            tracked_ped.pose.pose.position.y = float(ped_pos_in_robot[1])
            tracked_ped.twist.twist.linear.x = float(ped_vel_in_robot[0])
            tracked_ped.twist.twist.linear.y = float(ped_vel_in_robot[1])

            tracked_peds.tracks.append(tracked_ped)

        self.track_ped_pub.publish(tracked_peds)


def main():
    rclpy.init()
    node = TrackPed()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()