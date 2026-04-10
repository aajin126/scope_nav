#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from people_msgs.msg import People, Person

import tf2_ros
from tf_transformations import euler_from_quaternion


def normalize_angle(angle: float) -> float:
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    while angle > math.pi:
        angle -= 2.0 * math.pi
    return angle


class HuNavPeopleTransform(Node):
    def __init__(self):
        super().__init__('hunav_people_transform')

        self.declare_parameter('input_topic', '/people')
        self.declare_parameter('output_topic', '/people_relative')
        self.declare_parameter('base_frame', 'base_footprint')

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        qos = QoSProfile(depth=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.people_sub = self.create_subscription(
            People,
            self.input_topic,
            self.people_callback,
            qos,
        )
        self.people_pub = self.create_publisher(
            People,
            self.output_topic,
            qos,
        )

    def people_callback(self, people_msg: People):
        source_frame = people_msg.header.frame_id or 'map'

        if source_frame == self.base_frame:
            out_msg = People()
            out_msg.header = people_msg.header
            out_msg.header.frame_id = self.base_frame
            out_msg.people = list(people_msg.people)
            self.people_pub.publish(out_msg)
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                rclpy.time.Time(),
            )
        except Exception as exc:
            self.get_logger().warn(
                f'Could not transform people from {source_frame} to {self.base_frame}: {exc}'
            )
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        _, _, yaw_tf = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
        cos_yaw = math.cos(yaw_tf)
        sin_yaw = math.sin(yaw_tf)

        out_msg = People()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = self.base_frame

        for person in people_msg.people:
            rel_person = Person()
            rel_person.name = person.name
            rel_person.position.x = cos_yaw * person.position.x - sin_yaw * person.position.y + translation.x
            rel_person.position.y = sin_yaw * person.position.x + cos_yaw * person.position.y + translation.y
            rel_person.position.z = normalize_angle(person.position.z + yaw_tf)
            rel_person.velocity.x = cos_yaw * person.velocity.x - sin_yaw * person.velocity.y
            rel_person.velocity.y = sin_yaw * person.velocity.x + cos_yaw * person.velocity.y
            rel_person.velocity.z = person.velocity.z
            rel_person.reliability = person.reliability
            rel_person.tags = list(person.tags)
            rel_person.tagnames = list(person.tagnames)
            out_msg.people.append(rel_person)

        self.people_pub.publish(out_msg)


def main():
    rclpy.init()
    node = HuNavPeopleTransform()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()