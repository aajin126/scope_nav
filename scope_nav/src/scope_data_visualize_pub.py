#!/usr/bin/env python3
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Image
from scope_msgs.msg import ScopeOutputData
from cv_bridge import CvBridge


class ScopeImageVisualizer(Node):
    def __init__(self):
        super().__init__('scope_visualizer')
        self.bridge = CvBridge()

        qos = QoSProfile(depth=10)

        # Sub: scope_output_data (ScopeOutputData)
        self.scope_sub = self.create_subscription(
            ScopeOutputData,
            'scope_output_data',
            self.scope_callback,
            qos
        )

        # Pub: prediction_img (sensor_msgs/Image)
        self.prediction_pub = self.create_publisher(
            Image,
            'prediction_img',
            qos
        )

        self.get_logger().info('ScopeImageVisualizer started (ROS2)')

    def scope_callback(self, msg: ScopeOutputData):
        # msg.occ_grid assumed to contain at least 64*64 floats in [0,1]
        scope_data = msg.occ_grid

        if scope_data is None or len(scope_data) < 64 * 64:
            self.get_logger().warn(f'occ_grid too short: {0 if scope_data is None else len(scope_data)}')
            return

        # prediction: first 64*64
        pred = np.array(scope_data[:64 * 64], dtype=np.float32).reshape(64, 64)

        # keep original behavior: np.flip (note: flips both axes)
        pred = np.flip(pred)

        # clamp to [0,1] to avoid weird colormap artifacts
        pred = np.clip(pred, 0.0, 1.0)

        # apply colormap (same as original)
        pred_u8 = (pred * 255.0).astype(np.uint8)
        prediction_img = cv2.applyColorMap(pred_u8, cv2.COLORMAP_BONE)

        # publish Image
        img_msg = self.bridge.cv2_to_imgmsg(prediction_img, encoding='passthrough')
        # (optional) stamp/frame: ROS1 code didn't set it, so we keep it empty.
        self.prediction_pub.publish(img_msg)


def main():
    rclpy.init()
    node = ScopeImageVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()