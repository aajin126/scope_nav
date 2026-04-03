#!/usr/bin/env python3
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import OccupancyGrid


class ScopeImageVisualizer(Node):
    def __init__(self):
        super().__init__('scope_visualizer')

        qos = QoSProfile(depth=10)

        # OpenCV window settings
        self.scale = 10  # enlarge map by this factor
        cv2.namedWindow('local_map', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('local_map', 256 * self.scale, 256 * self.scale)

        # OpenCV window for visualization
        cv2.namedWindow('scope_prediction', cv2.WINDOW_NORMAL)

        # Sub: local_map (OccupancyGrid)
        self.local_map_sub = self.create_subscription(
            OccupancyGrid,
            'local_map',
            self.local_map_callback,
            qos
        )

        self.get_logger().info('ScopeImageVisualizer started (ROS2, using /local_map)')

    def local_map_callback(self, msg: OccupancyGrid):
        # msg.data: int8 [-1,100], unknown=-1
        width = msg.info.width
        height = msg.info.height

        if width == 0 or height == 0 or len(msg.data) != width * height:
            self.get_logger().warn(
                f'Invalid local_map size: width={width}, height={height}, len(data)={len(msg.data)}'
            )
            return

        data = np.array(msg.data, dtype=np.int16).reshape((height, width))

        # treat unknown (-1) as free (0) for visualization
        data[data < 0] = 0

        # normalize 0-100 -> 0-255
        norm = np.clip(data.astype(np.float32), 0.0, 100.0) / 100.0
        img_u8 = (norm * 255.0).astype(np.uint8)

        color_img = cv2.applyColorMap(img_u8, cv2.COLORMAP_BONE)

        # scale up for easier viewing
        display_img = cv2.resize(
            color_img,
            None,
            fx=self.scale,
            fy=self.scale,
            interpolation=cv2.INTER_NEAREST,
        )

        cv2.imshow('local_map', display_img)
        cv2.waitKey(1)

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