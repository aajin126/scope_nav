#!/usr/bin/env python3
# ROS2 port of scope_costmap_data_pub.py (same behavior)
# - Sub: scope_input_data (scope_msgs/ScopeInputData), odom (nav_msgs/Odometry)
# - Pub: scope_output_data (scope_msgs/ScopeOutputData), scope_prediction (people_msgs/People),
#        local_map + updated_local_map (nav_msgs/OccupancyGrid)
# - Keeps model inference + timer-driven pipeline identical

import threading
import numpy as np
import rospy
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Header
from nav_msgs.msg import Odometry, OccupancyGrid
from people_msgs.msg import People, Person

from scope_msgs.msg import ScopeInputData, ScopeOutputData

from tf_transformations import euler_from_quaternion

# keep your original imports (must be in PYTHONPATH / same package)
from model import *                 # noqa: F401,F403
from local_occ_grid_map import LocalMap
from scope_gridmap_updater import GridMapTransformer  # noqa: F401
from reproj import reprojection


# Constants (same as original)
IMG_SIZE = 64
SEQ_LEN = 10
NUM_CLASSES = 1
NUM_INPUT_CHANNELS = 1
NUM_LATENT_DIM = 512
NUM_OUTPUT_CHANNELS = NUM_CLASSES
NUM_TP = 10

P_prior = 0.5
P_occ = 0.7
P_free = 0.3
MAP_X_LIMIT = [0, 6.4]
MAP_Y_LIMIT = [-3.2, 3.2]
RESOLUTION = 0.1
TRESHOLD_P_OCC = 0.8

# for reproducibility
set_seed(SEED1)  # noqa: F405
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


class ScopeCostmap(Node):
    def __init__(self):
        super().__init__('scope_nav')

        # buffers
        self.scan_ranges = np.zeros(1080, dtype=np.float32)
        self.curr_vel = np.zeros(2, dtype=np.float32)
        self.curr_pos = np.zeros(3, dtype=np.float32)
        self.curr_pos_np = None  # np.array([x,y,yaw])
        self.curr_odom = np.zeros(3, dtype=np.float32)
        self.scope_header = Header()

        self.scans = []
        self.positions = []
        self.velocities = []
        self.header = Header()
        self.lock = threading.Lock()

        qos10 = QoSProfile(depth=10)

        self.declare_parameter('statistics_file', '')
        occ_entropy_path = self.get_parameter('statistics_file').value
        truncnorm_skewcauchy_occ_entropy = np.load(occ_entropy_path)
        self.c_entropy_table = torch.tensor(truncnorm_skewcauchy_occ_entropy).to(device)
        self.p_bins = torch.linspace(0, 1, steps=16).to(device)
        
        self.scope_input_data_sub = self.create_subscription(
            ScopeInputData, 'scope_input_data', self.scope_input_data_callback, qos10
        )

        self.scope_output_data_pub = self.create_publisher(ScopeOutputData, 'scope_output_data', QoSProfile(depth=1))
        self.scope_prediction_pub = self.create_publisher(People, 'scope_prediction', QoSProfile(depth=1))
        self.local_map_pub = self.create_publisher(OccupancyGrid, 'local_map', QoSProfile(depth=1))

        # So-SCOPE model:
        self.model = so_scope( 
            input_channels=NUM_INPUT_CHANNELS,
            latent_dim=NUM_LATENT_DIM,
            output_channels=NUM_OUTPUT_CHANNELS,
        )
        self.model.to(device)
        self.model.eval()

        # parameter: model_file
        self.declare_parameter('model_file', './model/predocc_vae/v1.6/model.pth')
        model_file = self.get_parameter('model_file').value

        checkpoint = torch.load(model_file, map_location=device)
        self.model.load_state_dict(checkpoint['model'])
        self.get_logger().info(f'Finish loading predocc_vae model on {device} from: {model_file}')

        # timer
        self.ts_cnt = 0
        self.rate = 10.0
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def scope_input_data_callback(self, msg: ScopeInputData):
        with self.lock:
            self.scope_header = msg.header
            self.scan_ranges = np.array(msg.scan_ranges, dtype=np.float32)
            self.curr_vel = np.array(msg.curr_vel, dtype=np.float32)
            self.curr_pos = np.array(msg.curr_pos, dtype=np.float32)
            self.curr_odom = np.array(msg.curr_odom, dtype=np.float32)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        with self.lock:
            self.curr_pos_np = np.array([p.x, p.y, yaw], dtype=np.float32)

    def timer_callback(self):
        # collect one timestep
        with self.lock:
            self.header = self.scope_header
            self.scans.append(self.scan_ranges.copy())
            self.positions.append(self.curr_pos.copy())
            self.velocities.append(self.curr_vel.copy())
            curr_np = None if self.curr_pos_np is None else self.curr_pos_np.copy()

        self.ts_cnt += 1

        if self.ts_cnt != NUM_TP:
            return

        # need current pose for reprojection; if not yet available, just slide window and wait
        if curr_np is None:
            self.ts_cnt = NUM_TP - 1
            self.scans = self.scans[1:NUM_TP]
            self.positions = self.positions[1:NUM_TP]
            self.velocities = self.velocities[1:NUM_TP]
            return

        # build tensors
        scans = torch.from_numpy(np.asarray(self.scans, dtype=np.float32)).unsqueeze(0).to(device)
        positions = torch.from_numpy(np.asarray(self.positions, dtype=np.float32)).unsqueeze(0).to(device)

        batch_size = scans.size(0)
        prediction_maps = torch.zeros(SEQ_LEN, 1, IMG_SIZE, IMG_SIZE).to(device)

        # Create input grid maps
        input_gridMap = LocalMap(
            X_lim=MAP_X_LIMIT,
            Y_lim=MAP_Y_LIMIT,
            resolution=RESOLUTION,
            p=P_prior,
            size=[batch_size, SEQ_LEN],
            device=device,
        )

        pos_origin = positions[:, SEQ_LEN - 1]
        pos = positions[:, :SEQ_LEN]
        x_odom, y_odom, theta_odom = input_gridMap.robot_coordinate_transform(pos, pos_origin)

        distances = scans[:, :SEQ_LEN]
        angles = torch.linspace(-(135 * np.pi / 180.0), 135 * np.pi / 180.0, distances.shape[-1]).to(device)

        distances_x, distances_y = input_gridMap.lidar_scan_xy(distances, angles, x_odom, y_odom, theta_odom)
        input_binary_maps = input_gridMap.discretize(distances_x, distances_y).unsqueeze(2)

        input_gridMap.update(x_odom, y_odom, distances_x, distances_y, P_free, P_occ)
        input_occ_grid_map = input_gridMap.to_prob_occ_map(TRESHOLD_P_OCC)

        num_samples = 1
        inputs_samples = input_binary_maps.repeat(num_samples, 1, 1, 1, 1)
        inputs_occ_map_samples = input_occ_grid_map.repeat(num_samples, 1, 1, 1, 1)
        
        curr_pos_t = torch.from_numpy(curr_np).float().to(device).view(1, 3)
        dx = curr_pos_t[:, 0] - pos_origin[:, 0]
        dy = curr_pos_t[:, 1] - pos_origin[:, 1]
        theta_ref = pos_origin[:, 2]
        x_curr = torch.cos(theta_ref) * dx + torch.sin(theta_ref) * dy
        y_curr = torch.sin(-theta_ref) * dx + torch.cos(theta_ref) * dy
        th_curr = curr_pos_t[:, 2] - theta_ref

        with torch.no_grad():
            prediction, _kl = self.model(inputs_samples, inputs_occ_map_samples)

        prediction_seq = prediction[:, :SEQ_LEN]

        for t in range(SEQ_LEN):
            pred_mean = torch.mean(prediction_seq[:, t], dim=0, keepdim=True)
            prediction_maps[t, 0] = pred_mean.squeeze()

        merged_prediction_map = torch.amax(prediction_maps[:SEQ_LEN], dim=0, keepdim=True)
        fin_prediction_map, _ = reprojection(
            merged_prediction_map,
            x_curr,
            y_curr,
            th_curr,
            MAP_X_LIMIT,
            MAP_Y_LIMIT,
        )

        # publish People prediction
        occ_scope_pred = People()
        occ_scope_pred.header.stamp = self.get_clock().now().to_msg()
        occ_scope_pred.header.frame_id = 'base_scan'

        pred_map_occ = fin_prediction_map.squeeze(0).detach().cpu().numpy().copy()
        pred_map_occ[pred_map_occ < 0.5] = 0.0
        idx_occ = np.argwhere(pred_map_occ > 0.5)

        if len(idx_occ) > 0:
            px = MAP_X_LIMIT[0] + RESOLUTION * (idx_occ[:, 0] + 0.5)
            py = MAP_Y_LIMIT[0] + RESOLUTION * (idx_occ[:, 1] + 0.5)
            for i in range(len(px)):
                person = Person()
                person.position.x = float(px[i])
                person.position.y = float(py[i])
                person.position.z = 0.0
                occ_scope_pred.people.append(person)

        self.scope_prediction_pub.publish(occ_scope_pred)


        # publish OccupancyGrid local_map (pred_mean_map from last step)
        pred_mean_map = fin_prediction_map.detach().cpu().numpy()
        occ_map = OccupancyGrid()
        occ_map.header.stamp = self.get_clock().now().to_msg()
        occ_map.header.frame_id = 'base_scan'

        occ_map.info.map_load_time = occ_map.header.stamp
        occ_map.info.resolution = float(RESOLUTION)
        occ_map.info.width = int(IMG_SIZE)
        occ_map.info.height = int(IMG_SIZE)
        occ_map.info.origin.position.x = float(MAP_X_LIMIT[0])
        occ_map.info.origin.position.y = float(MAP_Y_LIMIT[0])
        occ_pred_map = pred_mean_map.squeeze().transpose().reshape(-1).tolist()
        occ_map.data = [int(float(val) * 100.0) for val in occ_pred_map]

        self.local_map_pub.publish(occ_map)

        # slide window (same as original)
        self.ts_cnt = NUM_TP - 1
        self.scans = self.scans[1:NUM_TP]
        self.positions = self.positions[1:NUM_TP]
        self.velocities = self.velocities[1:NUM_TP]


def main():
    rclpy.init()
    node = ScopeCostmap()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()