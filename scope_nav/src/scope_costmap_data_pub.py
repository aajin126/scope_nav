#!/usr/bin/env python
#
# revision history: xzt
#  20240604 (TE): first version
#
# usage: python scope_costmap_data_pub.py
#
# This script is the SCOPE Costmap code of the SCOPE-NAV navigation framework.
#------------------------------------------------------------------------------

from random import choice
import rospy
# custom define messages:
from scope_msgs.msg import ScopeInputData, ScopeOutputData
from geometry_msgs.msg import Point, PoseStamped, Twist, TwistStamped
from people_msgs.msg import People, Person
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Header
import tf.transformations as tft
from vox_msgs.msg import VoxGrid
# python: 
import numpy as np
import math
# import the model and all of its variables/functions
#
from model import *
from local_occ_grid_map import LocalMap
from reproj import reprojection, reprojection_to_map
import torch
import threading
from omegaconf import OmegaConf
from util import instantiate_from_config
from models.ddim import DDIMSampler
import time

# Constants
IMG_SIZE = 64 #80
SEQ_LEN = 10
NUM_CLASSES = 1
NUM_INPUT_CHANNELS = 1
NUM_LATENT_DIM = 512 #800 #512
NUM_OUTPUT_CHANNELS = NUM_CLASSES
NUM_TP = 10     # the number of timestamps

# Init map parameters
P_prior = 0.5	# Prior occupancy probability
P_occ = 0.7	    # Probability that cell is occupied with total confidence
P_free = 0.3	# Probability that cell is free with total confidence 
MAP_X_LIMIT = [0, 6.4]#[0, 8]      # Map limits on the x-axis
MAP_Y_LIMIT = [-3.2, 3.2]#[-4, 4]   # Map limits on the y-axis
RESOLUTION = 0.1        # Grid resolution in [m]'
TRESHOLD_P_OCC = 0.8    # Occupancy threshold

# for reproducibility, we seed the rng
#
set_seed(SEED1)        
# set the device to use GPU if available:
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class ScopeCostmap:
    # Constructor
    def __init__(self):
        # initialize data:  
        self.scan_ranges = np.zeros(1080)
        self.curr_vel = np.zeros(2)
        self.curr_pos = np.zeros(3)
        self.curr_odom = np.zeros(3)
        self.scope_header = Header()
        self.occ_grid = [] #np.zeros((IMG_SIZE, IMG_SIZE))
        # input:
        self.scans = []
        self.positions = []
        self.velocities = []
        self.header = Header() 
        self.tf_listener = None
        print(os.getcwd())

        # initialize ROS objects
        self.scope_input_data_sub = rospy.Subscriber("scope_input_data", ScopeInputData, self.scope_input_data_callback)
    
        self.scope_output_data_pub = rospy.Publisher('scope_output_data', ScopeOutputData, queue_size=1, latch=False)
        self.scope_prediction_pub = rospy.Publisher('scope_prediction', People, queue_size=1, latch=False)
        self.scope_uncertainty_pub = rospy.Publisher('scope_uncertainty', People, queue_size=1, latch=False)
        self.local_map_pub = rospy.Publisher('local_map', OccupancyGrid, queue_size=1, latch=False)
        self.voxgrid_pub = rospy.Publisher('plan_costmap_3D', VoxGrid, queue_size=1, latch=False)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        base_configs = rospy.get_param("~base_configs")
        if isinstance(base_configs, str):
            base_configs = [base_configs]
        ckpt_path = rospy.get_param("~model_file")
        self.ddim_steps = rospy.get_param("~ddim_steps", 8)
        self.ddim_eta = rospy.get_param("~ddim_eta", 1.0)

        configs = [OmegaConf.load(p) for p in base_configs]
        config = OmegaConf.merge(*configs)

        self.model = instantiate_from_config(config.model).to(self.device)

        state_dict = torch.load(ckpt_path, map_location="cpu")
        self.model.load_state_dict(state_dict, strict=False)
        self.model.eval()

        self.sampler = DDIMSampler(self.model)

        # Lock
        self.lock = threading.Lock() # lock to keep twist/time thread safe

        # timer:
        self.ts_cnt = 0
        self.rate = 10  # 20 Hz velocity controller
        self.timer = rospy.Timer(rospy.Duration(1./self.rate), self.timer_callback)
    

    # Callback function for the local map subscriber
    def scope_input_data_callback(self, scope_input_data_msg):
        # get the local occupancy grid map data:
        self.scope_header = scope_input_data_msg.header
        self.scan_ranges = np.array(scope_input_data_msg.scan_ranges, dtype=np.float32)
        self.curr_vel = np.array(scope_input_data_msg.curr_vel, dtype=np.float32)
        self.curr_pos = np.array(scope_input_data_msg.curr_pos, dtype=np.float32)
        self.curr_odom = np.array(scope_input_data_msg.curr_odom, dtype=np.float32)

    # function that runs every time the timer finishes to ensure that vae data are sent regularly
    def timer_callback(self, event):  
        # collect 10 time step data:
        self.lock.acquire()
        self.header = self.scope_header
        self.scans.append(self.scan_ranges)
        self.positions.append(self.curr_pos)
        self.velocities.append(self.curr_vel)
        self.lock.release()

        self.ts_cnt = self.ts_cnt + 1

        if(self.ts_cnt == NUM_TP): 
            ## predocc inference:
            # collect the samples as a batch:
            scans = torch.FloatTensor(self.scans).unsqueeze(0)
            scans = scans.to(device)
            positions = torch.FloatTensor(self.positions).unsqueeze(0)
            positions = positions.to(device)

            # create occupancy maps:
            batch_size = scans.size(0)
            prediction_maps = torch.zeros(SEQ_LEN, 1, IMG_SIZE, IMG_SIZE).to(device)
            # one prediction: 10 time steps:

            #t0 = time.perf_counter()

            # Create input grid maps: 
            input_gridMap = LocalMap(X_lim = MAP_X_LIMIT, 
                        Y_lim = MAP_Y_LIMIT, 
                        resolution = RESOLUTION, 
                        p = P_prior,
                        size=[batch_size, SEQ_LEN],
                        device = device)
            pos_origin = positions[:, SEQ_LEN-1]
            # robot positions:
            pos = positions[:,:SEQ_LEN]
            # Transform the robot past poses to the predicted reference frame.
            x_odom, y_odom, theta_odom =  input_gridMap.robot_coordinate_transform(pos, pos_origin)
            # Lidar measurements:
            distances = scans[:,:SEQ_LEN]
            # the angles of lidar scan: -135 ~ 135 degree
            angles = torch.linspace(-(135*np.pi/180), 135*np.pi/180, distances.shape[-1]).to(device)
            # Lidar measurements in X-Y plane: transform to the predicted robot reference frame
            distances_x, distances_y = input_gridMap.lidar_scan_xy(distances, angles, x_odom, y_odom, theta_odom)
            # discretize to binary maps:
            input_binary_maps = input_gridMap.discretize(distances_x, distances_y)
            # local occupancy map update:
            input_gridMap.update(x_odom, y_odom, distances_x, distances_y, P_free, P_occ)
            input_occ_grid_map = input_gridMap.to_prob_occ_map(TRESHOLD_P_OCC)
            # binary occupancy maps:
            input_binary_maps = input_binary_maps.unsqueeze(2)

            # feed the batch to the network:
            num_samples = 1
            inputs_samples = input_binary_maps.repeat(num_samples,1,1,1,1)
            inputs_occ_map_samples = input_occ_grid_map.repeat(num_samples,1,1,1)

            c, _ = self.model.get_encoding(inputs_samples, None, inputs_occ_map_samples)
            c_exp = c.repeat_interleave(self.model.first_stage_model.seq_len, dim=0) # (B*T, 32, 16, 16)
 
            # DDIM sampling from random noise
            with self.model.ema_scope("Evaluation"):
                z_samples, _ = self.model.sample_log(
                    cond=c_exp,
                    batch_size=c_exp.shape[0],
                    ddim=True,
                    ddim_steps=self.ddim_steps,
                    eta=self.ddim_eta
                )
            
            # Decode latent to sequence
            prediction_seq = self.model.decode_first_stage(z_samples)  # (num_samples, T, 1, H, W)

            for t in range(SEQ_LEN):
                pred_mean = torch.mean(prediction_seq[:, t], dim=0, keepdim=True)
                prediction_maps[t, 0] = pred_mean.squeeze()

            ##
            ## final prediction map: transform to the current robot frame
            ##
            if self.tf_listener is None:
                import tf
                self.tf_listener = tf.TransformListener()

            try:
                (trans, rot) = self.tf_listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
                (_, _, theta) = tft.euler_from_quaternion(rot)
                curr_pos_tf = np.array([trans[0], trans[1], theta], dtype=np.float32)
            except Exception as e:
                rospy.logwarn('TF lookup failed, fallback to last pose: %s', str(e))
                curr_pos_tf = self.curr_pos.copy()

            curr_pos_t = torch.from_numpy(curr_pos_tf).float().to(device).view(1, 1, 3)
            x_now, y_now, th_now = input_gridMap.robot_coordinate_transform(curr_pos_t, pos_origin)
            x_now = x_now[:, 0]
            y_now = y_now[:, 0]
            th_now = th_now[:, 0]

            merged_prediction_map = torch.amax(prediction_maps[:SEQ_LEN], dim=0, keepdim=True)

            fin_prediction_map, _ = reprojection(
                merged_prediction_map,
                x_now,
                y_now,
                th_now,
                MAP_X_LIMIT,
                MAP_Y_LIMIT,
            )

            # t1 = time.perf_counter()
            # print(f"[DEBUG] bit-packing time: {(t1 - t0)*1000:.3f} ms")

            ### visualize voxgrid map: 

            # xmin = MAP_X_LIMIT[0]
            # xmax = MAP_X_LIMIT[1]
            # ymin = MAP_Y_LIMIT[0]
            # ymax = MAP_Y_LIMIT[1]

            # corners_local = torch.tensor([
            #     [xmin, ymin],
            #     [xmin, ymax],
            #     [xmax, ymin],
            #     [xmax, ymax],
            # ], device=pos_origin.device, dtype=pos_origin.dtype)

            # x0 = pos_origin[0, 0]
            # y0 = pos_origin[0, 1]
            # th = pos_origin[0, 2]

            # ct = torch.cos(th)
            # st = torch.sin(th)

            # cx = corners_local[:, 0]
            # cy = corners_local[:, 1]

            # corners_x_map = x0 + ct * cx - st * cy
            # corners_y_map = y0 + st * cx + ct * cy

            # map_x_min = corners_x_map.min()
            # map_x_max = corners_x_map.max()
            # map_y_min = corners_y_map.min()
            # map_y_max = corners_y_map.max()

            # reprojected_maps = []
            # for t in range(SEQ_LEN):
            #     pred_map_t = prediction_maps[t:t+1]
            #     pos_origin_map = pos_origin[0]
            #     dx = pos_origin_map[0]
            #     dy = pos_origin_map[1]
            #     dtheta = pos_origin_map[2]
            #     fin_map_t = reprojection_to_map(
            #         pred_map_t,
            #         x0,
            #         y0,
            #         th,
            #         MAP_X_LIMIT,
            #         MAP_Y_LIMIT,
            #         map_x_min,
            #         map_x_max,
            #         map_y_min,
            #         map_y_max,
            #         IMG_SIZE,
            #         IMG_SIZE,
            #     )
            #     reprojected_maps.append(fin_map_t.squeeze(0).squeeze(0))  # (H, W)

            # # (T, H, W) -> uint8 [0,255]
            # reprojected_stack = torch.stack(reprojected_maps, dim=0)   # (T, H, W)
            # # vox_data = (reprojected_stack * 255).to(torch.uint8).cpu().numpy()  # (T, H, W)
            # vox_data = (reprojected_stack * 255).to(torch.uint8).cpu().numpy()   # (T, X, Y)
            # vox_data = np.transpose(vox_data, (0, 2, 1))  # (T, Y, X)         

            # # VoxGrid.msg fields (vox_msgs/VoxGrid):
            # # std_msgs/Header  header 
            # # uint32 height
            # # uint32 width
            # # uint32 depth
            # # float32 dl
            # # float32 dt
            # # geometry_msgs/Point origin
            # # float32 theta
            # # uint8[] data

            # vox_msg = VoxGrid()
            # vox_msg.header.stamp = rospy.Time.now()
            # vox_msg.header.frame_id = "map"
            # vox_msg.height = IMG_SIZE
            # vox_msg.width = IMG_SIZE
            # vox_msg.depth = SEQ_LEN
            # vox_msg.dl = (map_x_max - map_x_min) / IMG_SIZE
            # vox_msg.dt = 0.1
            # vox_msg.origin.x = map_x_min
            # vox_msg.origin.y = map_y_min
            # vox_msg.origin.z = 0.0
            # vox_msg.theta = 0.0
            # vox_msg.data = vox_data.flatten().tolist()

            # self.voxgrid_pub.publish(vox_msg)

            #----------------------------------------------------------#
            ##################  Publish local map  #####################

            # get the output:
            pred_mean_map = fin_prediction_map.detach().cpu().numpy()

            # visualize the local occupancy map:
            # create message:
            occ_map = OccupancyGrid()
            # initialize header:
            #occ_map.header = self.header
            occ_map.header.stamp = rospy.Time.now()
            occ_map.header.frame_id = "hokuyo_link" #scan_msg.header.frame_id
            # initialize info:
            occ_map.info.map_load_time = rospy.Time.now()
            occ_map.info.resolution = RESOLUTION #xy_resolution
            occ_map.info.width = IMG_SIZE #width
            occ_map.info.height = IMG_SIZE #height
            occ_map.info.origin.position.x = MAP_X_LIMIT[0] #min_x
            occ_map.info.origin.position.y = MAP_Y_LIMIT[0]# min_y
            # initialize data:
            occ_pred_map = pred_mean_map.squeeze().transpose().reshape(-1).tolist()
            occ_map.data = [int(val*100) for val in occ_pred_map]#.transpose() for val in sublist]

            # publish local map msg:
            self.local_map_pub.publish(occ_map)


            #----------------------------------------------------------#
            ##################  Publish People  #####################

            occ_scope_pred = People()
            #occ_scope_pred.header = self.header
            occ_scope_pred.header.stamp = rospy.Time.now()
            occ_scope_pred.header.frame_id = "hokuyo_link"

            # get occupied indicies:
            pred_mean_occ = fin_prediction_map.squeeze()
            idx_occ = torch.nonzero((pred_mean_occ > 0.05))

            # translate grid indicies to the physical positions:
            px = MAP_X_LIMIT[0] + RESOLUTION*(idx_occ[:, 0] + 0.5)
            py = MAP_Y_LIMIT[0] + RESOLUTION*(idx_occ[:, 1] + 0.5)
            px = px.detach().cpu().numpy()
            py = py.detach().cpu().numpy()
            idx_occ = idx_occ.detach().cpu().numpy()

            for i in range(len(px)):
                o_pose = Person()
                o_pose.position.x = px[i]
                o_pose.position.y = py[i]
                o_pose.position.z = 0
                row, col = idx_occ[i]
                o_pose.probability = float(pred_mean_occ[row, col].item() * 254.0)
                occ_scope_pred.people.append(o_pose)
            # publish prediction map:
            self.scope_prediction_pub.publish(occ_scope_pred)
        
            # reset the position data list:
            self.ts_cnt = NUM_TP-1
            self.scans = self.scans[1:NUM_TP]
            self.positions = self.positions[1:NUM_TP]
            self.velocities = self.velocities[1:NUM_TP]
        
        
if __name__ == '__main__':
    rospy.init_node('scope_nav')
    scope_costmap = ScopeCostmap()
    rospy.spin()



