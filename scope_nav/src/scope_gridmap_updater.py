#!/usr/bin/env python
#
# revision history: xzt
#  20250108 (AI): grid map coordinate transform utilities
#
# This module provides utility functions for transforming occupancy grid maps
# between different coordinate frames (pos_origin -> map -> hokuyo_link)
#------------------------------------------------------------------------------

import numpy as np

# Constants
IMG_SIZE = 64
RESOLUTION = 0.1
MAP_X_LIMIT = [0, 6.4]
MAP_Y_LIMIT = [-3.2, 3.2]


class GridMapTransformer:
    """
    Utility class for transforming occupancy grid maps between different coordinate frames
    """
    
    def __init__(self):
        pass
    
    @staticmethod
    def transform_map_to_map_frame(pred_mean_map, pos_origin):
        """
        Transform the local grid map from pos_origin reference frame to map (global) frame.
        
        Args:
            pred_mean_map: Prediction map [IMG_SIZE, IMG_SIZE]
            pos_origin: Origin position [x, y, theta]
            
        Returns:
            tuple: (x_global, y_global) coordinate arrays in global frame
        """
        if pred_mean_map is None or pos_origin is None:
            return None
        
        # Create a grid of map indices
        x_indices = np.arange(IMG_SIZE)
        y_indices = np.arange(IMG_SIZE)
        xx, yy = np.meshgrid(x_indices, y_indices)
        
        # Convert grid indices to physical coordinates in pos_origin frame
        x_pos_origin = MAP_X_LIMIT[0] + RESOLUTION * (xx + 0.5)
        y_pos_origin = MAP_Y_LIMIT[0] + RESOLUTION * (yy + 0.5)
        
        # Transform from pos_origin frame to global map frame
        theta_origin = pos_origin[2]
        cos_theta = np.cos(theta_origin)
        sin_theta = np.sin(theta_origin)
        
        # Apply rotation and translation
        x_global = (x_pos_origin * cos_theta - y_pos_origin * sin_theta + 
                   pos_origin[0])
        y_global = (x_pos_origin * sin_theta + y_pos_origin * cos_theta + 
                   pos_origin[1])
        
        return x_global, y_global
    
    @staticmethod
    def transform_map_to_hokuyo_frame(pred_mean_map, x_global, y_global, curr_pos):
        """
        Transform the map from global frame to current hokuyo_link frame.
        
        Args:
            pred_mean_map: Prediction map [IMG_SIZE, IMG_SIZE]
            x_global: x-coordinates in global frame
            y_global: y-coordinates in global frame
            curr_pos: Current robot position [x, y, theta]
            
        Returns:
            transformed_map: numpy array of shape [IMG_SIZE, IMG_SIZE]
        """
        if x_global is None or y_global is None:
            return None
        
        # Current robot position and orientation
        robot_x = curr_pos[0]
        robot_y = curr_pos[1]
        robot_theta = curr_pos[2]
        
        # Transform from global frame to hokuyo_link frame
        x_rel = x_global - robot_x
        y_rel = y_global - robot_y
        
        # Rotate by -robot_theta
        cos_theta = np.cos(-robot_theta)
        sin_theta = np.sin(-robot_theta)
        
        x_hokuyo = x_rel * cos_theta - y_rel * sin_theta
        y_hokuyo = x_rel * sin_theta + y_rel * cos_theta
        
        # Convert from global coordinates back to grid indices
        # based on hokuyo_link frame
        grid_x = (x_hokuyo - MAP_X_LIMIT[0]) / RESOLUTION - 0.5
        grid_y = (y_hokuyo - MAP_Y_LIMIT[0]) / RESOLUTION - 0.5
        
        # Create output map
        transformed_map = np.zeros((IMG_SIZE, IMG_SIZE), dtype=np.float32)
        
        # Bilinear interpolation
        for i in range(IMG_SIZE):
            for j in range(IMG_SIZE):
                # Find valid indices
                if 0 <= grid_x[i, j] < IMG_SIZE-1 and 0 <= grid_y[i, j] < IMG_SIZE-1:
                    x0 = int(np.floor(grid_x[i, j]))
                    x1 = x0 + 1
                    y0 = int(np.floor(grid_y[i, j]))
                    y1 = y0 + 1
                    
                    dx = grid_x[i, j] - x0
                    dy = grid_y[i, j] - y0
                    
                    # Bilinear interpolation
                    transformed_map[i, j] = (
                        pred_mean_map[x0, y0] * (1-dx) * (1-dy) +
                        pred_mean_map[x1, y0] * dx * (1-dy) +
                        pred_mean_map[x0, y1] * (1-dx) * dy +
                        pred_mean_map[x1, y1] * dx * dy
                    )
        
        return transformed_map
    
    @staticmethod
    def transform_pred_map_to_hokuyo(pred_mean_map, pos_origin, curr_pos):
        """
        Complete transformation pipeline: pos_origin -> global -> hokuyo_link
        
        Args:
            pred_mean_map: Prediction map [IMG_SIZE, IMG_SIZE]
            pos_origin: Origin position [x, y, theta]
            curr_pos: Current robot position [x, y, theta]
            
        Returns:
            transformed_map: Map in hokuyo_link frame [IMG_SIZE, IMG_SIZE]
        """
        # Step 1: Transform to global frame
        coords = GridMapTransformer.transform_map_to_map_frame(pred_mean_map, pos_origin)
        if coords is None:
            return None
        
        x_global, y_global = coords
        
        # Step 2: Transform to hokuyo_link frame
        transformed_map = GridMapTransformer.transform_map_to_hokuyo_frame(
            pred_mean_map, x_global, y_global, curr_pos
        )
        
        return transformed_map
