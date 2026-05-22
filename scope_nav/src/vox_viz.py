#!/usr/bin/env python

import rospy
from vox_msgs.msg import VoxGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def voxgrid_callback(msg):
    marker = Marker()
    marker.header = msg.header
    marker.ns = "voxgrid"
    marker.id = 0
    marker.type = Marker.CUBE_LIST
    marker.action = Marker.ADD
    marker.scale.x = msg.dl
    marker.scale.y = msg.dl
    marker.scale.z = msg.dt
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.5

    for z in range(msg.depth):     
        for y in range(msg.height): 
            for x in range(msg.width):
                idx = z * msg.height * msg.width + y * msg.width + x
                if msg.data[idx] > 0:  # threshold
                    p = Point()
                    p.x = msg.origin.x + (x + 0.5) * msg.dl
                    p.y = msg.origin.y + (y + 0.5) * msg.dl
                    p.z = msg.origin.z + (z + 0.5) * msg.dt
                    marker.points.append(p)
    marker_pub.publish(marker)

if __name__ == "__main__":
    rospy.init_node("voxgrid_viz")
    marker_pub = rospy.Publisher("voxgrid_marker", Marker, queue_size=1)
    rospy.Subscriber("/temporal_grid_local_map", VoxGrid, voxgrid_callback)
    rospy.spin()