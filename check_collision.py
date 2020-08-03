#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import String
import numpy as np

def check_occupancy(data):
    ogm = data
    rospy.loginfo("Substribe to costmap/costmap")
    rospy.loginfo(ogm.info.height)
    rospy.loginfo(ogm.info.width)
    grid = np.asarray(ogm.data).reshape((ogm.info.height, ogm.info.width))
    positionX = ogm.info.origin.position.x
    positionY = ogm.info.origin.position.y
    resolution = ogm.info.resolution
    px = input("Enter the x corrdinate: ")
    py = input("Enter the y corrdinate: ")
    x = px - positionX/resolution
    y = py - positionY/resolution
    if(0 <= x <= 120 or 0 <= y <= 120):
        if(grid[int(x),int(y)] >= 100):
            rospy.loginfo("Occupided")
        elif(0 <= grid[int(x),int(y)] < 100):
            rospy.loginfo("Free")
        else:
            rospy.loginfo("Unknown")
    else:
        rospy.loginfo("point is not in the map")

def check_collision():
    rospy.init_node('check_collision', anonymous = True)
    rospy.Subscriber('/costmap/costmap', OccupancyGrid, check_occupancy)

    rospy.spin()

if __name__ == "__main__":
    
    check_collision()