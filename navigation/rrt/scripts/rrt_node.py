#! /usr/bin/env python

import rospy
import message_filters
from std_msgs.msg import String
import numpy as np
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PoseStamped

from rrt import RRTPlanner


class RRTROSNode(object):
    '''TODO:
    '''

    def __init__(self):
        '''TODO:
        '''
        self.node_name = rospy.get_name()
        
        # Read parameters
        #TODO: read parameters
        # second parameter is default value
        self.example = rospy.get_param('~frequency', 10)
        
        self.rrt_planner = RRTPlanner()
        
        # Create publishers
        # TODO:
        # second parameter is the message type for the topic
        self.example_pub  = rospy.Publisher('RRT_Path', Path,
                                            queue_size=1)

        # Create subscribers to costmap and 2dpose goal 
        self.occupancy_sub = rospy.Subscriber('/costmap/costmap', OccupancyGrid, self.callback)
        # self.pose_sub = rospy.Subscriber('/move_base_simple/pose', PoseStamped)
        self.goal_sub = rospy.Subscriber('/goal', Pose, self.computePath)
        # self.sub = message_filters.TimeSynchronizer([self.occupancy_sub, self.pose_sub], 10)
        # self.sub.registerCallback(self.example_callback)

        rospy.loginfo('[%s] Node started!', self.node_name)

    def computePath(self, msg):

        goalState = DubinState(x=msg.position.x, y=msg.position.y, yaw = 0, v = 0, omega = 0)
        path = self.rrt_planner.plan(goalState)
        finalpath = Path()
        for state in path:
            pose = PoseStamped()
            pose.pose.position.x = state.x
            pose.pose.position.y = state.y
            pose.pose.orientation = # yaw transform into orient
        self.example_pub.publish(finalpath)


    def callback(self, msg):

        self.rrt_planner.costmap = msg

        '''Example message callback.
        ogm = msg
        height = ogm.info.height
        width = ogm.info.width
        grid = np.asarray(ogm.data).reshape((height, width))
        positionX = ogm.info.origin.position.x
        positionY = ogm.info.origin.position.y
        resolution = ogm.info.resolution
        x = 0 - positionX/resolution
        y = 0 - positionY/resolution
        if(0 <= x <= width or 0 <= y <= height):
            if(grid[int(x),int(y)] >= 100):
                rospy.loginfo("Occupided")
            elif(0 <= grid[int(x),int(y)] < 100):
                rospy.loginfo("Free")
            else:
                rospy.loginfo("Unknown")
        else:
            rospy.loginfo("point is not in the map")'''


if __name__ == "__main__":
    # Initialize node with rospy
    rospy.init_node('rrt', anonymous=False)
    # Create the node object
    _ = RRTROSNode()
    # Keep the node alive
    rospy.spin()
