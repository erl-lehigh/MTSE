#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import OccupancyGrid

import numpy as np


class StaticCostmapPublisher(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        # Read parameters
        self.debug = rospy.get_param("debug", True)

        self.pub_grid = rospy.Publisher("/costmap",     OccupancyGrid,
                                        queue_size=1,   latch=True)
        self.listener = tf.TransformListener()

        self.init_grid()

        self.timer = rospy.Timer(rospy.Duration(1), self.publish_grid)

        if self.debug:
            rospy.loginfo("[%s] Initialized. ", self.node_name)

    def init_grid(self):
        self.grid_msg = OccupancyGrid()
        # header
        self.grid_msg.header.frame_id = 'world'# FIXME: hack

        # info
        self.grid_msg.info.map_load_time = rospy.Time.now()
        self.grid_msg.info.resolution = 0.3
        self.grid_msg.info.width = 120 # 40/0.3
        self.grid_msg.info.height = self.grid_msg.info.width
        self.grid_msg.info.origin.position.x = 0 - self.grid_msg.info.width * self.grid_msg.info.resolution/2
        self.grid_msg.info.origin.position.y = 0 - self.grid_msg.info.height * self.grid_msg.info.resolution/2
        self.grid_msg.info.origin.position.z = 0

        #change 100 to 0 below and set case to 'None' for an obstacle free costmap
        grid  = 100 * np.ones((self.grid_msg.info.height, self.grid_msg.info.width),
                              dtype=np.int8)
        w = 10
        case = ''

        if case == 'medium2':
            grid[self.grid_msg.info.height/2-2*w:self.grid_msg.info.height/2+2*w, :] = 0
            grid[self.grid_msg.info.height/2-2*w:self.grid_msg.info.height/2+2*w+1, :] = \
                np.tile(np.abs(np.linspace(-100, 100, num=4*w+1)),
                        (self.grid_msg.info.width, 1)).T
            grid[self.grid_msg.info.height/2-w:self.grid_msg.info.height/2+w/3, self.grid_msg.info.width/2+2*w:self.grid_msg.info.width/2+4*w] = 100
        elif case == 'medium3':
            w=29
            grid[self.grid_msg.info.height/2-2*w:self.grid_msg.info.height/2+2*w+1, :] = \
                    np.tile(np.abs(np.linspace(-100, 100, num=4*w+1)),
                            (self.grid_msg.info.width, 1)).T
        else:
            pass

        if case == 'hard':
            grid[self.grid_msg.info.height/2-w:self.grid_msg.info.height/2+w/3, self.grid_msg.info.width/2+w:self.grid_msg.info.width/2+3*w] = 100
            grid[self.grid_msg.info.height/2+w/3:self.grid_msg.info.height/2+w, self.grid_msg.info.width/2+w:self.grid_msg.info.width/2+3*w] = \
                np.tile(np.abs(np.linspace(-90, 90, num=2*w/3+1)),
                        (2*w, 1)).T
        elif case == 'medium':
            grid[self.grid_msg.info.height/2-w:self.grid_msg.info.height/2+w/3, self.grid_msg.info.width/2+2*w:self.grid_msg.info.width/2+4*w] = 100
            grid[self.grid_msg.info.height/2+w/3:self.grid_msg.info.height/2+w, self.grid_msg.info.width/2+2*w:self.grid_msg.info.width/2+4*w] = \
                np.tile(np.abs(np.linspace(-90, 90, num=2*w/3+1)),
                        (2*w, 1)).T
        elif case == 'infeasible':
            grid[self.grid_msg.info.height/2-w:self.grid_msg.info.height/2+w, self.grid_msg.info.width/2+2*w:self.grid_msg.info.width/2+4*w] = 100
        elif case == 'invalid goal':
            grid[self.grid_msg.info.height/2-w:self.grid_msg.info.height/2+w/3, self.grid_msg.info.height-w/4:self.grid_msg.info.height] = 100

        self.grid_msg.data = grid.flatten()

    def publish_grid(self, event):
        self.grid_msg.header.stamp = rospy.Time.now()
        rospy.loginfo('Publish map!')
        self.pub_grid.publish(self.grid_msg)

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('test_omplrrt_publisher', anonymous=False)
    # Create the object
    node = StaticCostmapPublisher()
    # Keep it spinning to keep the node alive
    rospy.spin()
