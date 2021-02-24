#!/usr/bin/env python
'''
TODO:
'''

# import itertools as it

import rospy
import tf
from nav_msgs.msg import OccupancyGrid

import numpy as np
# from scipy import interpolate
# from scipy.spatial.distance import euclidean as dist


class DummyPublisher(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        # rospy.loginfo("[%s] Initializing " %(self.node_name))
        # Read parameters
        self.debug = rospy.get_param("debug", True)

        self.pub_grid = rospy.Publisher("/todo/costmap_node/costmap/costmap",
                                        OccupancyGrid, queue_size=1,
                                        latch=True)
        self.listener = tf.TransformListener()

        self.init_grid()

        self.timer = rospy.Timer(rospy.Duration(1), self.publish_grid)

        if self.debug:
            rospy.loginfo("[%s] Initialized. ", self.node_name)

    def init_grid(self):
        self.grid_msg = OccupancyGrid()
        # header
        self.grid_msg.header.frame_id = 'TODO:'# FIXME: hack

        # self.listener.waitForTransform('todo/base_link', '/todo/map',
        #                                rospy.Time.now(), rospy.Duration(4.0))
        # (trans,rot) = self.listener.lookupTransform('/todo/map',
        #                               'todo/base_link', rospy.Time(0))
        #
        # rospy.loginfo('Position of vehicle: %s, %s', trans[0], trans[1])

        # info
        self.grid_msg.info.map_load_time = rospy.Time.now()
        self.grid_msg.info.resolution = 0.3
        self.grid_msg.info.width = 120 # 40/0.3
        self.grid_msg.info.height = self.grid_msg.info.width
        self.grid_msg.info.origin.position.x = 32.33 - self.grid_msg.info.width * self.grid_msg.info.resolution/2
        self.grid_msg.info.origin.position.y = 23.55 - self.grid_msg.info.height * self.grid_msg.info.resolution/2
        self.grid_msg.info.origin.position.z = 0

        grid  = 100 * np.ones((self.grid_msg.info.height, self.grid_msg.info.width),
                              dtype=np.int8)
        w = 10
        case = 'medium2'

        if case == 'medium2':
            grid[self.grid_msg.info.height/2-2*w:self.grid_msg.info.height/2+2*w, :] = 0
            grid[self.grid_msg.info.height/2-2*w:self.grid_msg.info.height/2+2*w+1, :] = \
                np.tile(np.abs(np.linspace(-100, 100, num=4*w+1)),
                        (self.grid_msg.info.width, 1)).T;
            grid[self.grid_msg.info.height/2-w:self.grid_msg.info.height/2+w/3, self.grid_msg.info.width/2+2*w:self.grid_msg.info.width/2+4*w] = 100
        elif case == 'medium3':
            w=29
            grid[self.grid_msg.info.height/2-2*w:self.grid_msg.info.height/2+2*w+1, :] = \
                    np.tile(np.abs(np.linspace(-100, 100, num=4*w+1)),
                            (self.grid_msg.info.width, 1)).T;
        else:
            grid[self.grid_msg.info.height/2-w:self.grid_msg.info.height/2+w, :] = 0
            grid[self.grid_msg.info.height/2-w:self.grid_msg.info.height/2+w+1, :] = \
                np.tile(np.abs(np.linspace(-100, 100, num=2*w+1)),
                        (self.grid_msg.info.width, 1)).T;

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
        # else: pass # no obstacle

        self.grid_msg.data = grid.flatten()


    def publish_grid(self, event):
        self.grid_msg.header.stamp = rospy.Time.now()
        rospy.loginfo('Publish map!')
        self.pub_grid.publish(self.grid_msg)

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('test_omplrrt_publisher', anonymous=False)
    # Create the object
    node = DummyPublisher()
    # Keep it spinning to keep the node alive
    rospy.spin()
