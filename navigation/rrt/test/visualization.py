#!/usr/bin/env python
'''
TODO:
'''

import itertools as it

from copy import deepcopy
import threading as th

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# TODO: import needs to match package name, e.g., "rrt"
from TODO.msg import TreeStamped


class TreeVisualizer(object):

    def __init__(self):
        self.lock = th.Lock()

        # TODO: expose topic name as parameter
        self.tree_pub = rospy.Publisher("planner_tree", Marker, queue_size=1,
                                        latch=True)
        self.tree_marker = Marker()

        # TODO: expose topic name as parameter
        self.tree_sub = rospy.Subscriber("rrts_tree", TreeStamped,
                                         self.updateTree)

        self.timer = rospy.Timer(rospy.Duration(1), self.publish_tree)

    def updateTree(self, msg):
        points = []
        for pose, parent in it.izip(msg.nodes, msg.parents):
            if parent >= 0:
                points.append(pose.position)
                points.append(msg.nodes[parent].position)

        self.lock.acquire()
        self.tree_marker.header = msg.header
        self.tree_marker.ns = 'tree'
        self.tree_marker.id = 0 #TODO:
        self.tree_marker.type = Marker.LINE_LIST
        self.tree_marker.action = Marker.MODIFY
        self.tree_marker.pose.position.x = 0
        self.tree_marker.pose.position.y = 0
        self.tree_marker.pose.position.z = 0
        self.tree_marker.pose.orientation.x = 0
        self.tree_marker.pose.orientation.y = 0
        self.tree_marker.pose.orientation.z = 0
        self.tree_marker.pose.orientation.w = 1
        self.tree_marker.scale.x = 0.05 #TODO: expose line width as parameter
        self.tree_marker.scale.y = 1.0
        self.tree_marker.scale.z = 1.0
        self.tree_marker.color.g = 1.0
        self.tree_marker.color.a = 1.0
        self.tree_marker.points = points
        self.lock.release()

    def publish_tree(self, event):
        self.lock.acquire()
        self.tree_pub.publish(self.tree_marker)
        self.lock.release()

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('omplrrt_visualization', anonymous=False)
    # Create the object
    node = TreeVisualizer()
    # Keep it spinning to keep the node alive
    rospy.spin()
