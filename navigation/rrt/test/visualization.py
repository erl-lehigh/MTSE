#!/usr/bin/env python

import itertools as it

from copy import deepcopy
import threading as th

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from rrt.msg import TreeStamped


class TreeVisualizer(object):

    def __init__(self):

        # get params
        self.tree_pub_topic = rospy.get_param("tree_pub_topic", "planner_tree")
        self.tree_sub_topic = rospy.get_param("tree_sub_topic", "rrts_tree")
        self.tree_line_width = rospy.get_param("tree_line_width", 0.05)
        self.tree_pub = rospy.Publisher(self.tree_pub_topic, Marker, queue_size=1,
                                        latch=True)
        self.tree_marker = Marker()

        self.tree_sub = rospy.Subscriber(self.tree_sub_topic, TreeStamped,
                                         self.updateTree)

        self.timer = rospy.Timer(rospy.Duration(1), self.publish_tree)

    def updateTree(self, msg):
        points = []
        for pose, parent in it.izip(msg.nodes, msg.parents):
            if parent >= 0:
                points.append(pose.position)
                points.append(msg.nodes[parent].position)

        self.tree_marker.header = msg.header
        self.tree_marker.ns = 'tree'
        self.tree_marker.id = 0 
        self.tree_marker.type = Marker.LINE_LIST
        self.tree_marker.action = Marker.MODIFY
        self.tree_marker.pose.position.x = 0
        self.tree_marker.pose.position.y = 0
        self.tree_marker.pose.position.z = 0
        self.tree_marker.pose.orientation.x = 0
        self.tree_marker.pose.orientation.y = 0
        self.tree_marker.pose.orientation.z = 0
        self.tree_marker.pose.orientation.w = 1
        self.tree_marker.scale.x = self.tree_line_width
        self.tree_marker.scale.y = 1.0
        self.tree_marker.scale.z = 1.0
        self.tree_marker.color.g = 1.0
        self.tree_marker.color.a = 1.0
        self.tree_marker.points = points

    def publish_tree(self, event):
        self.tree_pub.publish(self.tree_marker)

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('omplrrt_visualization', anonymous=False)
    # Create the object
    node = TreeVisualizer()
    # Keep it spinning to keep the node alive
    rospy.spin()
