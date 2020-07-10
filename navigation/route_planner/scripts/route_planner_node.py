#! /usr/bin/env python

import rospy
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
#import sys
#print(sys.path)
#import Rtree==0.8.3
import osmnx as ox
import networkx as nx

from route_planner import RoutePlanner
from std_msgs.msg import *
#RoutePlanner
class RoutePlannerNode(object):
    '''TODO:
    '''
    @staticmethod
    def printGraph(msg):
        center_point = (40.6038914, -75.3739361)
        g = ox.graph_from_point(center_point, distance=1500, network_type='drive')
        orig = (msg.data[0],msg.data[1]) 
        dest = (40.6054017, -75.3758301)
        origin_node = ox.get_nearest_node(g, orig)
        destination_node = ox.get_nearest_node(g, dest)
        route = nx.shortest_path(g, origin_node, destination_node)
        fig, ax = ox.plot.plot_graph_route(g, route, route_linewidth=6, node_size=0, bgcolor='k')
        ox.plot_graph(g)

    def __init__(self):
        '''TODO:
        '''
        self.node_name = rospy.get_name()

        # Read parameters
        #TODO: read parameters
        # second parameter is default value
        self.example = rospy.get_param('~parameter', 0)

        # Create publishers
        # TODO:
        # second parameter is the message type for the topic
        self.example_pub = rospy.Publisher('currPos', Float64MultiArray , queue_size=10)

        # Create subscribers
        #CHECK MESSAGE FORMAT FOR FLOAT64MultiArray
        rospy.Subscriber('currPos', Float64MultiArray, RoutePlannerNode.printGraph)
        rospy.loginfo('[%s] Node started!', self.node_name)



if __name__ == "__main__":
    # Initialize node with rospy
    rospy.init_node('route_planner', anonymous=True)
    # Create the node object
    node1 = RoutePlannerNode()
    #curr_location = (40.6119486, -75.3785700) #Where the car is
    curr_location = (40.6079000,-75.3810000)
    msg = Float64MultiArray()
    rate = rospy.Rate(0.2) # 0.2hz
    msg.data.append(curr_location[0])
    msg.data.append(curr_location[1])
    while not rospy.is_shutdown():
        curr_location = (40.6079000,-75.3810000)
        msg.data[0] = curr_location[0]
        msg.data[1] = curr_location[1]
        rospy.loginfo('current location x: %f, y: %f', msg.data[0], msg.data[1]) 
        node1.example_pub.publish(msg)
        rate.sleep()
        #fig.close()
        curr_location = (40.6083000, -75.3743000)
        msg.data[0] = curr_location[0]
        msg.data[1] = curr_location[1]
        rospy.loginfo('current location x: %f, y: %f', msg.data[0], msg.data[1]) 
        node1.example_pub.publish(msg)
        rate.sleep()
    # Keep the node alive
    #rospy.spin()
