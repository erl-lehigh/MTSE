#! /usr/bin/env python

import rospy
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
import Rtree==0.8.3
import osmnx==0.9 as ox
import networkx as nx

from route_planner import RoutePlanner

#RoutePlanner
class RoutePlannerNode(object):
    '''TODO:
    '''

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
        self.example_pub  = rospy.Publisher('publisher_example', Path,
                                            queue_size=1)

        # Create subscribers
        self.example_sub = rospy.Subscriber('~subscriber_example', Path,
                                            self.example_callback)

        rospy.loginfo('[%s] Node started!', self.node_name)

    def example_callback(self, msg):
        '''Example message callback.'''
        #print('Message received.')
        g = ox.graph_from_address('19 Memorial Dr W, Bethlehem, Pennsylvania', distance=1000, network_type='drive')
        orig = (-75.3868, 40.6116)
        dest = (-75.3684, 40.6039)
        origin_node = ox.get_nearest_node(g, orig)
        destination_node = ox.get_nearest_node(g, dest)
        route = nx.shortest_path(g, origin_node, destination_node)
        fig, ax = ox.plot_graph_route(g, route, route_linewidth=6, node_size=0, bgcolor='k')
        ox.plot_graph(g)


if __name__ == "__main__":
    # Initialize node with rospy
    rospy.init_node('route_planner', anonymous=False)
    # Create the node object
    _ = RoutePlannerNode()
    # Keep the node alive
    rospy.spin()
