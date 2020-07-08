#! /usr/bin/env python

import rospy
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
#import Rtree==0.8.3
#import osmnx==0.9 as ox
import networkx as nx

#from route_planner import RoutePlanner

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
        '''self.example_pub  = rospy.Publisher('publisher_example', Path,
                                            queue_size=1)'''
        curr_location = (40.6119486, -75.3785700)
        self.example_pub = rospy.Publisher('', float64, queue_size=10)
        rospy.init_node('location', anonymous=True)
        rate = rospy.Rate(0.25) # 0.25hz = 4s
        msg = curr_location
        while not rospy.is_shutdown():
            rospy.loginfo('current location x: %f, y: %f', msg[0], msg[1])
            pub.publish(msg)
            rate.sleep()

        # Create subscribers
        '''self.example_sub = rospy.Subscriber('~subscriber_example', Path,
                                            self.example_callback)'''
        rospy.init_node('location', anonymous=True)
        rospy.Subscriber('', float64, example_callback(self, msg))
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        rospy.loginfo('[%s] Node started!', self.node_name)

    def example_callback(self, msg):
        '''Example message callback.'''
        #print('Message received.')
        center_point = (40.6038914, -75.3739361)
        g = ox.graph_from_point(center_point, distance=1500, network_type='drive')
        orig = msg  
        dest = (40.6054017, -75.3758301)
        origin_node = ox.get_nearest_node(g, orig)
        destination_node = ox.get_nearest_node(g, dest)
        route = nx.shortest_path(g, origin_node, destination_node)
        fig, ax = ox.plot.plot_graph_route(g, route, route_linewidth=6, node_size=0, bgcolor='k')
        ox.plot_graph(g)



if __name__ == "__main__":
    # Initialize node with rospy
    rospy.init_node('route_planner', anonymous=False)
    # Create the node object
    _ = RoutePlannerNode()
    # Keep the node alive
    rospy.spin()
