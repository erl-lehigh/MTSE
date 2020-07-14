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
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

#RoutePlanner
class RoutePlannerNode(object):
    '''TODO:
    '''

    '''
    [258574407, 258574525, 259255502]
    [INFO] [1594667836.981949]: current location x: 40.608300, y: -75.374300
    [258574401, 259349128, 258574406, 258574407, 258574525, 259255502]

    print(g.get_edge_data(route[0],route[1])) #THIS LINE GIVES:
    {0: {'length': 135.28, 'osmid': 31977833, 'oneway': False, 'geometry': <shapely.geometry.linestring.LineString object at 0x7f1c7d853e90>, 'highway': u'residential', 'name': u'Taylor Street'}}
    [INFO] [1594677784.641976]: current location x: -75.381000, y: 40.607900
    {0: {'length': 127.473, 'osmid': 30956376, 'oneway': False, 'geometry': <shapely.geometry.linestring.LineString object at 0x7f1c7c238490>, 'highway': u'residential', 'name': u'West Packer Avenue'}}

    '''
    def printGraph(self, msg):
        #The center point of the map
        center_point = (40.6038914, -75.3739361) #(y,x)
        #Gets all the roads a distance away on which can be driven
        g = ox.graph_from_point(center_point, distance=1500, network_type='drive')
        #Gets the current location from the message sent via pub/sub
        orig = (msg.data[0],msg.data[1]) 
        #Sets the destination point
        dest = (40.6054017, -75.3758301) #(y,x)
        #Finds the nearest intersection to the current location
        origin_node = ox.get_nearest_node(g, orig)
        #Finds the nearest intersection to the destination point
        destination_node = ox.get_nearest_node(g, dest)
        #Calculates the shortest path from the current location to the destination
        route = nx.shortest_path(g, origin_node, destination_node)
        #print(g.node[route[0]]) #Turns node IDs into Coordinates
        #Graphs the figure
        fig, ax = ox.plot.plot_graph_route(g, route, route_linewidth=6, node_size=0, bgcolor='k')

        #Broadcasts the intersections (nodes) remaining on the route
        msg2 = Path() #Instantiate the msg2 object as a Path object
        msg2.header.frame_id = "/map" #Set the frame_id
        msg2.header.stamp =  rospy.Time.now() #Set the stamp
        for node in route: #For each node in route
            pose = PoseStamped() #Instantiate the pose object as a PoseStamped object
            pose.pose.position.x = g.node[node]["x"] #Set the message's x value to the node's x value
            pose.pose.position.y = g.node[node]["y"] #Set the message's y value to the node's y value
            msg2.poses.append(pose) #Add the coordinate to the list of coordinates in msg2
        self.route_pub.publish(msg2) #Broadcast the message
        print(msg2) #Print for debugging

        #Broadcasts the roads (edges) remaining on the route (Geometrical Path)
        msg3 = Path() #Instantiate the msg2 object as a Path object
        msg3.header.frame_id = "/map" #Set the frame_id
        msg3.header.stamp = rospy.Time.now() #Set the stamp
        for node in range(len(route)-1):
            msg3.poses.append(g.get_edge_data(route[node], route[node+1])[0]["geometry"].coords) #Add each edge to msg3
        self.refPath_pub.publish(msg3) #Broadcast the message
        print(msg3) #Print for debugging




    def __init__(self):
        #Sets the node's name
        self.node_name = rospy.get_name()
        #Gets any parameters on the node
        self.example = rospy.get_param('~parameter', 0)

        # Creates publishers:

        #Publishes Current Postion
        self.currPos_pub = rospy.Publisher('currPos', Float64MultiArray , queue_size=10)
        #Publish the Nodes along a path (that can be turned into coordinates with g.node[])
        self.route_pub = rospy.Publisher('route', Path , queue_size=10)
        #Publish the reference path
        self.refPath_pub = rospy.Publisher('referencePath', Path , queue_size=10)

        # Create subscribers
        rospy.Subscriber('currPos', Float64MultiArray, self.printGraph)
        rospy.loginfo('[%s] Node started!', self.node_name)



if __name__ == "__main__":
    # Initialize node with rospy
    rospy.init_node('route_planner', anonymous=True)
    # Create the node object
    node1 = RoutePlannerNode()
    #Initializes the msg as a Float64MultiArray
    msg = Float64MultiArray()
    #Sets how often the messages are sent
    rate = rospy.Rate(0.1) # 0.1hz
    #Initializes the msg to a generic location
    msg.data = [0.0, 0.0]
    
    #Control Loop
    while not rospy.is_shutdown():
        plt.ion()
        #Sets the First Point
        curr_location = (40.6079000,-75.3810000) #(y,x)
        msg.data = curr_location #Updates msg with new location
        rospy.loginfo('current location x: %f, y: %f', msg.data[1], msg.data[0]) #Outputs Debugging information
        node1.currPos_pub.publish(msg) #Publishes the msg
        rate.sleep()  #Waits the sleep time


        #Sets the Second Point
        curr_location = (40.6083000, -75.3743000) #(y,x)
        msg.data = curr_location #Updates msg with new location
        rospy.loginfo('current location x: %f, y: %f', msg.data[1], msg.data[0]) #Outputs Debugging information
        node1.currPos_pub.publish(msg) #Publishes the msg
        rate.sleep() #Waits the sleep time

