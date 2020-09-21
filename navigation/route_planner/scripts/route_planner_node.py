#! /usr/bin/env python

'''
Route Planner Node that communicates with ROS
'''

import rospy
import tf2_ros
import time
import os

from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from route_planner import RoutePlanner


class RoutePlannerNode(object):
    '''
    Route Planner Node that communicates the route planner through ROS nodes.

    Attributes
    ----------
        node_name : string
            the name of the node.
        rate : integer
            how many times per second the node updates.
        parent_frame : string
            the parent reference frame (often world).
        child_frame : string
            the child reference frame (often the vehicle).
        address : string
            the central address of the map.
        network_range : integer
            radius away from the center of the map to generate.
        network_type : string
            what type of street network to get.
        destination_type : string
            'auto' or 'manual', the method of generating a destination.
        period : float
            the inverse of the rate.
        route_planner : RoutePlanner()
            the object that does all of the route planning.
        dest : tuple
            the location of the destination.
        header : std_msgs.msg.Header
            common header for all pose messages.
        route_msg : nav_msgs.msg.Path
            path message for route publishing.
        path_msg : nav_msgs.msg.Path
            path message for reference path publishing.
        route_pub : rospy.Publisher
            route publisher.
        reference_path_pub : rospy.Publisher
            reference path publisher.
        tf_buffer : tf2_ros.Buffer
            buffer for transforms associated with the listener.
        ts_listener : tf2_ros.TransformListener
            buffered transform listener.
        timer : rospy.Timer
            control loop fixed rate timer.

    Methods
    -------
        plot_target(self, target_point):
            Plots the target point on the map (in green).
        get_vehicle_location(self):
            Returns the location of the vehicle based on the transform between
            the parent and child frames using a tf2 listener.
        coordinates_to_poses(self, coords):
            Transforms list of coordinates to a list of ros geometry poses.
        control_loop(self, event):
            Updates the route based on changing location. Then publishes both
            the route and its reference path.
    '''

    def __init__(self):
        '''
        RoutePlannerNode Constructor.

        Parameters
        ----------
        None

        Returns
        -------
        None
        '''
        # Set the node's name
        self.node_name = rospy.get_name()
        # Gets any parameters on the node
        self.rate = rospy.get_param('~rate', 1)
        self.parent_frame = rospy.get_param('~parent_frame', 'world')
        self.child_frame = rospy.get_param('~child_frame', 'vehicle')
        self.address = rospy.get_param('~address')
        self.network_range = rospy.get_param('~network_range', 1500)
        self.network_type = rospy.get_param('~network_type', 'drive')
        self.destination_type = rospy.get_param('~destination_type', 'auto')

        self.period = rospy.Duration(1.0 / self.rate)

        if(self.destination_type == 'manual'):
            # Gets the destination from the user
            destination = input("Address of Destination (in quotes) : ")
            # Converts the address given to latitude and longitude
            self.dest = self.route_planner.geocode(query=destination)
            self.route_planner = RoutePlanner(self.address,
                                         distance=self.network_range,
                                         network_type=self.network_type)
        else:
            # An automatic destination on the carla map
            self.dest = (93.383 , 132.856)
            # Initializes the RoutePlanner with the map in route_planner/scripts
            self.route_planner = RoutePlanner(
                os.path.abspath(os.path.join(
                    os.path.dirname(os.path.abspath(__file__)),
                        '..', 'scripts', 'carla_map.yaml')))

        # Plot graph
        self.route_planner.setup_plot()

        # Gets the destination from the user
        destination = input("Address of Destination (in quotes) : ")

        # Converts the address given to latitude and longitude
        self.dest = self.route_planner.geocode(query=destination)

        # Common header for all
        self.header = Header(frame_id=self.parent_frame)

        self.route_msg = Path()  # Path message for route publishing
        self.route_msg.header.frame_id = self.parent_frame  # Set the frame_id

        self.path_msg = Path()  # Path message for reference path publishing
        self.path_msg.header.frame_id = self.parent_frame  # Set the frame_id

        # Creates publishers
        # Publisher for route
        self.route_pub = rospy.Publisher('route', Path, queue_size=10)
        # Publisher for reference path
        self.reference_path_pub = rospy.Publisher('planned_path', Path,
                                                  queue_size=10)
        # Publisher for rViz reference path
        self.reference_path_viz_pub = rospy.Publisher('planned_path_viz', Path,
                                                  queue_size=10)
        self.pub_ref = False

        # Gives the tf time to start before plotting
        time.sleep(4)

        # Create transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.ts_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Crate timers
        self.timer = rospy.Timer(self.period, self.control_loop)


        # Subscribers
        rospy.Subscriber("/target",PoseStamped, self.plot_target)

        rospy.loginfo('[%s] Node started!', self.node_name)

    def plot_target(self, target_point):
        '''
        Plots the target point on the map (in green).

        Parameters
        ----------
        target_point : tuple
            the point that will be plotted.

        Returns
        -------
        None
        '''

        # Uses plot_route to plot a route containing 1 point
        self.route_planner.plot_route(
            [(target_point.pose.position.x, target_point.pose.position.y),
            (target_point.pose.position.x, target_point.pose.position.y)],
            color='green')

    def get_vehicle_location(self):
        '''
        Returns the location of the vehicle based on the transform between the
        parent and child frames using a tf2 listener.

        Parameters
        ----------
        None

        Returns
        -------
        tuple : (x, y)
            location of the vehicle.
        '''
        try:
            trans = self.tf_buffer.lookup_transform(self.parent_frame,
                                                    self.child_frame,
                                                    rospy.Time.now(),
                                                    self.period)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        return trans.transform.translation.x, trans.transform.translation.y

    def coordinates_to_poses(self, coords):
        '''
        Transforms list of coordinates to a list of ros geometry poses.

        Parameters
        ----------
        coords : list of tuples
            the coordinates to be converted into poses

        Returns
        -------
        list of geometry_msgs.msg.PoseStamped
            the pose equivalent of the inputted coordinates
        '''
        poses = []
        self.header.stamp = rospy.Time.now()
        for x, y in coords:  # Add poses (stamped)
            pose = PoseStamped(header=self.header)
            pose.pose.position.x = x
            pose.pose.position.y = y
            poses.append(pose)
        return poses

    def control_loop(self, event):
        '''
        Updates the route based on changing location. Then publishes both the
        route and its reference path.

        Parameters
        ----------
        event : rospy.timer.TimerEvent
            the current event state

        Returns
        -------
        None
        '''
        # Get the current location
        if not self.pub_ref:
            self.orig = self.get_vehicle_location()
            self.pub_ref = True
        orig = self.get_vehicle_location()

        # Plot the current location as a black diamond
        self.route_planner.plot_route([orig, orig],'black')
        rospy.loginfo('Current Location: (%f, %f)', orig[0], orig[1])

        if orig is None:
            rospy.logdebug('Vehicle position not available!')
            return

        route = self.route_planner.get_route(self.orig, self.dest)
        route_coords = self.route_planner.get_route_coords(route)
        road_coords = self.route_planner.get_road_coords(route)

        # Shows the route on the map
        # self.route_planner.plot_route(road_coords)

        # Publish route
        self.route_msg.header.stamp = rospy.Time.now()  # Set the stamp
        self.route_msg.poses = self.coordinates_to_poses(route_coords)
        self.route_pub.publish(self.route_msg)

        # Publish reference path associated with roads
        self.path_msg.header.stamp = rospy.Time.now()  # Set the stamp
        self.path_msg.poses = self.coordinates_to_poses(road_coords)
        self.reference_path_pub.publish(self.path_msg)
        self.reference_path_viz_pub.publish(self.path_msg)


if __name__ == '__main__':
    # Initialize node with rospy
    rospy.init_node('route_planner', anonymous=True)

    # Create the node object
    route_planner_node = RoutePlannerNode()

    while not rospy.is_shutdown():
        route_planner_node.route_planner.update_plot()
