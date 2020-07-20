#! /usr/bin/env python

'''
This is the Route Planner Node. It takes in the location of the vehicle and calculates theroute and the reference path needed to reach a destination.
'''

import rospy
import tf2_ros

from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from route_planner import RoutePlanner


class RoutePlannerNode(object):
    '''
    The Route planner Node class contains various methods.
    The __init__ method acts as a constructor for the RoutePlannerNode Object.
    The get_vehicle_location method returns the current location of the vehicle.
    The coordinates_to_poses method converts coordinates to pose objects.
    The control_loop updates the route based on the new location.
    '''

    def __init__(self):
        '''
        Initializes the RoutePlannerNode object.
        Takes in various parameters and assigns them to the object.
        Creates and plots a blank map of the roads to traverse.
        Marks the destination goal point.
        Creates a message for both the path and route.
        Publishes the message.
        Listens for changes in location that would update the data.
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

        self.period = rospy.Duration(1.0 / self.rate)

        self.route_planner = RoutePlanner(self.address,
                                          distance=self.network_range,
                                          network_type=self.network_type)
        # Plot graph
        self.route_planner.setup_plot()

        # Sets the destination point
        self.dest = (40.6054017, -75.3758301)  # (y,x) #TODO: get from user

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
        self.reference_path_pub = rospy.Publisher('reference_path', Path,
                                                  queue_size=10)

        # Create transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.ts_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Crate timers
        self.timer = rospy.Timer(self.period, self.control_loop)

        rospy.loginfo('[%s] Node started!', self.node_name)

    def get_vehicle_location(self):
        '''
        Uses a tf buffer to get the location of the vehicle and return its coordinates
        '''
        try:
            trans = self.tf_buffer.lookup_transform(self.child_frame,
                                                    self.parent_frame,
                                                    rospy.Time.now(),
                                                    self.period)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        return trans.transform.translation.x, trans.transform.translation.y

    def coordinates_to_poses(self, coords):
        '''
        Iterates through the coordinates to create a pose for each.
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
        Updates the route based on changing location.
        Then publishes both the route and its reference path.
        '''
        # Get the current location
        orig = self.get_vehicle_location()
        if orig is None:
            rospy.logdebug('Vehicle position not available!')
            return

        route = self.route_planner.get_route(orig, self.dest)
        route_coords = self.route_planner.get_route_coords(route)
        road_coords = self.route_planner.get_road_coords(route)
        self.route_planner.plot_route(road_coords)

        # Publish route
        self.route_msg.header.stamp = rospy.Time.now()  # Set the stamp
        self.route_msg.poses = self.coordinates_to_poses(route_coords)
        self.route_pub.publish(self.route_msg)
        rospy.logdebug('Route message: %s', self.route_msg)

        # Publish reference path associated with roads
        self.path_msg.header.stamp = rospy.Time.now()  # Set the stamp
        self.path_msg.poses = self.coordinates_to_poses(road_coords)
        self.reference_path_pub.publish(self.path_msg)
        rospy.logdebug('Reference path: %s', self.path_msg)


if __name__ == '__main__':
    # Initialize node with rospy
    rospy.init_node('route_planner', anonymous=True)
    # Create the node object
    route_planner_node = RoutePlannerNode()
    # Keep the node alive
    # rospy.spin()
    # Hack to update plot from the main thread due to TkInter issue
    while not rospy.is_shutdown():
        route_planner_node.route_planner.update_plot()
        rospy.sleep(route_planner_node.period)
