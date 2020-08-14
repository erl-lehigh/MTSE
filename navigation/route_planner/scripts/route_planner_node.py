#! /usr/bin/env python

'''
Route Planner Node that communicates with ROS
'''

import rospy
import tf2_ros

from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from route_planner import RoutePlanner


class RoutePlannerNode(object):
    '''
    Attributes
    ----------
        node_name : string
            the name of the node
        rate : integer
            how many times per second the node updates
        parent_frame : string
            the parent reference frame (often world)
        child_frame : string
            the child reference frame (often the vehicle)
        address : string
            the central address of the map
        network_range : integer
            radius away from the center of the map to generate
        network_type : string
            what type of street network to get 
        period : float
            the inverse of the rate
        route_planner : RoutePlanner()
            the object that does all of the route planning
            
    Methods
    -------
        get_vehicle_location(self):
            Uses a tf buffer to get the location of the vehicle and return its coordinates
        coordinates_to_poses(self, coords):
            Iterates through the coordinates to create a pose for each.
        control_loop(self, event):
            Updates the route based on changing location.
            Then publishes both the route and its reference path.


    '''

    def __init__(self):
        '''
        RoutePlannerNode Constructor

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

        self.period = rospy.Duration(1.0 / self.rate)

        #self.route_planner = RoutePlanner(self.address,
        #                                  distance=self.network_range,
        #                                  network_type=self.network_type)
        self.route_planner = RoutePlanner(
            '/home/nathan/mtse_catkin/src/navigation/route_planner/scripts/carla_map.yaml')

        # Plot graph
        self.route_planner.setup_plot()

        # Gets the destination from the user
        #destination = input("Address of Destination (in quotes) : ")
        destination = (220.091 , -9.808)
        # Converts the address given to latitude and longitude
        #self.dest = self.route_planner.geocode(query=destination)
        self.dest = destination
        
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

        # Create transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.ts_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Crate timers
        self.timer = rospy.Timer(self.period, self.control_loop)


        # Subscribers
        rospy.Subscriber("/target",PoseStamped, 
            self.plot_target)

        rospy.loginfo('[%s] Node started!', self.node_name)

    def set_graph(self, themap):
        self.graph = themap

    def plot_target(self, target_point):
        self.route_planner.plot_route([(target_point.pose.position.x, target_point.pose.position.y), (target_point.pose.position.x, target_point.pose.position.y)])

    def get_vehicle_location(self):
        '''
        Uses a tf buffer to get the location of the vehicle and return its coordinates

        Parameters
        ----------
        None

        Returns
        -------
        (x,y) point
            location of the vehicle
        '''
        try:
            trans = self.tf_buffer.lookup_transform(self.child_frame,
                                                    self.parent_frame,
                                                    rospy.Time.now(),
                                                    self.period)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        return -trans.transform.translation.x, -trans.transform.translation.y

    def coordinates_to_poses(self, coords):
        '''
        Iterates through the coordinates to create a pose for each.

        Parameters
        ----------
        coords : list of coordinates
            the coordinates to be converted into poses

        Returns
        -------
        list of poses
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
        Updates the route based on changing location.
        Then publishes both the route and its reference path.

        Parameters
        ----------
        event : event
            the current event state

        Returns
        -------
        None
        '''
        # Get the current location
        orig = self.get_vehicle_location()
        if orig is None:
            rospy.logdebug('Vehicle position not available!')
            return

        # Shows a straight line between the current location and the destination
        self.route_planner.plot_route([orig, self.dest])

        route = self.route_planner.get_route(orig, self.dest)
        route_coords = self.route_planner.get_route_coords(route)
        road_coords = self.route_planner.get_road_coords(route)
        self.route_planner.plot_route(road_coords)
        # rospy.logwarn('[route planner] coords route:', road_coords)

        # Publish route
        self.route_msg.header.stamp = rospy.Time.now()  # Set the stamp
        self.route_msg.poses = self.coordinates_to_poses(route_coords)
        self.route_pub.publish(self.route_msg)
        # rospy.logdebug('Route message: %s', self.route_msg)

        # Publish reference path associated with roads
        self.path_msg.header.stamp = rospy.Time.now()  # Set the stamp
        self.path_msg.poses = self.coordinates_to_poses(road_coords)
        self.reference_path_pub.publish(self.path_msg)
        # rospy.logdebug('Reference path: %s', self.path_msg)


if __name__ == '__main__':
    # Initialize node with rospy
    rospy.init_node('route_planner', anonymous=True)
    # Create the node object
    route_planner_node = RoutePlannerNode()
    # Keep the node alive
    # rospy.spin()
    # Hack to update plot from the main thread due to TkInter issue
    while not rospy.is_shutdown():
        pass
        route_planner_node.route_planner.update_plot()
        # rospy.sleep(route_planner_node.period)
        # route_planner_node.control_loop(event=True)
