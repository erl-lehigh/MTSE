#! /usr/bin/env python

'''
Route Planner Node that communicates with ROS
'''

import rospy
import tf2_ros   # difference between tf2_ros and tf2?

from std_msgs.msg import Header       #the header message type contains metadata and timestamped stuff
from nav_msgs.msg import Path            #path type - an array of poses that represents a Path for a robot to follow
from geometry_msgs.msg import PoseStamped  #another message type

from route_planner import RoutePlanner  #from the route planner package in the navigation folder, import the class RoutePlanner


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
            Returns the location of the vehicle based on the transform between
            the parent and child frames using a tf2 listener.
        coordinates_to_poses(self, coords):
            Iterates through the coordinates to create a pose for each.
        control_loop(self, event):
            Updates the route based on changing location. Then publishes both
            the route and its reference path.
    '''

    def __init__(self):        #this method runs when the program is called after the '__main__' thing.  It looks like it does a lot of different tasks
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
        self.node_name = rospy.get_name()        #I think the node name becomes the name of the node initialized in the '__main__'
        # Gets any parameters on the node
        self.rate = rospy.get_param('~rate', 1)            # gets parameters from route_planner.yaml file in config folder
        self.parent_frame = rospy.get_param('~parent_frame', 'world')        #if no parameter for 'parent_frame', deault it's value to 'world'
        self.child_frame = rospy.get_param('~child_frame', 'vehicle')
        self.address = rospy.get_param('~address')
        self.network_range = rospy.get_param('~network_range', 1500)
        self.network_type = rospy.get_param('~network_type', 'drive')

        self.period = rospy.Duration(1.0 / self.rate)    

        self.route_planner = RoutePlanner(self.address,           # this is creating object for the RoutePlanner class.  It contains some of the parameters from above in the argument
                                          distance=self.network_range,
                                          network_type=self.network_type)
        # Plot graph
        self.route_planner.setup_plot()  #this is a method in RoutePlanner that Displays the static road map

        # Gets the destination from the user
        destination = input("Address of Destination (in quotes) : ")

        # Converts the address given to latitude and longitude
        self.dest = self.route_planner.geocode(query=destination)        #goes to geocode method in RoutePlanner class

        # Common header for all
        self.header = Header(frame_id=self.parent_frame)                #idemtifies things going on each instance

        self.route_msg = Path()  # Path message for route publishing                   #empty path array
        self.route_msg.header.frame_id = self.parent_frame  # Set the frame_id

        self.path_msg = Path()  # Path message for reference path publishing      #how is this different from the Path() called 3 lines earlier?
        self.path_msg.header.frame_id = self.parent_frame  # Set the frame_id        #??

        # Creates publishers
        # Publisher for route
        self.route_pub = rospy.Publisher('route', Path, queue_size=10)      #route is name of topic being published to, Path is the message type
        # Publisher for reference path
        self.reference_path_pub = rospy.Publisher('reference_path', Path,   #publishes to 'reference_path' topic with Path as message type
                                                  queue_size=10)

        # Create transform listener
        self.tf_buffer = tf2_ros.Buffer()            # stores a small time frame of incoming transforms
        self.ts_listener = tf2_ros.TransformListener(self.tf_buffer)  #Listens to the transforms and adds to the buffer

        # Create timers
        self.timer = rospy.Timer(self.period, self.control_loop)    #arguments are the period and the method control loop?  What is the timer for?

        rospy.loginfo('[%s] Node started!', self.node_name)   #displays that the node has started

    def get_vehicle_location(self):
        '''
        Returns the location of the vehicle based on the transform between the
        parent and child frames using a tf2 listener.

        Parameters
        ----------
        None

        Returns
        -------
        (x,y) point
            location of the vehicle
        '''
        try:
            trans = self.tf_buffer.lookup_transform(self.child_frame,         #why is it now using tf_ instead of tf2_?
                                                    self.parent_frame,
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
        orig = self.get_vehicle_location()      #the get_vehicle_location() is a method in this code
        if orig is None:
            rospy.logdebug('Vehicle position not available!')
            return

        route = self.route_planner.get_route(orig, self.dest)          #calls methods forRoutePlanner(self.address, # this is creating object for the RoutePlanner class.  It contains some of the parameters from above in the argument
                                    

        # Publish route
        self.route_msg.header.stamp = rospy.Time.now()  # Set the stamp
        self.route_msg.poses = self.coordinates_to_poses(route_coords)      #having trouble understanding the route_msg thing (related to Path)
        self.route_pub.publish(self.route_msg)                            #the route_pub publishes route_msg!!!
        rospy.logdebug('Route message: %s', self.route_msg)                 

        # Publish reference path associated with roads
        self.path_msg.header.stamp = rospy.Time.now()  # Set the stamp
        self.path_msg.poses = self.coordinates_to_poses(road_coords)
        self.reference_path_pub.publish(self.path_msg)                       #the other publisher publishes the same or different thing?
        rospy.logdebug('Reference path: %s', self.path_msg)


if __name__ == '__main__':     #this is the first section of code that runs when the script is run
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
