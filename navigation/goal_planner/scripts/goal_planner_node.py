#! /usr/bin/env python

'''
Goal Planner Node that communicates with ROS
'''

import rospy
import tf2_ros
import tf.transformations as tr

from std_msgs.msg import Header
from nav_msgs.msg import Path
from shapely.geometry import LineString
from geometry_msgs.msg import PoseStamped

from goal_planner import GoalPlanner


class GoalPlannerNode(object):
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
        goal_planner : GoalPlanner()
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
        GoalPlannerNode Constructor
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

        self.path = LineString()


        # Create transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.ts_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.period = rospy.Duration(1.0 / self.rate)

        # Common header for all
        self.header = Header(frame_id=self.parent_frame)

        # Path message for route publishing
        self.goal_point_msg = PoseStamped()
        # Set the frame_id  
        self.goal_point_msg.header.frame_id = self.parent_frame  

        #Creates a subscriber
        rospy.Subscriber("reference_path", Path, self.set_path)

        # Creates publisher
        # Publisher for goal point
        self.goal_point_pub = rospy.Publisher('goal_point', PoseStamped,
            queue_size=10)


        self.goal_planner = GoalPlanner(self.get_vehicle_location(), 
            self.get_vehicle_orientation(), self.path)

        # Create transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.ts_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Crate timers
        self.timer = rospy.Timer(self.period, self.control_loop)

        rospy.loginfo('[%s] Node started!', self.node_name)

    def set_path(self, msg):
        '''
        Generates a path LineString (to be tracked) from a set of position
        coordinates (pose).
        Parameters
        ----------
        msg :  nav_msgs.msg.Path
            ROS navigation path message
        Returns
        -------
        None
        '''

        vehicle_pose = self.get_vehicle_pose()
        self.purepursuit.set_vehicle_pose(vehicle_pose)
        pose_list = [(pose.pose.position.x, pose.pose.position.y)
                     for pose in msg.poses]
        self.purepursuit.path = LineString(pose_list)



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
        return trans.transform.translation.x, trans.transform.translation.y

    def get_vehicle_orientation(self):
        '''
        Returns the vehicle's orientation.
        Parameters
        ----------
        None
        Returns
        -------
        Tuple # Is this correct???
            vehicle coordinates (x,y) and orientation (angle)
        '''
        try:
            trans = self.tf_buffer.lookup_transform(self.child_frame,
                                                    self.parent_frame,
                                                    rospy.Time.now(),
                                                    self.period)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        quaternion = trans.transform.rotation
        quaternion = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        _, _, orientation = tr.euler_from_quaternion(quaternion)
        return orientation

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

        #route = self.goal_planner.get_route(orig, self.dest)
        goal_node = self.goal_planner.get_goal_node()

        # Publish route
        self.goal_point_msg.header.stamp = rospy.Time.now()  # Set the stamp
        self.goal_point_msg.poses = self.coordinates_to_poses(goal_node)
        self.goal_point_pub.publish(self.goal_point_msg)
        rospy.logdebug('Goal Point: %s', self.goal_point_msg)




if __name__ == '__main__':
    # Initialize node with rospy
    rospy.init_node('goal_planner', anonymous=True)
    # Create the node object
    goal_planner_node = GoalPlannerNode()
    # Keep the node alive
    # rospy.spin()
    # Hack to update plot from the main thread due to TkInter issue
    while not rospy.is_shutdown():
        goal_planner_node.goal_planner.update_plot()
        rospy.sleep(goal_planner_node.period)