#! usr/bin/env python
import math

import rospy
import tf2_ros
import tf.transformations as tr
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Float64
from shapely.geometry import LineString

class CheckSurroundingNode(object):
    '''
    A class to make calculation about surrounding obsticles

    Attributes
    ----------
    node_name : str
        name of the node
    parent_frame : str
        parent Reference frame
    child_frame : str
        child Reference frame
    period : rospy.Duration
        specifies the duration, which is a period of time
    rate : float
        specifies the rate of the control loop and speed command publishing
    surrounding_pub : rospy.Publisher
        the publisher to update if there is an obstacle in the surrounding
        that needs to have the car avoid
    obstacle_sub : rospy.Subscriber
        the subsciber to check for the poses of obstacles
    obstacles : List
        the list of the obstacles since the last time the obstacle sub was
        run
    tf_buffer : tf2_ros.Buffer
        the transform listener buffer
    tf_listener : tf2_ros.TransformListener
        the listener for ROS transforms
    timer : rospy.Timer
        the control loop timer

    Methods
    -------
    get_vehicle_pose():
        returns the vehicle coordinates (position) for pure pursuit computation
    get_obstacles():
        gets a list of poses from the obstacles_sub
    update_surrounding():
        published message if there is an obstacle that should make the car stop
    '''

    def __init__(self):
        '''
        Initializes the node that subscribes to another node that published all
        the other vehicle/obstacle poses. 
        '''
        self.node_name = rospy.get_name()

        # Read the parameters from config
        self.rate = rospy.get_param('~rate', 1)
        self.parent_frame = rospy.get_param('~parent_frame', 'world')
        self.child_frame = rospy.get_param('~child_frame', 'vehicle')

        self.period = rospy.Duration(1.0 / self.rate)

        self.surrounding_pub = rospy.Publisher('surrounding',
                                                PoseStamped, #TODO: should switch to be like Dan's message
                                                queue_size=1)

        self.obstacle_sub = rospy.Subscriber('obstacles',
                                             PoseArray,
                                             self.get_obstacles)

        self.obstacles = []

        self.tf_buffer = tf2_ros.Buffer()
        self.ts_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Create timers
        self.timer = rospy.Timer(self.period,
                                 self.update_surrounding)

        rospy.logdebug('[%s] Node started!', self.node_name)

    def get_vehicle_pose(self):
        '''
        Returns the vehicle's pose (position and orientation).

        Parameters
        ----------
        None

        Returns
        -------
        tuple
            vehicle coordinates (x,y), orientation (angle) and quaterion
            representation for orientation
        Note
        ----
            The code in the try section had problems in its state in main.
            It is now correct here and in the fix/purepursuit_integration. This
            correct version of code is also posted in the TF guide on the drive
            and has a littl explanation as to why there might have been an issue.
        '''

        # order of tf lokup is parent, child time period
        try:
            trans = self.tf_buffer.lookup_transform(self.parent_frame,
                                                    self.child_frame,
                                                    rospy.Time(),
                                                    self.period)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as err:
            rospy.logdebug("e: %s", err)
            return
        quaternion_message = trans.transform.rotation
        quaternion = (quaternion_message.x, quaternion_message.y,
                      quaternion_message.z, quaternion_message.w)
        _, _, orientation = tr.euler_from_quaternion(quaternion)
        return (trans.transform.translation.x, trans.transform.translation.y,
                orientation, quaternion_message)

    def get_obstacles(self, msg):
        self.obstacles = msg.poses       # get the poses of the obstacles
        # TODO: might need the header as well

    def update_surrounding(self, event=None):
        ego_pose = self.get_vehicle_pose()
        ego_x = ego_pose[0]
        ego_y = ego_pose[1]
        ego_orientation = ego_pose[2]
        surrounding_msg = PoseStamped()
        for obstacle in self.obstacles:
            obstacle_x = obstacle.position.x
            obstacle_y = obstacle.position.y
            distance = math.sqrt((ego_x - obstacle_x)**2 + (ego_y - obstacle_y)**2)
        self.surrounding_pub.publish(surrounding_msg)




