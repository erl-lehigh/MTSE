#! /usr/bin/env python

import rospy
import tf2_ros
import tf.transformations as tr

from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseStamped

from shapely.geometry import LineString

from purepursuit import PurePursuit


class PurePursuitNode(object):
    '''
    A class to represent a pure pursuit node in ROS

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
    purepursuit : purepursuit.PurePursuit
        path tracker object using the pure pursuit method
    command_pub : rospy.Publisher
        the speed command publisher
    target_pub : rospy.Publisher
        the pure pursuit (moving) target publisher
    path_sub : rospy.Subscriber
        the subscriber for the tracked path
    tf_buffer : tf2_ros.Buffer
        the transform listener buffer
    tf_listener : tf2_ros.TransformListener
        the listener for ROS transforms
    timer : rospy.Timer
        the control loop timer

    Methods
    -------
    set_path(msg):
        generates a path LineString (to be tracked) from a set of position
        coordinates (pose)
    control_loop(event=None):
        publishes AckermannDrive msg consisting of the computed vehicle speed
        and steering angle
    get_vehicle_pose():
        returns the vehicle coordinates (position) for purepursuit computation

    '''

    def __init__(self):
        '''
        Initializes the purepursuit_node object by passing input parameters
        which subscribes to planned_path topic to recive a path and then
        geometrically compute the goal point, vehicle speed and steering angle,
        and then publishes to the ~/target using Pose msg and speed_command
        topic using AckermannDrive msg so that the vehcile can track the path.

        '''
        self.node_name = rospy.get_name()

        # Read parameters
        self.rate = rospy.get_param('~rate', 1)
        self.parent_frame = rospy.get_param('~parent_frame', 'world')
        self.child_frame = rospy.get_param('~child_frame', 'vehicle')
        lookahead = rospy.get_param('~lookahead', 4)
        wheelbase = rospy.get_param('~wheelbase', 1)
        speed = rospy.get_param('~speed', 3)

        self.period = rospy.Duration(1.0 / self.rate)

        self.purepursuit = PurePursuit(wheelbase, lookahead, speed)

        # Create publishers
        self.command_pub = rospy.Publisher('speed_command', AckermannDrive,
                                           queue_size=1)

        self.target_pub = rospy.Publisher('~/target', PoseStamped, queue_size=1)

        # Create subscribers
        self.path_sub = rospy.Subscriber('planned_path', Path, self.set_path)

        # Create transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.ts_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Create timers
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate),
                                 self.control_loop)

        rospy.loginfo('[%s] Node started!', self.node_name)

    def get_vehicle_pose(self):
        '''
        Returns the vehicle coordinates (position) for purepursuit computation.

        Parameters
        ----------
        None

        Returns
        -------
        tuple
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
        return (trans.transform.translation.x, trans.transform.translation.y,
                orientation)

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

    def control_loop(self, event=None):
        '''
        Publishes AckermannDrive msg consisting of the computed vehicle speed
        and steering angle if a path is passed.

        Parameters
        ----------
        event=None : rospy.TimerEvent
            information about the event that generated this call

        Returns
        -------
        None
        '''
        msg = AckermannDrive()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.parent_frame
        if self.purepursuit.path is not None:
            position = self.purepursuit.future_point()
            pose_msg.pose.position.x = position.x
            pose_msg.pose.position.y = position.y
            msg.speed = self.purepursuit.speed
            msg.steering_angle = self.purepursuit.compute_steering_angle()
        self.command_pub.publish(msg)
        self.target_pub.publish(pose_msg)


if __name__ == "__main__":
    # Initialize node with rospy
    rospy.init_node('purepursuit', anonymous=False)
    # Create the node object
    _ = PurePursuitNode()
    # Keep the node alive
    rospy.spin()
