#! /usr/bin/env python

import rospy
import tf2_ros
import tf.transformations as tr

from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDrive

from shapely.geometry import LineString

from purepursuit import PurePursuit


class PurePursuitNode(object):
    '''TODO: docstring
    '''

    def __init__(self):
        '''TODO: docstring
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
        # Create subscribers
        self.example_sub = rospy.Subscriber('planned_path', Path, self.set_path)
        # Create transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.ts_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Create timers
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate),
                                 self.control_loop)

        rospy.loginfo('[%s] Node started!', self.node_name)

    def get_vehicle_pose(self):
        '''TODO: docstring
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
        '''TODO: docstring
        '''
        vehicle_pose = self.get_vehicle_pose()
        self.purepursuit.set_vehicle_pose(vehicle_pose)
        pose_list = [(pose.pose.position.x, pose.pose.position.y)
                     for pose in msg.poses]
        self.purepursuit.path = LineString(pose_list)

    def control_loop(self, event=None):
        '''TODO: docstring
        '''
        msg = AckermannDrive()
        if self.purepursuit.path is not None:
            msg.speed = self.purepursuit.speed
            msg.steering_angle = self.purepursuit.compute_steering_angle()
        self.command_pub.publish(msg)


if __name__ == "__main__":
    # Initialize node with rospy
    rospy.init_node('purepursuit', anonymous=False)
    # Create the node object
    _ = PurePursuitNode()
    # Keep the node alive
    rospy.spin()
