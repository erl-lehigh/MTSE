#! /usr/bin/env python

import rospy
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDrive

from shapely.geometry import Point, LineString

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
        lookahead = rospy.get_param('~lookahead', 4)
        wheelbase = rospy.get_param('~wheelbase', 1)

        self.purepursuit = PurePursuit(wheelbase, lookahead)
        # Initialize the speed of the vehicle
        self.purepursuit.speed = 3

        # Create publishers
        self.command_pub = rospy.Publisher('speed_command', AckermannDrive,
                                           queue_size=1)
        # Create subscribers
        self.example_sub = rospy.Subscriber('planned_path', Path, self.set_path)
        # Create timers
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate),
                                 self.control_loop)

        rospy.loginfo('[%s] Node started!', self.node_name)

    def set_path(self, msg):
        '''TODO: docstring
        '''
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
