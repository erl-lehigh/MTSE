#! /usr/bin/env python
import math

import rospy
import tf2_ros
import tf.transformations as tr
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Header
from shapely.geometry import LineString

from purepursuit import PurePursuit


class StraightForwardNode(object):
    '''
    A class to send an ackermann command to just drive straight

    Attributes
    ----------
    node_name : str
        name of the node
    timer : rospy.Timer
        the control loop timer

    Methods
    -------
    publish_forward
        published a forward message
    '''

    def __init__(self):
        '''
        Initializes the purepursuit node that subscribes to a planned path
        topic, computes steering angle commands based on the received paths, and
        and publishes Ackermann commands. The goal point computed by the
        purepursuit method is also publised.
        '''
        self.node_name = rospy.get_name()

        # Create publishers
        self.command_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_max/input/navigation',
                                           AckermannDriveStamped,
                                           queue_size=1)

        # Create timers
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate),
                                 self.publish_forward)

        rospy.logdebug('[%s] Node started!', self.node_name)

    def publish_forward(self, event=None):
        msg = AckermannDrive(0, 0, 1, 0, 0)
        stmp = rospy.Time.now()
        frame = ''
        header = Header()
        header.stamp = stmp
        header.frame = frame
        drive_stamped = AckermannDriveStamped()
        drive_stamped.header = header
        drive_stamped.drive = msg
        self.command_pud.publish(drive_stamped)

if __name__ == "__main__":
    # initialize node with rospy
    rospy.init_node('straight_forward', anonymous=False)
    # add ', log_level=rospy.DEBUG' into the above if
    # if you would like the log information
    # create the node object
    _ = StraightForwardNode()
    # keep the node alive
    rospy.spin()