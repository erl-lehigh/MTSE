#! /usr/bin/env python

import math
import time

import rospy
import tf2_ros
import tf.transformations as tr
from std_msgs.msg import Float64


class VariableSpeedNode(object):
    '''
    This node is meant for calculating a variable speed in order to
    show the relationship between lookahead and speed

    Attributes
    ----------
    reference_speed : rospy.Publisher
        the reference speed, Float64
    startTime : float
        the time the node is started

    Method
    ------
    sinusoidalSpeed():
        changes the speed sinusoidally (for testing)
    '''

    def __init__(self):
        '''
        Initializes by setting publisher and set the start time
        which will be used to calculate the speed
        '''
        #set start time
        self.startTime = time.time()
        self.node_name = rospy.get_name()
        self.rate = rospy.get_param('~rate', 1)
        self.amplitude = amplitude = rospy.get_param('~speed_amplitude', 3)
        self.base_speed = rospy.get_param('~base_speed', 3.35)

        #Create Publishers
        self.reference_speed_pub = rospy.Publisher('reference_speed',
                                        Float64,
                                        queue_size = 1)

        # Create timers
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate),
                                 self.sinusoidalSpeed)
        rospy.logdebug('[%s] Node started!', self.node_name)


    def sinusoidalSpeed(self, event=None):
        '''
        Used the difference in time start and now to change the speed.
        The speed is centered at 3.35 which is the midpoint between
        the lookaway velocity bounds.

        Parameters
        ----------
        None

        Returns
        -------
        None
        '''
        difTime = time.time() - self.startTime #difference in time
        thetaTime = difTime/3 #divide delta by 60 (make more smooth)
        #calculate the speed
        speed = self.base_speed + self.amplitude * math.cos(thetaTime)
        speed_msg = Float64() #initialize the message
        speed_msg.data = speed #set msg data section
        self.reference_speed_pub.publish(speed_msg)    #publish message

if __name__ == "__main__":
    # initialize node with rospy
    rospy.init_node('variable_speed', anonymous=False)
    # create the node object
    _ = VariableSpeedNode()
    # keep the node alive
    rospy.spin()
