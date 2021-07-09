#! /usr/bin/env python
# note: copied code from the pure pursuit scripts  and modifying it for my needs
#most of the code is copied from tf_updater_node.py but also stuff from the other 2 files
#this is what runs if the arbiter decides the car needs to stop 


import rospy
import math
import time

from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDrive

from purepursuit import PurePursuit

import geometry_msgs.msg

class StopBehaviorNode(object):
    '''
    The node simulates the vehicle motion based on the bycicle model, and
    Ackermann speed command messages.
    Attributes
    ----------
    current_command : AckermannDrive.ackermann_msgs.msg
        the current Ackermann,
    Methods
    -------
    getMessage(msg):
        gets the tf messages and stores them for the node
    updateVehicle(event=None):
        uses the message information to update the vehicle location
    stopthecar
        update how far the vehicle is from the stop point 
        this may be helpful in determining how fast the car should come to a stop

    '''
    
    def __init__(self):
        '''
        Initializes by setting suscriber that listens to the 
        original speed comands and a publisher that publisher the modified speed commnds that make the car stop
        
        '''
        #Initial Stuff
        self.rate = rospy.get_param('~rate', 1)
        self.period = rospy.Duration(1.0 / self.rate)
        self.node_name = rospy.get_name()
        self.parent_frame = rospy.get_param('~parent_frame', 'world')
        self.child_frame = rospy.get_param('~child_frame', 'vehicle')

        #Create Publisher objects 
        # self.stop_command_pub = rospy.Publisher('stop_speed_command',   #this will publish the commands telling the car to slow to a stop
        #                                    AckermannDrive,        #idk what this message type is like
        #                                    queue_size=1)

        #Create Subscriber 
        self.command_sub = rospy.Subscriber('speed_command',   #suscribes to the 'spped_command' topic
                            AckermannDrive,         #message type
                             self.callback)         #every time something is recieved, it runs this

        #one-shot timer to stop the car 
        timer = rospy.Timer(rospy.Duration(10), self.stop_the_car, oneshot=True)


    def callback(self, data):
        rospy.loginfo('speed is %s m/s', data.speed)  #logs message recieved (speed) in terminal
        self.ackerman_data = data.speed


    def stop_the_car(self, event=None): 

        '''
         Method that takes the speed commands and modifies them to stop the car
        ----------
        None
        Returns
        -------
        None   
        '''
        self.msg = AckermannDrive(1.0, 0, 0, 0, 0)

        # msg.speed = self.purepursuit.speed
        # msg.steering_angle = self.purepursuit.compute_steering_angle()

        # self.stop_command_pub.publish(msg)

        rospy.loginfo('BEGIN TO SLOW DOWN!')



if __name__ == "__main__":
    # initialize node with rospy
    rospy.init_node('stop_behavior', anonymous=False)
    # create the node object
    StopBehaviorNode()
    # keep the node alive
    rospy.spin()