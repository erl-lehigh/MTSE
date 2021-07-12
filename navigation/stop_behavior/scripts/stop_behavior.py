#! /usr/bin/env python

import rospy
import math
import time

from nav_msgs.msg import Path                          
from ackermann_msgs.msg import AckermannDrive                   

from purepursuit import PurePursuit
from std_msgs.msg import Float32

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

        self.ackerman_speed = 0     #speed of car
        self.ackerman_steerangle = 0 #steering angle                                                
        self.distance = 0      #gets the distance to the stop sign when it is detected              
        self.time_stamp = 0    #variable to mark the time that a stop sign is detected
        self.new_sign = False     #becomes true when a stop sign is detected                        


        #Create Publisher objects 
        self.stop_command_pub = rospy.Publisher('stop_speed_command',   #this will publish the commands telling the car to slow to a stop
                                                AckermannDrive,        #message type
                                                queue_size=1)

        #Create Subscriber objects 
        self.command_sub = rospy.Subscriber('speed_command',   #suscribes to the 'spped_command' topic
                            AckermannDrive,         #message type
                             self.callback)         #every time something is recieved from this topic, it runs this

        self.stop_sign_sub = rospy.Subscriber('stop_sign',   #suscribes to 'stop sign' topic that tells you the distance to the stop sign
                            Float32,         #message type
                             self.sign_detector)         #every time something is recieved, it runs this

        #one-shot timer to stop the car 
        timer = rospy.Timer(rospy.Duration(10), self.stop_the_car, oneshot=True)


    def callback(self, data):
        rospy.loginfo('speed is %s m/s', data.speed)  #logs message recieved (speed) in terminal
        self.ackerman_speed = data.speed
        self.ackerman_steering = data.steering_angle



    def sign_detector(self, data):
        self.time_stamp = time.time()        #gets the initial time that the car detects the stop sign
        self.distance = data.data
        rospy.loginfo('Distance to sign is %s m', self.distance)  #logs message recieved (speed) in terminal
        self.new_sign = True   #this variable is true
        self.ackerman_steering = data.steering_angle



    def stop_the_car(self, event=None):        #need to figure out a way to call this method to stop the car

        '''
         Method that takes the speed commands and modifies them to stop the car
        ----------
        None
        Returns
        -------
        None   
        '''
        self.msg = AckermannDrive()  #Initialize message type for AckermanDrive

        self.msg.steering_angle = self.ackerman_steerangle
        self.msg.steering_angle_velocity = 0
        self.msg.speed = self.ackerman_speed
        self.msg.acceleration = 0
        self.msg.jerk = 0


        self.stop_command_pub.publish(self.msg)

        rospy.loginfo('BEGIN TO SLOW DOWN! %s', self.msg)

        #put math for the slowing down



if __name__ == "__main__":
    # initialize node with rospy
    rospy.init_node('stop_behavior', anonymous=False)
    # create the node object
    StopBehaviorNode()
    # keep the node alive
    rospy.spin()

