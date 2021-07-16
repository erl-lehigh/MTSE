#! /usr/bin/env python

import rospy
import math
import time

from nav_msgs.msg import Path                          
from ackermann_msgs.msg import AckermannDrive                   
from std_msgs.msg import Float32
import geometry_msgs.msg


class StopBehaviorNode(object):
    '''
    Description of the code here

    Attributes
    ----------
    current_command : AckermannDrive.ackermann_msgs.msg
        the current Ackermann,
    Methods
    -------
    sign_detector
        a callback type method that gives the time stamp of the car at a new location and its distance from the stop sign
    
    callback
        callback that gets the car's speed and steering angle

    stop_the_car
        update how far the vehicle is from the stop point 
        this may be helpful in determining how fast the car should come to a stop

    '''
    
    def __init__(self):
        '''
        Initializes necessary variables and creates publisher and subscriber objects
        
        '''
        #Initial Stuff
        self.rate = rospy.get_param('~rate', 1)
        self.period = rospy.Duration(1.0 / self.rate)
        self.node_name = rospy.get_name()
        self.parent_frame = rospy.get_param('~parent_frame', 'world')
        self.child_frame = rospy.get_param('~child_frame', 'vehicle')

        self.ackerman_speed = 0.0     #speed of car
        self.ackerman_steerangle = 0.0 #steering angle  
                                                      
        self.distance = 0.0      #gets the distance to the stop sign when it is detected             #maybe make these into their own message type at some point
        self.new_sign = False     #becomes true when a stop sign is detected  

        self.time_stamp = 0.0    #variable to mark the time that a stop sign is detected         
        self.counter = 1         #counter used in the math for slowing car down

        #Create Publisher objects 
        self.stop_command_pub = rospy.Publisher('stop_speed_command',   #this will publish the commands telling the car to slow to a stop
                                                AckermannDrive,        #message type
                                                queue_size=1)

        #Create Subscriber objects 
        self.command_sub = rospy.Subscriber('speed_command',   #subscribes to the 'speed_command' topic
                            AckermannDrive,         #message type
                            self.callback)          #every time something is recieved from this topic, it runs callback

        self.stop_sign_sub = rospy.Subscriber('stop_sign',   #subscribes to 'stop sign' topic that tells you the distance to the stop sign using hokuyo
                            Float32,         #message type
                             self.sign_detector)         #every time something is recieved, runs sign_detector method

        #one-shot timer to mimic when a sign is detected
        timer = rospy.Timer(rospy.Duration(7), self.sign_detector, oneshot=True)



    def callback(self, data):                                    #gets speed & steering angle
        if self.new_sign == False:           #if there is no sign, keep speed command the same as the origional from pure pursuit
            rospy.loginfo('speed is %d m/s  &  Stop sign: %s', data.speed, self.new_sign)  #logs message recieved (speed) in terminal
            self.ackerman_speed = data.speed
        
        self.ackerman_steering = data.steering_angle



    def sign_detector(self, data):                   #may want to add code that re-checks if there is a need to stop
        
        self.time_stamp = time.time()            #gets the distance that the car is from the sign and the time at which it recieves the message
        self.distance =  15.0    #data.data    in meters   
        self.new_sign = True     #True means there is a sign
        print('Stop sign = %s  Distance to sign is %3d m' %(self.new_sign, self.distance))  #logs message recieved (speed) in terminal
        
        while self.new_sign == True:
            self.stop_the_car()
            self.counter = self.counter + 1

            if self.msg.speed <= 0:
                #self.new_sign = False
                print("Car Stopped")
                break

            time.sleep(1)

        self.new_sign = False



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

        self.msg.steering_angle = self.ackerman_steering
        self.msg.steering_angle_velocity = 0.0
        #self.msg.speed = self.ackerman_speed - 1*self.counter
        self.msg.acceleration = 0.0
        self.msg.jerk = 0.0


        #math for the slowing down  (constant acceleration) 
        #initial is at the time stamp when the car detects a vehicle
        #known variables: initial time, initial velocity, initial distance to stop sign
        #unknown variables: acceleration, time to stop
        
        self.initTime = self.time_stamp
        self.initVelo = self.ackerman_speed               
        #self.distance = already initialized
        self.timeToStop = (2*self.distance) / self.initVelo    #may have a certain scenario where need to stop as soon as possible so we can define this and solve for distance
        self.acceleration = (self.initVelo**2) / (2*self.distance)  #since this loop is currently runnung every (1) second can do (speed - acceleration) to slow car down

        self.msg.speed = self.ackerman_speed - (self.acceleration*self.counter)
                
        self.stop_command_pub.publish(self.msg)
        rospy.loginfo('SLOWING DOWN! %s m/s', self.msg.speed)



if __name__ == "__main__":
    # initialize node with rospy
    rospy.init_node('stop_behavior', anonymous=False)
    # create the node object
    StopBehaviorNode()
    # keep the node alive
    rospy.spin()

