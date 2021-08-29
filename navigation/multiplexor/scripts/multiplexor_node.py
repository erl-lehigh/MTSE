#! /usr/bin/env python

import rospy
import math
import time
                       
from ackermann_msgs.msg import AckermannDrive                   
from std_msgs.msg import Float32
from stop_behavior.msg import TrafficSignStamped

class Multiplexor(object):
    '''
    This node slows the car to a stop when a stop sign or other obstacle requires the car to stop at a certain point.

    Attributes
    ----------
        new_sign : Boolean
            True when sign is detected in ahead of vehicle, False when no sign is detected
        sign_type : String
            stores the type of sign
        stop_message : AckermannDrive
            stores stop behavior speed commands
        purepursuit_message : AckermannDrive
            stores purepursuit speed commands
        timer : Timer
            Continuosly running timer that calls siend_to_car function at a certain rate
        
    Publishers
    --------
        vehicle_command

    Subscribers
    --------
        pure_pursuit
        stop_sign
        traffic_sign

    '''
    
    def __init__(self):
        '''
        Initializes necessary variables and creates publisher and subscriber objects
        '''
        #Initial Stuff
        self.rate = 10   #rospy.get_param('~rate',1)
        self.period = rospy.Duration(1.0 / self.rate)   #period is 1 hertz (1 cyle per second)
        self.node_name = rospy.get_name()
        self.parent_frame = rospy.get_param('~parent_frame', 'world')
        self.child_frame = rospy.get_param('~child_frame', 'vehicle')                 
           
        self.new_sign = False     
        self.sign_type = ""
        self.stop_message = AckermannDrive()
        self.purepursuit_message = AckermannDrive()
        self.emergency_message =  AckermannDrive()
        self.emergency_message.speed = 0.0
        self.switch_back = "no"

        #Create Publisher objects 
        self.vehicle_command_pub = rospy.Publisher('multiplexor_command',   #this will publish the commands for the vehicle
                                                AckermannDrive,        
                                                queue_size=1)

        #Create Subscriber objects 
        self.pure_pursuit_sub = rospy.Subscriber('speed_command',  #subscribes to the 'speed_command' topic that pure pursuit publishes to
                            AckermannDrive,        
                            self.run_purepursuit)          

        self.stop_sign_sub = rospy.Subscriber('stop_speed_command',  #subscribes to commands pubished by stop_behavior_node
                            AckermannDrive,
                            self.run_stopbehavior)     

        self.traffic_sign_sub = rospy.Subscriber('traffic_sign', #subscribes to 'traffic_sign' topic which published info about sign whe detected
                            TrafficSignStamped,       
                            self.sign_detected)        


        self.timer = rospy.Timer(self.period, self.send_to_car)  #calls sign_detected function at a certain rate


    def run_purepursuit(self, data):            
        '''
        a callback type method that stores purepursuit commands
        '''
        self.purepursuit_message = data

    def run_stopbehavior(self, data):                  
        '''
        a callback type method that stores stop behavior commands
        '''
        self.stop_message = data
        if self.stop_message.jerk == 10.0:
            self.switch_back = "yes"
        
    def sign_detected(self, data):      
        '''
       a callback type method that is run when a sign is detected
        '''
        if data.traffic_sign == "stop":
            self.new_sign = True
        else:
            self.new_sign = False
        self.sign_type = data.traffic_sign

    def send_to_car(self, event=None):
        ''' 
        method that is constatnly publishing the correct commands to the car 
        '''
        if self.new_sign == True and self.sign_type == "stop":
            self.vehicle_command_pub.publish(self.stop_message)
            rospy.logdebug("sending stop_behavior commands") 
            if self.switch_back == "yes":
                rospy.logdebug("Car done stopping. Switching to purepursuit")
                self.new_sign = False
            
        elif self.sign_type == "emergency":
            self.vehicle_command_pub.publish(self.emergency_message)
            rospy.logdebug('EMERGENCY STOP')

        else:
            self.switch_back = "no"
            self.new_sign = False
            self.vehicle_command_pub.publish(self.purepursuit_message)
            rospy.logdebug("sending purepursuit commands")


if __name__ == "__main__":
    # initialize node with rospy
    rospy.init_node('multiplexor', anonymous=False, log_level=rospy.DEBUG)
    # create the node object
    Multiplexor()
    # keep the node alive
    rospy.spin()