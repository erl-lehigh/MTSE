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
        distance : Float32
            the distance from car to stop point. 
            Will be calculated by camera/Hokuyo sensors and published to topic /traffic_sign that this code subscribes to
            It will also be updated as the car approaches the stop point
        new_sign : Boolean
            True when sign is detected in ahead of vehicle, False when no sign is detected
        period : Duration
            the amount of times per second the cwill run
        timer : Timer
            Continuosly running timer that calls sign_detector function at a certain rate
        time_stamp : Float32
            holds the time that a message is recieved to the traffic_sign topic
        current_distance : Float32
            keeps track of current distance car is from stop point
        current_speed : Float32
            keeps track of current speed of car

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


        self.stop_message = AckermannDrive()
        self.purepursuit_message = AckermannDrive()

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
        a callback type method that makes the purepursuit commands get sent to the car

        '''
        self.purepursuit_message = data

    def run_stopbehavior(self, data):                  
        '''
        a callback type method that makes the stop behavior commands get sent to the car
        
        '''
        self.stop_message = data
        
    def sign_detected(self, data):      
        '''
       a callback type method that is run when a sign is detected.  It then directs the code to perform the proper action. 
       
        '''
        if data.traffic_sign == "stop":
            self.new_sign = True
        else:
            self.new_sign = False
    

    def send_to_car(self, event=None):
        ''' 
        method that is constatnly publishing to the car 
        
        '''

        if self.new_sign == True:
            self.vehicle_command_pub.publish(self.stop_message)
            print("sending stop_behavior commands") 
        else:
            self.new_sign = False
            self.vehicle_command_pub.publish(self.purepursuit_message)
            print("sending purepursuit commands")


if __name__ == "__main__":
    # initialize node with rospy
    rospy.init_node('multiplexor', anonymous=False)
    # create the node object
    Multiplexor()
    # keep the node alive
    rospy.spin()