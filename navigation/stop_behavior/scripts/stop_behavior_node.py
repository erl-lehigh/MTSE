#! /usr/bin/env python

import rospy
import math
import time
                       
from ackermann_msgs.msg import AckermannDrive                   
from stop_behavior.msg import TrafficSignStamped

class StopBehaviorNode(object):
    '''
    This node slows the car to a stop when a stop sign or other obstacle requires the car to stop at a certain point.

    Attributes
    ----------
        ackermann_speed : Float32
            Speed of car,  from ackermann_msgs/AckermannDrive.msg
        ackermann_steerangle : Float32
            Steering angle  
        distance : Float32
            the distance from car to stop point. 
            Will be calculated by camera/Hokuyo sensors and published to topic /traffic_sign that this code subscribes to
            It will also be updated as the car approaches the stop point
        new_sign : Boolean
            True when sign is detected in ahead of vehicle, False when no sign is detected
        min_stop_distance: Float32
            A constant value that determines the minimum distance from the car to the stop light where the car is good to be stopped (so the code doesn't break if the car is barely in front or behind stop point)
        period : Duration
            the amount of times per second the cwill run
        timer : Timer
            Continuosly running timer that calls sign_detector function at a certain rate
        time_stamp : Float32
            holds the time that a message is recieved to the traffic_sign topic
        time_left : Float 32
            the time left that the car must remain stopped at stop sign
        current_distance : Float32
            keeps track of current distance car is from stop point
        current_speed : Float32
            keeps track of current speed of car

    Publishers
    --------
        stop_command 

    Subscribers
    --------
        command
        stop_sign

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
        self.min_stop_distance = rospy.get_param('~min_stop_distance', 0.4)  
        self.stopping_time = rospy.get_param('~stopping_time', 5) 

        self.ackermann_speed = 0.0     
        self.ackermann_steering = 0.0 
                                                      
        self.distance = 0.0      
        self.new_sign = False     

        self.time_stamp = 0.0          
        self.time_left = self.stopping_time

        self.current_distance = 0.0
        self.current_speed = 0.0

        #Create Publisher objects 
        self.stop_command_pub = rospy.Publisher('stop_speed_command',   #this will publish the commands telling the car to slow to a stop
                                                AckermannDrive,        #message type
                                                queue_size=1)

        #Create Subscriber objects 
        self.command_sub = rospy.Subscriber('speed_command',   #subscribes to the 'speed_command' topic that pure pursuit publishes to
                            AckermannDrive,        
                            self.get_commands)          #every time something is recieved in this topic, it runs get_commads to get speed and steering angle

        self.stop_sign_sub = rospy.Subscriber('traffic_sign',   #subscribes to 'traffic_sign' topic.  When a stop sign is present, the traffic_sign topic gets a message that is the distance from the car to the sign
                            TrafficSignStamped,        #replace with custom sign message that will have all the stuff in it
                            self.sign_detector)         #every time something is recieved in this topic, runs sign_detector method

        self.timer = rospy.Timer(self.period, self.stop_the_car)  #calls sign_detector function at a certain rate 



    def get_commands(self, data):            #gets speed & steering angle generated from pure pursuit
        '''
        a callback type method that gets the car's speed and steering angle each time they are published to speed_command topic

        '''
        if self.new_sign == False:           
            rospy.loginfo('speed is %d m/s  &  Stop sign: %s', data.speed, self.new_sign)  #logs message recieved (speed) in terminal
        self.ackermann_speed = data.speed
        self.ackermann_steering = data.steering_angle



    def sign_detector(self, data):                   #may want to add code that re-checks if there is a need to stop
        '''
        a callback type method that gives the distance from the car to the stop sign and time stamps it
        
        '''
        if data.traffic_sign == 'stop':
            self.time_stamp = data.header.stamp          
            self.distance =  data.distance    #in meters   
            self.current_speed = self.ackermann_speed
            self.current_distance = self.distance
            self.new_sign = True     #True means there is a stop sign
            print("%s sign = %s  Distance to sign is %3d m" %(data.traffic_sign, self.new_sign, self.distance))  #logs message recieved (speed) in terminal
        

    def stop_the_car(self, event=None):        #need to figure out a way to call this method to stop the car,  event needed with the timer

        '''
        Method that takes the speed commands and modifies them to stop the car
        Only changing the speed of the car NOT the acceleration, jerk, steering velocity
        '''
        
        self.msg = AckermannDrive()  #Initialize message as AckermanDrive type

        self.msg.steering_angle = self.ackermann_steering
        self.msg.steering_angle_velocity = 0.0
        self.msg.acceleration = 0.0
        self.msg.jerk = 0.0


        #math for the slowing down  (constant acceleration) 
        #**we are ONLY changing speed commands not acceleration in this code**

        if self.new_sign == True:
            if self.current_distance >= self.min_stop_distance:  # if car is not close enough to the stop point (most likely it will not stop exactly at point) and protects from divide by zero
                #self.timeToStop = (2*self.distance) / self.initVelo    #may have a certain scenario where need to stop as soon as possible so we can define this and solve for distance
                self.acceleration = (self.current_speed**2) / (2*self.current_distance)  
                self.msg.speed = self.current_speed - (self.acceleration*self.period.to_sec())
                self.stop_command_pub.publish(self.msg)
                
                #update distance and speed
                self.current_distance = self.current_distance - (((self.current_speed + self.msg.speed) / 2)*(self.period.to_sec()))   #current_velo in this equation is actually the previous speed
                self.current_speed = self.msg.speed  

                rospy.loginfo('SLOWING DOWN! %2.5s m/s   Acc = %2.5s m/s^2, Dist = %2.5s ', self.msg.speed, self.acceleration, self.current_distance)

            else: 
                self.msg.speed = 0
                self.stop_command_pub.publish(self.msg)
                print("Car Stopped ", self.time_left)

                if self.time_left <= 0:
                    self.time_left = self.stopping_time
                    self.new_sign = False
                else:
                    self.time_left = self.time_left - self.period.to_sec()


if __name__ == "__main__":
    # initialize node with rospy
    rospy.init_node('stop_behavior', anonymous=False)
    # create the node object
    StopBehaviorNode()
    # keep the node alive
    rospy.spin()