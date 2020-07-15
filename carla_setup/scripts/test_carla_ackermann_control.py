#!/usr/bin/env python

import random
import time
import numpy as np
import cv2
import rospy
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleInfo 
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge

class VehicleControllerNode(object):
    IM_WIDTH = 640
    IM_HEIGHT = 480
    def __init__(self):
        #Publishers
        #Creates a publisher node, broadcasting the AckermannDrive messages found in /carla/ego_vehicle/ackermann_cmd
        self.aCtrl_pub = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDrive, queue_size=10)

        #Node for setting the vehicle information
        self.vInfo_pub = rospy.Publisher('/carla/ego_vehicle/vehicle_info', CarlaEgoVehicleInfo, queue_size=10)

        #Subscribers
        rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color", Image, self.process_img)
        rospy.Subscriber("/carla/ego_vehicle/gnss/front/gnss", NavSatFix, self.print_location)

    def process_img(self, image):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(image, "bgr8")
        cv2.imshow("", img)
        cv2.waitKey(1)

    def print_location(self, nav):
        print("lat: %f, lon: %f", nav.latitude, nav.longitude)

def control(s, a, j, st, av):
        msg.speed = s
        msg.acceleration = a
        msg.jerk = j
        msg.steering_angle =  st
        msg.steering_angle_velocity = av
        rospy.loginfo('Desired, s: %f, a: %f, j: %f, st: %f, av: %f', msg.speed, msg.acceleration, msg.jerk, msg.steering_angle, msg.steering_angle_velocity) #prints text
        vNode.aCtrl_pub.publish(msg) #Broadcasts the message
        rate.sleep() #Sleeps for time equal to the rate

if __name__ == '__main__':
    try:
        # Initialize nodes with rospy
        rospy.init_node('ackermannCtrl', anonymous=True)

        # Create the node object
        vNode = VehicleControllerNode()
        #Initializes the msgs
        msg = AckermannDrive()
        msg2 = CarlaEgoVehicleInfo()
        #Sets how often the messages are sent
        rate = rospy.Rate(0.2) # 0.2hz
        #Initializes the vehicle type
        msg2.type = "prius"
             
        #Message publication
        rospy.loginfo('Set vehicle type to %s', msg2.type)
        vNode.vInfo_pub.publish(msg2) #Broadcasts the message

        #Control Loop
        while not rospy.is_shutdown():
            #Speed in m/s
            #Acceleration in m/s^2
            #Jerk in m/s^3
            #Steering angle in radians
            #Sterring angle velocity in radians/s

            pi = 3.14159265358979
            straight = 0.0
            right = pi / 3.0
            left = pi / -3.0

            control(5 ,1 ,0.3 ,straight ,0.2)
            control(5 ,1 ,0.3 ,right, 0.2)
            control(5 ,1 ,0.3 ,left, 0.2)

           

    except rospy.ROSInterruptException:
        pass
