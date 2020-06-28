#!/usr/bin/env python

import random
import time
import numpy as np
import cv2
import rospy
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleInfo 
from sensor_msgs.msg import Image

def speed_and_steering():
    #Creates a publisher node, broadcasting the AckermannDrive messages found in /carla/ego_vehicle/ackermann_cmd
    pub = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDrive, queue_size=10)
    #Initializes the node on which to broadcast
    rospy.init_node('ackermannCtrl', anonymous=True)
    #Sets how often it broadcasts
    rate = rospy.Rate(0.25) # 0.25hz = 4s
    #Stores the AcermanDrive data into a message object
    msg = AckermannDrive()
    while not rospy.is_shutdown():
        #Changes the speed
        msg.speed = 1.0 - msg.speed
        msg.steering_angle = 1.0 - msg.steering_angle
        #Broadcasts text
        rospy.loginfo('Set speed to: %f, and steering to %f', msg.speed, msg.steering_angle)
        #Broadcasts the message
        pub.publish(msg)
        #Sleeps for time equal to the rate
        rate.sleep()

def v_info():
    pub = rospy.Publisher('/carla/ego_vehicle/vehicle_info', CarlaEgoVehicleInfo, queue_size=10)
    rospy.init_node('vehicle_info', anonymous=True)
    #rate = rospy.Rate(0.25) # 0.25hz = 4s
    msg = CarlaEgoVehicleInfo()
    msg.type = "prius"
    rospy.loginfo('Set vehicle type to %f', msg.type)
    #Broadcasts the message
    pub.publish(msg)
    #Sleeps for time equal to the rate
    #rate.sleep()

def rgbCam():
    rospy.init_node('rgbCam', anonymous=True)
    rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color", Image, process_img)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


IM_WIDTH = 640
IM_HEIGHT = 480

def process_img(image):
    i = np.array(image)
    #i = np.split(image, (image.height*image.width))
	#print(dir(image))
	#i2 = i.reshape((image.height, image.width, 4))
    #i3 = i[:, :, :3] #Just the RGB values of RGBA
    cv2.imshow("", i)
    cv2.waitKey(1)
    #return i3/255.0

if __name__ == '__main__':
    try:
        #v_info()
        rgbCam()
        speed_and_steering()
    except rospy.ROSInterruptException:
        pass
