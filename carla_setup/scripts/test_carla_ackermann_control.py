#!/usr/bin/env python

import numpy as np
import cv2
import rospy

from cv_bridge import CvBridge

from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleInfo 
from sensor_msgs.msg import Image, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance


class VehicleControllerNode(object):
    IM_WIDTH = 640
    IM_HEIGHT = 480

    def __init__(self):

        #---Publishers---#
        #Node to broadcast driving commands
        self.ack_control_pub = rospy.Publisher(
            '/carla/ego_vehicle/ackermann_cmd', AckermannDrive,
            queue_size=10)
        #Node for setting the vehicle information
        self.vehicle_info_pub = rospy.Publisher(
            '/carla/ego_vehicle/vehicle_info', CarlaEgoVehicleInfo, queue_size=10)
            
        #---Subscribers---#
        #Camera
        rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color",
            Image, self.process_img)
        #Gnss
        rospy.Subscriber("/carla/ego_vehicle/gnss/gnss1/fix", NavSatFix,
             self.print_location)
        #Odometry
        rospy.Subscriber("/carla/ego_vehicle/odometry",Odometry, 
            self.print_location2)

    def process_img(self, image):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(image, "bgr8")
        cv2.imshow("", img)
        cv2.waitKey(1)

    def print_location(self, nav):
        rospy.loginfo('lat: %f, lon: %f', nav.latitude, nav.longitude)
        rate.sleep()

    def print_location2(self, loc):
        rospy.loginfo('x: %f, y: %f, z: %f', loc.pose.pose.position.x ,
             loc.pose.pose.position.y, loc.pose.pose.position.z)
        rate.sleep()


def control(s, a, j, st, av):
        ackermann_msg.speed = s
        ackermann_msg.acceleration = a
        ackermann_msg.jerk = j
        ackermann_msg.steering_angle =  st
        ackermann_msg.steering_angle_velocity = av
        rospy.loginfo('Desired, s: %f, a: %f, j: %f, st: %f, av: %f', 
            ackermann_msg.speed, ackermann_msg.acceleration, ackermann_msg.jerk, ackermann_msg.steering_angle, ackermann_msg.steering_angle_velocity) #prints text
        vehicle_node.ack_control_pub.publish(ackermann_msg) #Broadcasts the message
        rate.sleep() #Sleeps for time equal to the rate

if __name__ == '__main__':
    try:
        # Initialize nodes with rospy
        rospy.init_node('ackermannCtrl', anonymous=True)

        # Create the node object
        vehicle_node = VehicleControllerNode()
        #Initializes the msgs
        ackermann_msg = AckermannDrive()
        vehicle_info_msg = CarlaEgoVehicleInfo()
        #Sets how often the messages are sent
        rate = rospy.Rate(0.2) # 0.2hz
        #Initializes the vehicle type
        vehicle_info_msg.type = "prius"
             
        #Message publication
        rospy.loginfo('Set vehicle type to %s', vehicle_info_msg.type)
        vehicle_node.vehicle_info_pub.publish(vehicle_info_msg) #Broadcasts the message

        #Control Loop
        while not rospy.is_shutdown():
            #Speed in m/s
            #Acceleration in m/s^2
            #Jerk in m/s^3
            #Steering angle in radians
            #Sterring angle velocity in radians/s

            pi = np.pi
            straight = 0.0
            right = pi / 3.0
            left = pi / -3.0

            control(5 ,1 ,0.3 ,straight ,0.2)
            control(5 ,1 ,0.3 ,right, 0.2)
            control(5 ,1 ,0.3 ,left, 0.2)

    except rospy.ROSInterruptException:
        pass
