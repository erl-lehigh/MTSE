#!/usr/bin/env python

'''
Node that controls the vehicle in CARLA using ROS
'''

import numpy as np
import cv2
import rospy
import networkx as nx
import tf_conversions
import tf2_ros
import math


from cv_bridge import CvBridge

from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleInfo 
from sensor_msgs.msg import Image, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, TransformStamped, PolygonStamped, Point32
from carla_ros_bridge.world_info import CarlaWorldInfo
import matplotlib.pyplot as plt

import carla


class VehicleControllerNode(object):
    '''
    Attributes
    ----------
        IM_WIDTH : integer
            the width, in pixels, of the display window
        IM_HEIGHT : integer
            the height, in pixels, of the display window
        ack_control_pub : rospy.Publisher
            ackermann control publisher
        vehicle_info_pub : rospy.Publisher
            vehicle information publisher
                     
    Methods
    -------
        process_img(self, image):
            Uses the cv-bridge to convert the rgb-camera's image for viewing
        print_GNSS_location(self, nav):
            Prints and broadcasts the navigation information as latitude and
            longitude.
        print_odometry_location(self, loc):
            Prints and broadcasts the location of the vehicle with respect to
            the world as an (x,y,z) point.
    '''

    IM_WIDTH = 640
    IM_HEIGHT = 480

    def __init__(self):
        '''
        VehicleControllerNode Constructor
        Parameters
        ----------
        None

        Returns
        -------
        None
        '''

        #---Publishers---#
        # Node to broadcast driving commands
        self.ack_control_pub = rospy.Publisher(
            '/carla/ego_vehicle/ackermann_cmd', AckermannDrive,
            queue_size=10)
        # Node for setting the vehicle information
        self.vehicle_info_pub = rospy.Publisher(
            '/carla/ego_vehicle/vehicle_info', CarlaEgoVehicleInfo, queue_size=10)
        # Node for polygon in rviz
        self.vehicle_base_pub = rospy.Publisher(
            '/vehicle_polygon_base', PolygonStamped, queue_size=10)
            
        #---Subscribers---#
        # Camera
        rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color",
            Image, self.process_img)
        # Gnss
        rospy.Subscriber("/carla/ego_vehicle/gnss/gnss1/fix", NavSatFix,
             self.print_GNSS_location)
        # Odometry
        rospy.Subscriber("/carla/ego_vehicle/odometry",Odometry, 
            self.print_odometry_location)
        # Commands
        rospy.Subscriber("speed_command", AckermannDrive, control)

    def process_img(self, image):
        '''
        Processes the RGB-sensor image into a viewable version.
        Parameters
        ----------
        image : sensor_msgs.msg.Image
            the image from the rgb-sensor

        Returns
        -------
        None
        '''
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(image, "bgr8")
        cv2.imshow("", img)
        cv2.waitKey(1)

    def print_GNSS_location(self, nav):
        '''
        Converts the navigation msg into latitude and longitude.
        Parameters
        ----------
        nav : sensor_msgs.msg.NavSatFix
            the navigation information from a GNSS sensor

        Returns
        -------
        None
        '''
        # rospy.loginfo('lat: %f, lon: %f', nav.latitude, nav.longitude)
        rate.sleep()

    def print_odometry_location(self, loc):
        '''
        Converts the location msg into an (x,y,z) point and sends it to the tf.
        Parameters
        ----------
        loc : nav_msgs.msg.Odometry
            the odometry information from the vehicle's onboard odometry

        Returns
        -------
        None
        '''
        handle_location(loc.pose.pose, "ego_vehicle")
        # create_polygon(loc.pose.pose)
        # rospy.loginfo('x: %f, y: %f, z: %f', loc.pose.pose.position.x ,
            #  loc.pose.pose.position.y, loc.pose.pose.position.z)
        rate.sleep()

def handle_location(msg, childframe):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = childframe
    t.transform.translation.x = msg.position.x
    t.transform.translation.y = msg.position.y
    t.transform.translation.z = 0.0
    q = msg.orientation
    t.transform.rotation.x = q.x
    t.transform.rotation.y = q.y
    t.transform.rotation.z = q.z
    t.transform.rotation.w = q.w
    br.sendTransform(t)

def create_polygon(msg):
        poly_msg = PolygonStamped()
        poly_msg.header.stamp = rospy.Time.now()
        poly_msg.header.frame_id = "map"
        poly_msg.polygon.points.append(Point32())
        poly_msg.polygon.points.append(Point32())
        poly_msg.polygon.points.append(Point32())
        poly_msg.polygon.points.append(Point32())
        mult = 5
        poly_msg.polygon.points[0].x = msg.position.x+mult*math.cos(math.atan2(msg.orientation.y, msg.orientation.x))
        poly_msg.polygon.points[0].y = msg.position.y+mult*math.sin(math.atan2(msg.orientation.y, msg.orientation.x))
        poly_msg.polygon.points[1].x = msg.position.x+mult*math.cos(math.atan2(msg.orientation.y, msg.orientation.x))
        poly_msg.polygon.points[1].y = msg.position.y-mult*math.sin(math.atan2(msg.orientation.y, msg.orientation.x))
        poly_msg.polygon.points[2].x = msg.position.x-mult*math.cos(math.atan2(msg.orientation.y, msg.orientation.x))
        poly_msg.polygon.points[2].y = msg.position.y-mult*math.sin(math.atan2(msg.orientation.y, msg.orientation.x))
        poly_msg.polygon.points[3].x = msg.position.x-mult*math.cos(math.atan2(msg.orientation.y, msg.orientation.x))
        poly_msg.polygon.points[3].y = msg.position.y+mult*math.sin(math.atan2(msg.orientation.y, msg.orientation.x))
        # Broadcasts the message
        vehicle_node.vehicle_base_pub.publish(poly_msg)


def control(cmd_msgs):
        '''
        Drives the vehicle based on given values
        Parameters
        ----------
        s : float
            desired speed (m/s)
        st : float
            desired steering angle (radians)
        a : float
            desired acceleration (m/s^2)
        j : float
            deseired change in acceleration (jerk) (m/s^3)
        av : float
            desired steering angle velocity (radians/second)

        Returns
        -------
        None
        '''
        a=0
        j=0
        av=0
        ackermann_msg.speed = cmd_msgs.speed
        ackermann_msg.acceleration = a
        ackermann_msg.jerk = j
        ackermann_msg.steering_angle = cmd_msgs.steering_angle
        ackermann_msg.steering_angle_velocity = av
        rospy.loginfo(
            'Desired\n speed: %5.3f m/s\n acceleration: %5.3f m/s^2\n jerk: %5.3f m/s^3\n steering angle: %5.4f radians\n angular velocity: %5.4f radians/s', ackermann_msg.speed, ackermann_msg.acceleration, ackermann_msg.jerk, ackermann_msg.steering_angle, ackermann_msg.steering_angle_velocity) # Prints text
        # Broadcasts the message
        vehicle_node.ack_control_pub.publish(ackermann_msg) 
        rate.sleep() # Sleeps for time equal to the rate

if __name__ == '__main__':
    try:
        # Initialize nodes with rospy
        rospy.init_node('ackermannCtrl', anonymous=True)

        # Create the node object
        vehicle_node = VehicleControllerNode()
        # Initializes the msgs
        ackermann_msg = AckermannDrive()
        vehicle_info_msg = CarlaEgoVehicleInfo()
        # Sets how often the messages are sent
        rate = rospy.Rate(5) # hz
        # Initializes the vehicle type
        vehicle_info_msg.type = "model3"
             
        # Message publication
        rospy.loginfo('Set vehicle type to %s', vehicle_info_msg.type)
        # Broadcasts the message
        vehicle_node.vehicle_info_pub.publish(vehicle_info_msg) 

        # Control Loop
        while not rospy.is_shutdown():
            pi = np.pi
            straight = 0.0
            left = pi / 3.0
            right = pi / -3.0

            #control(5 ,straight ,1 ,0.3 ,0.2)
            #control(5 ,right, 1 ,0.3 , 0.2)
            #control(5 ,left, 1 ,0.3 , 0.2)

    except rospy.ROSInterruptException:
        pass
