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
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, TransformStamped
from carla_ros_bridge.world_info import CarlaWorldInfo
import matplotlib.pyplot as plt

import carla


class VehicleControllerNode(object):
    '''
    Attributes
    ----------
        IM_WIDTH : integer
            the width, in pixels, of the display window.
        IM_HEIGHT : integer
            the height, in pixels, of the display window.
        ack_control_pub : rospy.Publisher
            ackermann control publisher.
        vehicle_info_pub : rospy.Publisher
            vehicle information publisher.
                     
    Methods
    -------
        process_img(self, image):
            Uses the cv-bridge to convert the rgb-camera's image for viewing.
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
        VehicleControllerNode Constructor.

        Parameters
        ----------
        None

        Returns
        -------
        None
        '''

        # Set the node's name
        self.node_name = rospy.get_name()

        # Gets any parameters on the node
        self.rate = rospy.get_param('~rate', 5)
        self.vehicle_type = rospy.get_param('~vehicle_type', 'model3')

        # Sets a default current position to check against later
        self.current_pos = None

        #Publishers:
        # Node to broadcast driving commands
        self.ack_control_pub = rospy.Publisher(
            '/carla/ego_vehicle/ackermann_cmd', AckermannDrive,
            queue_size=10)
        # Node for setting the vehicle information
        self.vehicle_info_pub = rospy.Publisher(
            '/carla/ego_vehicle/vehicle_info', CarlaEgoVehicleInfo, queue_size=10)
            
        #Subscribers:
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
            the image from the rgb-sensor.

        Returns
        -------
        None
        '''
        # Sets up a CV Bridge
        bridge = CvBridge()

        # Converts the image and displays it
        img = bridge.imgmsg_to_cv2(image, "bgr8")
        cv2.imshow("", img)
        cv2.waitKey(1)

    def print_GNSS_location(self, nav):
        '''
        Converts the navigation msg into latitude and longitude.

        Parameters
        ----------
        nav : sensor_msgs.msg.NavSatFix
            the navigation information from a GNSS sensor.

        Returns
        -------
        None
        '''
        # Here GNSS data can be used
        # rospy.loginfo('lat: %f, lon: %f', nav.latitude, nav.longitude)
        rate.sleep()

    def print_odometry_location(self, loc):
        '''
        Converts the location msg into an (x,y,z) point and sends it to the tf.

        Parameters
        ----------
        loc : nav_msgs.msg.Odometry
            the odometry information from the vehicle's onboard odometry.

        Returns
        -------
        None
        '''
        # Simplify the message information
        pose = loc.pose.pose

        # Initialize current position
        if(self.current_pos == None):
            self.current_pos = pose

        # Calculate displacements
        xdisp = pose.position.x - self.current_pos.position.x
        ydisp = pose.position.y - self.current_pos.position.y

        # Broadcast the location only if the movement is realistic
        # This eliminates noise
        tolerance = 15
        if(abs(xdisp)<tolerance and abs(ydisp)<tolerance):
            self.current_pos = pose
            handle_location(pose, "ego_vehicle_filtered")
        else:
            handle_location(self.current_pos, "ego_vehicle_filtered")

def handle_location(msg, childframe):
    '''
    Uses a tf to broadcast the location data.

    Parameters
    ----------
    msg : Pose
        the pose that needs to be broadcast.
    childframe : string
        the name of the child frame.

    Returns
    -------
    None
    '''
    # Set up the transforms
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    # Set the header information
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = childframe

    # Set (x,y,z)
    t.transform.translation.x = msg.position.x
    t.transform.translation.y = msg.position.y
    t.transform.translation.z = 0.0

    # Set Rotation
    q = msg.orientation
    t.transform.rotation.x = q.x
    t.transform.rotation.y = q.y
    t.transform.rotation.z = q.z
    t.transform.rotation.w = q.w

    # Broadcast the information
    br.sendTransform(t)


def control(cmd_msgs):
        '''
        Drives the vehicle based on given values.

        Parameters
        ----------
        cmd_msgs : ackermann_msgs.msg.AckermannDrive
            Ackermann Drive message containing the speed (m/s), acceleration (m/s^2), jerk (m/s^3), steering angle (radians), and steering angle velocity (radians/s).

        Returns
        -------
        None
        '''
        # Store all of the values into a new message
        ackermann_msg.speed = cmd_msgs.speed
        ackermann_msg.acceleration = cmd_msgs.acceleration
        ackermann_msg.jerk = cmd_msgs.jerk
        ackermann_msg.steering_angle = cmd_msgs.steering_angle
        ackermann_msg.steering_angle_velocity = cmd_msgs.steering_angle_velocity

        # Debug information
        rospy.loginfo(
            'Desired\n speed: %5.3f m/s\n acceleration: %5.3f m/s^2\n jerk: %5.3f m/s^3\n steering angle: %5.4f radians\n angular velocity: %5.4f radians/s', ackermann_msg.speed, ackermann_msg.acceleration, ackermann_msg.jerk, ackermann_msg.steering_angle, ackermann_msg.steering_angle_velocity)

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

        # Sets how often the messages are sent (hz)
        rate = rospy.Rate(vehicle_node.rate)

        # Initializes the vehicle type
        vehicle_info_msg.type = vehicle_node.vehicle_type
        rospy.loginfo('Set vehicle type to %s', vehicle_info_msg.type)
        vehicle_node.vehicle_info_pub.publish(vehicle_info_msg)

        # Keeps the node alive
        rospy.spin() 

    except rospy.ROSInterruptException:
        pass
