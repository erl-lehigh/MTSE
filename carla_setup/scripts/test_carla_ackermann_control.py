#!/usr/bin/env python

'''
Node that controls the vehicle in CARLA using ROS
'''

import numpy as np
import cv2
import rospy
import networkx as nx


from cv_bridge import CvBridge

from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleInfo
from sensor_msgs.msg import Image, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from carla_ros_bridge.world_info import CarlaWorldInfo

import carla


class VehicleControllerNode(object):
    '''
    Attributes
    ----------
        IM_WIDTH : integer
            the width, in pixels, of the display window
        IM_HEIGHT : integer
            the height, in pixels, of the display window
        rate: rospy.Rate
            update rate
        ack_control_pub : rospy.Publisher
            ackermann control publisher
        vehicle_info_pub : rospy.Publisher
            vehicle information publisher
        ackermann_msg : ackermann_msgs.msg.AckermannDrive
            message to publish Ackermann commands to the vehicle

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

    def __init__(self, rate=1):
        '''
        VehicleControllerNode Constructor
        Parameters
        ----------
        None

        Returns
        -------
        None
        '''

        self.rate = rospy.Rate(rate)

        #---Publishers---#
        # Node to broadcast driving commands
        self.ack_control_pub = rospy.Publisher(
            '/carla/ego_vehicle/ackermann_cmd', AckermannDrive,
            queue_size=10)
        # Node for setting the vehicle information
        self.vehicle_info_pub = rospy.Publisher(
            '/carla/ego_vehicle/vehicle_info', CarlaEgoVehicleInfo,
            queue_size=10)

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
        # Map
        rospy.Subscriber("/carla/world_info", CarlaWorldInfo,
            self.convert_to_2D_map)

        # Initializes the msg
        self.ackermann_msg = AckermannDrive()

        # Initializes the vehicle type
        vehicle_info_msg = CarlaEgoVehicleInfo()
        vehicle_info_msg.type = "prius"
        # Message publication
        rospy.loginfo('Set vehicle type to %s', vehicle_info_msg.type)
        # Broadcasts the message
        self.vehicle_info_pub.publish(vehicle_info_msg)

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
        rospy.loginfo('lat: %f, lon: %f', nav.latitude, nav.longitude)
        self.rate.sleep()

    def print_odometry_location(self, loc):
        '''
        Converts the location msg into an (x,y,z) point.
        Parameters
        ----------
        loc : nav_msgs.msg.Odometry
            the odometry information from the vehicle's onboard odometry

        Returns
        -------
        None
        '''
        rospy.loginfo('x: %f, y: %f, z: %f', loc.pose.pose.position.x ,
             loc.pose.pose.position.y, loc.pose.pose.position.z)
        self.rate.sleep()

    def convert_to_2D_map(self, map3D):
        '''
        Converts the opendrive map msg into a 2D topological map.
        Parameters
        ----------
        map3D : CarlaWorldInfo.msg
            info about the CARLA world/level (e.g. OPEN Drive map)

        Returns
        -------
        None
        '''

        rospy.loginfo(map3D.opendrive)

        client = carla.Client('localhost', 2000)
        client.set_timeout(2)
        carla_world = client.get_world()
        cmap = carla_world.get_map()

        # The following is a list(tuple(carla.Waypoint,carla.Waypoint))
        cmap_topology = cmap.get_topology()

        rospy.loginfo(cmap_topology[0][0].transform.location)
        graph = nx.DiGraph()
        graph.add_edges_from(cmap_topology)
        for u in graph:
            location = u.transform.location
            graph.node[u]['location'] = (location.x, location.y)
        graph = nx.convert_node_labels_to_integers(graph)

        nx.write_yaml(graph, "carla_map.yaml")

        self.rate.sleep()

    def control(self, s, a, j, st, av):
        '''
        Drives the vehicle based on given values
        Parameters
        ----------
        s : float
            desired speed (m/s)
        a : float
            desired acceleration (m/s^2)
        j : float
            deseired change in acceleration (jerk) (m/s^3)
        st : float
            desired steering angle (radians)
        av : float
            desired steering angle velocity (radians/second)

        Returns
        -------
        None
        '''
        self.ackermann_msg.speed = s
        self.ackermann_msg.acceleration = a
        self.ackermann_msg.jerk = j
        self.ackermann_msg.steering_angle =  st
        self.ackermann_msg.steering_angle_velocity = av
        rospy.loginfo('Desired, s: %f, a: %f, j: %f, st: %f, av: %f',
            self.ackermann_msg.speed, self.ackermann_msg.acceleration,
            self.ackermann_msg.jerk, self.ackermann_msg.steering_angle,
            self.ackermann_msg.steering_angle_velocity) # Prints text
        # Broadcasts the message
        self.ack_control_pub.publish(self.ackermann_msg)
        self.rate.sleep() # Sleeps for time equal to the rate

if __name__ == '__main__':
    try:
        # Initialize nodes with rospy
        rospy.init_node('ackermannCtrl', anonymous=True)

        # Create the node object
        vehicle_node = VehicleControllerNode(rate=0.2)

        # Control Loop
        while not rospy.is_shutdown():
            pi = np.pi
            straight = 0.0
            right = pi / 3.0
            left = pi / -3.0

            vehicle_node.control(5 ,1 ,0.3 ,straight ,0.2)
            vehicle_node.control(5 ,1 ,0.3 ,right, 0.2)
            vehicle_node.control(5 ,1 ,0.3 ,left, 0.2)

    except rospy.ROSInterruptException:
        pass
