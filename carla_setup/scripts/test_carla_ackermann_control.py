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
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from geometry_msgs.msg import TransformStamped
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
        rate: rospy.Rate
            how many times per second the node updates.
        ack_control_pub : rospy.Publisher
            ackermann control publisher.
        vehicle_info_pub : rospy.Publisher
            vehicle information publisher.
        node_name : string
            the name of the node.
        vehicle_type : string
            the type of vehicle to set the car to.
        ackermann_msg : ackermann_msgs.msg.AckermannDrive
            message to publish Ackermann commands to the vehicle

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
        rate = rospy.get_param('~rate', 5)
        self.vehicle_type = rospy.get_param('~vehicle_type', 'model3')

        # Sets a default current position to check against later
        self.current_pos = None

        # Sets how often the messages are sent (hz)
        self.rate = rospy.Rate(rate)

        # Publishers
        # Node to broadcast driving commands
        self.ack_control_pub = rospy.Publisher(
            '/carla/ego_vehicle/ackermann_cmd', AckermannDrive, queue_size=10)
        # Node for setting the vehicle information
        self.vehicle_info_pub = rospy.Publisher(
            '/carla/ego_vehicle/vehicle_info', CarlaEgoVehicleInfo,
            queue_size=10)

        # Subscribers
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
        rospy.Subscriber("speed_command", AckermannDrive, self.control)
        # Map
        rospy.Subscriber("/carla/world_info", CarlaWorldInfo,
                         self.convert_to_2D_map)

        # Initializes the msg
        self.ackermann_msg = AckermannDrive()

        # Initializes the vehicle type
        vehicle_info_msg = CarlaEgoVehicleInfo()
        vehicle_info_msg.type = self.vehicle_type
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
        self.rate.sleep()

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
            self.handle_location(pose, "ego_vehicle_filtered")
        else:
            self.handle_location(self.current_pos, "ego_vehicle_filtered")

    def handle_location(self, msg, childframe):
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

    def control(self, cmd_msgs):
        '''
        Drives the vehicle based on given values.

        Parameters
        ----------
        cmd_msgs : ackermann_msgs.msg.AckermannDrive
            Ackermann Drive message containing the speed (m/s), acceleration
            (m/s^2), jerk (m/s^3), steering angle (radians), and steering angle
            velocity (radians/s).

        Returns
        -------
        None
        '''
        # Store all of the values into a new message
        self.ackermann_msg = cmd_msgs

        # Debug information
        rospy.loginfo('\n'.join(['Desired',
                                 'speed: %5.3f m/s',
                                 'acceleration: %5.3f m/s^2',
                                 'jerk: %5.3f m/s^3',
                                 'steering angle: %5.4f radians',
                                 'angular velocity: %5.4f radians/s']),
                      self.ackermann_msg.speed,
                      self.ackermann_msg.acceleration,
                      self.ackermann_msg.jerk,
                      self.ackermann_msg.steering_angle,
                      self.ackermann_msg.steering_angle_velocity)

        # Broadcasts the message
        self.ack_control_pub.publish(self.ackermann_msg)
        self.rate.sleep() # Sleeps for time equal to the rate

if __name__ == '__main__':
    try:
        # Initialize nodes with rospy
        rospy.init_node('ackermannCtrl', anonymous=True)

        # Create the node object
        vehicle_node = VehicleControllerNode()

        # Keeps the node alive
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
