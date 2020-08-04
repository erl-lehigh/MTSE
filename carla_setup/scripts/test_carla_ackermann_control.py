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
        # Commands
        rospy.Subscriber("pure_commands", AckermannDrive, control)

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
        rate.sleep()

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
        handle_location(loc.pose.pose, "vehicle")
        rospy.loginfo('x: %f, y: %f, z: %f', loc.pose.pose.position.x ,
             loc.pose.pose.position.y, loc.pose.pose.position.z)
        rate.sleep()

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
        
        #rospy.loginfo(map3D.opendrive)

        client = carla.Client('localhost', 2000)
        client.set_timeout(2)
        carla_world = client.get_world()
        #carla_world = client.load_world('Town01')
        cmap = carla_world.get_map()

        ###Code Credit: YashBansod
        # Invert the y axis since we follow UE4 coordinates
        plt.gca().invert_yaxis()
        plt.margins(x=0.7, y=0)

        topology = cmap.get_topology()
        road_list = []

        for wp_pair in topology:
            current_wp = wp_pair[0]
            # Check if there is a road with no previus road, this can happen
            # in opendrive. Then just continue.
            if current_wp is None:
                continue
            # First waypoint on the road that goes from wp_pair[0] to wp_pair[1].
            current_road_id = current_wp.road_id
            wps_in_single_road = [current_wp]
            # While current_wp has the same road_id (has not arrived to next road).
            while current_wp.road_id == current_road_id:
                # Check for next waypoints in aprox distance.
                available_next_wps = current_wp.next(20.0)
                # If there is next waypoint/s?
                if available_next_wps:
                    # We must take the first ([0]) element because next(dist) can
                    # return multiple waypoints in intersections.
                    current_wp = available_next_wps[0]
                    wps_in_single_road.append(current_wp)
                else: # If there is no more waypoints we can stop searching for more.
                    break
            road_list.append(wps_in_single_road)

        # Plot each road (on a different color by default)
        for road in road_list:
            plt.plot(
                [wp.transform.location.x for wp in road],
                [wp.transform.location.y for wp in road])

        plt.show()

        graph = nx.DiGraph()

        list3 = {}
        count = 0

        for road in road_list:
            list1 = [wp.transform.location.x for wp in road]
            list2 = [wp.transform.location.y for wp in road]
            tuples = [(list1[i], list2[i]) for i in range(0, len(list1))]
            graph.add_nodes_from(tuples)
            for point in range(0, len(list1)):
                list3[(list1[point], list2[point])] = (list1[point], list2[point])
                count += 1

        '''
        for u in graph:
            location = u.transform.location
            graph.node[u]['location'] = (location.x, location.y)
        '''
        nx.draw_networkx(graph, pos=list3, arrows=True, node_size=10, font_size=1)
        plt.show()

        nx.write_yaml(graph,
         "/home/nathan/mtse_catkin/src/navigation/route_planner/scripts/carla_map.yaml")

        rate.sleep()

def handle_location(msg, childframe):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
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


def control(s, a, j, st, av):
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
        ackermann_msg.speed = s
        ackermann_msg.acceleration = a
        ackermann_msg.jerk = j
        ackermann_msg.steering_angle =  st
        ackermann_msg.steering_angle_velocity = av
        rospy.loginfo('Desired, s: %f, a: %f, j: %f, st: %f, av: %f', 
            ackermann_msg.speed, ackermann_msg.acceleration, ackermann_msg.jerk, ackermann_msg.steering_angle, ackermann_msg.steering_angle_velocity) # Prints text
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
        rate = rospy.Rate(0.2) # 0.2hz
        # Initializes the vehicle type
        vehicle_info_msg.type = "prius"
             
        # Message publication
        rospy.loginfo('Set vehicle type to %s', vehicle_info_msg.type)
        # Broadcasts the message
        vehicle_node.vehicle_info_pub.publish(vehicle_info_msg) 

        # Control Loop
        while not rospy.is_shutdown():
            pi = np.pi
            straight = 0.0
            right = pi / 3.0
            left = pi / -3.0

            control(5 ,1 ,0.3 ,straight ,0.2)
            control(5 ,1 ,0.3 ,right, 0.2)
            control(5 ,1 ,0.3 ,left, 0.2)

    except rospy.ROSInterruptException:
        pass
