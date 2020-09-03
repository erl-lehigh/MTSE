#!/usr/bin/env python

'''
Node that controls the vehicle in CARLA using ROS
'''

import os
import glob
import sys
import random
import time
import cv2
import rosbag
import rospy

from cv_bridge import CvBridge

import numpy as np
import networkx as nx
import osmnx as ox
import matplotlib.pyplot as plt

from shapely.geometry import LineString
from sensor_msgs.msg import Image, PointCloud

import carla


class SaveLidarData(object):
    '''
    Saves the lidar data created by the ego_vehicle to a bag file using rosbag.
    
    Parameters
    ----------
    None

    Returns
    -------
    None
    '''
    def __init__(self):
        rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color",
        Image, self.process_img)

        rospy.Subscriber("/carla/ego_vehicle/lidar/front/point_cloud",
        PointCloud, self.process_lidar)

        bag = rosbag.Bag('Lidar_data.bag', 'w')


        # Read bag contents:

        # bag = rosbag.Bag('Lidar_data.bag')
        # for topic, msg, t in bag.read_messages(topics=['Point_Cloud_Data']):
        #     print(msg)
        # bag.close()


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

    def process_lidar(self, pt_cloud):
        print("here")
        bag.write('Point_Cloud_Data', pt_cloud)
        


if __name__ == '__main__':
    try:
        rospy.init_node('lidar_info', anonymous=True)
        data_node = SaveLidarData()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

