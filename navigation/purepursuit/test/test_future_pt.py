#! /usr/bin/env python

import rostest
import unittest
from math import pi
import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString
from purepursuit import PurePursuit
import rospy
from test_paths import test_paths

def dpp():

    # input data
    #paths = [LineString([(1, 1), (8, 4)]), LineString([(1, 1), (8, 12)]), LineString([(8, 2), (8, 4)]), LineString([(10, 1), (8, 4)])] # given line path
    distances = []
    speed = 3   # given vehicle speed
    vehicle_cords = Point(2,3)   # initial vehicle coordinates
    theta = 0  # rad
    lookahead = rospy.get_param('~lookahead', 4)
    wheelbase = rospy.get_param('~wheelbase', 1)
    lookahead_min = rospy.get_param('~lookahead_min', 3)
    lookahead_max = rospy.get_param('~lookahead_max', 12)
    lower_threshold_v = rospy.get_param('~lower_threshold_v', 1.34)
    upper_threshold_v = rospy.get_param('~upper_threshold_v', 5.36)
    lookahead_gain = rospy.get_param('~lookahead_gain', 2.24)

    for path_List in test_paths:
    	#instance_of_PurePursuit = PurePursuit(path, speed,vehicle_cords, theta)	#should be  (wheelbase, lookahead, speed=None, vehicle_pose=None, path=None):
	path = LineString(path_List)
    	instance_of_PurePursuit = PurePursuit(wheelbase, lookahead, lookahead_min, lookahead_max, lower_threshold_v, upper_threshold_v, lookahead_gain, speed=speed, vehicle_pose=(vehicle_cords.x, vehicle_cords.y, theta), path=path)
    	x,y = instance_of_PurePursuit.construct_path()
    	closest_pt = instance_of_PurePursuit.closest_point()
    	future_pt = instance_of_PurePursuit.future_point()
    	instance_of_PurePursuit.update_lookahead(speed, lookahead_min, lookahead_max, lower_threshold_v, upper_threshold_v, lookahead_gain)
    	lookahead = instance_of_PurePursuit.get_lookahead()
	distances.append(path.distance(future_pt)) 

    return distances

if __name__ == '__main__':
    print(dpp())
