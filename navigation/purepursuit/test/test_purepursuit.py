#! /usr/bin/env python

from math import pi
import rostest
import unittest
import matplotlib.pyplot as plt

from shapely.geometry import Point, LineString
from purepursuit import PurePursuit
import rospy

def dpp():
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)

    # input data
    path = LineString([(1, 1), (8, 4)]) # given line path
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

    instance_of_PurePursuit = PurePursuit(wheelbase, lookahead_min, lookahead_max, lower_threshold_v, upper_threshold_v, lookahead_gain, speed=speed, vehicle_pose=(vehicle_cords.x, vehicle_cords.y, theta), path=path)

    print('vehicle coords: '+ str(vehicle_cords.x) + ',' + str(vehicle_cords.y))

    x,y = instance_of_PurePursuit.construct_path()

    # plot the path to be tracked
    ax1.plot(x,y)

    ax1.set_ylim(ymin=0, ymax=10)
    ax1.set_xlim(xmin=0, xmax=10)
    # plot vehicle coordinate and its base point with color blue
    ax1.arrow(vehicle_cords.x,vehicle_cords.y,dx=1,dy=1)
    ax1.plot(vehicle_cords.x,vehicle_cords.y,'ob')

    closest_pt = instance_of_PurePursuit.closest_point()

    print('closest pt: ' + str(closest_pt.x) + ',' + str(closest_pt.y))

    ### Print the path in dashed pink
    c, d = path.xy
    ax1.plot(c, d, '--p')

    #############
    #draw the vehicle orientation
    #a line connecting the rear and front axle
    front_pt = instance_of_PurePursuit.vehicle_front_point()
    ax1.plot(front_pt.x, front_pt.y, 'or')
    vehicle_line = LineString([(vehicle_cords.x, vehicle_cords.y), (front_pt.x, front_pt.y)])
    a, b = vehicle_line.xy
    ax1.plot(a, b)

    #############
    ax1.plot(closest_pt.x, closest_pt.y, 'og')
    ax1.axis('equal')

    project = LineString([(vehicle_cords.x, vehicle_cords.y), (closest_pt.x, closest_pt.y)])
    x1,y1 = project.xy
    ax1.plot(x1,y1)

    #############

    future_pt = instance_of_PurePursuit.future_point()
    ax1.plot(future_pt.x, future_pt.y, 'oy')

    goal = LineString([(vehicle_cords.x, vehicle_cords.y), (future_pt.x, future_pt.y)])
    x2,y2 = goal.xy
    ax1.plot(x2,y2, color='y')

    #############

    instance_of_PurePursuit.update_lookahead(speed, lookahead_min, lookahead_max, lower_threshold_v, upper_threshold_v, lookahead_gain)
    lookahead = instance_of_PurePursuit.get_lookahead()

    ###########

    r = instance_of_PurePursuit.compute_turning_radius()
    curv = instance_of_PurePursuit.compute_curvature()
    front_pt = instance_of_PurePursuit.vehicle_front_point()
    delta = instance_of_PurePursuit.compute_steering_angle()
    omega = instance_of_PurePursuit.compute_angular_speed()

    print('front vehicle pt: ' + str(front_pt))
    print('radius: ' + str(r))
    print('curvature: ' + str(curv))
    print('speed: ' + str(speed))
    print('lookahead: ' + str(lookahead))
    print('steering angle (rad): ' + str(delta))
    print('angular speed (rad/s): ' + str(omega))
    print('future point: ' + str(future_pt))
	 
    plt.show()
    return front_pt.x, front_pt.y, r, curv, speed, delta, omega, lookahead, future_pt, path

if __name__ == '__main__':
    print(dpp())
