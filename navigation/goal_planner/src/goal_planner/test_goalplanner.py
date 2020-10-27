import itertools as it

import networkx as nx
import osmnx as ox
import matplotlib.pyplot as plt
import requests
import math
import shapely

import rospy
import tf2_ros
import tf.transformations as tr
from shapely.geometry import Point, LineString

from goal_planner import GoalPlanner


if __name__ == '__main__':
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)

    path = LineString([(1, 1), (8, 4)]) # given line path
    vehicle_cords = Point(2,3)   # initial vehicle coordinates
    theta = 0  # rad

    instance_of_GoalPlanner = GoalPlanner(vehicle_cords, theta, path)
    #instance_of_PurePursuit = PurePursuit(path, speed,vehicle_cords, theta)


    ax1.set_ylim(ymin=0, ymax=10)
    ax1.set_xlim(xmin=0, xmax=10)

    # plot vehicle coordinate and its base point with color blue
    ax1.arrow(vehicle_cords.x,vehicle_cords.y,dx=1,dy=1)
    ax1.plot(vehicle_cords.x,vehicle_cords.y,'ob')

    

    #############
    #draw the vehicle orientation
    #a line connecting the rear and front axle                                                        !! Need this? !!


    # ax1.plot(front_pt.x, front_pt.y, 'or')
    # vehicle_line = LineString([(vehicle_cords.x, vehicle_cords.y), (front_pt.x, front_pt.y)])       # !! Need pts? !! - same for later
    # a, b = vehicle_line.xy
    # ax1.plot(a, b)


    #############

    # ax1.plot(closest_pt.x, closest_pt.y, 'og')
    # ax1.axis('equal')

    # project = LineString([(vehicle_cords.x, vehicle_cords.y), (closest_pt.x, closest_pt.y)])
    # x1,y1 = project.xy
    # ax1.plot(x1,y1)

    # #############

    # future_pt = instance_of_GoalPlanner.future_point()
    # ax1.plot(future_pt.x, future_pt.y, 'oy')

    # goal = LineString([(vehicle_cords.x, vehicle_cords.y), (future_pt.x, future_pt.y)])
    # x2,y2 = goal.xy
    # ax1.plot(x2,y2, color='y')

    # #############                                                                                   #  !! ?? !!

    # r = instance_of_GoalPlanner.compute_r()
    # curv = instance_of_GoalPlanner.compute_curvature()
    # front_pt = instance_of_GoalPlanner.vehicle_front_pt()
    # delta = instance_of_GoalPlanner.compute_delta()
    # omega = instance_of_GoalPlanner.compute_omega()

    # print('front vehicle pt: ' + str(front_pt))
    # print('r: ' + str(r))
    # print('curvature: ' + str(curv))
    # print('speed: ' + str(speed))
    # print('delta (rad): ' + str(delta))
    # print('omega (rad/s): ' + str(omega))

    # plt.show()