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

    path = LineString([(1, 1), (4, 4), (10, 5), (11, 2)]) # given line path
    vehicle_cords = Point(5,3)   # initial vehicle coordinates
    theta = 0  # rad

    instance_of_GoalPlanner = GoalPlanner(vehicle_cords, theta, path)

    ax1.set_ylim(ymin=0, ymax=15)
    ax1.set_xlim(xmin=0, xmax=15)

    # plot vehicle (blue dot) & orientation (black line)
    ax1.arrow(vehicle_cords.x,vehicle_cords.y,
        dx=math.cos(theta),dy=math.sin(theta))
    ax1.plot(vehicle_cords.x,vehicle_cords.y,'ob')
    x, y = path.xy

    # plot the path
    ax1.plot(x,y,'-r')

    # plot the nodes on the path
    ax1.plot(x,y,'ok')

    # plot the goal node in green
    ax1.plot(instance_of_GoalPlanner.get_goal_node().x,
         instance_of_GoalPlanner.get_goal_node().y,'og')

    plt.show()

