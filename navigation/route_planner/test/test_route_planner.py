#! /usr/bin/env python

import networkx as nx
import matplotlib.pyplot as plt

from route_planner import RoutePlanner

if __name__ == '__main__':
    g = nx.grid_2d_graph(4, 4) # creates 4 x 4 grid graph
    initial  = (0, 1)
    destination = (3, 3)

    route_planner = Routeplanner(g, initial)
    route = route_planner.get_route(destination)
    print(route) # prints the shortest path

    nx.draw(route_planner.g)
    plt.show() # displays graph is separate window
