#!/usr/bin/env python

'''
Node that controls the vehicle in CARLA using ROS
'''

import os

import numpy as np
import networkx as nx
import osmnx as ox
import matplotlib.pyplot as plt

from shapely.geometry import LineString

import carla


def convert_to_2D_map():
    '''
    Converts the opendrive map msg into a 2D topological map.

    Parameters
    ----------
    None

    Returns
    -------
    None
    '''
    
    # Initialize the carla client to pull the map from
    client = carla.Client('localhost', 2000)
    client.set_timeout(2)
    carla_world = client.get_world()

    # Use this line if you want to change the map
    # carla_world = client.load_world('Town01')

    cmap = carla_world.get_map()

    # Format the plot
    plt.margins(x=0.7, y=0)
    plt.axis('equal')

    # Pull the topology from the 3D map
    topology = cmap.get_topology()
    
    # Initialize an empty, undirected, MultiGraph
    graph = nx.MultiGraph()

    # For each waypoint in the topology, take the x and y coordinates
    # Y is negative due to a flip in UE4
    xy_topology = list()
    for wp1, wp2 in topology:
        xy_topology.append(((wp1.transform.location.x, -wp1.transform.location.y),(wp2.transform.location.x, -wp2.transform.location.y)))

    # Extract the edge information from the topology into the graph
    graph.add_edges_from(xy_topology)

    # Set the geometry between each edge to a straight line
    for source, target in graph.edges():
        graph[source][target][0]['geometry'] = LineString(
            [(source[0], source[1]),
             (target[0], target[1])])

    # Key the nodes their their values (for osmnx)
    for n1, n2 in list(graph.edges()):
        graph.node[n1]['x'] = n1[0]
        graph.node[n1]['y'] = n1[1]
        graph.node[n2]['x'] = n2[0]
        graph.node[n2]['y'] = n2[1]

    # Simplify the naming of the nodes to increasing integers
    graph = nx.convert_node_labels_to_integers(graph, 
        ordering="increasing degree")

    # Locate and Remove all nodes with no neighbors
    degree0_nodes = list()
    for node in graph:
        if(graph.degree(graph)[node]==0):
            degree0_nodes.append(node)
    graph.remove_nodes_from(degree0_nodes)

    # Set needed information for osmnx
    graph.graph['crs'] = None
    graph.graph['name'] = 'Carla Town'

    # Write a yaml version of the graph to the route_planner/scripts folder
    out_dir =  os.path.abspath(os.path.join(os.path.dirname(
                                           os.path.abspath(__file__)),
                        '..', '..', 'navigation', 'route_planner', 'scripts'))
    nx.write_yaml(graph, os.path.join(out_dir, 'carla_map.yaml'))

    # Plot the graph for a visual
    ox.plot_graph(graph, annotate=True)

if __name__ == '__main__':
    convert_to_2D_map()
