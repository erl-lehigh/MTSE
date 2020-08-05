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
    
    client = carla.Client('localhost', 2000)
    client.set_timeout(2)
    carla_world = client.get_world()
    #carla_world = client.load_world('Town01')
    cmap = carla_world.get_map()

    ###Code Credit: YashBansod
    # Invert the y axis since we follow UE4 coordinates
    plt.gca().invert_yaxis()
    plt.margins(x=0.7, y=0)
    plt.axis('equal')

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
            available_next_wps = current_wp.next(2.0)
            # If there is next waypoint/s?
            if available_next_wps:
                # We must take the first ([0]) element because next(dist) can
                # return multiple waypoints in intersections.
                current_wp = available_next_wps[0]
                wps_in_single_road.append(current_wp)
            else: # If there is no more waypoints we can stop searching for more.
                break
        road_list.append(wps_in_single_road)
    '''
    # Plot each road (on a different color by default)
    for road in road_list:
        plt.plot(
            [wp.transform.location.x for wp in road],
            [wp.transform.location.y for wp in road])

    plt.show()
    '''
    
    intersection_nodes = {}
    road_edges = []
    for road in road_list:
        start = (road[0].transform.location.x, road[0].transform.location.y)
        stop = (road[-1].transform.location.x, road[-1].transform.location.y)
        
        xy = [(wp.transform.location.x, wp.transform.location.y) for wp in road]
        edge_data = {'geometry': LineString(xy)}

        road_edges.append((start, stop, edge_data))

        # Set nodes
        intersection_nodes[start] = {'x': start[0], 'y': start[1]}
        intersection_nodes[stop] = {'x': stop[0], 'y': stop[1]}

    
    graph = nx.MultiDiGraph()
    graph.add_nodes_from(intersection_nodes.items())
    graph.add_edges_from(road_edges)

    graph = nx.convert_node_labels_to_integers(graph)
    
    positions = {node: (data['x'], data['y'])
                 for node, data in graph.nodes(data=True)}
    # nx.draw_networkx(graph, pos=positions, arrows=True,
    #                     node_size=10, font_size=1)
    # plt.show()

   
    origin = (0, 0)
    origin_node = ox.get_nearest_node(graph, origin)
    print('origin:', origin_node)

    graph.graph['crs'] = None
    graph.graph['name'] = 'Carla Town'

    # TODO:
    out_dir =  os.path.abspath(os.path.join(os.path.dirname(
                                           os.path.abspath(__file__)),
                        '..', '..', 'navigation', 'route_planner', 'scripts'))
    nx.write_yaml(graph, os.path.join(out_dir, 'carla_map.yaml'))


    ox.plot_graph(graph)

if __name__ == '__main__':
    convert_to_2D_map()
