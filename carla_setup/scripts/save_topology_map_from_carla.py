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
    # carla_world = client.load_world('Town01')
    cmap = carla_world.get_map()

    ###Code Credit: YashBansod
    # Invert the x and y axes since we follow UE4 coordinates
    # plt.gca().invert_yaxis()
    # plt.gca().invert_xaxis()
    plt.margins(x=0.7, y=0)
    plt.axis('equal')

    topology = cmap.get_topology()
    waypoints = cmap.generate_waypoints(5.0)

    # road_list = []
    # for wp_pair in topology:
    #     current_wp = wp_pair[0]
    #     # Check if there is a road with no previus road, this can happen
    #     # in opendrive. Then just continue.
    #     if current_wp is None:
    #         continue
    #     # First waypoint on the road that goes from wp_pair[0] to wp_pair[1].
    #     current_road_id = current_wp.road_id
    #     wps_in_single_road = [current_wp]
    #     # While current_wp has the same road_id (has not arrived to next road).
    #     while current_wp.road_id == current_road_id:
    #         # Check for next waypoints in aprox distance.
    #         available_next_wps = current_wp.next(2.0)
    #         # If there is next waypoint/s?
    #         if available_next_wps:
    #             # We must take the first ([0]) element because next(dist) can
    #             # return multiple waypoints in intersections.
    #             current_wp = available_next_wps[0]
    #             wps_in_single_road.append(current_wp)
    #         else: # If there is no more waypoints we can stop searching for more.
    #             break
    #     road_list.append(wps_in_single_road)
    # '''
    # # Plot each road (on a different color by default)
    # for road in road_list:
    #     plt.plot(
    #         [wp.transform.location.x for wp in road],
    #         [wp.transform.location.y for wp in road])

    # plt.show()
    # '''
    
    intersection_nodes = {}
    road_edges = []

    # Negative y values to account for UE4 integration shift
    # for road in road_list:
        
    #     start = (round(road[0].transform.location.x, 1), -round(road[0].transform.location.y,1))
    #     stop = (round(road[-1].transform.location.x,1 ), -round(road[-1].transform.location.y,1))
        
    #     xy = [(round(wp.transform.location.x,1), 
    #         -round(wp.transform.location.y,1)) for wp in road]
    #     edge_data = {'geometry': LineString(xy)}

    #     road_edges.append((start, stop, edge_data))    

    #     # Set nodes
    #     intersection_nodes[start] = {'x': start[0], 'y': start[1]}
    #     intersection_nodes[stop] = {'x': stop[0], 'y': stop[1]}
    
    graph = nx.MultiGraph()
    # graph.add_nodes_from(intersection_nodes.items())
    # graph.add_edges_from(road_edges)

    # for waypoint in waypoints:
    #     intersection_nodes[waypoint] = {
    #         'x': waypoint.transform.location.x,
    #         'y': -waypoint.transform.location.y}
    
    # graph.add_nodes_from(intersection_nodes.items())
    xy_topology = list()
    for wp1, wp2 in topology:
        xy_topology.append(((wp1.transform.location.x, -wp1.transform.location.y),(wp2.transform.location.x, -wp2.transform.location.y)))
    print(xy_topology)

    graph.add_edges_from(xy_topology)
    for source, target in graph.edges():
        graph[source][target][0]['geometry'] = LineString(
            [(source[0], source[1]),
             (target[0], target[1])])
    for n1, n2 in list(graph.edges()):
        graph.node[n1]['x'] = n1[0]
        graph.node[n1]['y'] = n1[1]
        graph.node[n2]['x'] = n2[0]
        graph.node[n2]['y'] = n2[1]

    '''
    graph.add_edges_from(topology)
    for source, target in graph.edges():
        graph[source][target][0]['geometry'] = LineString(
            [(source.transform.location.x, -source.transform.location.y),
             (target.transform.location.x, -target.transform.location.y)])
    for n1, n2 in list(graph.edges()):
        graph.node[n1]['x'] = n1.transform.location.x
        graph.node[n1]['y'] = -n1.transform.location.y
        graph.node[n2]['x'] = n2.transform.location.x
        graph.node[n2]['y'] = -n2.transform.location.y
    '''
    # nx.set_edge_attributes(graph, edge_data)
    # print(graph.edges(data=True))
    # print(graph.nodes(data=True))

    # nodes_to_remove = list()
    # for node in graph:
    #     try:
    #         graph.nodes(data=True)[node]['x']
    #     except KeyError:
    #         nodes_to_remove.append(node)
    # graph.remove_nodes_from(nodes_to_remove)

    # print(graph.nodes(data=True))

    # print(next(nx.strongly_connected_components(graph)))

    graph = nx.convert_node_labels_to_integers(graph, 
        ordering="increasing degree")
    # print(graph.edges())
    # degree1_nodes = list()
    # for node in graph:
    #     if(graph.degree(graph)[node]==1):
    #         degree1_nodes.append(node)

    # graph.remove_nodes_from(degree1_nodes)

    degree0_nodes = list()
    for node in graph:
        if(graph.degree(graph)[node]==0):
            degree0_nodes.append(node)

    graph.remove_nodes_from(degree0_nodes)

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


    ox.plot_graph(graph, annotate=True)

if __name__ == '__main__':
    convert_to_2D_map()
