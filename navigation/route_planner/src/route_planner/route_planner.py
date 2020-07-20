'''
The Route Planner Object contains all of the information needed to create a route
and is called by the route_planner_node to plan the route from a position to a
given destination.
'''

import itertools as it

import networkx as nx
import osmnx as ox
import matplotlib.pyplot as plt



class RoutePlanner(object):
    '''
    Methods needed to plan a route from a given postion to a destination on a map
    '''

    def __init__(self, address, distance, network_type):
        '''
        Constructor that takes the central address, a radial distance away
        from that address and what type of paths to display in order to
        generate a map of all of the roads.
        '''
        self.address = address
        self.distance = distance
        self.network_type = network_type

        # Gets all the roads a distance away on which can be driven
        self.g = ox.graph_from_address(self.address, distance=self.distance,
                                       network_type=self.network_type)

    def get_route_coords(self, route):
        '''
        Takes each node along the route and returns their corresponding coordinates
        '''
        return [(self.g.node[u]['x'], self.g.node[u]['y']) for u in route]

    def get_road_coords(self, route):
        '''
        Takes each edge and breaks it into nodes with linear connections and
        returns the nodes' coordinates
        '''
        # Concatenate all road geometries
        return list(it.chain(*[self.g.get_edge_data(u, v)[0]['geometry'].coords
                               for u, v in zip(route, route[1:])]))

    def get_route(self, origin, destination):
        '''
        Uses Dijkstra's algorithm to compute the shortest distance between
        the vehicles current location (origin) and a given destination.
        '''
        # Find the nearest intersection to the current location
        origin_node = ox.get_nearest_node(self.g, origin)
        # Find the nearest intersection to the destination point
        destination_node = ox.get_nearest_node(self.g, destination)
        # Get the shortest path from the current location to the destination
        return nx.shortest_path(self.g, origin_node, destination_node)

    def setup_plot(self):
        '''
        Displays the blank map.
        '''
        self.figure, self.ax = ox.plot_graph(self.g, show=False, close=False)
        # Create route line
        self.route_line, = plt.plot([], [], '-', color='red', linewidth=6)
        plt.draw() # Draw canvas and mark as changed
        plt.pause(0.001) # Display changes

    def plot_route(self, route_coords):
        '''
        Plots the route onto the blank map.

        NOTE: Due to the use of osmnx version 0.9, we need to manually draw the
        route.
        '''
        # # Graphs the figure
        # fig, ax = ox.plot_graph_route(self.g, route, route_linewidth=6,
        #                               node_size=0, bgcolor='k')
        self.route_line.set_data(zip(*route_coords))

    def update_plot(self):
        '''
        Updates the map with the new route.
        '''
        plt.draw()
        plt.pause(0.001)
