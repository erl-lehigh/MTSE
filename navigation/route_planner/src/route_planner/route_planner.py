'''
TODO: docstring
'''

import itertools as it

import networkx as nx
import osmnx as ox
import matplotlib.pyplot as plt



class RoutePlanner(object):
    '''TODO: docstring
    '''

    def __init__(self, address, distance, network_type):
        '''TODO: docstring
        '''
        self.address = address
        self.distance = distance
        self.network_type = network_type

        # Gets all the roads a distance away on which can be driven
        self.g = ox.graph_from_address(self.address, distance=self.distance,
                                       network_type=self.network_type)

    def get_route_coords(self, route):
        '''TODO: docstring
        '''
        return [(self.g.node[u]['x'], self.g.node[u]['y']) for u in route]

    def get_road_coords(self, route):
        '''TODO: docstring
        '''
        # Concatenate all road gemotries
        return list(it.chain(*[self.g.get_edge_data(u, v)[0]['geometry'].coords
                               for u, v in zip(route, route[1:])]))

    def get_route(self, origin, destination):
        '''TODO: docstring
        '''
        # Find the nearest intersection to the current location
        origin_node = ox.get_nearest_node(self.g, origin)
        # Find the nearest intersection to the destination point
        destination_node = ox.get_nearest_node(self.g, destination)
        # Get the shortest path from the current location to the destination
        return nx.shortest_path(self.g, origin_node, destination_node)

    def setup_plot(self):
        '''TODO: docstring
        '''
        self.figure, self.ax = ox.plot_graph(self.g, show=False, close=False)
        # Create route line
        self.route_line, = plt.plot([], [], '-', color='red', linewidth=6)
        plt.draw() # Draw canvas and mark as changed
        plt.pause(0.001) # Display changes

    def plot_route(self, route_coords):
        '''TODO: docstring

        NOTE: Due to the use of osmnx version 0.9, we need to manually draw the
        route.
        '''
        # # Graphs the figure
        # fig, ax = ox.plot_graph_route(self.g, route, route_linewidth=6,
        #                               node_size=0, bgcolor='k')
        self.route_line.set_data(zip(*route_coords))

    def update_plot(self):
        '''TODO: docstring
        '''
        plt.draw()
        plt.pause(0.001)
