'''
Route planning using Open Street Maps
'''

import itertools as it

import networkx as nx
import osmnx as ox
import matplotlib.pyplot as plt



class RoutePlanner(object):
    '''
    Route planner from a given position to a destination on a map
    Attributes
    ----------
    address :  string
        the address to geocode and use as the central point around which to construct the graph
    distance : int
        retain only those nodes within this many meters of the center of the graph  
    network_type : string
        what type of street network to get
    g : networkx multidigraph or tuple
        multidigraph or optionally (multidigraph, tuple)
    TODO 

    Methods
    -------
    get_route_coords(self, route):
        Takes each node along the route and returns their corresponding coordinates
    get_road_coords(self, route):
        TODO
    get_route(self, origin, destination):
        TODO
    setup_plot(self):
        TODO
    plot_route(self, route_coords):
        TODO    
    update_plot(self):
        TODO
    
    '''

    def __init__(self, address, distance, network_type):
        '''
        RoutePlanner Constructor

        Parameters
        ----------
        address :  string
            the address to geocode and use as the central point around which to construct the graph
        distance : int
            retain only those nodes within this many meters of the center of the graph  
        network_type : string
            what type of street network to get 

        Returns
        -------
        None
        
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
