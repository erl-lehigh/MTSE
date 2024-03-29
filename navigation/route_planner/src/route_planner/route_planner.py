'''
Route planning using Open Street Maps
'''

import itertools as it

import networkx as nx
import osmnx as ox
import matplotlib.pyplot as plt
import requests




class RoutePlanner(object):
    '''
    Route planner from a given position to a destination on a map
    Attributes
    ----------
    address :  string
        the address to geocode and use as the central point around which to
        construct the graph
    distance : int
        retain only those nodes within this many meters of the center of the
        graph
    network_type : string
        what type of street network to get
    g : networkx multidigraph or tuple
        multidigraph or optionally (multidigraph, tuple) representing the road
        network
    figure : tuple
        figure information
    ax : tuple
        axis information
    route_line : MatPlotLib line artist
        The lines representing the route

    Methods
    -------
    get_route_coords(self, route):
        Takes each node along the route and returns their corresponding
        coordinates.
    get_road_coords(self, route):
        Takes each edge and breaks it into nodes with linear connections and
        returns the nodes' coordinates.
    get_route(self, origin, destination):
        Uses Dijkstra's algorithm to compute the shortest route between
        the vehicles current location (origin) and a given destination.
    get_point_of_interest(self, destination, distance):
        Gets the coordinates of a point of interest (address).
    setup_plot(self):
        Displays the static map.
    plot_route(self, route_coords):
        Plots the route onto the map.
    update_plot(self):
        Updates the map with the new route.

    '''

    def __init__(self, address, distance, network_type):
        '''
        RoutePlanner Constructor.

        Parameters
        ----------
        address :  string
            the address to geocode and use as the central point around which to
            construct the graph
        distance : int
            retain only those nodes within this many meters of the center of the
            graph
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
        Takes each node along the route and returns their corresponding
        coordinates.

        Parameters
        ----------
        route :  tuple
            the route you want the coordinates of

        Returns
        -------
        tuple
            the coordinates corresponding to the given route
        '''
        return [(self.g.node[u]['x'], self.g.node[u]['y']) for u in route]

    def get_road_coords(self, route):
        '''
        Takes each edge and breaks it into nodes with linear connections and
        returns the nodes' coordinates.

        Parameters
        ----------
        route :  tuple
            the route you want the coordinates of

        Returns
        -------
        list
            the coordinates of all the roads along the route
        '''
        # Concatenate all road geometries
        return list(it.chain(*[self.g.get_edge_data(u, v)[0]['geometry'].coords
                               for u, v in zip(route, route[1:])]))

    def get_route(self, origin, destination):
        '''
        Uses Dijkstra's algorithm to compute the shortest route between
        the vehicles current location (origin) and a given destination.

        Parameters
        ----------
        origin :  point
            the location of the vehicle
        destination : point
            the desired destination point

        Returns
        -------
        route
            the route connecting the origin and the destination
        '''
        # Find the nearest intersection to the current location
        origin_node = ox.get_nearest_node(self.g, origin)
        # Find the nearest intersection to the destination point
        destination_node = ox.get_nearest_node(self.g, destination)
        # Get the shortest path from the current location to the destination
        return nx.shortest_path(self.g, origin_node, destination_node)

    def geocode(self, query):
        """
        Returns the geographical coordinates from an address.

        Parameters
        ----------
        query : string
            the query string to geocode

        Returns
        -------
        point : tuple
            the (lat, lon) coordinates returned by the geocoder
        """

        # send the query to the nominatim geocoder and parse the json response
        url_template = ('https://nominatim.openstreetmap.org/'
                        'search?format=json&limit=1&q={}')
        url = url_template.format(query)
        response = requests.get(url, timeout=60)
        results = response.json()

        # if results were returned, parse lat and long out of the result
        if len(results) > 0 and 'lat' in results[0] and 'lon' in results[0]:
            lat = float(results[0]['lat'])
            lon = float(results[0]['lon'])
            point = (lat, lon)
            return point
        else:
            raise Exception('Nominatim geocoder returned no'
                            ' results for query "{}"'.format(query))

    def setup_plot(self):
        '''
        Displays the static road map.

        Parameters
        ----------
        None

        Returns
        -------
        None
        '''
        self.figure, self.ax = ox.plot_graph(self.g, show=False, close=False)
        # Create route line
        self.route_line, = plt.plot([], [], '-', color='red', linewidth=6)
        plt.draw() # Draw canvas and mark as changed
        plt.pause(0.001) # Display changes

    def plot_route(self, route_coords):
        '''
        Plots the route onto the map.

        Parameters
        ----------
        route_coords : list
            the coordinates of each point along the route

        Returns
        -------
        None

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

        Parameters
        ----------
        None

        Returns
        -------
        None
        '''
        plt.draw()
        plt.pause(0.001)
