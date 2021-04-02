'''
Route planning using Open Street Maps
'''

import itertools as it

import networkx as nx
import osmnx as ox
import matplotlib.pyplot as plt
import requests
import math
import shapely




class GoalPlanner(object):
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
    figure : tuple
        figure information
    ax : tuple
        axis information
    route_line : MatPlotLib plot
        The lines representing the route
    Methods
    -------
    get_route_coords(self, route):
        Takes each node along the route and returns their corresponding coordinates
    get_road_coords(self, route):
        Takes each edge and breaks it into nodes with linear connections and
        returns the nodes' coordinates
    get_route(self, origin, destination):
        Uses Dijkstra's algorithm to compute the shortest distance between
        the vehicles current location (origin) and a given destination.
    get_point_of_interest(self, destination, distance):
        Gets the coordinates of a point of interest (address)
    setup_plot(self):
        Displays the blank map.
    plot_route(self, route_coords):
        Plots the route onto the map.    
    update_plot(self):
        Updates the map with the new route.
    
    '''

    def __init__(self, current_location, orientation, route):
        '''
        RoutePlanner Constructor
        Parameters
        ----------
        current_location :  tuple
            the current location of the vehicle (x,y)
        orientation : tuple
            the direction of the vehicle (-1 is 360deg, 1 is 0deg, 0 is 180deg)
        graph : networkx multidigraph or tuple
            multidigraph or optionally (multidigraph, tuple)
         route :  tuple
        the route being traversed
        Returns
        -------
        None
        
        '''
        ref_path = list(route.coords)
        try:
            node1 = ref_path[0]
            node2 = ref_path[1]
        except (IndexError):
            print("Out of bounds!!")
            node1 = (1.0,1.0)
            node2 = (2.0,1.0)
        
        cn_x = node1[0]
        cn_y = node1[1]
        # Determines the difference between the current location and nearest x,y
        try:
            current_location[0]
            current_location[1]
        except (IndexError, TypeError):
            print("Out of bounds!!")
            current_location = tuple([0.0,0.0])

        dx = cn_x - current_location[0]
        dy = cn_y - current_location[1]
        # Calculates the angle, in radians, to the nearest node
        needed_orientaion = math.atan2(dy,dx)
        # Converts from radians to w
        w = (math.pi - needed_orientaion)/math.pi
        # If the nearest node is in front, its the goal, else, its the next node
        if (orientation - w < 1.0 and orientation - w > -1.0):
            self.goal_node = node1
        else:
            self.goal_node = node2


        '''
        # Gets the two neighbors of the nearest node
        pn = graph.neighbors(ox.get_nearest_node(graph, current_location))
        # Sets the previous and next nodes to the nearest node
        self.previous_node = pn[0]
        self.next_node = pn[1]
        # Gets the closest node
        self.closest_node = ox.get_nearest_node(graph, current_location)
        # Converts the node to x,y
        cn_x = self.graph.node[self.closest_node]['x']
        cn_y = self.graph.node[self.closest_node]['y']
        # Determines the difference between the current location and nearest x,y
        dx = cn_x - current_location.x
        dy = cn_y - current_location.y
        # Calculates the angle, in radians, to the nearest node
        needed_orientaion = math.atan2(dy,dx)
        # Converts from radians to w
        w = (math.pi - needed_orientaion)/math.pi
        # If the nearest node is in front, its the goal, else, its the next node
        if (orientation - w < 1.0 and orientation - w > -1.0):
            self.goal_node = self.closest_node
        else:
            self.goal_node = self.next_node
        '''



    def get_goal_node(self):
        return self.goal_node

    def get_route_coords(self, graph, route):
        '''
        Takes each node along the route and returns their corresponding coordinates
        Parameters
        ----------
        graph : networkx multidigraph or tuple
            multidigraph or optionally (multidigraph, tuple)
        route :  tuple
            the route you want the coordinates of
        Returns
        -------
        tuple
            the coordinates corresponding to the given route
        '''
        return [(self.graph.node[u]['x'], self.graph.node[u]['y']) 
            for u in route]

    def geocode(self, query):
        """
        Geocode a query string to (lat, lon) with the Nominatim geocoder.
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
        url_template = 'https://nominatim.openstreetmap.org/search?format=json&limit=1&q={}'
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
            raise Exception('Nominatim geocoder returned no results for query "{}"'.format(query))