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
import sys

from shapely.geometry import Point




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
        index = 0
        min_index = 0
        min_dist = sys.maxint
        # Locates the closest node to the current location on the path
        for pt in ref_path:
            if(self.dist_to_point(pt,current_location)<min_dist):
                min_dist = self.dist_to_point(pt,current_location)
                min_index = index
            index = index + 1
        
        dx = ref_path[min_index][0] - current_location.x
        dy = ref_path[min_index][1] - current_location.y
        # Calculates the angle, in radians, to the nearest node
        needed_orientation = math.atan2(dy,dx)

        # Make both orientations positive
        if(needed_orientation < 0):
            needed_orientation = needed_orientation + 2 * math.pi
        if(orientation < 0):
            orientation = orientation + 2 * math.pi

        # If the point is in front of the vehicle, it is the goal point
        # Otherwise, the next point is the goal point    
        if(needed_orientation - orientation < 2 and
                 needed_orientation - orientation > -2):
            self.goal_node = Point(ref_path[min_index][0],
                ref_path[min_index][1])
        else:
            self.goal_node = Point(ref_path[min_index+1][0],
                ref_path[min_index+1][1])
            

    def dist_to_point(self, pt, current_location):
        return math.sqrt(math.pow((pt[0]-current_location.x),2)+
            math.pow((pt[1]-current_location.y),2))
        

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