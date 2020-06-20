'''
TODO:
'''

import networkx as nx


class RoutePlanner(object):
    '''TODO:
    '''

    def __init__(self, g, initial):
        '''
        TODO:
        '''

        self.g = g
        self.initial = initial
        self.current = self.initial

    def get_route(self, destination):
        '''TODO:
        '''
        return nx.shortest_path(g, source=self.current, target=destination)
