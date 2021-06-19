'''
RRT Planner.
@author: Cristian-Ioan Vasile (cvasile@lehigh.edu)
'''

import sys
import itertools as it

import rospy

import numpy as np
import networkx as nx

from dubins import DubinsState
from dubins import dubins_isclose, position_distance as distance
from dubins import dubins_path_planning as vehicle_path
from dubins import DynamicDubinsVehicle


DBL_MAX = sys.float_info.max


class RRTPlanner(object):
    '''Implementation of RRT*.'''

    def __init__(self, vehicle, max_iterations=2000, gamma=40.0):
        '''Constructor
        '''
        self.vehicle = vehicle

        # initialize RRT tree
        self.g = nx.DiGraph()

        self.check_path = lambda path: True
        self.sample = None

        self.max_iterations = max_iterations
        self.gamma = gamma
        self.dist_threshold = rospy.get_param("dist_threshold", 10)
        self.angle_threshold = np.deg2rad(rospy.get_param("angle_threshold", 20)) 
        self.step_size = rospy.get_param("step_size", 10)

    def plan(self, goal):
        '''Generates a dynamically feasible path for the ego-car and associated
        speeds (linear, turning).
        Parameters
        ----------
        goal (DubinsState) - goal state to plan towards

        Returns
        -------
        path (list of DubinsState) - dynamically feasible path
        '''
        start = self.vehicle.current_state

        self.g.clear()
        self.g.add_node(start, cost_from_root=0)

        for k in range(self.max_iterations):
            q_rand = self.sample(goal)
            q_nearest = self.nearest(q_rand)

            new_node = self.steer(q_nearest, q_rand)
            q_new = new_node[0]
            if self.check_path(new_node[1]['trajectory']):
                near_states = self.near(q_new)
                new_node = self.choose_parent(new_node, near_states)

                if new_node is not None:
                    q_new, data_new = new_node
                    parent = data_new['parent']
                    total_cost = data_new['cost_from_root']
                    added_cost = data_new['cost_from_parent']
                    trajectory = data_new['trajectory']

                    self.g.add_node(q_new, cost_from_root=total_cost)
                    self.g.add_edge(parent, q_new, cost_from_parent=added_cost,
                                    trajectory=trajectory)

                    self.rewire(new_node, near_states)

        # save best plan

        self.best_state = self.get_best_state(goal)
        if self.best_state is None:
            path = [self.vehicle.current_state] * 2 # Stay in place
        else:
            path = self.get_trajectory_from_root(self.best_state)

        return path

    def get_best_state(self, goal):
        '''Returns the optimal state w.r.t. cost from root that is close to the
        target (goal) state.
        Parameters
        ----------
        goal (DubinsState) - goal state to plan towards
        Returns
        -------
        node (tuple) - pair of DubinsState and associated data from the RRT tree
        '''
        nodes = [(state, data) for state, data in self.g.nodes(True)
                    if dubins_isclose(state, goal,self.dist_threshold,
                                      self.angle_threshold)]
        if not nodes:
            return None
        node = min(nodes,
                   key=lambda x: distance(x[0], goal))
#                    key=lambda x: x[1]['cost_from_root']))
        return node[0]

    def get_trajectory_from_root(self, state):
        '''Returns the trajectory from the state at the root of the RRT tree to
        the state given state.  It assumes that the state is in the RRT tree.
        Parameters
        ----------
        state (DubinsState) - a state in the RRT tree
        Returns
        -------
        trajectory (list of DubinsState) - the trajectory from the root to the
            given state
        '''

        trajectory = [state]
        while self.g.pred[state]:
            parent = next(iter(self.g.predecessors(state)))
            from_parent = self.g[parent][state]['trajectory']
            trajectory = from_parent[:-1] + trajectory
            state = parent
        return trajectory

    def steer(self, start, end):
        '''Steers the vehicle from the start state towards the end state, and
        returns a node (the new state and it's associated data).
        Parameters
        ----------
        start (DubinsState) - state where the motion starts
        end (DubinsState) - state towards the vehicle is steered
        Returns
        -------
        node (tuple) - pair of DubinsState and associated data
        '''
        state, _, path, length = self.vehicle.drive(start, end)
        data = {
            'cost_from_parent': length,
            'cost_from_root': self.g.node[start]['cost_from_root'] + length,
            'parent': start,
            'trajectory': path
        }
        return state, data

    def choose_parent(self, node, near_states):
        '''Determine the best parent for the given node from the given set of
        of near states in the RRT tree.
        Parameters
        ---------
        node (DubinsState) - pair of DubinsState and associated data dict
        near_states (list of DubinsState) - state in the RRT tree near the given
            node
        Return
        ------
        new_node (tuple or None) the parent node (pair of DubinsState and
            associated data dict) for the given node.
        '''
        state = node[0]
        if not near_states:
            return None
        cost = DBL_MAX
        new_node = None
        for candidate in near_states:
            candidate_node = self.steer(candidate, state)
            q_new, data_new = candidate_node

            if q_new is not None:
                if self.check_path(data_new['trajectory']):
                    candidate_cost = data_new['cost_from_root']
                    if candidate_cost < cost:
                        new_node = candidate_node

        return new_node

    def nearest(self, q_rand):
        '''Returns the nearest state in the RRT tree.
        Parameters
        ----------
        q_rand (DubinsState) - the query state
        Returns
        -------
        (DubinsState) - the closes state in the RRT tree to `q_rand`
        '''
        return min(self.g.nodes(), key=lambda q: distance(q_rand, q))

    def near(self, q_rand):
        '''Returns all states from the tree that are within the RRT* radius from
        the random sample.
        Parameters
        ----------
        q_rand (DubinsState) - the query state
        Returns
        -------
        (list of DubinsState) - states from the RRT tree that are close to
            `q_rand`
        '''
        # Compute the ball radius
        n = self.g.number_of_nodes()
        # NOTE: Assumes planar workspace, i.e., dimension is 2
        r = self.gamma * np.sqrt(np.log(n + 1.0)/(n + 1.0))
        return [v for v in self.g.nodes() if distance(q_rand, v) <= r]

    def rewire(self, node, near_states):
        '''Re-configures the tree based on the newly added node if costs to
        nearby states can be improved.
        Parameters
        ----------
        node (pair DubinsState and dict) - state and associated data from the
            RRT tree
        near_states (list of DubinsState) - nearby states in the RRT tree whose
            costs may be improved by routing via the given `node`
        '''
        state = node[0]
        if near_states is None:
            return

        for candidate in near_states:
            if candidate == node[1]['parent']:
                continue

            data = self.g.node[candidate]

            candidate_node = self.steer(state, candidate)
            q_new, data_new = candidate_node

            if dubins_isclose(q_new, candidate, 0.01, 0.05):
                improved = (data_new['cost_from_root'] < data['cost_from_root'])

                if self.check_path(data_new['trajectory']) and improved:
                    # remove old parent
                    parent = next(iter(self.g.predecessors(candidate)))
                    self.g.remove_edge(parent, candidate)
                    # add new parent and update cost from root
                    data['cost_from_root'] = data_new['cost_from_root']
                    added_cost = data_new['cost_from_parent']
                    trajectory = data_new['trajectory']
                    self.g.add_edge(state, candidate,
                                    cost_from_parent=added_cost,
                                    trajectory=trajectory)


def main():
    print("start " + __file__)

    np.random.seed(1) # set random number generator seed to get repeatability

    rrt = RRTPlanner(DynamicDubinsVehicle(DubinsState(0, 0, 0, 0, 0)))

    def sample(goal):
        max_rand = 100
        min_rand = -100
        x, y = np.random.uniform(min_rand, max_rand, 2)
        return DubinsState(x, y, 0, 0, 0)

    rrt.sample = sample

    path = rrt.plan(DubinsState(x=29.988612382203414, y=-4.3868679359233935,
                                yaw=0.082673490883941936,
                                v=10.0, omega=0.082673490883941936))
    print(path)

if __name__ == '__main__':
    main()
