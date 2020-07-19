'''
TODO: docstring
'''


import numpy as np
from shapely.geometry import Point


class PurePursuit:
    '''TODO: docstring
    '''

    # constructor
    def __init__(self, wheelbase, lookahead, speed=None, vehicle_pose=None,
                 path=None):
        '''TODO: docstring

        vehicle_pose = (x, y, theta)
        '''
        # distance between front and rear axles
        self.wheelbase = wheelbase
        # look ahead distance to the
        self.lookahead = lookahead

        self.path = path
        self.speed = speed
        self.set_vehicle_pose(vehicle_pose)

    def set_vehicle_pose(self, vehicle_pose):
        '''TODO: docstring
        '''
        if vehicle_pose is None:
            self.vehicle_position = None
            self.vehicle_orientation = None
        else:
            self.vehicle_position = Point(vehicle_pose[:2])
            self.vehicle_orientation = vehicle_pose[2]

    def closest_point(self):
        '''TODO: docstring
        '''
        path_length = self.path.project(self.vehicle_position)
        return self.path.interpolate(path_length)

    def future_point(self):
        '''TODO: docstring
        '''
        closest_dist = min(self.vehicle_position.distance(self.closest_point()),
                           self.lookahead)
        dist_on_path = (self.lookahead ** 2 - closest_dist ** 2) ** 0.5
        return self.path.interpolate(dist_on_path)

    def compute_speed(self):
        '''TODO: docstring
        '''
        return self.speed

    def vehicle_front_point(self):
        '''TODO: docstring
        '''
        direction = np.array((np.cos(self.vehicle_orientation),
                              np.sin(self.vehicle_orientation)))
        return Point(self.vehicle_position + self.wheelbase * direction)

    def compute_steering_angle(self):
        '''TODO: docstring
        '''
        lookahead_point = np.array(self.future_point()) - self.vehicle_position
        line_of_sight_angle = np.arctan2(lookahead_point[1], lookahead_point[0])
        eta = line_of_sight_angle - self.vehicle_orientation
        return -np.arctan(2 * self.wheelbase * np.sin(eta) / self.lookahead)

    def compute_angular_speed(self):
        '''TODO: docstring
        '''
        return self.speed * tan(self.compute_steering_angle()) / self.wheelbase

    def compute_curvature(self):
        '''TODO: docstring
        '''
        lookahead_point = np.array(self.future_point()) - self.vehicle_position
        line_of_sight_angle = np.arctan2(lookahead_point[1], lookahead_point[0])
        eta = line_of_sight_angle - self.vehicle_orientation
        return 2 * np.abs(np.sin(eta)) / self.lookahead

    def compute_r(self):
        '''TODO: docstring
        '''
        return 1.0 / self.compute_curvature()
