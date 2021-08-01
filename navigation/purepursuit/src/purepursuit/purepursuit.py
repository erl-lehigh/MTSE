#! /usr/bin/env python

import numpy as np
from shapely.geometry import Point


class PurePursuit:
    '''
    A class to represent a PurePursuit path tracker.

    Attributes
    ----------
    wheelbase : float
        specifies the distance between the midpoint of the read and front axle
    lookahead : float
        specifies the distance to the target position along the tracked path
    path : Path
        path to be tracked by the vehicle
    speed : float
        specifies the forward speed of the vehicle
    vehicle_position : Point
        position of vehicle
    vehicle_orientation : float
        orientation of vehicle

    Methods
    -------

    set_vehicle_pose(vehicle_pose)
        sets the vehicles position and the orientation if the parameter
        is not none
    closest_point()
        Returns the computed closest point on the path from the midpoint
        of the rear axle of the vehicle.
    future_point()
        Returns the computed future point on the path for the vehicle to
        keep track of.
    update_lookahead(self, v_cmd, lookahead_min, lookahead_max,
             lower_threshold_v, upper_threshold_v, lookahead_gain)
        Updates the lookahead based on the most recent commanded speed
    get_lookahead()
        Returns the most up-to-date lookahead distance
    compute_speed()
        Returns the computed speed of the vehicle.
    vehicle_front_point()
        Returns the computed front axle midpoint of vehicle.
    compute_steering_angle()
        Returns the computed steering angle of the vehicle based on where the
        vehicle is in relation to the path that it is following.
    compute_angular_speed()
        Returns the computed angular speed of the vehicle.
    compute_curvature()
        Returns the computed curvature.
    compute_turning_radius()
        Returns the computed turning radius.
    construct_path()
        This is just a smaller version at the purepursuit test can be run.
        Uses LineString as the way to store the path in the test.
    '''

    # constructor
    def __init__(self, wheelbase, lookahead_min,
                 lookahead_max, lower_threshold_v, upper_threshold_v,
                 lookahead_gain, speed=None, vehicle_pose=None,
                 path=None):
        '''
        Initializes the PurePursuit object by passing input parameters
        (vehicle wheelbase, lookahead distance, speed, position, and path) and
        does geometric computation and then outputs the goal point, speed, and
        steering angle for the vehicle to stay on the path to be followed based
        on the aforementioned inputs.
        Parameters
        ----------
        wheelbase : float
            specifies the distance between the midpoint of the read and
            front axle
        lookahead : float
            specifies the lookahead distance to the path
        lookahead_min : float
            the minimum the lookahead can be (set in configs)
        lookahead_max : float
            the maximum the lookahead can be (set in configs)
        lower_velocity_threshold : float
            the lower bound speed, under this speed uses the minimum
            lookahead_min distance
        upper_velocity_threshold : float
            the upper bound speed, above this speed uses the maximum
            lookahead_max distance
        lookahead_gain : float
            the scalar to multiply to get lookahead by multiplying with
            v_cmd when the lookahead is not outside the max or min.
        path : Path
            path to be tracked for the vehicle
        speed : float
            specifies the speed of the vehicle
        vehicle_pose : tuple
            pose of the vehicle in tuple form (x, y, theta)
        '''

        # distance between front and rear axles
        self.wheelbase = wheelbase
        # look ahead distance to the
        self.lookahead = lookahead_min
        self.lookahead_min = lookahead_min
        self.lookahead_max = lookahead_max
        self.lower_threshold_v = lower_threshold_v
        self.upper_threshold_v = upper_threshold_v
        self.lookahead_gain = lookahead_gain
        self.path = path
        self.speed = speed
        self.set_vehicle_pose(vehicle_pose)

    def set_vehicle_pose(self, vehicle_pose):
        '''
        Sets the vehicle position and the orientation
        if `vehicle_pose` is not
        `None`.

        Parameters
        ----------
        vehicle_pose : tuple
            pose of the vehicle as a 3-tuple (x, y, yaw)

        Returns
        -------
        None
        '''
        if vehicle_pose is None:
            self.vehicle_position = None
            self.vehicle_orientation = None
        else:
            self.vehicle_position = Point(vehicle_pose[:2])
            self.vehicle_orientation = vehicle_pose[2]

    def closest_point(self):
        '''
        Returns the computed closest point on the path from the
        midpoint of the rear axle of the vehicle.

        Parameters
        ----------
        None

        Returns
        -------
        shapely.geometry.Point
            computed closest point
        '''
        path_length = self.path.project(self.vehicle_position)
        return self.path.interpolate(path_length)

    def future_point(self):
        '''
        Returns the computed future point on the path for the
        vehicle to keep track of.

        NOTE: When either the path or vehicle_positions are not set, then `None`
        is returned.

        Parameters
        ----------
        None

        Returns
        -------
        shapely.geometry.Point
            computed future point
        '''
        if self.path == None:
            return Point(0.0, 0.0)
        closest_dist = 0
        if self.vehicle_position != None:
            closest_dist = min(
                        self.vehicle_position.distance(self.closest_point()),
                        self.lookahead)
            dist_on_path = (self.lookahead ** 2 - closest_dist ** 2) ** 0.5
            arc_dist = self.path.project(self.vehicle_position)
            return self.path.interpolate(arc_dist + dist_on_path)
        return None

    def update_lookahead(self, v_cmd,
                         lookahead_min, lookahead_max,
                         lower_threshold_v, upper_threshold_v,
                         lookahead_gain):
        '''
        Updates the lookahead based on the most recent commanded speed

        Parameters
        ----------
        float
            v_cmd - the commanded speed of the vehicle/path
        float
            lookahead_min - the minimum the lookahead can be (set in configs)
        float
            lookahead_max - the maximum the lookahead can be (set in configs)
        float
            lower_threshold_v - the lower bound speed, under this speed uses
                                the minimum lookahead_min distance
        float
            upper_threshold_v - the upper bound speed, above this speed
                                uses the maximum lookahead_max distance
        float
            lookahead_gain - the scalar multiplied by v_cmd to get the
                             lookahead when the value is not set to max or min
        Return
        ------
        none
        '''
        if (v_cmd < lower_threshold_v):
            self.lookahead = lookahead_min
        elif (lower_threshold_v <= v_cmd and v_cmd < upper_threshold_v):
            self.lookahead = lookahead_gain*v_cmd
        else:
            self.lookahead = lookahead_max

    def get_lookahead(self):
        '''
        Returns the most up-to-date lookahead distance

        Parameters
        ----------
        None

        Return
        ------
        float
            lookahead distance
        '''
        return self.lookahead

    def compute_speed(self):
        '''
        Returns the computed speed of the vehicle.

        Parameters
        ----------
        None

        Returns
        -------
        float
            vehicle speed
        '''
        return self.speed

    def vehicle_front_point(self):
        '''
        Returns the computed front axle midpoint of vehicle.

        Parameters
        ----------
        None

        Returns
        -------
        shapely.geometry.Point
            vehicle front axle midpoint
        '''
        direction = np.array((np.cos(self.vehicle_orientation),
                              np.sin(self.vehicle_orientation)))
        return Point(self.vehicle_position + self.wheelbase * direction)

    def compute_steering_angle(self):
        '''
        Returns the computed steering angle of the vehicle based on where the
        vehicle is in relation to the path that it is following.

        Parameters
        ----------
        None

        Returns
        -------
        float
            computed steering angle
        '''
        lookahead_point = np.array(self.future_point()) - self.vehicle_position
        line_of_sight_angle = np.arctan2(lookahead_point[1], lookahead_point[0])
        eta = line_of_sight_angle - self.vehicle_orientation
        return np.arctan(2 * self.wheelbase * np.sin(eta) / self.lookahead)
        # There was a negative sign above, but it was returning the opposite
        # commands.

    def compute_angular_speed(self):
        '''
        Returns the computed angular speed of the vehicle.

        Parameters
        ----------
        None

        Returns
        -------
        float
            computed speed
        '''
        return ((self.speed
               * np.tan(self.compute_steering_angle()))
               / self.wheelbase)

    def compute_curvature(self):
        '''
        Returns the computed curvature.

        Parameters
        ----------
        None

        Returns
        -------
        float
            computed curvature
        '''
        if self.future_point() == None or self.vehicle_position == None:
            return .5
        lookahead_point = np.array(self.future_point()) - self.vehicle_position
        line_of_sight_angle = np.arctan2(lookahead_point[1], lookahead_point[0])
        eta = line_of_sight_angle - self.vehicle_orientation
        return 2 * np.abs(np.sin(eta)) / self.lookahead

    def compute_turning_radius(self):
        '''
        Returns the computed turning radius.

        Parameters
        ----------
        None

        Returns
        -------
        float
            computed turning radius
        '''
        return 1.0 / self.compute_curvature()

    def construct_path(self):
        '''
        This is just a smaller version at the purepursuit test can be run.
        Uses LineString as the way to store the path in the test.
        '''
        if self.path is None:
            return [], []
        else:
            return self.path.xy
