'''
Pure Pursuit Algorithm for path tracking and speed/steerig angle computation
'''


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
    set_path(msg):
        generates a path LineString (to be tracked) from a set of position
        coordinates (pose)
    control_loop(event=None):
    '''

    # constructor
    def __init__(self, wheelbase, lookahead, speed=None, vehicle_pose=None,
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
            specifies the distance between the midpoint of the read and front
            axle
        lookahead : float
            specifies the lookahead distance to the path
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
        self.lookahead = lookahead

        self.path = path
        self.speed = speed
        self.set_vehicle_pose(vehicle_pose)

    def set_vehicle_pose(self, vehicle_pose):
        '''
        Sets the vehicle position and the orientation if vehicle_pose is not
        None.
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
        Returns the computed closest point on the path from the midpoint of the
        rear axle of the vehicle.
        Parameters
        ----------
        None
        Returns
        -------
        self.path.interpolate(path_length) : shapely.geometry.Point
            computed closest point
        '''
        path_length = self.path.project(self.vehicle_position)
        return self.path.interpolate(path_length)

    def future_point(self):
        '''
        Returns the computed future point on the path for the vehicle to keep
        track of.
        Parameters
        ----------
        None
        Returns
        -------
        self.path.interpolate(dist_on_path) : shapely.geometry.Point
            computed future point
        '''
        closest_dist = min(self.vehicle_position.distance(self.closest_point()),
                           self.lookahead)
        dist_on_path = (self.lookahead ** 2 - closest_dist ** 2) ** 0.5
        return self.path.interpolate(closest_dist + self.lookahead)

    def compute_speed(self):
        '''
        Returns the computed speed of the vehicle.
        Parameters
        ----------
        None
        Returns
        -------
        vehicle speed : float
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
        vehicle front point : shapely.geometry.Point
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
        steering angle : float
            computed steering angle
        '''
        lookahead_point = np.array(self.future_point()) - self.vehicle_position
        line_of_sight_angle = np.arctan2(lookahead_point[1], lookahead_point[0])
        eta = line_of_sight_angle - self.vehicle_orientation
        return np.arctan(2 * self.wheelbase * np.sin(eta) / self.lookahead)

    def compute_angular_speed(self):
        '''
        Returns the computed angular speed of the vehicle.
        Parameters
        ----------
        None
        Returns
        -------
        angular speed : float
            computed speed
        '''
        return self.speed * tan(self.compute_steering_angle()) / self.wheelbase

    def compute_curvature(self):
        '''
        Returns the computed curvature.
        Parameters
        ----------
        None
        Returns
        -------
        curvature : float
            computed curvature
        '''
        lookahead_point = np.array(self.future_point()) - self.vehicle_position
        line_of_sight_angle = np.arctan2(lookahead_point[1], lookahead_point[0])
        eta = line_of_sight_angle - self.vehicle_orientation
        return 2 * np.abs(np.sin(eta)) / self.lookahead

    def compute_turning_radius(self):
        '''
        Returns the computed r (radius).
        Parameters
        ----------
        None
        Returns
        -------
        turning radius : float
            computed turning radius
        '''
        return 1.0 / self.compute_curvature()