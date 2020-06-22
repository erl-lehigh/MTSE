from math import cos, sin, radians, atan, degrees, tan
from shapely.geometry import Point, LineString
import numpy as np
class PurePursuit:

    # data members / class attributes
    L = 1 # distance between front and rear axle
    lookahead = 4  # look ahead distance set to 4 unit

    # constructor
    def __init__(self, path, speed, vehicle_cords, theta):
        self.path = path
        self.speed = speed
        self.vehicle_cords = vehicle_cords
        self.theta = theta

    def construct_path(self):
        line = self.path
        x, y = line.xy
        return x, y

    def closest_point(self):
        d = (self.path).project(self.vehicle_cords)
        closest_pt = (self.path).interpolate(d)
        return closest_pt

    def future_point(self):
        closest_path = min(self.vehicle_cords.distance(self.closest_point()), self.lookahead)
        dist_on_path = (self.lookahead ** 2 - closest_path ** 2) ** 0.5
        future_pt = self.path.interpolate(dist_on_path)
        return future_pt

    def compute_speed(self):
        print(str(self.speed))

    def vehicle_front_pt(self):
        ft_x = self.vehicle_cords.x + self.L * cos(self.theta)
        ft_y = self.vehicle_cords.y + self.L * sin(self.theta)
        front_pt = Point(ft_x, ft_y)
        return front_pt

    # compute steering angle
    def compute_delta(self):
        ls1 = LineString([self.vehicle_cords, self.vehicle_front_pt()])
        ls2 = LineString([self.vehicle_cords, self.future_point()])
        eta = radians(self.compute_angle(ls1, ls2))
        delta = atan((2* self.L * sin(eta)) / self.lookahead)
        return (delta)

    # compute angular velocity
    def compute_omega(self):
        omega = (self.speed/self.L)*tan(self.compute_delta())
        return omega

    def compute_curvature(self):
        closest_path = min(self.vehicle_cords.distance(self.closest_point()), self.lookahead)
        curvature = 1/((self.lookahead ** 2) / (2 * closest_path))
        return curvature

    def compute_r(self):
        r = self.compute_curvature()
        return r

    def compute_angle(self, ls1, ls2):
        seg = np.array(ls2)
        seg = seg[1] - seg[0]
        angle_l2 = np.angle(complex(*(seg)), deg=True)
        seg_1 = np.array(ls1)
        seg_1 = seg_1[1] - seg_1[0]
        angle_l1 = np.angle(complex(*(seg_1)), deg=True)
        return abs(angle_l1 - angle_l2)





