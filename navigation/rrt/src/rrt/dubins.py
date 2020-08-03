'''
Dubins path planner.

@author: Cristian-Ioan Vasile (cvasile@lehigh.edu)

Based on code by Atsushi Sakai (@Atsushi_twi)

Created on Mar 14, 2019
'''

from collections import namedtuple

import math
import numpy as np
from scipy.spatial import cKDTree as kdtree
from shapely.geometry import Polygon
import shapely.affinity as sa


DubinsState = namedtuple('DubinsState',
                         field_names=('x', 'y', 'yaw', 'v', 'omega'))


def mod2pi(theta):
    return theta - 2.0 * math.pi * math.floor(theta / 2.0 / math.pi)


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


LSL, RSR, LSR, RSL, LRL, RLR = range(6)
modes = [LSL, RSR, LSR, RSL, LRL, RLR]
mode_names = ['LSL', 'RSR', 'LSR', 'RSL', 'LRL', 'RLR']


def init_compute(alpha, beta):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)
    return sa, sb, ca, cb, c_ab


def lsl_path(alpha, beta, d):
    sa, sb, ca, cb, c_ab = init_compute(alpha, beta)

    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb))
    if p_squared < 0:
        return None, None, None, LSL
    theta = math.atan2(cb - ca, d + sa - sb)
    t = mod2pi(-alpha + theta)
    p = math.sqrt(p_squared)
    q = mod2pi(beta - theta)

    return t, p, q, LSL


def rsr_path(alpha, beta, d):
    sa, sb, ca, cb, c_ab = init_compute(alpha, beta)

    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa))
    if p_squared < 0:
        return None, None, None, RSR
    theta = math.atan2(ca - cb, d - sa + sb)
    t = mod2pi(alpha - theta)
    p = math.sqrt(p_squared)
    q = mod2pi(-beta + theta)

    return t, p, q, RSR


def lsr_path(alpha, beta, d):
    sa, sb, ca, cb, c_ab = init_compute(alpha, beta)

    p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb))
    if p_squared < 0:
        return None, None, None, LSR
    p = math.sqrt(p_squared)
    theta = math.atan2((-ca - cb), (d + sa + sb)) - math.atan2(-2.0, p)
    t = mod2pi(-alpha + theta)
    q = mod2pi(-beta + theta)

    return t, p, q, LSR


def rsl_path(alpha, beta, d):
    sa, sb, ca, cb, c_ab = init_compute(alpha, beta)

    p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb))
    if p_squared < 0:
        return None, None, None, RSL
    p = math.sqrt(p_squared)
    theta = math.atan2((ca + cb), (d - sa - sb)) - math.atan2(2.0, p)
    t = mod2pi(alpha - theta)
    q = mod2pi(beta - theta)

    return t, p, q, RSL


def rlr_path(alpha, beta, d):
    sa, sb, ca, cb, c_ab = init_compute(alpha, beta)

    tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0
    if abs(tmp_rlr) > 1.0:
        return None, None, None, RLR

    p = mod2pi(2 * math.pi - math.acos(tmp_rlr))
    theta = math.atan2(ca - cb, d - sa + sb)
    t = mod2pi(alpha - theta + p / 2.0)
    q = mod2pi(alpha - beta - t + p)
    return t, p, q, RLR


def lrl_path(alpha, beta, d):
    sa, sb, ca, cb, c_ab = init_compute(alpha, beta)

    tmp_lrl = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (- sa + sb)) / 8.0
    if abs(tmp_lrl) > 1.0:
        return None, None, None, LRL
    p = mod2pi(2 * math.pi - math.acos(tmp_lrl))
    theta = math.atan2(ca - cb, d + sa - sb)
    t = mod2pi(-alpha - theta + p / 2.0)
    q = mod2pi(beta - alpha - t + p)

    return t, p, q, LRL


def dubins_path_length(path):
    t, p, q, _ = path
    if t is None:
        return float('inf')
    return abs(t) + abs(p) + abs(q)


def straight_path(start, length, step=0.2, atol=1e-6):
    steps_dist = int(length / step) * step
    if np.isclose(length, steps_dist, rtol=0, atol=atol):
        steps = np.arange(0, steps_dist, step, dtype=np.float)
    else:
        steps = np.arange(0, steps_dist + step, step, dtype=np.float)
    if not len(steps):
        return []
    steps[-1] = length
    xy_steps = np.zeros((len(steps), 2))
    xy_steps[:, 0] = start.x + np.cos(start.yaw) * steps
    xy_steps[:, 1] = start.y + np.sin(start.yaw) * steps
    assert np.isclose(length, np.sqrt(np.sum((xy_steps[0] - xy_steps[-1])**2)),
                      rtol=0, atol=1e-10)
    return [DubinsState(x, y, start.yaw, 0, 0) for x, y in xy_steps]


def arc_path(start, radius, angle_diff, direction='left', step=0.05, atol=1e-6):
    if direction == 'right':
        angle_diff = -angle_diff
        radius = -radius
        step = -step
    steps_angle = int(angle_diff/step) * step
    if np.isclose(angle_diff, steps_angle, rtol=0, atol=atol):
        steps = np.arange(0, steps_angle, step, dtype=np.float)
    else:
        steps = np.arange(0, steps_angle + step, step, dtype=np.float)
    if not len(steps):
        return []
    steps[-1] = angle_diff
    steps += start.yaw - np.pi/2
    # Compute center of rotation
    x_center = start.x - np.sin(start.yaw) * radius
    y_center = start.y + np.cos(start.yaw) * radius
    # Generate poses
    pose_steps= np.zeros((len(steps), 3))
    pose_steps[:, 0] = x_center + np.cos(steps) * radius
    pose_steps[:, 1] = y_center + np.sin(steps) * radius
    pose_steps[:, 2] = steps + np.pi/2

    if len(steps) > 1:
        assert np.all(np.isclose(pose_steps[0], (start.x, start.y, start.yaw),
                                 rtol=0, atol=1e-8)), (pose_steps[0], start)
    return [DubinsState(x, y, pi_2_pi(yaw), 0, 0) for x, y, yaw in pose_steps]


def dubins_path(start, curve, mode, radius, dist_step, turn_step, tolerance):
    '''
    Generates the Dubins path from curve parameters.

    Parameters
    ----------
    - start - start pose
    - curve - 3-tuple of curve parameters
    - mode - the mode used to generate the curve
    - radius - turning radius
    - dist_step - step size for forward motion [m]
    - turn_step - step size for turning motion [rad]
    - tolerance - parameter tolerance

    Returns
    -------
    - path - list of Dubins states

    '''
    path = []
    for mode_name, parameter in zip(mode_names[mode], curve):
        if parameter < tolerance:
            continue

        start = path[-1] if path else start
        if mode_name == 'S':
            path += straight_path(start, parameter * radius, dist_step)
        elif mode_name == 'L':
            path += arc_path(start, radius, parameter, step=turn_step)
        elif mode_name == 'R':
            path += arc_path(start, radius, parameter, 'right', step=turn_step)

    return path


def dubins_path_planning(start, end, radius, dist_step=0.05, turn_step=0.01745,
                         tolerance=1e-10):
    '''
    Dubins path plannner.

    Parameters
    ----------
    - start - start pose
    - end - end pose
    - radius - turning radius [m]
    - dist_step - step size for forward motion [m]
    - turn_step - step size for turning motion [rad]
    - tolerance - parameter tolerance

    Return
    ------
    - path
    - mode
    - length

    '''
    # Translate to origin
    dx = end.x - start.x
    dy = end.y - start.y
    theta = math.atan2(dy, dx)
    # Nomalize
    d = math.sqrt(dx ** 2.0 + dy ** 2.0) / radius
    alpha = mod2pi(start.yaw - theta)
    beta = mod2pi(end.yaw - theta)

    planners = [lsl_path, rsr_path, lsr_path, rsl_path, rlr_path, lrl_path]
    results = [planner(alpha, beta, d) for planner in planners]
    t, p, q, mode = min(results, key=dubins_path_length)
    clen = dubins_path_length((t, p, q, mode))

    # Generate SE(2) path
    path = dubins_path(start, (t, p, q), mode, radius, dist_step, turn_step,
                       tolerance)
    end_path = path[-1]
    assert np.all(np.isclose([end.x, end.y, np.sin(end.yaw)],
                             [end_path.x, end_path.y, np.sin(end_path.yaw)],
                             rtol=0, atol=1e-3)), (end, end_path)

    return path, mode, clen

def position_distance(state1, state2):
        '''Returns the distance between two states as the Euclidean distance
        between their locations.
        Parameters
        ---------
        a (DubinsState)
        b (DubinsState)
        Return
        ------
        distance (float) the Euclidean distance between the locations associate
            with the two states.
        '''
        dx = state1.x - state2.x
        dy = state1.y - state2.y
        return np.sqrt(dx * dx + dy * dy)

def dubins_isclose(state1, state2, dtol=0.01, atol=0.05):
    '''Determines if two states are close in terms of position and
    orientation.
    Parameters
    ----------
    state1 (DubinsState)
    state2 (DubinsState)
    dtol (float, default=0.01) distance tolerance
    atol (float, default=0.05) angle tolerance in radians
    Returns
    -------
    (bool) - whether the two Dubins states are close
    '''
    return (position_distance(state1, state2) <= dtol
            and np.abs(pi_2_pi(state1.yaw - state2.yaw)) <= atol)


class DynamicDubinsVehicle(object):
    '''
    TODO: classdocs
    '''

    def __init__(self, initial_state, sensor=None, footprint=None):
        '''
        TODO: Constructor

        ..math::

            \dot{x} = v \cos(\theta)
            \dot{y} = v \sin(\theta)
            \dot{\theta} = \omega
            \dot{v} = a

        '''
        self.initial_state = initial_state
        self.current_state = self.initial_state
        self.sensor = sensor

        self.nominal_speed = 1.0 # TODO: not sure if needed
        self.radius = 1.5 #TODO: not sure if needed

        self.time_step = 1
        self.minimum_speed = 0.05
        self.maximum_speed = 15.0
        self.maximum_turning = np.pi / (6 * self.time_step)

        self.speed_resolution = 20
        self.turn_resolution = 20
        self.path_resolution = 10

        self.initialize()

        if footprint:
            self.footprint = footprint
        else:
            self.footprint = Polygon(((-2.5, -1), (2.6, -1),
                                      (2.6, 1), (-2.5, 1)))

    def initialize(self):
        '''TODO:
        '''
        size = self.speed_resolution * self.turn_resolution
        self.inputs = np.zeros((size, 2), dtype=np.float)
        points = np.zeros((size, 2), dtype=np.float)
        self.paths = []

        k = 0
        for v in np.linspace(self.minimum_speed, self.maximum_speed,
                             self.speed_resolution):
            for omega in np.linspace(-self.maximum_turning,
                                     self.maximum_turning,
                                     self.turn_resolution):
                time  = np.linspace(0, self.time_step, self.path_resolution)

                x = (v / omega) * np.sin(omega * time)
                y = (v / omega) * (1 - np.cos(omega * time))
                theta = time * omega
                self.paths.append(zip(x, y, theta))

                points[k, :] = (x[-1], y[-1])
                self.inputs[k, :] = (v, omega)

                k += 1

        self.tree = kdtree(points)

    def get_footprint(self, pose):
        '''TODO:
        '''
        transformed = sa.translate(self.footprint, pose.x, pose.y)
        transformed = sa.rotate(transformed, angle=pose.yaw, origin='centroid',
                                use_radians=True)
        return transformed

    def move(self, acceleration, steering, dt):
        '''Moves the vehicle in the environment.
        '''
        raise NotImplementedError

    def drive(self, start, end):
        '''Computes a path the from ``start'' towards ``end'' within
        ``time_step'' time.  It also returns the associated control inputs used
        to obtain the path.
        Parameters
        ----------
        start (DubinsState) - the state the vehicle should start driving from
        end (DubinsState) - the target state the vehicle should get closer to
        Return
        ------
        state (DubinsState)- the state the vehicle can drive to in ``time_step''
            seconds starting from ``start'' towards ``end''
        controls (np.ndarray of 2 floats) - the input forward and turning speeds
        path (list of DubinsState) - the path between ``start'' and ``state''
        length (float) - the length of the returned path
        '''
        # Compute rotation transform in start body frame
        c, s = np.cos(start.yaw), np.sin(start.yaw)
        R = np.array(((c, s), (-s, c)))
        origin = np.array([start.x, start.y])

        # Compute the target point in start body frame
        query = R.dot(np.array([end.x, end.y]) - origin)
        # Query reachability tableau
        _, index = self.tree.query(query, k=1)

        # Retrieve controls, path, state, and path length corresponding to
        # closest point. The path is computed in world frame
        controls = self.inputs[index]
        path = [DubinsState(start.x + x * c - y * s, start.y + x * s + y * c,
                            start.yaw + yaw, controls[0], controls[1])
                for x, y, yaw in self.paths[index]]
        state = path[-1]
        length = self.time_step * controls[0]
        return state, controls, path, length


def plot_arrow(state, length=1, width=0.5, fc='r', ec='k', marker='o'):
    x, y, yaw, _, _ = state
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              fc=fc, ec=ec, head_width=width, head_length=width)
    plt.plot(x, y, marker=marker, color=fc)

def show_dubins_tableau(query_point=(1, 0),
                        minimum_velocity=0.1, maximum_velocity=10,
                        velocity_resolution=20,
                        maximum_turning=np.pi/6, turning_resolution=20,
                        time_horizon=1, time_resolution=50,
                        figure_number=1, arrow_length=0.3, arrow_width=0.05):

    plt.figure(figure_number)
    plt.axis('equal')
    plt.grid(True)

    plt.title('Tableau for Dubins vehicle')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.plot([0], [0], 'ko')
    plt.arrow(0, 0, dx=arrow_length, dy=0, color='k')

    tableau_size = (velocity_resolution * turning_resolution, 2)
    inputs = np.zeros(tableau_size, dtype=np.float)
    points = np.zeros(tableau_size, dtype=np.float)
    time  = np.linspace(0, time_horizon, time_resolution)

    k = 0
    for v in np.linspace(minimum_velocity, maximum_velocity,
                         velocity_resolution):
        for angle in np.linspace(-maximum_turning, maximum_turning,
                                 turning_resolution):

            omega = angle / time_horizon

            x = (v / omega) * np.sin(omega * time)
            y = (v / omega) * (1 - np.cos(omega * time))
            plt.plot(x, y, '-', color='gray')
            plot_arrow((x[-1], y[-1], angle, v, omega), arrow_length,
                       arrow_width, 'r')

            points[k, :] = (x[-1], y[-1])
            inputs[k, :] = (v, omega)
            k += 1

    tree = kdtree(points)

    dd, ii = tree.query(query_point, k=1)

    plt.plot(query_point[0], query_point[1], 'bD')
    x, y = tree.data[ii]
    plt.plot([x], [y], 'kD')
    plt.plot([query_point[0], x], [query_point[1], y], 'k-')

    plt.grid()

    return dd, tree.data[ii], inputs[ii]

def test_vehicle_drive(start=None, end=None, figure_number=1, arrow_length=1,
                       arrow_width=0.5):
    if start is None:
        start = DubinsState(x=29.988612382203414, y=-4.3868679359233935,
                            yaw=0.082673490883941936,
                            v=10.0, omega=0.082673490883941936)
    if end is None:
        end = DubinsState(x=101.55, y=57.0, yaw=0.0, v=0, omega=0)

    veh = DynamicDubinsVehicle(start)

    state, controls, path, length = veh.drive(start, end)

    plt.figure(figure_number)
    plt.axis('equal')
    plt.grid(True)

    plt.title("Dubins vehicle drive method\n$v={:.3f}$, $\omega={:.3f}$,"
              " $\|p\|_2={}$".format(controls[0], controls[1], length))
    plt.xlabel('x')
    plt.ylabel('y')

    x, y, _, _, _ = zip(*path)
    plt.plot(x, y, 'b-')

    plot_arrow(start, arrow_length, arrow_width, 'b')
    plot_arrow(end, arrow_length, arrow_width, 'g')
    plot_arrow(state, arrow_length, arrow_width, 'r')

def test_vehicle_path(start=None, end=None, radius=1.0, figure_number=1,
                      arrow_length=1.0, arrow_width=0.5):

    plt.figure(figure_number)
    plt.axis('equal')
    plt.grid(True)

    plt.title('Dubins path planner')
    plt.xlabel('x')
    plt.ylabel('y')

    if start is None:
        start = DubinsState(x=0.0, y=0.0, yaw=np.deg2rad(0.0), v=0, omega=0)
    if end is None:
        end = DubinsState(x=0.0, y=2.0, yaw=np.deg2rad(45.0), v=0, omega=0)

    path, mode, _ = dubins_path_planning(start, end, radius)
    px, py, _, _, _ = zip(*path)

    plt.plot(px, py, 'o', label="final course " + mode_names[mode])
    plt.plot(px[0], py[0], marker='D', color='blue', label='Start')
    plt.plot(px[-1], py[-1], marker='x', color='green', label='Target')

    # plotting
    plot_arrow(start, arrow_length, arrow_width)
    plot_arrow(end, arrow_length, arrow_width)

    plt.legend()


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    show_dubins_tableau(figure_number=1)
    test_vehicle_drive(figure_number=2)
    test_vehicle_path(figure_number=3)
    plt.show()
