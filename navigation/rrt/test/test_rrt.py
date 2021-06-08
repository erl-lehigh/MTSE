import matplotlib.pyplot as plt
import sys
from rrt import RRTPlanner
from dubins import DubinsState
from dubins import dubins_isclose, position_distance as distance
from dubins import dubins_path_planning as vehicle_path
from dubins import DynamicDubinsVehicle
import numpy as np

if __name__ == '__main__':
    np.random.seed(1) # set random number generator seed to get repeatability
    
    
    rrt = RRTPlanner(DynamicDubinsVehicle(DubinsState(0, 0, 0, 0, 0)))
    def sample(goal):
        max_rand = 100
        min_rand = -100
        x, y = np.random.uniform(min_rand, max_rand, 2)
        return DubinsState(x, y, 0, 0, 0)
    rrt.sample = sample
    path = rrt.plan(DubinsState(x=29.988612382203414, y=-1,
                                yaw=0.082673490883941936,
                                v=10.0, omega=0.082673490883941936))
    print(path)
    
    
    plt.axis([-10, 40, -2, 4])
    plt.plot([-10, 60], [-0.15, -0.15], '--k' )
    xs = []
    ys = []
    for state in path:
        xs.append(state.x)
        ys.append(state.y)

    plt.plot(xs, ys)    
    plt.plot(0, 0, 'sr', label='vehicle')
    plt.text(0, 0, 'vehicle')
    plt.plot(29.988612382203414, -1, '--sr', label='goal point')
    plt.text(29.988612382203414, -1, 'goal point')
   
