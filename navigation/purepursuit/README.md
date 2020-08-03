This module contains the implementation of pure pursuit, which is a path tracking algorithm, using python scripts. (More info: https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)

# Functionality

The pure pursuit algorithm works by geometrically computing the goal points along the path that the vehicle needs to steer towards to follow the path. It also computes the required vehicle speed, angular velocity, and steering angle that is needed to follow the path.

# Implementation

All the geometric computation of algorithm is written in [Python] (https://www.python.org/) using mathematical libraries like [Shapely] (https://pypi.org/project/Shapely/) and [NumPy] (https://numpy.org/). The algorithm utilizes [ROS] (http://wiki.ros.org/) to publish and subscribe to topics for it to function.

# Nodes

## Publishers

### command_pub
[AckermannDrive.msg] (http://docs.ros.org/jade/api/ackermann_msgs/html/msg/AckermannDrive.html)

### target_pub
[Pose.msg] (http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html)

## Subscribers

### path_sub
[Path.msg] (http://docs.ros.org/melodic/api/nav_msgs/html/msg/Path.html)

# Launch / Configuration Files

## purepursuit.launch

Launches the purepursuit algorithm with test path (set of points).

## purepursuit.yaml

Configuration file holding some startup information.

# Tests

## purepursuit_node.test

Holds a list of nodes to be launched to pass a static dummy path for testing.
