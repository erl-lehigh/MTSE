This contains the python scripts used to do route planning (route planner) and to communicate the route to ROS (route planner node)
# Functionality

## Route Planning

The most useful functionality of this script is generating a route along either a .yaml map or a map from OSMNx. This is accomplished using Dijkstra's algorith to compute the shortest distance bewtween the vehicle's current location and a given destination. The route is published to the *route* topic and the path (the route containing only straight connections) is published to the *planned_path* topic.

## Plotting

The route planner package plots the map alongside the simulator. The planner can plot the route, current vehicle location, and pure-pursuits goal location in a red line, black diamond, and green diamond respectively.

# Implementaiton

The code is written in [python](https://www.python.org/) using [ROS](http://wiki.ros.org/) to publish and subscribe to topics. The map is simulated in [MatPlotLib](https://matplotlib.org/) or [Open Street Maps](https://geoffboeing.com/2016/11/osmnx-python-street-networks/). Path planning is done through [Network-x](https://networkx.github.io/).

# Nodes

## Publishers

### 'route'

[Path.msg](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Path.html)

### 'planned_path'

[Path.msg](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Path.html)

### 'planned_path_viz'

[Path.msg](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Path.html)

# Launch / Configuration Files

## route_planner.launch

Launches both the route planner and the route planner node.

## route_planner.yaml

Configuration file holding some startup information.

# Tests

Currently **None**. 




