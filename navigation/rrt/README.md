# RRT
Implementation of RRT* for Dubins vehicles.

## Running an isolated instance
`source devel/setup.bash`
- Start the RRT node
  - `roslaunch rrt rrt_node.launch`
- Start the static costmap publisher
  - `python test/static_costmap_publisher.py`
- Start visualization and rrt
  - `python test/visualization.py`
  - `rviz -d rrt_config.rviz`
