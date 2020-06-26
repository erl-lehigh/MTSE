This contains the python scripts used to communicate with Carla through ROS.
The current scripts are:
nathan.py
test_carla_ackermann_control.py

nathan.py: Holds a generic script that creates a vehicle in carla and attaches an rgb camera to the vehicle. The script subscribes to the ackermann topic and gets fed steering and velocity data.

test_carla_ackermann_control.py: Broadcasts velocity and steering data to be subscribed to by other scripts that subscribe to the same topic.

