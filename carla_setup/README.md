This contains the python scripts used to communicate with Carla through ROS using the [Carla-ROS-Bridge](https://github.com/carla-simulator/ros-bridge).

# Functionality

## Driving Commands (Ackermann Control)

The most useful functionality of this script is sending driving commands to CARLA. To accomplish this, desired steering changes are published to the */carla/ego_vehicle/ackermann_cmd* topic, where the topics are read by CARLA and the adjustments are made.

The *control(s, a, j, st, av)* method takes in speed (m/s), acceleration (m/s^2), jerk (m/s^3), steering angle (radians), and steering angle velocity (radians/second) to broadcast the changes.

## Vehicle Information (W.I.P.)

**Not currently functional**
Sets vehicle type by publishing changes to the */carla/ego_vehicle/vehicle_info* topic.

## Camera

Displays an RGB camera mounted on the hood of the car so the user can see their changes making the vehicle move.

## GNSS

Provides the approximate location of the vehicle with noise (latitude and longitude) using a GNSS sensor.

## Odometry

Provides the exact location, (x,y,z), of the vehicle with reference to the world using onboard vehicle odometry. This is published to a tf transform.

# Implementaiton

The code is written in [python](https://www.python.org/) using [ROS](http://wiki.ros.org/) to publish and subscribe to topics. The vehicle movement and sensor data is simulated in [CARLA](http://carla.org/). Visualization is done using [OpenCV](https://opencv.org/) with help from [Numpy](https://numpy.org/).

# Nodes

## Publishers

### '/carla/ego_vehicle/ackermann_cmd'

[AckermannDrive.msg](http://docs.ros.org/jade/api/ackermann_msgs/html/msg/AckermannDrive.html)

### '/carla/ego_vehicle/vehicle_info'

[CarlaEgoVehicleInfo.msg](https://carla.readthedocs.io/en/latest/ros_msgs/#carlaegovehicleinfomsg)

## Subscribers

### "/carla/ego_vehicle/camera/rgb/front/image_color"

[Image.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)

### "/carla/ego_vehicle/gnss/gnss1/fix"

[NavSatFix.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html)

### "/carla/ego_vehicle/odometry"

[Odometry.msg](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html)

### "speed_command"

[AckermannDrive.msg](http://docs.ros.org/jade/api/ackermann_msgs/html/msg/AckermannDrive.html)

# Launch / Configuration Files

## carla.launch

Launches the Carla-Ros-Bridge and spawns in a vehicle.

## ackermann.yaml

Configuration file holding some startup information.

## ego_vehicle_sensor.json

Sets up sensors on the ego_vehicle.

# Tests

Currently **None**. 
*The test_carla_ackermann_control.py file does however have a main method that provides test input.*



