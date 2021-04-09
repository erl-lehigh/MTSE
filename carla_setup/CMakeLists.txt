cmake_minimum_required(VERSION 2.8.3)
project(carla_setup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  ackermann_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  carla_msgs
  carla_ros_bridge
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  ackermann_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  carla_msgs
  carla_ros_bridge
)
