cmake_minimum_required(VERSION 3.0.2)
project(launch-all)
find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_python_setup()

catkin_package(
  LIBRARIES launch-all
  CATKIN_DEPENDS rospy
  DEPENDS system_lib
)