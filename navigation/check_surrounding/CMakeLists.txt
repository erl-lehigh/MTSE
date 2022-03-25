cmake_minimum_required(VERSION 3.0.2)
project(check_surrounding)

find_package(catkin REQUIRED COMPONENTS
  rospy
  ackermann_msgs
  geometry_msgs
  nav_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  ackermann_msgs
  geometry_msgs
  nav_msgs
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/purepursuit_node.test) # TODO: this needs to be changed
endif()
