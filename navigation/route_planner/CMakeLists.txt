cmake_minimum_required(VERSION 2.8.3)
project(route_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_python_setup()

catkin_package(
      CATKIN_DEPENDS
      rospy
      std_msgs
      nav_msgs
      geometry_msgs
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/route_planner_node.test)
endif()
