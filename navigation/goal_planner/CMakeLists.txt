cmake_minimum_required(VERSION 2.8.3)
project(goal_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

catkin_package(
      CATKIN_DEPENDS
      rospy
)

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest(test/route_planner_node.test)
#endif()
