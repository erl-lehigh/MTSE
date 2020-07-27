cmake_minimum_required(VERSION 2.8.3)
project(carla_setup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
      CATKIN_DEPENDS
      rospy
)

