cmake_minimum_required(VERSION 3.0.2)
project(light_follower)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/light_follower_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
