cmake_minimum_required(VERSION 3.0.2)
project(voice_mcp)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs std_srvs
)

catkin_install_python(PROGRAMS
  scripts/voice_listener_node.py
  scripts/command_parser.py
  scripts/whisper_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
