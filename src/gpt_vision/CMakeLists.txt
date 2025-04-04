cmake_minimum_required(VERSION 3.0.2)
project(gpt_vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/scripts/get_gpt_response.py
  src/scripts/vision_grid_state.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
