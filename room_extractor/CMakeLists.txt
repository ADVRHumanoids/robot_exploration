cmake_minimum_required(VERSION 3.0.2)
project(room_extractor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/RoomExtractor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
