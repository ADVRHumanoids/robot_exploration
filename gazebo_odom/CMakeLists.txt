cmake_minimum_required(VERSION 3.0)
project(gazebo_odom)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  tf
  tf_conversions
  eigen_conversions
  geometry_msgs
  gazebo_msgs
)


catkin_package(
# INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs tf tf_conversions gazebo_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include_directories()
  ${catkin_INCLUDE_DIRS}
  )

add_executable(sim_odom_connect src/sim_odom_connect.cpp)
add_dependencies(sim_odom_connect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sim_odom_connect ${catkin_LIBRARIES})


install(TARGETS sim_odom_connect
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
#install(DIRECTORY include/
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE
#)
