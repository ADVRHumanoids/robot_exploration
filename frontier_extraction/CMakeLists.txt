cmake_minimum_required(VERSION 3.0)
project(frontier_extraction)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf
  tf_conversions
  eigen_conversions
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 Frontier.msg
)

add_service_files(
 FILES
 GetFrontiers.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
#generate_dynamic_reconfigure_options( 
#    config/rqt_planner.cfg
#    )

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES perception_thesis_gap
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs nav_msgs tf tf_conversions message_generation message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
  #${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_library(FrontierExtractionManager src/FrontierExtractionManager.cpp)
add_dependencies(FrontierExtractionManager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(FrontierExtractionManager ${catkin_LIBRARIES})

add_executable(FrontierExtractionNode src/FrontierExtractionNode.cpp)
target_link_libraries(FrontierExtractionNode ${catkin_LIBRARIES} FrontierExtractionManager)


## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

