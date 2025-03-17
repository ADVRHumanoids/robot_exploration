cmake_minimum_required(VERSION 3.5)
project(exploration_manager)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  behaviortree_cpp_v3
  frontier_extraction
  message_generation
  actionlib_msgs
  centauro_ros_nav
)

#find_package(BehaviorTreeV3 REQUIRED)
#Add action file
add_action_files(DIRECTORY action FILES RequestExploration.action)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs behaviortree_cpp_v3 frontier_extraction  actionlib_msgs message_generation centauro_ros_nav
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#ACTIONS
add_library(CheckExplorationRequest src/CheckExplorationRequest.cpp)
add_dependencies(CheckExplorationRequest  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(CheckExplorationRequest ${catkin_LIBRARIES})

add_library(CollectObjectPose src/CollectObjectPose.cpp)
add_dependencies(CollectObjectPose  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(CollectObjectPose ${catkin_LIBRARIES})

add_library(CollectFrontiers src/CollectFrontiers.cpp)
add_dependencies(CollectFrontiers  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(CollectFrontiers ${catkin_LIBRARIES})

add_library(Explore src/Explore.cpp)
add_dependencies(Explore  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Explore ${catkin_LIBRARIES})

add_library(SendNavPose src/SendNavPose.cpp)
add_dependencies(SendNavPose  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(SendNavPose ${catkin_LIBRARIES})


#CONDITIONS
add_library(IsExplorationRequired src/IsExplorationRequired.cpp)
add_dependencies(IsExplorationRequired  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(IsExplorationRequired ${catkin_LIBRARIES})

add_library(CheckLocomotionStatus src/CheckLocomotionStatus.cpp)
add_dependencies(CheckLocomotionStatus  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(CheckLocomotionStatus ${catkin_LIBRARIES})


add_executable(exploration_main src/exploration_main.cpp)
target_link_libraries(exploration_main ${catkin_LIBRARIES} ${behaviortree_cpp_v3_LIBRARIES}  CheckExplorationRequest CollectFrontiers Explore SendNavPose
                                                                                             IsExplorationRequired CollectObjectPose CheckLocomotionStatus)


 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
