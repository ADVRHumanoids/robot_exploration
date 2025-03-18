cmake_minimum_required(VERSION 3.8)
project(exploration_manager)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(exploration_manager_actions REQUIRED)
find_package(frontier_extraction_msgs REQUIRED)
find_package(frontier_extraction_srvs REQUIRED)
find_package(centauro_ros_nav_srvs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  exploration_manager_actions
  frontier_extraction_msgs
  frontier_extraction_srvs
  centauro_ros_nav_srvs
  behaviortree_cpp
  tf2
  tf2_ros
  tf2_eigen
  nav2_msgs
  actionlib_msgs
)

# BT Actions-Conditions
add_library(bt_nodes SHARED
  src/check_exploration_request.cpp
  src/is_exploration_required.cpp
  src/collect_object_pose.cpp
  src/collect_frontiers.cpp
  src/explore.cpp
  src/check_locomotion_status.cpp
  src/send_nav_pose.cpp
)

ament_target_dependencies(bt_nodes
  ${dependencies}
)

install(TARGETS bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(exploration_main src/exploration_main.cpp)
target_link_libraries(exploration_main bt_nodes)
ament_target_dependencies(exploration_main ${dependencies})

install(TARGETS
  exploration_main
  DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()