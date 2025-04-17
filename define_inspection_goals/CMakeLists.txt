cmake_minimum_required(VERSION 3.8)
project(define_inspection_goals)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(octomap REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(define_inspection_goals_srvs REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  geometry_msgs
  nav2_msgs
  visualization_msgs
  define_inspection_goals_srvs
)

# frontier 2D
add_library(inspection_goals_srv_manager SHARED
  src/inspection_goals_srv_manager.cpp
)

ament_target_dependencies(inspection_goals_srv_manager
  ${dependencies}
)

install(TARGETS inspection_goals_srv_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(inspection_goals_srv_node src/inspection_goals_srv_node.cpp)
target_link_libraries(inspection_goals_srv_node inspection_goals_srv_manager)
ament_target_dependencies(inspection_goals_srv_node ${dependencies})

install(TARGETS
  inspection_goals_srv_node
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