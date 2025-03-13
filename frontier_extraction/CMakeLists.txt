cmake_minimum_required(VERSION 3.8)
project(frontier_extraction)

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
find_package(nav_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(frontier_extraction_srvs REQUIRED)
find_package(frontier_extraction_msgs REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  nav_msgs
  octomap_msgs
  visualization_msgs
  frontier_extraction_srvs
  frontier_extraction_msgs
)

add_library(frontier_2d_extraction_manager SHARED
  src/frontier_2d_extraction_manager.cpp
)

ament_target_dependencies(frontier_2d_extraction_manager
  ${dependencies}
)

install(TARGETS frontier_2d_extraction_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(frontier_2d_extraction_node src/frontier_2d_extraction_node.cpp)
target_link_libraries(frontier_2d_extraction_node frontier_2d_extraction_manager)
ament_target_dependencies(frontier_2d_extraction_node ${dependencies})

install(TARGETS
  frontier_2d_extraction_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
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