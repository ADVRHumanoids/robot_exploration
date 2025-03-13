cmake_minimum_required(VERSION 3.8)
project(frontier_extraction_srvs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(frontier_extraction_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  frontier_extraction_msgs
)

set(srv_files
  "srv/GetFrontiers.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES geometry_msgs std_msgs frontier_extraction_msgs)

ament_package()