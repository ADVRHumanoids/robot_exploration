#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <frontier_extraction/frontier_3d_extraction_manager.h>

using namespace frontier_extraction;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<frontier_extraction::Frontier3DExtractionManager>());
  rclcpp::shutdown();
  return 0;
}