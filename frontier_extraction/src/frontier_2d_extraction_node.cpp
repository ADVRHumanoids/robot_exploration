#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <frontier_extraction/frontier_2d_extraction_manager.h>

using namespace frontier_extraction;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<frontier_extraction::Frontier2DExtractionManager>());
  rclcpp::shutdown();
  return 0;
}