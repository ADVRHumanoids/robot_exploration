#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <define_inspection_goals/inspection_goals_srv_manager.h>

using namespace inspection_goals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InspectionGoalsSrvManager>());
  rclcpp::shutdown();
  return 0;
}