#ifndef __DEFINE_INSPECTION_GOALS__
#define __DEFINE_INSPECTION_GOALS__
  
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include <exploration_manager/SharedClass.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <centauro_ros_nav_srvs/srv/send_candidate_nav_target.hpp>

#include "define_inspection_goals_srvs/srv/get_inspection_goals.hpp"

#include <chrono>

using namespace BT;
using namespace std::chrono_literals;

class DefineInspectionGoals : public BT::SyncActionNode
{
  public:
    DefineInspectionGoals(const std::string& name,
                          const BT::NodeConfig &config,
                          rclcpp::Node::SharedPtr node);
    
    static BT::PortsList providedPorts() {
        return {};
    }
    
    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    
  private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Client<define_inspection_goals_srvs::srv::GetInspectionGoals>::SharedPtr get_insp_goals_srv_;

    define_inspection_goals_srvs::srv::GetInspectionGoals::Request::SharedPtr get_insp_goals_req_;
    rclcpp::Client< define_inspection_goals_srvs::srv::GetInspectionGoals>::SharedFuture get_insp_goals_fut_;
    define_inspection_goals_srvs::srv::GetInspectionGoals::Response::SharedPtr get_insp_goals_res_;

    double angle_;
    double min_distance_to_object_, max_distance_to_object_;
};

#endif
