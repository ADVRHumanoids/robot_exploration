#ifndef __DEFINE_INSPECTION_GOALS__
#define __DEFINE_INSPECTION_GOALS__
  
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include <exploration_manager/SharedClass.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <centauro_ros_nav_srvs/srv/send_candidate_nav_target.hpp>

using namespace BT;

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
    double angle_;

    double min_distance_to_object_, max_distance_to_object_;
};

#endif
