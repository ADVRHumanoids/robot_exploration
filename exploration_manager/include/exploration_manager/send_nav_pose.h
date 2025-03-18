#ifndef __SEND_NAV_POSE__
#define __SEND_NAV_POSE__
  
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include <exploration_manager/SharedClass.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <centauro_ros_nav_srvs/srv/send_candidate_nav_target.hpp>

using namespace BT;

class SendNavPose : public BT::SyncActionNode
{
  public:
    SendNavPose(const std::string& name,
                      const BT::NodeConfig &config,
                      rclcpp::Node::SharedPtr node);
    
    static BT::PortsList providedPorts() {
        return {};
    }
    
    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    
  private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Client<centauro_ros_nav_srvs::srv::SendCandidateNavTarget>::SharedPtr send_candidate_nav_target_;

    centauro_ros_nav_srvs::srv::SendCandidateNavTarget::Request::SharedPtr candidate_nav_target_req_;
    rclcpp::Client< centauro_ros_nav_srvs::srv::SendCandidateNavTarget>::SharedFuture candidate_nav_target_fut_;
    centauro_ros_nav_srvs::srv::SendCandidateNavTarget::Response::SharedPtr candidate_nav_target_res_;
};

#endif
