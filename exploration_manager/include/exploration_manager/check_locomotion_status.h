#ifndef __CHECK_LOCOM_STATUS__
#define __CHECK_LOCOM_STATUS__

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include <exploration_manager/SharedClass.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <actionlib_msgs/msg/goal_status_array.hpp>

using namespace BT;
using std::placeholders::_1;
using GoalStatus = action_msgs::msg::GoalStatus;

class CheckLocomotionStatus : public BT::SyncActionNode
{
  public:
    CheckLocomotionStatus(const std::string& name,
                          const BT::NodeConfig &config,
                          rclcpp::Node::SharedPtr node);
    
    static BT::PortsList providedPorts() {
        return {};
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    
  private:
    rclcpp::Node::SharedPtr node_;
    
		std::shared_ptr<tf2_ros::TransformListener> tf_listener_ {nullptr};
		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_status_sub_;
    
    action_msgs::msg::GoalStatusArray::SharedPtr status_msg_;
    GoalStatus prev_status_msg_;

    double min_nav_target_distance_, min_frontier_distance_, distance_target_object_;
    int status_msg_id_;
    double temp_distance_;

    void getNavStatus(const action_msgs::msg::GoalStatusArray::SharedPtr msg);};

#endif