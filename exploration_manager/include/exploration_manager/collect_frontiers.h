#ifndef __COLLECT_FRONTIERS__
#define __COLLECT_FRONTIERS__
  
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <exploration_manager/SharedClass.h>

#include "frontier_extraction_msgs/msg/frontier.hpp"
#include "frontier_extraction_srvs/srv/get_frontiers.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace BT;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class CollectFrontiers : public BT::SyncActionNode
{
  public:
    CollectFrontiers(const std::string& name,
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
    //Froneiter Service Client
    rclcpp::Client<frontier_extraction_srvs::srv::GetFrontiers>::SharedPtr frontier_extract_srv_;

    frontier_extraction_srvs::srv::GetFrontiers::Request::SharedPtr get_frontiers_req_;
    rclcpp::Client<frontier_extraction_srvs::srv::GetFrontiers>::SharedFuture get_frontiers_fut_;
    frontier_extraction_srvs::srv::GetFrontiers::Response::SharedPtr get_frontiers_res_;

    //Nav2 Action Client
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_ptr_;

    //Timer
    rclcpp::Time now_, prev_time_;
    float time_diff_;
    float timer_frontiers_; // To avoid segmenting the frontiers to often
};

#endif
