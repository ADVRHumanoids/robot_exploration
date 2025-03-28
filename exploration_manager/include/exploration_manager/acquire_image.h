#ifndef __ACQUIRE_IMAGE__
#define __ACQUIRE_IMAGE__
  
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include <exploration_manager/SharedClass.h>

#include "std_srvs/srv/trigger.hpp"

using namespace BT;

class AcquireImage : public BT::SyncActionNode
{
  public:
    AcquireImage(const std::string& name,
                 const BT::NodeConfig &config,
                 rclcpp::Node::SharedPtr node);
    
    static BT::PortsList providedPorts() {
        return {};
    }
    
    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    
  private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr acquire_image_srv_;

    std_srvs::srv::Trigger::Request::SharedPtr acquire_image_req_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture acquire_image_fut_;
    std_srvs::srv::Trigger::Response::SharedPtr acquire_image_res_;
};

#endif
