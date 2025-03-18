#ifndef __COLLECT_OBJ_POSE__
#define __COLLECT_OBJ_POSE__
  
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <exploration_manager/SharedClass.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace BT;

class CollectObjectPose : public BT::SyncActionNode
{
  public:

    CollectObjectPose(const std::string& name,
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

    double angle_, distance_target_object_;
};

#endif
