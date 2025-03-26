#ifndef __COLLECT_OBJ_POSE__
#define __COLLECT_OBJ_POSE__
  
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <exploration_manager/SharedClass.h>

#include <object_detection_srvs/srv/get_objects_info.hpp>
#include <object_detection_msgs/msg/object_info.hpp>

#include <chrono>

using namespace BT;
using namespace std::chrono_literals;

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
    
    rclcpp::Client<object_detection_srvs::srv::GetObjectsInfo>::SharedPtr get_objects_info_srv_;

    object_detection_srvs::srv::GetObjectsInfo::Request::SharedPtr get_objects_req_;
    rclcpp::Client< object_detection_srvs::srv::GetObjectsInfo>::SharedFuture get_objects_fut_;
    object_detection_srvs::srv::GetObjectsInfo::Response::SharedPtr get_objects_res_;

    double angle_, distance_to_object_pose_;
    bool service_available_;
};

#endif
