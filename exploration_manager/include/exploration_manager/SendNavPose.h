#ifndef __SEND_NAV_POSE__
#define __SEND_NAV_POSE__
  
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <exploration_manager/SharedClass.h>

#include <geometry_msgs/PoseStamped.h>
#include <centauro_ros_nav/SendCandidateNavTarget.h>

using namespace BT;

class SendNavPose : public BT::AsyncActionNode
{
  public:
    SendNavPose(const std::string& name);
    
    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    void halt() override;
    
  private:
    ros::NodeHandle nh_;

    ros::ServiceClient send_candidate_nav_target_;
    // ros::Publisher send_nav_target_;

    // geometry_msgs::PoseStamped nav_target_;    
    centauro_ros_nav::SendCandidateNavTarget candidate_nav_target_;
    
};

#endif
