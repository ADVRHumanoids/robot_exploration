#ifndef __CHECK_LOCOM_STATUS__
#define __CHECK_LOCOM_STATUS__

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <exploration_manager/common.h>

#include <actionlib_msgs/GoalStatusArray.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <tf/transform_listener.h>
using namespace BT;

class CheckLocomotionStatus : public BT::AsyncActionNode
{
  public:
    CheckLocomotionStatus(const std::string& name);
    
    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    void halt() override;
    
  private:
    ros::NodeHandle nh_;
    
    tf::TransformListener listener_;
    actionlib_msgs::GoalStatusArrayConstPtr msg_; 
};

#endif