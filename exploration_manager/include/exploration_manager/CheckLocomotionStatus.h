#ifndef __CHECK_LOCOM_STATUS__
#define __CHECK_LOCOM_STATUS__

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <exploration_manager/SharedClass.h>

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
    ros::Subscriber nav_status_sub_;
    
    tf::TransformListener listener_;
    actionlib_msgs::GoalStatusArrayConstPtr msg_; 

    float min_nav_target_distance_, min_frontier_distance_;

    void getNavStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
};

#endif