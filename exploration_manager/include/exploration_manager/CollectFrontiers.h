#ifndef __COLLECT_FRONTIERS__
#define __COLLECT_FRONTIERS__
  
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

#include <exploration_manager/common.h>

#include <frontier_extraction/Frontier.h>
#include <frontier_extraction/GetFrontiers.h>
#include <tf/transform_listener.h>

using namespace BT;

class CollectFrontiers : public BT::AsyncActionNode
{
  public:
    CollectFrontiers(const std::string& name);
    
    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    void halt() override;
    
  private:
    ros::NodeHandle nh_;
    
    tf::TransformListener listener_;
    ros::ServiceClient frontier_extract_srv_;
  
    frontier_extraction::GetFrontiers get_frontiers_;

    //Timer
    ros::Time now_, prev_time_;
    float time_diff_;
    float timer_frontiers_; // To avoid segmenting the frontiers to often
};

#endif
