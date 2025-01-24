#ifndef __EXPLORE__
#define __EXPLORE__
  
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

#include <exploration_manager/common.h>

#include <tf/transform_listener.h>
using namespace BT;

class Explore : public BT::AsyncActionNode
{
  public:
    Explore(const std::string& name);
    
    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    void halt() override;
    
  private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    
    float cost_, cost_max_, squared_euclidean_distance_, squared_distance_to_prev_target_;
    int max_frontier_idx_;
    
};

#endif
