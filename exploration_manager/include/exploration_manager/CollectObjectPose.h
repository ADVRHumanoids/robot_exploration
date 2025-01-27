#ifndef __COLLECT_OBJ_POSE__
#define __COLLECT_OBJ_POSE__
  
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <exploration_manager/SharedClass.h>

#include <tf/transform_listener.h>

using namespace BT;

class CollectObjectPose : public BT::AsyncActionNode
{
  public:
    CollectObjectPose(const std::string& name);
    
    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    void halt() override;
    
  private:
    ros::NodeHandle nh_;
    
    tf::TransformListener listener_;

    float angle_, distance_target_object_;
};

#endif
