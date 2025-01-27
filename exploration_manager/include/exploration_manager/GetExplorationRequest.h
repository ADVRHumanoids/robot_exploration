#ifndef __GET_EXPLORATION_REQUEST__
#define __GET_EXPLORATION_REQUEST__
  
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <ros/ros.h>
#include <exploration_manager/SharedClass.h>

#include <exploration_manager/RequestExplorationAction.h>
#include <actionlib/server/simple_action_server.h>

using namespace BT;

typedef actionlib::SimpleActionServer<exploration_manager::RequestExplorationAction> ActionServer;

class GetExplorationRequest : public BT::AsyncActionNode
{
  public:
    GetExplorationRequest(const std::string& name);
    
    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    void halt() override;
    
  private:
    ros::NodeHandle nh_;
    std::shared_ptr<ActionServer> action_server_;
    exploration_manager::RequestExplorationFeedback action_feedback_;
    exploration_manager::RequestExplorationResult action_result_;

    bool new_task_received_;

    void manageActionRequest(const exploration_manager::RequestExplorationGoalConstPtr& goal);
};

#endif
