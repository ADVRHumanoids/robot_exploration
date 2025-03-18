#ifndef __GET_EXPLORATION_REQUEST__
#define __GET_EXPLORATION_REQUEST__
  
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <exploration_manager_actions/action/request_exploration.hpp>

#include <exploration_manager/SharedClass.h>

using namespace BT;
using namespace std::placeholders;

using RequestExploration = exploration_manager_actions::action::RequestExploration;
using GoalHandleRequestExploration = rclcpp_action::ServerGoalHandle<RequestExploration>;
  
class CheckExplorationRequest : public BT::SyncActionNode
{
  public:

    CheckExplorationRequest(const std::string& name,
                            const BT::NodeConfig &config,
                            rclcpp::Node::SharedPtr node);
    
    static BT::PortsList providedPorts() {
        return {};
    }
    
    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    
  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Server<RequestExploration>::SharedPtr action_server_;
    RequestExploration::Result::SharedPtr action_result_;
    RequestExploration::Feedback::SharedPtr action_feedback_;

    std::shared_ptr<GoalHandleRequestExploration> action_goal_handle_;
    

    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const RequestExploration::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleRequestExploration> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleRequestExploration> goal_handle);

    void execute(const std::shared_ptr<GoalHandleRequestExploration> goal_handle);

    void resetState();
};

#endif
