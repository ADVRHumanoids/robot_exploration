#include <exploration_manager/check_exploration_request.h>

CheckExplorationRequest::CheckExplorationRequest(const std::string& name,
                                                 const BT::NodeConfig &config,
                                                 rclcpp::Node::SharedPtr node) :
    BT::SyncActionNode(name, config), node_(node)
{
    action_server_ = rclcpp_action::create_server<RequestExploration>(
                        node_,
                        "/request_exploration",
                        std::bind(&CheckExplorationRequest::handle_goal, this, _1, _2),
                        std::bind(&CheckExplorationRequest::handle_cancel, this, _1),
                        std::bind(&CheckExplorationRequest::handle_accepted, this, _1));

    action_feedback_ = std::make_shared<RequestExploration::Feedback>();
    action_result_ = std::make_shared<RequestExploration::Result>();

    RCLCPP_INFO(node_->get_logger(), "Initialized");
}

BT::NodeStatus CheckExplorationRequest::tick(){

    //If NodeHandle not nullptr
    if(action_goal_handle_ == nullptr){
        return BT::NodeStatus::FAILURE;
    }

    //If goal canceled
    if(action_goal_handle_->is_canceling()){
        action_result_->found = false;
        action_goal_handle_->canceled(action_result_);
        RCLCPP_INFO(node_->get_logger(), "Goal canceled");
        resetState();
        return BT::NodeStatus::FAILURE;
    }
    
    //If exploration finished
    if(bt_data_->finished_exploration){
        action_feedback_->finished = true;
        action_result_->found = true;
        RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
        action_goal_handle_->succeed(action_result_);

        //Reset goal handle
        action_goal_handle_ = nullptr;
        return BT::NodeStatus::SUCCESS;
    }
    else{
        action_feedback_->finished = false;
        action_result_->found = false;
    }

    action_goal_handle_->publish_feedback(action_feedback_);
    
    return BT::NodeStatus::SUCCESS;
}

rclcpp_action::GoalResponse CheckExplorationRequest::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const RequestExploration::Goal> goal)
{
    RCLCPP_INFO(node_->get_logger(), "Received goal: Object to find: %s", goal->object_name.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CheckExplorationRequest::handle_cancel(
    const std::shared_ptr<GoalHandleRequestExploration> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CheckExplorationRequest::handle_accepted(const std::shared_ptr<GoalHandleRequestExploration> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "Handle Accepted");
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&CheckExplorationRequest::execute, this, _1), goal_handle}.detach();
}

void CheckExplorationRequest::resetState(){
    bt_data_->object_name = "";
    bt_data_->known_object_pose = false;
    bt_data_->force_frontier_update = false;
    bt_data_->need_exploration = false;
    bt_data_->finished_exploration = false;
}

void CheckExplorationRequest::execute(const std::shared_ptr<GoalHandleRequestExploration> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "New Exploration Request!!");
    action_goal_handle_ = goal_handle;

    if(action_goal_handle_ == nullptr)
        RCLCPP_INFO(node_->get_logger(), "GoalHandle nullptr");

    auto goal = goal_handle->get_goal();

    bt_data_->object_name = goal->object_name;
    bt_data_->known_object_pose = false;
    bt_data_->force_frontier_update = true;
    bt_data_->need_exploration = true;
    bt_data_->finished_exploration = false;

    action_feedback_->finished = false;
    action_result_->found = false;
}