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

    //Service Client for Nav2 goal cancelling
    cancel_nav_goal_srv_ = node_->create_client<action_msgs::srv::CancelGoal>("/navigate_to_pose/_action/cancel_goal");
    cancel_nav_goal_req_ = std::make_shared<action_msgs::srv::CancelGoal::Request>();
    bt_data_->ros_status.cancel_nav_srv = cancel_nav_goal_srv_->wait_for_service(10s);

    resetState();
}

BT::NodeStatus CheckExplorationRequest::tick(){

    //If NodeHandle not nullptr
    if(action_goal_handle_ == nullptr){
        return BT::NodeStatus::FAILURE;
    }

    // RCLCPP_INFO(node_->get_logger(), "TaskId: %d,  ActiveTask:%d", bt_data_->tasks[bt_data_->current_task].id, bt_data_->active_task);
    
    //If exploration finished (and at least one task was assigned)
    if(static_cast<int>(bt_data_->tasks.size()) > 0 && bt_data_->current_task  >= static_cast<int>(bt_data_->tasks.size()) && !bt_data_->active_task){

        if(bt_data_->need_exploration && !bt_data_->finished_exploration){
            action_feedback_->finished = true;
            action_result_->found = false;
            RCLCPP_INFO(node_->get_logger(), "Failed Exploration!");
        }
        else{
            action_feedback_->finished = true;
            action_result_->found = true;
            RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
        }

        action_goal_handle_->succeed(action_result_);

        //Reset goal handle
        if(!bt_data_->ros_status.cancel_nav_srv){
            //Try again
            bt_data_->ros_status.cancel_nav_srv = cancel_nav_goal_srv_->wait_for_service(1s);
            RCLCPP_WARN(node_->get_logger(), "CancelGoal Srv Unavailable");
            return BT::NodeStatus::FAILURE;
        }
        
        action_goal_handle_ = nullptr;
        cancel_nav_goal_srv_->async_send_request(cancel_nav_goal_req_);
        resetState();

        return BT::NodeStatus::FAILURE;
    }
    else if(action_goal_handle_->is_canceling()){
        // action_result_->found = false;
        // action_goal_handle_->canceled(action_result_);
        RCLCPP_INFO(node_->get_logger(), "Goal canceled - Stop Robot Nav");
        
        if(!bt_data_->ros_status.cancel_nav_srv){
            //Try again
            bt_data_->ros_status.cancel_nav_srv = cancel_nav_goal_srv_->wait_for_service(1s);
            RCLCPP_WARN(node_->get_logger(), "CancelGoal Srv Unavailable");
            return BT::NodeStatus::FAILURE;
        }

        cancel_nav_goal_srv_->async_send_request(cancel_nav_goal_req_);
        bt_data_->is_driving = false;

        resetState();

        action_result_->found = false;
        action_goal_handle_->succeed(action_result_);
        //Reset goal handle
        action_goal_handle_ = nullptr;
        
        return BT::NodeStatus::FAILURE;
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
    bt_data_->known_object_pose = false;
    bt_data_->force_frontier_update = false;
    bt_data_->need_exploration = false;
    bt_data_->finished_exploration = false;
    bt_data_->tasks.clear();
    bt_data_->current_task = 0;
    bt_data_->active_task = false;
    bt_data_->acquire_image = false;
}

void CheckExplorationRequest::execute(const std::shared_ptr<GoalHandleRequestExploration> goal_handle)
{
    resetState();

    RCLCPP_INFO(node_->get_logger(), "New Exploration Request!!");
    action_goal_handle_ = goal_handle;

    if(action_goal_handle_ == nullptr)
        RCLCPP_INFO(node_->get_logger(), "GoalHandle nullptr");

    auto goal = action_goal_handle_->get_goal();

    //New Task
    switch(goal->task_id){
        case 1:
            bt_data_->tasks.push_back(Task(1, goal->object_name));
            break;
        case 2:
            bt_data_->tasks.push_back(Task(2, goal->object_name));
            break;
        default:
            bt_data_->tasks.push_back(Task());
            break;
    }

    bt_data_->force_frontier_update = true;
    bt_data_->need_exploration = true;
    bt_data_->active_task = true;
    
    action_feedback_->finished = false;
    action_result_->found = false;

    action_goal_handle_->publish_feedback(action_feedback_);
}