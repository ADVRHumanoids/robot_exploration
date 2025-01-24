#include <exploration_manager/GetExplorationRequest.h>

GetExplorationRequest::GetExplorationRequest(const std::string& name) :
    BT::AsyncActionNode(name, {})
{
    action_server_ = std::make_shared<ActionServer>(nh_, "/request_exploration", boost::bind(&GetExplorationRequest::manageActionRequest, this, _1), false);
    action_server_->start();

    new_task_received_ = false;
}

BT::NodeStatus GetExplorationRequest::tick(){

    if(bt_data.finished_exploration){
        action_feedback_.finished = true;
        action_result_.found = true;
    }
    else{
        action_feedback_.finished = false;
        action_result_.found = false;
    }

    if(action_server_->isActive()){
        action_server_->setSucceeded(action_result_);
        action_server_->publishFeedback(action_feedback_);
    }
        
    return BT::NodeStatus::FAILURE;
}

void GetExplorationRequest::manageActionRequest(const exploration_manager::RequestExplorationGoalConstPtr& goal)
{
    ROS_WARN("New Exploration Request!!");

    if (action_server_->isPreemptRequested()){
        ROS_INFO("Exploration Request Preempted");
        // set the action state to preempted
        action_server_->setPreempted();
    }
    else{
        
        bt_data.object_name = goal->object_name;
        bt_data.need_exploration = true;
        new_task_received_ = true;

        action_feedback_.finished = false;
        action_result_.found = false;
        bt_data.finished_exploration = false;

        action_server_->setSucceeded(action_result_);
        action_server_->publishFeedback(action_feedback_);
    }
}

void GetExplorationRequest::halt(){

}
