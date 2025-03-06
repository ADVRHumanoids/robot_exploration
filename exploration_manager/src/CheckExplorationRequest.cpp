#include <exploration_manager/CheckExplorationRequest.h>

CheckExplorationRequest::CheckExplorationRequest(const std::string& name) :
    BT::AsyncActionNode(name, {})
{
    action_server_ = std::make_shared<ActionServer>(nh_, "/request_exploration", boost::bind(&CheckExplorationRequest::manageActionRequest, this, _1), false);
    action_server_->start();
}

BT::NodeStatus CheckExplorationRequest::tick(){

    if(bt_data_->finished_exploration){
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
    
    return BT::NodeStatus::SUCCESS;
}

void CheckExplorationRequest::manageActionRequest(const exploration_manager::RequestExplorationGoalConstPtr& goal)
{
    ROS_WARN("New Exploration Request!!");

    if (action_server_->isPreemptRequested()){
        ROS_INFO("Exploration Request Preempted");
        // set the action state to preempted
        action_server_->setPreempted();
    }
    else{
        
        bt_data_->object_name = goal->object_name;
        bt_data_->known_object_pose = false;
        bt_data_->force_frontier_update = true;
        bt_data_->need_exploration = true;
        bt_data_->finished_exploration = false;

        action_feedback_.finished = false;
        action_result_.found = false;

        action_server_->setSucceeded(action_result_);
        action_server_->publishFeedback(action_feedback_);
    }
}

void CheckExplorationRequest::halt(){

}
